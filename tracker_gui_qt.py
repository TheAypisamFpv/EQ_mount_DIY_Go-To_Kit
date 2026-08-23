#!/usr/bin/env python3
"""
EQ Mount Tracker - Qt/OpenGL GUI (rendering-layer experiment)
================================================================
A SECOND, independent GUI for the same mount - NOT a replacement for tracker_gui.py (the
CustomTkinter/Tkinter version), which stays the primary, fully-featured app. See CLAUDE.md.

Why this exists: tracker_gui.py's sky viz canvas was laggy at high zoom/pan because Tkinter's
Canvas rebuilds every star/DSO/grid line as an individual real widget item on every redraw (see
GUI_VERSION 1.0.4's changelog in tracker_gui.py for the full writeup and the raster-blit fix
applied there). This file swaps the RENDERING LAYER ONLY to Qt (PySide6) with a QOpenGLWidget
viewport, so the whole sky viz panel is drawn with Qt's C++ QPainter straight onto a
GPU-composited surface every frame, with no intermediate Python image step at all. Everything
else - the serial protocol, the sky/solar-system/ISS math, the catalog file itself - is REUSED
directly from tracker_gui.py / sky_data/*, not reimplemented, so the two GUIs can never disagree
about where the mount thinks it is or what CMD,* strings mean.

Scope: this is the sky viz + core control surface (connect, mode, goto, start/stop tracking,
sync, live log) - not full parity with tracker_gui.py (no Time Travel, no alignment-sequence
polish, no EEPROM location save/load, no debug dump viewer, RA/DEC entry is decimal degrees
only, no named-star sync dropdown). Extend this file the same way tracker_gui.py grew, rather
than duplicating logic that already lives there.

Run:
    python tracker_gui_qt.py
"""

import sys
import os
import math
import time
import json
import bisect
import threading
import queue
from collections import deque
from datetime import datetime, timezone
from typing import Optional

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, Signal, QObject
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, QPainterPath, QFontMetrics,
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QComboBox, QLabel, QLineEdit, QTextEdit, QRadioButton, QButtonGroup,
    QGroupBox,
)
from PySide6.QtOpenGLWidgets import QOpenGLWidget

# Reused, unmodified, straight from the Tkinter app - the serial protocol, camera/meridian/zoom
# geometry constants, and the Milky Way band builder. Importing tracker_gui.py as a plain module
# is safe here: its `ctk.set_appearance_mode(...)`/`set_default_color_theme(...)` module-level
# calls just set CustomTkinter's internal defaults (no Tk root is created), and EQMountApp's
# class body only defines methods - nothing runs until something actually instantiates it, which
# this file never does.
from tracker_gui import (
    SerialHandler, BAUD_RATE, POLL_INTERVAL_MS, POS_UI_UPDATE_MIN_INTERVAL_S,
    CAMERA_FOV_W_DEG, CAMERA_FOV_H_DEG, CAMERA_FOV_DEG_PER_PIXEL,
    MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG, MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG,
    MILKY_WAY_BAND_HALF_WIDTHS_DEG, MILKY_WAY_BAND_COLORS, MILKY_WAY_L_STEP_DEG,
    VIZ_ZOOM_MIN, VIZ_ZOOM_MAX, VIZ_ZOOM_STEP_BASE, VIZ_GRID_TARGET_LINES,
    STAR_LOD_TIER_MAG_CUTOFFS, _galactic_to_radec, TRACKING_STABLE_ERR_DEG,
    POS_UPDATE_RATE_MS,
)
from sky_data import sky_catalog, iss_tracker, solar_system


# ============================================================
# PURE GEOMETRY/STYLE HELPERS
# ============================================================
# Small, stateless helpers ported directly from the matching EQMountApp methods in tracker_gui.py
# (same formulas, same East-is-leftward/North-is-up screen convention) - kept as free functions
# here since SkyViewWidget isn't an EQMountApp subclass.

def _dso_render_style(type_code):
    """See EQMountApp._dso_render_style's docstring for the color/shape reasoning - identical
    mapping, just returning a QColor instead of a hex string."""
    t = type_code or ""
    if t in ("OCl", "GCl"):
        return "cluster", QColor("#ffee88")
    if t == "PN":
        return "ring", QColor("#dd88ff")
    if t.startswith("G") and t not in ("GCl",):
        return "ellipse", QColor("#ffaa66")
    if t in ("HII", "EmN"):
        return "ellipse", QColor("#ff6666")
    if t == "RfN":
        return "ellipse", QColor("#77aaff")
    if t == "DrkN":
        return "ellipse", QColor("#998877")
    if t == "SNR":
        return "ellipse", QColor("#ff9933")
    return "ellipse", QColor("#66ddcc")


def _darken_hex_color(hex_color, factor=0.28):
    hex_color = hex_color.lstrip('#')
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    return f"#{int(r * factor):02x}{int(g * factor):02x}{int(b * factor):02x}"


def _rotate_ne(u, v, pos_ang_deg):
    pa = math.radians(pos_ang_deg)
    sin_pa, cos_pa = math.sin(pa), math.cos(pa)
    east = u * sin_pa + v * cos_pa
    north = u * cos_pa - v * sin_pa
    return east, north


def _rotated_ellipse_points(cx, cy, half_major, half_minor, pos_ang_deg, n=28):
    """QPolygonF of an ellipse rotated by pos_ang_deg (position angle, from North towards East) -
    see EQMountApp._rotated_ellipse_points for the full derivation."""
    pa = math.radians(pos_ang_deg)
    sin_pa, cos_pa = math.sin(pa), math.cos(pa)
    poly = QPolygonF()
    for i in range(n):
        t = 2 * math.pi * i / n
        ct, st = math.cos(t), math.sin(t)
        east = half_major * ct * sin_pa + half_minor * st * cos_pa
        north = half_major * ct * cos_pa - half_minor * st * sin_pa
        poly.append(QPointF(cx - east, cy - north))
    return poly


def _camera_fov_half_size(pixels_per_deg):
    return (CAMERA_FOV_W_DEG / 2.0) * pixels_per_deg, (CAMERA_FOV_H_DEG / 2.0) * pixels_per_deg


def _camera_rect_points(cx, cy, half_w, half_h, pos_ang_deg):
    poly = QPolygonF()
    for u, v in ((half_h, -half_w), (half_h, half_w), (-half_h, half_w), (-half_h, -half_w)):
        east, north = _rotate_ne(u, v, pos_ang_deg)
        poly.append(QPointF(cx - east, cy - north))
    return poly


def _camera_top_marker_points(cx, cy, half_w, half_h, pos_ang_deg):
    tip_e, tip_n = _rotate_ne(half_h + 10, 0, pos_ang_deg)
    l_e, l_n = _rotate_ne(half_h, -6, pos_ang_deg)
    r_e, r_n = _rotate_ne(half_h, 6, pos_ang_deg)
    poly = QPolygonF()
    poly.append(QPointF(cx - tip_e, cy - tip_n))
    poly.append(QPointF(cx - l_e, cy - l_n))
    poly.append(QPointF(cx - r_e, cy - r_n))
    return poly


def _illuminated_disc_path(x, y, r, bearing_deg, illum_fraction):
    """Terminator ellipse (the boundary between lit/dark hemispheres) as a QPainterPath, using
    the same half-disc + terminator-ellipse technique as EQMountApp._draw_illuminated_disc -
    see its docstring for the full derivation. Returns (lit_half_path, terminator_path); the
    caller fills lit_half_path with the lit color and terminator_path with lit-or-dark color
    depending on phase, same layering as the Tk version."""
    # Qt's arcTo (like Tk's create_arc, style="chord") measures angles in degrees, 0 at the
    # 3 o'clock position, increasing counter-clockwise - the exact same convention the original
    # Tk call (start=bearing_deg, extent=180, style="chord") relies on, so no sign/offset
    # translation is needed. arcTo() as the first op on an empty path implicitly moves to the
    # arc's own start point; closeSubpath() then draws the straight chord back to it.
    path_lit = QPainterPath()
    path_lit.arcTo(QRectF(x - r, y - r, 2 * r, 2 * r), bearing_deg, 180)
    path_lit.closeSubpath()

    cos_phase = 1.0 - 2.0 * illum_fraction
    half_w = r * abs(cos_phase)
    bearing_rad = math.radians(bearing_deg)
    sin_b, cos_b = math.sin(bearing_rad), math.cos(bearing_rad)
    poly = QPolygonF()
    for i in range(24):
        t = 2 * math.pi * i / 24
        ct, st = math.cos(t), math.sin(t)
        poly.append(QPointF(x - half_w * ct * sin_b - r * st * cos_b,
                             y - half_w * ct * cos_b + r * st * sin_b))
    path_term = QPainterPath()
    path_term.addPolygon(poly)
    path_term.closeSubpath()
    return path_lit, path_term


def _dashed_pen(color, width=1.0):
    """QPen(..., Qt.DashLine)'s default cap style (SquareCap) draws each dash as a little square
    rather than a short line segment at these thin widths - reads as a row of tiny boxes instead
    of a dashed line/outline. FlatCap + an explicit dash pattern fixes that."""
    pen = QPen(color, width)
    pen.setDashPattern([3, 2])
    pen.setCapStyle(Qt.FlatCap)
    return pen


def _draw_centered_text(p, x, y, text, font, color, anchor="center"):
    """QPainter.drawText() positions by baseline-left, not a center point - this replicates
    Tk's create_text() anchor behavior (default "center", plus "w"/"e" used for the corner
    RA/DEC labels) so ported label positions (x, y - half_h - 8, ...) land the same way."""
    p.setFont(font)
    p.setPen(QPen(color))
    fm = QFontMetrics(font)
    rect = fm.boundingRect(text)
    ty = y + rect.height() / 2.0 - fm.descent()
    if anchor == "w":
        tx = x
    elif anchor == "e":
        tx = x - rect.width()
    else:
        tx = x - rect.width() / 2.0
    p.drawText(QPointF(tx, ty), text)


# ============================================================
# SKY VIEW WIDGET
# ============================================================
class SkyViewWidget(QOpenGLWidget):
    """The sky viz panel - Qt/OpenGL equivalent of tracker_gui.py's viz_canvas. Every frame is
    drawn fresh with QPainter directly onto the GPU-composited surface (paintEvent below) - no
    persistent per-star/per-DSO objects, no intermediate raster image. Qt's own C++ painter and
    OpenGL compositing are fast enough that this "just redraw everything" approach (the same
    conceptual model as tracker_gui.py's _draw_viz_grid, minus the Tk-item and Pillow-blit
    layers it needed to stay smooth) is already smooth at the star/DSO counts this catalog has.

    Coordinate convention matches tracker_gui.py exactly: RA increases LEFTWARD on screen, DEC
    increases upward - see ra_to_x/dec_to_y below, ported 1:1 from EQMountApp's versions."""

    targetPicked = Signal(float, float)   # emitted (ra_deg, dec_deg) on double-click
    hoverInfoChanged = Signal(str)        # emitted with a "RA ... DEC ... | name (extra)" string

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 300)
        self.setMouseTracking(True)

        # View state
        self.zoom = VIZ_ZOOM_MIN
        self.center_ra = 180.0
        self.center_dec = 0.0
        self.view_mode = "FREE"  # FREE / TELESCOPE / TARGET

        # Live mount state (set by MainWindow from parsed POS lines)
        self.current_ra, self.current_dec = 0.0, 0.0
        self.target_ra, self.target_dec = 0.0, 0.0
        self.telescope_flipped = False
        self.camera_orientation_deg = 0.0
        self.stability_color = QColor("#888888")

        # Observer location/time - needed for horizon/meridian-limit/LST, and passed through to
        # solar_system.py/iss_tracker.py by MainWindow's background update threads.
        self.lat, self.lon = 0.0, 0.0

        # Catalog data - populated by MainWindow.load_catalog() (see its docstring)
        self.sky_stars_by_ra = []
        self.sky_stars_ra_values = []
        self.star_tiers = []          # [(mag_cutoff, stars, ra_values), ...] - see STAR_LOD_TIER_MAG_CUTOFFS
        self.sky_dso = []
        self.const_line_segments = []
        self.star_by_hip = {}
        self.constellations_enabled = True
        # See EQMountApp._min_dso_size_filter_enabled's comment - real major-axis size projected
        # onto the actual camera sensor (CAMERA_FOV_DEG_PER_PIXEL), not on-screen pixel size.
        self.min_dso_size_filter_enabled = False
        self.min_dso_size_px = 100.0
        self.iss_enabled = False  # gates the ISS marker; background fetch runs regardless (cheap)

        # Real-time bodies - populated by MainWindow's background update timers
        self.iss_ra = self.iss_dec = None
        self.iss_above = False
        self.sun_ra = self.sun_dec = self.sun_diam = None
        self.moon_ra = self.moon_dec = self.moon_diam = None
        self.moon_illum = 0.0
        self.moon_waxing = True
        self.planet_positions = {}  # name -> (ra, dec, ang_diam, ring_ang_diam, illum_fraction)

        self.target_object_name = None  # set by MainWindow when a named target is selected

        self._milky_way_bands = None   # built lazily, see _milky_way_bands_cached
        self._pan_last = None
        self._visible_hits = []        # [{x,y,ra,dec,radius,name,extra,labeled}, ...] - hit-test list, rebuilt every paint
        self._hovered = None

        self._font_small = QFont("Consolas", 8)
        self._font_small_bold = QFont("Consolas", 8)
        self._font_small_bold.setBold(True)
        self._font_med = QFont("Consolas", 9)
        self._font_med_bold = QFont("Consolas", 9)
        self._font_med_bold.setBold(True)
        self._font_label = QFont("Consolas", 10)
        self._font_label_bold = QFont("Consolas", 10)
        self._font_label_bold.setBold(True)
        self._font_dec_title = QFont("Consolas", 11)

    # ---------------- geometry (ported 1:1 from EQMountApp) ----------------
    def view_span(self):
        return 360.0 / self.zoom, 180.0 / self.zoom

    def view_bounds(self):
        ra_span, dec_span = self.view_span()
        ra_half, dec_half = ra_span / 2.0, dec_span / 2.0
        return (self.center_ra - ra_half, self.center_ra + ra_half,
                self.center_dec - dec_half, self.center_dec + dec_half)

    def clamp_center(self):
        ra_span, dec_span = self.view_span()
        if ra_span >= 360.0:
            self.center_ra = 180.0
        else:
            self.center_ra = max(ra_span / 2.0, min(360.0 - ra_span / 2.0, self.center_ra))
        if dec_span >= 180.0:
            self.center_dec = 0.0
        else:
            self.center_dec = max(-90.0 + dec_span / 2.0, min(90.0 - dec_span / 2.0, self.center_dec))

    @staticmethod
    def ra_to_x(ra, w, margin, ra_min, ra_span):
        return margin + (w - 2 * margin) * (1.0 - (ra - ra_min) / ra_span)

    @staticmethod
    def dec_to_y(dec, h, margin, dec_min, dec_span):
        return margin + (h - 2 * margin) * (1.0 - (dec - dec_min) / dec_span)

    @staticmethod
    def x_to_ra(x, w, margin, ra_min, ra_span):
        return ra_min + ra_span * (1.0 - (x - margin) / (w - 2 * margin))

    @staticmethod
    def y_to_dec(y, h, margin, dec_min, dec_span):
        return dec_min + dec_span * (1.0 - (y - margin) / (h - 2 * margin))

    @staticmethod
    def pick_grid_step(span_deg, target_lines=VIZ_GRID_TARGET_LINES):
        span_deg = max(span_deg, 1e-9)
        raw_step = span_deg / target_lines
        magnitude = 10 ** math.floor(math.log10(raw_step))
        for mult in (1, 2, 3, 6, 10):
            step = mult * magnitude
            if step >= raw_step - 1e-12:
                return step
        return 10 * magnitude

    @staticmethod
    def grid_line_style(value, step):
        def near_multiple(v, m):
            if m <= 1e-12:
                return False
            r = v / m
            return abs(r - round(r)) < 1e-6
        if abs(value) < 1e-9 or near_multiple(value, step * 10):
            return QColor("#666688"), 1.5
        elif near_multiple(value, step * 5):
            return QColor("#444455"), 1.0
        return QColor("#3a3a4d"), 1.0

    def current_lst_deg(self):
        """Approximate LST in degrees - identical formula to EQMountApp._get_current_lst_deg."""
        utc = datetime.now(timezone.utc)
        year, month, day = utc.year, utc.month, utc.day
        hour = utc.hour + utc.minute / 60.0 + utc.second / 3600.0
        if month <= 2:
            year -= 1
            month += 12
        a = year // 100
        b = 2 - a + (a // 4)
        jd = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + b - 1524.5
        jd += hour / 24.0
        d = jd - 2451545.0
        gmst = (18.697374558 + 24.06570982441908 * d) % 24
        lst_h = (gmst + self.lon / 15.0) % 24
        return lst_h * 15.0

    def bearing_to_sun(self, obj_ra, obj_dec):
        if self.sun_ra is None:
            return 90.0
        delta_ra = ((self.sun_ra - obj_ra + 180.0) % 360.0) - 180.0
        delta_dec = self.sun_dec - obj_dec
        return math.degrees(math.atan2(delta_ra, delta_dec)) % 360.0

    def star_mag_limit_for_zoom(self):
        t = math.log10(max(self.zoom, VIZ_ZOOM_MIN)) / math.log10(VIZ_ZOOM_MAX)
        t = max(0.0, min(1.0, t))
        return 3.2 + t * (sky_catalog.STAR_MAG_LIMIT - 3.2)

    def _milky_way_bands_cached(self):
        if self._milky_way_bands is None:
            bands = []
            n_steps = int(round(360.0 / MILKY_WAY_L_STEP_DEG))
            for half_width, color in zip(MILKY_WAY_BAND_HALF_WIDTHS_DEG, MILKY_WAY_BAND_COLORS):
                top = [_galactic_to_radec(i * MILKY_WAY_L_STEP_DEG, half_width) for i in range(n_steps + 1)]
                bot = [_galactic_to_radec(i * MILKY_WAY_L_STEP_DEG, -half_width) for i in range(n_steps + 1)]
                quads = []
                for i in range(n_steps):
                    ra_t0, dec_t0 = top[i]
                    ra_t1, dec_t1 = top[i + 1]
                    ra_b0, dec_b0 = bot[i]
                    ra_b1, dec_b1 = bot[i + 1]
                    if abs(ra_t1 - ra_t0) > 180.0 or abs(ra_b1 - ra_b0) > 180.0:
                        continue
                    quads.append(((ra_t0, dec_t0), (ra_t1, dec_t1), (ra_b1, dec_b1), (ra_b0, dec_b0)))
                bands.append((QColor(color), quads))
            self._milky_way_bands = bands
        return self._milky_way_bands

    # ---------------- painting ----------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        margin = 12
        cx, cy = w // 2, h // 2

        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min

        p.fillRect(self.rect(), QColor("#0a0a0f"))
        p.setPen(QPen(QColor("#222233"), 2))
        p.setBrush(QBrush(QColor("#111118")))
        p.drawRect(QRectF(8, 8, w - 16, h - 16))

        self._draw_horizon(p, w, h, margin, ra_min, ra_max, dec_min, dec_max)
        self._draw_milky_way(p, w, h, margin, ra_min, ra_max, dec_min, dec_max)
        self._draw_meridian_limit_zone(p, w, h, margin, ra_min, ra_max, dec_min, dec_max)
        self._draw_grid(p, w, h, margin, ra_min, ra_max, dec_min, dec_max, ra_span, dec_span)
        self._draw_sky_objects(p, w, h, margin, ra_min, ra_max, dec_min, dec_max)
        pixels_per_deg = (h - 2 * margin) / dec_span
        self._draw_realtime_bodies(p, w, h, margin, ra_min, ra_max, dec_min, dec_max, pixels_per_deg)
        self._draw_crosshairs_and_cardinals(p, w, h, margin, ra_min, ra_max, dec_min, dec_max, ra_span, dec_span)
        self._draw_corner_labels(p, w, h, cx, cy, ra_min, ra_max, dec_min, dec_max)
        self._draw_camera_and_reticle(p, w, h, cx, cy, margin, ra_min, ra_span, dec_min, dec_span, pixels_per_deg)
        p.end()

    def _draw_horizon(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        lat = self.lat
        lst = self.current_lst_deg()

        def horizon_dec_for_ra(ra):
            ha = (lst - ra) % 360.0
            ha_rad = math.radians(ha)
            lat_rad = math.radians(lat)
            if abs(math.sin(lat_rad)) < 1e-8:
                return 0.0
            tan_dec = -(math.cos(lat_rad) / math.sin(lat_rad)) * math.cos(ha_rad)
            return max(-90.0, min(90.0, math.degrees(math.atan(tan_dec))))

        margin_deg = max(1.0, ra_span * 0.05)
        sample_start = max(0.0, ra_min - margin_deg)
        sample_end = min(360.0, ra_max + margin_deg)
        step = max(0.05, min(3.0, (sample_end - sample_start) / 20.0))

        horizon_points = []
        ra = sample_start
        while ra <= sample_end + 1e-9:
            dec = horizon_dec_for_ra(ra)
            x = self.ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self.dec_to_y(dec, h, margin, dec_min, dec_span)
            horizon_points.append(QPointF(x, y))
            ra += step
        if len(horizon_points) < 2:
            return

        if ra_min <= 1e-6 and ra_max >= 360.0 - 1e-6:
            y0 = self.dec_to_y(horizon_dec_for_ra(0.0), h, margin, dec_min, dec_span)
            y360 = self.dec_to_y(horizon_dec_for_ra(360.0), h, margin, dec_min, dec_span)
            x0, x360 = float(margin), float(w - margin)
        else:
            x0, y0 = horizon_points[0].x(), horizon_points[0].y()
            x360, y360 = horizon_points[-1].x(), horizon_points[-1].y()

        horizon_dec_at_meridian = horizon_dec_for_ra(lst)
        ground_toward_dec_min = lat >= horizon_dec_at_meridian
        ground_edge_y = float(h - margin) if ground_toward_dec_min else float(margin)

        poly = QPolygonF([QPointF(x0, y0)] + horizon_points + [QPointF(x360, y360),
                          QPointF(x360, ground_edge_y), QPointF(x0, ground_edge_y)])
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#1a2e2e")))
        p.drawPolygon(poly)

        line_poly = QPolygonF([QPointF(x0, y0)] + horizon_points + [QPointF(x360, y360)])
        p.setPen(QPen(QColor("#555577"), 1))
        p.drawPolyline(line_poly)

    def _draw_milky_way(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        pad = 10.0
        p.setPen(Qt.NoPen)
        for color, quads in self._milky_way_bands_cached():
            p.setBrush(QBrush(color))
            for quad in quads:
                quad_ra_min = min(pt[0] for pt in quad)
                quad_ra_max = max(pt[0] for pt in quad)
                quad_dec_min = min(pt[1] for pt in quad)
                quad_dec_max = max(pt[1] for pt in quad)
                if quad_ra_max < ra_min - pad or quad_ra_min > ra_max + pad:
                    continue
                if quad_dec_max < dec_min - pad or quad_dec_min > dec_max + pad:
                    continue
                poly = QPolygonF()
                for ra, dec in quad:
                    poly.append(QPointF(self.ra_to_x(ra, w, margin, ra_min, ra_span),
                                         self.dec_to_y(dec, h, margin, dec_min, dec_span)))
                p.drawPolygon(poly)

    def _draw_meridian_limit_zone(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        lst = self.current_lst_deg()
        if self.telescope_flipped:
            danger_lo, danger_hi = lst % 360.0, (lst + 180.0) % 360.0
        else:
            danger_lo, danger_hi = (lst - 180.0) % 360.0, lst % 360.0
        intervals = [(danger_lo, danger_hi)] if danger_lo <= danger_hi else [(danger_lo, 360.0), (0.0, danger_hi)]
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        dec_bands = []
        upper_lo = max(MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG, dec_min)
        if upper_lo < dec_max:
            dec_bands.append((upper_lo, dec_max))
        lower_hi = min(MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG, dec_max)
        if dec_min < lower_hi:
            dec_bands.append((dec_min, lower_hi))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#3a2814")))
        for lo, hi in intervals:
            clip_lo, clip_hi = max(lo, ra_min), min(hi, ra_max)
            if clip_lo >= clip_hi:
                continue
            x_lo = self.ra_to_x(clip_lo, w, margin, ra_min, ra_span)
            x_hi = self.ra_to_x(clip_hi, w, margin, ra_min, ra_span)
            x_left, x_right = min(x_lo, x_hi), max(x_lo, x_hi)
            for dec_lo, dec_hi in dec_bands:
                y_top = self.dec_to_y(dec_hi, h, margin, dec_min, dec_span)
                y_bot = self.dec_to_y(dec_lo, h, margin, dec_min, dec_span)
                p.drawRect(QRectF(x_left, y_top, x_right - x_left, y_bot - y_top))

    def _draw_grid(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max, ra_span, dec_span):
        dec_step = self.pick_grid_step(dec_span)
        d = math.ceil(dec_min / dec_step) * dec_step
        while d <= dec_max + 1e-9:
            y = self.dec_to_y(d, h, margin, dec_min, dec_span)
            col, wd = self.grid_line_style(d, dec_step)
            p.setPen(QPen(col, wd))
            p.drawLine(QPointF(8, y), QPointF(w - 8, y))
            d += dec_step
        ra_step = self.pick_grid_step(ra_span)
        r = math.ceil(ra_min / ra_step) * ra_step
        while r <= ra_max + 1e-9:
            x = self.ra_to_x(r, w, margin, ra_min, ra_span)
            col, wd = self.grid_line_style(r, ra_step)
            p.setPen(QPen(col, wd))
            p.drawLine(QPointF(x, 8), QPointF(x, h - 8))
            r += ra_step

    def _draw_sky_objects(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        """Draws stars + DSOs and rebuilds self._visible_hits (hover/click hit-test list) fresh
        every paint - same LOD-tier/bisect/small-DSO-fast-path approach as tracker_gui.py's
        _draw_sky_objects (see GUI_VERSION 1.0.4's changelog for why the small-DSO fast path
        exists)."""
        self._visible_hits = []
        if not self.sky_stars_by_ra and not self.sky_dso:
            return
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        pixels_per_deg = (h - 2 * margin) / dec_span
        mag_limit = self.star_mag_limit_for_zoom()

        if self.constellations_enabled and self.const_line_segments:
            p.setPen(QPen(QColor("#4a5a7a"), 1))
            for hip_a, hip_b in self.const_line_segments:
                star_a = self.star_by_hip.get(hip_a)
                star_b = self.star_by_hip.get(hip_b)
                if not star_a or not star_b:
                    continue
                ax = self.ra_to_x(star_a["ra"], w, margin, ra_min, ra_span)
                ay = self.dec_to_y(star_a["dec"], h, margin, dec_min, dec_span)
                bx = self.ra_to_x(star_b["ra"], w, margin, ra_min, ra_span)
                by = self.dec_to_y(star_b["dec"], h, margin, dec_min, dec_span)
                p.drawLine(QPointF(ax, ay), QPointF(bx, by))

        tier_stars_by_ra, tier_ra_values = self.sky_stars_by_ra, self.sky_stars_ra_values
        for cutoff, tier_stars, tier_ra in self.star_tiers:
            if cutoff >= mag_limit:
                tier_stars_by_ra, tier_ra_values = tier_stars, tier_ra
                break

        ra_lo = bisect.bisect_left(tier_ra_values, ra_min)
        ra_hi = bisect.bisect_right(tier_ra_values, ra_max)
        p.setPen(Qt.NoPen)
        for s in tier_stars_by_ra[ra_lo:ra_hi]:
            if s["mag"] > mag_limit:
                continue
            ra, dec = s["ra"], s["dec"]
            if not (dec_min <= dec <= dec_max):
                continue
            x = self.ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self.dec_to_y(dec, h, margin, dec_min, dec_span)
            radius = max(0.6, min(3.0, (mag_limit - s["mag"]) * 0.5 + 0.6))
            p.setBrush(QBrush(QColor(s["color"])))
            p.drawEllipse(QPointF(x, y), radius, radius)
            hip = s.get("hip")
            aliases = s.get("aliases") or []
            name = s.get("name") or (f"HIP {hip}" if hip else None) or (aliases[0] if aliases else None) or "unnamed star"
            self._visible_hits.append({"x": x, "y": y, "ra": ra, "dec": dec, "radius": 12.0,
                                        "name": name, "extra": f"mag {s['mag']:.1f}", "labeled": False})

        for d in self.sky_dso:
            ra, dec = d["ra"], d["dec"]
            if not (ra_min <= ra <= ra_max and dec_min <= dec <= dec_max):
                continue
            x = self.ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self.dec_to_y(dec, h, margin, dec_min, dec_span)
            shape, color = _dso_render_style(d["type"])
            maj = d.get("size_maj_arcmin")
            min_ax = d.get("size_min_arcmin") or maj
            if self.min_dso_size_filter_enabled:
                sensor_px = (maj / 60.0 / CAMERA_FOV_DEG_PER_PIXEL) if maj else 0.0
                if sensor_px < self.min_dso_size_px:
                    continue
            if maj:
                half_w = max(3.0, (maj / 60.0 / 2.0) * pixels_per_deg)
                half_h = max(3.0, ((min_ax or maj) / 60.0 / 2.0) * pixels_per_deg)
            else:
                half_w = half_h = 4.0
            if shape == "cluster":
                p.setBrush(Qt.NoBrush)
                p.setPen(_dashed_pen(color))
                p.drawEllipse(QPointF(x, y), half_w, half_h)
            elif shape == "ring":
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(color, 1.5))
                p.drawEllipse(QPointF(x, y), half_w, half_h)
            elif half_w < 6.0 and half_h < 6.0:
                # Below ~6px, a rotated/dashed outline reads as a plain dot anyway - see
                # tracker_gui.py's GUI_VERSION 1.0.4 changelog for the profiling behind this.
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(color))
                p.drawEllipse(QPointF(x, y), half_w, half_h)
            else:
                p.setBrush(Qt.NoBrush)
                p.setPen(_dashed_pen(color))
                poly = _rotated_ellipse_points(x, y, half_w, half_h, d.get("pos_ang_deg") or 0.0)
                p.drawPolygon(poly)
            already_labeled = half_w >= 8 and self.zoom >= 3.0
            if already_labeled:
                label = d.get("name") or d["designation"]
                _draw_centered_text(p, x, y - half_h - 8, label, self._font_small, color)
            self._visible_hits.append({"x": x, "y": y, "ra": ra, "dec": dec,
                                        "radius": max(12.0, half_w, half_h),
                                        "name": d.get("name") or d["designation"],
                                        "extra": f"{d['designation']} · {d['type']}",
                                        "labeled": already_labeled})

    def _draw_realtime_bodies(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max, pixels_per_deg):
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        if self.iss_enabled and self.iss_ra is not None and ra_min <= self.iss_ra <= ra_max and dec_min <= self.iss_dec <= dec_max:
            ix = self.ra_to_x(self.iss_ra, w, margin, ra_min, ra_span)
            iy = self.dec_to_y(self.iss_dec, h, margin, dec_min, dec_span)
            iss_color = QColor("#00ffaa" if self.iss_above else "#336655")
            r = 4
            p.setPen(QPen(iss_color, 1.5))
            p.drawLine(QPointF(ix - r, iy), QPointF(ix + r, iy))
            p.drawLine(QPointF(ix, iy - r), QPointF(ix, iy + r))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(ix, iy), r, r)
            _draw_centered_text(p, ix, iy - r - 8, "ISS", self._font_small_bold, iss_color)
            self._visible_hits.append({"x": ix, "y": iy, "ra": self.iss_ra, "dec": self.iss_dec,
                                        "radius": 12.0, "name": "ISS",
                                        "extra": "above horizon" if self.iss_above else "below horizon",
                                        "labeled": True})

        if self.sun_ra is not None and ra_min <= self.sun_ra <= ra_max and dec_min <= self.sun_dec <= dec_max:
            sx = self.ra_to_x(self.sun_ra, w, margin, ra_min, ra_span)
            sy = self.dec_to_y(self.sun_dec, h, margin, dec_min, dec_span)
            sr = max(4.0, (self.sun_diam / 2.0) * pixels_per_deg)
            p.setPen(QPen(QColor("#aa7700"), 1))
            p.setBrush(QBrush(QColor("#ffcc33")))
            p.drawEllipse(QPointF(sx, sy), sr, sr)
            _draw_centered_text(p, sx, sy - sr - 8, "Sun", self._font_small_bold, QColor("#ffcc33"))
            self._visible_hits.append({"x": sx, "y": sy, "ra": self.sun_ra, "dec": self.sun_dec,
                                        "radius": max(12.0, sr), "name": "Sun",
                                        "extra": f"diam {self.sun_diam*60:.1f}'", "labeled": True})

        if self.moon_ra is not None and ra_min <= self.moon_ra <= ra_max and dec_min <= self.moon_dec <= dec_max:
            mx = self.ra_to_x(self.moon_ra, w, margin, ra_min, ra_span)
            my = self.dec_to_y(self.moon_dec, h, margin, dec_min, dec_span)
            mr = max(4.0, (self.moon_diam / 2.0) * pixels_per_deg)
            bearing_deg = self.bearing_to_sun(self.moon_ra, self.moon_dec)
            self._draw_illuminated_disc(p, mx, my, mr, bearing_deg, self.moon_illum,
                                         QColor("#e8e8e0"), QColor("#1a1a22"), QColor("#888888"))
            waxwane = "waxing" if self.moon_waxing else "waning"
            _draw_centered_text(p, mx, my - mr - 8, f"Moon ({self.moon_illum*100:.0f}%)",
                                 self._font_small_bold, QColor("#cccccc"))
            self._visible_hits.append({"x": mx, "y": my, "ra": self.moon_ra, "dec": self.moon_dec,
                                        "radius": max(12.0, mr), "name": "Moon",
                                        "extra": f"{self.moon_illum*100:.0f}% illuminated, {waxwane}",
                                        "labeled": True})

        planet_colors = {
            "Mercury": "#aaaaaa", "Venus": "#e8dcb0", "Mars": "#cc6644",
            "Jupiter": "#d8b088", "Saturn": "#e0d0a0", "Uranus": "#9fd8d8", "Neptune": "#6e8fd8",
        }
        PLANET_MIN_R, PLANET_PHASE_R = 3.5, 6.0
        for pname, (pra, pdec, pang_diam, pring_ang_diam, pillum) in self.planet_positions.items():
            if not (ra_min <= pra <= ra_max and dec_min <= pdec <= dec_max):
                continue
            px = self.ra_to_x(pra, w, margin, ra_min, ra_span)
            py = self.dec_to_y(pdec, h, margin, dec_min, dec_span)
            pcolor = QColor(planet_colors.get(pname, "#cccccc"))
            pr = max(PLANET_MIN_R, (pang_diam / 2.0) * pixels_per_deg)
            prr = 0.0
            if pname == "Saturn" and pring_ang_diam:
                prr = max(pr + 1.5, (pring_ang_diam / 2.0) * pixels_per_deg)
                p.setPen(QPen(pcolor, max(1.0, prr * 0.12)))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(px, py), prr, prr * 0.45)
            if pr >= PLANET_PHASE_R:
                bearing_deg = self.bearing_to_sun(pra, pdec)
                self._draw_illuminated_disc(p, px, py, pr, bearing_deg, pillum,
                                             pcolor, QColor(_darken_hex_color(pcolor.name())), pcolor)
            else:
                p.setPen(QPen(QColor("#000000"), 0.5))
                p.setBrush(QBrush(pcolor))
                p.drawEllipse(QPointF(px, py), pr, pr)
            _draw_centered_text(p, px, py - pr - 8, pname, self._font_small_bold, pcolor)
            self._visible_hits.append({"x": px, "y": py, "ra": pra, "dec": pdec,
                                        "radius": max(12.0, pr, prr), "name": pname,
                                        "extra": f"planet, {pillum*100:.0f}% illuminated", "labeled": True})

    def _draw_illuminated_disc(self, p, x, y, r, bearing_deg, illum_fraction, lit_color, dark_color, outline_color):
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(dark_color))
        p.drawEllipse(QPointF(x, y), r, r)
        path_lit, path_term = _illuminated_disc_path(x, y, r, bearing_deg, illum_fraction)
        p.setBrush(QBrush(lit_color))
        p.drawPath(path_lit)
        terminator_color = lit_color if illum_fraction > 0.5 else dark_color
        p.setBrush(QBrush(terminator_color))
        p.drawPath(path_term)
        p.setPen(QPen(outline_color, 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(x, y), r, r)

    def _draw_crosshairs_and_cardinals(self, p, w, h, margin, ra_min, ra_max, dec_min, dec_max, ra_span, dec_span):
        p.setPen(QPen(QColor("#555566"), 1))
        if dec_min <= 0.0 <= dec_max:
            cy0 = self.dec_to_y(0.0, h, margin, dec_min, dec_span)
            p.drawLine(QPointF(8, cy0), QPointF(w - 8, cy0))
        if ra_min <= 180.0 <= ra_max:
            cx0 = self.ra_to_x(180.0, w, margin, ra_min, ra_span)
            p.drawLine(QPointF(cx0, 8), QPointF(cx0, h - 8))

        lst = self.current_lst_deg()
        cardinal_ra = {"S": lst % 360.0, "N": (lst + 180.0) % 360.0,
                       "E": (lst + 90.0) % 360.0, "W": (lst - 90.0) % 360.0}
        pen = _dashed_pen(QColor("#aa8844"))
        for label, cra in cardinal_ra.items():
            if ra_min <= cra <= ra_max:
                line_x = self.ra_to_x(cra, w, margin, ra_min, ra_span)
                p.setPen(pen)
                p.drawLine(QPointF(line_x, 8), QPointF(line_x, h - 8))
                _draw_centered_text(p, line_x, 20, label, self._font_label_bold, QColor("#ddaa55"))

    def _draw_corner_labels(self, p, w, h, cx, cy, ra_min, ra_max, dec_min, dec_max):
        col = QColor("#8888aa")
        _draw_centered_text(p, 18, 18, f"{dec_max:+.2f}°", self._font_label, col, anchor="w")
        _draw_centered_text(p, 18, h - 16, f"{dec_min:+.2f}°", self._font_label, col, anchor="w")
        _draw_centered_text(p, w - 16, cy - 12, f"RA {ra_max:.2f}° → {ra_min:.2f}°",
                             self._font_label, col, anchor="e")
        _draw_centered_text(p, cx, 18, "DEC", self._font_dec_title, QColor("#aaaacc"))
        zoom_hint = "scroll to zoom, drag to pan, middle-click to reset, double-click to set target" \
            if self.zoom <= VIZ_ZOOM_MIN + 1e-6 \
            else f"zoom {self.zoom:.1f}x - drag to pan, middle-click to reset, double-click to set target"
        _draw_centered_text(p, cx, h - 16, zoom_hint, QFont("Consolas", 9), QColor("#666688"))

    def _draw_camera_and_reticle(self, p, w, h, cx, cy, margin, ra_min, ra_span, dec_min, dec_span, pixels_per_deg):
        x = self.ra_to_x(self.current_ra % 360.0, w, margin, ra_min, ra_span)
        y = self.dec_to_y(self.current_dec, h, margin, dec_min, dec_span)
        cam_half_w, cam_half_h = _camera_fov_half_size(pixels_per_deg)

        p.setPen(_dashed_pen(QColor("#5599ff"), 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(_camera_rect_points(x, y, cam_half_w, cam_half_h, self.camera_orientation_deg))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#5599ff")))
        p.drawPolygon(_camera_top_marker_points(x, y, cam_half_w, cam_half_h, self.camera_orientation_deg))

        fov_radius = max(5.0, math.hypot(cam_half_w, cam_half_h))
        p.setPen(QPen(self.stability_color, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(x, y), fov_radius, fov_radius)

        center_ref_radius = 8 + 2 + 6
        for hit in self._visible_hits:
            if hit["name"] == getattr(self, "target_object_name", None):
                center_ref_radius = max(center_ref_radius, hit["radius"])
                break
        center_ref_radius *= 1.1
        p.setPen(_dashed_pen(self.stability_color))
        p.drawEllipse(QPointF(x, y), center_ref_radius, center_ref_radius)

        tx = self.ra_to_x(self.target_ra % 360.0, w, margin, ra_min, ra_span)
        ty = self.dec_to_y(self.target_dec, h, margin, dec_min, dec_span)
        radius, gap, tick_len = 8, 2, 6
        p.setPen(QPen(QColor("#ffaa00"), 1.5))
        p.drawEllipse(QPointF(tx, ty), radius, radius)
        p.drawLine(QPointF(tx, ty - radius - gap), QPointF(tx, ty - radius - gap - tick_len))
        p.drawLine(QPointF(tx, ty + radius + gap), QPointF(tx, ty + radius + gap + tick_len))
        p.drawLine(QPointF(tx + radius + gap, ty), QPointF(tx + radius + gap + tick_len, ty))
        p.drawLine(QPointF(tx - radius - gap, ty), QPointF(tx - radius - gap - tick_len, ty))

    # ---------------- mouse/wheel interaction ----------------
    def wheelEvent(self, event):
        """Zoom centered on the cursor - same math as EQMountApp._on_viz_zoom."""
        w, h = max(200, self.width()), max(150, self.height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        pos = event.position()
        frac_x = (pos.x() - margin) / max(1, (w - 2 * margin))
        frac_y = (pos.y() - margin) / max(1, (h - 2 * margin))
        cursor_ra = self.x_to_ra(pos.x(), w, margin, ra_min, ra_span)
        cursor_dec = dec_min + dec_span * (1.0 - frac_y)

        steps = event.angleDelta().y() / 120.0
        zoom_factor = VIZ_ZOOM_STEP_BASE ** steps
        new_zoom = max(VIZ_ZOOM_MIN, min(VIZ_ZOOM_MAX, self.zoom * zoom_factor))
        if new_zoom == self.zoom:
            return
        self.zoom = new_zoom
        new_ra_span, new_dec_span = self.view_span()
        self.center_ra = cursor_ra + new_ra_span * (frac_x - 0.5)
        self.center_dec = cursor_dec + new_dec_span * (0.5 - (1.0 - frac_y))
        self.clamp_center()
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.zoom = VIZ_ZOOM_MIN
            self.center_ra, self.center_dec = 180.0, 0.0
            self.view_mode = "FREE"
            self.update()
            return
        if event.button() == Qt.LeftButton:
            self._pan_last = event.position()

    def mouseMoveEvent(self, event):
        pos = event.position()
        w, h = max(200, self.width()), max(150, self.height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min

        if self._pan_last is not None and (event.buttons() & Qt.LeftButton) and self.zoom > VIZ_ZOOM_MIN:
            if self.view_mode != "FREE":
                self.view_mode = "FREE"
            dx, dy = pos.x() - self._pan_last.x(), pos.y() - self._pan_last.y()
            self.center_ra += dx * ra_span / max(1, (w - 2 * margin))
            self.center_dec += dy * dec_span / max(1, (h - 2 * margin))
            self.clamp_center()
            self._pan_last = pos
            self.update()
            return
        self._pan_last = pos if (event.buttons() & Qt.LeftButton) else None

        # Hover hit-test against the list _draw_sky_objects/_draw_realtime_bodies built on the
        # last paint - same approach as EQMountApp._on_viz_mouse_move.
        ra = self.x_to_ra(pos.x(), w, margin, ra_min, ra_span) % 360.0
        dec = max(-90.0, min(90.0, self.y_to_dec(pos.y(), h, margin, dec_min, dec_span)))
        hit, best_d2 = None, None
        for obj in self._visible_hits:
            d2 = (obj["x"] - pos.x()) ** 2 + (obj["y"] - pos.y()) ** 2
            if d2 < obj["radius"] ** 2 and (best_d2 is None or d2 < best_d2):
                best_d2, hit = d2, obj
        self._hovered = hit
        if hit is None:
            self.hoverInfoChanged.emit(f"Cursor: RA {ra:.4f}°  DEC {dec:+.4f}°")
        else:
            self.hoverInfoChanged.emit(
                f"Cursor: RA {ra:.4f}°  DEC {dec:+.4f}°   |   {hit['name']}  ({hit['extra']})")
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._pan_last = None

    def mouseDoubleClickEvent(self, event):
        """Double-click: target whatever's hovered (precise catalog RA/DEC), or the raw cursor
        sky position otherwise - same behavior as EQMountApp._on_viz_double_click."""
        w, h = max(200, self.width()), max(150, self.height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        if self._hovered is not None:
            ra, dec = self._hovered["ra"], self._hovered["dec"]
        else:
            pos = event.position()
            ra = self.x_to_ra(pos.x(), w, margin, ra_min, ra_span) % 360.0
            dec = max(-90.0, min(90.0, self.y_to_dec(pos.y(), h, margin, dec_min, dec_span)))
        self.targetPicked.emit(ra, dec)

    def leaveEvent(self, event):
        self._hovered = None
        self.hoverInfoChanged.emit("Cursor: -")
        self.update()


# ============================================================
# CATALOG LOADING (simplified port of EQMountApp._build_sky_catalog_state)
# ============================================================
def load_catalog_state():
    """Reads the SAME sky_catalog.json tracker_gui.py builds/caches (via sky_catalog.py) and
    derives the RA-sorted/magnitude-tiered structures SkyViewWidget needs. Returns None if the
    file doesn't exist yet - unlike tracker_gui.py, this app doesn't drive the network
    build/refresh itself (see module docstring's Scope note); run tracker_gui.py at least once
    first to populate sky_catalog.json."""
    if not os.path.exists(sky_catalog.CATALOG_PATH):
        return None
    with open(sky_catalog.CATALOG_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    sky_stars = data.get("stars", [])
    sky_dso = data.get("dso", [])
    for d in sky_dso:
        if "designation" not in d:
            d["designation"] = d.get("messier") or ""
    const_line_segments = data.get("const_lines", [])
    star_by_hip = {s["hip"]: s for s in sky_stars if "hip" in s}
    sky_stars_by_ra = sorted(sky_stars, key=lambda s: s["ra"])
    sky_stars_ra_values = [s["ra"] for s in sky_stars_by_ra]
    star_tiers = []
    for cutoff in STAR_LOD_TIER_MAG_CUTOFFS:
        tier_stars = [s for s in sky_stars_by_ra if s["mag"] <= cutoff]
        star_tiers.append((cutoff, tier_stars, [s["ra"] for s in tier_stars]))
    star_tiers.append((float("inf"), sky_stars_by_ra, sky_stars_ra_values))
    return {
        "sky_stars_by_ra": sky_stars_by_ra, "sky_stars_ra_values": sky_stars_ra_values,
        "star_tiers": star_tiers, "sky_dso": sky_dso,
        "const_line_segments": const_line_segments, "star_by_hip": star_by_hip,
        "n_stars": len(sky_stars), "n_dso": len(sky_dso),
    }


# ============================================================
# BACKGROUND WORKERS
# ============================================================
# Each does its (potentially slow/network) work off the GUI thread and reports back via a Qt
# signal - connecting a background thread's signal to a slot on a QObject that lives on the GUI
# thread is automatically delivered as a queued (thread-safe) call, no manual queue/timer needed
# the way SerialHandler's plain queue.Queue is (see MainWindow._poll_serial_queue).

class _CatalogWorker(QObject):
    loaded = Signal(object)
    failed = Signal(str)

    def run(self):
        try:
            state = load_catalog_state()
        except Exception as e:
            self.failed.emit(str(e))
            return
        self.loaded.emit(state)


class _SolarSystemWorker(QObject):
    updated = Signal(object)
    failed = Signal(str)

    def __init__(self, get_lat_lon_fn):
        super().__init__()
        self.get_lat_lon_fn = get_lat_lon_fn

    def run(self):
        try:
            lat, lon = self.get_lat_lon_fn()
            sun_ra, sun_dec, sun_diam = solar_system.get_sun_info(lat, lon)
            moon_ra, moon_dec, moon_diam, illum, phase, waxing = solar_system.get_moon_info(lat, lon)
            planets = solar_system.get_planets_info(lat, lon)
        except Exception as e:
            self.failed.emit(str(e))
            return
        self.updated.emit((sun_ra, sun_dec, sun_diam, moon_ra, moon_dec, moon_diam,
                            illum, waxing, planets))


class _IssWorker(QObject):
    updated = Signal(object)
    failed = Signal(str)

    def __init__(self, get_lat_lon_fn):
        super().__init__()
        self.get_lat_lon_fn = get_lat_lon_fn

    def run(self):
        try:
            iss_tracker.ensure_tle_current()
            lat, lon = self.get_lat_lon_fn()
            ra, dec, alt, az, above = iss_tracker.get_current_radec(lat, lon)
        except Exception as e:
            self.failed.emit(str(e))
            return
        self.updated.emit((ra, dec, above))


def _run_in_thread(worker):
    """Starts `worker.run()` on a daemon thread. The caller is responsible for keeping `worker`
    referenced (see MainWindow._workers) until it reports back - nothing here holds one, and a
    QObject with no live Python reference can be garbage-collected out from under its own
    in-flight thread."""
    thread = threading.Thread(target=worker.run, daemon=True)
    thread.start()
    return thread


# ============================================================
# MAIN WINDOW
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EQ Mount DIY Go-To Kit - Qt Controller (experimental)")
        self.resize(1440, 900)

        self.msg_queue = queue.Queue()
        self.serial = SerialHandler(self.msg_queue, get_time_fn=lambda: datetime.now(timezone.utc))

        self.current_ra = self.current_dec = 0.0
        self.target_ra = self.target_dec = 0.0
        self.mount_ra_angle = self.mount_dec_angle = 0.0
        self.speed_ra = self.speed_dec = 0.0
        self.err_ra = self.err_dec = 0.0
        self.ra_pos_history = deque()
        self.dec_pos_history = deque()
        self.ra_offset = 0.0
        self.dec_offset = 0.0
        self.tracking = False
        self.arduino_debug_enabled = False
        self._last_pos_ui_update = 0.0
        self._workers = []  # keeps background QObjects referenced while their thread runs
        self._search_index = []  # built alongside the catalog - see _on_catalog_loaded

        # Named calibration points/stars for the Sync dropdown - same values/HIP numbers as
        # EQMountApp.cal_stars/_cal_star_hip; "East"/"West" resolve from LST at sync time (see
        # _sync), named stars prefer the exact catalog RA/DEC (via viz.star_by_hip) once loaded.
        self.cal_stars = {
            "East (horizon, DEC=0)": "EAST",
            "West (horizon, DEC=0)": "WEST",
            "Vega (alpha Lyrae)": (279.234, 38.784),
            "Arcturus (alpha Bootis)": (213.915, 19.182),
            "Altair (alpha Aquilae)": (297.696, 8.868),
            "Custom (use fields below)": None,
        }
        self.cal_star_hip = {
            "Vega (alpha Lyrae)": 91262, "Arcturus (alpha Bootis)": 69673, "Altair (alpha Aquilae)": 97649,
        }

        self.viz = SkyViewWidget()
        self.viz.targetPicked.connect(self._on_target_picked)
        self.viz.hoverInfoChanged.connect(self._on_hover_info)

        self._build_ui()
        self._set_arduino_controls_enabled(False)

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_serial_queue)
        self.poll_timer.start(POLL_INTERVAL_MS)

        self.resync_timer = QTimer(self)
        self.resync_timer.timeout.connect(lambda: self.serial.send_time_if_connected())
        self.resync_timer.start(60000)

        self.solar_timer = QTimer(self)
        self.solar_timer.timeout.connect(self._trigger_solar_system_update)
        self.solar_timer.start(2000)

        self.iss_timer = QTimer(self)
        self.iss_timer.timeout.connect(self._trigger_iss_update)
        self.iss_timer.start(5000)

        self._load_catalog()
        self._refresh_ports()
        QTimer.singleShot(500, self._trigger_solar_system_update)
        QTimer.singleShot(1000, self._trigger_iss_update)

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QWidget()
        outer = QHBoxLayout(central)
        self.setCentralWidget(central)

        left = QVBoxLayout()
        outer.addLayout(left, stretch=3)
        right = QVBoxLayout()
        right.setSpacing(8)
        outer.addLayout(right, stretch=1)

        # ---- top row: connection + location (left column, above the status bar) ----
        top_row = QHBoxLayout()
        conn_box = QGroupBox("Connection")
        conn_l = QGridLayout(conn_box)
        self.port_combo = QComboBox()
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._disconnect)
        self.disconnect_btn.setEnabled(False)
        self.conn_status = QLabel("● Disconnected")
        self.conn_status.setStyleSheet("color: gray;")
        conn_l.addWidget(self.port_combo, 0, 0, 1, 2)
        conn_l.addWidget(refresh_btn, 0, 2)
        conn_l.addWidget(self.connect_btn, 1, 0)
        conn_l.addWidget(self.disconnect_btn, 1, 1)
        conn_l.addWidget(self.conn_status, 1, 2)
        top_row.addWidget(conn_box, stretch=2)

        loc_box = QGroupBox("Location")
        loc_l = QGridLayout(loc_box)
        self.lat_edit = QLineEdit("0.0")
        self.lon_edit = QLineEdit("0.0")
        set_loc_btn = QPushButton("Set")
        set_loc_btn.clicked.connect(self._set_location)
        loc_l.addWidget(QLabel("Lat:"), 0, 0)
        loc_l.addWidget(self.lat_edit, 0, 1)
        loc_l.addWidget(QLabel("Lon:"), 0, 2)
        loc_l.addWidget(self.lon_edit, 0, 3)
        loc_l.addWidget(set_loc_btn, 0, 4)
        top_row.addWidget(loc_box, stretch=2)
        left.addLayout(top_row)

        # ---- prominent status bar ----
        self.status_display = QLabel("DISCONNECTED - Select COM port and click Connect")
        self.status_display.setAlignment(Qt.AlignCenter)
        self.status_display.setStyleSheet(
            "background-color: #1a1a2e; color: #ffaa00; font-weight: bold; font-size: 14px; padding: 6px;")
        left.addWidget(self.status_display)

        # ---- position readout panel ----
        pos_box = QGroupBox("Telescope Position (Sky)")
        pos_l = QGridLayout(pos_box)
        mono = "font-family: Consolas;"
        self.sky_ra_label = QLabel("000.0000°")
        self.sky_ra_label.setStyleSheet(mono + "color: #33ffaa; font-size: 16px; font-weight: bold;")
        self.sky_dec_label = QLabel("+00.0000°")
        self.sky_dec_label.setStyleSheet(mono + "color: #33ffaa; font-size: 16px; font-weight: bold;")
        self.mount_ra_label = QLabel("0.0000°")
        self.mount_dec_label = QLabel("0.0000°")
        self.speed_ra_label = QLabel("0.000000 °/s")
        self.speed_dec_label = QLabel("0.000000 °/s")
        for lbl in (self.mount_ra_label, self.mount_dec_label, self.speed_ra_label, self.speed_dec_label):
            lbl.setStyleSheet(mono + "color: #aaaacc;")

        self.ra_offset_edit = QLineEdit("0.0")
        self.ra_offset_edit.setFixedWidth(60)
        self.dec_offset_edit = QLineEdit("0.0")
        self.dec_offset_edit.setFixedWidth(60)
        ra_off_minus = QPushButton("-")
        ra_off_plus = QPushButton("+")
        dec_off_minus = QPushButton("-")
        dec_off_plus = QPushButton("+")
        ra_off_minus.clicked.connect(lambda: self._adjust_offset(-1, 0))
        ra_off_plus.clicked.connect(lambda: self._adjust_offset(1, 0))
        dec_off_minus.clicked.connect(lambda: self._adjust_offset(0, -1))
        dec_off_plus.clicked.connect(lambda: self._adjust_offset(0, 1))
        for b in (ra_off_minus, ra_off_plus, dec_off_minus, dec_off_plus):
            b.setFixedWidth(28)

        pos_l.addWidget(QLabel("Sky RA:"), 0, 0)
        pos_l.addWidget(self.sky_ra_label, 0, 1)
        pos_l.addWidget(QLabel("Mount:"), 0, 2)
        pos_l.addWidget(self.mount_ra_label, 0, 3)
        pos_l.addWidget(QLabel("Speed:"), 0, 4)
        pos_l.addWidget(self.speed_ra_label, 0, 5)
        pos_l.addWidget(QLabel("Offset:"), 0, 6)
        pos_l.addWidget(ra_off_minus, 0, 7)
        pos_l.addWidget(self.ra_offset_edit, 0, 8)
        pos_l.addWidget(ra_off_plus, 0, 9)

        pos_l.addWidget(QLabel("Sky DEC:"), 1, 0)
        pos_l.addWidget(self.sky_dec_label, 1, 1)
        pos_l.addWidget(QLabel("Mount:"), 1, 2)
        pos_l.addWidget(self.mount_dec_label, 1, 3)
        pos_l.addWidget(QLabel("Speed:"), 1, 4)
        pos_l.addWidget(self.speed_dec_label, 1, 5)
        pos_l.addWidget(QLabel("Offset:"), 1, 6)
        pos_l.addWidget(dec_off_minus, 1, 7)
        pos_l.addWidget(self.dec_offset_edit, 1, 8)
        pos_l.addWidget(dec_off_plus, 1, 9)

        self.offset_inc_edit = QLineEdit("0.01")
        self.offset_inc_edit.setFixedWidth(60)
        pos_l.addWidget(QLabel("Offset increment:"), 2, 0, 1, 2)
        pos_l.addWidget(self.offset_inc_edit, 2, 2)

        self.error_label = QLabel("Error: RA 0.0000°  DEC 0.0000°")
        self.error_label.setStyleSheet("color: #ffaa00;")
        pos_l.addWidget(self.error_label, 3, 0, 1, 5)
        self.tracking_status_label = QLabel("TRACKING: OFF")
        self.tracking_status_label.setAlignment(Qt.AlignCenter)
        self.tracking_status_label.setStyleSheet(
            "background-color: #333344; color: #ccc; font-weight: bold; padding: 4px;")
        pos_l.addWidget(self.tracking_status_label, 3, 5, 1, 5)
        left.addWidget(pos_box)

        # ---- toolbar row above the sky viz ----
        toolbar = QHBoxLayout()
        self.view_mode_btn = QPushButton("View: Free")
        self.view_mode_btn.clicked.connect(self._cycle_view_mode)
        self.iss_btn = QPushButton("ISS: OFF")
        self.iss_btn.clicked.connect(self._toggle_iss)
        self.const_btn = QPushButton("Constellations: ON")
        self.const_btn.clicked.connect(self._toggle_constellations)
        self.min_size_btn = QPushButton("Min Size: OFF")
        self.min_size_btn.clicked.connect(self._toggle_min_size_filter)
        self.min_size_edit = QLineEdit("100.0")
        self.min_size_edit.setFixedWidth(50)
        self.min_size_edit.editingFinished.connect(self._on_min_size_commit)
        cam_minus = QPushButton("◄")
        cam_plus = QPushButton("►")
        cam_minus.setFixedWidth(28)
        cam_plus.setFixedWidth(28)
        cam_minus.clicked.connect(lambda: self._adjust_camera_orientation(-5.0))
        cam_plus.clicked.connect(lambda: self._adjust_camera_orientation(5.0))
        self.cam_rot_edit = QLineEdit("0.0")
        self.cam_rot_edit.setFixedWidth(50)
        self.cam_rot_edit.editingFinished.connect(self._on_cam_rot_commit)
        self.find_edit = QLineEdit()
        self.find_edit.setPlaceholderText("Find: star/DSO/Sun/Moon/ISS/planet name, Enter to target")
        self.find_edit.returnPressed.connect(self._on_find_enter)
        toolbar.addWidget(self.view_mode_btn)
        toolbar.addWidget(self.iss_btn)
        toolbar.addWidget(self.const_btn)
        toolbar.addWidget(self.min_size_btn)
        toolbar.addWidget(self.min_size_edit)
        toolbar.addWidget(QLabel("Cam Rot:"))
        toolbar.addWidget(cam_minus)
        toolbar.addWidget(self.cam_rot_edit)
        toolbar.addWidget(cam_plus)
        toolbar.addWidget(self.find_edit, stretch=1)
        left.addLayout(toolbar)

        left.addWidget(self.viz, stretch=1)
        self.hover_label = QLabel("Cursor: -")
        self.hover_label.setStyleSheet("color: #9aa; font-family: Consolas;")
        left.addWidget(self.hover_label)

        # ---- bottom bar ----
        bottom = QHBoxLayout()
        self.mode_tracking_label = QLabel("MODE: SIDEREAL  |  TRACKING: OFF")
        self.live_error_label = QLabel("Live Error: RA 0.0000°  DEC 0.0000°")
        self.live_error_label.setStyleSheet("color: #ffaa00;")
        bottom.addWidget(self.mode_tracking_label)
        bottom.addStretch(1)
        bottom.addWidget(self.live_error_label)
        left.addLayout(bottom)

        # ==================== RIGHT SIDEBAR ====================
        title = QLabel("Tracking Controls")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 15px; font-weight: bold;")
        right.addWidget(title)

        mode_box = QGroupBox("Tracking Mode")
        mode_l = QVBoxLayout(mode_box)
        self.mode_group = QButtonGroup(self)
        for i, name in enumerate(("SIDEREAL", "SOLAR", "LUNAR")):
            rb = QRadioButton(name)
            if i == 0:
                rb.setChecked(True)
            self.mode_group.addButton(rb, i)
            mode_l.addWidget(rb)
        self.mode_group.idClicked.connect(self._on_mode_changed)
        right.addWidget(mode_box)

        self.tracking_btn = QPushButton("▶ Start Tracking")
        self.tracking_btn.setStyleSheet("background-color: #006400; color: white; font-weight: bold; padding: 8px;")
        self.tracking_btn.clicked.connect(self._toggle_tracking)
        right.addWidget(self.tracking_btn)

        self.safe_target_btn = QPushButton("Safe Target (DEC to 0°)")
        self.safe_target_btn.clicked.connect(self._safe_target)
        right.addWidget(self.safe_target_btn)
        self.home_axes_btn = QPushButton("Home Axes (RA/DEC to 0°)")
        self.home_axes_btn.clicked.connect(self._home_axes)
        right.addWidget(self.home_axes_btn)

        self.flipped_label = QLabel("Telescope Flipped: OFF")
        self.flipped_label.setAlignment(Qt.AlignCenter)
        right.addWidget(self.flipped_label)

        sync_box = QGroupBox("Sync / Calibrate (choose star)")
        sync_l = QVBoxLayout(sync_box)
        self.sync_combo = QComboBox()
        self.sync_combo.addItems(list(self.cal_stars.keys()))
        self.sync_combo.setCurrentText("Vega (alpha Lyrae)")
        sync_l.addWidget(self.sync_combo)
        sync_btn = QPushButton("Sync Position (send to Arduino)")
        sync_btn.clicked.connect(self._sync)
        sync_l.addWidget(sync_btn)
        right.addWidget(sync_box)

        target_box = QGroupBox("Sidereal Target (degrees) - used by Start Tracking / Goto")
        target_l = QGridLayout(target_box)
        self.ra_edit = QLineEdit("0.0")
        self.dec_edit = QLineEdit("0.0")
        goto_btn = QPushButton("Goto")
        goto_btn.clicked.connect(self._goto)
        target_l.addWidget(QLabel("RA:"), 0, 0)
        target_l.addWidget(self.ra_edit, 0, 1)
        target_l.addWidget(QLabel("DEC:"), 1, 0)
        target_l.addWidget(self.dec_edit, 1, 1)
        target_l.addWidget(goto_btn, 2, 0, 1, 2)
        right.addWidget(target_box)

        resync_btn = QPushButton("Resync Arduino Time to Laptop")
        resync_btn.clicked.connect(lambda: self._manual_resync())
        right.addWidget(resync_btn)

        debug_box = QGroupBox("Arduino Debug (verbose var dump)")
        debug_l = QHBoxLayout(debug_box)
        self.debug_toggle_btn = QPushButton("Verbose Debug: OFF")
        self.debug_toggle_btn.clicked.connect(self._toggle_arduino_debug)
        dump_btn = QPushButton("Force Dump")
        dump_btn.clicked.connect(lambda: self.serial.request_debug_dump())
        debug_l.addWidget(self.debug_toggle_btn)
        debug_l.addWidget(dump_btn)
        right.addWidget(debug_box)

        stop_btn = QPushButton("STOP (panic)")
        stop_btn.setStyleSheet("background-color: #661111; color: white; font-weight: bold; padding: 6px;")
        stop_btn.clicked.connect(self._stop)
        right.addWidget(stop_btn)

        log_box = QGroupBox("Status / Messages")
        log_l = QVBoxLayout(log_box)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setStyleSheet("background-color: #111; color: #ccc; font-family: Consolas;")
        log_l.addWidget(self.log_edit)
        right.addWidget(log_box, stretch=1)

        self._arduino_widgets = [
            self.safe_target_btn, self.home_axes_btn, sync_btn, goto_btn,
            self.tracking_btn, self.debug_toggle_btn, dump_btn,
            ra_off_minus, ra_off_plus, dec_off_minus, dec_off_plus,
        ]

    def _log(self, msg):
        self.log_edit.append(msg)

    # ---------------- location / lat-lon ----------------
    def _get_lat_lon(self):
        try:
            return float(self.lat_edit.text()), float(self.lon_edit.text())
        except ValueError:
            return 0.0, 0.0

    def _set_location(self):
        lat, lon = self._get_lat_lon()
        self.viz.lat, self.viz.lon = lat, lon
        self.viz.update()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")
        self._log(f"Location set: LAT {lat:.4f}, LON {lon:.4f}")

    # ---------------- catalog ----------------
    def _start_worker(self, worker, *connections):
        """Registers `worker` (see _run_in_thread's docstring for why) and unregisters it again
        once it reports back either way, so self._workers doesn't grow forever across the
        repeating 2s/5s solar-system/ISS timers."""
        self._workers.append(worker)
        def _cleanup(*_a):
            try:
                self._workers.remove(worker)
            except ValueError:
                pass
        for signal, slot in connections:
            signal.connect(slot)
        worker.loaded.connect(_cleanup) if hasattr(worker, "loaded") else None
        worker.updated.connect(_cleanup) if hasattr(worker, "updated") else None
        worker.failed.connect(_cleanup)
        _run_in_thread(worker)

    def _load_catalog(self):
        self._log("Loading sky catalog...")
        worker = _CatalogWorker()
        self._start_worker(worker,
                            (worker.loaded, self._on_catalog_loaded),
                            (worker.failed, lambda err: self._log(f"Catalog load failed: {err}")))

    def _on_catalog_loaded(self, state):
        if state is None:
            self._log(f"No sky_catalog.json found at {sky_catalog.CATALOG_PATH} - "
                       f"run tracker_gui.py once to build it.")
            return
        self.viz.sky_stars_by_ra = state["sky_stars_by_ra"]
        self.viz.sky_stars_ra_values = state["sky_stars_ra_values"]
        self.viz.star_tiers = state["star_tiers"]
        self.viz.sky_dso = state["sky_dso"]
        self.viz.const_line_segments = state["const_line_segments"]
        self.viz.star_by_hip = state["star_by_hip"]
        self.viz.update()
        self._log(f"Sky catalog loaded: {state['n_stars']} stars, {state['n_dso']} DSOs.")

        # Search index for the Find box - named stars + all DSOs (by designation/name), same
        # substring-match source EQMountApp._build_sky_catalog_state builds, minus the live
        # Sun/Moon/ISS/planet entries (handled separately in _on_find_enter, since their RA/DEC
        # come from self.viz.*_ra/_dec instead of a fixed catalog value).
        index = []
        for s in self.viz.sky_stars_by_ra:
            if not s.get("name"):
                continue
            index.append((s["name"].lower(), s["name"], s["ra"], s["dec"]))
        for d in self.viz.sky_dso:
            display = f"{d['designation']} ({d['name']})" if d.get("name") else d["designation"]
            index.append((display.lower(), display, d["ra"], d["dec"]))
        self._search_index = index

    def _resolve_live_body(self, name_lower):
        if name_lower == "sun" and self.viz.sun_ra is not None:
            return self.viz.sun_ra, self.viz.sun_dec
        if name_lower == "moon" and self.viz.moon_ra is not None:
            return self.viz.moon_ra, self.viz.moon_dec
        if name_lower == "iss" and self.viz.iss_ra is not None:
            return self.viz.iss_ra, self.viz.iss_dec
        for pname, pos in self.viz.planet_positions.items():
            if pname.lower() == name_lower:
                return pos[0], pos[1]
        return None

    # ---------------- solar system / ISS ----------------
    def _trigger_solar_system_update(self):
        worker = _SolarSystemWorker(self._get_lat_lon)
        self._start_worker(worker,
                            (worker.updated, self._on_solar_system_updated),
                            (worker.failed, lambda err: self._log(f"Sun/Moon/planet update failed: {err}")))

    def _on_solar_system_updated(self, result):
        sun_ra, sun_dec, sun_diam, moon_ra, moon_dec, moon_diam, illum, waxing, planets = result
        self.viz.sun_ra, self.viz.sun_dec, self.viz.sun_diam = sun_ra, sun_dec, sun_diam
        self.viz.moon_ra, self.viz.moon_dec, self.viz.moon_diam = moon_ra, moon_dec, moon_diam
        self.viz.moon_illum, self.viz.moon_waxing = illum, waxing
        self.viz.planet_positions = planets
        self.viz.update()

    def _trigger_iss_update(self):
        worker = _IssWorker(self._get_lat_lon)
        self._start_worker(worker,
                            (worker.updated, self._on_iss_updated),
                            (worker.failed, lambda err: self._log(f"ISS update failed: {err}")))

    def _on_iss_updated(self, result):
        ra, dec, above = result
        self.viz.iss_ra, self.viz.iss_dec, self.viz.iss_above = ra, dec, above
        self.viz.update()

    # ---------------- serial connection ----------------
    def _refresh_ports(self):
        ports = self.serial.list_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports if ports else ["No ports found"])

    def _connect(self):
        port = self.port_combo.currentText()
        if not port or "No ports" in port:
            self._log("No valid port selected.")
            return
        self._log(f"Connecting to {port} ...")
        if self.serial.connect(port):
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self._set_arduino_controls_enabled(True)
            self._set_location()
            self._send_current_offsets()
            self.serial.send_pos_update_rate(POS_UPDATE_RATE_MS)

    def _disconnect(self):
        self.serial.disconnect()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self._set_arduino_controls_enabled(False)

    def _set_arduino_controls_enabled(self, enabled: bool):
        """Mirrors EQMountApp._set_arduino_controls_enabled - only controls that actually send
        something to the Arduino are gated; viz zoom/pan/follow/ISS/Constellations/search etc.
        all work fine offline."""
        for w in getattr(self, "_arduino_widgets", []):
            w.setEnabled(enabled)

    # ---------------- commands ----------------
    def _on_mode_changed(self, idx):
        mode = self.mode_group.button(idx).text()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_mode("SIDEREAL")  # wire protocol is always SIDEREAL - see tracker_gui.py's _on_mode_changed
        self._log(f"Mode changed to {mode}")
        self._update_mode_tracking_label()

    def _update_mode_tracking_label(self):
        mode = self.mode_group.checkedButton().text() if self.mode_group.checkedButton() else "SIDEREAL"
        state = "ON" if self.tracking else "OFF"
        self.mode_tracking_label.setText(f"MODE: {mode}  |  TRACKING: {state}")
        self.tracking_status_label.setText(f"TRACKING: {state}")
        self.tracking_status_label.setStyleSheet(
            ("background-color: #006400;" if self.tracking else "background-color: #333344;")
            + " color: white; font-weight: bold; padding: 4px;")

    def _cycle_view_mode(self):
        order = ["FREE", "TELESCOPE", "TARGET"]
        self.viz.view_mode = order[(order.index(self.viz.view_mode) + 1) % len(order)]
        self.view_mode_btn.setText(f"View: {self.viz.view_mode.title()}")

    def _toggle_iss(self):
        self.viz.iss_enabled = not self.viz.iss_enabled
        self.iss_btn.setText(f"ISS: {'ON' if self.viz.iss_enabled else 'OFF'}")
        self.viz.update()

    def _toggle_constellations(self):
        self.viz.constellations_enabled = not self.viz.constellations_enabled
        self.const_btn.setText(f"Constellations: {'ON' if self.viz.constellations_enabled else 'OFF'}")
        self.viz.update()

    def _toggle_min_size_filter(self):
        self.viz.min_dso_size_filter_enabled = not self.viz.min_dso_size_filter_enabled
        self.min_size_btn.setText(f"Min Size: {'ON' if self.viz.min_dso_size_filter_enabled else 'OFF'}")
        self.viz.update()

    def _on_min_size_commit(self):
        try:
            self.viz.min_dso_size_px = float(self.min_size_edit.text())
            self.viz.update()
        except ValueError:
            self._log("Invalid min DSO size value.")

    def _adjust_camera_orientation(self, delta_deg):
        self._set_camera_orientation(self.viz.camera_orientation_deg + delta_deg)

    def _on_cam_rot_commit(self):
        try:
            self._set_camera_orientation(float(self.cam_rot_edit.text()))
        except ValueError:
            self._log("Invalid camera rotation value.")
            self.cam_rot_edit.setText(f"{self.viz.camera_orientation_deg:.1f}")

    def _set_camera_orientation(self, angle_deg):
        self.viz.camera_orientation_deg = (angle_deg + 180.0) % 360.0 - 180.0
        self.cam_rot_edit.setText(f"{self.viz.camera_orientation_deg:.1f}")
        self.viz.update()

    def _on_find_enter(self):
        """Simplified version of EQMountApp's sky-search box: substring match against the index
        built in _on_catalog_loaded (or the live Sun/Moon/ISS/planets), targets the first hit -
        not the multi-result picker list the Tk version has."""
        query = self.find_edit.text().strip().lower()
        if not query:
            return
        live = self._resolve_live_body(query)
        if live is not None:
            ra, dec = live
            self._on_target_picked(ra, dec)
            self._log(f"Found: {query} -> RA {ra:.4f} DEC {dec:.4f}")
            return
        for blob, display, ra, dec in self._search_index:
            if query in blob:
                self._on_target_picked(ra, dec)
                self._log(f"Found: {display} -> RA {ra:.4f} DEC {dec:.4f}")
                return
        self._log(f"No match for '{query}'.")

    def _parse_target_inputs(self):
        try:
            return float(self.ra_edit.text()), float(self.dec_edit.text())
        except ValueError:
            self._log("Invalid RA/DEC - enter decimal degrees.")
            return None

    def _goto(self):
        parsed = self._parse_target_inputs()
        if parsed is None:
            return
        ra, dec = parsed
        self.target_ra, self.target_dec = ra, dec
        self.viz.target_ra, self.viz.target_dec = ra, dec
        self.viz.update()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_goto(ra, dec, risk_ok=True)
            self._log(f"Goto RA {ra:.4f} DEC {dec:.4f}")
        else:
            self._log("Not connected.")

    def _sync(self):
        """Mirrors EQMountApp._sync_position: resolves the chosen calibration point (named star
        via the catalog's own RA/DEC when loaded, horizon East/West via current LST, or the
        RA/DEC fields for "Custom") and sends it as the calibration position."""
        star_name = self.sync_combo.currentText()
        star_pos = self.cal_stars.get(star_name)
        if star_pos in ("EAST", "WEST"):
            lst = self.viz.current_lst_deg()
            ra = (lst + 90.0) % 360.0 if star_pos == "EAST" else (lst - 90.0) % 360.0
            dec = 0.0
        elif star_pos is not None:
            ra, dec = star_pos
            hip = self.cal_star_hip.get(star_name)
            catalog_star = self.viz.star_by_hip.get(hip) if hip else None
            if catalog_star is not None:
                ra, dec = catalog_star["ra"], catalog_star["dec"]
        else:
            parsed = self._parse_target_inputs()
            if parsed is None:
                return
            ra, dec = parsed
        self.ra_edit.setText(f"{ra:.4f}")
        self.dec_edit.setText(f"{dec:.4f}")
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_sync(ra, dec)
            self._log(f"Sync sent: RA={ra:.6f} DEC={dec:.6f} (star: {star_name})")
        else:
            self._log("Not connected.")

    def _adjust_offset(self, ra_dir, dec_dir):
        """Mirrors EQMountApp._adjust_offset - increments are sent LIVE as a CMD,SYNC_OFFSET
        delta, and also tracked cumulatively (ra_offset/dec_offset) so a fresh connection can
        resend the total via _send_current_offsets."""
        try:
            increment = float(self.offset_inc_edit.text())
        except ValueError:
            self._log("Invalid offset increment.")
            return
        ra_adj = ra_dir * increment
        dec_adj = dec_dir * increment
        self.ra_offset = round(self.ra_offset + ra_adj, 6)
        self.dec_offset = round(self.dec_offset + dec_adj, 6)
        self.ra_offset_edit.setText(f"{self.ra_offset}")
        self.dec_offset_edit.setText(f"{self.dec_offset}")
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_sync_offset(ra_adj, dec_adj)
            self._log(f"Offset adjustment: RA={ra_adj:.6f}° DEC={dec_adj:.6f}° "
                      f"(cumulative: RA={self.ra_offset:.6f}° DEC={self.dec_offset:.6f}°)")
        else:
            self._log("Offset adjustment queued (not connected).")

    def _send_current_offsets(self):
        if self.serial.ser and self.serial.ser.is_open and (abs(self.ra_offset) > 1e-4 or abs(self.dec_offset) > 1e-4):
            self.serial.send_sync_offset(self.ra_offset, self.dec_offset)
            self._log(f"Sent cumulative offsets: RA={self.ra_offset:.6f}° DEC={self.dec_offset:.6f}°")

    def _safe_target(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        self.serial.send_stop()
        self.serial.send_safe_target()
        self.tracking = False
        self._update_mode_tracking_label()
        self._log("Safe Target sent: sky DEC -> 0°, RA unchanged, no tracking.")

    def _home_axes(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        self.serial.send_stop()
        self.serial.send_home_axes()
        self.tracking = False
        self._update_mode_tracking_label()
        self._log("Home Axes sent: RA/DEC -> mount angle 0°, no tracking.")

    def _manual_resync(self):
        if self.serial.send_time_if_connected():
            self._log("Arduino time resynced to laptop.")
        else:
            self._log("Not connected - time will sync on next connect.")

    def _toggle_arduino_debug(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        self.arduino_debug_enabled = not self.arduino_debug_enabled
        self.serial.send_debug(self.arduino_debug_enabled)
        self.debug_toggle_btn.setText(f"Verbose Debug: {'ON' if self.arduino_debug_enabled else 'OFF'}")
        self._log(f"Sent CMD,DEBUG,{'ON' if self.arduino_debug_enabled else 'OFF'}")

    def _toggle_tracking(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        if not self.tracking:
            if self.serial.send_start_tracking(risk_ok=True):
                self.tracking = True
                self.tracking_btn.setText("■ Stop Tracking")
                self.tracking_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold; padding: 8px;")
        else:
            self.serial.send_stop()
            self.tracking = False
            self.tracking_btn.setText("▶ Start Tracking")
            self.tracking_btn.setStyleSheet("background-color: #006400; color: white; font-weight: bold; padding: 8px;")
        self._update_mode_tracking_label()

    def _stop(self):
        self.serial.send_stop()
        self.tracking = False
        self.tracking_btn.setText("▶ Start Tracking")
        self.tracking_btn.setStyleSheet("background-color: #006400; color: white; font-weight: bold; padding: 8px;")
        self._update_mode_tracking_label()

    def _on_target_picked(self, ra, dec):
        self.ra_edit.setText(f"{ra:.4f}")
        self.dec_edit.setText(f"{dec:.4f}")

    def _on_hover_info(self, text):
        self.hover_label.setText(text)

    @staticmethod
    def _calc_smoothed_speed(history, window_s=0.250):
        """Ported 1:1 from EQMountApp._calc_smoothed_speed - rate of change over the last
        window_s of a (value, timestamp) deque, falling back to the single last delta if the
        previous sample is older than that."""
        if len(history) < 2:
            return 0.0
        curr_ang, curr_t = history[-1]
        window_start = curr_t - window_s
        start_idx = 0
        for i, (ang, t) in enumerate(history):
            if t >= window_start:
                start_idx = i
                break
        if start_idx == len(history) - 1:
            prev_ang, prev_t = history[-2]
            dt = curr_t - prev_t
            return (curr_ang - prev_ang) / dt if dt > 0 else 0.0
        total_dangle, total_dt = 0.0, 0.0
        for i in range(start_idx, len(history) - 1):
            a0, t0 = history[i]
            a1, t1 = history[i + 1]
            total_dangle += a1 - a0
            total_dt += t1 - t0
        return total_dangle / total_dt if total_dt > 0 else 0.0

    # ---------------- serial queue polling ----------------
    def _poll_serial_queue(self):
        try:
            while True:
                msg = self.msg_queue.get_nowait()
                self._handle_message(msg)
        except queue.Empty:
            pass

    def _handle_message(self, msg: str):
        if msg.startswith("CONNECTED:"):
            port = msg.split(":", 1)[1]
            self.conn_status.setText(f"● Connected {port}")
            self.conn_status.setStyleSheet("color: #00FF88;")
            self._update_status_display("CONNECTED - WAITING FOR PING RESPONSE", "#ffaa00")
            self._log(f"Connected to {port}. Waiting for Arduino response...")
        elif msg.startswith("RX:STATUS:PONG"):
            self._update_status_display("CONNECTED - READY", "#00ff88")
            self._log("Arduino responded to PING - connection fully established")
        elif msg.startswith("DISCONNECTED"):
            self.conn_status.setText("● Disconnected")
            self.conn_status.setStyleSheet("color: gray;")
            self.tracking = False
            self._update_mode_tracking_label()
            self._update_status_display("DISCONNECTED", "#ff4444")
            self._log("Disconnected")
        elif msg.startswith("RX:"):
            self._parse_arduino_line(msg[3:])
        elif msg.startswith("TX:"):
            if "SYNC" in msg or "GOTO" in msg:
                self._log(msg)
        elif msg.startswith("ERROR:"):
            self._log("ERROR: " + msg[6:])
            self.error_label.setText("Error: " + msg[6:])

    def _update_status_display(self, text, color="#ffaa00"):
        self.status_display.setText(text)
        self.status_display.setStyleSheet(
            f"background-color: #1a1a2e; color: {color}; font-weight: bold; font-size: 14px; padding: 6px;")

    def _parse_arduino_line(self, line: str):
        line = line.strip()
        if not line:
            return
        if line.startswith("STATUS:"):
            self._log(line)
            if not line.startswith("STATUS:WAITING"):
                self._update_status_display(line[len("STATUS:"):])
            return
        if line.startswith("POS,"):
            # POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking,flipped - see
            # the matching comment above sendPositionUpdate() in the .ino / tracker_gui.py's
            # _parse_arduino_line for the full field-by-field breakdown.
            fields = line.split(",")
            try:
                self.current_ra = float(fields[1])
                self.current_dec = float(fields[2])
                self.target_ra = float(fields[3])
                self.target_dec = float(fields[4])
                self.mount_ra_angle = float(fields[5])
                self.mount_dec_angle = float(fields[6])
                self.err_ra = abs(self.current_ra - self.target_ra)
                self.err_dec = abs(self.current_dec - self.target_dec)

                now = time.time()
                cutoff = now - 1.0
                self.ra_pos_history.append((self.mount_ra_angle, now))
                while self.ra_pos_history and self.ra_pos_history[0][1] < cutoff:
                    self.ra_pos_history.popleft()
                self.speed_ra = self._calc_smoothed_speed(self.ra_pos_history)

                self.dec_pos_history.append((self.mount_dec_angle, now))
                while self.dec_pos_history and self.dec_pos_history[0][1] < cutoff:
                    self.dec_pos_history.popleft()
                self.speed_dec = self._calc_smoothed_speed(self.dec_pos_history)
            except (ValueError, TypeError, IndexError):
                return
            if len(fields) > 8:
                pos_track = fields[8].strip() == "1"
                if pos_track and not self.tracking:
                    self.tracking = True
                    self._update_mode_tracking_label()
            if len(fields) > 9:
                try:
                    flipped = fields[9].strip() == "1"
                    self.viz.telescope_flipped = flipped
                    self.flipped_label.setText(f"Telescope Flipped: {'ON' if flipped else 'OFF'}")
                except (ValueError, IndexError):
                    pass

            self.viz.current_ra, self.viz.current_dec = self.current_ra, self.current_dec
            self.viz.target_ra, self.viz.target_dec = self.target_ra, self.target_dec
            # Simplified stability color vs EQMountApp's windowed-derivative STABLE/SETTLING/
            # DRIFTING classification (see TRACKING_STABLE_ERR_DEG/_DERIV_DEG_S/_WINDOW_S there) -
            # just a live error-magnitude threshold here.
            if not self.tracking:
                self.viz.stability_color = QColor("#888888")
            elif max(self.err_ra, self.err_dec) <= TRACKING_STABLE_ERR_DEG:
                self.viz.stability_color = QColor("#33ff88")
            else:
                self.viz.stability_color = QColor("#ffaa00")

            if self.viz.view_mode == "TELESCOPE":
                self.viz.center_ra, self.viz.center_dec = self.current_ra % 360.0, self.current_dec
                self.viz.clamp_center()
            elif self.viz.view_mode == "TARGET":
                self.viz.center_ra, self.viz.center_dec = self.target_ra % 360.0, self.target_dec
                self.viz.clamp_center()

            now = time.time()
            if now - self._last_pos_ui_update >= POS_UI_UPDATE_MIN_INTERVAL_S:
                self._last_pos_ui_update = now
                self.sky_ra_label.setText(f"{self.current_ra:07.4f}°")
                self.sky_dec_label.setText(f"{self.current_dec:+08.4f}°")
                self.mount_ra_label.setText(f"{self.mount_ra_angle:.4f}°")
                self.mount_dec_label.setText(f"{self.mount_dec_angle:.4f}°")
                self.speed_ra_label.setText(f"{self.speed_ra:.6f} °/s")
                self.speed_dec_label.setText(f"{self.speed_dec:.6f} °/s")
                self.error_label.setText(f"Error: RA {self.err_ra:.4f}°  DEC {self.err_dec:.4f}°")
                self.live_error_label.setText(f"Live Error: RA {self.err_ra:.4f}°  DEC {self.err_dec:.4f}°")
                self._update_mode_tracking_label()
                self.viz.update()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

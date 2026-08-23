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
from datetime import datetime, timezone
from typing import Optional

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, Signal, QObject
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, QPainterPath, QFontMetrics,
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QComboBox, QLabel, QLineEdit, QTextEdit, QRadioButton, QButtonGroup,
    QGroupBox, QSizePolicy, QMessageBox,
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
    STAR_LOD_TIER_MAG_CUTOFFS, _galactic_to_radec,
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
        if self.iss_ra is not None and ra_min <= self.iss_ra <= ra_max and dec_min <= self.iss_dec <= dec_max:
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
        self.resize(1280, 800)

        self.msg_queue = queue.Queue()
        self.serial = SerialHandler(self.msg_queue, get_time_fn=lambda: datetime.now(timezone.utc))

        self.current_ra = self.current_dec = 0.0
        self.target_ra = self.target_dec = 0.0
        self.tracking = False
        self._last_pos_ui_update = 0.0
        self._workers = []  # keeps background QObjects referenced while their thread runs

        self.viz = SkyViewWidget()
        self.viz.targetPicked.connect(self._on_target_picked)
        self.viz.hoverInfoChanged.connect(self._on_hover_info)

        self._build_ui()

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
        layout = QHBoxLayout(central)
        self.setCentralWidget(central)

        left = QVBoxLayout()
        left.addWidget(self.viz, stretch=1)
        self.hover_label = QLabel("Cursor: -")
        self.hover_label.setStyleSheet("color: #9aa; font-family: Consolas;")
        left.addWidget(self.hover_label)
        layout.addLayout(left, stretch=3)

        right = QVBoxLayout()
        right.setSpacing(8)
        layout.addLayout(right, stretch=1)

        # Connection
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
        conn_l.addWidget(self.conn_status, 2, 0, 1, 3)
        right.addWidget(conn_box)

        # Location
        loc_box = QGroupBox("Location")
        loc_l = QGridLayout(loc_box)
        self.lat_edit = QLineEdit("0.0")
        self.lon_edit = QLineEdit("0.0")
        set_loc_btn = QPushButton("Set Location")
        set_loc_btn.clicked.connect(self._set_location)
        loc_l.addWidget(QLabel("Lat:"), 0, 0)
        loc_l.addWidget(self.lat_edit, 0, 1)
        loc_l.addWidget(QLabel("Lon:"), 1, 0)
        loc_l.addWidget(self.lon_edit, 1, 1)
        loc_l.addWidget(set_loc_btn, 2, 0, 1, 2)
        right.addWidget(loc_box)

        # Mode
        mode_box = QGroupBox("Mode")
        mode_l = QHBoxLayout(mode_box)
        self.mode_group = QButtonGroup(self)
        for i, name in enumerate(("SIDEREAL", "SOLAR", "LUNAR")):
            rb = QRadioButton(name)
            if i == 0:
                rb.setChecked(True)
            self.mode_group.addButton(rb, i)
            mode_l.addWidget(rb)
        self.mode_group.idClicked.connect(self._on_mode_changed)
        right.addWidget(mode_box)

        # Target / Goto / Sync
        target_box = QGroupBox("Target (decimal degrees)")
        target_l = QGridLayout(target_box)
        self.ra_edit = QLineEdit("0.0")
        self.dec_edit = QLineEdit("0.0")
        goto_btn = QPushButton("Goto")
        goto_btn.clicked.connect(self._goto)
        sync_btn = QPushButton("Sync (calibrate here)")
        sync_btn.clicked.connect(self._sync)
        target_l.addWidget(QLabel("RA:"), 0, 0)
        target_l.addWidget(self.ra_edit, 0, 1)
        target_l.addWidget(QLabel("DEC:"), 1, 0)
        target_l.addWidget(self.dec_edit, 1, 1)
        target_l.addWidget(goto_btn, 2, 0)
        target_l.addWidget(sync_btn, 2, 1)
        right.addWidget(target_box)

        # Tracking
        track_box = QGroupBox("Tracking")
        track_l = QHBoxLayout(track_box)
        self.tracking_btn = QPushButton("Start Tracking")
        self.tracking_btn.clicked.connect(self._toggle_tracking)
        stop_btn = QPushButton("STOP (panic)")
        stop_btn.setStyleSheet("background-color: #661111; color: white; font-weight: bold;")
        stop_btn.clicked.connect(self._stop)
        track_l.addWidget(self.tracking_btn)
        track_l.addWidget(stop_btn)
        right.addWidget(track_box)

        # View follow mode
        view_box = QGroupBox("Sky Viz Follow")
        view_l = QHBoxLayout(view_box)
        self.view_mode_group = QButtonGroup(self)
        for i, name in enumerate(("Free", "Telescope", "Target")):
            rb = QRadioButton(name)
            if i == 0:
                rb.setChecked(True)
            self.view_mode_group.addButton(rb, i)
            view_l.addWidget(rb)
        self.view_mode_group.idClicked.connect(self._on_view_mode_changed)
        right.addWidget(view_box)

        # Log
        log_box = QGroupBox("Log")
        log_l = QVBoxLayout(log_box)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setStyleSheet("background-color: #111; color: #ccc; font-family: Consolas;")
        log_l.addWidget(self.log_edit)
        right.addWidget(log_box, stretch=1)

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

    def _disconnect(self):
        self.serial.disconnect()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    # ---------------- commands ----------------
    def _on_mode_changed(self, idx):
        mode = self.mode_group.button(idx).text()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_mode("SIDEREAL")  # wire protocol is always SIDEREAL - see tracker_gui.py's _on_mode_changed
        self._log(f"Mode changed to {mode}")

    def _on_view_mode_changed(self, idx):
        self.viz.view_mode = self.view_mode_group.button(idx).text().upper()

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
        parsed = self._parse_target_inputs()
        if parsed is None:
            return
        ra, dec = parsed
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_sync(ra, dec)
            self._log(f"Sync at RA {ra:.4f} DEC {dec:.4f}")
        else:
            self._log("Not connected.")

    def _toggle_tracking(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        if not self.tracking:
            if self.serial.send_start_tracking(risk_ok=True):
                self.tracking = True
                self.tracking_btn.setText("Stop Tracking")
        else:
            self.serial.send_stop()
            self.tracking = False
            self.tracking_btn.setText("Start Tracking")

    def _stop(self):
        self.serial.send_stop()
        self.tracking = False
        self.tracking_btn.setText("Start Tracking")

    def _on_target_picked(self, ra, dec):
        self.ra_edit.setText(f"{ra:.4f}")
        self.dec_edit.setText(f"{dec:.4f}")

    def _on_hover_info(self, text):
        self.hover_label.setText(text)

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
            self._log(f"Connected to {port}.")
            lat, lon = self._get_lat_lon()
            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")
        elif msg.startswith("DISCONNECTED"):
            self.conn_status.setText("● Disconnected")
            self.conn_status.setStyleSheet("color: gray;")
            self.tracking = False
            self.tracking_btn.setText("Start Tracking")
            self._log("Disconnected")
        elif msg.startswith("RX:"):
            self._parse_arduino_line(msg[3:])
        elif msg.startswith("TX:"):
            if "SYNC" in msg or "GOTO" in msg:
                self._log(msg)
        elif msg.startswith("ERROR:"):
            self._log("ERROR: " + msg[6:])

    def _parse_arduino_line(self, line: str):
        line = line.strip()
        if not line:
            return
        if line.startswith("STATUS:"):
            self._log(line)
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
            except (ValueError, IndexError):
                return
            if len(fields) > 8:
                self.tracking = fields[8].strip() == "1" or self.tracking
                self.tracking_btn.setText("Stop Tracking" if self.tracking else "Start Tracking")
            if len(fields) > 9:
                try:
                    self.viz.telescope_flipped = fields[9].strip() == "1"
                except (ValueError, IndexError):
                    pass
            self.viz.current_ra, self.viz.current_dec = self.current_ra, self.current_dec
            self.viz.target_ra, self.viz.target_dec = self.target_ra, self.target_dec
            if self.viz.view_mode == "TELESCOPE":
                self.viz.center_ra, self.viz.center_dec = self.current_ra % 360.0, self.current_dec
                self.viz.clamp_center()
            elif self.viz.view_mode == "TARGET":
                self.viz.center_ra, self.viz.center_dec = self.target_ra % 360.0, self.target_dec
                self.viz.clamp_center()
            now = time.time()
            if now - self._last_pos_ui_update >= POS_UI_UPDATE_MIN_INTERVAL_S:
                self._last_pos_ui_update = now
                self.viz.update()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

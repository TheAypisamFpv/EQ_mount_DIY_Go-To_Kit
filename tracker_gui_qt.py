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
import re
import math
import time
import json
import bisect
import threading
import queue
from collections import deque
from datetime import datetime, timezone, timedelta
from typing import Optional

from PySide6.QtCore import Qt, QTimer, QThread, QPointF, QRectF, Signal, QObject, QEvent
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, QPainterPath, QFontMetrics,
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QComboBox, QLabel, QLineEdit, QTextEdit, QRadioButton, QButtonGroup,
    QGroupBox, QCheckBox, QListWidget, QMessageBox, QFrame,
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
    TRACKING_STABLE_DERIV_DEG_S, TRACKING_STABILITY_WINDOW_S,
    POS_UPDATE_RATE_MS, POSITION_BROADCAST_HZ, DEFAULT_LAT, DEFAULT_LON,
    SIDEREAL_RATE_DEG_S, MERIDIAN_LIMIT_WARNING_S, MERIDIAN_LIMIT_URGENT_S,
    CONFIG_PATH, ISS_TRACKING_UPDATE_MS, ISS_DISPLAY_UPDATE_MS,
)
from sky_data import sky_catalog, iss_tracker, solar_system


# ============================================================
# OKLCH COLOR PALETTE
# ============================================================
# Every non-gray/near-gray accent color in this GUI (buttons, badges, status text, the target
# reticle, camera FOV overlay, DSO/planet/ISS markers, ...) is generated from ONE shared
# (lightness, chroma) pair in OKLCH space - only the hue differs per semantic meaning. Qt's
# stylesheet engine has no native oklch() support, so these are converted to sRGB hex once at
# import time (see _oklch_to_hex) and used as plain hex strings everywhere below, same as any
# other color literal. Backgrounds, panel borders, muted gray-blue label text, and the star
# field's own catalog-sourced B-V colors are intentionally NOT part of this palette (they're
# gray/near-gray, or - for star colors - real astronomical data, not a decorative UI choice).
_PALETTE_L = 0.78   # lightness - fixed for every hue below
_PALETTE_C = 0.10   # chroma - fixed for every hue below; this (not a saturated ~0.2+) is what
                    # makes every accent color read as muted/pastel rather than neon


def _oklch_to_hex(L, C, h_deg):
    """OKLCH -> sRGB hex, via Bjorn Ottosson's OKLab (https://bottosson.github.io/posts/oklab/)."""
    h = math.radians(h_deg)
    a, b = C * math.cos(h), C * math.sin(h)
    l_ = L + 0.3963377774 * a + 0.2158037573 * b
    m_ = L - 0.1055613458 * a - 0.0638541728 * b
    s_ = L - 0.0894841775 * a - 1.2914855480 * b
    l, m, s = l_ ** 3, m_ ** 3, s_ ** 3
    r = 4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s
    g = -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s
    bl = -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s

    def _encode(c):
        c = max(0.0, min(1.0, c))
        c = 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1 / 2.4)) - 0.055
        return max(0, min(255, round(c * 255)))
    return "#%02x%02x%02x" % (_encode(r), _encode(g), _encode(bl))


# Hues spread evenly around the OKLCH wheel, one per semantic role - see each usage site for
# what it's used for (tracking-stable badge, target reticle, DSO type legend, planet markers...).
_PALETTE_HUES = {
    "red": 15, "salmon": 35, "orange": 55, "sun": 75, "yellow": 95,
    "green": 145, "teal": 175, "cyan": 205, "blue": 255, "purple": 300, "pink": 330,
}
PALETTE = {name: _oklch_to_hex(_PALETTE_L, _PALETTE_C, hue) for name, hue in _PALETTE_HUES.items()}

# One shared, deliberately lower-chroma red (same hue as PALETTE["red"], still generated via
# OKLCH, not a hand-picked hex) for BOTH stop-related controls - the ISO 13850 emergency-stop
# actuator and the Start/Stop Tracking toggle's "Stop" state - so the two read as clearly related
# rather than two different reds. Toned down from a fully-saturated safety red, but still well
# above PALETTE's own chroma.
STOP_RED = _oklch_to_hex(0.55, 0.16, _PALETTE_HUES["red"])
STOP_RED_HOVER = _oklch_to_hex(0.60, 0.16, _PALETTE_HUES["red"])
STOP_RED_PRESSED = _oklch_to_hex(0.45, 0.16, _PALETTE_HUES["red"])
TRACKING_STOP_RED = STOP_RED


# ============================================================
# PURE GEOMETRY/STYLE HELPERS
# ============================================================
# Small, stateless helpers ported directly from the matching EQMountApp methods in tracker_gui.py
# (same formulas, same East-is-leftward/North-is-up screen convention) - kept as free functions
# here since SkyViewWidget isn't an EQMountApp subclass.

def _dso_render_style(type_code):
    """See EQMountApp._dso_render_style's docstring for the color/shape reasoning - same
    per-type mapping, but every color comes from PALETTE (see its module docstring) instead of
    its own hand-picked hex, so the DSO-type legend reads as one consistent color system rather
    than a grab-bag of independently chosen saturations. DrkN (dark nebula) is the one exception
    left as its own muted brownish-gray literal - it's meant to read as dust/near-gray, not a
    vivid accent, so it's exempt from the uniform-chroma palette the same way backgrounds are."""
    t = type_code or ""
    if t in ("OCl", "GCl"):
        return "cluster", QColor(PALETTE["yellow"])
    if t == "PN":
        return "ring", QColor(PALETTE["purple"])
    if t.startswith("G") and t not in ("GCl",):
        return "ellipse", QColor(PALETTE["salmon"])
    if t in ("HII", "EmN"):
        return "ellipse", QColor(PALETTE["red"])
    if t == "RfN":
        return "ellipse", QColor(PALETTE["blue"])
    if t == "DrkN":
        return "ellipse", QColor("#998877")
    if t == "SNR":
        return "ellipse", QColor(PALETTE["orange"])
    return "ellipse", QColor(PALETTE["teal"])


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
    # translation is needed.
    # Qt's docs say arcTo() on an empty path implicitly moveTo()s the arc's own start point -
    # it doesn't, at least on this PySide6 version: it instead inserts a MoveTo(0, 0) (the
    # path's default origin) followed by a LineTo the arc's start, which drew a long stray wedge
    # from the top-left of the canvas to the Moon/planet disc. Explicit moveTo first avoids it.
    start_rad = math.radians(bearing_deg)
    path_lit = QPainterPath()
    path_lit.moveTo(x + r * math.cos(start_rad), y - r * math.sin(start_rad))
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

    targetPicked = Signal(float, float, object)   # emitted (ra_deg, dec_deg, name_or_None) on double-click
    hoverInfoChanged = Signal(str)        # emitted with a "RA ... DEC ... | name (extra)" string
    viewModeChanged = Signal(str)         # emitted whenever view_mode changes FROM WITHIN this
                                           # widget (drag/middle-click) - see _set_view_mode -
                                           # so MainWindow's view_mode_btn label/color can follow

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
        # "Now" source for LST - overridden by MainWindow to its _get_effective_utc_now (Time
        # Travel-aware) once the window exists; defaults to real time so this widget also works
        # standalone/under test.
        self.get_time_fn = lambda: datetime.now(timezone.utc)

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

    def _set_view_mode(self, mode):
        """Sets view_mode and emits viewModeChanged if it actually changed - used (rather than
        assigning self.view_mode directly) anywhere INSIDE this widget that drops out of
        follow mode (drag-pan, middle-click reset), so MainWindow's view_mode_btn label/color
        stays in sync instead of still showing "Telescope"/"Target" after the viz itself has
        already switched back to Free."""
        if mode != self.view_mode:
            self.view_mode = mode
            self.viewModeChanged.emit(mode)

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
        utc = self.get_time_fn()
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
        self._draw_hover(p)
        p.end()

    def _draw_hover(self, p):
        """White ring around whatever mouseMoveEvent last hit-tested (see self._hovered) - the
        object a double-click would target right now. Ported from EQMountApp._on_viz_mouse_move:
        the ring always shows for whatever's hovered; the floating name text next to it is
        skipped only for objects that already have a permanent on-canvas label (Sun/Moon/ISS
        always do; DSOs do once zoomed in enough - see the "labeled" flag in _draw_sky_objects/
        _draw_realtime_bodies), since that would just be a redundant second copy of the name.
        Drawn last (on top of everything else, including the camera/target overlay) so the ring
        is never hidden by anything it happens to sit near."""
        hit = self._hovered
        if hit is None:
            return
        # hit["radius"] already has its own per-type minimum baked in (stars/DSOs/Sun/Moon/
        # planets/ISS all clamp to at least 12px - see _draw_sky_objects/_draw_realtime_bodies),
        # so this keeps that same floor while growing 10% past the object's own real on-screen
        # size once it's bigger than that - e.g. a large DSO's ring visibly circumscribes it
        # with a bit of clearance, rather than the ring being a fixed size regardless of what's
        # actually under it.
        hx, hy, r = hit["x"], hit["y"], hit["radius"] * 1.1
        p.setPen(QPen(QColor("#ffffff"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(hx, hy), r, r)
        if not hit.get("labeled"):
            _draw_centered_text(p, hx + r + 4, hy, hit["name"], self._font_med_bold, QColor("#ffffff"), anchor="w")

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
            iss_color = QColor(PALETTE["green"] if self.iss_above else "#336655")
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
            p.setPen(QPen(QColor(_darken_hex_color(PALETTE["sun"])), 1))
            p.setBrush(QBrush(QColor(PALETTE["sun"])))
            p.drawEllipse(QPointF(sx, sy), sr, sr)
            _draw_centered_text(p, sx, sy - sr - 8, "Sun", self._font_small_bold, QColor(PALETTE["sun"]))
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

        # Mercury stays its own neutral gray (a near-gray "no strong color" is accurate for it,
        # and exempt from the uniform-chroma palette the same way any other gray is) - the rest
        # each get a distinct PALETTE hue so the planets stay visually distinguishable from each
        # other and from the DSO-type legend colors, all at the same muted lightness/chroma.
        planet_colors = {
            "Mercury": "#aaaaaa", "Venus": PALETTE["yellow"], "Mars": PALETTE["red"],
            "Jupiter": PALETTE["sun"], "Saturn": PALETTE["orange"], "Uranus": PALETTE["cyan"],
            "Neptune": PALETTE["blue"],
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
        pen = _dashed_pen(QColor(PALETTE["yellow"]))
        for label, cra in cardinal_ra.items():
            if ra_min <= cra <= ra_max:
                line_x = self.ra_to_x(cra, w, margin, ra_min, ra_span)
                p.setPen(pen)
                p.drawLine(QPointF(line_x, 8), QPointF(line_x, h - 8))
                _draw_centered_text(p, line_x, 20, label, self._font_label_bold, QColor(PALETTE["yellow"]))

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

        p.setPen(_dashed_pen(QColor(PALETTE["blue"]), 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(_camera_rect_points(x, y, cam_half_w, cam_half_h, self.camera_orientation_deg))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(PALETTE["blue"])))
        p.drawPolygon(_camera_top_marker_points(x, y, cam_half_w, cam_half_h, self.camera_orientation_deg))

        fov_radius = max(5.0, math.hypot(cam_half_w, cam_half_h))
        p.setPen(QPen(self.stability_color, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(x, y), fov_radius, fov_radius)

        center_ref_radius = 8 + 2 + 6
        for hit in self._visible_hits:
            if hit["name"] == self.target_object_name:
                center_ref_radius = max(center_ref_radius, hit["radius"])
                break
        center_ref_radius *= 1.1
        p.setPen(_dashed_pen(self.stability_color))
        p.drawEllipse(QPointF(x, y), center_ref_radius, center_ref_radius)

        tx = self.ra_to_x(self.target_ra % 360.0, w, margin, ra_min, ra_span)
        ty = self.dec_to_y(self.target_dec, h, margin, dec_min, dec_span)
        radius, gap, tick_len = 8, 2, 6
        p.setPen(QPen(QColor(PALETTE["orange"]), 1.5))
        p.drawEllipse(QPointF(tx, ty), radius, radius)
        p.drawLine(QPointF(tx, ty - radius - gap), QPointF(tx, ty - radius - gap - tick_len))
        p.drawLine(QPointF(tx, ty + radius + gap), QPointF(tx, ty + radius + gap + tick_len))
        p.drawLine(QPointF(tx + radius + gap, ty), QPointF(tx + radius + gap + tick_len, ty))
        p.drawLine(QPointF(tx - radius - gap, ty), QPointF(tx - radius - gap - tick_len, ty))

    # ---------------- mouse/wheel interaction ----------------
    def wheelEvent(self, event):
        """Zoom centered on the cursor - same math as EQMountApp._on_viz_zoom - EXCEPT while
        following the telescope or target (view_mode != FREE): the center stays anchored on
        whatever's being followed (kept fresh every POS update by MainWindow) instead of
        re-centering on the cursor and then needing a correction snap back to the followed
        point on the very next update - reported as a visible jump when zooming while in
        View: Telescope/Target."""
        steps = event.angleDelta().y() / 120.0
        zoom_factor = VIZ_ZOOM_STEP_BASE ** steps
        new_zoom = max(VIZ_ZOOM_MIN, min(VIZ_ZOOM_MAX, self.zoom * zoom_factor))
        if new_zoom == self.zoom:
            return

        if self.view_mode != "FREE":
            self.zoom = new_zoom
            self.clamp_center()
            self.update()
            return

        w, h = max(200, self.width()), max(150, self.height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        pos = event.position()
        frac_x = (pos.x() - margin) / max(1, (w - 2 * margin))
        frac_y = (pos.y() - margin) / max(1, (h - 2 * margin))
        cursor_ra = self.x_to_ra(pos.x(), w, margin, ra_min, ra_span)
        cursor_dec = dec_min + dec_span * (1.0 - frac_y)

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
            self._set_view_mode("FREE")
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
            self._set_view_mode("FREE")
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
        """Double-click: target whatever's hovered (precise catalog RA/DEC AND its name - see
        _hovered, built by the same hit-test mouseMoveEvent uses for the hover ring), or the raw
        cursor sky position (no name) otherwise - same behavior as EQMountApp._on_viz_double_click.
        The name matters here (not just RA/DEC) so the target-name badge shows the right thing
        for ANY double-clicked object, not just ones picked via the Find box - previously this
        signal only carried ra/dec, so double-clicking a star/DSO/planet/Sun/Moon in the viz
        itself never populated the name, regardless of what was actually clicked."""
        w, h = max(200, self.width()), max(150, self.height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self.view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        if self._hovered is not None:
            ra, dec, name = self._hovered["ra"], self._hovered["dec"], self._hovered["name"]
        else:
            pos = event.position()
            ra = self.x_to_ra(pos.x(), w, margin, ra_min, ra_span) % 360.0
            dec = max(-90.0, min(90.0, self.y_to_dec(pos.y(), h, margin, dec_min, dec_span)))
            name = None
        self.targetPicked.emit(ra, dec, name)

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

    def __init__(self, get_lat_lon_fn, at_time=None):
        super().__init__()
        self.get_lat_lon_fn = get_lat_lon_fn
        self.at_time = at_time  # Time Travel-aware "now" - see MainWindow._get_effective_utc_now

    def run(self):
        try:
            lat, lon = self.get_lat_lon_fn()
            sun_ra, sun_dec, sun_diam = solar_system.get_sun_info(lat, lon, at_time=self.at_time)
            moon_ra, moon_dec, moon_diam, illum, phase, waxing = solar_system.get_moon_info(
                lat, lon, at_time=self.at_time)
            planets = solar_system.get_planets_info(lat, lon, at_time=self.at_time)
        except Exception as e:
            self.failed.emit(str(e))
            return
        self.updated.emit((sun_ra, sun_dec, sun_diam, moon_ra, moon_dec, moon_diam,
                            illum, waxing, planets))


class _IssTrackingThread(QThread):
    """ONE persistent background thread for ISS position updates, for the thread's entire
    lifetime - NOT a fresh worker+thread spawned on every tick the way this used to work (up to
    50/sec, at ISS_TRACKING_UPDATE_MS, while the ISS was the actively tracked target). That
    turned out to be enough thread-creation/GIL-handoff churn to visibly stutter the GUI's own
    rendering (reported as "the mount is fine, but the viz and axis angles stutter" while
    tracking the ISS) - the mount itself was never affected, since its motion comes from the
    firmware's own extrapolation between whatever SET_TARGET updates it gets, not from the GUI's
    render loop. Loops internally instead, sleeping between fetches via a threading.Event (so an
    external wake() call can cut a wait short for an immediate refresh, e.g. after a Time Travel
    jump) rather than blocking in one-shot worker runs."""
    updated = Signal(object)
    failed = Signal(str)

    def __init__(self, get_lat_lon_fn, get_time_fn, state_fn, parent=None):
        super().__init__(parent)
        self.get_lat_lon_fn = get_lat_lon_fn
        self.get_time_fn = get_time_fn
        self.state_fn = state_fn  # () -> (enabled: bool, interval_ms: int)
        self._stop_requested = False
        self._wake = threading.Event()

    def run(self):
        while not self._stop_requested:
            enabled, interval_ms = self.state_fn()
            if enabled:
                try:
                    iss_tracker.ensure_tle_current()
                    lat, lon = self.get_lat_lon_fn()
                    at_time = self.get_time_fn()
                    ra, dec, alt, az, above = iss_tracker.get_current_radec(lat, lon, at_time=at_time)
                    self.updated.emit((ra, dec, above))
                except Exception as e:
                    self.failed.emit(str(e))
            else:
                interval_ms = max(interval_ms, 1000)  # idle poll - no fetch needed until re-enabled
            self._wake.wait(timeout=max(0.001, interval_ms / 1000.0))
            self._wake.clear()

    def wake(self):
        """Cuts the current sleep short so the next fetch happens right away, instead of
        waiting out whatever interval was already in progress."""
        self._wake.set()

    def stop(self):
        self._stop_requested = True
        self._wake.set()


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

        # Time Travel - THE single source of "now" for this whole app, exactly like
        # EQMountApp._get_effective_utc_now/_time_travel_offset: real UTC time shifted by this
        # offset (zero = real time). Set before self.serial so its get_time_fn can close over
        # self._get_effective_utc_now from the start.
        self._time_travel_offset = timedelta(0)

        self.msg_queue = queue.Queue()
        self.serial = SerialHandler(self.msg_queue, get_time_fn=self._get_effective_utc_now)

        self.current_ra = self.current_dec = 0.0
        self.target_ra = self.target_dec = 0.0
        self.mount_ra_angle = self.mount_dec_angle = 0.0
        self.speed_ra = self.speed_dec = 0.0
        self.err_ra = self.err_dec = 0.0
        self.ra_pos_history = deque()
        self.dec_pos_history = deque()
        # Error-trend history for the STABLE/SETTLING/DRIFTING classification - see
        # _update_tracking_stability (ported from EQMountApp._update_tracking_stability).
        self.err_ra_history = deque()
        self.err_dec_history = deque()
        self.ra_offset = 0.0
        self.dec_offset = 0.0
        self.tracking = False
        self.slewing = False
        self.arduino_debug_enabled = False
        self.last_status = ""
        self._last_pos_ui_update = 0.0
        self._workers = []  # keeps background QObjects referenced while their thread runs
        self._search_index = []  # built alongside the catalog - see _on_catalog_loaded
        self._log_history = []  # (msg, masked_msg) pairs - see _log/_rebuild_log_display
        self._sky_search_matches = []
        self.target_object_name = None  # see _select_sky_target
        self._target_name_box_ra_str = ""
        self._target_name_box_dec_str = ""
        self._force_full_align_next_start = True
        self._align_phase = "IDLE"  # IDLE / ALIGNING / TRACKING - drives the stability badge
        self.gps_mode = False
        self.streamer_mode = False

        # START/STOP retry-until-confirmed state - see _request_tracking_action.
        self._pending_action = None
        self._pending_action_confirmed = False
        self._pending_action_retry_count = 0
        self._pending_action_timer = QTimer(self)
        self._pending_action_timer.setSingleShot(True)

        self._connection_timeout_timer = QTimer(self)
        self._connection_timeout_timer.setSingleShot(True)
        self._connection_timeout_timer.timeout.connect(self._handle_connection_timeout)

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
        self.viz.get_time_fn = self._get_effective_utc_now
        self.viz.targetPicked.connect(
            lambda ra, dec, name: self._select_sky_target(ra, dec, "Viz double-click", target_name=name))
        self.viz.hoverInfoChanged.connect(self._on_hover_info)
        self.viz.viewModeChanged.connect(self._on_viz_view_mode_changed)

        self._build_ui()
        self._set_arduino_controls_enabled(False)
        self._reset_time_travel_inputs_to_now()

        # Same gui_config.json tracker_gui.py reads/writes (CONFIG_PATH, gitignored) - NOT a
        # separate config file for this GUI, so whichever one you last set your location in is
        # what the other picks up too.
        self._location_save_timer = QTimer(self)
        self._location_save_timer.setSingleShot(True)
        self._location_save_timer.timeout.connect(self._on_location_changed)
        self._load_gui_config()
        self.lat_edit.textChanged.connect(lambda _: self._location_save_timer.start(200))
        self.lon_edit.textChanged.connect(lambda _: self._location_save_timer.start(200))
        self.gps_edit.textChanged.connect(lambda _: self._location_save_timer.start(200))

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_serial_queue)
        self.poll_timer.start(POLL_INTERVAL_MS)

        self.resync_timer = QTimer(self)
        self.resync_timer.timeout.connect(lambda: self.serial.send_time_if_connected())
        self.resync_timer.start(60000)

        self.solar_timer = QTimer(self)
        self.solar_timer.timeout.connect(self._trigger_solar_system_update)
        self.solar_timer.start(2000)

        # One persistent thread for the whole app lifetime - see _IssTrackingThread's docstring
        # for why this isn't a repeating timer spawning a fresh worker+thread every tick.
        self.iss_thread = _IssTrackingThread(self._get_lat_lon, self._get_effective_utc_now,
                                             self._iss_thread_state)
        self.iss_thread.updated.connect(self._on_iss_updated)
        self.iss_thread.failed.connect(lambda err: self._log(f"ISS update failed: {err}"))
        self.iss_thread.start()

        self.meridian_timer = QTimer(self)
        self.meridian_timer.timeout.connect(self._update_meridian_warning)
        self.meridian_timer.start(500)

        # Self-rescheduling (see _update_realtime_dot's identical reasoning), not a fixed 1000ms
        # repeating timer - that could show a stale HH:MM:SS for up to a full second after the
        # real second actually changed (reported as looking "about half a second out of sync"),
        # since a timer started at arbitrary app-launch time has no reason to land on real
        # second boundaries. _update_time_travel_label now reschedules itself precisely instead.
        self.time_travel_label_timer = QTimer(self)
        self.time_travel_label_timer.setSingleShot(True)
        self.time_travel_label_timer.setTimerType(Qt.PreciseTimer)
        self.time_travel_label_timer.timeout.connect(self._update_time_travel_label)
        self._update_time_travel_label()

        # Self-rescheduling (see _update_realtime_dot) rather than a fixed-interval repeating
        # timer - a 200ms poll could be showing a stale state for up to 200ms after an actual
        # second/half-second boundary passed, which read as "not quite in sync with the computer
        # clock". Scheduling each update to land AT the next boundary keeps the visible lag down
        # to whatever the OS's own timer/scheduler jitter is (typically low single-digit ms).
        self.blink_timer = QTimer(self)
        self.blink_timer.setSingleShot(True)
        self.blink_timer.setTimerType(Qt.PreciseTimer)
        self.blink_timer.timeout.connect(self._update_realtime_dot)
        self._update_realtime_dot()

        self._load_catalog()
        self._refresh_ports()
        QTimer.singleShot(500, self._trigger_solar_system_update)

    def closeEvent(self, event):
        self.iss_thread.stop()
        self.iss_thread.wait(2000)
        super().closeEvent(event)

    def _get_effective_utc_now(self) -> datetime:
        """THE single source of "now" - see _time_travel_offset's comment. Every position lookup
        (LST/stars/DSOs via viz.get_time_fn, Sun/Moon/planets, ISS, and the Arduino's own clock
        via SerialHandler.get_time_fn) reads from this one function."""
        return datetime.now(timezone.utc) + self._time_travel_offset

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
        self._flat_btn(refresh_btn, "cyan")
        # Single Connect<->Disconnect toggle (like tracking_btn), not two separate buttons.
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        self._flat_btn(self.connect_btn, "green")
        self.conn_status = QLabel("● Disconnected")
        self.conn_status.setStyleSheet("color: gray;")
        conn_l.addWidget(self.port_combo, 0, 0, 1, 2)
        conn_l.addWidget(refresh_btn, 0, 2)
        conn_l.addWidget(self.connect_btn, 1, 0, 1, 2)
        conn_l.addWidget(self.conn_status, 1, 2)
        top_row.addWidget(conn_box, stretch=2)

        loc_box = QGroupBox("Location")
        loc_l = QGridLayout(loc_box)
        self.lat_edit = QLineEdit(str(DEFAULT_LAT))
        self.lon_edit = QLineEdit(str(DEFAULT_LON))
        self.gps_edit = QLineEdit(f"{DEFAULT_LAT}, {DEFAULT_LON}")
        self.gps_mode_check = QCheckBox("lat, lon (GPS/Maps format)")
        self.gps_mode_check.toggled.connect(self._on_gps_mode_toggle)
        set_loc_btn = QPushButton("Set")
        set_loc_btn.clicked.connect(self._set_location)
        self._flat_btn(set_loc_btn, "green")
        self.location_label = QLabel("Loc: not set")
        self.location_label.setStyleSheet("color: #8888aa; font-size: 10px;")
        self.streamer_mode_check = QCheckBox("Streamer Mode (hide GPS)")
        self.streamer_mode_check.toggled.connect(self._on_streamer_mode_toggle)

        self.lat_label = QLabel("Lat:")
        self.lon_label = QLabel("Lon:")
        self.gps_label = QLabel("GPS:")
        loc_l.addWidget(self.gps_mode_check, 0, 0, 1, 5)
        loc_l.addWidget(self.lat_label, 1, 0)
        loc_l.addWidget(self.lat_edit, 1, 1)
        loc_l.addWidget(self.lon_label, 1, 2)
        loc_l.addWidget(self.lon_edit, 1, 3)
        # gps_label/gps_edit occupy the SAME cells as lat/lon (columns 0-3) - only one set is
        # ever visible at a time (toggled in _on_gps_mode_toggle), never both, so they don't
        # overlap visually the way lat_label/lon_label previously stayed visible underneath the
        # combined field (the Lon: label rendering in the middle of the GPS text).
        loc_l.addWidget(self.gps_label, 1, 0)
        loc_l.addWidget(self.gps_edit, 1, 1, 1, 3)
        loc_l.addWidget(set_loc_btn, 1, 4)
        loc_l.addWidget(self.location_label, 2, 0, 1, 4)
        loc_l.addWidget(self.streamer_mode_check, 2, 4)
        self.gps_label.setVisible(False)
        self.gps_edit.setVisible(False)
        top_row.addWidget(loc_box, stretch=3)
        left.addLayout(top_row)

        # ---- Time Travel row ----
        tt_row = QHBoxLayout()
        # Blinking dot instead of a static clock icon - green and blinking once per real second
        # while showing the real-time sky, orange (still blinking) while a Time Travel preview
        # is active - see _update_realtime_dot, driven by wall-clock time so the blink itself is
        # also a live "yes, this is still ticking" signal, not just the color.
        self.realtime_dot = QLabel("●")
        self.realtime_dot.setStyleSheet(f"color: {PALETTE['green']}; font-size: 15px; font-weight: bold;")
        tt_row.addWidget(self.realtime_dot)
        tt_row.addWidget(QLabel("Time Travel (UTC):"))
        self.tt_date_edit = QLineEdit()
        self.tt_date_edit.setFixedWidth(90)
        self.tt_date_edit.setPlaceholderText("YYYY-MM-DD")
        self.tt_time_edit = QLineEdit()
        self.tt_time_edit.setFixedWidth(90)
        self.tt_time_edit.setPlaceholderText("HH:MM:SS UTC")
        self.tt_date_edit.returnPressed.connect(self._apply_time_travel)
        self.tt_time_edit.returnPressed.connect(self._apply_time_travel)
        self.preview_btn = QPushButton("Preview")
        self.preview_btn.clicked.connect(self._apply_time_travel)  # baseline (inactive) style set in _update_time_travel_label
        now_btn = QPushButton("Now (Real Time)")
        now_btn.clicked.connect(self._reset_time_travel)
        self._flat_btn(now_btn, "green")
        self.tt_label = QLabel("Showing: real-time sky")
        self.tt_label.setStyleSheet("color: #8888aa;")
        tt_row.addWidget(self.tt_date_edit)
        tt_row.addWidget(self.tt_time_edit)
        tt_row.addWidget(self.preview_btn)
        tt_row.addWidget(now_btn)
        tt_row.addWidget(self.tt_label, stretch=1)
        left.addLayout(tt_row)

        # ---- prominent status bar ----
        self.status_display = QLabel("DISCONNECTED - Select COM port and click Connect")
        self.status_display.setAlignment(Qt.AlignCenter)
        self.status_display.setStyleSheet(
            f"background-color: #1a1a2e; color: {PALETTE['orange']}; font-weight: bold; font-size: 14px; padding: 6px;")
        left.addWidget(self.status_display)

        # ---- position readout panel ----
        pos_box = QGroupBox("Telescope Position (Sky)")
        pos_l = QGridLayout(pos_box)
        mono = "font-family: Consolas;"
        self.sky_ra_label = QLabel("000.0000°")
        self.sky_ra_label.setStyleSheet(mono + f"color: {PALETTE['green']}; font-size: 16px; font-weight: bold;")
        self.sky_dec_label = QLabel("+00.0000°")
        self.sky_dec_label.setStyleSheet(mono + f"color: {PALETTE['green']}; font-size: 16px; font-weight: bold;")
        # Same font size as the Sky RA/DEC readout (was smaller/plain gray) - and, like
        # EQMountApp's own mount_ra_label ("#aaffaa")/speed_ra_label ("#ffaa88"), each gets its
        # own distinct color rather than sharing one muted gray: teal for Mount (a green-adjacent
        # hue, related to but distinguishable from Sky's green), salmon for Speed (the same
        # peachy-orange role EQMountApp's speed color plays) - both from PALETTE (OKLCH), not a
        # one-off hex.
        self.mount_ra_label = QLabel("0.0000°")
        self.mount_dec_label = QLabel("0.0000°")
        self.speed_ra_label = QLabel("0.000000 °/s")
        self.speed_dec_label = QLabel("0.000000 °/s")
        for lbl in (self.mount_ra_label, self.mount_dec_label):
            lbl.setStyleSheet(mono + f"color: {PALETTE['teal']}; font-size: 16px; font-weight: bold;")
        for lbl in (self.speed_ra_label, self.speed_dec_label):
            lbl.setStyleSheet(mono + f"color: {PALETTE['salmon']}; font-size: 16px; font-weight: bold;")

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
            self._flat_btn(b, "teal")  # same hue as the Mount readout these offsets adjust

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

        # Visible delimiter between the Sky RA and Sky DEC lines - a plain QGridLayout with no
        # per-cell background otherwise has no boundary between them at all (see _APP_STYLESHEET
        # for why the GroupBox's own outer border needed the same fix).
        ra_dec_sep = QFrame()
        ra_dec_sep.setFrameShape(QFrame.HLine)
        ra_dec_sep.setStyleSheet("color: #3a3a4a;")
        pos_l.addWidget(ra_dec_sep, 1, 0, 1, 10)

        pos_l.addWidget(QLabel("Sky DEC:"), 2, 0)
        pos_l.addWidget(self.sky_dec_label, 2, 1)
        pos_l.addWidget(QLabel("Mount:"), 2, 2)
        pos_l.addWidget(self.mount_dec_label, 2, 3)
        pos_l.addWidget(QLabel("Speed:"), 2, 4)
        pos_l.addWidget(self.speed_dec_label, 2, 5)
        pos_l.addWidget(QLabel("Offset:"), 2, 6)
        pos_l.addWidget(dec_off_minus, 2, 7)
        pos_l.addWidget(self.dec_offset_edit, 2, 8)
        pos_l.addWidget(dec_off_plus, 2, 9)

        below_dec_sep = QFrame()
        below_dec_sep.setFrameShape(QFrame.HLine)
        below_dec_sep.setStyleSheet("color: #3a3a4a;")
        pos_l.addWidget(below_dec_sep, 3, 0, 1, 10)

        self.offset_inc_edit = QLineEdit("0.01")
        self.offset_inc_edit.setFixedWidth(60)
        pos_l.addWidget(QLabel("Offset increment:"), 4, 0, 1, 2)
        pos_l.addWidget(self.offset_inc_edit, 4, 2)

        self.error_label = QLabel("Error: RA 0.0000° | DEC 0.0000°")
        self.error_label.setAlignment(Qt.AlignCenter)
        self.error_label.setStyleSheet(f"color: {PALETTE['orange']}; font-size: 18px; font-weight: bold;")
        pos_l.addWidget(self.error_label, 5, 0, 1, 10)

        # Stability badge - STABLE/SETTLING/DRIFTING/ALIGNING/TRACKING:OFF, driven by
        # _update_tracking_stability (ported from EQMountApp._update_tracking_stability's
        # error-trend classification, not just a plain magnitude threshold).
        self.tracking_status_label = QLabel("OFF")
        self.tracking_status_label.setAlignment(Qt.AlignCenter)
        self.tracking_status_label.setStyleSheet(
            "background-color: #444455; color: white; font-weight: bold; font-size: 14px; padding: 5px; border-radius: 8px;")
        self.tracking_status_label.setFixedWidth(300)
        pos_l.addWidget(self.tracking_status_label, 6, 0, 1, 10, alignment=Qt.AlignCenter)

        # Fills the space below the badge (previously just empty) with whatever's actually
        # being tracked, e.g. a named star/DSO/Sun/Moon/ISS from a search selection or
        # double-click (see _select_sky_target/target_object_name) - blank when tracking an
        # arbitrary manually-entered RA/DEC, same as EQMountApp's target_name_label.
        self.target_name_label = QLabel("")
        self.target_name_label.setAlignment(Qt.AlignCenter)
        self.target_name_label.setStyleSheet(f"color: {PALETTE['yellow']}; font-weight: bold; font-size: 26px; padding: 10px;")
        pos_l.addWidget(self.target_name_label, 7, 0, 1, 10)

        self.meridian_warning_label = QLabel("")
        self.meridian_warning_label.setAlignment(Qt.AlignCenter)
        self.meridian_warning_label.setStyleSheet(f"color: {PALETTE['orange']}; font-weight: bold;")
        pos_l.addWidget(self.meridian_warning_label, 8, 0, 1, 10)

        stability_note = QLabel(
            f"Stable = error ≤ {TRACKING_STABLE_ERR_DEG:.4f}° (½ camera pixel) AND flat/"
            f"shrinking over {TRACKING_STABILITY_WINDOW_S:.0f}s (growth ≤ "
            f"{TRACKING_STABLE_DERIV_DEG_S:.4f}°/s)  ·  Settling = still converging  ·  "
            f"Drifting = error growing")
        stability_note.setWordWrap(True)
        stability_note.setAlignment(Qt.AlignCenter)
        stability_note.setStyleSheet("color: #8888aa; font-size: 9px;")
        pos_l.addWidget(stability_note, 9, 0, 1, 10)
        left.addWidget(pos_box)

        # ---- toolbar row above the sky viz ----
        toolbar = QHBoxLayout()
        self.view_mode_btn = QPushButton("View: Free")
        self.view_mode_btn.clicked.connect(self._cycle_view_mode)
        self._update_view_mode_btn_style()
        self.iss_btn = QPushButton("ISS: OFF")
        self.iss_btn.clicked.connect(self._toggle_iss)
        self._style_toggle_btn(self.iss_btn, False)
        self.const_btn = QPushButton("Constellations: ON")
        self.const_btn.clicked.connect(self._toggle_constellations)
        self._style_toggle_btn(self.const_btn, True)  # constellations_enabled defaults True
        self.min_size_btn = QPushButton("Min Size: OFF")
        self.min_size_btn.clicked.connect(self._toggle_min_size_filter)
        self._style_toggle_btn(self.min_size_btn, False)
        self.min_size_edit = QLineEdit("100.0")
        self.min_size_edit.setFixedWidth(50)
        self.min_size_edit.editingFinished.connect(self._on_min_size_commit)
        cam_minus = QPushButton("◄")
        cam_plus = QPushButton("►")
        cam_minus.setFixedWidth(28)
        cam_plus.setFixedWidth(28)
        self._flat_btn(cam_minus, "blue")  # camera/telescope hue, matching the FOV overlay
        self._flat_btn(cam_plus, "blue")
        cam_minus.clicked.connect(lambda: self._adjust_camera_orientation(-5.0))
        cam_plus.clicked.connect(lambda: self._adjust_camera_orientation(5.0))
        self.cam_rot_edit = QLineEdit("0.0")
        self.cam_rot_edit.setFixedWidth(50)
        self.cam_rot_edit.editingFinished.connect(self._on_cam_rot_commit)
        self.find_edit = QLineEdit()
        self.find_edit.setPlaceholderText("Find: star/DSO/Sun/Moon/ISS/planet name...")
        self.find_edit.textChanged.connect(self._update_sky_search_results)
        self.find_edit.returnPressed.connect(self._select_first_sky_search_result)
        # Down from the search box -> jump into the results list, top item selected; Up from
        # the list's top item -> back to the search box to keep editing the query. Handled via
        # an event filter (see eventFilter below) since neither QLineEdit nor QListWidget has a
        # per-key signal the way Tk's bind() does.
        self.find_edit.installEventFilter(self)
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

        # Live-filtered multi-result list (see _update_sky_search_results) - hidden until there's
        # something to show, same as EQMountApp's sky_search_results Listbox.
        self.sky_search_results = QListWidget()
        self.sky_search_results.setMaximumHeight(110)
        self.sky_search_results.itemActivated.connect(self._on_sky_search_result_chosen)
        self.sky_search_results.hide()
        self.sky_search_results.installEventFilter(self)
        left.addWidget(self.sky_search_results)

        left.addWidget(self.viz, stretch=1)

        # ---- bottom row: cursor position, emergency stop, mode/tracking, live error ----
        self.hover_label = QLabel("Cursor: -")
        self.hover_label.setStyleSheet("color: #9aa; font-family: Consolas;")

        # Styled per ISO 13850 (emergency stop function) actuator convention - RED actuator on a
        # YELLOW background, the one color pairing that standard exists specifically to make
        # unambiguous - deliberately NOT drawn from PALETTE (see its module docstring). Actuation
        # itself already matches the standard's intent too - one click, immediate (_stop ->
        # _request_tracking_action), no confirmation dialog in the way, same as every other STOP
        # path in this app (Delete key, panic byte). On the same row as the cursor/mode/error
        # readouts directly under the viz (not the sidebar) - the one control that should be
        # reachable without hunting for it regardless of what else is on screen, closest to
        # where your eyes already are while watching the viz.
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {STOP_RED};
                color: white;
                border: 2px solid #FFD500;
                border-radius: 5px;
                font-weight: bold;
                font-size: 12px;
                padding: 4px;
            }}
            QPushButton:hover {{ background-color: {STOP_RED_HOVER}; }}
            QPushButton:pressed {{ background-color: {STOP_RED_PRESSED}; }}
        """)
        self.stop_btn.setFixedSize(92, 46)  # width doubled from the previous 46x46 square
        self.stop_btn.setToolTip("Immediately stops tracking/slewing - single action, no confirmation (ISO 13850 emergency stop convention).")
        self.stop_btn.clicked.connect(self._stop)

        self.mode_tracking_label = QLabel("MODE: SIDEREAL  |  TRACKING: OFF")
        self.live_error_label = QLabel("Live Error: RA 0.0000°  DEC 0.0000°")
        self.live_error_label.setStyleSheet(f"color: {PALETTE['orange']};")

        error_row = QHBoxLayout()
        error_row.addWidget(self.mode_tracking_label)
        error_row.addSpacing(16)
        error_row.addWidget(self.live_error_label)
        error_box = QWidget()
        error_box.setLayout(error_row)

        # Equal *stretch factors* on either side of the button do NOT by themselves guarantee
        # the button lands at the row's true midpoint - stretch only governs how LEFTOVER space
        # (beyond each side's own natural content width) gets split, so with the cursor label
        # (narrow) on one side and the mode/error labels (much wider) on the other, the button
        # still ends up off-center toward the narrower side. Forcing both side containers to the
        # SAME explicit width (the wider of the two, computed from their actual content) is what
        # actually centers it - content that doesn't fill its side just sits left/right-aligned
        # within that fixed width instead.
        side_w = max(self.hover_label.sizeHint().width(), error_box.sizeHint().width())
        left_wrap = QWidget()
        left_wrap_l = QHBoxLayout(left_wrap)
        left_wrap_l.setContentsMargins(0, 0, 0, 0)
        left_wrap_l.addWidget(self.hover_label)
        left_wrap_l.addStretch(1)
        left_wrap.setFixedWidth(side_w)

        right_wrap = QWidget()
        right_wrap_l = QHBoxLayout(right_wrap)
        right_wrap_l.setContentsMargins(0, 0, 0, 0)
        right_wrap_l.addStretch(1)
        right_wrap_l.addWidget(error_box)
        right_wrap.setFixedWidth(side_w)

        bottom = QHBoxLayout()
        bottom.addWidget(left_wrap)
        bottom.addStretch(1)
        bottom.addWidget(self.stop_btn)
        bottom.addStretch(1)
        bottom.addWidget(right_wrap)
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
        self.tracking_btn.setStyleSheet(f"background-color: {PALETTE['green']}; color: black; font-weight: bold; padding: 8px;")
        self.tracking_btn.clicked.connect(self._toggle_tracking)
        right.addWidget(self.tracking_btn)

        self.safe_target_btn = QPushButton("Safe Target (DEC to 0°)")
        self.safe_target_btn.clicked.connect(self._safe_target)
        self._flat_btn(self.safe_target_btn, "yellow")  # caution, echoing EQMountApp's olive fg_color for these two
        right.addWidget(self.safe_target_btn)
        self.home_axes_btn = QPushButton("Home Axes (RA/DEC to 0°)")
        self.home_axes_btn.clicked.connect(self._home_axes)
        self._flat_btn(self.home_axes_btn, "sun")  # distinct from Safe Target's yellow, same warm/caution family
        right.addWidget(self.home_axes_btn)

        # Manual meridian-flip toggle - see EQMountApp._toggle_telescope_flipped's docstring:
        # requests the OPPOSITE of the last CONFIRMED state; the button text/color only updates
        # once the firmware actually confirms (POS's "flipped" field / STATUS:FLIP_COMPLETE),
        # not optimistically, since the RA axis physically has to rotate ~180° first.
        self.flipped_btn = QPushButton("Telescope Flipped: OFF")
        self.flipped_btn.clicked.connect(self._toggle_telescope_flipped)
        right.addWidget(self.flipped_btn)

        sync_box = QGroupBox("Sync / Calibrate (choose star)")
        sync_l = QVBoxLayout(sync_box)
        self.sync_combo = QComboBox()
        self.sync_combo.addItems(list(self.cal_stars.keys()))
        self.sync_combo.setCurrentText("Vega (alpha Lyrae)")
        sync_l.addWidget(self.sync_combo)
        sync_btn = QPushButton("Sync Position (send to Arduino)")
        sync_btn.clicked.connect(self._sync)
        self._flat_btn(sync_btn, "green")
        sync_l.addWidget(sync_btn)
        right.addWidget(sync_box)

        target_box = QGroupBox("Sidereal Target (degrees) - used by Start Tracking / Goto")
        target_l = QGridLayout(target_box)
        self.ra_edit = QLineEdit("0.0")
        self.dec_edit = QLineEdit("0.0")
        goto_btn = QPushButton("Goto")
        goto_btn.clicked.connect(self._goto)
        self._flat_btn(goto_btn, "blue")
        target_l.addWidget(QLabel("RA:"), 0, 0)
        target_l.addWidget(self.ra_edit, 0, 1)
        target_l.addWidget(QLabel("DEC:"), 1, 0)
        target_l.addWidget(self.dec_edit, 1, 1)
        target_l.addWidget(goto_btn, 2, 0, 1, 2)
        right.addWidget(target_box)

        resync_btn = QPushButton("Resync Arduino Time to Laptop")
        resync_btn.clicked.connect(lambda: self._manual_resync())
        self._flat_btn(resync_btn, "cyan")
        right.addWidget(resync_btn)

        self.pos_rate_label = QLabel(
            f"Arduino POS Update Rate: requesting {POS_UPDATE_RATE_MS} ms (~{POSITION_BROADCAST_HZ:.0f} Hz)...")
        self.pos_rate_label.setWordWrap(True)
        self.pos_rate_label.setStyleSheet("color: #8888aa; font-size: 10px;")
        right.addWidget(self.pos_rate_label)

        debug_box = QGroupBox("Arduino Debug (verbose var dump)")
        debug_l = QHBoxLayout(debug_box)
        self.debug_toggle_btn = QPushButton("Verbose Debug: OFF")
        self.debug_toggle_btn.clicked.connect(self._toggle_arduino_debug)
        self._style_toggle_btn(self.debug_toggle_btn, False)
        dump_btn = QPushButton("Force Dump")
        dump_btn.clicked.connect(lambda: self.serial.request_debug_dump())
        self._flat_btn(dump_btn, "cyan")
        debug_l.addWidget(self.debug_toggle_btn)
        debug_l.addWidget(dump_btn)
        right.addWidget(debug_box)

        log_box = QGroupBox("Status / Messages")
        log_l = QVBoxLayout(log_box)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setStyleSheet("background-color: #111; color: #ccc; font-family: Consolas;")
        log_l.addWidget(self.log_edit)
        right.addWidget(log_box, stretch=1)

        self._arduino_widgets = [
            self.safe_target_btn, self.home_axes_btn, sync_btn, goto_btn,
            self.tracking_btn, self.debug_toggle_btn, dump_btn, self.flipped_btn,
            ra_off_minus, ra_off_plus, dec_off_minus, dec_off_plus,
        ] + [self.mode_group.button(i) for i in range(3)]

    def _log(self, msg, masked_msg=None):
        """Appends to the Status/Messages log. `masked_msg` is an optional alternate text to
        show instead, whenever Streamer Mode is on, for any log line that would otherwise leak
        the real GPS coordinates (see _set_location/_load_gui_config's calls) - Streamer Mode
        is supposed to hide the location entirely (see _on_streamer_mode_toggle), and the log
        box was the one place that kept showing it in plain text regardless. Every line is kept
        in self._log_history (msg, masked_msg) so toggling Streamer Mode mid-session can
        retroactively re-render everything already logged, not just lines logged from then on."""
        self._log_history.append((msg, masked_msg))
        self.log_edit.append(masked_msg if (self.streamer_mode and masked_msg is not None) else msg)

    def _rebuild_log_display(self):
        self.log_edit.clear()
        for msg, masked_msg in self._log_history:
            self.log_edit.append(masked_msg if (self.streamer_mode and masked_msg is not None) else msg)

    # ---------------- location / lat-lon ----------------
    def _get_lat_lon(self):
        """Mirrors EQMountApp._get_lat_lon_from_input - robust number extraction so a pasted
        Google Maps "lat, lon" string works even before clicking Set, in either input mode."""
        if self.gps_mode:
            nums = re.findall(r"[-+]?\d*\.?\d+", self.gps_edit.text().strip())
            if len(nums) >= 2:
                try:
                    return float(nums[0]), float(nums[1])
                except ValueError:
                    pass
            return DEFAULT_LAT, DEFAULT_LON
        try:
            return float(self.lat_edit.text()), float(self.lon_edit.text())
        except ValueError:
            return DEFAULT_LAT, DEFAULT_LON

    def _on_gps_mode_toggle(self, checked):
        self.gps_mode = checked
        if checked:
            try:
                lat, lon = float(self.lat_edit.text()), float(self.lon_edit.text())
                self.gps_edit.setText(f"{lat}, {lon}")
            except ValueError:
                pass
            self.lat_label.setVisible(False)
            self.lat_edit.setVisible(False)
            self.lon_label.setVisible(False)
            self.lon_edit.setVisible(False)
            self.gps_label.setVisible(True)
            self.gps_edit.setVisible(True)
        else:
            nums = re.findall(r"[-+]?\d*\.?\d+", self.gps_edit.text())
            if len(nums) >= 2:
                self.lat_edit.setText(nums[0])
                self.lon_edit.setText(nums[1])
            self.gps_label.setVisible(False)
            self.gps_edit.setVisible(False)
            self.lat_label.setVisible(True)
            self.lat_edit.setVisible(True)
            self.lon_label.setVisible(True)
            self.lon_edit.setVisible(True)
        if hasattr(self, "_location_save_timer"):
            self._location_save_timer.start(200)

    def _on_streamer_mode_toggle(self, checked):
        """Mirrors EQMountApp._on_streamer_mode_toggle - masks the location display only, never
        clears the underlying values, so sync/Sun-Moon/etc. keep working normally while hidden.
        Also masks the Status/Messages log (see _log/_rebuild_log_display) - previously that box
        kept showing the real coordinates in plain text regardless of this toggle."""
        self.streamer_mode = checked
        mode = QLineEdit.Password if checked else QLineEdit.Normal
        self.lat_edit.setEchoMode(mode)
        self.lon_edit.setEchoMode(mode)
        self.gps_edit.setEchoMode(mode)
        if checked:
            self._set_location_label("Loc: •••••• (hidden)")
        else:
            lat, lon = self._get_lat_lon()
            self._set_location_label(f"Loc: {lat}, {lon}")
        self._rebuild_log_display()

    def _set_location_label(self, text):
        self.location_label.setText("Loc: •••••• (hidden)" if self.streamer_mode else text)

    def _set_location(self):
        lat, lon = self._get_lat_lon()
        self.viz.lat, self.viz.lon = lat, lon
        self.viz.update()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")
        self._set_location_label(f"Loc: {lat}, {lon}")
        self._log(f"Location set: LAT {lat:.4f}, LON {lon:.4f}", masked_msg="Location set: LAT ••••••, LON ••••••")

    def _on_location_changed(self):
        """Debounced (200ms, see _location_save_timer) reaction to editing any of the location
        fields - saves gui_config.json, pushes the new location live to the Arduino if
        connected, and redraws the viz's horizon, same as EQMountApp._on_location_var_changed.
        Not tied to the Set button - typing a new value alone is enough, matching the Tk app."""
        self._save_gui_config()
        lat, lon = self._get_lat_lon()
        self.viz.lat, self.viz.lon = lat, lon
        self.viz.update()
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")

    def _load_gui_config(self):
        """Loads the single GPS coordinate from CONFIG_PATH (gui_config.json) - the SAME file
        tracker_gui.py reads/writes, not a separate config for this GUI (see its
        _load_gui_config's docstring: only "gps"/"location", a combined "lat, lon" string, is
        stored). Switches to GPS-format mode on load, same as the Tk app."""
        if not os.path.exists(CONFIG_PATH):
            print(f"[gui_config] no config file at {CONFIG_PATH} - using default location")
            return
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        except Exception as e:
            self._log(f"Could not load gui_config.json: {e}")
            return
        if not isinstance(cfg, dict):
            return
        gps_val = cfg.get("gps") or cfg.get("location")
        if not gps_val:
            return
        gps_str = str(gps_val).strip()
        self.gps_edit.setText(gps_str)
        parts = [p.strip() for p in gps_str.split(",")]
        if len(parts) == 2:
            self.lat_edit.setText(parts[0])
            self.lon_edit.setText(parts[1])
        self.gps_mode_check.setChecked(True)  # triggers _on_gps_mode_toggle to show gps_edit
        lat, lon = self._get_lat_lon()
        self.viz.lat, self.viz.lon = lat, lon
        self._set_location_label(f"Loc: {lat}, {lon} (loaded)")
        self._log(f"Loaded saved location from gui_config.json: {gps_str}",
                  masked_msg="Loaded saved location from gui_config.json: •••••• (hidden)")

    def _save_gui_config(self):
        """Same schema as EQMountApp._save_gui_config: {"gps": "lat, lon"} written to
        CONFIG_PATH - the same gui_config.json the Tk app uses, so either GUI's last-set
        location is what the other one loads next time."""
        lat, lon = self._get_lat_lon()
        gps = f"{lat}, {lon}"
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump({"gps": gps}, f, indent=2)
        except OSError as e:
            self._log(f"Could not save gui_config.json: {e}")

    # ---------------- Time Travel (ported from EQMountApp) ----------------
    def _reset_time_travel_inputs_to_now(self):
        utc_now = datetime.now(timezone.utc)
        self.tt_date_edit.setText(utc_now.strftime("%Y-%m-%d"))
        self.tt_time_edit.setText(utc_now.strftime("%H:%M:%S"))

    def _apply_time_travel(self):
        """Interpreted as UTC directly (not local time) - see EQMountApp._apply_time_travel's
        docstring for why that matters for previewing a real published event time exactly."""
        date_str = self.tt_date_edit.text().strip()
        time_str = self.tt_time_edit.text().strip() or "00:00:00"
        if time_str.count(":") == 1:
            time_str += ":00"
        try:
            naive = datetime.strptime(f"{date_str} {time_str}", "%Y-%m-%d %H:%M:%S")
        except ValueError:
            self._log(f"Time Travel: invalid date/time - use YYYY-MM-DD and HH:MM:SS UTC "
                      f"(got '{date_str} {time_str}')")
            return
        target_utc = naive.replace(tzinfo=timezone.utc)
        self._time_travel_offset = target_utc - datetime.now(timezone.utc)
        pushed = self.serial.send_time_if_connected()
        self._log(f"Time Travel: previewing sky as of {naive.strftime('%Y-%m-%d %H:%M:%S')} UTC"
                  + (" - Arduino clock synced to it too" if pushed else
                     " - Arduino not connected, will sync on connect"))
        self._update_time_travel_label()
        self._trigger_solar_system_update()
        self._trigger_iss_update()
        self.viz.update()

    def _reset_time_travel(self):
        self._time_travel_offset = timedelta(0)
        self.serial.send_time_if_connected()
        self._reset_time_travel_inputs_to_now()
        self._log("Time Travel: back to real-time sky (Arduino clock resynced to real time)")
        self._update_time_travel_label()
        self._trigger_solar_system_update()
        self._trigger_iss_update()
        self.viz.update()

    def _update_time_travel_label(self):
        if self._time_travel_offset == timedelta(0):
            self.tt_label.setText("Showing: real-time sky")
            self.tt_label.setStyleSheet("color: #8888aa;")
            self.preview_btn.setStyleSheet("")
            # Keep the date/time fields ticking forward live while showing the real-time sky,
            # instead of staying frozen at whatever _reset_time_travel_inputs_to_now() last set
            # them to - skipped while either field has focus, so this can't clobber a date/time
            # the user is actively typing in to set up a preview.
            if not (self.tt_date_edit.hasFocus() or self.tt_time_edit.hasFocus()):
                self._reset_time_travel_inputs_to_now()
        else:
            eff_utc = self._get_effective_utc_now()
            connected = bool(self.serial.ser and self.serial.ser.is_open)
            suffix = "" if connected else " - Arduino not connected"
            self.tt_label.setText(f"⚠ TIME TRAVEL: {eff_utc.strftime('%Y-%m-%d %H:%M:%S')} UTC{suffix}")
            self.tt_label.setStyleSheet("color: " + PALETTE["orange"] + ";")
            # Same orange as the real-time dot in preview mode (see _update_realtime_dot) - and,
            # like every other accent color in this app, from PALETTE (OKLCH), not a one-off hex.
            self.preview_btn.setStyleSheet(
                f"background-color: {PALETTE['orange']}; color: black; font-weight: bold;")
        # Reschedule for the exact next real second boundary (see this timer's setup in
        # __init__) instead of a fixed 1000ms poll - same technique as _update_realtime_dot.
        frac = time.time() % 1.0
        self.time_travel_label_timer.start(max(1, round((1.0 - frac) * 1000)))

    def _update_realtime_dot(self):
        """Blinks once per real (wall-clock) second - LIT the instant each second starts, OFF
        from the half-second mark until the next second rolls over - green while showing the
        real-time sky, orange while a Time Travel preview is active.

        Self-rescheduling (via blink_timer, a single-shot PreciseTimer) rather than a fixed-
        interval repeating poll: computes the current phase from time.time() itself (so it's
        correct regardless of drift/jitter since the last call), then schedules the NEXT call to
        land as close as possible to the next actual 0.0/0.5-second boundary, instead of polling
        every N ms and potentially showing a stale state for up to N ms after a boundary passed
        (reported as "not always well in sync with the computer time")."""
        frac = time.time() % 1.0
        lit = frac < 0.5
        base_color = PALETTE["green"] if self._time_travel_offset == timedelta(0) else PALETTE["orange"]
        self.realtime_dot.setStyleSheet(
            f"color: {base_color if lit else '#333340'}; font-size: 15px; font-weight: bold;")
        next_boundary = 0.5 if lit else 1.0
        self.blink_timer.start(max(1, round((next_boundary - frac) * 1000)))

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

        # Search index for the Find box - named stars, all DSOs (by designation/name), plus
        # Sun/Moon/ISS/planets as "live" entries whose ra/dec aren't fixed here (resolved at
        # selection time in _choose_sky_search_entry) - same structure as
        # EQMountApp._build_sky_catalog_state's _sky_search_index.
        index = []
        for s in self.viz.sky_stars_by_ra:
            if not s.get("name"):
                continue
            index.append({"name": s["name"], "ra": s["ra"], "dec": s["dec"], "live": None})
        for d in self.viz.sky_dso:
            display = f"{d['designation']} ({d['name']})" if d.get("name") else d["designation"]
            index.append({"name": display, "ra": d["ra"], "dec": d["dec"], "live": None})
        for live_name in ("Sun", "Moon", "ISS", "Mercury", "Venus", "Mars", "Jupiter", "Saturn",
                          "Uranus", "Neptune"):
            index.append({"name": live_name, "ra": None, "dec": None, "live": live_name})
        self._search_index = index

    def _resolve_live_body(self, live_name):
        if live_name == "Sun":
            return (self.viz.sun_ra, self.viz.sun_dec) if self.viz.sun_ra is not None else (None, None)
        if live_name == "Moon":
            return (self.viz.moon_ra, self.viz.moon_dec) if self.viz.moon_ra is not None else (None, None)
        if live_name == "ISS":
            return (self.viz.iss_ra, self.viz.iss_dec) if self.viz.iss_ra is not None else (None, None)
        pos = self.viz.planet_positions.get(live_name)
        return (pos[0], pos[1]) if pos else (None, None)

    # ---------------- solar system / ISS ----------------
    def _trigger_solar_system_update(self):
        worker = _SolarSystemWorker(self._get_lat_lon, at_time=self._get_effective_utc_now())
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

    def _iss_actively_tracked(self):
        """True while the ISS is the currently active tracked target - see
        EQMountApp._iss_update_tick's docstring for why this matters: it's what decides between
        a leisurely 1Hz marker-only refresh and a fast ISS_TRACKING_UPDATE_MS one that actually
        keeps the Arduino's target current enough to follow the ISS's real orbital motion."""
        mode = self.mode_group.checkedButton().text() if self.mode_group.checkedButton() else ""
        return self.target_object_name == "ISS" and self.tracking and mode == "SIDEREAL"

    def _iss_thread_state(self):
        """Called from _IssTrackingThread's own loop (background thread) each cycle - returns
        (enabled, interval_ms): whether to fetch at all (skip entirely while the ISS toggle is
        off, rather than always polling in the background regardless of whether anything's
        displaying it) and how long to sleep until the next cycle, fast (ISS_TRACKING_UPDATE_MS)
        while the ISS is the actively tracked target, otherwise a leisurely 1Hz
        (ISS_DISPLAY_UPDATE_MS) marker-only refresh."""
        return self.viz.iss_enabled, (ISS_TRACKING_UPDATE_MS if self._iss_actively_tracked() else ISS_DISPLAY_UPDATE_MS)

    def _trigger_iss_update(self):
        """Wakes the persistent ISS thread for an immediate refresh right now, instead of
        waiting out whatever interval it's currently sleeping through - used after a Time Travel
        jump (position needs to reflect the new time right away) and when ISS tracking actually
        starts (see _select_sky_target)."""
        self.iss_thread.wake()

    def _on_iss_updated(self, result):
        ra, dec, above = result
        self.viz.iss_ra, self.viz.iss_dec, self.viz.iss_above = ra, dec, above
        self.viz.update()

        # If the ISS is the currently active tracked target, keep the Arduino's target fresh
        # too - not just the on-screen marker. Same CONTINUATION:1 mechanism as
        # EQMountApp._on_iss_position: tells the firmware this is the SAME tracked object being
        # refreshed (not a fresh target), so it's safe to derive a tracking rate from consecutive
        # updates, the same SIDEREAL SET_TARGET + continuous-correction machinery any other
        # search-selected target uses.
        if self._iss_actively_tracked() and self.serial.ser and self.serial.ser.is_open:
            self.target_ra, self.target_dec = ra, dec
            self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f},CONTINUATION:1")

    # ---------------- serial connection ----------------
    def _refresh_ports(self):
        ports = self.serial.list_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports if ports else ["No ports found"])

    def _toggle_connection(self):
        if self.serial.ser and self.serial.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText()
        if not port or "No ports" in port:
            self._log("No valid port selected.")
            return
        self._log(f"Connecting to {port} ...")
        if self.serial.connect(port):
            self.connect_btn.setText("Disconnect")
            self._flat_btn(self.connect_btn, "red")
            self._set_arduino_controls_enabled(True)
            self._set_location()
            self._send_current_offsets()
            QTimer.singleShot(1200, lambda: self.serial.send_pos_update_rate(POS_UPDATE_RATE_MS))
            self._connection_timeout_timer.start(3000)

    def _handle_connection_timeout(self):
        self._update_status_display("CONNECTED - NO ARDUINO RESPONSE", PALETTE["red"])
        self._log("Warning: Arduino did not respond to PING command. Connection may be unstable.")

    def _disconnect(self):
        self._connection_timeout_timer.stop()
        self.serial.disconnect()
        self.connect_btn.setText("Connect")
        self._flat_btn(self.connect_btn, "green")
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
        self._force_stop_tracking()
        self._force_full_align_next_start = True
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_mode("SIDEREAL")  # wire protocol is always SIDEREAL - see tracker_gui.py's _on_mode_changed
        self._log(f"Mode changed to {mode} (tracking stopped)")
        self._update_mode_tracking_label()

    def _update_mode_tracking_label(self):
        mode = self.mode_group.checkedButton().text() if self.mode_group.checkedButton() else "SIDEREAL"
        state = "ON" if self.tracking else "OFF"
        self.mode_tracking_label.setText(f"MODE: {mode}  |  TRACKING: {state}")
        self._update_tracking_stability()
        self._update_tracking_btn_style()

    def _update_tracking_btn_style(self):
        """Start/Stop Tracking is one toggle button (like EQMountApp's tracking_btn), separate
        from the always-visible ISO 13850 STOP actuator below it (see its own comment) - that one
        is the immediate, no-questions-asked emergency stop; this one is the normal
        start<->stop toggle for the current tracking session, and reflects which action it would
        currently perform. Called from _update_mode_tracking_label, the one place every tracking
        state change already funnels through."""
        if self.tracking:
            self.tracking_btn.setText("■ Stop Tracking")
            # TRACKING_STOP_RED, not PALETTE["red"] - a bit less saturated than even the rest of
            # the palette's own red, at the user's request, for this specific button.
            self.tracking_btn.setStyleSheet(
                f"background-color: {TRACKING_STOP_RED}; color: black; font-weight: bold; padding: 8px;")
        else:
            self.tracking_btn.setText("▶ Start Tracking")
            self.tracking_btn.setStyleSheet(
                f"background-color: {PALETTE['green']}; color: black; font-weight: bold; padding: 8px;")

    @staticmethod
    def _style_toggle_btn(btn, on):
        """A plain QPushButton looks identical whether the thing it toggles is on or off (no
        default "pressed/active" look the way a checkable toolbar button would have) - this
        gives every on/off toolbar button (ISS, Constellations, Min Size, Verbose Debug, ...) a
        distinct green fill while active, matching CustomTkinter's fg_color-swap convention in
        tracker_gui.py, instead of the text label being the only thing that changes. OFF is left
        at the plain default look - only the active state gets a color treatment."""
        btn.setStyleSheet(f"background-color: {PALETTE['green']}; color: black; font-weight: bold;" if on else "")

    @staticmethod
    def _flat_btn(btn, palette_name):
        """Flat-filled button in one PALETTE color, black text (the palette is light/pastel -
        see PALETTE's own module docstring - so black reads better than white on it, same
        reasoning as every other filled button/badge in this app). Every plain QPushButton that
        isn't already a stateful toggle (see _style_toggle_btn) or otherwise state-driven
        (tracking_btn, flipped_btn, stop_btn, preview_btn) gets one of these instead of the
        default unstyled look, loosely echoing which color role EQMountApp gave the matching
        button (green ~ CTk's default "go" blue/positive actions, red ~ its disconnect red,
        yellow ~ its olive safe-target/home-axes, cyan ~ neutral utility actions it left default)."""
        btn.setStyleSheet(f"background-color: {PALETTE[palette_name]}; color: black; font-weight: bold;")

    def _cycle_view_mode(self):
        order = ["FREE", "TELESCOPE", "TARGET"]
        self.viz.view_mode = order[(order.index(self.viz.view_mode) + 1) % len(order)]
        self.view_mode_btn.setText(f"View: {self.viz.view_mode.title()}")
        self._update_view_mode_btn_style()

    def _on_viz_view_mode_changed(self, mode):
        """The viz itself drops back to Free on a manual drag-pan or middle-click reset (see
        SkyViewWidget._set_view_mode) - keeps view_mode_btn's label/color in sync with that,
        instead of it still reading "Telescope"/"Target" after the viz has already switched."""
        self.view_mode_btn.setText(f"View: {mode.title()}")
        self._update_view_mode_btn_style()

    def _update_view_mode_btn_style(self):
        """Free = default button color. Telescope = PALETTE["blue"], the same as the camera FOV
        rectangle overlay (see SkyViewWidget._draw_camera_and_reticle). Target =
        PALETTE["orange"], the same as the target reticle (same place) - so the button's color
        itself tells you which overlay the viz is currently following, matching that overlay's
        own color."""
        if self.viz.view_mode == "TELESCOPE":
            self.view_mode_btn.setStyleSheet(f"background-color: {PALETTE['blue']}; color: black; font-weight: bold;")
        elif self.viz.view_mode == "TARGET":
            self.view_mode_btn.setStyleSheet(f"background-color: {PALETTE['orange']}; color: black; font-weight: bold;")
        else:
            self.view_mode_btn.setStyleSheet("")

    def _toggle_iss(self):
        self.viz.iss_enabled = not self.viz.iss_enabled
        self.iss_btn.setText(f"ISS: {'ON' if self.viz.iss_enabled else 'OFF'}")
        self._style_toggle_btn(self.iss_btn, self.viz.iss_enabled)
        if self.viz.iss_enabled:
            self.iss_thread.wake()  # don't wait out the idle poll interval before the first fetch
        self.viz.update()

    def _toggle_constellations(self):
        self.viz.constellations_enabled = not self.viz.constellations_enabled
        self.const_btn.setText(f"Constellations: {'ON' if self.viz.constellations_enabled else 'OFF'}")
        self._style_toggle_btn(self.const_btn, self.viz.constellations_enabled)
        self.viz.update()

    def _toggle_min_size_filter(self):
        self.viz.min_dso_size_filter_enabled = not self.viz.min_dso_size_filter_enabled
        self.min_size_btn.setText(f"Min Size: {'ON' if self.viz.min_dso_size_filter_enabled else 'OFF'}")
        self._style_toggle_btn(self.min_size_btn, self.viz.min_dso_size_filter_enabled)
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

    def _update_sky_search_results(self, text=None):
        """Live-filter self._search_index by substring match as the Find box is typed into -
        ported from EQMountApp._update_sky_search_results, including its ranking (prefix match
        first, then shortest name) and result cap."""
        query = self.find_edit.text().strip().lower()
        self.sky_search_results.clear()
        if not query:
            self._sky_search_matches = []
            self.sky_search_results.hide()
            return
        matches = [e for e in self._search_index if query in e["name"].lower()]
        matches.sort(key=lambda e: (not e["name"].lower().startswith(query), len(e["name"])))
        self._sky_search_matches = matches[:30]
        if not self._sky_search_matches:
            self.sky_search_results.hide()
            return
        for entry in self._sky_search_matches:
            self.sky_search_results.addItem(entry["name"])
        self.sky_search_results.show()

    def _select_first_sky_search_result(self):
        """Enter in the search box itself picks the top (best-ranked) match directly."""
        if self._sky_search_matches:
            self._choose_sky_search_entry(self._sky_search_matches[0])

    def _on_sky_search_result_chosen(self, item):
        row = self.sky_search_results.row(item)
        if 0 <= row < len(self._sky_search_matches):
            self._choose_sky_search_entry(self._sky_search_matches[row])

    def _choose_sky_search_entry(self, entry):
        """Resolves the chosen entry to a real RA/DEC (live-looked-up for Sun/Moon/ISS/planets)
        and targets it exactly like a viz double-click - see _select_sky_target."""
        if entry["live"] is not None:
            ra, dec = self._resolve_live_body(entry["live"])
            if ra is None:
                if entry["live"] == "ISS":
                    self._log("ISS position isn't available - enable its tracking toggle first.")
                else:
                    self._log(f"{entry['live']} position isn't available yet - still fetching, try again shortly.")
                return
        else:
            ra, dec = entry["ra"], entry["dec"]
        self.find_edit.setText("")
        self.sky_search_results.clear()
        self.sky_search_results.hide()
        self._select_sky_target(ra, dec, f"Search: {entry['name']}", target_name=entry["name"])

    def _parse_target_inputs(self):
        try:
            return float(self.ra_edit.text()), float(self.dec_edit.text())
        except ValueError:
            self._log("Invalid RA/DEC - enter decimal degrees.")
            return None

    def _is_target_risky(self, ra, dec):
        """Client-side estimate mirroring the firmware's pastMeridianLimit() - see
        EQMountApp._is_target_risky's docstring. Only decides whether to show a confirmation
        dialog; the firmware re-checks this independently and authoritatively regardless."""
        if MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG <= dec <= MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG:
            return False
        lst = self.viz.current_lst_deg()
        ha = (lst - ra + 180.0) % 360.0 - 180.0
        return (ha <= 0.0) if self.viz.telescope_flipped else (ha >= 0.0)

    def _confirm_risky_slew(self, ra, dec):
        if not self._is_target_risky(ra, dec):
            return True
        # Same reasoning as the solar-tracking confirm's explicit defaultButton=No - a collision
        # risk shouldn't be the thing an accidental Enter press confirms.
        return QMessageBox.question(
            self, "Meridian Limit Warning",
            f"Target RA={ra:.3f}° DEC={dec:.3f}° is past the meridian limit for the mount's "
            f"current configuration - continuing risks the OTA colliding with the tripod/mount.\n\n"
            f"Proceed anyway?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        ) == QMessageBox.Yes

    def _update_meridian_warning(self):
        """Client-side ESTIMATE of time remaining until the meridian limit stops tracking - see
        EQMountApp._update_meridian_warning's docstring. Purely informational; the firmware
        enforces the actual stop independently."""
        mode = self.mode_group.checkedButton().text() if self.mode_group.checkedButton() else "SIDEREAL"
        if not self.tracking or self.viz.telescope_flipped:
            self.meridian_warning_label.setText("")
            return
        lst = self.viz.current_lst_deg()
        ha = (lst - self.target_ra + 180.0) % 360.0 - 180.0
        time_to_limit_s = -ha / SIDEREAL_RATE_DEG_S
        if time_to_limit_s > MERIDIAN_LIMIT_WARNING_S or time_to_limit_s < 0:
            self.meridian_warning_label.setText("")
            return
        minutes, seconds = divmod(int(time_to_limit_s), 60)
        urgent = time_to_limit_s <= MERIDIAN_LIMIT_URGENT_S
        self.meridian_warning_label.setText(f"⚠ MERIDIAN LIMIT IN {minutes}m {seconds:02d}s - flip may be needed")
        self.meridian_warning_label.setStyleSheet(f"color: {PALETTE['red'] if urgent else PALETTE['orange']}; font-weight: bold;")

    def _goto(self):
        parsed = self._parse_target_inputs()
        if parsed is None:
            return
        ra, dec = parsed
        if not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        risky = self._is_target_risky(ra, dec)
        if not self._confirm_risky_slew(ra, dec):
            self._log("GoTo cancelled - target past the meridian limit, not confirmed.")
            return
        # Same staleness check as _toggle_tracking's SIDEREAL branch: if the RA/DEC boxes no
        # longer match whatever _select_sky_target last wrote there, this Goto is to a manually
        # edited point, not the previously named target - clear the badge instead of leaving a
        # name showing next to coordinates it no longer actually corresponds to.
        if self.ra_edit.text().strip() != self._target_name_box_ra_str or \
                self.dec_edit.text().strip() != self._target_name_box_dec_str:
            self.target_object_name = None
            self._update_target_name_label()
        self.target_ra, self.target_dec = ra, dec
        self.viz.target_ra, self.viz.target_dec = ra, dec
        self.viz.update()
        self.serial.send_goto(ra, dec, risk_ok=risky)
        self._log(f"Goto RA {ra:.4f} DEC {dec:.4f}")

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
        self._force_stop_tracking()
        self.serial.send_safe_target()
        self._log("Safe Target sent: sky DEC -> 0°, RA unchanged, no tracking.")

    def _home_axes(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        self._force_stop_tracking()
        self.serial.send_home_axes()
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
        self._style_toggle_btn(self.debug_toggle_btn, self.arduino_debug_enabled)
        self._log(f"Sent CMD,DEBUG,{'ON' if self.arduino_debug_enabled else 'OFF'}")

    def _toggle_telescope_flipped(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        requested = not self.viz.telescope_flipped
        self.serial.send_command(f"CMD,SET_FLIPPED,{1 if requested else 0}")
        self._log(f"Requested telescope flip: {'ON' if requested else 'OFF'} "
                  f"(RA will rotate ~180° - watch for STATUS:FLIP_COMPLETE)")

    def _on_telescope_flipped_confirmed(self, flipped):
        if flipped == self.viz.telescope_flipped:
            return
        self.viz.telescope_flipped = flipped
        self.flipped_btn.setText(f"Telescope Flipped: {'ON' if flipped else 'OFF'}")
        self.flipped_btn.setStyleSheet(f"background-color: {PALETTE['orange']}; color: black; font-weight: bold;" if flipped else "")
        self.viz.update()  # the meridian-limit shading only draws while not flipped

    def _toggle_tracking(self):
        """Ported from EQMountApp._toggle_tracking: SOLAR gets a safety confirmation dialog,
        SIDEREAL/SOLAR/LUNAR each resolve their own target (input boxes vs. live Sun/Moon
        position) and send it via CMD,SET_TARGET before starting, and a meridian-risk dialog
        gates the actual start the same way _goto's does."""
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        if self.tracking:
            self._request_tracking_action("STOP", self.serial.send_stop, "Stop Tracking button", "STOPPING...")
            return

        mode = self.mode_group.checkedButton().text() if self.mode_group.checkedButton() else "SIDEREAL"
        if mode == "SOLAR":
            # Explicit buttons + defaultButton=No - QMessageBox.question() with neither
            # specified defaults its Enter-activated button to Yes, which is the wrong default
            # for a "did you actually attach the solar filter" safety gate: an accidental/
            # reflexive Enter press should never be the thing that confirms this.
            if QMessageBox.question(
                self, "Solar Tracking Safety Check",
                "Solar tracking will point the telescope directly at the Sun.\n\n"
                "Confirm a proper solar filter is attached to the telescope/camera BEFORE "
                "proceeding - without one, this can cause permanent eye damage or destroy a "
                "camera sensor.\n\nIs the solar filter in place?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
            ) != QMessageBox.Yes:
                self._log("Solar tracking start cancelled - solar filter not confirmed.")
                return

        if mode == "SIDEREAL":
            parsed = self._parse_target_inputs()
            if parsed is None:
                return
            ra, dec = parsed
            if self.ra_edit.text().strip() != self._target_name_box_ra_str or \
                    self.dec_edit.text().strip() != self._target_name_box_dec_str:
                self.target_object_name = None
                self._update_target_name_label()
            self.target_ra, self.target_dec = ra, dec
            self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f}")
        else:
            ra, dec = (self.viz.sun_ra, self.viz.sun_dec) if mode == "SOLAR" else (self.viz.moon_ra, self.viz.moon_dec)
            if ra is None:
                self._log(f"{mode.title()} position not available yet (still loading skyfield ephemeris?) - "
                          f"using previous sync/target values.")
            else:
                self.target_object_name = "Sun" if mode == "SOLAR" else "Moon"
                self._update_target_name_label()
                self.target_ra, self.target_dec = ra, dec
                self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f}")

        risky = self._is_target_risky(self.target_ra, self.target_dec)
        if not self._confirm_risky_slew(self.target_ra, self.target_dec):
            self._log("Start Tracking cancelled - target past the meridian limit, not confirmed.")
            return

        if self._force_full_align_next_start:
            self._force_full_align_next_start = False
            self._request_tracking_action("START", lambda: self.serial.send_start_tracking(risk_ok=risky),
                                          "normal alignment, forced after mode change", "STARTING...")
        else:
            distance = self._calc_angular_distance(self.current_ra, self.current_dec,
                                                    self.target_ra, self.target_dec)
            if distance <= 5.0:
                self._request_tracking_action(
                    "START", lambda: self.serial.send_start_tracking_skip_dec_reset(risk_ok=risky),
                    f"DEC reset skip, target within {distance:.2f}°", "STARTING...")
            else:
                self._request_tracking_action("START", lambda: self.serial.send_start_tracking(risk_ok=risky),
                                              f"normal alignment, target {distance:.2f}° away", "STARTING...")

    @staticmethod
    def _calc_angular_distance(ra1, dec1, ra2, dec2):
        ra1_r, dec1_r, ra2_r, dec2_r = map(math.radians, (ra1, dec1, ra2, dec2))
        d_ra, d_dec = ra2_r - ra1_r, dec2_r - dec1_r
        a = math.sin(d_dec / 2) ** 2 + math.cos(dec1_r) * math.cos(dec2_r) * math.sin(d_ra / 2) ** 2
        return math.degrees(2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

    def _force_stop_tracking(self):
        if self.tracking:
            self._request_tracking_action("STOP", self.serial.send_stop, "mode change", "STOPPING (mode change)...")
            self.slewing = False

    def _request_tracking_action(self, action, send_fn, reason, status_text):
        """Ported from EQMountApp._request_tracking_action: sends START/STOP and keeps
        resending on a timer until the Arduino actually confirms it (see _parse_arduino_line's
        DEBUG:CMD_ACTION:START_TRACKING / STATUS:STOPPED handling), instead of assuming the
        first attempt worked. Starting a new action cancels whatever retry was still pending
        from a previous one, so a stale START can't keep resending after a STOP click."""
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        self._pending_action_timer.stop()
        try:
            self._pending_action_timer.timeout.disconnect()
        except (TypeError, RuntimeError):
            pass
        self._pending_action = action
        self._pending_action_confirmed = False
        self._pending_action_retry_count = 0
        self.tracking = (action == "START")
        self._align_phase = "ALIGNING" if action == "START" else "IDLE"
        # Real bug, not just cosmetic: these were never cleared, so the STABLE/SETTLING/DRIFTING
        # trend calc (_update_tracking_stability) kept blending old error samples from BEFORE
        # this start (the previous session's tracking, or this session's own alignment slew,
        # where error is large and changing fast) into its rolling TRACKING_STABILITY_WINDOW_S
        # window for up to that long after a fresh Start - a genuinely stable new tracking
        # session could read as SETTLING/DRIFTING for that whole stretch, since the trend was
        # computed over old-large + new-small data mixed together, not the new data alone.
        # Reported as "tracking sometimes needs a stop+restart to show as stable" - restarting
        # didn't actually fix anything by itself; it just happened to be far enough after the
        # poisoned window that it had already aged out on its own by the time of the retry.
        # Clearing here (both START and STOP) means every fresh tracking session's trend is
        # judged purely on its own data from the start.
        self.err_ra_history.clear()
        self.err_dec_history.clear()
        self._update_mode_tracking_label()
        self._update_status_display(status_text, PALETTE["orange"])
        ok = send_fn()
        self._log(f"Sent {action} ({reason})" if ok else f"{action} write failed ({reason}) - will retry")
        retry_ms = 150 if action == "STOP" else 800
        self._pending_action_timer.timeout.connect(lambda: self._verify_pending_action(action, send_fn))
        self._pending_action_timer.start(retry_ms)

    def _verify_pending_action(self, action, send_fn):
        if self._pending_action != action or self._pending_action_confirmed:
            return
        if not (self.serial.ser and self.serial.ser.is_open):
            return
        retry_ms = 150 if action == "STOP" else 800
        max_retries = 20 if action == "STOP" else 3
        if self._pending_action_retry_count >= max_retries:
            self._log(f"WARNING: Arduino never confirmed {action} after retries - check the connection/mount.")
            self._update_status_display(f"{action} NOT CONFIRMED - CHECK CONNECTION", PALETTE["red"])
            return
        self._pending_action_retry_count += 1
        ok = send_fn()
        self._log(f"{action} not yet confirmed, resending (attempt {self._pending_action_retry_count + 1})" if ok
                  else f"{action} resend also failed to write")
        self._pending_action_timer.start(retry_ms)

    def _update_target_name_label(self):
        self.target_name_label.setText(self.target_object_name or "")
        # SkyViewWidget.target_object_name (default None, never otherwise assigned) is what
        # _draw_camera_and_reticle's center-reference-circle sizing keys off of - without this,
        # getattr(self, "target_object_name", None) there always fell back to None (never
        # actually matching any real hit's name), so the circle was permanently stuck at its
        # fixed default size instead of ever tracking the real target's on-screen size.
        self.viz.target_object_name = self.target_object_name

    def _select_sky_target(self, ra, dec, source_label, target_name=None):
        """Ported from EQMountApp._select_sky_target, plus one addition: selecting the Sun or
        Moon specifically (double-click or search) now switches Tracking Mode to SOLAR/LUNAR
        first, so Start Tracking actually follows the real body (and SOLAR still gets its filter-
        confirmation dialog via _toggle_tracking) instead of being forced into a fixed SIDEREAL
        point at wherever the Sun/Moon happened to be at selection time. Any other selection
        (star/DSO/ISS/planet/manual point) still goes through SIDEREAL as before - a double-
        click/search result isn't a meaningful way to pick a *different* body to follow.

        A double-click or search selection always fills the target fields; if tracking is
        already active that's all it does (must not interrupt a running session)."""
        self.target_object_name = target_name
        self._update_target_name_label()
        ra_str, dec_str = f"{ra:.4f}", f"{dec:.4f}"
        self._target_name_box_ra_str, self._target_name_box_dec_str = ra_str, dec_str
        self.ra_edit.setText(ra_str)
        self.dec_edit.setText(dec_str)

        if self.tracking:
            self._log(f"{source_label}: placed in target fields (tracking already active - not slewing)")
            return
        if not (self.serial.ser and self.serial.ser.is_open):
            self._log(f"{source_label}: RA/DEC placed in target fields (not connected, so not starting tracking).")
            return

        self.target_ra, self.target_dec = ra, dec
        mode_index = {"Sun": 1, "Moon": 2}.get(target_name, 0)  # SIDEREAL=0, SOLAR=1, LUNAR=2
        mode_name = self.mode_group.button(mode_index).text()
        if self.mode_group.checkedButton() is not self.mode_group.button(mode_index):
            self.mode_group.button(mode_index).setChecked(True)
            self._on_mode_changed(mode_index)
        self._log(f"{source_label}: starting {mode_name.lower()} tracking there")
        self._toggle_tracking()
        if target_name == "ISS" and self.tracking:
            if not self.viz.iss_enabled:
                self.viz.iss_enabled = True
                self.iss_btn.setText("ISS: ON")
            # Kick off the fast ISS_TRACKING_UPDATE_MS refresh cadence immediately (see
            # _trigger_iss_update/_iss_actively_tracked) instead of waiting up to 5s for
            # whatever slow-interval tick happened to already be scheduled.
            self._trigger_iss_update()

    def _stop(self):
        """The standalone "STOP (panic)" button - not in EQMountApp's UI (there tracking
        start/stop is a single toggle button, plus the global Delete/Suppr key shortcut - see
        _keyPressEvent) - kept here as an always-visible panic stop, routed through the same
        confirm-retry machinery as the Delete key would use."""
        if self.serial.ser and self.serial.ser.is_open:
            self._request_tracking_action("STOP", self.serial.send_stop, "STOP (panic) button", "STOPPING...")
        else:
            self._log("Not connected.")

    def _on_hover_info(self, text):
        self.hover_label.setText(text)

    def _set_stability(self, text, color, text_color="black"):
        self.tracking_status_label.setText(text)
        self.tracking_status_label.setStyleSheet(
            f"background-color: {color}; color: {text_color}; font-weight: bold; font-size: 14px; padding: 5px; border-radius: 8px;")
        self.viz.stability_color = QColor(color)
        self.viz.update()

    def _update_tracking_stability(self):
        """Ported from EQMountApp._update_tracking_stability: classifies tracking as
        STABLE/SETTLING/DRIFTING from the *trend* of the error (not just its instantaneous
        size) - a large-but-shrinking error is fine, a small-but-growing one is an early
        warning, which a plain magnitude threshold would miss either way.

        Badge fill colors all come from PALETTE (see its module docstring) - black text on
        those (rather than white) since the palette is deliberately light/pastel, not the dark
        saturated fills white text would need. TRACKING: OFF keeps its own neutral gray with
        white text - gray is exempt from the uniform accent palette the same way backgrounds
        are, and it's dark enough for white to read fine."""
        if self._align_phase == "ALIGNING":
            self._set_stability("ALIGNING", PALETTE["cyan"])
            return
        if not self.tracking or self._align_phase != "TRACKING":
            self._set_stability("OFF", "#444455", text_color="white")
            return
        if len(self.err_ra_history) < 4 or len(self.err_dec_history) < 4:
            self._set_stability("SETTLING", PALETTE["orange"])
            return
        deriv_ra = self._calc_smoothed_speed(self.err_ra_history, window_s=TRACKING_STABILITY_WINDOW_S)
        deriv_dec = self._calc_smoothed_speed(self.err_dec_history, window_s=TRACKING_STABILITY_WINDOW_S)
        worst_err = max(abs(self.err_ra), abs(self.err_dec))
        worst_growth = max(deriv_ra, deriv_dec)
        if worst_err <= TRACKING_STABLE_ERR_DEG and worst_growth <= TRACKING_STABLE_DERIV_DEG_S:
            self._set_stability("STABLE", PALETTE["green"])
        elif worst_growth > TRACKING_STABLE_DERIV_DEG_S:
            self._set_stability("DRIFTING", PALETTE["red"])
        else:
            self._set_stability("● SETTLING", PALETTE["orange"])

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
            self.conn_status.setStyleSheet(f"color: {PALETTE['green']};")
            self.arduino_debug_enabled = False
            self.debug_toggle_btn.setText("Verbose Debug: OFF")
            self._style_toggle_btn(self.debug_toggle_btn, False)
            self._update_status_display("CONNECTED - WAITING FOR PING RESPONSE", PALETTE["orange"])
            self._log(f"Connected to {port}. Waiting for Arduino response...")
        elif msg.startswith("RX:STATUS:PONG"):
            self._connection_timeout_timer.stop()
            self._update_status_display("CONNECTED - READY", PALETTE["green"])
            self._log("Arduino responded to PING - connection fully established")
        elif msg.startswith("DISCONNECTED"):
            self.conn_status.setText("● Disconnected")
            self.conn_status.setStyleSheet("color: gray;")
            self.tracking = False
            self.slewing = False
            self.arduino_debug_enabled = False
            self.debug_toggle_btn.setText("Verbose Debug: OFF")
            self._style_toggle_btn(self.debug_toggle_btn, False)
            self._update_mode_tracking_label()
            self._update_status_display("DISCONNECTED", PALETTE["red"])
            self._log("Disconnected")
        elif msg.startswith("RX:"):
            self._parse_arduino_line(msg[3:])
        elif msg.startswith("TX:"):
            if "SYNC" in msg or "GOTO" in msg:
                self._log(msg)
        elif msg.startswith("ERROR:"):
            self._log("ERROR: " + msg[6:])
            self.error_label.setText("Error: " + msg[6:])

    def _update_status_display(self, text, color=PALETTE["orange"]):
        self.status_display.setText(text)
        self.status_display.setStyleSheet(
            f"background-color: #1a1a2e; color: {color}; font-weight: bold; font-size: 14px; padding: 6px;")

    def _parse_arduino_line(self, line: str):
        line = line.strip()
        if not line:
            return

        if line.startswith("DEBUG:"):
            self._log(line)
            # Earliest possible proof the Arduino received a START - see
            # EQMountApp._parse_arduino_line's comment on this same check.
            if "CMD_ACTION:START_TRACKING" in line and self._pending_action == "START":
                self._pending_action_confirmed = True

        if line.startswith("STATUS:"):
            if not line.startswith("STATUS:WAITING"):
                self._log(line)
            self.last_status = line
            important = ["TRACKING", "ALIGNING", "RESETTING", "WAITING", "STOPPED", "SYNCED",
                        "ERROR", "SLEW", "FLIPPING", "FLIP_COMPLETE", "AUTO_CALIBRATED"]
            if any(kw in line for kw in important):
                status_text = line.replace("STATUS:", "STATUS: ")
                color = PALETTE["cyan"]
                if "MERIDIAN_LIMIT" in line and "STOPPED" in line:
                    status_text = ("STOPPED - MERIDIAN LIMIT REACHED - flip the telescope in the "
                                  "clamps, then press Telescope Flipped ON to resume")
                    color = PALETTE["red"]
                    self.tracking = False
                    self._align_phase = "IDLE"
                elif "TRACKING_STARTED" in line or "TRACKING" in line:
                    color = PALETTE["green"]
                    self.tracking = True
                    if "AXIS:RA" not in line:
                        self._align_phase = "TRACKING"
                elif "ALIGNING" in line or "RESETTING" in line or "WAITING" in line:
                    color = PALETTE["orange"]
                    self._align_phase = "ALIGNING"
                elif "FLIPPING" in line:
                    color = PALETTE["orange"]
                elif "FLIP_COMPLETE" in line:
                    color = PALETTE["cyan"]
                elif "STOPPED" in line or "ERROR" in line:
                    color = PALETTE["red"]
                    self.tracking = False
                    self._align_phase = "IDLE"
                    if "STOPPED" in line and self._pending_action == "STOP":
                        self._pending_action_confirmed = True
                self._update_status_display(status_text, color)
                self._update_mode_tracking_label()

            m = re.search(r"UPDATE_RATE.*?MS:(\d+)", line)
            if m:
                self.pos_rate_label.setText(f"Arduino POS Update Rate: confirmed {m.group(1)} ms")
            return

        if line.startswith("POS,"):
            # POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking,flipped - see
            # the matching comment above sendPositionUpdate() in the .ino / tracker_gui.py's
            # _parse_arduino_line for the full field-by-field breakdown.
            fields = line.split(",")
            try:
                self.current_ra = float(fields[1])
                self.current_dec = float(fields[2])
                # Skip while actively tracking the ISS - _on_iss_updated already keeps
                # target_ra/dec fresh from the GUI's own live skyfield computation (the same
                # value actually being sent as CONTINUATION updates), up to 50Hz. This field is
                # just the Arduino's ECHO of whatever target it last received, lagged by real
                # serial round-trip time - overwriting the fresher client-side value with that
                # stale echo on every single POS line (also up to 50Hz, on an unsynchronized
                # clock relative to the ISS thread) made target_ra/dec - and so the View: Target
                # viz center - visibly flicker between the fresh and stale value. Reported as
                # "the viz sometimes jumps when in View: Target" while tracking the ISS.
                if not self._iss_actively_tracked():
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

                cutoff2 = now - TRACKING_STABILITY_WINDOW_S - 0.5
                self.err_ra_history.append((self.err_ra, now))
                while self.err_ra_history and self.err_ra_history[0][1] < cutoff2:
                    self.err_ra_history.popleft()
                self.err_dec_history.append((self.err_dec, now))
                while self.err_dec_history and self.err_dec_history[0][1] < cutoff2:
                    self.err_dec_history.popleft()
            except (ValueError, TypeError, IndexError):
                return
            if len(fields) > 8:
                pos_track = fields[8].strip() == "1"
                if pos_track and not self.tracking:
                    self.tracking = True
                    self._update_mode_tracking_label()
            if len(fields) > 9:
                try:
                    self._on_telescope_flipped_confirmed(fields[9].strip() == "1")
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
                self.sky_ra_label.setText(f"{self.current_ra:07.4f}°")
                self.sky_dec_label.setText(f"{self.current_dec:+08.4f}°")
                self.mount_ra_label.setText(f"{self.mount_ra_angle:.4f}°")
                self.mount_dec_label.setText(f"{self.mount_dec_angle:.4f}°")
                self.speed_ra_label.setText(f"{self.speed_ra:.6f} °/s")
                self.speed_dec_label.setText(f"{self.speed_dec:.6f} °/s")
                self.error_label.setText(f"Error: RA {self.err_ra:.4f}° | DEC {self.err_dec:.4f}°")
                self.live_error_label.setText(f"Live Error: RA {self.err_ra:.4f}° | DEC {self.err_dec:.4f}°")
                self._update_mode_tracking_label()
                self._update_tracking_stability()
                self.viz.update()
            return

        if "LOCATION_SET" in line or "SYNCED" in line or "TIME_SET" in line:
            self._log(line)

    def eventFilter(self, obj, event):
        """Find-box <-> results-list keyboard navigation (installed on both widgets in
        _build_ui): Down in the search box jumps into the results list with the top item
        selected; Up on the list's top item jumps back to the search box (cursor at the end, so
        typing continues naturally) instead of just doing nothing the way a plain QListWidget
        would at row 0. Down within the list otherwise still gets Qt's normal list navigation -
        only Up-at-the-top and Down-from-the-box are intercepted here."""
        if event.type() == QEvent.KeyPress:
            if obj is self.find_edit and event.key() == Qt.Key_Down:
                if self.sky_search_results.count() > 0:
                    self.sky_search_results.setFocus()
                    self.sky_search_results.setCurrentRow(0)
                    return True
            elif obj is self.sky_search_results and event.key() == Qt.Key_Up:
                if self.sky_search_results.currentRow() <= 0:
                    self.find_edit.setFocus()
                    self.find_edit.setCursorPosition(len(self.find_edit.text()))
                    return True
        return super().eventFilter(obj, event)

    # ---------------- keyboard shortcuts (ported from EQMountApp) ----------------
    def keyPressEvent(self, event):
        """Delete/Backspace: emergency stop, unconditionally (even if the GUI's tracking flag
        happens to be stale). Enter/Return: start tracking, UNLESS focus is in a text entry or
        the search results list (each of those already has its own Enter handling) - mirrors
        EQMountApp._stop_tracking_key/_start_tracking_key.

        The linked button (stop_btn / tracking_btn) is held visually pressed for as long as the
        physical key is - setDown(True) here, setDown(False) in keyReleaseEvent - not
        animateClick()'s fixed ~100ms auto-release, which unpressed the button on its own timer
        regardless of whether the key was still actually held down. isAutoRepeat() presses are
        ignored (OS key-repeat while held would otherwise re-fire the action many times a
        second) - the button just stays down through those, same as it would under a real
        continuously-held mouse click."""
        focus_widget = self.focusWidget()
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace) and not isinstance(focus_widget, QLineEdit):
            if not event.isAutoRepeat():
                self.stop_btn.setDown(True)
                self._stop()
            return
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            if isinstance(focus_widget, (QLineEdit, QListWidget)):
                super().keyPressEvent(event)
                return
            if not event.isAutoRepeat() and not self.tracking:
                self.tracking_btn.setDown(True)
                self._toggle_tracking()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace):
            self.stop_btn.setDown(False)
        elif event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self.tracking_btn.setDown(False)
        else:
            super().keyReleaseEvent(event)


# Default Fusion-style QGroupBox borders render nearly invisibly against this app's dark
# palette (thin, low-contrast line) - every panel (Connection, Location, Telescope Position,
# Tracking Controls, etc.) needs a clearly visible border to actually read as a distinct,
# delimited block instead of text floating with no boundary.
_APP_STYLESHEET = """
QGroupBox {
    border: 1px solid #4a4a5a;
    border-radius: 5px;
    margin-top: 10px;
    padding-top: 6px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
"""


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(_APP_STYLESHEET)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
EQ Mount Tracker - shared backend

Toolkit-agnostic infrastructure used by tracker_gui.py (the GUI, PySide6/Qt): serial protocol
constants, viz/tracking-math constants, the galactic-to-equatorial coordinate helper, and
SerialHandler (the background serial I/O thread). Nothing in this file imports Qt, Tkinter, or
any other GUI toolkit - it has no window/widget of its own.

History: until the GUI was ported from Tkinter/CustomTkinter to Qt, this module WAS
tracker_gui.py itself - the Tkinter EQMountApp class lived directly below everything in this
file, and the Qt GUI (then tracker_gui_qt.py) imported this shared portion from it. Once the Qt
GUI reached full feature parity, the Tkinter GUI was discarded, tracker_gui_qt.py took over the
tracker_gui.py name, and this shared portion was split out here (under its former filename) so
the new tracker_gui.py doesn't import from itself.
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
import math
import os
from datetime import datetime, timezone
from typing import Optional

# ============================================================
# CONFIG
# ============================================================
# Bumped on every functional change, same convention as (and independent of) the Arduino's own
# FIRMWARE_VERSION in EQMountTracker.ino - see CLAUDE.md's commit workflow. Not sent over serial
# or otherwise load-bearing; purely a changelog anchor so "what did the GUI look like when X was
# true" is answerable from git history without guessing at commit dates.
#   1.0.0 - Baseline (first version tracked here).
#   1.0.1 - Time Travel simplified to one single time source: SerialHandler now pulls "now" via
#           a get_time_fn callback (EQMountApp._get_effective_utc_now) instead of a separately
#           mirrored offset, and the ISS (iss_tracker.get_current_radec) now takes at_time too -
#           every position lookup in the app (Sun, Moon, planets, stars/DSOs via LST, ISS, and
#           the Arduino's own clock sync) reads from that one function, so a Time Travel preview
#           genuinely covers everything, not just a subset. No Arduino firmware changes needed -
#           the simulated time is just sent as an ordinary CMD,SET_TIME.
#   1.0.2 - Sync/Calibrate dropdown gained "East"/"West" horizon (DEC=0) calibration points,
#           above the existing named stars - useful for re-syncing against the horizon itself
#           (e.g. after a manual re-home) without needing a visible star. Unlike the named
#           stars, their RA isn't a fixed J2000 value (it depends on the current LST), so
#           they're resolved at sync time in _sync_position() using the same Hour Angle
#           convention as autoCalibrateFromHome() in the .ino (East=LST+90, West=LST-90).
#   1.0.3 - Two real bugs found testing Time Travel against a known solar eclipse:
#           (1) Time Travel's date/time fields were interpreted in the computer's LOCAL
#           timezone before converting to UTC - but published astronomical event times (like
#           eclipse totality) are always UTC, so typing the real published time in directly got
#           silently shifted by the local UTC offset (e.g. 2h for CEST). Now interpreted as UTC
#           directly, no conversion, fields/labels relabeled to say so explicitly.
#           (2) _schedule_viz_redraw() (the Sun/Moon/DSO marker refresh path) used
#           self.after_idle(), which only runs once Tk's event queue is genuinely empty - while
#           actively tracking, continuous POS traffic (~50-100Hz) can keep the queue busy long
#           enough to starve it indefinitely, so the Sun/Moon dot visibly froze during tracking
#           even though the underlying position (_solar_system_update_tick, every 2s) was
#           updating correctly the whole time - only "caught up" once tracking stopped and the
#           queue finally idled. Switched to self.after(0, ...), which isn't idle-gated.
#           _on_canvas_resize's after_idle had the same latent risk, fixed the same way.
#   1.0.4 - Sky viz canvas was laggy enough while zooming/panning to make the whole GUI look like
#           it froze/reloaded - every zoom/pan/follow tick rebuilt the ENTIRE background (grid,
#           Milky Way, horizon, meridian-limit shading, up to tens of thousands of catalog
#           stars/DSOs) as that many individual real Tk canvas items via delete("all") + a
#           create_line/create_oval/create_polygon/create_text per item, each one its own Tcl
#           call - the actual bottleneck, not the drawing math itself. That background is now
#           rasterized into a single Pillow image and blitted as ONE Tk canvas item per redraw
#           (see the new _RasterCanvas class) - the handful of items that genuinely need cheap
#           per-tick movement (camera FOV rect/marker, target reticle - see
#           _update_visualization) are unaffected, still real Tk items moved via c.coords().
#           Two more redraw-cost fixes found profiling this: (1) DSOs were always drawn via a
#           28-point rotated-ellipse polygon even when clamped to their ~3-4px on-screen minimum
#           size (the overwhelmingly common case at low zoom, where a rotated/dashed outline is
#           indistinguishable from a plain dot anyway) - now a plain dot below 6px. (2) Pillow was
#           re-parsing the same handful of hex color strings via a regex-based parser on every
#           single one of tens of thousands of draw calls - now cached (_pil_ink). New dependency:
#           Pillow (added to requirements.txt).
#   1.0.5 - ISS_TRACKING_UPDATE_MS lowered from 20ms (50Hz) to 50ms (20Hz) per request - the
#           firmware-side rate-derivation/extrapolation (see this constant's own comment) already
#           made raw update frequency far less important for smoothness than it used to be, so
#           20Hz is still plenty fresh against the ISS's real path.
#   1.0.6 - Added ISS_DISPLAY_UPDATE_MS (1Hz) as a distinct, slower refresh rate for when the ISS
#           is merely shown on the sky viz (not the actively tracked target) - previously this
#           case used a hardcoded 5000ms; now explicit and 1Hz per request.
#   1.0.7 - Fixed the remaining half of "tracking sometimes needs stop+restart to show stable":
#           an earlier fix cleared _err_ra_history/_err_dec_history on the initial Start click,
#           but the history kept accumulating unconditionally on every POS line straight through
#           the ALIGNING slew that follows, re-poisoning the window with slew-noise error samples
#           before TRACKING phase even began. Now also cleared at the true ALIGNING->TRACKING
#           transition (the final, non-AXIS:RA TRACKING_STARTED line), so the stability trend is
#           judged only on real tracking data.
#   1.0.8 - Added a Meridian Limit ON/OFF toggle (new meridian_limit_btn), mirroring the
#           firmware 1.8.52 CMD,SET_MERIDIAN_LIMIT toggle - turning it OFF requires an explicit
#           confirmation dialog every time, same pattern as the solar tracking safety gate.
#           Re-sent to the Arduino on every connect (_apply_initial_meridian_limit) and
#           reconciled against firmware-confirmed state via STATUS:MERIDIAN_LIMIT_ENABLED.
#   1.0.9 - Renamed the "Home Axes (RA/DEC to 0°)" button/log text to "Rewind Axes (RA/DEC to
#           0°)" per request - cosmetic label only, command (CMD,HOME_AXES)/handler names unchanged.
#   1.0.10 - Buttons gated on being connected (_set_arduino_controls_enabled) now visibly gray
#            out while disconnected instead of staying fully colored despite being unusable -
#            new shared _muted_hex_color() blends each button's own color toward neutral gray
#            (imported by tracker_gui_qt.py too, same math both sides) so the disabled look still
#            hints the original color rather than looking identical to every other grayed
#            control. New _apply_disabled_tint() applied in _set_arduino_controls_enabled;
#            flipped_toggle_btn/meridian_limit_btn were also missing from that gating list
#            entirely (could be clicked while disconnected, just logging "Not connected.") - now
#            included. Per explicit exception, the emergency STOP button is untouched by any of
#            this (Qt-only widget; stays fully colored/usable-looking regardless of connection,
#            matching ISO 13850 convention).
#   1.0.11 - ISS position updates (both display-only ISS_DISPLAY_UPDATE_MS and actively-tracking
#            ISS_TRACKING_UPDATE_MS) now sync to the wall clock - each reschedule computes the
#            delay to the next exact multiple of the interval since the epoch (same self-
#            rescheduling-to-the-boundary technique as _update_realtime_dot), instead of a
#            free-running fixed delay from whenever the previous fetch happened to finish, so
#            updates land on a predictable cadence (e.g. every real :00/:01/:02 second at 1Hz)
#            rather than drifting to an arbitrary phase. Both GUIs.
#   1.0.12 - Fixed the target reticle/View: Follow Target/live error readout freezing at wherever
#            a live-moving target (ISS, Sun, Moon, a planet) was the moment tracking stopped,
#            instead of continuing to follow its real current position - target_ra/dec updates
#            from _on_iss_position/_on_solar_system_position were gated on self.tracking, and the
#            POS handler unconditionally overwrote target_ra/dec with the firmware's echo (frozen
#            once there are no more fresh SET_TARGETs for it to reflect) the rest of the time. New
#            _target_is_live_body() gates both; target_ra/dec now stays live-updated for these
#            bodies purely from the GUI's own computation whenever one is the SELECTED target,
#            tracking or not - sending a fresh SET_TARGET to the Arduino remains gated on tracking
#            as before. tracker_gui_qt.py additionally gained a recurring 2s Sun/Moon/planet
#            refresh (solar_system_timer) - it was previously only ever triggered once at startup
#            and on a Time Travel jump, so this fix would have had nothing fresh to follow for
#            those bodies without it.
#   1.0.13 - Sun/Moon/planet refresh rate raised from 2000ms to 1Hz (new shared
#            SOLAR_SYSTEM_UPDATE_MS constant, replacing the previous hardcoded 2000/2000ms in
#            _solar_system_update_tick/solar_system_timer) per request - matches
#            ISS_DISPLAY_UPDATE_MS's cadence. Both GUIs.
#   1.0.14 - The Tkinter/CustomTkinter GUI (formerly tracker_gui.py's EQMountApp) is discarded
#            now that the Qt GUI has full feature parity - it was kept alongside the Qt GUI
#            purely so the two could be compared during development. tracker_gui_qt.py takes
#            over the tracker_gui.py name/launcher slot; this shared module (serial protocol,
#            viz/tracking-math constants, SerialHandler) is split out under the file's former
#            name, tracker_shared.py, so the new tracker_gui.py doesn't import from itself.
#            requirements.txt drops customtkinter/Pillow (no longer used by anything).
GUI_VERSION = "1.0.14"

BAUD_RATE = 250000
# GUI poll rate for the serial queue. Fast enough to comfortably keep up with the Arduino's 50Hz
# (20ms) POS broadcast rate (POSITION_BROADCAST_HZ below) with headroom to spare, per request to
# make the telescope position update as fast as possible - _poll_queue's drain loop is cheap when
# the queue is empty, so polling faster than strictly needed doesn't cost much.
POLL_INTERVAL_MS = 10
# Caps how often a POS line triggers real Tkinter UI work (redraw/labels/error), independent of
# how fast POS lines actually arrive from the Arduino (POSITION_BROADCAST_HZ below). Without
# this, raising POSITION_BROADCAST_HZ well above this makes _poll_queue drain several queued POS
# lines per tick and run the full UI-update battery for each one back to back, which is what
# actually froze/lagged the GUI - not the firmware/motors. The underlying data (self.current_ra,
# mount angles, speed history, etc.) is still updated on every POS line either way; only the
# expensive widget/canvas calls are throttled.
# (A version of this tied to the OS-reported screen refresh rate was tried and reverted - it made
# things worse, not better - so this is a plain fixed constant instead.)
# Matches POSITION_BROADCAST_HZ exactly - no point redrawing faster than fresh data actually
# arrives (per request, this and POLL_INTERVAL_MS above are now the two things that determine how
# fast the telescope marker updates, so both are set to the practical maximum).
POS_UI_UPDATE_MIN_INTERVAL_S = 1.0 / 50.0
# Arduino POS broadcast rate - fixed, not user-adjustable (used to be a configurable rate +
# presets in the UI; removed since a too-high rate was part of the original vibration/jitter
# investigation - see the 1.8.7-1.8.9 firmware changelog entries).
# Temporarily raised 5 -> 100 Hz to test whether firmware 1.8.24 (StepGen interrupt-driven
# stepping + finite fine-acceleration) is still affected by broadcast rate the way the old
# AccelStepper-polled firmware was - see the 1.8.9 changelog entry for the original "100Hz slows
# the axis down" finding this is re-testing.
POSITION_BROADCAST_HZ = 50
POS_UPDATE_RATE_MS = int(1000 / POSITION_BROADCAST_HZ)  # 20 ms

# How often fresh ISS RA/DEC is sent to the Arduino (CMD,SET_TARGET) while the ISS is the
# actively tracked target - see _iss_update_tick/_on_iss_position. Independent of the 1Hz interval
# (ISS_DISPLAY_UPDATE_MS, below) used when the ISS is merely displayed (not tracked).
#
# Smoothness no longer strictly depends on this being fast - firmware 1.8.29+ derives a tracking
# rate from recent SET_TARGETs and extrapolates continuously between them on-board (the same role
# SIDEREAL_RATE_DEG_S plays for Earth-rotation compensation) - so 20Hz (50ms) is still plenty
# fresh against the ISS's real path without needing the previous 50Hz. Lowered from 20ms per
# request (see GUI_VERSION 1.0.5's changelog entry).
ISS_TRACKING_UPDATE_MS = 50

# How often the ISS's RA/DEC is refreshed for display only (ISS shown on the sky viz / hover info
# but NOT the actively tracked target) - see _iss_update_tick. 1Hz is plenty for a display-only
# refresh; ISS_TRACKING_UPDATE_MS above is used instead once the ISS becomes the tracked target.
ISS_DISPLAY_UPDATE_MS = 1000

# How often Sun/Moon/planet positions are refreshed - see _solar_system_update_tick
# (tracker_gui.py) / solar_system_timer (tracker_gui_qt.py). Previously 2000ms (plenty for how
# slowly these move against the star background - the Moon, the fastest of them, moves
# ~33"/min, so even 2s only bounds staleness to ~1"), raised to match ISS_DISPLAY_UPDATE_MS's 1Hz
# per request - a background-thread ephemeris lookup that's cheap enough not to matter at 2x the
# old rate.
SOLAR_SYSTEM_UPDATE_MS = 1000

# Matches the firmware's own SIDEREAL_RATE_DEG_S exactly (EQMountTracker.ino) - used ONLY for the
# client-side meridian-limit countdown estimate (_update_meridian_warning), never for any
# safety-critical decision - the firmware enforces the actual stop independently and
# authoritatively (pastMeridianLimit()/stopForMeridianLimit()), exactly like the Delete-key panic
# stop is enforced firmware-side regardless of what the GUI thinks the state is.
SIDEREAL_RATE_DEG_S = 0.004178
MERIDIAN_LIMIT_WARNING_S = 300.0  # first warning: 5 minutes out
MERIDIAN_LIMIT_URGENT_S = 10.0    # escalate: 10 seconds out

# Matches the firmware's MERIDIAN_LIMIT_DEC_ALLOWED_MIN/MAX_DEG (EQMountTracker.ino) - at low
# declination the OTA/dovetail passes nowhere near the tripod/pier even past the meridian, so
# movement is allowed there regardless of Hour Angle. Used only to draw the danger-zone
# visualization correctly (_draw_meridian_limit_zone) - the firmware enforces the actual
# exemption independently and authoritatively, same as the HA check itself.
MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG = -15.0
MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG = 20.0

# Standard IAU 1958 galactic coordinate system reference points (J2000-epoch RA/Dec of the North
# Galactic Pole, and the galactic longitude of the North Celestial Pole) - used only to draw a
# schematic Milky Way band on the sky viz (_build_milky_way_bands/_draw_milky_way). There's no
# bundled all-sky Milky Way brightness image/outline (would be a large asset for a DIY project
# like this); instead this analytically converts a swept range of galactic latitude/longitude
# into RA/Dec, giving a geometrically correct band (right shape, width, and orientation, sweeping
# across the sky exactly where the real Milky Way does) even though it can't capture the real
# thing's patchiness/dark rifts/bulge brightness structure - a schematic "here's roughly where and
# how wide it is", not a photographic rendering.
_GAL_NGP_RA_DEG = 192.85948
_GAL_NGP_DEC_DEG = 27.12825
_GAL_L_NCP_DEG = 122.93192

# Half-widths (galactic latitude, degrees) of the nested bands drawn by _draw_milky_way, widest/
# faintest first so each narrower/stronger band is drawn on top of it - approximates a soft glow
# fading outward from the galactic plane without needing real alpha transparency (Tkinter canvas
# fills don't support that - see _draw_horizon/_draw_meridian_limit_zone for the same "solid but
# subtle color" approach used elsewhere in this file). Paired 1:1 with MILKY_WAY_BAND_COLORS.
MILKY_WAY_BAND_HALF_WIDTHS_DEG = [18.0, 10.0, 4.0]
MILKY_WAY_BAND_COLORS = ["#161b28", "#1d2436", "#26314e"]

# Galactic longitude sample step (degrees) for building each band's edge curves - smaller is
# smoother but more polygons to draw every redraw; 4 degrees (90 samples around the full 360) is
# smooth enough for a soft glow band at any zoom level actually used on this viz.
MILKY_WAY_L_STEP_DEG = 4.0


def _galactic_to_radec(l_deg, b_deg):
    """Converts galactic (l, b) to equatorial (RA, Dec), both in degrees - standard spherical
    coordinate rotation using the IAU 1958 galactic pole constants above. Verified against the
    well-known Galactic Center position (l=0, b=0 should give RA=266.4 deg, Dec=-28.9 deg)."""
    l_rad = math.radians(l_deg)
    b = math.radians(b_deg)
    ngp_dec = math.radians(_GAL_NGP_DEC_DEG)
    l_ncp = math.radians(_GAL_L_NCP_DEG)
    sin_dec = math.sin(b) * math.sin(ngp_dec) + math.cos(b) * math.cos(ngp_dec) * math.cos(l_ncp - l_rad)
    dec = math.asin(max(-1.0, min(1.0, sin_dec)))
    y = math.cos(b) * math.sin(l_ncp - l_rad)
    x = math.sin(b) * math.cos(ngp_dec) - math.cos(b) * math.sin(ngp_dec) * math.cos(l_ncp - l_rad)
    ra = (math.degrees(math.atan2(y, x)) + _GAL_NGP_RA_DEG) % 360.0
    return ra, math.degrees(dec)


def _muted_hex_color(hex_color: str, amount: float = 0.55) -> str:
    """Blends a "#RRGGBB" color toward a neutral dark gray by `amount` (0=unchanged, 1=fully
    gray) - used to gray out buttons that can't do anything while disconnected (see
    MainWindow._set_arduino_controls_enabled/_gated_style) while still leaving a visible hint of
    their normal color, per request. A stylesheet-styled QPushButton doesn't change its own
    background color on disable once that background has been explicitly overridden, so this is
    applied manually rather than relying on Qt's built-in disabled look. Returns the input
    unchanged if it isn't a recognizable "#RRGGBB"/"#RGB" hex string (e.g. "transparent" or an
    already-resolved Qt palette color) - callers should skip muting entirely in that case rather
    than pass it through blindly."""
    h = hex_color.lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    if len(h) != 6:
        return hex_color
    try:
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    except ValueError:
        return hex_color
    gray = 58  # neutral dark gray target, matching this app's general panel/background tone
    r = round(r + (gray - r) * amount)
    g = round(g + (gray - g) * amount)
    b = round(b + (gray - b) * amount)
    return f"#{r:02x}{g:02x}{b:02x}"

# Anchored to this script's own directory, NOT the process's current working directory - a
# bare relative "gui_config.json" depends on wherever the app happens to be launched from
# (shortcut, IDE default CWD, a different terminal dir, ...) and silently misses the file
# (os.path.exists just returns False, no error) if that doesn't match, even when the file is
# sitting right next to the script.
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui_config.json")

# Placeholder observer location shown before a real one is set/loaded. Deliberately NOT a real
# address - Royal Observatory Greenwich (home of the Prime Meridian) is a public astronomical
# landmark, not anyone's private location. Set your actual coordinates via the GPS/Lat-Lon
# fields in the GUI; they're saved locally to gui_config.json (which is gitignored), never
# committed here.
DEFAULT_LAT = 51.4769
DEFAULT_LON = -0.0005

# Visualization
TELESCOPE_FOCAL_LENGTH_MM = 750

# Camera sensor field of view, measured empirically via a solar transit: the Sun measured
# ~1202px wide on a 4069px-wide frame. The Sun's mean angular diameter is ~0.533 deg (32
# arcmin; it actually varies ~31.5-32.7 arcmin over the year with Earth's orbital distance, so
# this is +-~2% depending on the date the photo was taken) - so the frame's horizontal FOV is:
#   FOV_w = sun_angular_diameter_deg * (frame_width_px / sun_width_px)
#         = 0.533 * (4069 / 1202) ~= 1.804 deg
# Height is derived from the 3:2 sensor aspect ratio rather than hardcoded separately, so the
# two can't drift out of sync with each other.
CAMERA_ASPECT_W = 3.0
CAMERA_ASPECT_H = 2.0
_SUN_ANGULAR_DIAMETER_DEG = 0.533
CAMERA_FOV_W_DEG = _SUN_ANGULAR_DIAMETER_DEG * (4069.0 / 1202.0)
CAMERA_FOV_H_DEG = CAMERA_FOV_W_DEG * (CAMERA_ASPECT_H / CAMERA_ASPECT_W)

# Real sensor resolution (26.1 MP, 6240x4160) - note this is NOT the 4069px frame width used
# above (that was just whatever size the measurement photo happened to be exported/resized to;
# the Sun's transit still covered the same real angular FOV regardless of export resolution).
# Used to derive the angular size of a single sensor pixel, i.e. the finest error the camera
# could even resolve - see TRACKING_STABLE_ERR_DEG.
CAMERA_SENSOR_WIDTH_PX = 6240
CAMERA_SENSOR_HEIGHT_PX = 4160
CAMERA_FOV_DEG_PER_PIXEL = CAMERA_FOV_W_DEG / CAMERA_SENSOR_WIDTH_PX

# Sky viz zoom
VIZ_ZOOM_MIN = 1.0      # 1.0 = full sky (RA 0-360, DEC -90..+90)
VIZ_ZOOM_MAX = 1000.0   # ~0.36 deg RA / ~0.18 deg DEC visible span. Planets are astronomically
                        # tiny - even Jupiter (the biggest apparent disk of the lot) is only
                        # ~30-50 arcsec, so 200x still only worked out to ~4-5px real radius,
                        # barely above the fixed-marker floor and visually indistinguishable from
                        # it. 1000x gives Jupiter/Venus/Mars/Saturn+rings a real radius in the
                        # ~15-25px range - clearly bigger than the marker, not just technically so.
                        # Uranus/Neptune stay essentially point-like even here, which is
                        # astronomically accurate (real telescopes need real aperture/magnification
                        # to resolve them too, not just "zoom").
VIZ_ZOOM_STEP_BASE = 1.15   # zoom multiplier per "one full wheel notch" worth of scroll delta
VIZ_GRID_TARGET_LINES = 9.0  # aim for roughly this many grid lines per axis at any zoom level

# Star level-of-detail tiers (see _build_sky_catalog_state/_draw_sky_objects). At full-sky zoom
# the visible magnitude limit is only ~3.2 (a couple hundred stars), but bisecting the single
# full ~870000-star RA-sorted list still means touching nearly the WHOLE catalog every redraw
# once the visible RA span approaches 360 (the bisect range covers everything) - measured at
# ~110ms for just the magnitude-filter pass alone, more than 5x VIZ_FOLLOW_TICK_MS's own budget
# and enough to stall the entire GUI (not just the viz) while in Follow mode. Pre-building
# several smaller, magnitude-capped, RA-sorted subsets once at catalog-load time lets
# _draw_sky_objects() bisect through whichever is the SMALLEST tier guaranteed to already
# contain everything the current zoom level would show, instead of always the full list.
# Deliberately concentrated below mag 11 - the AT-HYG reduced_m11 catalog (see sky_catalog.py)
# is already ~99.8% mag<=11 stars, so tiers past that barely shrink anything; the real size
# range worth tiering is 3.2 (low zoom) to ~11 (where the full catalog is basically reached).
STAR_LOD_TIER_MAG_CUTOFFS = [3.2, 4.5, 6.5, 8.5, 11.0]
VIZ_FOLLOW_TICK_MS = POS_UPDATE_RATE_MS  # how often the follow-mode camera snaps to its target
                                 # AND the full grid/label/horizon rebuild (_redraw_viz()) runs
                                 # while following - tied directly to POS_UPDATE_RATE_MS (the real
                                 # position-update rate) per request, once 100ms was confirmed to
                                 # perform fine: no point centering faster than fresh position
                                 # data actually arrives, but also no reason to stay any coarser
                                 # than that once the rebuild cost is confirmed to keep up.
                                 #
                                 # Panning the background instead of rebuilding it (canvas.move
                                 # ("all", dx, dy) shifting existing items in place, tried at
                                 # several different cadences) was abandoned for good after three
                                 # separate failure modes, each only discovered by actually trying
                                 # it: (1) panning on its own coarse timer while nothing else
                                 # tracked what area it had actually drawn caused the view's
                                 # reference frame to race ahead of the background it's drawn
                                 # relative to - visible desync during fast slews; (2) calling the
                                 # pan at high frequency (~20Hz) directly from the POS handler was
                                 # itself too expensive for this canvas's item count and stalled
                                 # the whole UI; (3) even called from its own dedicated fast timer
                                 # (avoiding failure #2), it broke differently: the dark background
                                 # rectangle (viz_bg, sized to exactly cover the canvas) is one of
                                 # the items canvas.move("all", ...) shifts, so panning moves it
                                 # away from actually covering the canvas - the raw, undrawn canvas
                                 # showed through at whichever edge the view had panned towards,
                                 # visible as the viz "going dark". None of these are a throttle
                                 # value away from being fixed - a working version would need the
                                 # background redrawn to always overfill the canvas (so a pan can
                                 # never expose its edge) AND pre-rendered over a larger area than
                                 # the visible viewport (so a pan never reveals genuinely
                                 # undrawn sky either) AND a rebuild trigger based on actual
                                 # accumulated pan distance, not just a timer - real work, not
                                 # in scope for a follow-up throttle tweak. Back to a plain full
                                 # rebuild every tick; this constant is the only real lever until
                                 # that's built, or until _draw_viz_grid() itself is optimized to
                                 # cost less per call (fewer items, cached surfaces, etc.).
                                 #
                                 # Pushing this lower has a floor: each follow_tick() only
                                 # schedules its own next call via self.after(VIZ_FOLLOW_TICK_MS,
                                 # ...) AFTER _redraw_viz() returns, so if a rebuild ever takes
                                 # longer than this interval to run, calls don't overlap/race -
                                 # but the achieved rate silently drops to however long rebuilds
                                 # actually take, not this nominal value (this is exactly what was
                                 # measured at a previous 60ms attempt: "~1fps, not 60fps" - that
                                 # was with a much larger visible star count than typical follow-
                                 # mode zoom levels tend to have, though, so it isn't a hard rule
                                 # that this rate can't work; confirmed fine in practice at 100ms).
                                 # If centering ever visibly lags position updates again, that's
                                 # the sign this is back above the real per-rebuild cost floor for
                                 # whatever's currently on screen (zoom level, star density, etc).

# Tracking stability indicator. A raw "error < X" check is a poor stability signal on its own:
# a large-but-shrinking error is actually fine (still converging), while a small-but-growing
# one is a real problem in the making. So classification uses the *trend* (derivative) of the
# error over TRACKING_STABILITY_WINDOW_S, not just its instantaneous magnitude.
# "Stable" means sub-pixel: the error is smaller than half a camera sensor pixel's angular width,
# so there's nothing more for the mount to correct that the camera would ever see. Half, not a
# full pixel, because a pixel has width - it samples everything within +-0.5px of its center as
# the same value, so an error has to be under half a pixel to be guaranteed invisible; an error
# of e.g. 0.9px is still well within "one pixel" but would visibly shift which pixel a point
# source lands on.
TRACKING_STABLE_ERR_DEG = CAMERA_FOV_DEG_PER_PIXEL #/ 2.0   # sub-pixel magnitude, both axes, for STABLE
TRACKING_STABLE_DERIV_DEG_S = 0.0008    # error growth rate (deg/s) below which it's considered flat
TRACKING_STABILITY_WINDOW_S = 2.0       # trend-averaging window - smooths per-update noise

# ============================================================
# SERIAL HANDLER (background thread)
# ============================================================
class SerialHandler:
    def __init__(self, message_queue: queue.Queue, get_time_fn=None):
        self.ser: Optional[serial.Serial] = None
        self.queue = message_queue
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.port = ""
        self.last_error = ""
        # The ONE time source send_time() sends to the Arduino - a callable returning the current
        # datetime (UTC, tz-aware). MainWindow passes its own _get_effective_utc_now, the exact
        # same function every other position lookup in the app (LST, Sun/Moon/planets, ISS) reads
        # from - so there's a single place that decides "what time is it right now" (real, or a
        # Time Travel preview), not a copy mirrored between here and the GUI. Queried fresh on
        # every send_time() call (on connect, the periodic resync, and an explicit preview) rather
        # than cached, so it's always current. Falls back to genuine real time if no GUI is wired
        # up (e.g. standalone use/testing of SerialHandler).
        self.get_time_fn = get_time_fn or (lambda: datetime.now(timezone.utc))

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [f"{p.device} - {p.description}" for p in ports]

    def connect(self, port_str: str) -> bool:
        # Extract device name e.g. "COM3 - ..."
        device = port_str.split(" - ")[0].strip()
        try:
            # Use timeout=0 for non-blocking reads.
            # Explicit dsrdtr=False + rtscts=False + immediate dtr/rts=False to prevent
            # Arduino reset on open (keeps calibration state across GUI close/reopen).
            # The Arduino should continue running independently when GUI disconnects.
            self.ser = serial.Serial(
                device, 
                BAUD_RATE, 
                timeout=0, 
                rtscts=False, 
                dsrdtr=False, 
                write_timeout=0,
                exclusive=True  # Prevent other processes from accessing the port
            )
            
            # Critical: Disable DTR/RTS immediately to prevent Arduino reset
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.2)  # Increased delay to ensure control lines settle
            
            # Clear any stale bytes that may be in buffers from a previous session.
            # This is safe because we don't reset the Arduino.
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass
            
            # Additional delay to ensure Arduino is ready
            time.sleep(0.8)  # Increased safety delay
            
            # Final buffer clear
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
                
            self.port = device
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()

            # Send current time immediately (location is sent from GUI with real coords)
            # Note: Arduino should maintain its own timekeeping when GUI is disconnected
            self.send_time()
            
            # Send a "hello" command to wake up Arduino and ensure it's responsive
            self.send_command("CMD,PING")

            # NOTE: the PING-response timeout is scheduled by the caller (MainWindow._connect),
            # not here - SerialHandler is a plain object with no GUI event loop of its own, so
            # calling self.after(...) here always raised AttributeError, which made connect()
            # throw and return False on EVERY attempt - the serial port itself opened fine (the
            # reader thread starts before this point and works independently), but "CONNECTED:"
            # below was never reached, so the GUI's connected/disconnected indicator, button
            # states, and post-connect setup (send location/offsets/rate) never ran even though
            # the Arduino connection was actually live and working.
            self.queue.put("CONNECTED:" + device)
            return True
        except Exception as e:
            self.last_error = str(e)
            self.queue.put("ERROR:Connection failed: " + str(e))
            return False

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                # Explicitly deassert DTR/RTS before closing to avoid triggering
                # a reset on the Arduino when the port is closed/reopened.
                self.ser.dtr = False
                self.ser.rts = False
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.queue.put("DISCONNECTED")

    def _reader_loop(self):
        # Use bytes buffer for cleaner line splitting (Arduino sends \r\n)
        buffer = b""
        while self.running and self.ser and self.ser.is_open:
            try:
                # Pure non-blocking poll using in_waiting. This is the most reliable pattern
                # for sustained high-frequency data from Arduino (POS + debug dumps).
                inw = 0
                try:
                    inw = self.ser.in_waiting
                except Exception:
                    inw = 0

                if inw > 0:
                    try:
                        data = self.ser.read(inw)
                        if data:
                            buffer += data
                            # Process all complete lines
                            while b'\n' in buffer:
                                line, buffer = buffer.split(b'\n', 1)
                                line = line.strip()
                                if line:
                                    decoded = line.decode('ascii', errors='ignore')
                                    self.queue.put("RX:" + decoded)
                    except Exception as read_err:
                        # transient read error, continue
                        pass
                else:
                    # No data right now, short sleep to not spin CPU
                    time.sleep(0.002)
            except Exception as e:
                # Log error but keep the reader alive
                self.queue.put("ERROR:reader " + str(e))
                time.sleep(0.05)

    def send_command(self, cmd: str) -> bool:
        """Returns True only if the full command was actually written to the port.
        With write_timeout=0 (non-blocking writes, needed so a stalled/unplugged port can't
        hang the GUI), pyserial's write() can return having written FEWER bytes than given
        instead of raising - the caller MUST check the return value, since previously that
        was silently discarded and a partial write looked identical to a full success.

        Deliberately does NOT call self.ser.flush() - unlike write(), flush() is NOT governed by
        write_timeout and is documented to always block until the OS has finished physically
        transmitting everything queued so far. At low broadcast rates the port is idle enough
        that this returns instantly and the block goes unnoticed; at the higher POS broadcast
        rates tested in this session (heavy simultaneous RX traffic), it can take real,
        multi-second time to drain, and since this runs on the main thread (e.g. every Stop
        Tracking / Delete-Suppr press), that stalls the entire Tkinter UI - a genuine freeze, not
        just a stuck status label. Dropping it doesn't drop any bytes - the OS still transmits
        everything written, this just stops the caller from waiting around for that to finish."""
        if not (self.ser and self.ser.is_open):
            return False
        payload = (cmd + "\n").encode('ascii')
        try:
            written = self.ser.write(payload)
            if written != len(payload):
                self.queue.put(f"ERROR:Send incomplete for '{cmd}' ({written}/{len(payload)} bytes) - port may be backed up")
                return False
            self.queue.put("TX:" + cmd)
            return True
        except Exception as e:
            self.queue.put("ERROR:Send failed - " + str(e))
            return False

    def send_time(self):
        # See self.get_time_fn's comment - real time unless a Time Travel preview is active, in
        # which case the Arduino's clock is set to the same simulated time as everything else.
        now = self.get_time_fn()
        cmd = (f"CMD,SET_TIME,Y:{now.year},M:{now.month},D:{now.day},"
               f"h:{now.hour},min:{now.minute},s:{now.second}")
        self.send_command(cmd)

    def send_time_if_connected(self) -> bool:
        """Returns whether the time was actually sent (i.e. we're connected) - callers like
        MainWindow._apply_time_travel use this to tell the user whether the Arduino's clock was
        just synced or will only pick up the change once a connection exists."""
        if self.ser and self.ser.is_open:
            self.send_time()
            return True
        return False

    def send_mode(self, mode: str):
        self.send_command(f"CMD,MODE,{mode}")

    def send_start_tracking(self, risk_ok: bool = False) -> bool:
        # risk_ok: the GUI already confirmed a meridian-limit warning for this target with the
        # user (see App._confirm_risky_slew) - see riskAcknowledged's comment in the .ino.
        cmd = "CMD,START_TRACKING,RISK_OK:1" if risk_ok else "CMD,START_TRACKING"
        return self.send_command(cmd)

    def send_start_tracking_skip_dec_reset(self, risk_ok: bool = False) -> bool:
        cmd = "CMD,START_TRACKING,SKIP_DEC_RESET,RISK_OK:1" if risk_ok else "CMD,START_TRACKING,SKIP_DEC_RESET"
        return self.send_command(cmd)

    def send_stop(self) -> bool:
        # A bare '!' byte is sent first, immediately, ahead of the normal text command - the
        # firmware treats it as a dedicated panic-stop sentinel, checked before any line
        # buffering/parsing (see readSerialCommands() in the .ino), so it can't be delayed by
        # whatever's going on with the text-command protocol (observed: STOP sometimes taking
        # several seconds to be acted on - see the 1.8.15 changelog entry). The full "CMD,STOP"
        # text command still follows for logging/status-response consistency, but the '!' byte
        # alone is what actually guarantees the stop happens promptly.
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'!')  # no flush() here either - see send_command()'s comment
            except Exception:
                pass
        return self.send_command("CMD,STOP")

    def send_sync(self, ra: float, dec: float):
        self.send_command(f"CMD,SYNC,RA:{ra:.6f},DEC:{dec:.6f}")

    def send_sync_offset(self, ra_offset: float, dec_offset: float):
        self.send_command(f"CMD,SYNC_OFFSET,RA:{ra_offset:.6f},DEC:{dec_offset:.6f}")

    def send_goto(self, ra: float, dec: float, risk_ok: bool = False):
        cmd = f"CMD,GOTO,RA:{ra:.6f},DEC:{dec:.6f}"
        if risk_ok:
            cmd += ",RISK_OK:1"  # see riskAcknowledged's comment in the .ino
        self.send_command(cmd)

    def send_safe_target(self):
        self.send_command("CMD,SAFE_TARGET")

    def send_home_axes(self):
        self.send_command("CMD,HOME_AXES")

    def send_pos_update_rate(self, ms: int):
        """Tell Arduino how often to broadcast POS updates (in milliseconds)."""
        if ms < 10:
            ms = 10
        self.send_command(f"CMD,SET_POS_UPDATE,MS:{ms}")

    def send_debug(self, enable: bool):
        """Turn Arduino heavy variable debug dump on/off (CMD,DEBUG,ON/OFF)."""
        self.send_command("CMD,DEBUG,ON" if enable else "CMD,DEBUG,OFF")

    def send_save_location_to_eeprom(self):
        """Request Arduino to save current location to EEPROM (requires firmware support)."""
        self.send_command("CMD,SAVE_LOCATION_EEPROM")

    def send_load_location_from_eeprom(self):
        """Request Arduino to load location from EEPROM (requires firmware support)."""
        self.send_command("CMD,LOAD_LOCATION_EEPROM")

    def send_query_location(self):
        """Query Arduino for its current location (requires firmware support)."""
        self.send_command("CMD,QUERY_LOCATION")

    def send_meridian_limit(self, enable: bool):
        """Enable/disable the firmware's meridian-limit safety check (see meridianLimitEnabled
        in the .ino) - requires firmware 1.8.52+."""
        self.send_command(f"CMD,SET_MERIDIAN_LIMIT,ENABLED:{1 if enable else 0}")

    def send_query_meridian_limit(self):
        """Query Arduino for its current meridian-limit enabled state (requires firmware 1.8.52+)."""
        self.send_command("CMD,QUERY_MERIDIAN_LIMIT")

    def request_debug_dump(self):
        """Ask Arduino for one full debug state dump right now."""
        self.send_command("CMD,DEBUG,DUMP")

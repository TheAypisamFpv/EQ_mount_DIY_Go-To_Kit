#!/usr/bin/env python3
"""
EQ Mount Tracker - Python Desktop GUI
Dark-themed astronomy control application using CustomTkinter + pyserial.

Features:
- Serial connection to Arduino Mega at 250000 baud
- COM port selection + auto refresh
- Real-time RA/DEC position display + simple visualization
- Sidereal / Lunar / Solar mode selection
- Start/Stop tracking (triggers full alignment sequence on Arduino)
- Sync (tell Arduino the current pointed sky position)
- Sidereal target input boxes (RA/DEC for Start Tracking)
- Status and error display during alignment

Dependencies:
    pip install customtkinter pyserial

Run:
    python tracker_gui.py
"""

import customtkinter as ctk
import tkinter as tk  # only for widgets customtkinter doesn't provide (Listbox, for sky search results)
from tkinter import messagebox  # solar tracking safety confirmation - see _toggle_tracking
from PIL import Image, ImageDraw, ImageFont, ImageTk, ImageColor  # sky viz raster background - see _RasterCanvas
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
import math
import json
import os
import bisect
from collections import deque
from datetime import datetime, timezone, timedelta
from typing import Optional
from sky_data import sky_catalog, iss_tracker, solar_system

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
GUI_VERSION = "1.0.13"

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

# Anchored to this script's own directory, NOT the process's current working directory - a
# bare relative "gui_config.json" depends on wherever the app happens to be launched from
# (shortcut, IDE default CWD, a different terminal dir, ...) and silently misses the file
# (os.path.exists just returns False, no error) if that doesn't match, even when the file is
# sitting right next to the script.
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui_config.json")

# Same anchoring reasoning as CONFIG_PATH above. Every line that goes into the Status/message box
# is also appended here in real time (see App._log) - the on-screen box trims old lines to stay
# responsive (see _log's line-count check below), but nothing is ever lost from this file. Opened
# once in append mode (never "w", never truncated) so relaunching the GUI keeps piling onto the
# same running history rather than overwriting it - see App.__init__/on_closing.
LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tracker_log.txt")

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

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ============================================================
# SKY VIZ RASTER BACKGROUND (see _RasterCanvas)
# ============================================================
# Cache of loaded Pillow fonts, keyed by (size, bold) - see _get_viz_pil_font. Avoids hitting the
# filesystem/FreeType to rebuild the same font object for every single label drawn on every redraw.
_VIZ_FONT_CACHE = {}

# Cache of hex-string -> RGB-tuple color conversions - see _RasterCanvas._clean. A full-sky redraw
# makes tens of thousands of draw calls (mostly tiny star dots), and Pillow's ImageDraw re-parses
# a plain hex string (via ImageColor.getcolor, a small regex-based parser) on every single one of
# them - profiling showed that alone as the single biggest cost of a redraw, well above the actual
# pixel drawing. The color vocabulary here is small and fixed (grid lines, DSO type colors, star
# B-V colors, etc.), so caching the parsed tuple the first time a given hex string is seen and
# reusing it skips that re-parsing entirely on every later draw call with the same color.
_VIZ_COLOR_CACHE = {}


def _muted_hex_color(hex_color: str, amount: float = 0.55) -> str:
    """Blends a "#RRGGBB" color toward a neutral dark gray by `amount` (0=unchanged, 1=fully
    gray) - used to gray out buttons that can't do anything while disconnected (both GUIs'
    "_set_arduino_controls_enabled") while still leaving a visible hint of their normal color,
    per request. Neither CTkButton nor a stylesheet-styled QPushButton changes its own
    background color on disable once that background has been explicitly overridden, so this is
    applied manually rather than relying on either toolkit's built-in disabled look. Shared by
    tracker_gui.py and tracker_gui_qt.py (imported from here) so both use identical math.
    Returns the input unchanged if it isn't a recognizable "#RRGGBB"/"#RGB" hex string (e.g. a
    CTk theme tuple element, "transparent", or an already-resolved Qt palette color) - callers
    should skip muting entirely in that case rather than pass it through blindly."""
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


def _pil_ink(color):
    ink = _VIZ_COLOR_CACHE.get(color)
    if ink is None:
        ink = ImageColor.getrgb(color)
        _VIZ_COLOR_CACHE[color] = ink
    return ink


def _get_viz_pil_font(size, bold=False):
    """Loads (and caches) the Consolas font used for viz canvas labels, matching the
    ("Consolas", size[, "bold"]) tuples the drawing code already passes to create_text.
    Falls back to Pillow's built-in bitmap font if Consolas isn't installed (e.g. non-Windows) -
    labels will look plainer but the canvas still renders instead of raising."""
    key = (size, bold)
    font = _VIZ_FONT_CACHE.get(key)
    if font is not None:
        return font
    windir = os.environ.get("WINDIR", "C:\\Windows")
    names = ("consolab.ttf",) if bold else ("consola.ttf",)
    font = None
    for name in names:
        try:
            font = ImageFont.truetype(os.path.join(windir, "Fonts", name), size)
            break
        except Exception:
            continue
    if font is None:
        try:
            font = ImageFont.load_default(size)  # Pillow >=10.1 - scalable default font
        except TypeError:
            font = ImageFont.load_default()
    _VIZ_FONT_CACHE[key] = font
    return font


class _RasterCanvas:
    """Minimal Tkinter-Canvas-shaped adapter that draws into a Pillow image instead of creating
    real Tk canvas items - a drop-in stand-in for the `c` parameter the viz drawing helpers
    (_draw_horizon, _draw_milky_way, _draw_meridian_limit_zone, _draw_sky_objects,
    _draw_illuminated_disc, and the grid/ISS/Sun/Moon/planet/label code inline in
    _draw_viz_grid) already take, only implementing the handful of create_* calls they actually
    use - not a general Canvas replacement.

    Why: the viz canvas's static background (grid lines, Milky Way band, horizon shading, up to
    thousands of catalog stars/DSOs) used to be rebuilt as that many individual REAL Tk canvas
    items via c.delete("all") + create_line/create_oval/create_polygon/create_text, on every
    single zoom/pan/follow tick. Each of those is its own Tcl call - fine for the handful of
    interactive overlay items (camera FOV rect, target reticle - those still use the real
    Tkinter Canvas, see _draw_viz_grid), but the dominant cost, and the actual cause of zoom/pan
    feeling like ~1-4fps (and heavy POS serial traffic on top of it stalling the whole Tk event
    loop long enough to look like the GUI froze/reloaded), once it's thousands of them rebuilt
    every frame. Rasterizing all of that into one Pillow image and blitting it as a SINGLE Tk
    canvas image item turns "thousands of Tcl calls" into "one", regardless of how much content
    is actually on screen.

    tags= is accepted everywhere and ignored: it only ever existed so _safe_tag_raise could fix
    up draw order after the fact - unnecessary here since these calls already happen in the
    correct back-to-front order and painting into an image bakes that order in directly."""

    def __init__(self, w, h, bg):
        self.image = Image.new("RGB", (max(1, int(w)), max(1, int(h))), bg)
        self.draw = ImageDraw.Draw(self.image)

    @staticmethod
    def _clean(color):
        return _pil_ink(color) if color else None

    @staticmethod
    def _flatten(args):
        flat = list(args[0]) if len(args) == 1 else list(args)
        return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

    def create_line(self, *args, fill=None, width=1, dash=None, tags=None, **kw):
        pts = self._flatten(args)
        lw = max(1, round(width))
        color = self._clean(fill) or "#000000"
        if dash and len(pts) == 2:
            self._draw_dashed_segment(pts[0], pts[1], color, lw, dash)
        else:
            self.draw.line(pts, fill=color, width=lw)

    def _draw_dashed_segment(self, p0, p1, color, width, dash):
        on = dash[0]
        off = dash[1] if len(dash) > 1 else dash[0]
        x0, y0 = p0
        x1, y1 = p1
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-6:
            return
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        pos, drawing = 0.0, True
        while pos < length:
            seg = on if drawing else off
            end = min(pos + seg, length)
            if drawing:
                self.draw.line([(x0 + ux * pos, y0 + uy * pos), (x0 + ux * end, y0 + uy * end)],
                                fill=color, width=width)
            pos = end
            drawing = not drawing

    def create_oval(self, x0, y0, x1, y1, fill=None, outline=None, width=1, dash=None, tags=None, **kw):
        outline_c = self._clean(outline)
        self.draw.ellipse([x0, y0, x1, y1], fill=self._clean(fill), outline=outline_c,
                          width=max(1, round(width)) if outline_c else 1)

    def create_rectangle(self, x0, y0, x1, y1, fill=None, outline=None, width=1, tags=None, **kw):
        outline_c = self._clean(outline)
        self.draw.rectangle([x0, y0, x1, y1], fill=self._clean(fill), outline=outline_c,
                            width=max(1, round(width)) if outline_c else 1)

    def create_polygon(self, *args, fill=None, outline=None, width=1, dash=None, tags=None, **kw):
        pts = self._flatten(args)
        if len(pts) < 3:
            return
        outline_c = self._clean(outline)
        self.draw.polygon(pts, fill=self._clean(fill), outline=outline_c)
        if outline_c and width and width > 1:
            # Pillow's polygon() outline is always ~1px regardless of width - go over the
            # (already-closed, since polygon() implicitly closes back to the first point) edge
            # again as an explicit line for anything thicker.
            self.draw.line(pts + [pts[0]], fill=outline_c, width=max(1, round(width)))

    def create_arc(self, x0, y0, x1, y1, start=0, extent=180, style="chord",
                   fill=None, outline=None, width=1, tags=None, **kw):
        # Only style="chord" is used anywhere in this file. Sampled manually (rather than
        # delegated to Pillow's own arc/chord primitives) to guarantee it matches Tk's own angle
        # convention exactly: 0 deg = 3 o'clock, increasing counter-clockwise in real space,
        # i.e. y = cy - r*sin(theta) since screen y grows downward - same convention the
        # docstring on _draw_illuminated_disc's create_arc call already relies on.
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        rx, ry = abs(x1 - x0) / 2.0, abs(y1 - y0) / 2.0
        n = 24
        pts = []
        for i in range(n + 1):
            theta = math.radians(start + extent * i / n)
            pts.append((cx + rx * math.cos(theta), cy - ry * math.sin(theta)))
        # polygon() auto-closes the last point back to the first, which for a partial arc IS
        # exactly the "chord" style (a straight line connecting the two open ends) - no separate
        # center point needed the way "pieslice" style would require.
        self.draw.polygon(pts, fill=self._clean(fill), outline=self._clean(outline))

    def create_text(self, x, y, text="", fill=None, font=None, anchor=None, tags=None, **kw):
        size = font[1] if font and len(font) > 1 else 10
        bold = bool(font and len(font) > 2 and "bold" in font[2])
        pil_anchor = {"w": "lm", "e": "rm"}.get(anchor, "mm")  # Tk default anchor is "center"
        self.draw.text((x, y), text, fill=self._clean(fill) or "#ffffff",
                       font=_get_viz_pil_font(size, bold), anchor=pil_anchor)

    def to_photoimage(self):
        return ImageTk.PhotoImage(self.image)


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
        # datetime (UTC, tz-aware). EQMountApp passes its own _get_effective_utc_now, the exact
        # same function every other position lookup in the app (LST, Sun/Moon/planets, ISS) reads
        # from - so there's a single place that decides "what time is it right now" (real, or a
        # Time Travel preview), not a copy mirrored between here and the App. Queried fresh on
        # every send_time() call (on connect, the periodic resync, and an explicit preview) rather
        # than cached, so it's always current. Falls back to genuine real time if no App is wired
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

            # NOTE: the PING-response timeout is scheduled by the caller (EQMountApp._connect),
            # not here - SerialHandler is a plain object with no Tk event loop / .after(), so
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
        EQMountApp._apply_time_travel use this to tell the user whether the Arduino's clock was
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


# ============================================================
# MAIN APPLICATION
# ============================================================
class EQMountApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("EQ Mount DIY Go-To Kit - Controller")
        self.geometry("980x720")
        self.minsize(900, 620)

        self.message_queue: queue.Queue = queue.Queue()
        # get_time_fn=self._get_effective_utc_now wires SerialHandler.send_time() to the same
        # single time source everything else uses (see _get_effective_utc_now's comment) - no
        # separate offset mirrored/kept in sync here, just one function queried live.
        self.serial = SerialHandler(self.message_queue, get_time_fn=self._get_effective_utc_now)

        # Opened once, in append mode, for the life of the app - see LOG_FILE_PATH's comment.
        # Kept open (rather than open/close per line) so a session with heavy traffic (e.g.
        # Arduino debug dump ON) doesn't pay a filesystem open() per log line; flushed after every
        # write in _log() instead, so "real time" still holds - a crash loses at most the last
        # unflushed line, not the whole session. Failure here (e.g. read-only install dir) must
        # not prevent the GUI from starting - falls back to None, and _log() just skips the file
        # write when that happens.
        try:
            self._log_file = open(LOG_FILE_PATH, "a", encoding="utf-8")
        except OSError as e:
            self._log_file = None
            print(f"WARNING: could not open log file {LOG_FILE_PATH}: {e}")

        self.current_ra = 0.0
        self.current_dec = 0.0
        self.target_ra = 0.0
        self.target_dec = 0.0
        self.target_object_name = None  # e.g. "Vega", "M31" - see _select_sky_target/_update_target_name_label
        # Exact text _select_sky_target last wrote into goto_ra/goto_dec, so _toggle_tracking can
        # tell "user pressed Start Tracking on what was just selected" from "user edited the boxes
        # since" by comparing strings directly - see _toggle_tracking's SIDEREAL branch. Comparing
        # parsed floats against self.target_ra/dec instead doesn't work here: self.target_ra/dec
        # aren't updated by _select_sky_target when tracking is already active (it must not
        # interrupt a running session), so they can go stale relative to the boxes even though
        # nothing was manually edited.
        self._target_name_box_ra_str = None
        self._target_name_box_dec_str = None
        self.err_ra = 0.0
        self.err_dec = 0.0
        self.mount_ra_angle = 0.0  # physical mount angle (deg)
        self.mount_dec_angle = 0.0
        self.speed_ra = 0.0
        self.speed_dec = 0.0
        self.ra_pos_history = deque()  # (angle, time) pairs - calc uses positions within last 250ms (or last one if older)
        self.dec_pos_history = deque()
        self._last_pos_ui_update = 0.0  # see POS_UI_UPDATE_MIN_INTERVAL_S / the POS handler below
        self._err_ra_history = deque()  # (error_deg, time) pairs - used for tracking stability trend
        self._err_dec_history = deque()
        self.current_mode = "SIDEREAL"
        self.tracking = False
        self.slewing = False
        # Last CONFIRMED telescope-flip state (from POS's "flipped" field or STATUS:FLIP_COMPLETE)
        # - see _toggle_telescope_flipped/flipped_toggle_btn and the .ino's setTelescopeFlipped().
        self._telescope_flipped = False
        # Local mirror of the firmware's meridianLimitEnabled flag - see
        # _toggle_meridian_limit/meridian_limit_btn and the .ino's CMD,SET_MERIDIAN_LIMIT.
        # Defaults True (enabled/safe), matching the firmware's own boot default, and is
        # re-sent to the Arduino on every connect (_apply_initial_meridian_limit) so a firmware
        # that was left disabled from a previous session (Arduino stayed powered through a GUI
        # restart) gets corrected back to whatever the GUI currently shows, rather than the two
        # silently disagreeing.
        self._meridian_limit_enabled = True
        self.arduino_debug_enabled = False
        self.last_status = "Disconnected"
        self.system_state = "DISCONNECTED"
        self._last_sent_location = None  # Track last sent location to avoid duplicates
        self._connection_timeout_id = None  # For tracking connection timeout

        # Time Travel: the ONE piece of state behind _get_effective_utc_now, the single time
        # source every position lookup in the app reads from - GUI (Sun/Moon/planet/star/DSO/ISS
        # positions) AND the Arduino's own onboard clock (SerialHandler.get_time_fn) alike. Stored
        # as an offset from real UTC (rather than a frozen instant) so the preview keeps advancing
        # at 1x, same as actually being at that moment, instead of freezing - see
        # _get_effective_utc_now / the Time Travel controls in _build_ui.
        self._time_travel_offset = timedelta(0)

        # Sky viz zoom/pan state. zoom=1.0 shows the full sky (RA 0-360, DEC -90..+90);
        # center is the (RA, DEC) at the middle of the canvas. See _get_viz_view_bounds().
        self._viz_zoom = 1.0
        self._viz_center_ra = 180.0
        self._viz_center_dec = 0.0
        self._viz_pan_last_xy = None
        self._viz_redraw_pending = False  # see _schedule_viz_redraw
        self._viz_view_mode = "FREE"  # FREE -> TELESCOPE -> TARGET -> FREE, see _cycle_viz_view_mode
        # Camera FOV rectangle's display rotation (position angle, North-up=0, increasing
        # towards East - same convention as _rotated_ellipse_points) - purely a GUI overlay
        # setting to match however the camera is actually mounted on the telescope; the mount
        # itself has no notion of camera rotation. See _adjust_camera_orientation.
        self._camera_orientation_deg = 0.0

        # Night-sky object catalog (stars + Messier DSOs) drawn on the viz - populated by
        # _load_sky_catalog_async() below, once the UI (and self.status_text, which self._log()
        # needs) actually exists. Just empty placeholders here.
        self._sky_stars = []
        self._sky_dso = []
        self._star_by_hip = {}
        self._const_line_segments = []
        self._sky_stars_by_ra = []       # self._sky_stars sorted by RA - see _draw_sky_objects
        self._sky_stars_ra_values = []   # parallel list of just the RA values, for bisect
        self._star_tiers = []  # magnitude-capped RA-sorted subsets - see STAR_LOD_TIER_MAG_CUTOFFS
        self._visible_object_hits = []  # see _draw_sky_objects / _on_viz_mouse_move
        self._hovered_sky_object = None  # see _on_viz_mouse_move / _on_viz_double_click
        self._sky_search_index = []  # see _build_sky_catalog_state / _update_sky_search_results
        self._constellations_enabled = False  # toggled via the Constellations button

        # DSO min-apparent-size filter - hides deep-sky objects (galaxies/nebulae/clusters, NOT
        # stars, which have no meaningful angular size) that would project smaller than
        # self._min_dso_size_px on the ACTUAL camera sensor (CAMERA_FOV_DEG_PER_PIXEL - the real
        # measured optics/sensor spec, same one TRACKING_STABLE_ERR_DEG is built on), i.e. "is
        # this actually big enough to be worth imaging with this setup", NOT how big it happens to
        # render on the GUI's own arbitrary, independently-zoomable viz canvas - a fixed physical
        # quantity per object, unaffected by zooming/panning the viz. Off by default - see
        # _toggle_min_dso_size_filter/_on_min_dso_size_entry_commit and the size check in
        # _draw_sky_objects.
        self._min_dso_size_filter_enabled = False
        self._min_dso_size_px = 100.0

        # Real-time ISS marker (see iss_tracker.py). Off by default - it needs internet (TLE
        # fetch from CelesTrak) and the skyfield package, neither of which should be required
        # just to run the rest of the app.
        self._iss_enabled = False
        self._iss_ra = None
        self._iss_dec = None
        self._iss_above_horizon = False
        self._iss_update_after_id = None

        # Real-time Sun/Moon/planets (see solar_system.py) - always shown, no toggle (unlike the
        # ISS, which still needs a fresh internet TLE fetch every session and stays opt-in for
        # that reason). The one-time ~17MB JPL DE421 ephemeris download here only ever happens
        # once, cached locally afterward - same tradeoff already accepted for the sky catalog.
        # Positions simply stay None (nothing drawn) until the first successful background fetch
        # completes - see _solar_system_update_tick, kicked off automatically at startup.
        self._sun_ra = None
        self._sun_dec = None
        self._sun_ang_diam_deg = None
        self._moon_ra = None
        self._moon_dec = None
        self._moon_ang_diam_deg = None
        self._moon_illum_fraction = None
        self._moon_phase_deg = None
        self._moon_waxing = None
        self._planet_positions = {}  # name -> (ra_deg, dec_deg, ang_diam_deg, ring_ang_diam_deg, illum_fraction), see solar_system.get_planets_info
        self._solar_system_update_after_id = None
        # Set once if skyfield turns out not to be installed, so _solar_system_update_tick stops
        # retrying forever instead of re-attempting (and re-logging) every 60s - there's no toggle
        # button anymore to let the user turn this off manually the way the ISS one used to.
        self._solar_system_skyfield_missing = False

        # START/STOP confirmation-retry state - see _request_tracking_action
        self._pending_action = None       # "START", "STOP", or None
        self._pending_action_confirmed = True
        self._pending_action_retry_count = 0
        self._pending_action_after_id = None

        # "IDLE" / "ALIGNING" / "TRACKING" - separate from self.tracking (which flips true as
        # soon as RA settles, partway through alignment) so the stability badge can tell
        # "still aligning" apart from "actually holding the target" - see _update_tracking_stability.
        # Current stability badge color, mirrored onto the telescope FOV circlere - see _set_stability.
        self._align_phase = "IDLE"
        self._stability_color = "#444455"

        # See _on_mode_changed: forces the next Start Tracking to skip the "target is close"
        # fast path, since the GUI's cached target is stale (not yet recomputed for the new
        # mode) right after a mode switch.
        self._force_full_align_next_start = False

        self.lat_var = ctk.StringVar(value=str(DEFAULT_LAT))
        self.lon_var = ctk.StringVar(value=str(DEFAULT_LON))

        self.gps_mode_var = ctk.BooleanVar(value=False)
        self.gps_var = ctk.StringVar(value=f"{DEFAULT_LAT}, {DEFAULT_LON}")

        self._location_redraw_id = None
        self._config_loaded = False

        # Load saved GPS/location config (if present) BEFORE building UI
        # so the input frames and mode are correctly set on startup.
        self._load_gui_config()

        # Bright stars visible from northern hemisphere (approx J2000 RA/DEC in degrees) - used
        # as a fallback ONLY if the bundled catalog (self._star_by_hip, populated once
        # _load_sky_catalog_async() finishes) isn't loaded yet when Sync is used. Otherwise
        # _sync_position() resolves these by HIP number (see _cal_star_hip below) straight from
        # the same catalog entry the star's own dot on the viz is drawn from - these hand-entered
        # values were each off from the catalog's own by a couple arcseconds (a different amount
        # per star, since they came from a separate source than the bundled catalog), enough to
        # show as a visible misalignment between the synced target cross and the star at high
        # zoom even though picking the same star via search (which reads the catalog directly)
        # lined up correctly.
        # "East"/"West" are the two horizon calibration points (DEC=0, i.e. celestial equator
        # crossing the horizon - same points autoCalibrateFromHome() uses for the mount's own
        # home-position calibration in the .ino, see its comment: due East is Hour Angle -90,
        # due West is Hour Angle +90, so RA = LST -+ 90 respectively) - useful for re-syncing
        # against the horizon itself (e.g. after a manual re-home) rather than needing a visible
        # star. Unlike the named stars below, their RA isn't a fixed J2000 value - it depends on
        # the current LST - so these are stored as sentinel strings, not (ra, dec) tuples, and
        # resolved at sync time in _sync_position().
        self.cal_stars = {
            "East (horizon, DEC=0)": "EAST",
            "West (horizon, DEC=0)": "WEST",
            "Vega (alpha Lyrae)": (279.234, 38.784),
            "Arcturus (alpha Bootis)": (213.915, 19.182),
            "Altair (alpha Aquilae)": (297.696, 8.868),
            "Custom (use fields below)": None,
        }
        # HIP (Hipparcos) catalog numbers for the same named stars above, so _sync_position() can
        # look up their exact RA/DEC from self._star_by_hip (same source _draw_sky_objects draws
        # from) instead of relying on the approximate hand-entered values in cal_stars.
        self._cal_star_hip = {
            "Vega (alpha Lyrae)": 91262,
            "Arcturus (alpha Bootis)": 69673,
            "Altair (alpha Aquilae)": 97649,
        }

        self._build_ui()
        # Starts disconnected - every Arduino-interacting control should start disabled rather
        # than clickable-but-silently-no-op (see _set_arduino_controls_enabled).
        self._set_arduino_controls_enabled(False)
        # Needs self.status_text (created in _build_ui) for _log() to work - must come after it.
        self._load_sky_catalog_async()
        self._poll_queue()
        self._start_time_sync_timer()
        self._start_ground_update_timer()
        self._start_viz_follow_timer()
        self._start_time_travel_label_timer()
        # Sun/Moon/planets are always on (see the comment near their state above) - kicked off
        # once here, same as the sky catalog/ground timer, rather than needing a toggle press.
        self._solar_system_update_tick()

        # Live update of viz horizon when latitude input changes
        self.lat_var.trace_add("write", self._on_location_var_changed)
        self.lon_var.trace_add("write", self._on_location_var_changed)
        self.gps_var.trace_add("write", self._on_location_var_changed)
        self.gps_mode_var.trace_add("write", self._on_location_var_changed)

    # ---------------- UI CONSTRUCTION ----------------
    def _build_ui(self):
        # Top bar - Connection
        top = ctk.CTkFrame(self, corner_radius=8)
        top.pack(fill="x", padx=12, pady=(12, 6))

        # Streamer mode - packed first with side="right" so it lands at the far right end of
        # this same Connection row, rather than a separate row of its own. Hides the GPS
        # location display (lat/lon/GPS entries + the "Loc: ..." summary label) without touching
        # the underlying stored values, so it's safe to flip on/off mid-session without needing
        # to re-enter or re-send the location.
        self.streamer_mode_var = ctk.BooleanVar(value=False)
        self.streamer_mode_switch = ctk.CTkSwitch(
            top, text="Streamer Mode (hide GPS)", variable=self.streamer_mode_var,
            command=self._on_streamer_mode_toggle
        )
        self.streamer_mode_switch.pack(side="right", padx=10)

        ctk.CTkLabel(top, text="Connection", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=10)

        self.port_combo = ctk.CTkComboBox(top, values=["Select port..."], width=280)
        self.port_combo.pack(side="left", padx=6)
        self._refresh_ports()

        ctk.CTkButton(top, text="Refresh", width=80, command=self._refresh_ports).pack(side="left", padx=4)
        self.connect_btn = ctk.CTkButton(top, text="Connect", width=90, command=self._connect)
        self.connect_btn.pack(side="left", padx=4)
        self.disconnect_btn = ctk.CTkButton(top, text="Disconnect", width=90, fg_color="#8B0000", command=self._disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=4)

        self.conn_status = ctk.CTkLabel(top, text="● Disconnected", text_color="gray", font=ctk.CTkFont(size=13))
        self.conn_status.pack(side="left", padx=16)

        # GPS input mode toggle
        self.gps_mode_switch = ctk.CTkSwitch(
            top, text="lat, lon (GPS / Google Maps format)", variable=self.gps_mode_var,
            command=self._on_gps_mode_toggle
        )
        self.gps_mode_switch.pack(side="left", padx=8)

        # Lat/Lon separate inputs (default mode)
        self.latlon_frame = ctk.CTkFrame(top, fg_color="transparent")
        ctk.CTkLabel(self.latlon_frame, text="Lat:").pack(side="left", padx=(0,0))
        self.lat_entry = ctk.CTkEntry(self.latlon_frame, textvariable=self.lat_var, width=50)
        self.lat_entry.pack(side="left", padx=1)
        ctk.CTkLabel(self.latlon_frame, text="Lon:").pack(side="left", padx=1)
        self.lon_entry = ctk.CTkEntry(self.latlon_frame, textvariable=self.lon_var, width=50)
        self.lon_entry.pack(side="left", padx=1)
        self.latlon_frame.pack(side="left", padx=2)

        # Single Google Maps GPS input
        self.gps_frame = ctk.CTkFrame(top, fg_color="transparent")
        ctk.CTkLabel(self.gps_frame, text="GPS:").pack(side="left")
        self.gps_entry = ctk.CTkEntry(self.gps_frame, textvariable=self.gps_var, width=150)
        self.gps_entry.pack(side="left", padx=1)

        self.set_loc_btn = ctk.CTkButton(top, text="Set", width=35, command=self._set_location)
        self.set_loc_btn.pack(side="left", padx=2)

        self.location_label = ctk.CTkLabel(top, text="Loc: not set", font=ctk.CTkFont(size=11), text_color="#8888aa")
        self.location_label.pack(side="left", padx=8)

        # Ensure correct initial input mode is visible
        if self.gps_mode_var.get():
            self.latlon_frame.pack_forget()
            self.gps_frame.pack(side="left", padx=4)
        else:
            self.gps_frame.pack_forget()
            self.latlon_frame.pack(side="left", padx=4)

        # If config was loaded with saved GPS coordinate, show the single GPS value in the label
        if getattr(self, '_config_loaded', False):
            try:
                g = self.gps_var.get().strip()
                if g:
                    self._set_location_label(f"Loc: {g} (loaded)")
            except Exception:
                pass

        # Time Travel: preview Sun/Moon/planet/star/DSO positions at a chosen date/time instead of
        # right now (see _time_travel_offset's comment for what this does/doesn't affect).
        time_travel_bar = ctk.CTkFrame(self, corner_radius=6, fg_color="transparent")
        time_travel_bar.pack(fill="x", padx=12, pady=(0, 4))

        ctk.CTkLabel(time_travel_bar, text="\U0001F550 Time Travel (UTC):", font=ctk.CTkFont(size=12, weight="bold")).pack(side="left", padx=(10, 6))

        self.time_travel_date_var = ctk.StringVar()
        self.time_travel_time_var = ctk.StringVar()
        self._reset_time_travel_inputs_to_now()

        self.time_travel_date_entry = ctk.CTkEntry(time_travel_bar, textvariable=self.time_travel_date_var, width=90, placeholder_text="YYYY-MM-DD")
        self.time_travel_date_entry.pack(side="left", padx=2)
        self.time_travel_time_entry = ctk.CTkEntry(time_travel_bar, textvariable=self.time_travel_time_var, width=75, placeholder_text="HH:MM:SS UTC")
        self.time_travel_time_entry.pack(side="left", padx=2)
        # Enter in either field applies it immediately, same commit pattern as other entries
        # in this GUI (e.g. cam_rot_entry) rather than requiring a mouse click on Preview.
        self.time_travel_date_entry.bind("<Return>", lambda e: self._apply_time_travel())
        self.time_travel_time_entry.bind("<Return>", lambda e: self._apply_time_travel())

        ctk.CTkButton(time_travel_bar, text="Preview", width=70, command=self._apply_time_travel).pack(side="left", padx=4)
        ctk.CTkButton(time_travel_bar, text="Now (Real Time)", width=120, fg_color="#555555", command=self._reset_time_travel).pack(side="left", padx=4)

        self.time_travel_label = ctk.CTkLabel(time_travel_bar, text="Showing: real-time sky", font=ctk.CTkFont(size=11), text_color="#8888aa")
        self.time_travel_label.pack(side="left", padx=10)

        # Prominent system status at the very top (clear state)
        status_bar = ctk.CTkFrame(self, corner_radius=6, fg_color="#1a1a2e")
        status_bar.pack(fill="x", padx=12, pady=(0, 4))
        self.status_display = ctk.CTkLabel(
            status_bar,
            text="STATUS: DISCONNECTED",
            font=ctk.CTkFont(size=18, weight="bold"),
            text_color="#FF4444"
        )
        self.status_display.pack(pady=6)

        # Main area - two columns
        main = ctk.CTkFrame(self)
        main.pack(fill="both", expand=True, padx=12, pady=6)

        # LEFT: Position + Visualization
        left = ctk.CTkFrame(main, corner_radius=8)
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        ctk.CTkLabel(left, text="Telescope Position (Sky)", font=ctk.CTkFont(size=15, weight="bold")).pack(pady=(8, 4))

        # Big numeric displays - Sky (celestial) vs Mount (physical drive)
        pos_frame = ctk.CTkFrame(left)
        pos_frame.pack(fill="x", padx=16, pady=8)

        # RA row with integrated offset control
        ra_row = ctk.CTkFrame(pos_frame)
        ra_row.pack(fill="x", pady=1)
        ctk.CTkLabel(ra_row, text="Sky RA:", width=65, font=ctk.CTkFont(size=13)).pack(side="left")
        self.ra_label = ctk.CTkLabel(ra_row, text="000.000000°", font=ctk.CTkFont(size=18, weight="bold"), text_color="#00FFAA", width=110, anchor="e")
        self.ra_label.pack(side="left", padx=4)

        ctk.CTkLabel(ra_row, text="|", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left", padx=8)
        ctk.CTkLabel(ra_row, text="Mount (phys):", width=80, font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.mount_ra_label = ctk.CTkLabel(ra_row, text="0.0000°", font=ctk.CTkFont(size=13), text_color="#aaffaa", width=70, anchor="e")
        self.mount_ra_label.pack(side="left", padx=(0, 12))

        ctk.CTkLabel(ra_row, text="|", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left", padx=8)
        ctk.CTkLabel(ra_row, text="Speed:", width=50, font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.speed_ra_label = ctk.CTkLabel(ra_row, text="0.0000 °/s", font=ctk.CTkFont(size=13), text_color="#ffaa88", width=80, anchor="e")
        self.speed_ra_label.pack(side="left", padx=(0, 8))

        # RA Offset control integrated in the same line. Inverted per request: LEFT decreases,
        # RIGHT increases.
        ctk.CTkLabel(ra_row, text="Offset:", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.ra_offset_left_btn = ctk.CTkButton(ra_row, text="◀", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(-1, 0))
        self.ra_offset_left_btn.pack(side="left", padx=(2, 0))
        self.ra_offset_var = ctk.DoubleVar(value=0.0)
        self.ra_offset_entry = ctk.CTkEntry(ra_row, textvariable=self.ra_offset_var, width=50, font=ctk.CTkFont(size=11))
        self.ra_offset_entry.pack(side="left", padx=(4, 0))
        self.ra_offset_right_btn = ctk.CTkButton(ra_row, text="▶", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(1, 0))
        self.ra_offset_right_btn.pack(side="left", padx=(2, 4))

        # DEC row with integrated offset control
        dec_row = ctk.CTkFrame(pos_frame)
        dec_row.pack(fill="x", pady=1)
        ctk.CTkLabel(dec_row, text="Sky DEC:", width=65, font=ctk.CTkFont(size=13)).pack(side="left")
        self.dec_label = ctk.CTkLabel(dec_row, text="+00.000000°", font=ctk.CTkFont(size=18, weight="bold"), text_color="#00FFAA", width=110, anchor="e")
        self.dec_label.pack(side="left", padx=4)

        ctk.CTkLabel(dec_row, text="|", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left", padx=8)
        ctk.CTkLabel(dec_row, text="Mount (phys):", width=80, font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.mount_dec_label = ctk.CTkLabel(dec_row, text="0.0000°", font=ctk.CTkFont(size=13), text_color="#aaffaa", width=70, anchor="e")
        self.mount_dec_label.pack(side="left", padx=(0, 12))

        ctk.CTkLabel(dec_row, text="|", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left", padx=8)
        ctk.CTkLabel(dec_row, text="Speed:", width=50, font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.speed_dec_label = ctk.CTkLabel(dec_row, text="0.0000 °/s", font=ctk.CTkFont(size=13), text_color="#ffaa88", width=80, anchor="e")
        self.speed_dec_label.pack(side="left", padx=(0, 8))

        # DEC Offset control integrated in the same line. DEC is a vertical (up/down) axis,
        # so these use up/down arrows (same solid-triangle style as the RA left/right ones)
        # instead of left/right, in the same left-then-right button order.
        # Inverted per request: UP decreases, DOWN increases.
        ctk.CTkLabel(dec_row, text="Offset:", font=ctk.CTkFont(size=12), text_color="#8888aa").pack(side="left")
        self.dec_offset_up_btn = ctk.CTkButton(dec_row, text="▲", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(0, 1))
        self.dec_offset_up_btn.pack(side="left", padx=(2, 0))
        self.dec_offset_var = ctk.DoubleVar(value=0.0)
        self.dec_offset_entry = ctk.CTkEntry(dec_row, textvariable=self.dec_offset_var, width=50, font=ctk.CTkFont(size=11))
        self.dec_offset_entry.pack(side="left", padx=(4, 0))
        self.dec_offset_down_btn = ctk.CTkButton(dec_row, text="▼", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(0, -1))
        self.dec_offset_down_btn.pack(side="left", padx=(2, 4))

        # Live alignment/tracking error - updated together with position anytime (via POS)
        self.live_error_label = ctk.CTkLabel(
            pos_frame,
            text="Error: RA 0.0000° | DEC 0.0000°",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color="#ffaa00"
        )
        self.live_error_label.pack(pady=(6, 2))

        # Tracking stability indicator - based on the *trend* of the error (is it flat/shrinking
        # or actively growing), not just its instantaneous size. See _update_tracking_stability.
        # Styled as a solid-fill badge (not just colored text) with a bigger, bold font so it
        # actually catches the eye instead of blending into the rest of the readouts.
        self.stability_label = ctk.CTkLabel(
            pos_frame,
            text="TRACKING: —",
            font=ctk.CTkFont(size=17, weight="bold"),
            text_color="#ffffff",
            fg_color="#444455",
            corner_radius=8,
            width=220,
            height=30
        )
        self.stability_label.pack(pady=(2, 6))

        # Current target's object name (Vega, M31, ...) when the target was selected from the
        # sky search or by clicking/double-clicking a catalog object - see _select_sky_target and
        # target_object_name. Blank (no badge) for an arbitrary sky point (manual RA/DEC entry or
        # double-click on empty sky), since there's no name to show there. Same size as the
        # stability badge above so it's equally prominent, but no fg_color fill - it's informational,
        # not a status indicator like STABLE/SETTLING/DRIFTING.
        self.target_name_label = ctk.CTkLabel(
            pos_frame,
            text="",
            font=ctk.CTkFont(size=17, weight="bold"),
            text_color="#ffdd66"
        )
        self.target_name_label.pack(pady=(0, 6))

        # Meridian-limit countdown - separate from the main STATUS-driven status bar
        # (_update_status_display) deliberately: this updates every POS tick (~50Hz) while the
        # status bar only updates on real STATUS-line events (rare), so sharing one label would
        # mean the countdown constantly clobbers/flickers away real status messages like
        # TRACKING_STARTED or STOPPED almost as soon as they appear. Blank/hidden whenever not
        # relevant - see _update_meridian_warning.
        self.meridian_warning_label = ctk.CTkLabel(
            pos_frame,
            text="",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color="#ffaa00"
        )
        self.meridian_warning_label.pack(pady=(0, 4))

        # Spell out the actual numeric criteria behind the badge (built from the same
        # constants _update_tracking_stability uses, so it can't drift out of sync with them).
        ctk.CTkLabel(
            pos_frame,
            text=(f"Stable = error ≤ {TRACKING_STABLE_ERR_DEG:.4f}° (½ camera pixel) AND flat/"
                  f"shrinking over {TRACKING_STABILITY_WINDOW_S:.0f}s (growth ≤ "
                  f"{TRACKING_STABLE_DERIV_DEG_S:.4f}°/s)  ·  Settling = still converging  ·  "
                  f"Drifting = error growing"),
            font=ctk.CTkFont(size=9),
            text_color="#8888aa",
            wraplength=280,
            justify="center"
        ).pack(pady=(0, 4))

        # Offset increment control (shared for both RA and DEC)
        offset_inc_frame = ctk.CTkFrame(pos_frame)
        offset_inc_frame.pack(fill="x", pady=(4, 4))
        ctk.CTkLabel(offset_inc_frame, text="Offset Increment:", font=ctk.CTkFont(size=11)).pack(side="left")
        self.offset_inc_var = ctk.StringVar(value="0.01")
        self.offset_inc_entry = ctk.CTkEntry(offset_inc_frame, textvariable=self.offset_inc_var, width=60, font=ctk.CTkFont(size=11))
        self.offset_inc_entry.pack(side="left", padx=(4, 0))
        ctk.CTkLabel(offset_inc_frame, text="°", font=ctk.CTkFont(size=11), text_color="#8888aa").pack(side="left", padx=(2, 0))
        ctk.CTkLabel(offset_inc_frame, text="(Adjust RA/DEC offsets using < > buttons)", 
                     font=ctk.CTkFont(size=9), text_color="#8888aa").pack(side="left", padx=(8, 0))

        # Viz toolbar - view mode cycle button (Free / follow telescope / follow target)
        viz_toolbar = ctk.CTkFrame(left, fg_color="transparent")
        viz_toolbar.pack(fill="x", padx=4)
        self.viz_mode_btn = ctk.CTkButton(viz_toolbar, text="View: Free", height=22, width=160,
                                          font=ctk.CTkFont(size=11), command=self._cycle_viz_view_mode)
        self.viz_mode_btn.pack(side="left")

        self.iss_toggle_btn = ctk.CTkButton(viz_toolbar, text="ISS: OFF", height=22, width=90,
                                            font=ctk.CTkFont(size=11), fg_color="#444444",
                                            command=self._toggle_iss_tracking)
        self.iss_toggle_btn.pack(side="left", padx=(6, 0))

        self.const_toggle_btn = ctk.CTkButton(viz_toolbar, text="Constellations: OFF", height=22, width=140,
                                              font=ctk.CTkFont(size=11), fg_color="#444444",
                                              command=self._toggle_constellations)
        self.const_toggle_btn.pack(side="left", padx=(6, 0))

        # DSO min-apparent-size filter - see self._min_dso_size_filter_enabled's comment. The
        # threshold (camera SENSOR pixels, not viz screen pixels) is directly editable (same
        # pattern as cam_rot_entry below: type a value, Enter or click away commits it) rather
        # than fixed, since what counts as "too small to bother imaging" depends on the setup.
        self.min_size_toggle_btn = ctk.CTkButton(viz_toolbar, text="Min Size: OFF", height=22, width=110,
                                                 font=ctk.CTkFont(size=11), fg_color="#444444",
                                                 command=self._toggle_min_dso_size_filter)
        self.min_size_toggle_btn.pack(side="left", padx=(6, 0))
        self.min_dso_size_var = ctk.DoubleVar(value=self._min_dso_size_px)
        self.min_dso_size_entry = ctk.CTkEntry(viz_toolbar, textvariable=self.min_dso_size_var,
                                               width=45, font=ctk.CTkFont(size=11))
        self.min_dso_size_entry.pack(side="left", padx=(4, 0))
        self.min_dso_size_entry.bind("<Return>", self._on_min_dso_size_entry_commit)
        self.min_dso_size_entry.bind("<FocusOut>", self._on_min_dso_size_entry_commit)
        ctk.CTkLabel(viz_toolbar, text="cam-px", font=ctk.CTkFont(size=11), text_color="#8888aa").pack(side="left", padx=(2, 0))

        # Camera FOV rotation - purely a display setting to match the camera rectangle/top
        # marker overlay (see _camera_rect_points/_camera_top_marker_points) to however the
        # camera is actually mounted on the telescope; doesn't send anything to the Arduino.
        ctk.CTkLabel(viz_toolbar, text="Cam Rot:", font=ctk.CTkFont(size=11)).pack(side="left", padx=(10, 0))
        self.cam_rot_left_btn = ctk.CTkButton(viz_toolbar, text="◄", width=20, height=22,
                                              font=ctk.CTkFont(size=10, weight="bold"),
                                              command=lambda: self._adjust_camera_orientation(-15.0))
        self.cam_rot_left_btn.pack(side="left", padx=(4, 0))
        # Directly editable, same pattern as ra_offset_entry/dec_offset_entry - typing a value and
        # pressing Enter (or clicking away) sets the rotation absolutely, same as the ◄/► buttons
        # do relatively (see _adjust_camera_orientation/_on_cam_rot_entry_commit).
        self.cam_rot_var = ctk.DoubleVar(value=0.0)
        self.cam_rot_entry = ctk.CTkEntry(viz_toolbar, textvariable=self.cam_rot_var, width=45, font=ctk.CTkFont(size=11))
        self.cam_rot_entry.pack(side="left", padx=(4, 0))
        self.cam_rot_entry.bind("<Return>", self._on_cam_rot_entry_commit)
        self.cam_rot_entry.bind("<FocusOut>", self._on_cam_rot_entry_commit)
        ctk.CTkLabel(viz_toolbar, text="°", font=ctk.CTkFont(size=11), text_color="#8888aa").pack(side="left", padx=(2, 0))
        self.cam_rot_right_btn = ctk.CTkButton(viz_toolbar, text="►", width=20, height=22,
                                               font=ctk.CTkFont(size=10, weight="bold"),
                                               command=lambda: self._adjust_camera_orientation(15.0))
        self.cam_rot_right_btn.pack(side="left", padx=(4, 0))

        # Sky object search - name/Messier-id substring search over the bundled catalog (plus
        # Sun/Moon/ISS by name), live-filtered as you type. Selecting a result (double-click or
        # Enter) uses the exact same targeting logic as double-clicking the viz itself (see
        # _select_sky_target) - fills the target fields always, and also GoTos there + starts
        # sidereal tracking if nothing is tracking yet.
        search_frame = ctk.CTkFrame(left, fg_color="transparent")
        search_frame.pack(fill="x", padx=4, pady=(4, 0))
        ctk.CTkLabel(search_frame, text="Find:", font=ctk.CTkFont(size=11)).pack(side="left")
        self.sky_search_var = ctk.StringVar()
        self.sky_search_entry = ctk.CTkEntry(search_frame, textvariable=self.sky_search_var,
                                             placeholder_text="star or Messier object name...",
                                             font=ctk.CTkFont(size=11))
        self.sky_search_entry.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.sky_search_entry.bind("<Down>", self._focus_sky_search_results)
        self.sky_search_entry.bind("<Return>", self._select_first_sky_search_result)

        self.sky_search_results = tk.Listbox(left, height=5, bg="#1a1a22", fg="#dddddd",
                                             selectbackground="#2a5a8a", borderwidth=0,
                                             highlightthickness=1, highlightbackground="#333",
                                             font=("Consolas", 10))
        self.sky_search_results.bind("<Double-Button-1>", self._on_sky_search_result_chosen)
        self.sky_search_results.bind("<Return>", self._on_sky_search_result_chosen)
        # Not packed here - _update_sky_search_results shows/hides it based on whether there
        # are any results, so it doesn't permanently reserve space when the search box is empty.
        self._sky_search_matches = []
        # Added last, after every widget _update_sky_search_results touches already exists -
        # avoids any dependency on exactly when customtkinter might fire a variable write.
        self.sky_search_var.trace_add("write", lambda *_: self._update_sky_search_results())

        # Visualization canvas - fully dynamic width + height
        self.viz_canvas = ctk.CTkCanvas(left, bg="#0a0a0f", highlightthickness=1, highlightbackground="#333")
        self.viz_canvas.pack(pady=(4, 0), fill="both", expand=True)
        self.viz_canvas.bind("<Configure>", self._on_canvas_resize)

        # Zoom/pan on the sky viz. Bound to both the standard mouse wheel notch AND
        # trackpad two-finger scroll - on Windows both arrive as <MouseWheel> events, but
        # a trackpad emits many small, variable-magnitude deltas per gesture instead of one
        # fixed +-120 notch, so the handler scales the zoom by the actual delta magnitude
        # (see _on_viz_zoom) rather than treating every event as one full zoom step.
        self.viz_canvas.bind("<MouseWheel>", self._on_viz_zoom)
        # Click-drag to pan once zoomed in.
        self.viz_canvas.bind("<ButtonPress-1>", self._on_viz_pan_start)
        self.viz_canvas.bind("<B1-Motion>", self._on_viz_pan_drag)
        # Middle-click to reset to the full-sky view (moved off double-click, which now sets
        # a GoTo target/starts tracking at the clicked coordinates - see _on_viz_double_click).
        self.viz_canvas.bind("<Button-2>", self._on_viz_reset_view)
        # Double-click: set the clicked RA/DEC as the target. If not currently tracking, this
        # also GoTos there and starts sidereal tracking; if tracking is already active, it only
        # fills the target fields (no slew/mode change) so it can't interrupt an active session.
        self.viz_canvas.bind("<Double-Button-1>", self._on_viz_double_click)
        # Live RA/DEC readout under the pointer.
        self.viz_canvas.bind("<Motion>", self._on_viz_mouse_move)
        self.viz_canvas.bind("<Leave>", self._on_viz_mouse_leave)

        # Cursor position readout (below the canvas, not drawn inside it)
        self.viz_cursor_label = ctk.CTkLabel(left, text="Cursor: —", font=ctk.CTkFont(size=10),
                                             text_color="#8888aa", anchor="e")
        self.viz_cursor_label.pack(fill="x", padx=8, pady=(2, 0))

        # Initial draw (real size will come from <Configure> + scheduled redraw)
        self._draw_viz_grid()

        # Keyboard shortcuts
        self.bind("<Delete>", self._stop_tracking_key)
        self.bind("<KP_Delete>", self._stop_tracking_key)  # numpad delete / suppr
        self.bind("<Return>", self._start_tracking_key)
        self.bind("<KP_Enter>", self._start_tracking_key)  # numpad enter

        # Mode indicator
        self.mode_label = ctk.CTkLabel(left, text="MODE: SIDEREAL   |   TRACKING: OFF", font=ctk.CTkFont(size=13))
        self.mode_label.pack(pady=4)

        # RIGHT: Controls
        right = ctk.CTkFrame(main, corner_radius=8)
        right.pack(side="right", fill="both", expand=False, padx=(6, 0), ipadx=8)

        ctk.CTkLabel(right, text="Tracking Controls", font=ctk.CTkFont(size=15, weight="bold")).pack(pady=(10, 4))

        # Mode selection
        mode_frame = ctk.CTkFrame(right)
        mode_frame.pack(fill="x", padx=12, pady=4)

        ctk.CTkLabel(mode_frame, text="Tracking Mode:", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8, pady=(2,0))
        self.mode_var = ctk.StringVar(value="SIDEREAL")
        self.mode_radio_buttons = []
        for m in ["SIDEREAL", "SOLAR", "LUNAR"]:
            rb = ctk.CTkRadioButton(mode_frame, text=m, variable=self.mode_var, value=m,
                                    command=self._on_mode_changed)
            rb.pack(anchor="w", padx=16, pady=2)
            self.mode_radio_buttons.append(rb)

        # Single toggle button for start/stop
        btn_frame = ctk.CTkFrame(right)
        btn_frame.pack(fill="x", padx=12, pady=6)

        self.tracking_btn = ctk.CTkButton(
            btn_frame,
            text="▶ Start Tracking",
            height=42,
            fg_color="#006400",
            command=self._toggle_tracking
        )
        self.tracking_btn.pack(fill="x", pady=4)

        # One-shot safety move: sky DEC to SAFE_TARGET_DEC_DEG (0 deg, the celestial equator -
        # see that constant in the .ino), RA untouched, no tracking started afterward (see
        # CMD,SAFE_TARGET in the .ino - converted to a mount angle via calibration, same as GOTO's
        # DEC math, so it always means the same physical altitude regardless of mount
        # orientation). Always just this one button press, no target coordinates to pick.
        self.safe_target_btn = ctk.CTkButton(
            btn_frame,
            text="Safe Target (DEC to 0°)",
            height=28,
            fg_color="#555522",
            command=self._safe_target
        )
        self.safe_target_btn.pack(fill="x", pady=(0, 4))

        # Pure mechanical "return home": slews BOTH axes directly to mount angle 0 (CMD,HOME_AXES
        # in the .ino), bypassing sky-frame/calibration conversion entirely - unlike Safe Target
        # above (sky DEC, RA untouched), this needs no calibration and touches both axes.
        self.home_axes_btn = ctk.CTkButton(
            btn_frame,
            text="Rewind Axes (RA/DEC to 0°)",
            height=28,
            fg_color="#555522",
            command=self._home_axes
        )
        self.home_axes_btn.pack(fill="x", pady=(0, 4))

        # Manual meridian-flip toggle: the DEC axis has a hard mechanical stop at +-90° and can't
        # make the ~180° swing a real motorized flip needs, so the DEC half is done BY HAND -
        # loosen the OTA's rings/dovetail, rotate the tube 180° around the DEC axis, re-tighten -
        # and this toggle tells the firmware, which then auto-drives RA through the matching
        # 180° rotation (CMD,SET_FLIPPED - see setTelescopeFlipped() in the .ino). Always reflects
        # the last CONFIRMED state (from POS's "flipped" field or STATUS:FLIP_COMPLETE), not an
        # optimistic guess - the RA rotation takes real time, and showing "ON" before it's
        # actually settled would be misleading.
        self.flipped_toggle_btn = ctk.CTkButton(
            btn_frame,
            text="Telescope Flipped: OFF",
            height=28,
            fg_color="#444444",
            command=self._toggle_telescope_flipped
        )
        self.flipped_toggle_btn.pack(fill="x", pady=(0, 4))

        # Master on/off for the firmware's meridian-limit safety check (see meridianLimitEnabled
        # in the .ino / _toggle_meridian_limit below) - for advanced users with confirmed
        # mechanical clearance past the meridian. Defaults ON (safe); turning it OFF requires an
        # explicit confirmation dialog every time, same pattern as the solar tracking safety gate.
        self.meridian_limit_btn = ctk.CTkButton(
            btn_frame,
            text="Meridian Limit: ON",
            height=28,
            fg_color="#006400",
            command=self._toggle_meridian_limit
        )
        self.meridian_limit_btn.pack(fill="x", pady=(0, 4))

        # Sync with star selector
        sync_frame = ctk.CTkFrame(right)
        sync_frame.pack(fill="x", padx=12, pady=4)

        ctk.CTkLabel(sync_frame, text="Sync / Calibrate (choose star)", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8, pady=(2,0))

        self.sync_star_var = ctk.StringVar(value="Vega (alpha Lyrae)")
        self.sync_star_menu = ctk.CTkOptionMenu(
            sync_frame,
            values=list(self.cal_stars.keys()),
            variable=self.sync_star_var,
            width=220
        )
        self.sync_star_menu.pack(padx=8, pady=2, fill="x")

        self.sync_btn = ctk.CTkButton(sync_frame, text="Sync Position (send to Arduino)", height=32,
                                      command=self._sync_position)
        self.sync_btn.pack(fill="x", padx=8, pady=4)

        # Target input boxes for sidereal tracking.
        # Enter RA/DEC of desired star, then click Start Tracking (for SIDEREAL).
        # The values are sent via SET_TARGET so alignment uses them exactly like sun/moon targets.
        # (The separate GoTo slew button/function has been removed per request; alignment sequence
        # handles moving to the target when you Start Tracking.)
        goto_frame = ctk.CTkFrame(right)
        goto_frame.pack(fill="x", padx=12, pady=6)

        ctk.CTkLabel(goto_frame, text="Sidereal Target (degrees on sky) - used by Start Tracking", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8, pady=(2,0))

        ra_row = ctk.CTkFrame(goto_frame)
        ra_row.pack(fill="x", padx=8, pady=2)
        ctk.CTkLabel(ra_row, text="RA:", width=40).pack(side="left")
        self.goto_ra = ctk.CTkEntry(ra_row, placeholder_text="0-360")
        self.goto_ra.pack(side="left", fill="x", expand=True, padx=(4,0))
        self.goto_ra.insert(0, "279.23")

        dec_row = ctk.CTkFrame(goto_frame)
        dec_row.pack(fill="x", padx=8, pady=2)
        ctk.CTkLabel(dec_row, text="DEC:", width=40).pack(side="left")
        self.goto_dec = ctk.CTkEntry(dec_row, placeholder_text="-90 to +90")
        self.goto_dec.pack(side="left", fill="x", expand=True, padx=(4,0))
        self.goto_dec.insert(0, "38.78")

        # Add a quick time sync button
        self.time_btn = ctk.CTkButton(right, text="Resync Arduino Time to Laptop", height=26,
                                 command=lambda: self.serial.send_time_if_connected() if hasattr(self, 'serial') and self.serial else None)
        self.time_btn.pack(fill="x", padx=12, pady=(2,6))

        # Position update rate - not user-adjustable from the GUI (the GUI always requests
        # POS_UPDATE_RATE_MS on connect - see _apply_initial_rate), but the label below shows
        # whatever the Arduino actually confirms applying (see the UPDATE_RATE,MS: handler in
        # _parse_arduino_line), not just the requested constant. Used to be a configurable rate +
        # presets; removed per request so there's one fixed, known-good rate instead of a tunable
        # that could be set to something that causes problems (a very high rate was part of the
        # original vibration/jitter investigation - see the 1.8.7-1.8.9 changelog entries).
        rate_frame = ctk.CTkFrame(right)
        rate_frame.pack(fill="x", padx=12, pady=4)
        # Text is provisional (what the GUI is ABOUT to request) until the Arduino actually
        # confirms it via STATUS:UPDATE_RATE,MS: - see that handler in _parse_arduino_line, which
        # rewrites this label's text to the real confirmed value. Previously this was a static
        # string baked in at UI-build time, so it silently went stale/wrong whenever
        # POS_UPDATE_RATE_MS was changed, or never reflected what the Arduino actually applied.
        self.pos_rate_label = ctk.CTkLabel(
            rate_frame, text=f"Arduino POS Update Rate: requesting {POS_UPDATE_RATE_MS} ms (~{POSITION_BROADCAST_HZ:.0f} Hz)…",
            font=ctk.CTkFont(size=11), text_color="#8888aa")
        self.pos_rate_label.pack(anchor="w", padx=8, pady=4)

        # Arduino heavy debug dump controls (for diagnosing sync / cal / drift bugs)
        debug_frame = ctk.CTkFrame(right)
        debug_frame.pack(fill="x", padx=12, pady=4)

        ctk.CTkLabel(debug_frame, text="Arduino Debug (verbose var dump)", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8, pady=(2,0))

        dbg_row = ctk.CTkFrame(debug_frame)
        dbg_row.pack(fill="x", padx=8, pady=2)

        self.debug_toggle_btn = ctk.CTkButton(dbg_row, text="Verbose Debug: OFF", width=140, height=26,
                                              fg_color="#555555",
                                              command=self._toggle_arduino_debug)
        self.debug_toggle_btn.pack(side="left")

        self.debug_dump_btn = ctk.CTkButton(dbg_row, text="Force Dump", width=80, height=26,
                                            command=lambda: self.serial.request_debug_dump() if hasattr(self, 'serial') and self.serial and self.serial.ser and self.serial.ser.is_open else None)
        self.debug_dump_btn.pack(side="left", padx=4)

        ctk.CTkLabel(debug_frame, text="Toggles heavy dump of cal/mount/sky/target vars (~250ms when ON)", 
                     font=ctk.CTkFont(size=8), text_color="#8888aa").pack(anchor="w", padx=8, pady=(0,2))



        # Status / log area
        status_frame = ctk.CTkFrame(right)
        status_frame.pack(fill="both", expand=True, padx=12, pady=4)

        ctk.CTkLabel(status_frame, text="Status / Messages", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8)

        self.status_text = ctk.CTkTextbox(status_frame, height=160, wrap="word")
        self.status_text.pack(fill="both", expand=True, padx=8, pady=4)
        self.status_text.configure(state="disabled")

        # Bottom bar
        bottom = ctk.CTkFrame(self, height=28)
        bottom.pack(fill="x", padx=12, pady=(0, 10))
        self.error_label = ctk.CTkLabel(bottom, text="Error: —", text_color="#FFAA00", anchor="w")
        self.error_label.pack(side="left", padx=10, fill="x", expand=True)

        # Initialize port list
        self._refresh_ports()
        self._update_status_display("DISCONNECTED - Select COM port and click Connect", "#ffaa00")
        self._update_labels()   # initialize sky + mount displays + speeds
        # Initial label (will be updated properly when location is set)
        self._set_location_label(f"Loc: {self.lat_var.get()}N {self.lon_var.get()}E")
        # _update_slew_button removed (GoTo function removed)

        # Ensure viz is drawn with correct initial size (width + height)
        self.after(150, self._redraw_viz)

        # Make sure initial time will be sent on connect (plus periodic)

    def _get_effective_utc_now(self) -> datetime:
        """THE single source of "now" for this entire app - real UTC time, shifted by
        _time_travel_offset when a Time Travel preview is active (offset is zero, i.e. this is
        exactly real time, otherwise). Every position lookup reads from this one function and
        nothing else: LST (_get_current_lst_deg, so stars/DSOs alt-az follow it too), Sun/Moon/
        planets (_solar_system_update_tick), the ISS (_iss_update_tick), AND the Arduino's own
        onboard clock (SerialHandler.get_time_fn - see its constructor comment; send_time() calls
        this directly, not a separately-tracked copy). One offset, one accessor, everything
        derives from it - flipping Time Travel on/off changes what this function returns and that
        alone is enough for the whole app (GUI display and physical mount alike) to follow."""
        return datetime.now(timezone.utc) + self._time_travel_offset

    def _reset_time_travel_inputs_to_now(self):
        """Prefills the Time Travel date/time entries with the current UTC time - see
        _apply_time_travel's comment for why UTC, not local wall-clock."""
        utc_now = datetime.now(timezone.utc)
        self.time_travel_date_var.set(utc_now.strftime("%Y-%m-%d"))
        self.time_travel_time_var.set(utc_now.strftime("%H:%M:%S"))

    def _apply_time_travel(self):
        """Applies the date/time typed into the Time Travel fields - interpreted as UTC directly,
        NOT the computer's local timezone - by updating _time_travel_offset, the one single piece
        of state _get_effective_utc_now reads. UTC, not local: this is the convention every
        astronomical event (eclipses, transits, occultations - the whole point of a "type in a
        known moment and check the sky" preview) is published in, and it's what the rest of this
        app already uses (CMD,SET_TIME is UTC) - converting a typed-in UTC time through the
        computer's local timezone first (the pre-1.0.3 behavior) silently shifted it by however
        many hours local time differs from UTC, which is exactly why a solar eclipse typed in as
        its real published (UTC) totality time looked close but not exact: the actual instant
        previewed was off by the local UTC offset, e.g. 2 hours for CEST.
        Nothing else needs telling once _time_travel_offset is set: every consumer (LST/stars/
        DSOs, Sun/Moon/planets, ISS, and the Arduino's own clock via SerialHandler.get_time_fn)
        calls that same function fresh each time it needs "now", so this one assignment is the
        entire toggle. Stored as an offset from real time (not a frozen instant) so the preview
        keeps ticking forward at 1x from whatever moment was requested, counting up from the
        instant it's set, exactly like the real clock does. The Arduino gets an immediate push
        below rather than waiting for the periodic resync, so its physical pointing/tracking picks
        up the new time right away too."""
        date_str = self.time_travel_date_var.get().strip()
        time_str = self.time_travel_time_var.get().strip() or "00:00:00"
        if time_str.count(":") == 1:
            time_str += ":00"
        try:
            naive = datetime.strptime(f"{date_str} {time_str}", "%Y-%m-%d %H:%M:%S")
        except ValueError:
            self._log(f"Time Travel: invalid date/time - use YYYY-MM-DD and HH:MM:SS UTC (got '{date_str} {time_str}')")
            return
        target_utc = naive.replace(tzinfo=timezone.utc)
        self._time_travel_offset = target_utc - datetime.now(timezone.utc)
        pushed_to_arduino = self.serial.send_time_if_connected()
        self._log(f"Time Travel: previewing sky as of {naive.strftime('%Y-%m-%d %H:%M:%S')} UTC"
                   + (" - Arduino clock synced to it too" if pushed_to_arduino else
                      " - Arduino not connected, will sync on connect"))
        self._update_time_travel_label()
        self._force_solar_system_refresh()
        self._schedule_viz_redraw()

    def _reset_time_travel(self):
        """Cancels any active preview and goes back to showing the real-time sky - the same single
        _time_travel_offset assignment (this time to zero) is again the entire toggle; see
        _apply_time_travel. Also resyncs the Arduino's clock back to real time immediately."""
        self._time_travel_offset = timedelta(0)
        self.serial.send_time_if_connected()
        self._reset_time_travel_inputs_to_now()
        self._log("Time Travel: back to real-time sky (Arduino clock resynced to real time)")
        self._update_time_travel_label()
        self._force_solar_system_refresh()
        self._schedule_viz_redraw()

    def _update_time_travel_label(self):
        if not hasattr(self, 'time_travel_label') or not self.time_travel_label.winfo_exists():
            return
        if self._time_travel_offset == timedelta(0):
            self.time_travel_label.configure(text="Showing: real-time sky", text_color="#8888aa")
        else:
            eff_utc = self._get_effective_utc_now()
            connected = bool(self.serial and self.serial.ser and self.serial.ser.is_open)
            suffix = "" if connected else " - Arduino not connected"
            self.time_travel_label.configure(
                text=f"⚠ TIME TRAVEL: {eff_utc.strftime('%Y-%m-%d %H:%M:%S')} UTC{suffix}",
                text_color="#ff9944")

    def _start_time_travel_label_timer(self):
        """Keeps the Time Travel status label ticking forward once a second while a preview is
        active - separate from the 10ms _poll_queue loop (POLL_INTERVAL_MS) since a text label
        with 1-second resolution doesn't need updating anywhere near that often."""
        def tick():
            self._update_time_travel_label()
            self.after(1000, tick)
        tick()

    def _force_solar_system_refresh(self):
        """Re-fetches Sun/Moon/planet positions right away instead of waiting for the next
        scheduled _solar_system_update_tick (up to 2s away) - used after a Time Travel
        preview/reset so the jump feels immediate. Must cancel the pending scheduled call first:
        _solar_system_update_tick reschedules itself via self.after(2000, ...), so calling it
        again without cancelling would leave two independent recurring loops running forever."""
        if getattr(self, '_solar_system_update_after_id', None) is not None:
            try:
                self.after_cancel(self._solar_system_update_after_id)
            except Exception:
                pass
        self._solar_system_update_tick()

    def _get_current_lst_deg(self):
        """Approximate Local Sidereal Time in degrees using system time + longitude."""
        _, lon = self._get_lat_lon_from_input()
        utc = self._get_effective_utc_now()
        # Julian date (simplified)
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
        # GMST hours
        gmst = (18.697374558 + 24.06570982441908 * d) % 24
        lst_h = (gmst + lon / 15.0) % 24
        return lst_h * 15.0

    def _calculate_angular_distance(self, ra1, dec1, ra2, dec2):
        """Calculate the angular distance between two sky positions in degrees.
        
        Uses the haversine formula for spherical coordinates.
        Returns distance in degrees.
        """
        # Convert to radians
        ra1_rad = math.radians(ra1)
        dec1_rad = math.radians(dec1)
        ra2_rad = math.radians(ra2)
        dec2_rad = math.radians(dec2)
        
        # Haversine formula
        d_ra = ra2_rad - ra1_rad
        d_dec = dec2_rad - dec1_rad
        
        a = math.sin(d_dec/2)**2 + math.cos(dec1_rad) * math.cos(dec2_rad) * math.sin(d_ra/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return math.degrees(c)

    # ---------------- SKY VIZ ZOOM / PAN ----------------
    def _get_viz_view_span(self):
        """Currently visible (ra_span, dec_span) in degrees, given the current zoom level."""
        return 360.0 / self._viz_zoom, 180.0 / self._viz_zoom

    def _get_viz_view_bounds(self):
        """Currently visible (ra_min, ra_max, dec_min, dec_max), given zoom + pan center."""
        ra_span, dec_span = self._get_viz_view_span()
        ra_half, dec_half = ra_span / 2.0, dec_span / 2.0
        return (self._viz_center_ra - ra_half, self._viz_center_ra + ra_half,
                self._viz_center_dec - dec_half, self._viz_center_dec + dec_half)

    def _clamp_viz_center(self):
        """Keep the view window inside valid sky bounds (no RA wraparound support)."""
        ra_span, dec_span = self._get_viz_view_span()
        if ra_span >= 360.0:
            self._viz_center_ra = 180.0
        else:
            self._viz_center_ra = max(ra_span / 2.0, min(360.0 - ra_span / 2.0, self._viz_center_ra))
        if dec_span >= 180.0:
            self._viz_center_dec = 0.0
        else:
            self._viz_center_dec = max(-90.0 + dec_span / 2.0, min(90.0 - dec_span / 2.0, self._viz_center_dec))

    def _viz_ra_to_x(self, ra, w, margin, ra_min, ra_span):
        # Inverted (1.0 - fraction, not fraction) per request: RA=0 on the right, RA=360 on the
        # left, matching what the camera/real sky actually shows (a mirrored view) instead of the
        # increasing-rightward convention used before. Every star/DSO/marker/grid-line position
        # goes through this one function, so this single flip propagates everywhere consistently -
        # the only things that needed a SEPARATE, explicit fix were the few places with their own
        # independent East/West direction assumption baked in (position-angle rotation math in
        # _rotated_ellipse_points/_rotate_ne's callers, and the manual drag-pan/zoom-under-cursor
        # RA math that duplicated this formula inline instead of calling it).
        return margin + (w - 2 * margin) * (1.0 - (ra - ra_min) / ra_span)

    def _viz_dec_to_y(self, dec, h, margin, dec_min, dec_span):
        return margin + (h - 2 * margin) * (1.0 - (dec - dec_min) / dec_span)

    def _viz_x_to_ra(self, x, w, margin, ra_min, ra_span):
        return ra_min + ra_span * (1.0 - (x - margin) / (w - 2 * margin))

    def _viz_y_to_dec(self, y, h, margin, dec_min, dec_span):
        return dec_min + dec_span * (1.0 - (y - margin) / (h - 2 * margin))

    def _pick_grid_step(self, span_deg, target_lines=VIZ_GRID_TARGET_LINES):
        """'Nice number' grid spacing (1/2/3/6 * 10^n) that gives ~target_lines lines across
        span_deg. As zoom increases (span shrinks) this naturally walks 10 -> 6 -> 3 -> 2 -> 1 ->
        0.6 -> 0.3 -> ...

        Uses (1, 2, 3, 6) rather than the more textbook-standard (1, 2, 5) multiplier set,
        specifically so RA gridlines line up cleanly at the 0/360 wrap: 360 = 2^3*3^2*5, and every
        candidate this produces at the >=1 tier (10, 20, 30, 60, and their x10 repeats) divides
        360 evenly, whereas the standard (1,2,5,10) set includes 50 and 100, neither of which
        does - that's exactly what caused RA gridlines to fall at 0,50,100,...,350 instead of
        landing on 360 itself. 180 (the DEC span) divides evenly by all of these too, so DEC
        gridlines keep their existing, already-correct alignment (e.g. still 20 at the default
        full-sky zoom) - this only changes which steps get chosen when the old set would have
        picked 50/100/500/1000 etc."""
        span_deg = max(span_deg, 1e-9)
        raw_step = span_deg / target_lines
        magnitude = 10 ** math.floor(math.log10(raw_step))
        for mult in (1, 2, 3, 6, 10):
            step = mult * magnitude
            if step >= raw_step - 1e-12:
                return step
        return 10 * magnitude

    def _grid_line_style(self, value, step):
        """Classify a grid line as major/medium/minor based on alignment with coarser multiples
        of the current step, so brightness tiers scale automatically with zoom level."""
        def near_multiple(v, m):
            if m <= 1e-12:
                return False
            r = v / m
            return abs(r - round(r)) < 1e-6
        # Minor was previously "#2a2a3a" - too close in luminance to the ground fill ("#1a2e2e")
        # to read clearly against it, which is part of why grid lines looked "missing" over the
        # green area even though they were correctly stacked on top (verified with a bright
        # debug color). Bumped for better contrast against both the fill and the background.
        if abs(value) < 1e-9 or near_multiple(value, step * 10):
            return "#666688", 1.5   # major (incl. 0°)
        elif near_multiple(value, step * 5):
            return "#444455", 1.0   # medium
        else:
            return "#3a3a4d", 1.0   # minor

    def _schedule_viz_redraw(self):
        """Coalesce rapid-fire zoom/pan events into at most one redraw per tick, instead
        of a full redraw (delete("all") + recreate every grid line/label/horizon point) for
        every single wheel tick or mouse-motion event. A trackpad scroll or a drag gesture can
        fire many events per second - without this, each one triggered its own synchronous
        redraw, and since panning around is exactly what you do once zoomed in, that stacked up
        fast enough to feel like ~4fps at high zoom (the redraw work itself wasn't slower at
        high zoom, there were just far more of them queued up back to back).
        self.after(0, ...), NOT self.after_idle(...): Tk's "idle" callbacks only run once the
        event queue is genuinely empty, and while actively tracking, incoming POS lines arrive
        continuously (up to ~50-100Hz) - the queue can go long stretches without ever reaching
        that idle state, silently starving after_idle for as long as tracking keeps the queue
        busy. Reported as "the Sun/Moon dot froze while actively tracking, then jumped to the
        correct position the moment tracking stopped" - the underlying data (_solar_system_
        update_tick's background refresh) WAS updating every 2s the whole time, only the redraw
        that would show it was stuck waiting for an idle moment that heavy POS traffic kept
        postponing. self.after(0, ...) queues a normal (not idle-gated) event that runs on the
        next event-loop pass regardless of how busy the queue is - same coalescing behavior
        (still at most one pending redraw via _viz_redraw_pending), just not starvable."""
        if self._viz_redraw_pending:
            return
        self._viz_redraw_pending = True
        self.after(0, self._flush_viz_redraw)

    def _flush_viz_redraw(self):
        self._viz_redraw_pending = False
        self._redraw_viz()

    def _on_viz_zoom(self, event):
        """Mouse wheel / trackpad two-finger scroll -> zoom, centered on the cursor.

        On Windows both a physical mouse wheel notch and a trackpad scroll gesture arrive as
        <MouseWheel> events, but a trackpad sends many small, variable-size deltas per gesture
        instead of one fixed +-120 notch. Scaling the zoom factor by event.delta/120 (rather
        than applying one fixed zoom step per event) makes a slow trackpad scroll zoom gently
        and a fast flick zoom further, matching the actual gesture instead of feeling like a
        machine-gun of full zoom steps.
        """
        c = self.viz_canvas
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min

        # Data-space position under the cursor, before zooming, expressed as a fraction
        # of the current view (0..1) so it can be re-centered after the zoom changes.
        frac_x = (event.x - margin) / max(1, (w - 2 * margin))
        frac_y = (event.y - margin) / max(1, (h - 2 * margin))
        # Uses the shared helpers (not a duplicated inline formula) specifically so this can't
        # drift out of sync with _viz_ra_to_x's mirrored RA convention again in the future.
        cursor_ra = self._viz_x_to_ra(event.x, w, margin, ra_min, ra_span)
        cursor_dec = dec_min + dec_span * (1.0 - frac_y)

        steps = event.delta / 120.0
        zoom_factor = VIZ_ZOOM_STEP_BASE ** steps
        new_zoom = max(VIZ_ZOOM_MIN, min(VIZ_ZOOM_MAX, self._viz_zoom * zoom_factor))
        if new_zoom == self._viz_zoom:
            return
        self._viz_zoom = new_zoom

        # Re-center so the point under the cursor stays under the cursor.
        # (frac_x - 0.5), not (0.5 - frac_x): re-derived for _viz_ra_to_x's mirrored RA
        # convention (RA increasing leftward) - see cursor_ra's comment above.
        new_ra_span, new_dec_span = self._get_viz_view_span()
        self._viz_center_ra = cursor_ra + new_ra_span * (frac_x - 0.5)
        self._viz_center_dec = cursor_dec + new_dec_span * (0.5 - (1.0 - frac_y))
        self._clamp_viz_center()
        self._schedule_viz_redraw()

    def _on_viz_pan_start(self, event):
        self._viz_pan_last_xy = (event.x, event.y)

    def _on_viz_pan_drag(self, event):
        if self._viz_pan_last_xy is None or self._viz_zoom <= VIZ_ZOOM_MIN:
            self._viz_pan_last_xy = (event.x, event.y)
            return
        # A manual drag means the user wants to look somewhere else - drop out of
        # telescope/target-follow mode back to free view so the drag isn't immediately
        # overwritten by the next follow-mode tick.
        if self._viz_view_mode != "FREE":
            self._viz_view_mode = "FREE"
            self.viz_mode_btn.configure(text="View: Free")
        c = self.viz_canvas
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12
        ra_span, dec_span = self._get_viz_view_span()
        last_x, last_y = self._viz_pan_last_xy
        dx, dy = event.x - last_x, event.y - last_y
        # + dx, not -: RA now increases leftward on screen (see _viz_ra_to_x's mirrored
        # convention), so dragging right (dx>0) must INCREASE center_ra to keep the content
        # following the cursor the same intuitive way it did before the flip.
        self._viz_center_ra += dx * ra_span / max(1, (w - 2 * margin))
        self._viz_center_dec += dy * dec_span / max(1, (h - 2 * margin))
        self._clamp_viz_center()
        self._viz_pan_last_xy = (event.x, event.y)
        self._schedule_viz_redraw()

    def _on_viz_reset_view(self, event=None):
        self._viz_zoom = VIZ_ZOOM_MIN
        self._viz_center_ra = 180.0
        self._viz_center_dec = 0.0
        self._viz_view_mode = "FREE"
        self.viz_mode_btn.configure(text="View: Free")
        self._redraw_viz()

    def _build_milky_way_bands(self):
        """Precomputes, once, the (RA, Dec) quad segments for each nested Milky Way band (see
        MILKY_WAY_BAND_HALF_WIDTHS_DEG/_galactic_to_radec) - fixed in equatorial coordinates
        (unlike the horizon or meridian-limit zone, the galactic plane doesn't move with time), so
        there's no reason to redo this trig on every redraw. Called lazily from _draw_milky_way
        the first time it's needed and cached in self._milky_way_bands.

        Each band is a list of quads (4 (ra,dec) point tuples each), one quad per adjacent pair of
        sampled galactic-longitude points, covering the ring l=[i*STEP, (i+1)*STEP] between the
        band's +halfwidth and -halfwidth edges. Built as many small independent quads rather than
        one big polygon per band specifically so the RA=0/360 seam (crossed once per full lap of
        l) doesn't need special-case splitting - a quad whose two longitude samples land on
        opposite sides of the seam just gets skipped (a ~4 degree gap in a soft glow band is
        visually negligible), instead of drawing one huge wraparound chord across the whole
        canvas."""
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
                # Skip quads whose samples straddle the RA=0/360 seam - see docstring.
                if abs(ra_t1 - ra_t0) > 180.0 or abs(ra_b1 - ra_b0) > 180.0:
                    continue
                quads.append(((ra_t0, dec_t0), (ra_t1, dec_t1), (ra_b1, dec_b1), (ra_b0, dec_b0)))
            bands.append((color, quads))
        return bands

    def _draw_milky_way(self, c, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        """Draws the schematic Milky Way band - see MILKY_WAY_BAND_HALF_WIDTHS_DEG's comment for
        why it's an analytic galactic-coordinate band rather than a real brightness image, and
        _build_milky_way_bands for how the underlying (RA,Dec) geometry is built (once, cached,
        reused every redraw). Widest/faintest band drawn first, narrowest/brightest last, so they
        naturally layer into a glow that's strongest right at the galactic plane."""
        if not hasattr(self, "_milky_way_bands"):
            self._milky_way_bands = self._build_milky_way_bands()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        # Generous margin so a quad with one corner just outside the visible window (common, since
        # quads are ~4 degrees wide) doesn't get culled before its other corners are checked -
        # cheap bounding-box overlap test, not exact clipping (create_polygon handles off-canvas
        # coordinates fine on its own, this is purely to skip trig/draw calls for whole bands/
        # quads nowhere near the current view).
        pad = 10.0
        for color, quads in self._milky_way_bands:
            for quad in quads:
                quad_ra_min = min(p[0] for p in quad)
                quad_ra_max = max(p[0] for p in quad)
                quad_dec_min = min(p[1] for p in quad)
                quad_dec_max = max(p[1] for p in quad)
                if quad_ra_max < ra_min - pad or quad_ra_min > ra_max + pad:
                    continue
                if quad_dec_max < dec_min - pad or quad_dec_min > dec_max + pad:
                    continue
                pts = []
                for ra, dec in quad:
                    pts.append(self._viz_ra_to_x(ra, w, margin, ra_min, ra_span))
                    pts.append(self._viz_dec_to_y(dec, h, margin, dec_min, dec_span))
                c.create_polygon(pts, fill=color, outline="", tags="milky_way")

    def _draw_meridian_limit_zone(self, c, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        """Shades the RA band currently past the meridian limit (see MERIDIAN_LIMIT_MARGIN_DEG/
        pastMeridianLimit() in the .ino - the zone tracking auto-stops at, and GOTO/START_TRACKING
        now only warn-and-confirm about rather than refusing outright, since this mount's DEC axis
        can't make the ~180deg swing a real flip would otherwise need) in orange - "risky, will
        ask for confirmation", not "impossible" (see _confirm_risky_slew) - same rendering
        approach as _draw_horizon's ground shading (a solid, subtle-colored polygon), but
        geometrically simpler: Hour Angle
        only depends on RA, not DEC, so this isn't a latitude-dependent curve like the horizon -
        it's a plain vertical band spanning the full visible DEC range, from RA=(LST-180) to
        RA=LST (the 180-degree-wide "already past meridian" half of the sky), sweeping around as
        LST advances. Does NOT disappear once flipped - the collision constraint is symmetric
        (each pier side is only safe for roughly half the sky), so the band mirrors to the other
        half of RA instead (LST to LST+180) once flipped, matching the .ino's pastMeridianLimit()
        mirroring the same way for the same reason - see its comment. Excludes the
        MERIDIAN_LIMIT_DEC_ALLOWED_MIN/MAX_DEG band (movement is allowed there regardless of HA -
        see that constant's comment), so this is now up to two horizontal strips (above and below
        the allowed band) per RA sub-interval, not one full-height rectangle."""
        lst = self._get_current_lst_deg()
        if self._telescope_flipped:
            danger_lo, danger_hi = lst % 360.0, (lst + 180.0) % 360.0
        else:
            danger_lo, danger_hi = (lst - 180.0) % 360.0, lst % 360.0
        # 180-degree band may wrap across the RA=0/360 seam - split into up to two non-wrapping
        # sub-intervals, same technique used elsewhere in this file for RA wraparound.
        intervals = [(danger_lo, danger_hi)] if danger_lo <= danger_hi \
            else [(danger_lo, 360.0), (0.0, danger_hi)]
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        # DEC sub-bands still in danger: above the allowed band and below it, each clipped to the
        # visible DEC range. Skipped individually if the allowed band fully covers/exceeds it.
        dec_bands = []
        upper_lo = max(MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG, dec_min)
        if upper_lo < dec_max:
            dec_bands.append((upper_lo, dec_max))
        lower_hi = min(MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG, dec_max)
        if dec_min < lower_hi:
            dec_bands.append((dec_min, lower_hi))
        for lo, hi in intervals:
            clip_lo, clip_hi = max(lo, ra_min), min(hi, ra_max)
            if clip_lo >= clip_hi:
                continue
            x_lo = self._viz_ra_to_x(clip_lo, w, margin, ra_min, ra_span)
            x_hi = self._viz_ra_to_x(clip_hi, w, margin, ra_min, ra_span)
            # min()/max() on the pixel coords, not the RA values - RA increases leftward on
            # screen (_viz_ra_to_x's mirrored convention), so x_lo isn't necessarily the
            # visually-left edge.
            x_left, x_right = min(x_lo, x_hi), max(x_lo, x_hi)
            for dec_lo, dec_hi in dec_bands:
                y_top = self._viz_dec_to_y(dec_hi, h, margin, dec_min, dec_span)
                y_bot = self._viz_dec_to_y(dec_lo, h, margin, dec_min, dec_span)
                c.create_rectangle(x_left, y_top, x_right, y_bot,
                                   fill="#3a2814", outline="", tags="meridian_limit")

    def _draw_horizon(self, c, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        """Shade the region below the horizon and draw the horizon curve (simple approx)."""
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        lat, _ = self._get_lat_lon_from_input()
        lst = self._get_current_lst_deg()

        def _horizon_dec_for_ra(ra):
            ha = (lst - ra) % 360.0
            ha_rad = math.radians(ha)
            lat_rad = math.radians(lat)
            if abs(math.sin(lat_rad)) < 1e-8:
                return 0.0
            tan_dec = -(math.cos(lat_rad) / math.sin(lat_rad)) * math.cos(ha_rad)
            return max(-90.0, min(90.0, math.degrees(math.atan(tan_dec))))

        # Only sample the horizon curve across the currently visible RA window (plus a small
        # margin so the line still reaches the canvas edges), instead of always sweeping the
        # full 0-360 sky. At high zoom the visible span can be a few degrees or less, so this
        # collapses what used to be a fixed 121 samples - and the resulting huge, mostly
        # off-canvas polygon built from them - down to just a handful of points. That fixed
        # full-sky sampling was the dominant cost behind zoomed-in redraws being laggy: the
        # trig work didn't shrink with zoom, and the far-off-window points produced enormous
        # pixel coordinates (proportional to zoom level) that Tk still had to build a stippled
        # polygon around every redraw.
        # Sampling RA directly (rather than sweeping hour angle like the old fixed-grid version
        # did) also means points come out already sorted left-to-right, no separate sort needed.
        margin_deg = max(1.0, ra_span * 0.05)
        sample_start = max(0.0, ra_min - margin_deg)
        sample_end = min(360.0, ra_max + margin_deg)
        # ~20 samples across the visible window is plenty (the horizon curve is very close to
        # linear over any span this small), but never coarser than the original 3 deg spacing
        # so a full-sky view still looks exactly as smooth as before.
        step = max(0.05, min(3.0, (sample_end - sample_start) / 20.0))

        horizon_points = []
        ra = sample_start
        while ra <= sample_end + 1e-9:
            dec = _horizon_dec_for_ra(ra)
            x = self._viz_ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self._viz_dec_to_y(dec, h, margin, dec_min, dec_span)
            horizon_points.append((x, y))
            ra += step
        if len(horizon_points) < 2:
            return

        # At full-sky zoom, force the curve to touch the left/right canvas edges exactly at
        # RA=0/RA=360 (fixes a small visual gap there). When zoomed into a sub-range this
        # anchor isn't meaningful, so just let the curve run naturally off-canvas (Tk clips it).
        if ra_min <= 1e-6 and ra_max >= 360.0 - 1e-6:
            y0 = self._viz_dec_to_y(_horizon_dec_for_ra(0.0), h, margin, dec_min, dec_span)
            y360 = self._viz_dec_to_y(_horizon_dec_for_ra(360.0), h, margin, dec_min, dec_span)
            x0, x360 = margin, w - margin
        else:
            x0, y0 = horizon_points[0]
            x360, y360 = horizon_points[-1]

        # Which side of the horizon curve is actually ground depends on the observer's latitude
        # sign, not just "below" in the fixed sense of "toward lower DEC" - closing the polygon
        # toward dec_min unconditionally was wrong for negative latitudes (e.g. at the south
        # pole, DEC>0 is permanently below the horizon and DEC<0 is the visible sky - the
        # opposite of the north pole, even though the horizon curve itself is flat at DEC=0 in
        # both cases). The observer's zenith (always visible sky, by definition, at DEC=lat) is
        # the one point we know for certain is NOT ground, so compare it against the horizon
        # curve's own DEC at the observer's meridian (hour angle 0) to find which side it falls
        # on, instead of assuming based on the sign of latitude alone.
        horizon_dec_at_meridian = _horizon_dec_for_ra(lst)
        ground_toward_dec_min = lat >= horizon_dec_at_meridian
        ground_edge_y = (h - margin) if ground_toward_dec_min else margin

        # Polygon for ground: start at left horizon edge, follow curve, to right horizon edge,
        # then to whichever canvas edge (bottom or top) is actually the ground side.
        poly = [(x0, y0)] + horizon_points + [(x360, y360), (x360, ground_edge_y), (x0, ground_edge_y)]
        flat_poly = [coord for pt in poly for coord in pt]
        # Plain solid fill, not stipple="gray25" - Tk's canvas stipple fill is rasterized
        # pixel-by-pixel with no hardware acceleration, and is a well-known slow path for any
        # polygon of meaningful size. The "#1a2e2e" color is already chosen close to the
        # background to read as a subtle ground shade without needing the dithered look.
        c.create_polygon(flat_poly, fill="#1a2e2e", outline="", tags="horizon")

        # Horizon curve line that starts at left and ends at right
        line_pts = [(x0, y0)] + horizon_points + [(x360, y360)]
        flat_line = [coord for pt in line_pts for coord in pt]
        if len(flat_line) >= 4:
            c.create_line(flat_line, fill="#555577", width=1, tags="horizon")

    def _bearing_to_sun(self, obj_ra, obj_dec):
        """Direction from (obj_ra, obj_dec) toward the Sun, in the same North-through-East
        position-angle convention _rotated_ellipse_points/_draw_illuminated_disc use - shared by
        the Moon and planet phase rendering. Computed from real RA/DEC (not screen pixel
        positions, which break down right at the RA 0/360 seam - exactly where the Moon or an
        inferior planet tends to be closest to the Sun, i.e. right when getting this direction
        correct matters most for how thin/oriented the crescent looks). Falls back to an
        arbitrary bearing if the Sun's position isn't known yet (shouldn't normally happen once
        the solar-system update loop has run at least once)."""
        if self._sun_ra is None:
            return 90.0
        delta_ra = ((self._sun_ra - obj_ra + 180.0) % 360.0) - 180.0
        delta_dec = self._sun_dec - obj_dec
        return math.degrees(math.atan2(delta_ra, delta_dec)) % 360.0

    def _draw_illuminated_disc(self, c, x, y, r, bearing_deg, illum_fraction,
                                lit_color, dark_color, outline_color):
        """Phase rendering using the standard 'half-disc + terminator ellipse' technique common
        in moon-phase widgets - an ellipse of half-width r*|1-2*illum_fraction| exactly traces a
        sphere's terminator projected orthographically, so overlaying it in the dark or lit color
        (depending which side of the phase cycle we're on) carves out precisely the right
        crescent/gibbous shape at any phase, including illum_fraction=0/1 (full-width ellipse ->
        all dark/all lit) and illum_fraction=0.5 (zero-width ellipse -> exactly half-lit).
        illum_fraction alone (with bearing_deg for left/right orientation) is enough - a waxing
        and waning crescent/gibbous at the same illuminated fraction have the same |1-2*illum|
        (cosine is symmetric around the "full" point), so no separate waxing/age-angle input is
        needed the way the Moon's own phase_deg tracks (that's a superset of what shape-wise
        rendering actually needs).

        The lit side faces bearing_deg, following the same North-through-East position-angle
        convention DSO ellipse rotation uses (see _rotated_ellipse_points) - both the lit
        half-disc (via the arc's start angle, a pure rotation for a true circle) and the
        terminator ellipse (built directly from that bearing's basis vectors, rather than
        assuming a horizontal axis) are rotated to match. Used for the Moon (see _draw_moon_disc)
        and, since the same math applies to any illuminated sphere, planets too - Mercury/Venus
        show real, sometimes dramatic phases as inferior planets, Mars a slight gibbous, and the
        outer planets an imperceptible one (illum_fraction comes out ~1.0 for those - no special-
        casing which planets bother to show a visible phase)."""
        c.create_oval(x - r, y - r, x + r, y + r, fill=dark_color, outline="", tags="sky_obj")

        # tkinter's create_arc start angle is a pure rotation for a true (unstretched) circle,
        # so centering the lit half-disc on the sun's bearing is just start = bearing (not
        # -bearing - see the East-is-now-leftward comment below; East used to map to arc-angle 0
        # (screen right), so start=-bearing put the lit center at arc-angle 90-bearing, but East
        # now maps to arc-angle 180 (screen left), so the lit center needs to be at 90+bearing
        # instead, i.e. start=bearing since the lit center sits at start+90 for a 180deg extent).
        c.create_arc(x - r, y - r, x + r, y + r, start=bearing_deg, extent=180,
                    style="chord", fill=lit_color, outline="", tags="sky_obj")

        cos_phase = 1.0 - 2.0 * illum_fraction
        half_w = r * abs(cos_phase)
        terminator_color = lit_color if illum_fraction > 0.5 else dark_color
        bearing_rad = math.radians(bearing_deg)
        sin_b, cos_b = math.sin(bearing_rad), math.cos(bearing_rad)
        pts = []
        for i in range(24):
            t = 2 * math.pi * i / 24
            ct, st = math.cos(t), math.sin(t)
            # Extent half_w along the bearing direction (shrinks/grows with phase), extent r
            # along the perpendicular (the terminator's long axis, fixed at full radius). Minus,
            # not plus, on the x term - East is now leftward on screen (see _viz_ra_to_x), same
            # fix as _rotated_ellipse_points/_camera_rect_points; the y/North term is unaffected.
            pts.append(x - half_w * ct * sin_b - r * st * cos_b)
            pts.append(y - half_w * ct * cos_b + r * st * sin_b)
        c.create_polygon(pts, fill=terminator_color, outline="", tags="sky_obj")
        c.create_oval(x - r, y - r, x + r, y + r, outline=outline_color, width=1, fill="", tags="sky_obj")

    def _draw_moon_disc(self, c, x, y, r, bearing_deg):
        """Moon-specific wrapper around _draw_illuminated_disc - see that function's docstring
        for the shared rendering technique."""
        self._draw_illuminated_disc(c, x, y, r, bearing_deg, self._moon_illum_fraction,
                                     "#e8e8e0", "#1a1a22", "#888888")

    @staticmethod
    def _darken_hex_color(hex_color, factor=0.28):
        """hex_color scaled toward black, keeping factor (0-1) of its original brightness - used
        to derive a planet's unlit-side color from its own real lit-side color, so the dark side
        of its phase disc reads as a shaded version of that same body instead of an unrelated
        flat gray (which is what the Moon uses, since 'moon gray' is already a fairly neutral,
        recognizable color on its own - planets read better shaded from their own real hue)."""
        hex_color = hex_color.lstrip('#')
        r, g, b = (int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        return f"#{int(r*factor):02x}{int(g*factor):02x}{int(b*factor):02x}"

    def _dso_render_style(self, type_code):
        """Rough shape/color per OpenNGC type code - not meant to be exhaustive, just
        distinguishable at a glance. See https://github.com/mattiaverga/OpenNGC for the full type
        code list. Previously every non-galaxy/non-cluster/non-planetary-nebula type (HII,
        emission, reflection, dark nebulae, supernova remnants) shared one generic teal color,
        which was fine back when the catalog only had Messier's handful of each - now that the
        catalog covers far more of each type (see sky_catalog.py's DSO_INTERESTING_TYPES), that
        bucket needed splitting out for real. Colors loosely follow common astro-app convention:
        red for emission/HII (real H-alpha glow is red), blue for reflection (scattered starlight
        is blue), brown/gray for dark nebulae (dust silhouettes, don't glow at all), orange for
        supernova remnants, purple for planetary nebulae, orange-tan for galaxies, yellow for
        clusters, teal as the catch-all for anything else (generic "Neb", cluster+nebula combos,
        unrecognized codes)."""
        t = type_code or ""
        if t in ("OCl", "GCl"):
            return "cluster", "#ffee88"
        if t == "PN":
            return "ring", "#dd88ff"
        if t.startswith("G") and t not in ("GCl",):
            return "ellipse", "#ffaa66"
        if t in ("HII", "EmN"):
            return "ellipse", "#ff6666"
        if t == "RfN":
            return "ellipse", "#77aaff"
        if t == "DrkN":
            return "ellipse", "#998877"
        if t == "SNR":
            return "ellipse", "#ff9933"
        return "ellipse", "#66ddcc"

    def _star_mag_limit_for_zoom(self):
        """More, fainter stars as you zoom in - 'the more you zoom, the more you see'. Log-scaled
        against the zoom range so it ramps up gradually rather than jumping all at once."""
        t = math.log10(max(self._viz_zoom, VIZ_ZOOM_MIN)) / math.log10(VIZ_ZOOM_MAX)
        t = max(0.0, min(1.0, t))
        return 3.2 + t * (sky_catalog.STAR_MAG_LIMIT - 3.2)

    def _rotated_ellipse_points(self, cx, cy, half_major, half_minor, pos_ang_deg, n=28):
        """Points (flattened x,y list) for an ellipse rotated by pos_ang_deg - standard
        astronomical position angle convention: measured from North, increasing towards East.
        In this canvas, North is 'up' (dec increases upward - see _viz_dec_to_y) and East is
        'left' (ra increases leftward - see _viz_ra_to_x's mirrored convention), so an east
        offset subtracts from x (cx - east, not cx + east) to keep the ellipse's orientation
        correctly matched to the mirrored RA axis - it only otherwise takes care with the sign of
        the y-term, since screen y grows downward while DEC/North grows upward."""
        pa = math.radians(pos_ang_deg)
        sin_pa, cos_pa = math.sin(pa), math.cos(pa)
        pts = []
        for i in range(n):
            t = 2 * math.pi * i / n
            ct, st = math.cos(t), math.sin(t)
            east = half_major * ct * sin_pa + half_minor * st * cos_pa
            north = half_major * ct * cos_pa - half_minor * st * sin_pa
            pts.append(cx - east)  # East is now leftward on screen (see _viz_ra_to_x)
            pts.append(cy - north)  # screen y grows downward, North grows upward
        return pts

    def _draw_sky_objects(self, c, w, h, margin, ra_min, ra_max, dec_min, dec_max):
        """Draw catalog stars (colored by B-V, sized by brightness) and Messier DSOs (rough
        shape by type, oriented by real position angle when known, from apparent size when
        large enough to matter at the current zoom). Both are culled to the visible RA/DEC
        window first - with ~870000 stars in the bundled catalog (see sky_catalog.py's AT-HYG
        module docstring), iterating all of them unfiltered on every redraw would be the dominant
        cost by a wide margin, the same class of problem the horizon curve sampling fix (see
        _draw_horizon) already solved for a different piece of this canvas - see
        STAR_LOD_TIER_MAG_CUTOFFS's comment for how the star loop specifically stays cheap even
        at full-sky zoom despite the catalog's size.

        Also records each drawn object's screen position/name/hit-radius in
        self._visible_object_hits, so _on_viz_mouse_move can hit-test against just what's
        currently on screen instead of the full star catalog on every mouse movement.
        The ISS/Sun/Moon markers (drawn separately, see _draw_viz_grid) append to this same list."""
        self._visible_object_hits = []
        if not self._sky_stars and not self._sky_dso:
            return
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        pixels_per_deg = (h - 2 * margin) / dec_span
        mag_limit = self._star_mag_limit_for_zoom()

        # Constellation lines (toggle, see _toggle_constellations) - drawn before the stars so
        # the star dots render on top of the connecting lines rather than the lines covering
        # them. Only ~700 segments total, so no need for the bounding-box culling the star loop
        # below does for its ~62000 entries - just draw them all directly each redraw.
        if self._constellations_enabled:
            for hip_a, hip_b in self._const_line_segments:
                star_a = self._star_by_hip.get(hip_a)
                star_b = self._star_by_hip.get(hip_b)
                if not star_a or not star_b:
                    continue
                ax = self._viz_ra_to_x(star_a["ra"], w, margin, ra_min, ra_span)
                ay = self._viz_dec_to_y(star_a["dec"], h, margin, dec_min, dec_span)
                bx = self._viz_ra_to_x(star_b["ra"], w, margin, ra_min, ra_span)
                by = self._viz_dec_to_y(star_b["dec"], h, margin, dec_min, dec_span)
                c.create_line(ax, ay, bx, by, fill="#4a5a7a", width=1, tags="sky_obj")

        # Pick the smallest pre-built magnitude tier that's still guaranteed to contain every
        # star the current zoom level could show (see STAR_LOD_TIER_MAG_CUTOFFS's comment) - at
        # full-sky zoom this picks the ~200-star tier instead of bisecting/scanning the full
        # ~870000-star list just to discard 99.9% of it on magnitude alone. Falls back to the
        # full list (last tier, cutoff=inf) if star_tiers isn't built yet for some reason.
        tier_stars_by_ra, tier_ra_values = self._sky_stars_by_ra, self._sky_stars_ra_values
        for cutoff, tier_stars, tier_ra in self._star_tiers:
            if cutoff >= mag_limit:
                tier_stars_by_ra, tier_ra_values = tier_stars, tier_ra
                break

        # Bisect straight to the visible RA band within that tier (each tier is RA-sorted, same
        # as the full list - see _build_sky_catalog_state) instead of scanning all of it. Dec
        # still isn't sorted, so it's still checked per-item below, but for a zoomed-in view this
        # already cuts the scanned set roughly to (RA span/360) of the tier.
        ra_lo = bisect.bisect_left(tier_ra_values, ra_min)
        ra_hi = bisect.bisect_right(tier_ra_values, ra_max)
        for s in tier_stars_by_ra[ra_lo:ra_hi]:
            if s["mag"] > mag_limit:
                continue
            ra, dec = s["ra"], s["dec"]
            if not (dec_min <= dec <= dec_max):
                continue
            x = self._viz_ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self._viz_dec_to_y(dec, h, margin, dec_min, dec_span)
            # Brighter (lower/more negative mag) -> bigger dot. Clamped to a small pixel range -
            # this is a schematic reticle, not a realistic star field render.
            radius = max(0.6, min(3.0, (mag_limit - s["mag"]) * 0.5 + 0.6))
            c.create_oval(x - radius, y - radius, x + radius, y + radius,
                          fill=s["color"], outline="", tags="sky_obj")
            # Most of the AT-HYG catalog's ~870000 stars (see sky_catalog.py) have a blank
            # "name" (no curated proper/Bayer/Flamsteed name - see _build_stars()) but DO have
            # at least one alias (TYC/HD/HIP/Gliese - verified directly against the built
            # catalog: every single blank-name star has one), so falling straight through to a
            # generic "unnamed star" without checking aliases first showed that placeholder for
            # the vast majority of stars instead of their real catalog designation.
            hip = s.get("hip")
            aliases = s.get("aliases") or []
            name = s.get("name") or (f"HIP {hip}" if hip else None) or (aliases[0] if aliases else None) or "unnamed star"
            self._visible_object_hits.append({
                "x": x, "y": y, "ra": ra, "dec": dec, "radius": 12.0, "name": name,
                "extra": f"mag {s['mag']:.1f}", "labeled": False,  # stars never have a permanent label
            })

        for d in self._sky_dso:
            ra, dec = d["ra"], d["dec"]
            if not (ra_min <= ra <= ra_max and dec_min <= dec <= dec_max):
                continue
            x = self._viz_ra_to_x(ra, w, margin, ra_min, ra_span)
            y = self._viz_dec_to_y(dec, h, margin, dec_min, dec_span)
            shape, color = self._dso_render_style(d["type"])
            maj = d.get("size_maj_arcmin")
            min_ax = d.get("size_min_arcmin") or maj
            # Min-apparent-size clutter filter - see self._min_dso_size_filter_enabled's comment.
            # Deliberately NOT based on how big the object is drawn on the viz canvas (that
            # changes with the GUI's own arbitrary zoom level, which has nothing to do with
            # whether the object is actually a worthwhile imaging target) - instead this is the
            # object's real major-axis size projected onto the ACTUAL camera sensor
            # (CAMERA_FOV_DEG_PER_PIXEL, the same real measured optics/sensor spec used for
            # TRACKING_STABLE_ERR_DEG), a fixed physical quantity independent of any GUI setting.
            # No size data (maj is None) is treated as "unknown, so can't confirm it clears the
            # threshold" and excluded - same reasoning as before, just against the real optics
            # instead of screen pixels now.
            if self._min_dso_size_filter_enabled:
                sensor_px = (maj / 60.0 / CAMERA_FOV_DEG_PER_PIXEL) if maj else 0.0
                if sensor_px < self._min_dso_size_px:
                    continue
            if maj:
                half_w = max(3.0, (maj / 60.0 / 2.0) * pixels_per_deg)
                half_h = max(3.0, ((min_ax or maj) / 60.0 / 2.0) * pixels_per_deg)
            else:
                half_w = half_h = 4.0
            if shape == "cluster":
                c.create_oval(x - half_w, y - half_h, x + half_w, y + half_h,
                              outline=color, width=1, dash=(2, 2), fill="", tags="sky_obj")
            elif shape == "ring":
                c.create_oval(x - half_w, y - half_h, x + half_w, y + half_h,
                              outline=color, width=1.5, fill="", tags="sky_obj")
            elif half_w < 6.0 and half_h < 6.0:
                # Below ~6px on-screen, a rotated/dashed ellipse outline reads as nothing more
                # than a small dot anyway - skip the expensive 28-point rotated-ellipse polygon
                # computation (_rotated_ellipse_points) and just draw a plain filled dot. This is
                # the common case at low zoom: half_w/half_h are clamped to a 3-4px minimum (see
                # above), so at full-sky zoom essentially every one of the catalog's thousands of
                # DSOs was hitting the expensive path just to render a few-pixel dot - by far the
                # dominant cost of a full-sky redraw (confirmed via profiling).
                c.create_oval(x - half_w, y - half_h, x + half_w, y + half_h,
                              fill=color, outline="", tags="sky_obj")
            else:
                # Rotated by the object's real position angle instead of always axis-aligned,
                # and a dashed perimeter only (no fill) rather than a solid/stippled ellipse -
                # this is a schematic indicator of shape/orientation, not a realistic render.
                pts = self._rotated_ellipse_points(x, y, half_w, half_h, d.get("pos_ang_deg") or 0.0)
                c.create_polygon(pts, outline=color, width=1, fill="", dash=(3, 2), tags="sky_obj")
            # Label only once objects are large enough on-screen to have room for text, and only
            # a modest zoom in - otherwise a full-sky view would be solid text clutter (thousands
            # of DSOs all fighting for the same small area, now that the catalog covers more than
            # just the 110 Messier objects - see sky_catalog.py's DSO_INTERESTING_TYPES).
            already_labeled = half_w >= 8 and self._viz_zoom >= 3.0
            if already_labeled:
                label = d.get("name") or d["designation"]
                c.create_text(x, y - half_h - 8, text=label, fill=color,
                             font=("Consolas", 8), tags="sky_obj")

            self._visible_object_hits.append({
                "x": x, "y": y, "ra": ra, "dec": dec, "radius": max(12.0, half_w, half_h),
                "name": d.get("name") or d["designation"],
                "extra": f"{d['designation']} · {d['type']}",
                "labeled": already_labeled,
            })

    def _camera_fov_half_size(self, pixels_per_deg):
        """Half-width/half-height (px) of the camera's real angular FOV (CAMERA_FOV_W_DEG x
        CAMERA_FOV_H_DEG, measured empirically - see the constant's comment).

        Uses a SINGLE isotropic px/deg scale for both dimensions - NOT the RA-axis and DEC-axis
        scales separately. Those two only agree when the canvas is exactly 2:1 (matching the
        360:180 deg RA:DEC range); at any other aspect ratio (i.e. basically always, since this
        is a resizable panel) they differ, and using them independently silently stretched the
        rectangle away from its true 3:2 shape into whatever the canvas's aspect ratio happened
        to be - which is why it could look like ~16:9 instead of 3:2."""
        return (CAMERA_FOV_W_DEG / 2.0) * pixels_per_deg, (CAMERA_FOV_H_DEG / 2.0) * pixels_per_deg

    def _rotate_ne(self, u, v, pos_ang_deg):
        """Rotates a local (u=along-frame-"top", v=along-frame-"right") offset into screen
        (east, north) offsets, using the same position-angle convention as
        _rotated_ellipse_points (measured from North, increasing towards East) - at
        pos_ang_deg=0 the frame's "top" points North and "right" points East."""
        pa = math.radians(pos_ang_deg)
        sin_pa, cos_pa = math.sin(pa), math.cos(pa)
        east = u * sin_pa + v * cos_pa
        north = u * cos_pa - v * sin_pa
        return east, north

    def _camera_rect_points(self, cx, cy, half_w, half_h, pos_ang_deg):
        """Flattened (x1,y1,x2,y2,...) corner list for the camera FOV rectangle, rotated by
        pos_ang_deg (self._camera_orientation_deg - see _adjust_camera_orientation) about
        (cx, cy). half_w/half_h are the rectangle's East-West/North-South extents at
        pos_ang_deg=0 (top of frame = North, same as an unrotated camera)."""
        pts = []
        for u, v in ((half_h, -half_w), (half_h, half_w), (-half_h, half_w), (-half_h, -half_w)):
            east, north = self._rotate_ne(u, v, pos_ang_deg)
            pts.append(cx - east)  # East is now leftward on screen (see _viz_ra_to_x)
            pts.append(cy - north)  # screen y grows downward, North grows upward
        return pts

    def _camera_top_marker_points(self, cx, cy, half_w, half_h, pos_ang_deg):
        """Flattened corner list for a small filled triangle pointing outward from the midpoint
        of the camera rectangle's current "top" edge (see _camera_rect_points) - marks which way
        is "up" in the camera's own frame once pos_ang_deg != 0, i.e. the camera is mounted
        rotated relative to the sky rather than assumed to always be North-up."""
        tip_e, tip_n = self._rotate_ne(half_h + 10, 0, pos_ang_deg)
        l_e, l_n = self._rotate_ne(half_h, -6, pos_ang_deg)
        r_e, r_n = self._rotate_ne(half_h, 6, pos_ang_deg)
        return [cx - tip_e, cy - tip_n, cx - l_e, cy - l_n, cx - r_e, cy - r_n]

    def _draw_viz_grid(self):
        """Draw a larger, clearer reticle / position visualization. Grid density adapts to
        the current zoom level: as you zoom in, coarse ~10 deg lines give way to 5, 1, 0.1 deg
        and finer, always aiming for roughly VIZ_GRID_TARGET_LINES visible lines per axis."""
        c = self.viz_canvas
        c.delete("all")

        # Get actual current size (dynamic)
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        cx, cy = w // 2, h // 2
        margin = 12

        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min

        # Everything below down through the RA/zoom labels (background, horizon, Milky Way,
        # meridian-limit shading, grid lines, stars/DSOs, ISS/Sun/Moon/planets, crosshairs,
        # cardinal lines, corner labels) is rasterized into one Pillow image and blitted as a
        # SINGLE Tk canvas item (rc.to_photoimage() below) instead of being built as
        # hundreds/thousands of individual real Tk canvas items - see _RasterCanvas's docstring
        # for why. Draw order below IS paint/stacking order (no tag_raise needed - painting into
        # an image bakes the order in directly, unlike stacked Tk canvas items).
        # bg matches the canvas widget's own bg="#0a0a0f" (see viz_canvas's creation) - it shows
        # through as the outer 8px border around the "#111118" panel rectangle drawn next.
        rc = _RasterCanvas(w, h, "#0a0a0f")
        rc.create_rectangle(8, 8, w-8, h-8, outline="#222233", width=2, fill="#111118")

        # Horizon shading (below horizon gets darker color) - the absolute furthest-back layer of
        # the three below, right above the plain background rectangle: "below the horizon" means
        # physically unreachable regardless of what else is there, so the ground should mute/sit
        # behind the Milky Way band and the meridian danger zone wherever they overlap it, not
        # paint over and hide them.
        self._draw_horizon(rc, w, h, margin, ra_min, ra_max, dec_min, dec_max)

        # Milky Way band - fixed in equatorial coordinates (doesn't depend on time/location the
        # way horizon/meridian-limit do), drawn above the ground shading so it stays visible even
        # across the horizon - see _draw_milky_way's docstring.
        self._draw_milky_way(rc, w, h, margin, ra_min, ra_max, dec_min, dec_max)

        # Meridian-limit danger zone (red/orange) - drawn last of these three so it stays visible
        # on top of both the ground shading and the Milky Way band wherever they overlap. Drawn
        # regardless of flip state - the zone mirrors to the other half of RA once flipped rather
        # than disappearing, see _draw_meridian_limit_zone's docstring.
        self._draw_meridian_limit_zone(rc, w, h, margin, ra_min, ra_max, dec_min, dec_max)

        # DEC lines (horizontal), spacing adaptive to zoom
        dec_step = self._pick_grid_step(dec_span)
        d = math.ceil(dec_min / dec_step) * dec_step
        while d <= dec_max + 1e-9:
            y = self._viz_dec_to_y(d, h, margin, dec_min, dec_span)
            col, wd = self._grid_line_style(d, dec_step)
            rc.create_line(8, y, w-8, y, fill=col, width=wd)
            d += dec_step

        # RA lines (vertical), spacing adaptive to zoom
        ra_step = self._pick_grid_step(ra_span)
        r = math.ceil(ra_min / ra_step) * ra_step
        while r <= ra_max + 1e-9:
            x = self._viz_ra_to_x(r, w, margin, ra_min, ra_span)
            col, wd = self._grid_line_style(r, ra_step)
            rc.create_line(x, 8, x, h-8, fill=col, width=wd)
            r += ra_step

        # Stars + Messier DSOs from the bundled catalog (see sky_catalog.py) - drawn on top of
        # the grid but below the crosshairs/labels/camera overlay so those stay legible.
        self._draw_sky_objects(rc, w, h, margin, ra_min, ra_max, dec_min, dec_max)

        # Real-time ISS marker (see iss_tracker.py / _iss_update_tick) - a distinct marker since
        # it's a fast-moving, ephemeral position rather than a fixed catalog object. Drawn even
        # when below the horizon (dimmer) since "the ISS is currently below your horizon" is
        # itself useful information, not something to hide.
        if self._iss_ra is not None and ra_min <= self._iss_ra <= ra_max and dec_min <= self._iss_dec <= dec_max:
            ix = self._viz_ra_to_x(self._iss_ra, w, margin, ra_min, ra_span)
            iy = self._viz_dec_to_y(self._iss_dec, h, margin, dec_min, dec_span)
            iss_color = "#00ffaa" if self._iss_above_horizon else "#336655"
            r = 4
            rc.create_line(ix - r, iy, ix + r, iy, fill=iss_color, width=1.5)
            rc.create_line(ix, iy - r, ix, iy + r, fill=iss_color, width=1.5)
            rc.create_oval(ix - r, iy - r, ix + r, iy + r, outline=iss_color, width=1.5)
            rc.create_text(ix, iy - r - 8, text="ISS", fill=iss_color, font=("Consolas", 8, "bold"))
            self._visible_object_hits.append({
                "x": ix, "y": iy, "ra": self._iss_ra, "dec": self._iss_dec, "radius": 12.0,
                "name": "ISS", "extra": "above horizon" if self._iss_above_horizon else "below horizon",
                "labeled": True,  # ISS always shows its "ISS" text label unconditionally above
            })

        # Real-time Sun/Moon (see solar_system.py / _solar_system_update_tick) - drawn at their
        # real apparent angular size (both ~0.5deg, so this only actually shows to true scale once
        # zoomed in a fair bit; a minimum pixel radius keeps them visible/clickable-looking at
        # full-sky zoom too, same LOD idea as the DSO ellipses).
        pixels_per_deg_sm = (h - 2 * margin) / dec_span
        if self._sun_ra is not None and ra_min <= self._sun_ra <= ra_max and dec_min <= self._sun_dec <= dec_max:
            sx = self._viz_ra_to_x(self._sun_ra, w, margin, ra_min, ra_span)
            sy = self._viz_dec_to_y(self._sun_dec, h, margin, dec_min, dec_span)
            sr = max(4.0, (self._sun_ang_diam_deg / 2.0) * pixels_per_deg_sm)
            rc.create_oval(sx - sr, sy - sr, sx + sr, sy + sr, fill="#ffcc33", outline="#aa7700", width=1)
            rc.create_text(sx, sy - sr - 8, text="Sun", fill="#ffcc33", font=("Consolas", 8, "bold"))
            self._visible_object_hits.append({
                "x": sx, "y": sy, "ra": self._sun_ra, "dec": self._sun_dec, "radius": max(12.0, sr),
                "name": "Sun", "extra": f"diam {self._sun_ang_diam_deg*60:.1f}'",
                "labeled": True,  # Sun always shows its "Sun" text label unconditionally above
            })
        if self._moon_ra is not None and ra_min <= self._moon_ra <= ra_max and dec_min <= self._moon_dec <= dec_max:
            mx = self._viz_ra_to_x(self._moon_ra, w, margin, ra_min, ra_span)
            my = self._viz_dec_to_y(self._moon_dec, h, margin, dec_min, dec_span)
            mr = max(4.0, (self._moon_ang_diam_deg / 2.0) * pixels_per_deg_sm)
            bearing_deg = self._bearing_to_sun(self._moon_ra, self._moon_dec)
            self._draw_moon_disc(rc, mx, my, mr, bearing_deg)
            rc.create_text(mx, my - mr - 8, text=f"Moon ({self._moon_illum_fraction*100:.0f}%)",
                          fill="#cccccc", font=("Consolas", 8, "bold"))
            waxwane = "waxing" if self._moon_waxing else "waning"
            self._visible_object_hits.append({
                "x": mx, "y": my, "ra": self._moon_ra, "dec": self._moon_dec, "radius": max(12.0, mr),
                "name": "Moon", "extra": f"{self._moon_illum_fraction*100:.0f}% illuminated, {waxwane}",
                "labeled": True,  # Moon always shows its "Moon (xx%)" text label unconditionally above
            })

        # Real-time planets (Mercury-Neptune - see solar_system.py / _solar_system_update_tick).
        # Drawn at real apparent angular size once zoomed in enough to matter, same LOD idea as
        # the Sun/Moon above and the DSO ellipses: a small fixed-radius flat-color dot at low
        # zoom (all of these are sub-arcminute at typical distances, far too small to resolve or
        # usefully phase-render there), growing to true relative scale (and, past a further
        # threshold, real illumination phase via _draw_illuminated_disc - see its docstring for
        # why the same Moon-phase math applies directly) as the real size overtakes that minimum.
        _planet_colors = {
            "Mercury": "#aaaaaa", "Venus": "#e8dcb0", "Mars": "#cc6644",
            "Jupiter": "#d8b088", "Saturn": "#e0d0a0", "Uranus": "#9fd8d8", "Neptune": "#6e8fd8",
        }
        _PLANET_MIN_RADIUS_PX = 3.5
        _PLANET_PHASE_RADIUS_PX = 6.0  # below this, a phase-rendered disc is too small to read - flat dot instead
        for pname, (pra, pdec, pang_diam, pring_ang_diam, pillum) in self._planet_positions.items():
            if not (ra_min <= pra <= ra_max and dec_min <= pdec <= dec_max):
                continue
            px = self._viz_ra_to_x(pra, w, margin, ra_min, ra_span)
            py = self._viz_dec_to_y(pdec, h, margin, dec_min, dec_span)
            pcolor = _planet_colors.get(pname, "#cccccc")
            pr = max(_PLANET_MIN_RADIUS_PX, (pang_diam / 2.0) * pixels_per_deg_sm)
            # Saturn's rings, drawn behind the disk so the disk still reads as solid on top. Real
            # angular EXTENT (see ring_angular_diameter_deg's comment in solar_system.py), but the
            # TILT is a fixed illustrative flattening (not computed from Saturn's actual ring-
            # plane opening angle, which varies over its ~29-year orbit) - good enough to read as
            # "Saturn has rings" on this schematic reticle without modeling real ring geometry.
            prr = 0.0
            if pname == "Saturn" and pring_ang_diam:
                prr = max(pr + 1.5, (pring_ang_diam / 2.0) * pixels_per_deg_sm)
                rc.create_oval(px - prr, py - prr * 0.45, px + prr, py + prr * 0.45,
                              outline=pcolor, width=max(1.0, prr * 0.12))
            if pr >= _PLANET_PHASE_RADIUS_PX:
                bearing_deg = self._bearing_to_sun(pra, pdec)
                self._draw_illuminated_disc(rc, px, py, pr, bearing_deg, pillum,
                                            pcolor, self._darken_hex_color(pcolor), pcolor)
            else:
                rc.create_oval(px - pr, py - pr, px + pr, py + pr, fill=pcolor, outline="#000000", width=0.5)
            rc.create_text(px, py - pr - 8, text=pname, fill=pcolor, font=("Consolas", 8, "bold"))
            self._visible_object_hits.append({
                "x": px, "y": py, "ra": pra, "dec": pdec, "radius": max(12.0, pr, prr),
                "name": pname, "extra": f"planet, {pillum*100:.0f}% illuminated",
                "labeled": True,  # planets always show their name label unconditionally above
            })

        # Cross hairs (center 0/0), only drawn if within the visible window
        if dec_min <= 0.0 <= dec_max:
            cy0 = self._viz_dec_to_y(0.0, h, margin, dec_min, dec_span)
            rc.create_line(8, cy0, w-8, cy0, fill="#555566", width=1)
        if ra_min <= 180.0 <= ra_max:
            cx0 = self._viz_ra_to_x(180.0, w, margin, ra_min, ra_span)
            rc.create_line(cx0, 8, cx0, h-8, fill="#555566", width=1)

        # Cardinal direction reference lines (N/S/E/W). These mark RA, not a fixed sky location -
        # they shift as sidereal time advances, same as the horizon curve. N and S are exactly
        # the meridian and anti-meridian (RA = LST and RA = LST+180, i.e. hour angle 0 and 180) -
        # every point along either of those RA lines is "currently due north/south" at whatever
        # altitude that point's DEC puts it at, which is why a simplified vertical line (not just
        # a single horizon point) is a meaningful reference here, not just a convenient shortcut.
        # E and W are the RA where the celestial equator (dec=0) crosses the horizon (hour angle
        # +-90 - a standard spherical astronomy result: the equator always crosses the horizon
        # due east and due west, regardless of latitude) - only that single point on each line is
        # exactly "at the horizon", but the line still usefully marks "the east/west region of
        # sky" the same way the N/S lines mark the meridian.
        lst = self._get_current_lst_deg()
        cardinal_ra = {"S": lst % 360.0, "N": (lst + 180.0) % 360.0,
                       "E": (lst + 90.0) % 360.0, "W": (lst - 90.0) % 360.0}
        for label, cra in cardinal_ra.items():
            if ra_min <= cra <= ra_max:
                # NOTE: intentionally not named "cx" - that name belongs to the canvas-center
                # variable (w // 2) used below for the DEC/RA-range labels, and since Python loop
                # variables aren't scoped to the loop, reusing it here was overwriting that outer
                # value with whichever cardinal line happened to be drawn last.
                line_x = self._viz_ra_to_x(cra, w, margin, ra_min, ra_span)
                rc.create_line(line_x, 8, line_x, h-8, fill="#aa8844", width=1, dash=(5, 3))
                rc.create_text(line_x, 20, text=label, fill="#ddaa55", font=("Consolas", 10, "bold"))

        # Labels - show the actual visible bounds + zoom level (updates as you zoom/pan)
        rc.create_text(18, 18, text=f"{dec_max:+.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="w")
        rc.create_text(18, h-16, text=f"{dec_min:+.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="w")
        # ra_max first, then ra_min - RA now increases leftward on screen (see _viz_ra_to_x), so
        # this reads in the same left-to-right order the values actually appear on screen, with
        # the arrow pointing towards this label's own right-edge anchor (where ra_min sits).
        rc.create_text(w-16, cy - 12, text=f"RA {ra_max:.2f}° → {ra_min:.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="e")
        rc.create_text(cx, 18, text="DEC", fill="#aaaacc", font=("Consolas", 11))
        zoom_hint = "scroll/pinch to zoom, drag to pan, middle-click to reset, double-click to set target" \
            if self._viz_zoom <= VIZ_ZOOM_MIN + 1e-6 \
            else f"zoom {self._viz_zoom:.1f}x - drag to pan, middle-click to reset, double-click to set target"
        rc.create_text(cx, h-16, text=zoom_hint, fill="#666688", font=("Consolas", 9))

        # Blit the whole rasterized background as a SINGLE Tk canvas item - see _RasterCanvas's
        # docstring. Kept referenced on self (not just a local) so Tk doesn't garbage-collect the
        # backing PhotoImage out from under the canvas the moment this function returns.
        self._viz_bg_photo = rc.to_photoimage()
        c.create_image(0, 0, image=self._viz_bg_photo, anchor="nw")

        # Camera sensor framing: real measured camera FOV (see CAMERA_FOV_W_DEG). Uses a single
        # isotropic px/deg scale (the DEC-axis one) for both width and height so the rectangle
        # keeps its true 3:2 shape - using the RA-axis and DEC-axis scales independently here
        # would stretch it to whatever the canvas's aspect ratio happens to be instead (they
        # only agree when the canvas is exactly 2:1, matching the 360:180 deg RA:DEC range).
        pixels_per_deg = (h - 2 * margin) / dec_span
        cam_half_w, cam_half_h = self._camera_fov_half_size(pixels_per_deg)
        rect_pts = self._camera_rect_points(cx, cy, cam_half_w, cam_half_h, self._camera_orientation_deg)
        self.viz_camera_rect = c.create_polygon(rect_pts, outline="#5599ff", width=1.5,
                                                 dash=(4, 2), fill="")

        # Small filled triangle marking the "top" of the camera FOV rectangle - without it, a
        # rotated (self._camera_orientation_deg != 0, see the toolbar's Cam rotation control)
        # rectangle looks identical whichever way it's actually oriented, since a plain rectangle
        # has no visible "up". Same color as the rectangle outline it belongs to.
        marker_pts = self._camera_top_marker_points(cx, cy, cam_half_w, cam_half_h, self._camera_orientation_deg)
        self.viz_camera_top_marker = c.create_polygon(marker_pts, fill="#5599ff", outline="")

        # Telescope full-FOV circle: circumscribes the camera rectangle exactly (radius =
        # rectangle's on-screen diagonal), so its corners always touch the circle regardless of
        # the exact px/deg value used above. Outline color mirrors the stability badge (see
        # _set_stability) instead of a fixed color, so tracking health is visible at a glance
        # right on the reticle, not just in the small badge text.
        fov_radius = max(5, math.hypot(cam_half_w, cam_half_h))
        self.viz_dot = c.create_oval(cx - fov_radius, cy - fov_radius,
                                     cx + fov_radius, cy + fov_radius,
                                     fill="", outline=self._stability_color, width=2)

        # Reference circle centered on the TELESCOPE's actual current position (self.current_ra/
        # dec - positioned in _update_visualization, same as viz_dot/viz_camera_rect above, NOT
        # this fixed canvas center) - shows at a glance how big an area currently counts as "on
        # target": 110% of whichever is bigger, the target reticle itself (radius + gap + tick_len,
        # matching viz_target_circle/tick_* just after this) or the targeted object's own real/
        # hover size (see _visible_object_hits, same radius the white hover ring uses) - the 10%
        # margin (see _update_visualization) keeps the reticle/object visibly inside this circle
        # rather than exactly touching its edge. Same color as the FOV circle (viz_dot,
        # self._stability_color - see _set_stability, which keeps both in sync), dashed ("not
        # full") so it doesn't read as another real object marker or FOV boundary, just a
        # reference guide.
        self.viz_center_ref_circle = c.create_oval(0, 0, 0, 0, outline=self._stability_color,
                                                   width=1, dash=(4, 3))

        # Target reticle - a circle with an "inverted cross" (four short tick marks pointing
        # OUTWARD from the circle, not through the center) instead of a plain crosshair, so the
        # marker never covers whatever it's actually pointing at (a star dot, the ISS marker,
        # etc.) - the circle's interior and the gap around it stay completely clear. Positioned
        # in _update_visualization.
        self.viz_target_circle = c.create_oval(0, 0, 0, 0, outline="#ffaa00", width=1.5, fill="")
        self.viz_target_tick_n = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)
        self.viz_target_tick_s = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)
        self.viz_target_tick_e = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)
        self.viz_target_tick_w = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)

    def _update_visualization(self):
        """Map current RA/DEC to canvas position, honoring the current zoom/pan view."""
        c = self.viz_canvas

        # Get actual current size (dynamic)
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12

        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min

        x = self._viz_ra_to_x(self.current_ra % 360.0, w, margin, ra_min, ra_span)
        y = self._viz_dec_to_y(self.current_dec, h, margin, dec_min, dec_span)

        # Camera rectangle first, then the FOV circle is sized to exactly touch its corners
        # (see _draw_viz_grid for why this order/derivation, and the single isotropic scale,
        # both matter).
        pixels_per_deg = (h - 2 * margin) / dec_span
        cam_half_w, cam_half_h = self._camera_fov_half_size(pixels_per_deg)

        if hasattr(self, 'viz_camera_rect'):
            c.coords(self.viz_camera_rect,
                     *self._camera_rect_points(x, y, cam_half_w, cam_half_h, self._camera_orientation_deg))

        if hasattr(self, 'viz_camera_top_marker'):
            c.coords(self.viz_camera_top_marker,
                     *self._camera_top_marker_points(x, y, cam_half_w, cam_half_h, self._camera_orientation_deg))

        if hasattr(self, 'viz_dot'):
            fov_radius = max(5, math.hypot(cam_half_w, cam_half_h))
            c.coords(self.viz_dot, x - fov_radius, y - fov_radius,
                     x + fov_radius, y + fov_radius)

        # Center reference circle, at the telescope's actual current position (x, y above) - see
        # the comment where it's created (_draw_viz_grid) for the sizing logic.
        if hasattr(self, 'viz_center_ref_circle'):
            center_ref_radius = 8 + 2 + 6  # target reticle's own radius + gap + tick_len below
            if self.target_object_name:
                for hit in self._visible_object_hits:
                    if hit["name"] == self.target_object_name:
                        center_ref_radius = max(center_ref_radius, hit["radius"])
                        break
            center_ref_radius *= 1.1  # 110% of the target size, not a tight 100% fit - see _draw_viz_grid's comment
            c.coords(self.viz_center_ref_circle, x - center_ref_radius, y - center_ref_radius,
                     x + center_ref_radius, y + center_ref_radius)

        # Target reticle position - circle + outward-pointing ticks, see the comment where these
        # are created (_draw_viz_grid) for why it's not a plain crosshair.
        tx = self._viz_ra_to_x(self.target_ra % 360.0, w, margin, ra_min, ra_span)
        ty = self._viz_dec_to_y(self.target_dec, h, margin, dec_min, dec_span)

        radius = 8
        gap = 2
        tick_len = 6
        if hasattr(self, 'viz_target_circle'):
            c.coords(self.viz_target_circle, tx - radius, ty - radius, tx + radius, ty + radius)
            c.coords(self.viz_target_tick_n, tx, ty - radius - gap, tx, ty - radius - gap - tick_len)
            c.coords(self.viz_target_tick_s, tx, ty + radius + gap, tx, ty + radius + gap + tick_len)
            c.coords(self.viz_target_tick_e, tx + radius + gap, ty, tx + radius + gap + tick_len, ty)
            c.coords(self.viz_target_tick_w, tx - radius - gap, ty, tx - radius - gap - tick_len, ty)

    def _on_canvas_resize(self, event=None):
        """Redraw the visualization when the canvas size changes (both width and height)."""
        # self.after(0, ...), not after_idle() - same starvation risk under heavy POS traffic
        # while tracking as _schedule_viz_redraw's (see its comment); coalescing isn't needed
        # here (no pending-flag guard), just avoiding the idle-only gate.
        self.after(0, self._redraw_viz)

    def _redraw_viz(self):
        """Helper that redraws both grid and current position."""
        self._apply_viz_follow_mode()
        self._draw_viz_grid()
        self._update_visualization()

    # ---------------- ACTIONS ----------------
    def _refresh_ports(self):
        ports = self.serial.list_ports()
        self.port_combo.configure(values=ports if ports else ["No ports found"])
        if ports:
            self.port_combo.set(ports[0])

    def _set_arduino_controls_enabled(self, enabled: bool):
        """Enable/disable every control that actually sends something to the Arduino, so they're
        only pressable while connected instead of being clickable but silently no-op-ing with a
        'Not connected.' log line. Deliberately does NOT touch controls that are purely local/
        GUI-side (viz zoom/pan/follow-mode/ISS/Sun-Moon/Constellations toggles, the sky object
        search box, Connect/Disconnect/Refresh themselves) - those all work fine offline."""
        state = "normal" if enabled else "disabled"
        widgets = [
            self.set_loc_btn, self.ra_offset_left_btn, self.ra_offset_right_btn,
            self.dec_offset_up_btn, self.dec_offset_down_btn, self.tracking_btn,
            self.safe_target_btn, self.home_axes_btn, self.sync_btn, self.time_btn,
            self.debug_toggle_btn, self.debug_dump_btn, self.flipped_toggle_btn,
            self.meridian_limit_btn,
        ] + self.mode_radio_buttons
        for w in widgets:
            w.configure(state=state)
            self._apply_disabled_tint(w, enabled)

    def _apply_disabled_tint(self, w, enabled: bool):
        """CTkButton's built-in disabled state only dims text_color, not fg_color, so a colored
        button (Rewind Axes' olive, tracking_btn's green/red, flipped/meridian toggles' colors,
        etc.) stays fully lit even while unusable - looking exactly as pressable as it would if
        connected. Grays it toward a neutral tone (_muted_hex_color) while still hinting the
        original color, per request, and restores the exact original on re-enable. Captures the
        "true" fg_color the first time a widget goes disabled and holds onto it for the whole
        disabled stretch (rather than re-deriving it every call), since a disabled control can't
        legitimately have its color change out from under it anyway - nothing but this method
        touches its fg_color while disconnected."""
        if not enabled:
            if not hasattr(w, "_true_fg_color"):
                current = w.cget("fg_color")
                if isinstance(current, (tuple, list)) and len(current) == 2:
                    current = current[1 if ctk.get_appearance_mode() == "Dark" else 0]
                if isinstance(current, str) and current != "transparent":
                    w._true_fg_color = current
                    w.configure(fg_color=_muted_hex_color(current))
        else:
            true_fg = getattr(w, "_true_fg_color", None)
            if true_fg is not None:
                w.configure(fg_color=true_fg)
                del w._true_fg_color

    def _connect(self):
        port = self.port_combo.get()
        if not port or "No ports" in port:
            self._log("No valid port selected.")
            return
        self._log(f"Connecting to {port} ...")
        success = self.serial.connect(port)
        if success:
            self.connect_btn.configure(state="disabled")
            self.disconnect_btn.configure(state="normal")
            self._set_arduino_controls_enabled(True)
            # Scheduled here (not inside SerialHandler.connect()) because .after() needs a Tk
            # event loop, which only this App instance has.
            if self._connection_timeout_id is not None:
                try:
                    self.after_cancel(self._connection_timeout_id)
                except Exception:
                    pass
            self._connection_timeout_id = self.after(3000, self._handle_connection_timeout)

    def _handle_connection_timeout(self):
        """Called if Arduino doesn't respond to PING within timeout period"""
        if self._connection_timeout_id is not None:
            self._connection_timeout_id = None
            self._update_status_display("CONNECTED - NO ARDUINO RESPONSE", "#ff6666")
            self._log("Warning: Arduino did not respond to PING command. Connection may be unstable.")
            # Connection still works, but Arduino may need reset

    def _disconnect(self):
        # Cancel any pending connection timeout
        if self._connection_timeout_id is not None:
            self.after_cancel(self._connection_timeout_id)
            self._connection_timeout_id = None
        
        self.serial.disconnect()
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")
        self._set_arduino_controls_enabled(False)
        self.conn_status.configure(text="● Disconnected", text_color="gray")

    def _on_mode_changed(self):
        mode = self.mode_var.get()
        self.current_mode = mode

        # Force tracking to stop when switching modes (per requirement)
        self._force_stop_tracking()

        # Force the next Start Tracking to do a full alignment regardless of the (unreliable,
        # right after a mode change) target distance - see _toggle_tracking's SKIP_DEC_RESET check.
        self._force_full_align_next_start = True

        # SOLAR/LUNAR no longer have their own on-board Arduino mode - the wire protocol only
        # ever sees SIDEREAL now. The Arduino's own SolTrack/Meeus lunar-theory position
        # computation was found to disagree with the GUI's own skyfield-based Sun/Moon position
        # (used for the viz dot) by up to ~1.5deg, which showed up as "the target cross isn't on
        # the Moon" and jumpy/unstable tracking once on-target. Sun/Moon are now driven exactly
        # like the ISS already was: the GUI computes their real position via skyfield (same
        # source the viz dot uses, so they can never disagree) and streams it via
        # CMD,SET_TARGET,...,CONTINUATION:1 (see _on_solar_system_position), with the Arduino
        # just extrapolating a rate between updates through SIDEREAL mode's existing mechanism -
        # no separate on-board ephemeris involved at all. self.mode_var/current_mode still track
        # the user's SIDEREAL/SOLAR/LUNAR selection locally for the UI (mode label, which body's
        # position drives tracking) - only the wire-level CMD,MODE is now always SIDEREAL.
        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_mode("SIDEREAL")
        self._update_mode_label()
        self._log(f"Mode changed to {mode} (tracking stopped)")

    def _toggle_tracking(self):
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return

        if not self.tracking:
            # Start
            mode = self.mode_var.get()

            if mode == "SOLAR":
                # Safety gate: this will point the telescope (and whatever's attached to it -
                # eyepiece, camera sensor) directly at the Sun. Without a proper solar filter,
                # that's permanent eye damage in seconds if viewed, or a destroyed sensor if
                # imaged - require explicit confirmation EVERY time, not just once per session,
                # since the filter is physical hardware that can be forgotten/left off/knocked
                # loose between sessions in a way software has no way to detect.
                if not messagebox.askyesno(
                    "Solar Tracking Safety Check",
                    "Solar tracking will point the telescope directly at the Sun.\n\n"
                    "Confirm a proper solar filter is attached to the telescope/camera BEFORE "
                    "proceeding - without one, this can cause permanent eye damage or destroy "
                    "a camera sensor.\n\n"
                    "Is the solar filter in place?",
                    icon="warning",
                ):
                    self._log("Solar tracking start cancelled - solar filter not confirmed.")
                    return

            # The Arduino only ever tracks via SIDEREAL (SET_TARGET + continuous rate-corrected
            # holding) now - SOLAR/LUNAR are GUI-side pseudo-modes on top of that (see
            # _on_mode_changed). Whichever is selected, send an initial fresh target (no
            # CONTINUATION - this is a brand new target, not a refresh of one already being
            # tracked) before starting: SIDEREAL uses the input boxes; SOLAR/LUNAR use the
            # Sun/Moon position already computed by _solar_system_update_tick (same value the viz
            # dot is drawn from). _on_solar_system_position keeps this fresh with CONTINUATION:1
            # updates every ~2s once tracking is actually running, same as the ISS.
            if mode == "SIDEREAL":
                try:
                    # Robust parse: extract first two numbers even if user pastes "26.01, -15.9" or junk
                    ra_str = self.goto_ra.get().strip()
                    dec_str = self.goto_dec.get().strip()
                    ra = float(re.findall(r"[-+]?\d*\.?\d+", ra_str)[0]) if re.findall(r"[-+]?\d*\.?\d+", ra_str) else float(ra_str)
                    dec = float(re.findall(r"[-+]?\d*\.?\d+", dec_str)[0]) if re.findall(r"[-+]?\d*\.?\d+", dec_str) else float(dec_str)
                    # If the box TEXT still matches exactly what _select_sky_target last wrote
                    # (see _target_name_box_ra_str/_dec_str), this Start Tracking press is on
                    # whatever was just selected - keep the name. Deliberately a string compare,
                    # not a float-tolerance one: self.target_ra/dec aren't updated by
                    # _select_sky_target when tracking is already active (it must not interrupt a
                    # running session), so they can be stale relative to the boxes even when
                    # nothing was manually edited - comparing the actual displayed text sidesteps
                    # that entirely, and any real edit changes the text too.
                    if ra_str != self._target_name_box_ra_str or dec_str != self._target_name_box_dec_str:
                        self.target_object_name = None
                        self._update_target_name_label()
                    self.target_ra = ra
                    self.target_dec = dec
                    self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f}")
                except Exception:
                    self._log("Invalid RA/DEC in target boxes for sidereal; using previous sync values.")
            elif mode in ("SOLAR", "LUNAR"):
                ra, dec = (self._sun_ra, self._sun_dec) if mode == "SOLAR" else (self._moon_ra, self._moon_dec)
                if ra is not None and dec is not None:
                    self.target_object_name = "Sun" if mode == "SOLAR" else "Moon"
                    self._update_target_name_label()
                    self.target_ra = ra
                    self.target_dec = dec
                    self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f}")
                else:
                    self._log(f"{mode.title()} position not available yet (still loading skyfield ephemeris?) - "
                              f"using previous sync/target values.")

            risky = self._is_target_risky(self.target_ra, self.target_dec)
            if not self._confirm_risky_slew(self.target_ra, self.target_dec):
                # self.tracking is still False here - _request_tracking_action (not yet reached)
                # is what actually flips it, so there's nothing to unwind.
                self._log("Start Tracking cancelled - target past the meridian limit, not confirmed.")
                return

            if self._force_full_align_next_start:
                # See _on_mode_changed: right after a mode switch the GUI's cached target is
                # stale (Arduino hasn't recomputed it for the new mode yet, since that only
                # happens once tracking is active), so the distance-based fast-path decision
                # below can't be trusted this one time - always do a full alignment instead.
                self._force_full_align_next_start = False
                self._request_tracking_action("START", lambda: self.serial.send_start_tracking(risk_ok=risky),
                                               "normal alignment, forced after mode change", "STARTING…")
            else:
                # Check if target is within ±5° of current position to skip DEC reset
                distance = self._calculate_angular_distance(
                    self.current_ra, self.current_dec,
                    self.target_ra, self.target_dec
                )
                if distance <= 5.0:
                    # Target is close, skip DEC reset for faster alignment
                    self._request_tracking_action(
                        "START", lambda: self.serial.send_start_tracking_skip_dec_reset(risk_ok=risky),
                        f"DEC reset skip, target within {distance:.2f}°", "STARTING…")
                else:
                    # Target is far, use normal alignment sequence
                    self._request_tracking_action("START", lambda: self.serial.send_start_tracking(risk_ok=risky),
                                                   f"normal alignment, target {distance:.2f}° away", "STARTING…")
        else:
            # Stop
            self._request_tracking_action("STOP", self.serial.send_stop, "Stop Tracking button", "STOPPING…")

        self._update_tracking_button()

    def _safe_target(self):
        """One-shot safety move: sky DEC to 0deg, the celestial equator (SAFE_TARGET_DEC_DEG in
        the .ino, converted to a mount angle via calibration - see CMD,SAFE_TARGET's handler),
        RA untouched. Stops any active tracking first, same as any other slew command."""
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        self._force_stop_tracking()
        self.serial.send_safe_target()
        self.slewing = False
        self._log("Safe Target sent: sky DEC -> 0°, RA unchanged, no tracking.")

    def _home_axes(self):
        """Pure mechanical "return home": slews BOTH RA and DEC directly to mount angle 0
        (CMD,HOME_AXES in the .ino), bypassing sky-frame/calibration conversion entirely - needs
        no calibration at all, unlike Safe Target. Stops any active tracking first, same as any
        other slew command."""
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        self._force_stop_tracking()
        self.serial.send_home_axes()
        self.slewing = False
        self._log("Rewind Axes sent: RA/DEC -> mount angle 0°, no tracking.")

    def _stop_tracking_key(self, event=None):
        """Delete / Suppr key: emergency stop. Always attempts to send STOP if connected,
        regardless of what the GUI currently believes the tracking state is - an emergency
        stop shortcut must not silently no-op just because the local tracking flag happens to
        be out of sync with the Arduino (which is exactly the scenario it exists to recover
        from)."""
        if self.serial and self.serial.ser and self.serial.ser.is_open:
            self._request_tracking_action("STOP", self.serial.send_stop, "Delete/Suppr key", "STOPPING…")
        else:
            self._log("Not connected.")

    def _start_tracking_key(self, event=None):
        """Enter/Return key: start tracking, mirroring Delete/Suppr's global stop shortcut.
        Unlike stopping (always safe to send redundantly, so it fires unconditionally
        regardless of focus), starting can slew the telescope - this must NOT fire just
        because the user pressed Enter to commit a value in an unrelated text field (RA/DEC
        boxes, GPS lat/lon, offset increment, camera rotation, the sky search box/results
        list, etc.), each of which already has its own Enter handling. Skips entirely
        whenever focus is in an Entry or Listbox; otherwise starts tracking (does nothing if
        already tracking - this is a start shortcut, not a toggle, matching how Suppr is
        stop-only, not a toggle either)."""
        widget = event.widget if event is not None else self.focus_get()
        if isinstance(widget, (tk.Entry, tk.Listbox)):
            return
        if self.tracking:
            return
        self._toggle_tracking()

    def _update_tracking_button(self):
        if self.tracking:
            self.tracking_btn.configure(text="■ STOP TRACKING", fg_color="#8B0000")
        else:
            self.tracking_btn.configure(text="▶ Start Tracking", fg_color="#006400")

    def _adjust_camera_orientation(self, delta_deg: float):
        """Rotates the camera FOV rectangle/top-marker overlay by delta_deg, wrapped to
        (-180, 180]. Purely a local GUI display setting - matches however the camera is actually
        mounted on the telescope, sends nothing to the Arduino. Cheap: just repositions the
        already-existing overlay items via _update_visualization(), no grid rebuild needed."""
        self._set_camera_orientation(self._camera_orientation_deg + delta_deg)

    def _on_cam_rot_entry_commit(self, event=None):
        """Applies whatever's typed in cam_rot_entry directly (an absolute angle, unlike the
        ◄/► buttons' relative delta) - fires on Enter or on the entry losing focus, same commit
        pattern as everywhere else a value gets typed directly rather than via +/- buttons."""
        try:
            self._set_camera_orientation(float(self.cam_rot_var.get()))
        except (ValueError, tk.TclError):
            self._log("Invalid camera rotation value.")
            self.cam_rot_var.set(round(self._camera_orientation_deg))

    def _set_camera_orientation(self, angle_deg: float):
        """Sets the camera FOV overlay's rotation absolutely, wrapped to (-180, 180], and
        refreshes cam_rot_var/the overlay to match."""
        self._camera_orientation_deg = (angle_deg + 180.0) % 360.0 - 180.0
        self.cam_rot_var.set(round(self._camera_orientation_deg, 1))
        self._update_visualization()

    def _adjust_offset(self, ra_dir: int, dec_dir: int):
        """Adjust offset by increment in specified direction (-1, 0, or 1) and send LIVE to Arduino"""
        try:
            increment = float(self.offset_inc_var.get())
            current_ra_offset = self.ra_offset_var.get()
            current_dec_offset = self.dec_offset_var.get()
            
            # Calculate the offset adjustment (this is the amount to move, not cumulative)
            ra_adjustment = ra_dir * increment if ra_dir != 0 else 0
            dec_adjustment = dec_dir * increment if dec_dir != 0 else 0
            
            # Calculate new offset values (these are cumulative offsets from sync point)
            new_ra_offset = current_ra_offset + ra_adjustment
            new_dec_offset = current_dec_offset + dec_adjustment
            
            # Round to 6 decimal places to avoid floating-point precision issues
            new_ra_offset = round(new_ra_offset, 6)
            new_dec_offset = round(new_dec_offset, 6)
            
            # Update the GUI
            if ra_dir != 0:
                self.ra_offset_var.set(new_ra_offset)
            if dec_dir != 0:
                self.dec_offset_var.set(new_dec_offset)
            
            # Send the offset adjustment to Arduino
            if self.serial and self.serial.ser and self.serial.ser.is_open:
                self.serial.send_sync_offset(ra_adjustment, dec_adjustment)
                self._log(f"Offset adjustment: RA={ra_adjustment:.6f}° DEC={dec_adjustment:.6f}° (cumulative: RA={new_ra_offset:.6f}° DEC={new_dec_offset:.6f}°)")
            else:
                self._log(f"Offset adjustment queued: RA={ra_adjustment:.6f}° DEC={dec_adjustment:.6f}° (will send when connected)")
                
        except ValueError:
            self._log("Invalid increment value for offset adjustment")

    def _send_current_offsets(self):
        """Send the current cumulative offset values to Arduino (used when connecting)"""
        if self.serial and self.serial.ser and self.serial.ser.is_open:
            try:
                ra_offset = float(self.ra_offset_var.get())
                dec_offset = float(self.dec_offset_var.get())
                
                # Only send if non-zero offsets exist
                if abs(ra_offset) > 0.0001 or abs(dec_offset) > 0.0001:
                    # When connecting, send the cumulative offset as an adjustment from zero
                    self.serial.send_sync_offset(ra_offset, dec_offset)
                    self._log(f"Sent initial cumulative offsets: RA={ra_offset:.6f}° DEC={dec_offset:.6f}°")
            except ValueError:
                self._log("Invalid offset values when sending to Arduino")



    def _sync_position(self):
        star_name = self.sync_star_var.get()
        star_pos = self.cal_stars.get(star_name)

        if star_pos in ("EAST", "WEST"):
            # Horizon calibration points - DEC=0, RA derived from the current LST (see
            # cal_stars' comment for the Hour Angle convention this matches in the .ino).
            lst = self._get_current_lst_deg()
            ra = (lst + 90.0) % 360.0 if star_pos == "EAST" else (lst - 90.0) % 360.0
            dec = 0.0
            self.goto_ra.delete(0, "end")
            self.goto_ra.insert(0, f"{ra:.2f}")
            self.goto_dec.delete(0, "end")
            self.goto_dec.insert(0, f"{dec:.2f}")
            self.target_ra = ra
            self.target_dec = dec
            self._log(f"Using calibration point: {star_name} (LST={lst:.2f}°)")
        elif star_pos is not None:
            # Use predefined bright star coordinates - prefer the exact value from the bundled
            # catalog (same source the star's own dot on the viz is drawn from - see
            # _star_by_hip) over cal_stars' approximate hand-entered fallback, so the synced
            # target cross always lines up exactly with the star, not just approximately (see
            # cal_stars/_cal_star_hip's comments for why these could differ).
            ra, dec = star_pos
            hip = self._cal_star_hip.get(star_name)
            catalog_star = self._star_by_hip.get(hip) if hip else None
            if catalog_star is not None:
                ra, dec = catalog_star["ra"], catalog_star["dec"]
            self.goto_ra.delete(0, "end")
            self.goto_ra.insert(0, f"{ra:.2f}")
            self.goto_dec.delete(0, "end")
            self.goto_dec.insert(0, f"{dec:.2f}")
            self.target_ra = ra
            self.target_dec = dec
            self._log(f"Using calibration star: {star_name}")
        else:
            # Custom - use the fields
            try:
                ra = float(self.goto_ra.get())
                dec = float(self.goto_dec.get())
            except ValueError:
                ra = 0.0
                dec = 0.0
                self._log("Invalid custom RA/DEC for sync, using 0,0")

        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_sync(ra, dec)

            self._log(f"Sync sent: RA={ra:.6f} DEC={dec:.6f}  (star: {star_name}) - current position now calibrated to this star")
            self._update_status_display(f"SYNCED to {star_name}")
        else:
            self._log("Not connected.")

    def _manual_goto(self):
        try:
            ra = float(self.goto_ra.get())
            dec = float(self.goto_dec.get())
        except ValueError:
            self._log("Invalid RA/DEC numbers for GoTo.")
            return

        risky = self._is_target_risky(ra, dec)
        if not self._confirm_risky_slew(ra, dec):
            self._log("GoTo cancelled - target past the meridian limit, not confirmed.")
            return

        self.target_ra = ra
        self.target_dec = dec

        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_goto(ra, dec, risk_ok=risky)
            self._log(f"GoTo sent: RA={ra:.6f} DEC={dec:.6f}")
        else:
            self._log("Not connected.")

    def _toggle_slew(self):
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return

        if not self.slewing:
            # Initiate GoTo / slew
            try:
                ra = float(self.goto_ra.get())
                dec = float(self.goto_dec.get())
            except ValueError:
                self._log("Invalid RA/DEC numbers for GoTo.")
                return
            risky = self._is_target_risky(ra, dec)
            if not self._confirm_risky_slew(ra, dec):
                self._log("GoTo cancelled - target past the meridian limit, not confirmed.")
                return
            self.target_ra = ra
            self.target_dec = dec
            self.serial.send_goto(ra, dec, risk_ok=risky)
            self.slewing = True
            self._log(f"GoTo sent: RA={ra:.6f} DEC={dec:.6f}")
        else:
            # Stop current slew
            self.serial.send_command("CMD,STOP_SLEW")
            self.slewing = False
            self._log("Sent STOP_SLEW (aborting current slew)")

        self._update_slew_button()

    def _update_slew_button(self):
        # GoTo / slew button removed per request. This is a no-op stub to avoid breaking
        # other stop/reset paths that reference slewing state.
        pass

    def _set_arduino_debug(self, enable: bool):
        """Enable/disable heavy variable debug dumps from Arduino (used internally)."""
        if not hasattr(self, 'serial') or not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        self.serial.send_debug(enable)
        self.arduino_debug_enabled = enable
        self._update_debug_button()

    def _toggle_arduino_debug(self):
        """Single toggle button handler for verbose debug (ON/OFF combined as requested)."""
        if not hasattr(self, 'serial') or not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        self.arduino_debug_enabled = not self.arduino_debug_enabled
        self.serial.send_debug(self.arduino_debug_enabled)
        self._update_debug_button()
        state = "ON" if self.arduino_debug_enabled else "OFF"
        self._log(f"Sent CMD,DEBUG,{state} (Arduino verbose var dump)")

    def _update_debug_button(self):
        """Update the single debug toggle button text and color."""
        if hasattr(self, 'debug_toggle_btn') and self.debug_toggle_btn:
            if self.arduino_debug_enabled:
                self.debug_toggle_btn.configure(text="Verbose Debug: ON", fg_color="#006400")
            else:
                self.debug_toggle_btn.configure(text="Verbose Debug: OFF", fg_color="#555555")



    def _set_location(self):
        """Send real GPS coordinates to Arduino (used for Sun/Moon position calc).

        The *only* difference between modes is parsing + normalization of the
        Google Maps field. After that, both modes do exactly the same:
        - have clean lat/lon
        - call the exact same _save_gui_config()
        - send the same command
        - update UI the same way
        """
        if not hasattr(self, 'serial') or not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        try:
            if self.gps_mode_var.get():
                gps_str = self.gps_var.get().strip()
                nums = re.findall(r"[-+]?\d*\.?\d+", gps_str)
                if len(nums) < 2:
                    raise ValueError("Invalid format. Could not find two numbers (lat, lon).")
                lat = float(nums[0])
                lon = float(nums[1])
                # keep classic in sync (so _get works if mode changes later)
                self.lat_var.set(str(lat))
                self.lon_var.set(str(lon))
            else:
                lat = float(self.lat_var.get())
                lon = float(self.lon_var.get())

            # Normalize the single GPS coordinate (this is the only gmaps-specific
            # thing besides the parsing above). Both modes now have clean lat/lon
            # and the single string in gps_var.
            clean_gps = f"{lat}, {lon}"
            self.gps_var.set(clean_gps)

            # Save is now 100% identical for both modes (only parsing differed).
            self._save_gui_config()

            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")
            if self.gps_mode_var.get():
                self._set_location_label(f"Loc: {lat:.6f}, {lon:.6f} (sent)")
            else:
                self._set_location_label(f"Loc: {lat:.3f}N {lon:.3f}E (sent)")
            self._log(f"Sent location LAT={lat} LON={lon}")
            self._redraw_viz()
        except ValueError as e:
            self._log(f"Invalid GPS value: {e}")

    def _load_gui_config(self):
        """Load the single GPS coordinate from gui_config.json.
        On startup: set the GPS field, switch to GPS mode, and it will be sent to Arduino on connect.
        Only a single combined "lat, lon" GPS coordinate is stored (no split lat/lon)."""
        if not os.path.exists(CONFIG_PATH):
            # Runs before _build_ui(), so the log textbox doesn't exist yet - print() is the
            # only visible feedback available at this point. Worth seeing: a silently-missing
            # config previously looked identical to "no location saved yet", with no way to
            # tell the two apart short of checking the filesystem directly.
            print(f"[gui_config] no config file at {CONFIG_PATH} - using default location")
            return
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if isinstance(cfg, dict):
                gps_val = cfg.get("gps") or cfg.get("location")
                if gps_val:
                    gps_str = str(gps_val).strip()
                    self.gps_var.set(gps_str)
                    # Always use single GPS coordinate on load
                    self.gps_mode_var.set(True)
                    self._config_loaded = True
                    # Also populate classic fields by parsing the single gps string (for mode switching)
                    try:
                        parts = [p.strip() for p in gps_str.split(",")]
                        if len(parts) == 2:
                            self.lat_var.set(parts[0])
                            self.lon_var.set(parts[1])
                    except:
                        pass
        except Exception as e:
            self._log(f"Could not load gui_config.json: {e}")

    def _save_gui_config(self):
        """Save the single GPS coordinate in clean normal format.

        The save *logic* is now identical for both modes:
        - obtain current (lat, lon) via _get_lat_lon_from_input()
          (the *only* place that knows the parsing difference per mode)
        - format as single "lat, lon" string and write to config.
        """
        try:
            lat, lon = self._get_lat_lon_from_input()
            # Only save if we got valid values (not the default fallback)
            if lat == DEFAULT_LAT and lon == DEFAULT_LON:
                # Try to get values from the appropriate input field
                if self.gps_mode_var.get():
                    gps_str = self.gps_var.get().strip()
                    nums = re.findall(r"[-+]?\d*\.?\d+", gps_str)
                    if len(nums) >= 2:
                        lat, lon = float(nums[0]), float(nums[1])
                    else:
                        # If GPS string is invalid, use default
                        lat, lon = DEFAULT_LAT, DEFAULT_LON
                else:
                    try:
                        lat = float(self.lat_var.get())
                        lon = float(self.lon_var.get())
                    except:
                        lat, lon = DEFAULT_LAT, DEFAULT_LON
            gps = f"{lat}, {lon}"
        except Exception:
            gps = self.gps_var.get().strip() or f"{DEFAULT_LAT}, {DEFAULT_LON}"
        cfg = {"gps": gps}
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            self._log(f"Saved GPS config: {gps}")
        except Exception as e:
            self._log(f"Could not save gui_config.json: {e}")

    def _on_streamer_mode_toggle(self):
        """Mask/unmask the GPS location display. Only ever touches how things are DISPLAYED
        (Entry 'show' character, the summary label's text) - lat_var/lon_var/gps_var themselves
        are never cleared or altered, so turning this off just reveals the real values again,
        and everything location-related (sync, Sun/Moon calc, etc.) keeps working normally the
        whole time it's on."""
        hide = self.streamer_mode_var.get()
        mask_char = "•" if hide else ""
        self.lat_entry.configure(show=mask_char)
        self.lon_entry.configure(show=mask_char)
        self.gps_entry.configure(show=mask_char)
        if hide:
            self._set_location_label("Loc: •••••• (hidden)")
        else:
            lat, lon = self._get_lat_lon_from_input()
            self._set_location_label(f"Loc: {lat}, {lon}")

    def _set_location_label(self, text: str):
        """Every update to location_label should go through here, not call .configure()
        directly - otherwise a refresh from some other code path (live GPS updates, EEPROM
        confirmation, sending to Arduino, etc.) would silently leak the real coordinates back
        onto the label even while Streamer Mode is hiding them."""
        if self.streamer_mode_var.get():
            self.location_label.configure(text="Loc: •••••• (hidden)")
        else:
            self.location_label.configure(text=text)

    def _on_gps_mode_toggle(self):
        if self.gps_mode_var.get():
            # Switch to single lat,lon (GPS format) input
            self.latlon_frame.pack_forget()
            # Try to prefill from current lat/lon (clean format)
            try:
                lat = float(self.lat_var.get())
                lon = float(self.lon_var.get())
                self.gps_var.set(f"{lat}, {lon}")
            except:
                pass
            self.gps_frame.pack(side="left", padx=4)
        else:
            # Switch to separate Lat/Lon inputs
            self.gps_frame.pack_forget()
            # Try to parse back (robust)
            try:
                nums = re.findall(r"[-+]?\d*\.?\d+", self.gps_var.get())
                if len(nums) >= 2:
                    self.lat_var.set(nums[0])
                    self.lon_var.set(nums[1])
            except:
                pass
            self.latlon_frame.pack(side="left", padx=4)

        # Note: _save_gui_config() is now called via trace callback on gps_mode_var

    def _get_lat_lon_from_input(self):
        """Return (lat, lon) based on the current input mode (classic or GPS).
        Uses robust number extraction so Google Maps pasted values work even before clicking Set."""
        if self.gps_mode_var.get():
            try:
                s = self.gps_var.get().strip()
                nums = re.findall(r"[-+]?\d*\.?\d+", s)
                if len(nums) >= 2:
                    return float(nums[0]), float(nums[1])
            except Exception:
                pass
            return DEFAULT_LAT, DEFAULT_LON
        else:
            try:
                return float(self.lat_var.get()), float(self.lon_var.get())
            except Exception:
                return DEFAULT_LAT, DEFAULT_LON

    def _send_location_if_connected(self):
        """Send current GPS location to Arduino if connected."""
        if not hasattr(self, 'serial') or not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            return
        
        try:
            # Get current lat/lon based on input mode
            if self.gps_mode_var.get():
                gps_str = self.gps_var.get().strip()
                nums = re.findall(r"[-+]?\d*\.?\d+", gps_str)
                if len(nums) < 2:
                    return
                lat = float(nums[0])
                lon = float(nums[1])
            else:
                lat = float(self.lat_var.get())
                lon = float(self.lon_var.get())
            
            # Create location key to check for duplicates
            location_key = f"{lat:.6f},{lon:.6f}"
            
            # Only send if location has changed
            if self._last_sent_location == location_key:
                return
            
            # Send to Arduino
            self.serial.send_command(f"CMD,SET_LOCATION,LAT:{lat:.6f},LON:{lon:.6f}")
            self._log(f"Sent location LAT={lat:.6f} LON={lon:.6f}")
            
            # Automatically save to EEPROM (transparent to user)
            # This ensures Arduino retains the location across power cycles
            self.serial.send_save_location_to_eeprom()
            
            # Update tracking and location label
            self._last_sent_location = location_key
            if self.gps_mode_var.get():
                self._set_location_label(f"Loc: {lat:.6f}, {lon:.6f} (live)")
            else:
                self._set_location_label(f"Loc: {lat:.3f}N {lon:.3f}E (live)")
                
        except ValueError as e:
            self._log(f"Invalid GPS value for live send: {e}")

    def _on_location_var_changed(self, *args):
        """Debounced redraw of the viz and save GPS config when location input changes.
        This makes the horizon/ground curve update live and saves GPS coords live when you type a different latitude."""
        if self._location_redraw_id is not None:
            try:
                self.after_cancel(self._location_redraw_id)
            except Exception:
                pass
        
        def save_and_redraw():
            # Save GPS configuration live
            self._save_gui_config()
            # Send location to Arduino if connected
            self._send_location_if_connected()
            # Redraw visualization
            self._redraw_viz()
        
        self._location_redraw_id = self.after(200, save_and_redraw)

    def _apply_initial_rate(self):
        """Force the fixed POS_UPDATE_RATE_MS rate on the Arduino (used on connect) - the rate is
        no longer user-adjustable, so this always sends the same fixed value rather than
        whatever was last shown in a (now-removed) rate entry field."""
        if hasattr(self, 'serial') and self.serial and self.serial.ser and self.serial.ser.is_open:
            self.serial.send_pos_update_rate(POS_UPDATE_RATE_MS)

    def _apply_initial_meridian_limit(self):
        """Re-sends the GUI's current meridian-limit toggle state to the Arduino on every connect
        (see _meridian_limit_enabled's comment) - belt-and-suspenders against the firmware having
        been left disabled from a previous session while staying powered through a GUI restart."""
        if hasattr(self, 'serial') and self.serial and self.serial.ser and self.serial.ser.is_open:
            self.serial.send_meridian_limit(self._meridian_limit_enabled)

    # ---------------- QUEUE / MESSAGE HANDLING ----------------
    def _poll_queue(self):
        try:
            while True:
                msg = self.message_queue.get_nowait()
                try:
                    self._handle_message(msg)
                except Exception as e:
                    # Catch parsing/handling errors so they show in the GUI log instead of crashing the app
                    self._log(f"ERROR parsing message: {e}  |  raw={msg[:120]}")
                    self.error_label.configure(text=f"Parse error: {e}")
        except queue.Empty:
            pass

        # Periodic UI updates
        self._update_labels()
        self.after(POLL_INTERVAL_MS, self._poll_queue)

    def _start_time_sync_timer(self):
        # Sync Arduino time to laptop time on a regular basis
        def resync():
            if hasattr(self, 'serial') and self.serial and self.serial.ser and self.serial.ser.is_open:
                self.serial.send_time_if_connected()
            self.after(60000, resync)  # every 60 seconds
        self.after(2000, resync)  # first resync shortly after start

    def _start_ground_update_timer(self):
        # Update ground horizon visualization periodically as LST changes
        def update_ground():
            # Only update if we have a valid latitude (location set) and are connected
            try:
                lat, _ = self._get_lat_lon_from_input()
                if abs(lat) > 0.1:  # Valid latitude
                    # Check if we're connected or if we at least have the UI initialized
                    if hasattr(self, 'viz_canvas') and self.viz_canvas.winfo_exists():
                        # Only redraw if the canvas is actually visible and has size
                        if self.viz_canvas.winfo_width() > 100 and self.viz_canvas.winfo_height() > 100:
                            self._redraw_viz()
            except:
                pass
            self.after(30000, update_ground)  # every 30 seconds
        self.after(5000, update_ground)  # first update after 5 seconds

    def _start_viz_follow_timer(self):
        """While in TELESCOPE/TARGET follow mode, periodically snap the view center to its
        target and do a full grid redraw. Deliberately NOT done on every POS update (which can
        arrive at up to ~100 Hz) - a full _draw_viz_grid() clears and rebuilds the whole canvas,
        so that would be far too expensive to run at POS rate. The dot/cross themselves still
        move smoothly every POS update via _update_visualization(); only the grid/labels/
        centering update at this (deliberately coarse - see VIZ_FOLLOW_TICK_MS) rate."""
        def follow_tick():
            try:
                if self._viz_view_mode != "FREE" and self.viz_canvas.winfo_exists() and \
                        self.viz_canvas.winfo_width() > 100 and self.viz_canvas.winfo_height() > 100:
                    self._redraw_viz()
            except Exception:
                pass
            self.after(VIZ_FOLLOW_TICK_MS, follow_tick)
        self.after(VIZ_FOLLOW_TICK_MS, follow_tick)

    def _apply_viz_follow_mode(self):
        """If following the telescope or target, snap the view center directly to it.

        This used to ease toward the target (exponential smoothing) for continuous camera
        motion instead of a jump-cut, but that meant a full grid redraw every VIZ_FOLLOW_TICK_MS
        (previously 60ms, needed for the easing to look smooth) whenever follow mode was active
        - and if you zoomed while following, that timer's redraws stacked on top of the zoom-
        triggered ones, compounding into visible lag. A direct snap doesn't need frequent ticks
        to look right (see VIZ_FOLLOW_TICK_MS, now much coarser), which removes that source of
        constant background redraw load entirely instead of just making each redraw cheaper.
        No-op in FREE mode - center stays wherever pan/zoom left it."""
        if self._viz_view_mode == "TELESCOPE":
            self._viz_center_ra = self.current_ra % 360.0
            self._viz_center_dec = self.current_dec
        elif self._viz_view_mode == "TARGET":
            self._viz_center_ra = self.target_ra % 360.0
            self._viz_center_dec = self.target_dec
        else:
            return
        self._clamp_viz_center()

    def _build_sky_catalog_state(self):
        """Reads sky_catalog.json and builds every derived structure (HIP index, RA-sorted list
        for bisecting, search index) as a plain dict, WITHOUT touching any self.* attribute -
        safe to call from a background thread (see _load_sky_catalog_async). Returns None on
        failure (missing/corrupt file) rather than raising, so the caller can just skip applying
        anything and keep whatever was there before."""
        try:
            with open(sky_catalog.CATALOG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            sky_stars = data.get("stars", [])
            sky_dso = data.get("dso", [])
            # Backfill "designation" (added in CATALOG_SCHEMA_VERSION 2) for a catalog file still
            # on disk from before that change - every pre-2 entry has "messier" unconditionally
            # (that schema was Messier-only), so this is a full, lossless fix, not a guess. Needed
            # because this on-disk file gets loaded and rendered immediately, before
            # sky_catalog.ensure_catalog_current()'s own schema-version check has a chance to
            # trigger a real rebuild moments later (see _load_sky_catalog_async) - without this,
            # that gap crashes the loader thread with a bare KeyError on old files.
            for d in sky_dso:
                if "designation" not in d:
                    d["designation"] = d.get("messier") or ""
            const_line_segments = data.get("const_lines", [])
            # Indexed by Hipparcos number - constellation line segments only give HIP numbers,
            # so this is how their endpoints get resolved back to actual RA/DEC for drawing.
            star_by_hip = {s["hip"]: s for s in sky_stars if "hip" in s}
            # Sorted by RA once here (not per redraw) so _draw_sky_objects can bisect straight to
            # the visible RA band instead of scanning all ~870000 stars every single redraw - see
            # its docstring. Dec is still checked in the loop there (a 1-D RA sort can't also
            # narrow Dec), but for a zoomed-in view this cuts the scanned set roughly in
            # proportion to (visible RA span / 360), which is the common case.
            sky_stars_by_ra = sorted(sky_stars, key=lambda s: s["ra"])
            sky_stars_ra_values = [s["ra"] for s in sky_stars_by_ra]
            # Magnitude-capped subsets of the same RA-sorted list (filtering an already-sorted
            # list preserves order, no separate re-sort needed) - see STAR_LOD_TIER_MAG_CUTOFFS's
            # comment. The full list itself is the implicit last tier (cutoff=inf) for the rare
            # case a zoom level's mag_limit exceeds every configured cutoff.
            star_tiers = []
            for cutoff in STAR_LOD_TIER_MAG_CUTOFFS:
                tier_stars = [s for s in sky_stars_by_ra if s["mag"] <= cutoff]
                star_tiers.append((cutoff, tier_stars, [s["ra"] for s in tier_stars]))
            star_tiers.append((float("inf"), sky_stars_by_ra, sky_stars_ra_values))
        except (OSError, json.JSONDecodeError):
            return None

        # Search index for the "Find:" box - only named stars (searching by a bare HIP/TYC number
        # isn't useful for most people, and leaving out the ~750000 unnamed-but-still-drawn stars
        # added by the AT-HYG deep catalog - see sky_catalog.py's module docstring - keeps this
        # index exactly as small as it was on the old, much-smaller star catalog), all Messier
        # DSOs (by both messier id and common name), plus Sun/Moon/ISS/planets as "live" entries
        # whose ra/dec aren't fixed here - they're resolved from
        # self._sun_ra/_moon_ra/_iss_ra/_planet_positions at selection time instead (see
        # _choose_sky_search_entry), since baking in whatever position happened to be current
        # when the catalog loaded would go stale within minutes for the ISS and hours for the rest.
        #
        # "search_blob" (what the substring match runs against) includes every alias sky_catalog.py
        # built for this object - e.g. a star can be found by its constructed common name
        # ("Tau Ceti"), its raw catalog designation ("52 Ceti"), or its HIP/HD/TYC number, even
        # though only the primary "name" is what's actually displayed/shown as the result.
        sky_search_index = []
        for s in sky_stars:
            if not s.get("name"):
                continue
            blob = " ".join([s["name"]] + s.get("aliases", [])).lower()
            sky_search_index.append({
                "name": s["name"], "ra": s["ra"], "dec": s["dec"], "live": None, "search_blob": blob,
            })
        for d in sky_dso:
            display = f"{d['designation']} ({d['name']})" if d.get("name") else d["designation"]
            blob = " ".join([display] + d.get("aliases", [])).lower()
            sky_search_index.append({
                "name": display, "ra": d["ra"], "dec": d["dec"], "live": None, "search_blob": blob,
            })
        for live_name in ("Sun", "Moon", "ISS", "Mercury", "Venus", "Mars", "Jupiter", "Saturn",
                          "Uranus", "Neptune"):
            sky_search_index.append({
                "name": live_name, "ra": None, "dec": None, "live": live_name,
                "search_blob": live_name.lower(),
            })

        return {
            "sky_stars": sky_stars, "sky_dso": sky_dso, "const_line_segments": const_line_segments,
            "star_by_hip": star_by_hip, "sky_stars_by_ra": sky_stars_by_ra,
            "sky_stars_ra_values": sky_stars_ra_values, "sky_search_index": sky_search_index,
            "star_tiers": star_tiers,
        }

    def _apply_sky_catalog_state(self, state):
        """Assigns a _build_sky_catalog_state() result onto self.* and triggers a redraw - the
        Tk-thread half of the split (see _load_sky_catalog_async for why this is split at all)."""
        self._sky_stars = state["sky_stars"]
        self._sky_dso = state["sky_dso"]
        self._const_line_segments = state["const_line_segments"]
        self._star_by_hip = state["star_by_hip"]
        self._sky_stars_by_ra = state["sky_stars_by_ra"]
        self._sky_stars_ra_values = state["sky_stars_ra_values"]
        self._sky_search_index = state["sky_search_index"]
        self._star_tiers = state["star_tiers"]
        self._log(f"Sky catalog loaded: {len(self._sky_stars)} stars, {len(self._sky_dso)} DSOs, "
                  f"{len(self._const_line_segments)} constellation line segments.")
        self._schedule_viz_redraw()

    def _load_sky_catalog_async(self):
        """Load whatever catalog is already on disk, then check/refresh it from the network -
        both phases run in the same background thread, since the AT-HYG-based catalog (see
        sky_catalog.py's module docstring) is now ~90MB of JSON: json.load() + building the HIP
        index/RA-sorted list/search index/magnitude LOD tiers (see STAR_LOD_TIER_MAG_CUTOFFS)
        measured at ~2s total on the initial disk load alone, which would freeze the GUI for a
        very noticeable moment if done on the Tk thread the way this used to work when the
        catalog was a much smaller file (a few MB). Thread only touches plain Python data (file
        I/O, list building); the actual swap into self._sky_stars/_sky_dso and the redraw happen
        back on the Tk thread via self.after(), since Tkinter isn't thread-safe (see
        _build_sky_catalog_state/_apply_sky_catalog_state for the split)."""
        def worker():
            state = self._build_sky_catalog_state()
            if state is not None:
                self.after(0, lambda: self._apply_sky_catalog_state(state))
            # Logged BEFORE calling ensure_catalog_current() (which does the actual, potentially
            # multi-minute network fetch) specifically so a rebuild isn't silent - previously the
            # GUI just went quiet until it finished (or failed), which read as "nothing is
            # happening" during what can be a couple of minutes re-downloading the star catalog.
            if sky_catalog.needs_rebuild(sky_catalog.CATALOG_PATH):
                self.after(0, lambda: self._log(
                    "Sky catalog is outdated - downloading updated star/DSO data in the "
                    "background (can take a couple minutes); current stars/objects still usable "
                    "meanwhile, will refresh automatically once done."))
            try:
                updated = sky_catalog.ensure_catalog_current(sky_catalog.CATALOG_PATH)
            except Exception as e:
                # Python deletes the exception variable at the end of the except block, so it
                # must be captured into a plain string now, not referenced from the deferred lambda.
                msg = str(e)
                self.after(0, lambda: self._log(f"Sky catalog update skipped (no internet?): {msg}"))
                return
            if updated:
                state2 = self._build_sky_catalog_state()
                if state2 is not None:
                    self.after(0, lambda: self._apply_sky_catalog_state(state2))

        threading.Thread(target=worker, daemon=True).start()

    def _toggle_telescope_flipped(self):
        """Requests the opposite of the last CONFIRMED flip state (see self._telescope_flipped) -
        does NOT optimistically update the button; that only happens once the firmware confirms
        via POS's "flipped" field or STATUS:FLIP_COMPLETE (see _on_telescope_flipped_confirmed),
        since the RA axis physically has to rotate ~180° first."""
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        requested = not self._telescope_flipped
        self.serial.send_command(f"CMD,SET_FLIPPED,{1 if requested else 0}")
        self._log(f"Requested telescope flip: {'ON' if requested else 'OFF'} (RA will rotate ~180° - watch for STATUS:FLIP_COMPLETE)")

    def _on_telescope_flipped_confirmed(self, flipped: bool):
        """Updates self._telescope_flipped and the toggle button to match a CONFIRMED state -
        called from the POS parser's "flipped" field (so a reconnecting GUI picks up the real
        state instead of assuming OFF) and from STATUS:FLIP_COMPLETE. A no-op if already showing
        that state, so it's safe to call on every POS line without extra widget churn."""
        if flipped == self._telescope_flipped:
            return
        self._telescope_flipped = flipped
        if flipped:
            self.flipped_toggle_btn.configure(text="Telescope Flipped: ON", fg_color="#8B4500")
        else:
            self.flipped_toggle_btn.configure(text="Telescope Flipped: OFF", fg_color="#444444")
        self._schedule_viz_redraw()  # the meridian-limit shading only draws while not flipped

    def _toggle_meridian_limit(self):
        """Toggles the firmware's meridian-limit safety check on/off (CMD,SET_MERIDIAN_LIMIT -
        see meridianLimitEnabled in the .ino). Unlike the telescope-flip toggle, this applies
        instantly firmware-side (no motor movement to wait on), so the button updates
        optimistically here rather than waiting for STATUS:MERIDIAN_LIMIT_ENABLED confirmation -
        that confirmation still arrives and will correct the button if it ever disagrees (e.g.
        a write that silently failed)."""
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        requested = not self._meridian_limit_enabled
        if not requested:
            # Turning OFF a safety check that exists to prevent an OTA/dovetail-vs-tripod/pier
            # collision - require explicit confirmation every time, same pattern as the solar
            # tracking safety gate (_toggle_tracking), since this is exactly the kind of setting
            # that's easy to forget was left off between sessions.
            if not messagebox.askyesno(
                "Disable Meridian Limit Safety Check",
                "This disables the mount's meridian-limit safety check.\n\n"
                "With it off, GOTO/tracking will NOT be refused or stopped when the target "
                "crosses the meridian - only do this if you have confirmed your mount's actual "
                "mechanical clearance past the meridian.\n\n"
                "Disable the meridian limit check?",
                icon="warning",
            ):
                self._log("Meridian limit disable cancelled.")
                return
        self._meridian_limit_enabled = requested
        self.serial.send_meridian_limit(requested)
        if requested:
            self.meridian_limit_btn.configure(text="Meridian Limit: ON", fg_color="#006400")
        else:
            self.meridian_limit_btn.configure(text="Meridian Limit: OFF", fg_color="#8B0000")
        self._log(f"Meridian limit safety check: {'ON' if requested else 'OFF'}")
        self._schedule_viz_redraw()  # the meridian-limit shading only makes sense while enabled

    def _on_meridian_limit_confirmed(self, enabled: bool):
        """Reconciles self._meridian_limit_enabled/meridian_limit_btn with a firmware-confirmed
        state (STATUS:MERIDIAN_LIMIT_ENABLED, from our own SET_MERIDIAN_LIMIT or a QUERY reply on
        connect) - a no-op if already showing that state, so it's safe to call on every such line."""
        if enabled == self._meridian_limit_enabled:
            return
        self._meridian_limit_enabled = enabled
        if enabled:
            self.meridian_limit_btn.configure(text="Meridian Limit: ON", fg_color="#006400")
        else:
            self.meridian_limit_btn.configure(text="Meridian Limit: OFF", fg_color="#8B0000")
        self._schedule_viz_redraw()

    def _toggle_iss_tracking(self):
        self._iss_enabled = not self._iss_enabled
        if self._iss_enabled:
            self.iss_toggle_btn.configure(text="ISS: ON", fg_color="#006400")
            self._iss_update_tick()  # kick off immediately, then it reschedules itself
        else:
            self.iss_toggle_btn.configure(text="ISS: OFF", fg_color="#444444")
            if self._iss_update_after_id is not None:
                self.after_cancel(self._iss_update_after_id)
                self._iss_update_after_id = None
            self._iss_ra = None
            self._iss_dec = None
            self._schedule_viz_redraw()

    def _iss_update_tick(self):
        """Fetch/refresh the ISS TLE and compute its current topocentric RA/DEC in a background
        thread (network + skyfield propagation, neither of which should block the GUI), then
        hand the result back to the Tk thread. Reschedules itself every 5s while just displaying
        the ISS marker - fast enough for the reticle, nowhere near fast enough to actually slew
        the mount to follow it. While the ISS IS the active tracked target (see
        target_object_name / _on_iss_position), reschedules at ISS_TRACKING_UPDATE_MS instead -
        fast enough that the mount's existing near-instant tracking-correction path (see
        Axis::update() in the firmware) can keep up with the ISS's real angular motion, not just
        the on-screen marker."""
        if not self._iss_enabled:
            return
        lat, lon = self._get_lat_lon_from_input()
        at_time = self._get_effective_utc_now()  # single time source - see its comment

        def worker():
            try:
                iss_tracker.ensure_tle_current()
                ra, dec, alt, az, above = iss_tracker.get_current_radec(lat, lon, at_time=at_time)
            except ImportError:
                self.after(0, lambda: self._log(
                    "ISS tracking needs the 'skyfield' package - pip install skyfield"))
                self.after(0, lambda: self._toggle_iss_tracking())
                return
            except Exception as e:
                msg = str(e)
                self.after(0, lambda: self._log(f"ISS position update failed: {msg}"))
                return
            self.after(0, lambda: self._on_iss_position(ra, dec, above))

        threading.Thread(target=worker, daemon=True).start()
        actively_tracking_iss = (self.target_object_name == "ISS" and self.tracking
                                  and self.mode_var.get() == "SIDEREAL")
        interval_ms = ISS_TRACKING_UPDATE_MS if actively_tracking_iss else ISS_DISPLAY_UPDATE_MS
        # Synced to the wall clock (next exact multiple of interval_ms since the epoch), not a
        # free-running fixed delay from whenever this call happened to fire - same self-
        # rescheduling-to-the-boundary technique as _update_realtime_dot/_update_time_travel_label,
        # so ISS updates land on a predictable, human-readable cadence (e.g. every real :00/:01/:02
        # second at 1Hz) instead of drifting to an arbitrary phase that shifts by however long
        # get_current_radec()/the Thread start took each time.
        interval_s = interval_ms / 1000.0
        next_delay_ms = max(1, round((interval_s - (time.time() % interval_s)) * 1000))
        self._iss_update_after_id = self.after(next_delay_ms, self._iss_update_tick)

    def _target_is_live_body(self):
        """True while the selected target is a body whose sky position visibly changes over the
        course of a session (ISS, Sun, Moon, or a planet) and so needs target_ra/dec kept fresh
        from the GUI's own live computation (_on_iss_position/_on_solar_system_position) rather
        than the firmware's POS echo (fields[3]/[4]) - which only ever reflects whatever was in
        the last SET_TARGET sent, frozen once tracking stops (no more SET_TARGETs to echo), and
        lagged by real serial round-trip time even while tracking. A star/DSO's catalog RA/DEC
        doesn't change during a session, so the echo is perfectly fine (and, while actually
        tracking, arguably more accurate than the client-side value) for anything that isn't one
        of these."""
        name = self.target_object_name
        return name == "ISS" or name == "Sun" or name == "Moon" or name in getattr(self, "_planet_positions", {})

    def _on_iss_position(self, ra, dec, above_horizon):
        self._iss_ra = ra
        self._iss_dec = dec
        self._iss_above_horizon = above_horizon
        self._schedule_viz_redraw()

        # If the ISS is the currently active tracked target, keep the Arduino's target fresh too -
        # not just the on-screen marker. The firmware has no ISS-specific mode; this reuses
        # SIDEREAL tracking's existing CMD,SET_TARGET + continuous correction machinery (the same
        # thing search-selecting any star uses), just refreshed often enough (see
        # ISS_TRACKING_UPDATE_MS) to keep up with the ISS's real orbital motion instead of only
        # ever being sent once. CONTINUATION:1 explicitly tells the firmware this is the SAME
        # tracked object being refreshed (see the CMD,SET_TARGET handler in the .ino) so it's safe
        # to derive a tracking rate from consecutive updates - every other SET_TARGET sender
        # (double-click, search, manual entry) omits this, since those are always a fresh target.
        # Keep target_ra/dec (the viz target reticle/View: Follow Target/live error readout)
        # following the ISS's live position whenever it's the SELECTED target - regardless of
        # whether tracking is currently active. Previously this was gated on self.tracking too
        # (bundled with the "send a fresh target to the Arduino" decision below), so target_ra/
        # dec froze at wherever the ISS was the moment tracking stopped instead of continuing to
        # follow it - obvious within seconds given how fast the ISS moves. Reported as "when I
        # stop tracking the ISS, the target stays at the position I stopped tracking instead of
        # following the actual target." See _target_is_live_body's comment - the same fix applies
        # to Sun/Moon in _on_solar_system_position.
        if self.target_object_name == "ISS":
            self.target_ra = ra
            self.target_dec = dec

        # Sending a fresh target to the Arduino, however, still only makes sense while actually
        # tracking - CONTINUATION:1 tells the firmware this is the SAME tracked object being
        # refreshed (see the CMD,SET_TARGET handler in the .ino) so it's safe to derive a
        # tracking rate from consecutive updates.
        if (self.target_object_name == "ISS" and self.tracking
                and self.mode_var.get() == "SIDEREAL"
                and self.serial and self.serial.ser and self.serial.ser.is_open):
            self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f},CONTINUATION:1")

    def _toggle_constellations(self):
        """Unlike ISS/Sun-Moon, this needs no network call or background thread - the
        constellation line data is already part of the locally-loaded sky catalog (see
        self._const_line_segments), so toggling this on/off is just a redraw."""
        self._constellations_enabled = not self._constellations_enabled
        if self._constellations_enabled:
            self.const_toggle_btn.configure(text="Constellations: ON", fg_color="#006400")
        else:
            self.const_toggle_btn.configure(text="Constellations: OFF", fg_color="#444444")
        self._schedule_viz_redraw()

    def _toggle_min_dso_size_filter(self):
        """See self._min_dso_size_filter_enabled's comment - purely a display filter, no network/
        catalog reload needed, just a redraw."""
        self._min_dso_size_filter_enabled = not self._min_dso_size_filter_enabled
        if self._min_dso_size_filter_enabled:
            self.min_size_toggle_btn.configure(text="Min Size: ON", fg_color="#006400")
        else:
            self.min_size_toggle_btn.configure(text="Min Size: OFF", fg_color="#444444")
        self._schedule_viz_redraw()

    def _on_min_dso_size_entry_commit(self, event=None):
        """Applies whatever's typed in min_dso_size_entry - fires on Enter or on the entry losing
        focus, same commit pattern as cam_rot_entry (_on_cam_rot_entry_commit). Only redraws (has
        any visible effect) while the filter is actually ON, but the value is accepted/stored
        either way so turning the filter on afterward uses it immediately without retyping."""
        try:
            px = float(self.min_dso_size_var.get())
            if px < 0:
                raise ValueError("negative size")
            self._min_dso_size_px = px
        except (ValueError, tk.TclError):
            self._log("Invalid minimum DSO size value.")
            self.min_dso_size_var.set(self._min_dso_size_px)
            return
        if self._min_dso_size_filter_enabled:
            self._schedule_viz_redraw()

    def _solar_system_update_tick(self):
        """Fetches/refreshes Sun, Moon, and the naked-eye planets (Mercury-Saturn) together in one
        background call, since they all come from the same skyfield ephemeris (see
        solar_system.py). Used to refresh every 60s on the assumption that they move slowly
        enough across the sky that anything faster would just waste ephemeris lookups for no
        visible change - true for the viz's own on-screen dot at typical zoom, but NOT true once
        Lunar tracking is actually running: the Arduino recomputes the Moon's real position
        continuously (every loop() tick, see solar_system.py's own header comment on why
        Solar/Lunar modes don't have the discrete-jump problem ISON-style targets do), so the
        physical telescope stays accurately on the Moon while this GUI-side dot could be up to a
        full 60s stale - the Moon moves ~33"/min, so a 60s-stale dot could be visibly off from
        where the telescope (correctly) actually is, misread as "the telescope isn't centered on
        the Moon" when it's really just this reticle lagging. Refreshing at SOLAR_SYSTEM_UPDATE_MS
        (1Hz) instead bounds that staleness to a fraction of an arcsecond, imperceptible at any
        real zoom, for a background-thread ephemeris lookup that's cheap enough not to matter at
        60x the old rate. Always on (see the comment
        near the state this populates) - no toggle, so this reschedules itself forever once
        kicked off at startup, same pattern as _iss_update_tick but without the enabled-flag check
        (except the one-time "skyfield isn't installed" case, which stops rescheduling entirely
        rather than retrying and re-logging forever)."""
        if self._solar_system_skyfield_missing:
            return
        lat, lon = self._get_lat_lon_from_input()
        # Real time unless a Time Travel preview is active (see _get_effective_utc_now) - computed
        # here on the main thread (cheap, no I/O) and passed into the worker rather than read
        # inside it, so the whole background lookup reflects one single consistent instant.
        at_time = self._get_effective_utc_now()

        def worker():
            try:
                sun_ra, sun_dec, sun_diam = solar_system.get_sun_info(lat, lon, at_time=at_time)
                moon_ra, moon_dec, moon_diam, illum, phase, waxing = solar_system.get_moon_info(lat, lon, at_time=at_time)
                planets = solar_system.get_planets_info(lat, lon, at_time=at_time)
            except ImportError:
                self.after(0, lambda: self._log(
                    "Sun/Moon/planet display needs the 'skyfield' package - pip install skyfield"))
                self.after(0, lambda: setattr(self, '_solar_system_skyfield_missing', True))
                return
            except Exception as e:
                msg = str(e)
                self.after(0, lambda: self._log(f"Sun/Moon/planet position update failed: {msg}"))
                return
            self.after(0, lambda: self._on_solar_system_position(
                sun_ra, sun_dec, sun_diam, moon_ra, moon_dec, moon_diam, illum, phase, waxing, planets))

        threading.Thread(target=worker, daemon=True).start()
        self._solar_system_update_after_id = self.after(SOLAR_SYSTEM_UPDATE_MS, self._solar_system_update_tick)

    def _on_solar_system_position(self, sun_ra, sun_dec, sun_diam, moon_ra, moon_dec, moon_diam,
                                   illum, phase, waxing, planets):
        self._sun_ra, self._sun_dec, self._sun_ang_diam_deg = sun_ra, sun_dec, sun_diam
        self._moon_ra, self._moon_dec, self._moon_ang_diam_deg = moon_ra, moon_dec, moon_diam
        self._moon_illum_fraction, self._moon_phase_deg, self._moon_waxing = illum, phase, waxing
        self._planet_positions = planets
        self._schedule_viz_redraw()

        # Keep target_ra/dec (the viz target reticle/View: Follow Target/live error readout)
        # following the Sun/Moon/a planet's live position whenever one is the SELECTED target -
        # regardless of whether tracking is currently active. Same fix as _on_iss_position's for
        # the ISS; see _target_is_live_body's comment for why this matters even though these move
        # much slower - the position was still frozen at Stop, just less obviously/quickly than
        # the ISS's freeze.
        if self.target_object_name == "Sun":
            self.target_ra, self.target_dec = sun_ra, sun_dec
        elif self.target_object_name == "Moon":
            self.target_ra, self.target_dec = moon_ra, moon_dec
        elif self.target_object_name in planets:
            self.target_ra, self.target_dec = planets[self.target_object_name][0], planets[self.target_object_name][1]

        # Sending a fresh target to the Arduino, however, still only makes sense while SOLAR/LUNAR
        # tracking is actually active - not just displayed. Same CONTINUATION:1 mechanism as
        # _on_iss_position: the Arduino has no on-board Sun/Moon position mode anymore (see
        # _on_mode_changed), it just derives a rate from consecutive SET_TARGETs and extrapolates -
        # the Sun/Moon move ~1000x slower than the ISS, so this 2s cadence (vs
        # ISS_TRACKING_UPDATE_MS) is easily fast enough for a smooth, accurate rate.
        mode = self.mode_var.get()
        cmd_ra, cmd_dec, expected_name = (
            (sun_ra, sun_dec, "Sun") if mode == "SOLAR" else
            (moon_ra, moon_dec, "Moon") if mode == "LUNAR" else
            (None, None, None)
        )
        if (cmd_ra is not None and self.tracking and self.target_object_name == expected_name
                and self.serial and self.serial.ser and self.serial.ser.is_open):
            self.serial.send_command(f"CMD,SET_TARGET,RA:{cmd_ra:.6f},DEC:{cmd_dec:.6f},CONTINUATION:1")

    def _cycle_viz_view_mode(self):
        order = ["FREE", "TELESCOPE", "TARGET"]
        labels = {"FREE": "View: Free", "TELESCOPE": "View: Follow Telescope", "TARGET": "View: Follow Target"}
        self._viz_view_mode = order[(order.index(self._viz_view_mode) + 1) % len(order)]
        self.viz_mode_btn.configure(text=labels[self._viz_view_mode])
        self._redraw_viz()

    def _on_viz_mouse_move(self, event):
        """Show the RA/DEC under the cursor below the canvas (not drawn inside it), and hover-
        highlight the nearest sky object of ANY kind - star, Messier DSO, ISS, Sun, or Moon -
        showing its name (plus a short extra detail: magnitude for stars, type for DSOs, etc.).

        The white ring always shows for whatever's hovered. The floating name text next to it is
        skipped for objects that ALREADY have a permanent on-canvas label (Sun/Moon/ISS always
        do; DSOs do once zoomed in enough - see the "labeled" flag set in
        _draw_sky_objects/_draw_viz_grid) - only that redundant second copy of the name is
        suppressed, not the hover indicator itself.

        Hit-tests against self._visible_object_hits, built fresh each redraw by _draw_sky_objects
        (stars/DSOs) and the ISS/Sun/Moon drawing code in _draw_viz_grid, rather than against the
        full ~62000-star catalog - keeps this cheap even though it runs on every mouse-move.
        Each entry carries its own hit radius AND that same radius is used to draw the hover
        ring, so e.g. hovering a large DSO or the Sun/Moon draws a ring that actually
        circumscribes the object instead of a fixed tiny circle sitting inside it.

        Also remembers the hovered hit (self._hovered_sky_object) so a double-click while
        hovering can target - see _on_viz_double_click - the object's own precise RA/DEC rather
        than whatever the cursor pixel happens to correspond to."""
        c = self.viz_canvas
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        ra = self._viz_x_to_ra(event.x, w, margin, ra_min, ra_span) % 360.0
        dec = self._viz_y_to_dec(event.y, h, margin, dec_min, dec_span)
        dec = max(-90.0, min(90.0, dec))

        hit = None
        best_d2 = None
        for obj in getattr(self, "_visible_object_hits", []):
            d2 = (obj["x"] - event.x) ** 2 + (obj["y"] - event.y) ** 2
            if d2 < obj["radius"] ** 2 and (best_d2 is None or d2 < best_d2):
                best_d2 = d2
                hit = obj

        c.delete("hover")
        self._hovered_sky_object = hit
        if hit is None:
            self.viz_cursor_label.configure(text=f"Cursor: RA {ra:.4f}°  DEC {dec:+.4f}°")
            return

        hx, hy, name, extra = hit["x"], hit["y"], hit["name"], hit["extra"]
        self.viz_cursor_label.configure(
            text=f"Cursor: RA {ra:.4f}°  DEC {dec:+.4f}°   |   {name}  ({extra})")

        # The white ring always shows, regardless of "labeled" - it's the hover indicator, not
        # a duplicate of anything. Only the floating name TEXT is skipped for objects that
        # already have a permanent on-canvas label (Sun/Moon/ISS always do; DSOs do once zoomed
        # in enough - see the "labeled" flag set in _draw_sky_objects/_draw_viz_grid), since
        # THAT specifically would just be a redundant second copy of a name already shown there.
        r = hit["radius"]
        c.create_oval(hx - r, hy - r, hx + r, hy + r, outline="#ffffff", width=1, tags="hover")
        if not hit.get("labeled"):
            c.create_text(hx + r + 4, hy - r, text=name, fill="#ffffff", font=("Consolas", 9, "bold"),
                          anchor="w", tags="hover")

    def _on_viz_double_click(self, event):
        """Double-click on the sky viz: same targeting behavior as picking a result from the
        sky object search box - see _select_sky_target.

        If an (unlabeled - see _on_viz_mouse_move) object is currently hovered, targets that
        object's own precise catalog RA/DEC instead of wherever the cursor pixel happens to
        land - a few pixels of hover tolerance can correspond to a meaningful chunk of a degree
        at low zoom, so "I was hovering Vega and double-clicked" should hit Vega exactly, not
        some nearby point that merely happened to be under the pointer."""
        if self._hovered_sky_object is not None:
            hit = self._hovered_sky_object
            self._select_sky_target(hit["ra"], hit["dec"], f"Viz double-click on {hit['name']}",
                                     target_name=hit["name"])
            return

        c = self.viz_canvas
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        ra = self._viz_x_to_ra(event.x, w, margin, ra_min, ra_span) % 360.0
        dec = max(-90.0, min(90.0, self._viz_y_to_dec(event.y, h, margin, dec_min, dec_span)))
        self._select_sky_target(ra, dec, f"Viz double-click at RA={ra:.4f} DEC={dec:.4f}")

    def _select_sky_target(self, ra, dec, source_label, target_name=None):
        """The clicked/selected RA/DEC always gets written into the target/calibrate fields. If
        tracking is already active, that's ALL this does - it must not interrupt a running
        session. If not tracking, it also GoTos there and starts sidereal tracking (sidereal
        only - neither a double-click nor a search result is a good way to pick solar/lunar
        targets, which follow a specific body rather than a fixed sky point - even when the
        search result WAS the Sun or Moon, tracking it as a fixed sidereal point is still exactly
        as valid as tracking any other star, just not moving with the body's own proper motion).

        source_label is just for the log message, so double-click and search-selection produce
        distinguishable log entries despite sharing this same logic. target_name is the object's
        display name (e.g. "Vega", "M31") when this selection came from a named catalog object -
        None for an arbitrary sky point (double-click on empty sky) - see target_object_name /
        _update_target_name_label."""
        self.target_object_name = target_name
        self._update_target_name_label()
        ra_str, dec_str = f"{ra:.4f}", f"{dec:.4f}"
        self._target_name_box_ra_str = ra_str
        self._target_name_box_dec_str = dec_str
        self.goto_ra.delete(0, "end")
        self.goto_ra.insert(0, ra_str)
        self.goto_dec.delete(0, "end")
        self.goto_dec.insert(0, dec_str)

        if self.tracking:
            self._log(f"{source_label}: placed in target fields (tracking already active - not slewing)")
            return

        if not (self.serial.ser and self.serial.ser.is_open):
            self._log(f"{source_label}: RA/DEC placed in target fields (not connected, so not starting tracking).")
            return

        self.target_ra = ra
        self.target_dec = dec
        if self.mode_var.get() != "SIDEREAL":
            self.mode_var.set("SIDEREAL")
            self._on_mode_changed()
        self._log(f"{source_label}: starting sidereal tracking there")
        self._toggle_tracking()

        # self.tracking is now True (set by _toggle_tracking above) - only past this point does
        # _iss_update_tick/_on_iss_position's "actively tracking ISS" check see it that way, so
        # the real-time refresh loop is kicked off here, not before.
        if target_name == "ISS" and self.tracking:
            if not self._iss_enabled:
                self._iss_enabled = True
                self.iss_toggle_btn.configure(text="ISS: ON", fg_color="#006400")
            if self._iss_update_after_id is not None:
                self.after_cancel(self._iss_update_after_id)
                self._iss_update_after_id = None
            self._iss_update_tick()

    def _update_sky_search_results(self):
        """Live-filter self._sky_search_index by substring match as the search box is typed
        into, showing/hiding the results Listbox depending on whether there's anything to show
        (rather than always reserving space for it)."""
        query = self.sky_search_var.get().strip().lower()
        self.sky_search_results.delete(0, "end")
        if not query:
            self._sky_search_matches = []
            self.sky_search_results.pack_forget()
            return
        matches = [entry for entry in self._sky_search_index if query in entry["name"].lower()]
        # Prefer matches where the query is a prefix (e.g. "vega" over something that merely
        # contains "vega" mid-string), then shortest name first, capped to a manageable list.
        matches.sort(key=lambda e: (not e["name"].lower().startswith(query), len(e["name"])))
        self._sky_search_matches = matches[:30]
        if not self._sky_search_matches:
            self.sky_search_results.pack_forget()
            return
        for entry in self._sky_search_matches:
            self.sky_search_results.insert("end", entry["name"])
        self.sky_search_results.pack(fill="x", padx=4, before=self.viz_canvas)

    def _focus_sky_search_results(self, event=None):
        """Down arrow from the search entry moves focus into the results list (if any)."""
        if self._sky_search_matches:
            self.sky_search_results.focus_set()
            self.sky_search_results.selection_set(0)
        return "break"

    def _select_first_sky_search_result(self, event=None):
        """Enter in the search entry itself picks the top (best-ranked) match directly."""
        if self._sky_search_matches:
            self._choose_sky_search_entry(self._sky_search_matches[0])

    def _on_sky_search_result_chosen(self, event=None):
        """Double-click or Enter on an item in the results Listbox."""
        sel = self.sky_search_results.curselection()
        if not sel:
            return
        self._choose_sky_search_entry(self._sky_search_matches[sel[0]])

    def _choose_sky_search_entry(self, entry):
        """Resolve the chosen search entry to a real RA/DEC (live-looked-up for Sun/Moon/ISS/
        planets, since those move - see _build_sky_catalog_state) and target it, same as a viz
        double-click. Clears the search box/results afterward."""
        if entry["live"] is not None:
            live_lookup = {
                "Sun": (self._sun_ra, self._sun_dec),
                "Moon": (self._moon_ra, self._moon_dec),
                "ISS": (self._iss_ra, self._iss_dec),
            }
            # self._planet_positions entries are (ra, dec, ang_diam, ring_ang_diam) 4-tuples (see
            # solar_system.get_planets_info) - only ra/dec matter for targeting, so pull those out
            # rather than merging the raw 4-tuples in directly (which would break the 2-value
            # unpack below the moment a planet is looked up).
            for pname, ppos in self._planet_positions.items():
                live_lookup[pname] = (ppos[0], ppos[1])
            live_ra, live_dec = live_lookup.get(entry["live"], (None, None))
            if live_ra is None:
                if entry["live"] == "ISS":
                    self._log(f"{entry['live']} position isn't available - enable its tracking toggle first.")
                else:
                    self._log(f"{entry['live']} position isn't available yet - still fetching, try again shortly.")
                return
            ra, dec = live_ra, live_dec
        else:
            ra, dec = entry["ra"], entry["dec"]

        self.sky_search_var.set("")
        self.sky_search_results.delete(0, "end")
        self.sky_search_results.pack_forget()
        self._select_sky_target(ra, dec, f"Search: {entry['name']}", target_name=entry["name"])

    def _on_viz_mouse_leave(self, event=None):
        self.viz_canvas.delete("hover")
        self._hovered_sky_object = None
        self.viz_cursor_label.configure(text="Cursor: —")

    def _force_stop_tracking(self):
        """Force tracking off (used when changing modes etc)."""
        if self.tracking:
            self._request_tracking_action("STOP", self.serial.send_stop, "mode change", "STOPPING (mode change)…")
            self.slewing = False
            self._update_slew_button()

    def _request_tracking_action(self, action: str, send_fn, reason: str, status_text: str):
        """Send a START or STOP command and keep resending until the Arduino actually confirms
        it, instead of assuming the first attempt worked.

        Previously START was pure fire-and-forget (no confirmation at all), and STOP had its
        own separate, STOP-only confirmation mechanism. That split allowed a real race: a START
        that was slow to reach/be processed by the Arduino (write_timeout=0 non-blocking writes
        can silently write fewer bytes than given, and the Arduino can be momentarily busy and
        slow to service its RX buffer) could still be "in flight" when the user, seeing nothing
        happen, clicked Stop - and nothing tied those two together, so the stale START could
        land on the Arduino right as/after Stop did, briefly kicking off alignment right when
        the user expected a stop.

        Now both actions share one pending-confirmation slot: starting a NEW action (of either
        kind) immediately cancels whatever retry was still pending from a previous one, so a
        stale START can't keep resending itself after the user has already asked to STOP (or
        vice versa) - the two can't race each other, the latest click always wins.
        """
        if not (self.serial and self.serial.ser and self.serial.ser.is_open):
            self._log("Not connected.")
            return
        if self._pending_action_after_id is not None:
            try:
                self.after_cancel(self._pending_action_after_id)
            except Exception:
                pass
            self._pending_action_after_id = None
        self._pending_action = action
        self._pending_action_confirmed = False
        self._pending_action_retry_count = 0
        self.tracking = (action == "START")
        # Set optimistically so the stability badge doesn't flash stale SETTLING/STABLE/
        # DRIFTING (left over from a previous tracking session) in the gap before the first
        # STATUS:RESETTING/STOPPED confirmation arrives.
        self._align_phase = "ALIGNING" if action == "START" else "IDLE"
        # Clear the error-trend history too, not just the phase - otherwise stale (error, time)
        # pairs from the PREVIOUS tracking session are still sitting in the deques once TRACKING
        # phase is reached, and _update_tracking_stability's windowed derivative mixes them into
        # the trend, making a genuinely fresh/stable start look like it's still SETTLING or even
        # DRIFTING for up to TRACKING_STABILITY_WINDOW_S. This was the root cause of "tracking
        # sometimes needs stop+restart to show stable" - it looked identical to real settling, so
        # it went unnoticed until traced here.
        self._err_ra_history.clear()
        self._err_dec_history.clear()
        self._update_tracking_button()
        self._update_status_display(status_text, "#ffaa00")
        ok = send_fn()
        self._log(f"Sent {action} ({reason})" if ok else f"{action} write failed ({reason}) - will retry")
        # STOP retries much faster and more times than START - this is the emergency-stop path,
        # so "give up and just log a warning" after a couple seconds isn't acceptable the way it
        # might be for a slew start. (The '!' panic-byte sentinel - see send_stop() - is what
        # actually guarantees near-instant action; this retry loop is the backstop in case that
        # byte itself somehow doesn't arrive, e.g. a genuinely dead connection.)
        retry_ms = 150 if action == "STOP" else 800
        self._pending_action_after_id = self.after(retry_ms, lambda: self._verify_pending_action(action, send_fn))

    def _verify_pending_action(self, action: str, send_fn):
        self._pending_action_after_id = None
        # A different action superseded this one, or it already got confirmed - nothing to do.
        if self._pending_action != action or self._pending_action_confirmed:
            return
        if not (self.serial and self.serial.ser and self.serial.ser.is_open):
            return
        retry_ms = 150 if action == "STOP" else 800
        max_retries = 20 if action == "STOP" else 3
        if self._pending_action_retry_count >= max_retries:
            self._log(f"WARNING: Arduino never confirmed {action} after retries - check the connection/mount.")
            self._update_status_display(f"{action} NOT CONFIRMED - CHECK CONNECTION", "#ff4444")
            return
        self._pending_action_retry_count += 1
        ok = send_fn()
        self._log(f"{action} not yet confirmed, resending (attempt {self._pending_action_retry_count + 1})" if ok
                   else f"{action} resend also failed to write")
        self._pending_action_after_id = self.after(retry_ms, lambda: self._verify_pending_action(action, send_fn))

    def _handle_message(self, msg: str):
        if msg.startswith("CONNECTED:"):
            port = msg.split(":", 1)[1]
            self.conn_status.configure(text=f"● Connected {port}", text_color="#00FF88")
            self.arduino_debug_enabled = False
            self._update_debug_button()
            self._update_status_display("CONNECTED - WAITING FOR PING RESPONSE", "#ffaa00")
            self._log(f"Connected to {port}. Waiting for Arduino response...")

            # Send current (real) GPS coords from GUI fields to Arduino
            self._set_location()

            # Send any existing cumulative offsets to Arduino
            self._send_current_offsets()

            # Apply current GUI rate to Arduino shortly after connect
            self.after(1200, self._apply_initial_rate)

            # Re-sync the meridian-limit toggle too - see _apply_initial_meridian_limit's comment.
            self.after(1200, self._apply_initial_meridian_limit)

        elif msg.startswith("RX:STATUS:PONG"):
            # Arduino responded to our PING command - connection is fully established
            if self._connection_timeout_id is not None:
                self.after_cancel(self._connection_timeout_id)
                self._connection_timeout_id = None
            self._update_status_display("CONNECTED - READY", "#00ff88")
            self._log("Arduino responded to PING - connection fully established")

        elif msg.startswith("DISCONNECTED"):
            self.conn_status.configure(text="● Disconnected", text_color="gray")
            self.tracking = False
            self._update_tracking_button()
            self.slewing = False
            self._update_slew_button()
            self.arduino_debug_enabled = False
            self._update_debug_button()
            self._update_mode_label()
            self._update_status_display("DISCONNECTED", "#ff4444")
            self._log("Disconnected")
        elif msg.startswith("STATUS:FIRMWARE:"):
            firmware_version = msg.replace("STATUS:FIRMWARE:", "")
            self._log(f"Arduino firmware version: {firmware_version}")
            self.status_display.configure(text=f"CONNECTED - Firmware {firmware_version}", text_color="#00ff88")
            
            # Request Arduino to update to latest firmware if needed
            if "1.0" in firmware_version or "1.1" in firmware_version:
                self._log("Note: Newer firmware available. Consider updating for best performance.")
        elif msg.startswith("STATUS:OFFSET_APPLIED"):
            self._log("Alignment offset applied successfully")
            self._update_status_display("OFFSET APPLIED", "#00ff88")
        elif msg.startswith("ERROR:NOT_CALIBRATED"):
            self._log("ERROR: Cannot apply offset - mount not calibrated. Sync first.")
            self.error_label.configure(text="Error: Not calibrated - sync required")

        elif msg.startswith("RX:"):
            line = msg[3:]
            self._parse_arduino_line(line)

        elif msg.startswith("TX:"):
            # Log sent commands (especially useful to confirm what SYNC actually transmitted)
            if "SYNC" in msg or "GOTO" in msg:
                self._log(msg)
            # Optionally log all by removing the if, but SYNC/GOTO are the critical ones

        elif msg.startswith("ERROR:"):
            self._log("ERROR: " + msg[6:])
            self.error_label.configure(text="Error: " + msg[6:])

    def _parse_arduino_line(self, line: str):
        line = line.strip()
        if not line:
            return

        if line.startswith("DEBUG:"):
            self._log(line)
            # This is the earliest possible proof the Arduino actually received a START -
            # printed unconditionally at the top of parseAndExecuteCommand, before any
            # alignment-state logic runs - so it's used as START's confirmation signal (STOP
            # is instead confirmed by STATUS:STOPPED, since stopAll() itself is effectively
            # instantaneous so there's no meaningful gap to close by confirming earlier).
            if "CMD_ACTION:START_TRACKING" in line and self._pending_action == "START":
                self._pending_action_confirmed = True
            # fall through in case we add handling later, but don't treat as POS/STATUS

        # STATUS messages - update prominent top status
        if line.startswith("STATUS:"):
            # WAITING lines used to stream at 10 Hz for up to 3s per axis during alignment
            # (firmware's ALIGN_WAIT_RA/ALIGN_WAIT_DEC settle/verify phases) - logging every one
            # flooded the Textbox (each _log() call does an insert + index + see("end")) and
            # backed up the message queue badly enough that later log entries got timestamped
            # minutes late, which was the likely cause of the stability badge occasionally
            # appearing stuck on ALIGNING until Stop+restart (the final TRACKING_STARTED line
            # queued up behind a backlog of WAITING spam). Firmware 1.8.35 removed those wait
            # phases entirely, so STATUS:WAITING should no longer actually be sent - this check is
            # kept only as a harmless no-op guard against older firmware still running it.
            if not line.startswith("STATUS:WAITING"):
                self._log(line)
            self.last_status = line

            # Only update the TOP status bar for important messages.
            # Ignore routine ones like TIME_SET, LOCATION_SET, READY, UPDATE_RATE etc.
            # so they don't override tracking/alignment status on the prominent bar.
            # (All still logged in the status text box.)
            important = ["TRACKING", "ALIGNING", "RESETTING", "WAITING", "STOPPED", "SYNCED", "ERROR", "SLEW",
                         "FLIPPING", "FLIP_COMPLETE", "AUTO_CALIBRATED"]
            if any(kw in line for kw in important):
                # Update big clear status at top
                status_text = line.replace("STATUS:", "STATUS: ")
                color = "#00ccff"
                if "MERIDIAN_LIMIT" in line and "STOPPED" in line:
                    # More actionable than the generic STOPPED text below - this specific stop
                    # means the mount can't safely continue without a manual flip (see the .ino's
                    # pastMeridianLimit()/stopForMeridianLimit()), not just "someone hit Stop".
                    status_text = "STOPPED - MERIDIAN LIMIT REACHED - flip the telescope in the clamps, then press Telescope Flipped ON to resume"
                    color = "#ff6666"
                    self.tracking = False
                    self._update_tracking_button()
                    self._align_phase = "IDLE"
                elif "TRACKING_STARTED" in line or "TRACKING" in line:
                    color = "#00ff88"
                    self.tracking = True
                    self._update_tracking_button()
                    # The RA-only TRACKING_STARTED that fires partway through alignment (RA
                    # settled, DEC not yet aligned) deliberately does NOT count as the full
                    # sequence being done - only the final (DEC) one does, so the stability
                    # badge doesn't start judging error trend before DEC has even moved.
                    if "AXIS:RA" not in line:
                        # Clearing on the initial Start click (_request_tracking_action) isn't
                        # enough by itself: _err_ra_history/_err_dec_history keep accumulating
                        # unconditionally on every POS line straight through the ALIGNING slew
                        # that follows (RESETTING_DEC/RA/WAIT_RA/DEC/WAIT_DEC), where error is
                        # naturally large and fast-changing - exactly the kind of data this
                        # history exists to flag as unstable. By the time this STATUS line
                        # actually flips align_phase to TRACKING, the window is already
                        # re-poisoned with that slew noise, so a genuinely fresh, already-
                        # converged tracking session can still read as SETTLING/DRIFTING right
                        # out of the gate. Clear again right here, at the true ALIGNING->TRACKING
                        # transition, so the trend is judged only on data from actual tracking,
                        # never the slew that preceded it.
                        if self._align_phase != "TRACKING":
                            self._err_ra_history.clear()
                            self._err_dec_history.clear()
                        self._align_phase = "TRACKING"
                elif "ALIGNING" in line or "RESETTING" in line or "WAITING" in line:
                    color = "#ffaa00"
                    self._align_phase = "ALIGNING"
                elif "FLIPPING" in line:
                    # RA is physically rotating ~180° - see setTelescopeFlipped() in the .ino.
                    # self._telescope_flipped/the toggle button only update once POS/FLIP_COMPLETE
                    # confirms the NEW state, not here (this is just "in progress").
                    color = "#ffaa00"
                elif "FLIP_COMPLETE" in line:
                    color = "#00ccff"
                elif "STOPPED" in line or "ERROR" in line:
                    color = "#ff6666"
                    self.tracking = False
                    self._update_tracking_button()
                    self._align_phase = "IDLE"

                self._update_status_display(status_text, color)

            # Rate confirmation from Arduino - the rate isn't user-adjustable from the GUI, but
            # this is the actual value the Arduino applied (not just what the GUI requested), so
            # the label is updated from this, not from POS_UPDATE_RATE_MS directly - previously
            # the label was a static string set once at UI-build time and never touched again, so
            # it silently didn't reflect the real rate.
            if "UPDATE_RATE" in line or "UPDATE_RATE,MS:" in line:
                m = re.search(r"MS:(\d+)", line)
                if m:
                    confirmed_ms = int(m.group(1))
                    self._log(f"Arduino confirmed POS rate: {confirmed_ms} ms")
                    if hasattr(self, 'pos_rate_label'):
                        hz = 1000.0 / confirmed_ms if confirmed_ms > 0 else 0.0
                        self.pos_rate_label.configure(
                            text=f"Arduino POS Update Rate: {confirmed_ms} ms (~{hz:.1f} Hz)")

            # Firmware-confirmed meridian-limit enabled state - from our own SET_MERIDIAN_LIMIT or
            # a QUERY_MERIDIAN_LIMIT reply on connect (see _apply_initial_meridian_limit). Requires
            # firmware 1.8.52+; silently ignored (no attribute error) on older firmware that never
            # sends this line, so the GUI just keeps assuming its own default (enabled).
            if "MERIDIAN_LIMIT_ENABLED" in line:
                m = re.search(r"MERIDIAN_LIMIT_ENABLED,(\d)", line)
                if m:
                    self._on_meridian_limit_confirmed(m.group(1) == "1")

            # Extract error if present (from STATUS during alignment)
            if "ERROR:" in line:
                m = re.search(r"ERROR:([-\d.]+)", line)
                if m:
                    err = float(m.group(1))
                    # Update live error for the relevant axis if we can tell
                    if "AXIS:RA" in line:
                        self.err_ra = err
                    elif "AXIS:DEC" in line:
                        self.err_dec = err
                    self._update_live_error()
                    self.error_label.configure(text=f"Alignment error: {err:.6f}°")

            # Handle EEPROM responses (requires firmware support)
            if "EEPROM_SAVED" in line:
                # Transparent to user - no logging
                pass
            elif "EEPROM_LOADED" in line:
                # Arduino's setup() loads whatever is in EEPROM unconditionally and reports it
                # here - completely unsolicited from the GUI's perspective. This used to
                # overwrite lat_var/lon_var/gps_var with it, which meant that if the Arduino
                # happened to (re)boot around connect time, this stale/unsolicited broadcast
                # would silently clobber the location just correctly loaded from
                # gui_config.json. The GUI's config file is the source of truth for location
                # (it's what gets PUSHED to the Arduino via _set_location() on every connect) -
                # it must never be overwritten by whatever Arduino's EEPROM happens to hold, so
                # this is now purely informational (log only).
                if "LAT:" in line and "LON:" in line:
                    try:
                        lat_str = line.split("LAT:")[1].split(",")[0]
                        lon_str = line.split("LON:")[1].strip()
                        self._log(f"Arduino's saved EEPROM location is {lat_str}, {lon_str} (not applied - using your configured location instead)")
                    except:
                        pass
            elif "LOCATION_QUERY" in line:
                # Same reasoning as EEPROM_LOADED above - informational only, never overwrites
                # the GUI's own (authoritative) location fields.
                if "LAT:" in line and "LON:" in line:
                    try:
                        lat_str = line.split("LAT:")[1].split(",")[0]
                        lon_str = line.split("LON:")[1].strip()
                        self._log(f"Arduino currently reports its location as {lat_str}, {lon_str}")
                    except:
                        pass

            if "TRACKING_STARTED" in line:
                self.tracking = True
                self.slewing = False
                self._update_tracking_button()
                self._update_slew_button()
            if "STOPPED" in line:
                self.tracking = False
                self._update_tracking_button()
                if self._pending_action == "STOP":
                    self._pending_action_confirmed = True

            self._update_mode_label()

        # POS update
        elif line.startswith("POS,"):
            # Positional CSV (see the matching comment above sendPositionUpdate() in the .ino):
            # POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking,flipped
            # mode is a single char: S=SIDEREAL, O=SOLAR, L=LUNAR. tracking and flipped are 0/1.
            # ERR_RA/ERR_DEC are no longer sent on the wire - reconstructed below as abs(sky-target)
            # to save bytes (message used to blow past the Mega's 64-byte Serial TX buffer).
            fields = line.split(",")

            try:
                ra_val = float(fields[1])
                dec_val = float(fields[2])
                # Debug aid: log when sky jumps a lot (e.g. on sync). If you never see "POS sky jump" after clicking sync,
                # then either Arduino didn't send a calibrated POS or it didn't reach this parse branch.
                if abs(ra_val - self.current_ra) > 1.0 or abs(dec_val - self.current_dec) > 1.0:
                    self._log(f"DEBUG:POS sky jump RA={ra_val:.4f} DEC={dec_val:.4f}")
                self.current_ra = ra_val  # sky from Arduino ONLY (drifts with Earth rotation when not tracking)
                self.current_dec = dec_val  # sky from Arduino ONLY (drifts with Earth rotation when not tracking)
                # (labels + viz are refreshed once below, after target/error/mount/speed are all set)
                # Skip while the target is a live-moving body (ISS/Sun/Moon/a planet) -
                # _on_iss_position/_on_solar_system_position already keep target_ra/dec fresh from
                # the GUI's own live skyfield computation, tracking or not - see
                # _target_is_live_body's comment for why the firmware's echo isn't good enough for
                # these. Previously this always overwrote unconditionally, which is exactly what
                # made the target reticle freeze once tracking stopped (no more fresh SET_TARGETs
                # for the echo to reflect) - reported as "when I stop tracking the ISS, the target
                # stays at the position I stopped tracking instead of following the actual target."
                if not self._target_is_live_body():
                    self.target_ra = float(fields[3])
                    self.target_dec = float(fields[4])
                self.mount_ra_angle = float(fields[5])
                self.mount_dec_angle = float(fields[6])
                self.err_ra = abs(self.current_ra - self.target_ra)
                self.err_dec = abs(self.current_dec - self.target_dec)

                # Calculate speed on GUI side from mount angle deltas over last 250ms window for smoothing.
                # Average speed = total delta angle / total delta time over the points in the window.
                # This makes the displayed speed update at the exact same time as the position
                # and smooths out spikes (e.g. 17°/s jumps from single noisy delta).
                current_time = time.time()
                # Trim history older than ~1s to keep it bounded (calc only uses last 250ms anyway).
                # Entries are appended in time order, so stale ones are always at the left end -
                # popleft() them off instead of rebuilding the whole list every POS update.
                cutoff = current_time - 1.0
                self.ra_pos_history.append( (self.mount_ra_angle, current_time) )
                while self.ra_pos_history and self.ra_pos_history[0][1] < cutoff:
                    self.ra_pos_history.popleft()
                self.speed_ra = self._calc_smoothed_speed(self.ra_pos_history)

                self.dec_pos_history.append( (self.mount_dec_angle, current_time) )
                while self.dec_pos_history and self.dec_pos_history[0][1] < cutoff:
                    self.dec_pos_history.popleft()
                self.speed_dec = self._calc_smoothed_speed(self.dec_pos_history)
            except (ValueError, TypeError, IndexError):
                pass

            # Sky RA/DEC comes purely from Arduino's physics-based calculation (drift when not tracking)
            # mode/self.current_mode is NOT synced from the Arduino's echoed mode char here (used
            # to be) - the wire protocol only ever reports 'S' now, since SOLAR/LUNAR are GUI-side
            # pseudo-modes on top of SIDEREAL (see _on_mode_changed); trusting the echo would
            # stomp the user's SOLAR/LUNAR radio button selection back to SIDEREAL on every POS
            # update. self.current_mode/mode_var are purely local state now, same as
            # target_object_name already was for the ISS.

            if len(fields) > 8:
                pos_track = fields[8].strip() == "1"
                if pos_track:
                    # Only trust "tracking on" from POS. Never flip back to False from POS
                    # (POS during alignment often reports 0 even after we started)
                    if not self.tracking:
                        self.tracking = True
                        self._update_tracking_button()

            if len(fields) > 9:
                # Lets a reconnecting GUI learn/restore the real flip state instead of always
                # assuming OFF (which could otherwise cause it to send an erroneous un-flip) -
                # see _on_telescope_flipped_confirmed.
                try:
                    self._on_telescope_flipped_confirmed(fields[9].strip() == "1")
                except (ValueError, IndexError):
                    pass

            self._update_mode_label()
            # _apply_viz_follow_mode() IS called here (cheap - just updates the two
            # _viz_center_ra/dec floats and clamps them, no drawing) even though the expensive
            # full _draw_viz_grid() rebuild stays on its own separate follow_tick() timer (see
            # _start_viz_follow_timer/VIZ_FOLLOW_TICK_MS). Without this, the marker positioning
            # below used whatever center follow_tick's OWN independent ~20ms timer last set - two
            # separately-scheduled ~20ms timers (this POS handler's arrival rate and follow_tick's
            # self.after chain) drift in and out of phase with each other rather than firing in
            # lockstep, so the center the marker was drawn against was sometimes a full tick
            # stale relative to self.current_ra - reported as the telescope view occasionally
            # "jumping" relative to the frame. Recomputing the center fresh right before
            # positioning the marker removes that beat-frequency mismatch entirely; the
            # background/grid itself (from the last _draw_viz_grid() rebuild) can still be up to
            # one follow_tick behind, but that residual gap is now consistently small instead of
            # occasionally doubled by unlucky timer phase, and _draw_viz_grid() was already
            # confirmed to keep up at VIZ_FOLLOW_TICK_MS's current rate.
            self._apply_viz_follow_mode()
            now = time.time()
            if now - self._last_pos_ui_update >= POS_UI_UPDATE_MIN_INTERVAL_S:
                self._last_pos_ui_update = now
                self._update_visualization()
                self._update_labels()  # updates sky + mount positions + error together
                self._update_live_error()  # error updated together with position (anytime via POS)
                self._update_meridian_warning()

        elif line.startswith("STATUS:SYNCED") or line.startswith("STATUS:TIME_SET"):
            self._log(line)
        elif "LOCATION_SET" in line:
            self._log(line)
            # This just confirms the location the GUI itself already sent via _set_location() -
            # update the on-screen label to show it's been confirmed, but don't write back into
            # lat_var/lon_var/gps_var (the GUI's own config is the source of truth for those;
            # see the EEPROM_LOADED/LOCATION_QUERY handlers above for why blindly trusting
            # Arduino's echo to overwrite them is exactly the wrong direction).
            try:
                if "LAT:" in line and "LON:" in line:
                    lat_str = line.split("LAT:")[1].split(",")[0]
                    lon_str = line.split("LON:")[1].strip()
                    if self.gps_mode_var.get():
                        self._set_location_label(f"Loc: {lat_str}, {lon_str} (on Arduino)")
                    else:
                        self._set_location_label(f"Loc: {lat_str}N {lon_str}E (on Arduino)")
            except:
                pass
        elif "SLEW_STOPPED" in line or "SLEWING" in line:
            self._log(line)
            if "SLEW_STOPPED" in line:
                self.slewing = False
                self._update_slew_button()

    def _calc_smoothed_speed(self, history, window_s=0.250):
        """Compute a rate of change using the current + last positions within max window_s.
        If the immediate previous pos was older than that, fall back to that single delta.
        history: list/deque of (value, timestamp) tuples. Also used for the tracking error
        trend (see _update_tracking_stability), not just RA/DEC speed - "angle" below is
        just whatever value is being tracked.
        """
        if len(history) < 2:
            return 0.0
        curr_ang, curr_t = history[-1]
        window_start = curr_t - window_s

        # Find first index >= window_start
        start_idx = 0
        for i, (ang, t) in enumerate(history):
            if t >= window_start:
                start_idx = i
                break

        if start_idx == len(history) - 1:
            # previous was outside 250ms window → use single last delta
            prev_ang, prev_t = history[-2]
            dt = curr_t - prev_t
            if dt > 0:
                return (curr_ang - prev_ang) / dt
            return 0.0

        # average over the window (total delta / total time)
        total_dangle = 0.0
        total_dt = 0.0
        for i in range(start_idx + 1, len(history)):
            dangle = history[i][0] - history[i-1][0]
            dt = history[i][1] - history[i-1][1]
            if dt > 0:
                total_dangle += dangle
                total_dt += dt
        if total_dt > 0:
            return total_dangle / total_dt
        return 0.0

    def _update_labels(self):
        # Sky positions - PURELY from Arduino (RA/DEC fields): celestial coords with Earth rotation drift when not tracking
        # Field widths sized to the values' actual bounded ranges, not an arbitrary round number:
        # RA is always 0-360 (3 integer digits, no sign needed) -> width 8 ("XXX.XXXX");
        # DEC is always +-90 (2 integer digits, always signed) -> width 8 ("+XX.XXXX"). The
        # previous width (10 for both) padded in 1-2 extra leading zeros neither value can ever
        # actually use.
        self.ra_label.configure(text=f"{self.current_ra:08.4f}°")
        self.dec_label.configure(text=f"{self.current_dec:+08.4f}°")

        # Mount physical (relative to Earth/mount) - steps no longer displayed or sent
        # 4 decimals, not 6 - matches the precision the Arduino actually sends on the wire (see
        # sendPositionUpdate() in the .ino), so this doesn't display false precision.
        self.mount_ra_label.configure(text=f"{self.mount_ra_angle:.4f}°")
        self.mount_dec_label.configure(text=f"{self.mount_dec_angle:.4f}°")

        self.speed_ra_label.configure(text=f"{self.speed_ra:.6f} °/s")
        self.speed_dec_label.configure(text=f"{self.speed_dec:.6f} °/s")

        self._update_live_error()

    def _update_status_display(self, text: str, color: str = "#00ccff"):
        """Update the large clear status at the top of the window."""
        self.status_display.configure(text=text, text_color=color)
        self.system_state = text

    def _update_target_name_label(self):
        """Shows the current target's object name (see target_object_name / _select_sky_target)
        in a big badge matching the stability label's size, or nothing at all for an arbitrary
        sky point that isn't a named catalog object."""
        if hasattr(self, 'target_name_label'):
            self.target_name_label.configure(text=self.target_object_name or "")

    def _update_live_error(self):
        """Update the live alignment/tracking error (updated at the same time as current position via POS, anytime)."""
        # 4 decimals - err_ra/err_dec are derived from RA/DEC values the Arduino only ever sends
        # at 4-decimal precision, so anything past that is fabricated, not real precision.
        if hasattr(self, 'live_error_label'):
            self.live_error_label.configure(
                text=f"Error: RA {self.err_ra:.4f}° | DEC {self.err_dec:.4f}°"
            )
        if hasattr(self, 'error_label'):
            self.error_label.configure(
                text=f"Live Error: RA {self.err_ra:.4f}° | DEC {self.err_dec:.4f}°"
            )

        now = time.time()
        cutoff = now - TRACKING_STABILITY_WINDOW_S
        self._err_ra_history.append((self.err_ra, now))
        while self._err_ra_history and self._err_ra_history[0][1] < cutoff:
            self._err_ra_history.popleft()
        self._err_dec_history.append((self.err_dec, now))
        while self._err_dec_history and self._err_dec_history[0][1] < cutoff:
            self._err_dec_history.popleft()
        self._update_tracking_stability()

    def _is_target_risky(self, ra: float, dec: float) -> bool:
        """Client-side ESTIMATE mirroring the firmware's pastMeridianLimit() (EQMountTracker.ino) -
        used only to decide whether _confirm_risky_slew() should show a warning dialog before
        sending; the firmware re-checks this independently and authoritatively regardless of what
        this returns (a GOTO/START_TRACKING that's actually risky still gets refused there unless
        the RISK_OK flag this dialog attaches is present). DEC within
        MERIDIAN_LIMIT_DEC_ALLOWED_MIN/MAX_DEG is exempt regardless of Hour Angle - see that
        constant's comment in the .ino."""
        if MERIDIAN_LIMIT_DEC_ALLOWED_MIN_DEG <= dec <= MERIDIAN_LIMIT_DEC_ALLOWED_MAX_DEG:
            return False
        lst = self._get_current_lst_deg()
        ha = (lst - ra + 180.0) % 360.0 - 180.0
        return (ha <= 0.0) if self._telescope_flipped else (ha >= 0.0)

    def _confirm_risky_slew(self, ra: float, dec: float) -> bool:
        """Returns True if it's fine to send the GOTO/START_TRACKING - either the target isn't in
        the meridian danger zone, or the user explicitly confirmed it is and chose to proceed
        anyway. The actual command still needs ,RISK_OK:1 attached (see SerialHandler.send_goto/
        send_start_tracking) for the firmware to actually honor an accepted risky slew rather than
        refusing it - this dialog alone doesn't bypass anything firmware-side."""
        if not self._is_target_risky(ra, dec):
            return True
        return messagebox.askyesno(
            "Meridian Limit Warning",
            f"Target RA={ra:.3f}° DEC={dec:.3f}° is past the meridian limit for the "
            f"mount's current configuration - continuing risks the OTA colliding with the "
            f"tripod/mount.\n\nProceed anyway?",
            icon="warning",
        )

    def _update_meridian_warning(self):
        """Client-side ESTIMATE of time remaining until the meridian limit stops tracking (see
        MERIDIAN_LIMIT_MARGIN_DEG/pastMeridianLimit() in the .ino) - purely informational; the
        firmware enforces the actual stop independently and authoritatively, so this can't itself
        cause a missed/late stop, only a slightly-off countdown display. Uses target_ra (not
        current_ra) to match exactly what the firmware's own check is based on. Hidden whenever
        not relevant: not tracking, already flipped (this session's meridian risk is behind the
        mount), or comfortably before the limit."""
        if not self.tracking or self._telescope_flipped or self.mode_var.get() not in ("SIDEREAL", "SOLAR", "LUNAR"):
            self.meridian_warning_label.configure(text="")
            return
        lst = self._get_current_lst_deg()
        ha = (lst - self.target_ra + 180.0) % 360.0 - 180.0
        time_to_limit_s = -ha / SIDEREAL_RATE_DEG_S
        if time_to_limit_s > MERIDIAN_LIMIT_WARNING_S or time_to_limit_s < 0:
            self.meridian_warning_label.configure(text="")
            return
        minutes, seconds = divmod(int(time_to_limit_s), 60)
        urgent = time_to_limit_s <= MERIDIAN_LIMIT_URGENT_S
        self.meridian_warning_label.configure(
            text=f"⚠ MERIDIAN LIMIT IN {minutes}m {seconds:02d}s - flip may be needed",
            text_color="#ff4444" if urgent else "#ffaa00",
        )

    def _set_stability(self, text: str, color: str):
        """Update the stability badge AND recolor the telescope FOV circle AND the center
        reference circle in the sky viz to match, so the same STABLE/SETTLING/DRIFTING/ALIGNING
        state is visible at a glance in all three places instead of staying a fixed color
        regardless of tracking health."""
        self.stability_label.configure(text=text, fg_color=color, text_color="#ffffff")
        self._stability_color = color
        if hasattr(self, 'viz_dot'):
            try:
                self.viz_canvas.itemconfigure(self.viz_dot, outline=color)
            except Exception:
                pass
        if hasattr(self, 'viz_center_ref_circle'):
            try:
                self.viz_canvas.itemconfigure(self.viz_center_ref_circle, outline=color)
            except Exception:
                pass

    def _update_tracking_stability(self):
        """Classify tracking as STABLE/SETTLING/DRIFTING from the *trend* of the error, not
        just its instantaneous size - a large-but-shrinking error is fine (still converging),
        while a small-but-growing one is an early warning, which a plain magnitude threshold
        would miss either way."""
        if not hasattr(self, 'stability_label'):
            return
        if self._align_phase == "ALIGNING":
            # Mid-alignment (RESETTING/ALIGNING/WAITING): the mount hasn't settled onto the
            # target yet at all, so judging the error trend here doesn't mean anything -
            # explicitly distinct from SETTLING (which means "tracking, converging").
            self._set_stability("ALIGNING…", "#3a5a8a")
            return
        if not self.tracking or self._align_phase != "TRACKING":
            self._set_stability("TRACKING: OFF", "#444455")
            return
        if len(self._err_ra_history) < 4 or len(self._err_dec_history) < 4:
            self._set_stability("● SETTLING", "#8a5a00")
            return

        deriv_ra = self._calc_smoothed_speed(self._err_ra_history, window_s=TRACKING_STABILITY_WINDOW_S)
        deriv_dec = self._calc_smoothed_speed(self._err_dec_history, window_s=TRACKING_STABILITY_WINDOW_S)
        worst_err = max(abs(self.err_ra), abs(self.err_dec))
        worst_growth = max(deriv_ra, deriv_dec)  # most-positive axis = worst-case error growth rate

        # Solid-fill badge (not just colored text) with a bold, larger font so this is the kind
        # of thing you can glance at from across the room, not something you have to hunt for.
        if worst_err <= TRACKING_STABLE_ERR_DEG and worst_growth <= TRACKING_STABLE_DERIV_DEG_S:
            self._set_stability("●  STABLE", "#00a050")
        elif worst_growth <= TRACKING_STABLE_DERIV_DEG_S:
            # Error hasn't settled under the tight threshold yet, but it's flat or shrinking,
            # not getting worse - normal while converging or riding out small mechanical noise.
            self._set_stability("●  SETTLING", "#8a5a00")
        else:
            # Error is actively growing - a real tracking problem (lost sync, mechanical
            # slip/backlash, wrong rate, etc).
            self._set_stability("⚠ DRIFTING", "#c02020")

    def _update_mode_label(self):
        track_str = "ON" if self.tracking else "OFF"
        color = "#00FF88" if self.tracking else "#FFAA00"
        self.mode_label.configure(
            text=f"MODE: {self.current_mode}   |   TRACKING: {track_str}",
            text_color=color
        )
        # keep button in sync
        self._update_tracking_button()

    def _log(self, text: str):
        self.status_text.configure(state="normal")
        ts = time.strftime("%H:%M:%S")
        self.status_text.insert("end", f"[{ts}] {text}\n")
        if self._log_file is not None:
            # Full date+time here (unlike the on-screen box's bare HH:MM:SS) - this file
            # persists and accumulates across days/relaunches, so the date matters here in a way
            # it doesn't for the on-screen box (which is always "right now").
            try:
                self._log_file.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {text}\n")
                self._log_file.flush()
            except OSError:
                pass  # e.g. disk full - the on-screen box still has it, don't crash the GUI over this
        # Keep the log box from growing forever (important when Arduino debug dump is ON).
        # The line-count index() call is a Tk round-trip, so only check every few calls
        # instead of on every single line - a handful of extra lines before trimming is
        # harmless, but querying the widget on every log line adds up under load.
        self._log_count = getattr(self, "_log_count", 0) + 1
        if self._log_count % 10 == 0:
            try:
                end_idx = self.status_text.index("end-1c")
                line_count = int(end_idx.split('.')[0])
                if line_count > 350:
                    self.status_text.delete("1.0", f"{line_count - 250}.0")
            except Exception:
                pass
        self.status_text.see("end")
        self.status_text.configure(state="disabled")

    def on_closing(self):
        self.serial.disconnect()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        self.destroy()


if __name__ == "__main__":
    app = EQMountApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

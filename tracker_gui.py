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
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
import math
import json
import os
from collections import deque
from datetime import datetime, timezone
from typing import Optional

# ============================================================
# CONFIG
# ============================================================
BAUD_RATE = 250000
POLL_INTERVAL_MS = 50          # GUI poll rate for queue
POSITION_BROADCAST_HZ = 5

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
VIZ_ZOOM_MAX = 120.0    # ~3 deg RA / ~1.5 deg DEC visible span - eyepiece-FOV scale
VIZ_ZOOM_STEP_BASE = 1.15   # zoom multiplier per "one full wheel notch" worth of scroll delta
VIZ_GRID_TARGET_LINES = 9.0  # aim for roughly this many grid lines per axis at any zoom level
VIZ_FOLLOW_TICK_MS = 400        # how often the follow-mode camera snaps to its target (see
                                 # _apply_viz_follow_mode - no easing, so this can be coarse;
                                 # telescope/target position barely moves between ticks anyway)

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
TRACKING_STABLE_ERR_DEG = CAMERA_FOV_DEG_PER_PIXEL / 2.0   # sub-pixel magnitude, both axes, for STABLE
TRACKING_STABLE_DERIV_DEG_S = 0.0008    # error growth rate (deg/s) below which it's considered flat
TRACKING_STABILITY_WINDOW_S = 2.0       # trend-averaging window - smooths per-update noise

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ============================================================
# SERIAL HANDLER (background thread)
# ============================================================
class SerialHandler:
    def __init__(self, message_queue: queue.Queue):
        self.ser: Optional[serial.Serial] = None
        self.queue = message_queue
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.port = ""
        self.last_error = ""

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
        was silently discarded and a partial write looked identical to a full success."""
        if not (self.ser and self.ser.is_open):
            return False
        payload = (cmd + "\n").encode('ascii')
        try:
            written = self.ser.write(payload)
            self.ser.flush()  # make sure the command actually leaves the buffer (helps with delivery)
            if written != len(payload):
                self.queue.put(f"ERROR:Send incomplete for '{cmd}' ({written}/{len(payload)} bytes) - port may be backed up")
                return False
            self.queue.put("TX:" + cmd)
            return True
        except Exception as e:
            self.queue.put("ERROR:Send failed - " + str(e))
            return False

    def send_time(self):
        now = datetime.now(timezone.utc)
        cmd = (f"CMD,SET_TIME,Y:{now.year},M:{now.month},D:{now.day},"
               f"h:{now.hour},min:{now.minute},s:{now.second}")
        self.send_command(cmd)

    def send_time_if_connected(self):
        if self.ser and self.ser.is_open:
            self.send_time()

    def send_mode(self, mode: str):
        self.send_command(f"CMD,MODE,{mode}")

    def send_start_tracking(self) -> bool:
        return self.send_command("CMD,START_TRACKING")

    def send_start_tracking_skip_dec_reset(self) -> bool:
        return self.send_command("CMD,START_TRACKING,SKIP_DEC_RESET")

    def send_stop(self) -> bool:
        return self.send_command("CMD,STOP")

    def send_sync(self, ra: float, dec: float):
        self.send_command(f"CMD,SYNC,RA:{ra:.6f},DEC:{dec:.6f}")

    def send_sync_offset(self, ra_offset: float, dec_offset: float):
        self.send_command(f"CMD,SYNC_OFFSET,RA:{ra_offset:.6f},DEC:{dec_offset:.6f}")

    def send_goto(self, ra: float, dec: float):
        self.send_command(f"CMD,GOTO,RA:{ra:.6f},DEC:{dec:.6f}")

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
        self.serial = SerialHandler(self.message_queue)

        self.current_ra = 0.0
        self.current_dec = 0.0
        self.target_ra = 0.0
        self.target_dec = 0.0
        self.err_ra = 0.0
        self.err_dec = 0.0
        self.mount_ra_angle = 0.0  # physical mount angle (deg)
        self.mount_dec_angle = 0.0
        self.speed_ra = 0.0
        self.speed_dec = 0.0
        self.ra_pos_history = deque()  # (angle, time) pairs - calc uses positions within last 250ms (or last one if older)
        self.dec_pos_history = deque()
        self._err_ra_history = deque()  # (error_deg, time) pairs - used for tracking stability trend
        self._err_dec_history = deque()
        self.current_mode = "SIDEREAL"
        self.tracking = False
        self.slewing = False
        self.arduino_debug_enabled = False
        self.last_status = "Disconnected"
        self.system_state = "DISCONNECTED"
        self._last_sent_location = None  # Track last sent location to avoid duplicates
        self._connection_timeout_id = None  # For tracking connection timeout

        # Sky viz zoom/pan state. zoom=1.0 shows the full sky (RA 0-360, DEC -90..+90);
        # center is the (RA, DEC) at the middle of the canvas. See _get_viz_view_bounds().
        self._viz_zoom = 1.0
        self._viz_center_ra = 180.0
        self._viz_center_dec = 0.0
        self._viz_pan_last_xy = None
        self._viz_redraw_pending = False  # see _schedule_viz_redraw
        self._viz_view_mode = "FREE"  # FREE -> TELESCOPE -> TARGET -> FREE, see _cycle_viz_view_mode

        # START/STOP confirmation-retry state - see _request_tracking_action
        self._pending_action = None       # "START", "STOP", or None
        self._pending_action_confirmed = True
        self._pending_action_retry_count = 0
        self._pending_action_after_id = None

        # "IDLE" / "ALIGNING" / "TRACKING" - separate from self.tracking (which flips true as
        # soon as RA settles, partway through alignment) so the stability badge can tell
        # "still aligning" apart from "actually holding the target" - see _update_tracking_stability.
        self._align_phase = "IDLE"
        # Current stability badge color, mirrored onto the telescope FOV circle - see _set_stability.
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

        # Bright stars visible from northern hemisphere (approx J2000 RA/DEC in degrees)
        self.cal_stars = {
            "Vega (alpha Lyrae)": (279.234, 38.784),
            "Arcturus (alpha Bootis)": (213.915, 19.182),
            "Altair (alpha Aquilae)": (297.696, 8.868),
            "Custom (use fields below)": None,
        }

        self._build_ui()
        self._poll_queue()
        self._start_time_sync_timer()
        self._start_ground_update_timer()
        self._start_viz_follow_timer()

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

        ctk.CTkLabel(top, text="Connection", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=10)

        self.port_combo = ctk.CTkComboBox(top, values=["Select port..."], width=280)
        self.port_combo.pack(side="left", padx=6)
        self._refresh_ports()

        ctk.CTkButton(top, text="Refresh", width=80, command=self._refresh_ports).pack(side="left", padx=4)
        self.connect_btn = ctk.CTkButton(top, text="Connect", width=90, command=self._connect)
        self.connect_btn.pack(side="left", padx=4)
        self.disconnect_btn = ctk.CTkButton(top, text="Disconnect", width=90, fg_color="#8B0000", command=self._disconnect)
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
                    self.location_label.configure(text=f"Loc: {g} (loaded)")
            except Exception:
                pass

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
        ctk.CTkButton(ra_row, text="◀", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(-1, 0)).pack(side="left", padx=(2, 0))
        self.ra_offset_var = ctk.DoubleVar(value=0.0)
        self.ra_offset_entry = ctk.CTkEntry(ra_row, textvariable=self.ra_offset_var, width=50, font=ctk.CTkFont(size=11))
        self.ra_offset_entry.pack(side="left", padx=(4, 0))
        ctk.CTkButton(ra_row, text="▶", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(1, 0)).pack(side="left", padx=(2, 4))

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
        ctk.CTkButton(dec_row, text="▲", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(0, 1)).pack(side="left", padx=(2, 0))
        self.dec_offset_var = ctk.DoubleVar(value=0.0)
        self.dec_offset_entry = ctk.CTkEntry(dec_row, textvariable=self.dec_offset_var, width=50, font=ctk.CTkFont(size=11))
        self.dec_offset_entry.pack(side="left", padx=(4, 0))
        ctk.CTkButton(dec_row, text="▼", width=20, height=20, font=ctk.CTkFont(size=10, weight="bold"),
                      command=lambda: self._adjust_offset(0, -1)).pack(side="left", padx=(2, 4))

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
        # Double-click to reset to the full-sky view.
        self.viz_canvas.bind("<Double-Button-1>", self._on_viz_reset_view)
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
        for m in ["SIDEREAL", "SOLAR", "LUNAR"]:
            rb = ctk.CTkRadioButton(mode_frame, text=m, variable=self.mode_var, value=m,
                                    command=self._on_mode_changed)
            rb.pack(anchor="w", padx=16, pady=2)

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
        time_btn = ctk.CTkButton(right, text="Resync Arduino Time to Laptop", height=26,
                                 command=lambda: self.serial.send_time_if_connected() if hasattr(self, 'serial') and self.serial else None)
        time_btn.pack(fill="x", padx=12, pady=(2,6))

        # Position update rate control (how often Arduino sends POS updates)
        rate_frame = ctk.CTkFrame(right)
        rate_frame.pack(fill="x", padx=12, pady=4)

        ctk.CTkLabel(rate_frame, text="Arduino POS Update Rate", font=ctk.CTkFont(size=11)).pack(anchor="w", padx=8, pady=(2,0))

        rate_row = ctk.CTkFrame(rate_frame)
        rate_row.pack(fill="x", padx=8, pady=2)

        self.pos_rate_var = ctk.StringVar(value="20")  # 50 Hz default
        self.pos_rate_entry = ctk.CTkEntry(rate_row, textvariable=self.pos_rate_var, width=70)
        self.pos_rate_entry.pack(side="left")

        ctk.CTkLabel(rate_row, text="ms", width=25).pack(side="left")

        self.set_rate_btn = ctk.CTkButton(rate_row, text="Set", width=50, height=26,
                                          command=self._set_pos_update_rate)
        self.set_rate_btn.pack(side="left", padx=6)

        # Presets - 50Hz default
        self.rate_preset = ctk.CTkOptionMenu(
            rate_frame,
            values=["10 ms (100 Hz)", "20 ms (50 Hz)", "50 ms (20 Hz)", "100 ms (10 Hz)", "200 ms (5 Hz)"],
            command=self._apply_rate_preset,
            width=160
        )
        self.rate_preset.set("20 ms (50 Hz)")  # default
        self.rate_preset.pack(padx=8, pady=(0,4))

        ctk.CTkLabel(rate_frame, text="(changes how often Arduino sends current position + error)", 
                     font=ctk.CTkFont(size=9), text_color="#555577").pack(anchor="w", padx=8)

        self.current_rate_label = ctk.CTkLabel(rate_frame, text="Current: 20 ms (50 Hz)", font=ctk.CTkFont(size=10), text_color="#8888aa")
        self.current_rate_label.pack(anchor="w", padx=8, pady=(0,2))

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
        self.location_label.configure(text=f"Loc: {self.lat_var.get()}N {self.lon_var.get()}E")
        # _update_slew_button removed (GoTo function removed)

        # Ensure viz is drawn with correct initial size (width + height)
        self.after(150, self._redraw_viz)

        # Make sure initial time will be sent on connect (plus periodic)

    def _get_current_lst_deg(self):
        """Approximate Local Sidereal Time in degrees using system time + longitude."""
        _, lon = self._get_lat_lon_from_input()
        utc = datetime.now(timezone.utc)
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
        return margin + (w - 2 * margin) * ((ra - ra_min) / ra_span)

    def _viz_dec_to_y(self, dec, h, margin, dec_min, dec_span):
        return margin + (h - 2 * margin) * (1.0 - (dec - dec_min) / dec_span)

    def _viz_x_to_ra(self, x, w, margin, ra_min, ra_span):
        return ra_min + ra_span * ((x - margin) / (w - 2 * margin))

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
        """Coalesce rapid-fire zoom/pan events into at most one redraw per idle cycle, instead
        of a full redraw (delete("all") + recreate every grid line/label/horizon point) for
        every single wheel tick or mouse-motion event. A trackpad scroll or a drag gesture can
        fire many events per second - without this, each one triggered its own synchronous
        redraw, and since panning around is exactly what you do once zoomed in, that stacked up
        fast enough to feel like ~4fps at high zoom (the redraw work itself wasn't slower at
        high zoom, there were just far more of them queued up back to back)."""
        if self._viz_redraw_pending:
            return
        self._viz_redraw_pending = True
        self.after_idle(self._flush_viz_redraw)

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
        cursor_ra = ra_min + ra_span * frac_x
        cursor_dec = dec_min + dec_span * (1.0 - frac_y)

        steps = event.delta / 120.0
        zoom_factor = VIZ_ZOOM_STEP_BASE ** steps
        new_zoom = max(VIZ_ZOOM_MIN, min(VIZ_ZOOM_MAX, self._viz_zoom * zoom_factor))
        if new_zoom == self._viz_zoom:
            return
        self._viz_zoom = new_zoom

        # Re-center so the point under the cursor stays under the cursor.
        new_ra_span, new_dec_span = self._get_viz_view_span()
        self._viz_center_ra = cursor_ra + new_ra_span * (0.5 - frac_x)
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
        self._viz_center_ra -= dx * ra_span / max(1, (w - 2 * margin))
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

        # Polygon for ground: start at left horizon edge, follow curve, to right horizon edge, then bottom
        poly = [(x0, y0)] + horizon_points + [(x360, y360), (x360, h - margin), (x0, h - margin)]
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

        # Background
        c.create_rectangle(8, 8, w-8, h-8, outline="#222233", width=2, fill="#111118", tags="viz_bg")

        # Horizon shading (below horizon gets darker color) - drawn right after the background
        # so everything else (grid, labels, FOV circle, target cross) renders on top of it,
        # rather than the ground shading covering those. Still visible as a shaded layer against
        # the background, just at the back of the stack instead of the front.
        # tag_raise explicitly pins "horizon"-tagged items directly above "viz_bg", rather than
        # relying only on creation order - reported as still covering the grid even after
        # restarting with the creation-order fix in place, so this makes the stacking explicit
        # and unambiguous instead of depending on it falling out naturally from draw order.
        # (A bare tag_lower("horizon") would send it below viz_bg too, hiding it completely
        # behind the opaque background - has to be anchored relative to viz_bg specifically.)
        self._draw_horizon(c, w, h, margin, ra_min, ra_max, dec_min, dec_max)
        c.tag_raise("horizon", "viz_bg")

        # DEC lines (horizontal), spacing adaptive to zoom
        dec_step = self._pick_grid_step(dec_span)
        d = math.ceil(dec_min / dec_step) * dec_step
        while d <= dec_max + 1e-9:
            y = self._viz_dec_to_y(d, h, margin, dec_min, dec_span)
            col, wd = self._grid_line_style(d, dec_step)
            c.create_line(8, y, w-8, y, fill=col, width=wd)
            d += dec_step

        # RA lines (vertical), spacing adaptive to zoom
        ra_step = self._pick_grid_step(ra_span)
        r = math.ceil(ra_min / ra_step) * ra_step
        while r <= ra_max + 1e-9:
            x = self._viz_ra_to_x(r, w, margin, ra_min, ra_span)
            col, wd = self._grid_line_style(r, ra_step)
            c.create_line(x, 8, x, h-8, fill=col, width=wd)
            r += ra_step

        # Cross hairs (center 0/0), only drawn if within the visible window
        if dec_min <= 0.0 <= dec_max:
            cy0 = self._viz_dec_to_y(0.0, h, margin, dec_min, dec_span)
            c.create_line(8, cy0, w-8, cy0, fill="#555566", width=1)
        if ra_min <= 180.0 <= ra_max:
            cx0 = self._viz_ra_to_x(180.0, w, margin, ra_min, ra_span)
            c.create_line(cx0, 8, cx0, h-8, fill="#555566", width=1)

        # Labels - show the actual visible bounds + zoom level (updates as you zoom/pan)
        c.create_text(18, 18, text=f"{dec_max:+.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="w")
        c.create_text(18, h-16, text=f"{dec_min:+.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="w")
        c.create_text(w-16, cy - 12, text=f"RA {ra_min:.2f}° → {ra_max:.2f}°", fill="#8888aa", font=("Consolas", 10), anchor="e")
        c.create_text(cx, 18, text="DEC", fill="#aaaacc", font=("Consolas", 11))
        zoom_hint = "scroll/pinch to zoom, drag to pan, double-click to reset" if self._viz_zoom <= VIZ_ZOOM_MIN + 1e-6 \
            else f"zoom {self._viz_zoom:.1f}x - drag to pan, double-click to reset"
        c.create_text(cx, h-16, text=zoom_hint, fill="#666688", font=("Consolas", 9))

        # Camera sensor framing: real measured camera FOV (see CAMERA_FOV_W_DEG). Uses a single
        # isotropic px/deg scale (the DEC-axis one) for both width and height so the rectangle
        # keeps its true 3:2 shape - using the RA-axis and DEC-axis scales independently here
        # would stretch it to whatever the canvas's aspect ratio happens to be instead (they
        # only agree when the canvas is exactly 2:1, matching the 360:180 deg RA:DEC range).
        pixels_per_deg = (h - 2 * margin) / dec_span
        cam_half_w, cam_half_h = self._camera_fov_half_size(pixels_per_deg)
        self.viz_camera_rect = c.create_rectangle(cx - cam_half_w, cy - cam_half_h,
                                                   cx + cam_half_w, cy + cam_half_h,
                                                   outline="#5599ff", width=1.5, dash=(4, 2))

        # Telescope full-FOV circle: circumscribes the camera rectangle exactly (radius =
        # rectangle's on-screen diagonal), so its corners always touch the circle regardless of
        # the exact px/deg value used above. Outline color mirrors the stability badge (see
        # _set_stability) instead of a fixed color, so tracking health is visible at a glance
        # right on the reticle, not just in the small badge text.
        fov_radius = max(5, math.hypot(cam_half_w, cam_half_h))
        self.viz_dot = c.create_oval(cx - fov_radius, cy - fov_radius,
                                     cx + fov_radius, cy + fov_radius,
                                     fill="", outline=self._stability_color, width=2)

        # Target cross - positioned in _update_visualization
        self.viz_target_h = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)
        self.viz_target_v = c.create_line(0, 0, 0, 0, fill="#ffaa00", width=1.5)

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
            c.coords(self.viz_camera_rect, x - cam_half_w, y - cam_half_h,
                     x + cam_half_w, y + cam_half_h)

        if hasattr(self, 'viz_dot'):
            fov_radius = max(5, math.hypot(cam_half_w, cam_half_h))
            c.coords(self.viz_dot, x - fov_radius, y - fov_radius,
                     x + fov_radius, y + fov_radius)

        # Target cross position
        tx = self._viz_ra_to_x(self.target_ra % 360.0, w, margin, ra_min, ra_span)
        ty = self._viz_dec_to_y(self.target_dec, h, margin, dec_min, dec_span)

        cross_len = 11
        if hasattr(self, 'viz_target_h') and hasattr(self, 'viz_target_v'):
            c.coords(self.viz_target_h, tx - cross_len, ty, tx + cross_len, ty)
            c.coords(self.viz_target_v, tx, ty - cross_len, tx, ty + cross_len)

    def _on_canvas_resize(self, event=None):
        """Redraw the visualization when the canvas size changes (both width and height)."""
        # Use after_idle to avoid too many redraws during rapid resizing
        self.after_idle(self._redraw_viz)

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
        self.conn_status.configure(text="● Disconnected", text_color="gray")

    def _on_mode_changed(self):
        mode = self.mode_var.get()
        self.current_mode = mode

        # Force tracking to stop when switching modes (per requirement)
        self._force_stop_tracking()

        # The Arduino doesn't recompute targetRA/DEC for the new mode until tracking actually
        # (re)starts (see CMD,MODE handling - it only recomputes if trackingActive, which is
        # false right after the stop above), so it keeps broadcasting the OLD mode's target in
        # POS updates until then, and self.target_ra/dec mirrors that stale value. If Start
        # Tracking were clicked right now, the "is the target close?" distance check would
        # compare against that stale target and could wrongly pick the SKIP_DEC_RESET fast
        # path for a mode we have no real target for yet. Force the next Start Tracking to do
        # a full alignment regardless of that (unreliable, right after a mode change) distance.
        self._force_full_align_next_start = True

        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_mode(mode)
        self._update_mode_label()
        self._log(f"Mode changed to {mode} (tracking stopped)")

    def _toggle_tracking(self):
        if not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return

        if not self.tracking:
            # Start
            # For SIDEREAL: send the coordinates from the two input boxes as the target.
            # This uses SET_TARGET + START_TRACKING so Arduino alignment uses *exactly* the
            # same logic as solar/lunar (compute target, use cal to derive physical mount
            # angles for the slew in the alignment sequence, then track at sidereal rate).
            if self.mode_var.get() == "SIDEREAL":
                try:
                    # Robust parse: extract first two numbers even if user pastes "26.01, -15.9" or junk
                    ra_str = self.goto_ra.get().strip()
                    dec_str = self.goto_dec.get().strip()
                    ra = float(re.findall(r"[-+]?\d*\.?\d+", ra_str)[0]) if re.findall(r"[-+]?\d*\.?\d+", ra_str) else float(ra_str)
                    dec = float(re.findall(r"[-+]?\d*\.?\d+", dec_str)[0]) if re.findall(r"[-+]?\d*\.?\d+", dec_str) else float(dec_str)
                    self.target_ra = ra
                    self.target_dec = dec
                    self.serial.send_command(f"CMD,SET_TARGET,RA:{ra:.6f},DEC:{dec:.6f}")
                except Exception:
                    self._log("Invalid RA/DEC in target boxes for sidereal; using previous sync values.")
            if self._force_full_align_next_start:
                # See _on_mode_changed: right after a mode switch the GUI's cached target is
                # stale (Arduino hasn't recomputed it for the new mode yet, since that only
                # happens once tracking is active), so the distance-based fast-path decision
                # below can't be trusted this one time - always do a full alignment instead.
                self._force_full_align_next_start = False
                self._request_tracking_action("START", self.serial.send_start_tracking,
                                               "normal alignment, forced after mode change", "STARTING…")
            else:
                # Check if target is within ±5° of current position to skip DEC reset
                distance = self._calculate_angular_distance(
                    self.current_ra, self.current_dec,
                    self.target_ra, self.target_dec
                )
                if distance <= 5.0:
                    # Target is close, skip DEC reset for faster alignment
                    self._request_tracking_action("START", self.serial.send_start_tracking_skip_dec_reset,
                                                   f"DEC reset skip, target within {distance:.2f}°", "STARTING…")
                else:
                    # Target is far, use normal alignment sequence
                    self._request_tracking_action("START", self.serial.send_start_tracking,
                                                   f"normal alignment, target {distance:.2f}° away", "STARTING…")
        else:
            # Stop
            self._request_tracking_action("STOP", self.serial.send_stop, "Stop Tracking button", "STOPPING…")

        self._update_tracking_button()

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

    def _update_tracking_button(self):
        if self.tracking:
            self.tracking_btn.configure(text="■ STOP TRACKING", fg_color="#8B0000")
        else:
            self.tracking_btn.configure(text="▶ Start Tracking", fg_color="#006400")

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

        if star_pos is not None:
            # Use predefined bright star coordinates
            ra, dec = star_pos
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

        self.target_ra = ra
        self.target_dec = dec

        if self.serial.ser and self.serial.ser.is_open:
            self.serial.send_goto(ra, dec)
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
            self.target_ra = ra
            self.target_dec = dec
            self.serial.send_goto(ra, dec)
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

    def _set_pos_update_rate(self):
        """Send new position broadcast rate to Arduino."""
        if not hasattr(self, 'serial') or not self.serial or not self.serial.ser or not self.serial.ser.is_open:
            self._log("Not connected.")
            return
        try:
            ms = int(self.pos_rate_var.get())
            if ms < 10:
                ms = 10
            elif ms > 5000:
                ms = 5000
            self.serial.send_pos_update_rate(ms)
            self._log(f"Sent new POS update rate: {ms} ms")
            # Optimistic update of display (will be confirmed by Arduino)
            hz = int(1000 / ms) if ms > 0 else 0
            self.current_rate_label.configure(text=f"Requested: {ms} ms ({hz} Hz)")
        except ValueError:
            self._log("Invalid number for update rate.")

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
                self.location_label.configure(text=f"Loc: {lat:.6f}, {lon:.6f} (sent)")
            else:
                self.location_label.configure(text=f"Loc: {lat:.3f}N {lon:.3f}E (sent)")
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
                self.location_label.configure(text=f"Loc: {lat:.6f}, {lon:.6f} (live)")
            else:
                self.location_label.configure(text=f"Loc: {lat:.3f}N {lon:.3f}E (live)")
                
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

    def _apply_rate_preset(self, choice: str):
        """Parse preset like '200 ms (5Hz)' and apply it."""
        try:
            # Extract the number before " ms"
            ms_str = choice.split()[0]
            ms = int(ms_str)
            self.pos_rate_var.set(str(ms))
            self._set_pos_update_rate()
        except Exception:
            pass

    def _apply_initial_rate(self):
        """Apply the rate currently shown in the GUI entry (used on connect)."""
        if hasattr(self, 'serial') and self.serial and self.serial.ser and self.serial.ser.is_open:
            try:
                ms = int(self.pos_rate_var.get())
                self.serial.send_pos_update_rate(ms)
            except:
                pass

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

    def _cycle_viz_view_mode(self):
        order = ["FREE", "TELESCOPE", "TARGET"]
        labels = {"FREE": "View: Free", "TELESCOPE": "View: Follow Telescope", "TARGET": "View: Follow Target"}
        self._viz_view_mode = order[(order.index(self._viz_view_mode) + 1) % len(order)]
        self.viz_mode_btn.configure(text=labels[self._viz_view_mode])
        self._redraw_viz()

    def _on_viz_mouse_move(self, event):
        """Show the RA/DEC under the cursor below the canvas (not drawn inside it)."""
        c = self.viz_canvas
        w = max(200, c.winfo_width())
        h = max(150, c.winfo_height())
        margin = 12
        ra_min, ra_max, dec_min, dec_max = self._get_viz_view_bounds()
        ra_span, dec_span = ra_max - ra_min, dec_max - dec_min
        ra = self._viz_x_to_ra(event.x, w, margin, ra_min, ra_span) % 360.0
        dec = self._viz_y_to_dec(event.y, h, margin, dec_min, dec_span)
        dec = max(-90.0, min(90.0, dec))
        self.viz_cursor_label.configure(text=f"Cursor: RA {ra:.4f}°  DEC {dec:+.4f}°")

    def _on_viz_mouse_leave(self, event=None):
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
        self._update_tracking_button()
        self._update_status_display(status_text, "#ffaa00")
        ok = send_fn()
        self._log(f"Sent {action} ({reason})" if ok else f"{action} write failed ({reason}) - will retry")
        self._pending_action_after_id = self.after(800, lambda: self._verify_pending_action(action, send_fn))

    def _verify_pending_action(self, action: str, send_fn):
        self._pending_action_after_id = None
        # A different action superseded this one, or it already got confirmed - nothing to do.
        if self._pending_action != action or self._pending_action_confirmed:
            return
        if not (self.serial and self.serial.ser and self.serial.ser.is_open):
            return
        if self._pending_action_retry_count >= 3:
            self._log(f"WARNING: Arduino never confirmed {action} after retries - check the connection/mount.")
            self._update_status_display(f"{action} NOT CONFIRMED - CHECK CONNECTION", "#ff4444")
            return
        self._pending_action_retry_count += 1
        ok = send_fn()
        self._log(f"{action} not yet confirmed, resending (attempt {self._pending_action_retry_count + 1})" if ok
                   else f"{action} resend also failed to write")
        self._pending_action_after_id = self.after(800, lambda: self._verify_pending_action(action, send_fn))

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
            # WAITING lines stream at 10 Hz for up to 3s per axis during alignment.
            # Logging every one of them floods the Textbox (each _log() call does an
            # insert + index + see("end")) and backs up the message queue badly enough
            # that later log entries get timestamped minutes late. The live error label
            # and top status bar already show this info in real time, so skip the text log.
            if not line.startswith("STATUS:WAITING"):
                self._log(line)
            self.last_status = line

            # Only update the TOP status bar for important messages.
            # Ignore routine ones like TIME_SET, LOCATION_SET, READY, UPDATE_RATE etc.
            # so they don't override tracking/alignment status on the prominent bar.
            # (All still logged in the status text box.)
            important = ["TRACKING", "ALIGNING", "RESETTING", "WAITING", "STOPPED", "SYNCED", "ERROR", "SLEW"]
            if any(kw in line for kw in important):
                # Update big clear status at top
                status_text = line.replace("STATUS:", "STATUS: ")
                color = "#00ccff"
                if "TRACKING_STARTED" in line or "TRACKING" in line:
                    color = "#00ff88"
                    self.tracking = True
                    self._update_tracking_button()
                    # The RA-only TRACKING_STARTED that fires partway through alignment (RA
                    # settled, DEC not yet aligned) deliberately does NOT count as the full
                    # sequence being done - only the final (DEC) one does, so the stability
                    # badge doesn't start judging error trend before DEC has even moved.
                    if "AXIS:RA" not in line:
                        self._align_phase = "TRACKING"
                elif "ALIGNING" in line or "RESETTING" in line or "WAITING" in line:
                    color = "#ffaa00"
                    self._align_phase = "ALIGNING"
                elif "STOPPED" in line or "ERROR" in line:
                    color = "#ff6666"
                    self.tracking = False
                    self._update_tracking_button()
                    self._align_phase = "IDLE"

                self._update_status_display(status_text, color)

            # Handle rate confirmation from Arduino
            if "UPDATE_RATE" in line or "UPDATE_RATE,MS:" in line:
                m = re.search(r"MS:(\d+)", line)
                if m:
                    ms_str = m.group(1)
                    try:
                        ms = int(ms_str)
                        hz = int(1000 / ms) if ms > 0 else 0
                    except (ValueError, TypeError):
                        ms = 0
                        hz = 0
                    self.current_rate_label.configure(text=f"Current: {ms_str} ms ({hz} Hz)")
                    self.pos_rate_var.set(ms_str)
                    self._log(f"Arduino confirmed POS rate: {ms_str} ms")

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
            # POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking
            # mode is a single char: S=SIDEREAL, O=SOLAR, L=LUNAR. tracking is 0/1.
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
            mode_char = fields[7] if len(fields) > 7 else None
            if mode_char in ("S", "O", "L"):
                self.current_mode = {"S": "SIDEREAL", "O": "SOLAR", "L": "LUNAR"}[mode_char]
                self.mode_var.set(self.current_mode)

            if len(fields) > 8:
                pos_track = fields[8].strip() == "1"
                if pos_track:
                    # Only trust "tracking on" from POS. Never flip back to False from POS
                    # (POS during alignment often reports 0 even after we started)
                    if not self.tracking:
                        self.tracking = True
                        self._update_tracking_button()

            self._update_mode_label()
            # Recenter the follow-mode view on every position update, not just on the coarser
            # periodic timer - this is cheap (just updates two floats + clamps, no Tk rendering)
            # since _update_visualization() right after already runs on every POS update anyway
            # and only repositions existing canvas items via coords(), no full grid rebuild. The
            # periodic timer (_start_viz_follow_timer) still handles the expensive part (redrawing
            # the grid/labels/horizon) at its own coarser pace - this only makes the *centering*
            # track precisely instead of lagging up to one timer tick behind.
            self._apply_viz_follow_mode()
            self._update_visualization()
            self._update_labels()  # updates sky + mount positions + error together
            self._update_live_error()  # error updated together with position (anytime via POS)

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
                        self.location_label.configure(text=f"Loc: {lat_str}, {lon_str} (on Arduino)")
                    else:
                        self.location_label.configure(text=f"Loc: {lat_str}N {lon_str}E (on Arduino)")
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
        self.ra_label.configure(text=f"{self.current_ra:010.6f}°")
        self.dec_label.configure(text=f"{self.current_dec:+010.6f}°")

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

    def _set_stability(self, text: str, color: str):
        """Update the stability badge AND recolor the telescope FOV circle in the sky viz to
        match, so the same STABLE/SETTLING/DRIFTING/ALIGNING state is visible at a glance in
        both places instead of the circle staying a fixed color regardless of tracking health."""
        self.stability_label.configure(text=text, fg_color=color, text_color="#ffffff")
        self._stability_color = color
        if hasattr(self, 'viz_dot'):
            try:
                self.viz_canvas.itemconfigure(self.viz_dot, outline=color)
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
        self.destroy()


if __name__ == "__main__":
    app = EQMountApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

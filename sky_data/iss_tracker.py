"""
iss_tracker.py - real-time ISS sky position (RA/DEC as seen from a given location).

Uses CelesTrak's public GP/TLE endpoint (no API key) for current orbital elements, and the
'skyfield' package (SGP4 propagation) to convert that into topocentric RA/DEC for an observer.
skyfield is an optional dependency - only imported when actually used, so the rest of the app
works fine without it installed; get_current_radec() raises ImportError with a clear message
if it's missing.

open-notify.org's simpler "ISS now" API only returns the sub-satellite ground point (lat/lon),
which is NOT the same as where the ISS appears in the sky from a specific observer location -
that requires real orbital propagation (SGP4) from a TLE, which is what this module does instead.
"""

import os
import time
import urllib.request

CELESTRAK_TLE_URL = "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=tle"
TLE_CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "iss_tle.txt")

# TLEs go stale after roughly 1-2 weeks (satellite drag/maneuvers accumulate error) - refreshing
# every 12h keeps propagation accurate without hammering CelesTrak on every launch.
TLE_MAX_AGE_HOURS = 12.0


def ensure_tle_current(path=TLE_CACHE_PATH, max_age_hours=TLE_MAX_AGE_HOURS):
    """Fetch a fresh ISS TLE if the cached one is missing or stale. Returns True if it fetched."""
    if os.path.exists(path):
        age_h = (time.time() - os.path.getmtime(path)) / 3600.0
        if age_h < max_age_hours:
            return False
    with urllib.request.urlopen(CELESTRAK_TLE_URL, timeout=15) as resp:
        text = resp.read().decode("utf-8", errors="replace")
    lines = [l.strip() for l in text.strip().splitlines() if l.strip()]
    if len(lines) < 3 or not lines[1].startswith("1 ") or not lines[2].startswith("2 "):
        raise ValueError(f"Unexpected TLE response from CelesTrak: {text[:200]!r}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines[:3]) + "\n")
    return True


def _load_satellite():
    from skyfield.api import EarthSatellite, load  # optional dependency, imported lazily
    with open(TLE_CACHE_PATH, "r", encoding="utf-8") as f:
        lines = [l.strip() for l in f.read().splitlines() if l.strip()]
    ts = load.timescale()
    return EarthSatellite(lines[1], lines[2], lines[0], ts), ts


def get_current_radec(lat_deg, lon_deg, elevation_m=0.0):
    """Topocentric ISS position right now, as seen from (lat_deg, lon_deg).
    Returns (ra_deg, dec_deg, alt_deg, az_deg, above_horizon).
    Raises ImportError if skyfield isn't installed, or OSError/ValueError if no TLE is cached
    yet and none could be fetched (caller should call ensure_tle_current() first)."""
    from skyfield.api import wgs84  # optional dependency, imported lazily
    satellite, ts = _load_satellite()
    t = ts.now()
    observer = wgs84.latlon(lat_deg, lon_deg, elevation_m)
    topocentric = (satellite - observer).at(t)
    ra, dec, _ = topocentric.radec()
    alt, az, _ = topocentric.altaz()
    ra_deg = ra.hours * 15.0
    return ra_deg, dec.degrees, alt.degrees, az.degrees, alt.degrees > 0.0

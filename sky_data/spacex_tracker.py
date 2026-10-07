"""
spacex_tracker.py - live SpaceX Dragon / Starship sky position (RA/DEC as seen from a given location).

Data: the two public JSON snapshots behind https://www.spacex.com/vehicle-tracker (no key, no
account, unofficial, field names can change with the mission set). Each snapshot holds
ONE sample per vehicle, and the Dragon file is only rewritten about every 30s (measured
2026-10-08), so a raw sample is up to ~40s old by the time it is read - roughly 300km behind a
vehicle in low Earth orbit, several degrees on the sky.

Estimation ("previous step to estimate the current step"): every vehicle keeps its last two
distinct samples. The velocity at the latest sample is solved from the two positions by shooting
(guess a velocity, propagate back to the previous sample time under gravity, correct the guess by
the miss) and the state is then propagated forward to the requested time. Propagation is RK4 in
the rotating ECEF frame (point-mass gravity + J2 + Coriolis + centrifugal), so the result stays in
the same frame the feed publishes. A plain straight-line extrapolation would be off by ~15km after
60s from gravity alone. This is a coasting model: during a Starship engine burn or reentry drag
the estimate lags the real vehicle until the next sample replaces it. Dragon sample times are first
corrected against the ISS TLE (anchor_dragon_sample_time) - the file's shared gps_time is seconds
off per row, and on 2026-10-08 real data that turned a 23km one-sample-ahead miss into 0.5m.

When a new sample replaces the state, the estimate would jump by whatever the old prediction
missed by; that jump is blended out over HANDOFF_BLEND_S so the mount sees a smooth rate instead
of a step (the firmware treats a step above MAX_PLAUSIBLE_TARGET_RATE_DEG_S as a new target).

Vehicle keys (crew13, ship41, ...) come and go with the mission set, so nothing here hardcodes
them: every Dragon row and every non-metadata Starship row is its own vehicle, any number of each.
"""

import gzip
import json
import math
import threading
import time
import urllib.error
import urllib.request
from collections import namedtuple

DRAGON_FEED_URL = "https://sxcontent9668.azureedge.us/cms-assets/dragon_tracker_public.json"
STARSHIP_FEED_URL = "https://sxcontent9668.azureedge.us/cms-assets/starship_tracker_public.json"
FEED_TIMEOUT_S = 15

# GPS time -> Unix time. GPS seconds count from 1980-01-06 without leap seconds; GPS-UTC has been
# 18s since 2017-01-01 and stays that way until the next leap second is announced.
GPS_EPOCH_UNIX_S = 315964800
GPS_UTC_LEAP_SECONDS = 18

# A sample older than this (against the wall clock) is a post-flight leftover or a dead feed - the
# Starship file kept a 7-day-old splashdown sample on 2026-10-07. Not shown, not tracked.
SAMPLE_MAX_AGE_S = 300.0
# Two samples further apart than this do not define a usable velocity (missed polls, feed gap).
MAX_SAMPLE_GAP_S = 180.0
# Two samples closer than this are the same telemetry (e.g. one row re-anchored after a TLE
# refresh) - differencing them would divide a tiny position change by a tiny gap.
MIN_SAMPLE_GAP_S = 1.0
# Never propagate further than this from the latest sample (also hides vehicles in Time Travel).
MAX_EXTRAPOLATION_S = 300.0
# An implied speed above this between two samples means a desync or a renamed key, not a vehicle.
MAX_PLAUSIBLE_SPEED_M_S = 12000.0
# Time anchoring (anchor_dragon_sample_time): a correction larger than this, or a TLE-vs-row ISS
# mismatch left over larger than this, means the anchor is not trustworthy - keep gps_time.
MAX_TIME_ANCHOR_CORRECTION_S = 15.0
MAX_TIME_ANCHOR_RESIDUAL_M = 5000.0
# Dragon within this distance of the ISS prediction is docked (measured: ~7m on crew13).
DOCKED_DISTANCE_M = 1000.0
# A position closer to the geocenter than this is a zeroed / garbage row.
MIN_PLAUSIBLE_RADIUS_M = 6000000.0

HANDOFF_BLEND_S = 5.0
INTEGRATION_STEP_S = 5.0
VELOCITY_SOLVE_ITERATIONS = 8
VELOCITY_SOLVE_TOLERANCE_M = 0.5

EARTH_GM_M3_S2 = 3.986004418e14
EARTH_EQUATORIAL_RADIUS_M = 6378137.0
EARTH_J2 = 1.08262668e-3
EARTH_ROTATION_RAD_S = 7.2921150e-5

KIND_DRAGON = "Dragon"
KIND_STARSHIP = "Starship"

# iss_r_ecef: the ISS prediction a Dragon row carries for the same instant (None for Starship).
# time_anchored: unix_time was corrected by anchor_dragon_sample_time - two samples on different
# time bases (one anchored, one not) must never be differenced into a velocity.
VehicleSample = namedtuple("VehicleSample",
                           "key kind label unix_time mission_time r_ecef docked_to_iss iss_r_ecef time_anchored",
                           defaults=(False,))


def gps_to_unix(gps_seconds):
    return gps_seconds + GPS_EPOCH_UNIX_S - GPS_UTC_LEAP_SECONDS


def vehicle_label(kind, key):
    """Display name and target identity, e.g. "Dragon crew13", "Starship ship41"."""
    return f"{kind} {key}"


# ---------------- feed parsing ----------------
def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def _finite_vector3(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    components = [_finite_number(component) for component in value]
    return None if any(component is None for component in components) else tuple(components)


def _plausible_position(r_ecef):
    return r_ecef is not None and math.sqrt(sum(c * c for c in r_ecef)) >= MIN_PLAUSIBLE_RADIUS_M


def parse_dragon_feed(document):
    """One VehicleSample per live Dragon row. Idle slots (altitude, speed and mission time all 0)
    and rows with a missing/non-finite/zero position are skipped. Position is the feed's own
    predict_dgn_r_ecef_v3 (what the vehicle-tracker page draws - there is no separate measured
    lat/lon in this file)."""
    samples = []
    if not isinstance(document, dict):
        return samples
    for key, row in document.items():
        if not isinstance(row, dict):
            continue
        altitude = _finite_number(row.get("glass.dgn_alt_geod_f64")) or 0.0
        speed = _finite_number(row.get("glass.dgn_speed_f64")) or 0.0
        mission_time = _finite_number(row.get("glass.dragon.mission_time_f64")) or 0.0
        if all(abs(scalar) <= 1.0 for scalar in (altitude, speed, mission_time)):
            continue  # idle slot left in the file
        gps_time = _finite_number(row.get("glass.dragon.gps_time_f64"))
        r_ecef = _finite_vector3(row.get("glass.predict_dgn_r_ecef_v3"))
        if gps_time is None or not _plausible_position(r_ecef):
            continue
        iss_r_ecef = _finite_vector3(row.get("glass.predict_iss_r_ecef_v3"))
        docked = _plausible_position(iss_r_ecef) and math.dist(r_ecef, iss_r_ecef) <= DOCKED_DISTANCE_M
        samples.append(VehicleSample(key, KIND_DRAGON, vehicle_label(KIND_DRAGON, key),
                                     gps_to_unix(gps_time), mission_time, r_ecef, docked,
                                     iss_r_ecef if _plausible_position(iss_r_ecef) else None))
    return samples


def parse_starship_feed(document):
    """One VehicleSample per ship key (every key except "metadata" - any number of ships). Uses
    the measured `current` block only; `trajectory` is a prediction that keeps propagating after
    splashdown and is ignored."""
    samples = []
    if not isinstance(document, dict):
        return samples
    for key, row in document.items():
        if key == "metadata" or not isinstance(row, dict):
            continue
        current = row.get("current")
        if not isinstance(current, dict):
            continue
        gps_time = _finite_number(current.get("gps_time"))
        r_ecef = _finite_vector3(current.get("r_ecef"))
        if gps_time is None or not _plausible_position(r_ecef):
            continue
        mission_time = _finite_number(current.get("mission_time")) or 0.0
        samples.append(VehicleSample(key, KIND_STARSHIP, vehicle_label(KIND_STARSHIP, key),
                                     gps_to_unix(gps_time), mission_time, r_ecef, False, None))
    return samples


def anchor_dragon_sample_time(sample, iss_state_fn):
    """Dragon rows share ONE gps_time (the file generation time), but each row's predicted
    position belongs to that vehicle's own telemetry instant, which sits up to a few seconds off
    and jitters between files (measured 2026-10-08: crew12 steady at +0.3s, crew13 jumping from
    +0.2s to -2.9s = 23km at orbital speed). Each row also carries the ISS prediction for that same
    instant, so the true time is where the ISS's own TLE track passes that point: one along-track
    Newton step (twice) on time using the TLE velocity. iss_state_fn(unix_time) -> (r_m, v_m_s)
    in ECEF. Returns the sample with unix_time corrected, or unchanged when the row has no ISS
    prediction, the TLE is unavailable, or the match is poor (stale TLE, garbage row)."""
    if sample.iss_r_ecef is None:
        return sample
    corrected_time = sample.unix_time
    try:
        for _iteration in range(2):
            iss_r, iss_v = iss_state_fn(corrected_time)
            speed_sq = sum(component * component for component in iss_v)
            along_track_m = sum((sample.iss_r_ecef[axis] - iss_r[axis]) * iss_v[axis] for axis in range(3))
            corrected_time += along_track_m / speed_sq
        iss_r, _iss_v = iss_state_fn(corrected_time)
    except Exception:
        return sample
    if (abs(corrected_time - sample.unix_time) > MAX_TIME_ANCHOR_CORRECTION_S
            or math.dist(iss_r, sample.iss_r_ecef) > MAX_TIME_ANCHOR_RESIDUAL_M):
        return sample
    return sample._replace(unix_time=corrected_time, time_anchored=True)


class FeedClient:
    """GETs one feed URL with gzip and If-None-Match. fetch() returns the parsed JSON document,
    or None when the CDN answered 304 (body unchanged since the last fetch). Raises on HTTP
    errors / bad JSON - the caller keeps its last good samples (never invents a position)."""

    def __init__(self, url):
        self.url = url
        self._etag = None

    def fetch(self):
        headers = {"Accept-Encoding": "gzip"}
        if self._etag:
            headers["If-None-Match"] = self._etag
        request = urllib.request.Request(self.url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=FEED_TIMEOUT_S) as response:
                body = response.read()
                encoding = (response.headers.get("Content-Encoding") or "").lower()
                etag = response.headers.get("ETag")
        except urllib.error.HTTPError as http_error:
            if http_error.code == 304:
                return None
            raise
        if encoding == "gzip":
            body = gzip.decompress(body)
        document = json.loads(body.decode("utf-8"))
        self._etag = etag  # only remembered once the body actually parsed
        return document


# ---------------- orbital propagation (rotating ECEF frame) ----------------
def _acceleration_ecef(position, velocity):
    """Point-mass gravity + J2, plus the Coriolis and centrifugal terms of the Earth-fixed frame."""
    position_x, position_y, position_z = position
    velocity_x, velocity_y, _velocity_z = velocity
    radius_sq = position_x * position_x + position_y * position_y + position_z * position_z
    radius = math.sqrt(radius_sq)
    gravity_factor = -EARTH_GM_M3_S2 / (radius_sq * radius)
    j2_term = 1.5 * EARTH_J2 * (EARTH_EQUATORIAL_RADIUS_M * EARTH_EQUATORIAL_RADIUS_M) / radius_sq
    polar_ratio_sq = position_z * position_z / radius_sq
    equatorial_scale = gravity_factor * (1.0 + j2_term * (1.0 - 5.0 * polar_ratio_sq))
    polar_scale = gravity_factor * (1.0 + j2_term * (3.0 - 5.0 * polar_ratio_sq))
    omega = EARTH_ROTATION_RAD_S
    return (equatorial_scale * position_x + 2.0 * omega * velocity_y + omega * omega * position_x,
            equatorial_scale * position_y - 2.0 * omega * velocity_x + omega * omega * position_y,
            polar_scale * position_z)


def _add_scaled(base, delta, scale):
    return (base[0] + delta[0] * scale, base[1] + delta[1] * scale, base[2] + delta[2] * scale)


def propagate_ecef(position, velocity, duration_s):
    """RK4 from (position, velocity) over duration_s (negative = backward). Returns both."""
    step_count = max(1, math.ceil(abs(duration_s) / INTEGRATION_STEP_S))
    step = duration_s / step_count
    for _step_index in range(step_count):
        k1_r, k1_v = velocity, _acceleration_ecef(position, velocity)
        mid1_r, mid1_v = _add_scaled(position, k1_r, step / 2), _add_scaled(velocity, k1_v, step / 2)
        k2_r, k2_v = mid1_v, _acceleration_ecef(mid1_r, mid1_v)
        mid2_r, mid2_v = _add_scaled(position, k2_r, step / 2), _add_scaled(velocity, k2_v, step / 2)
        k3_r, k3_v = mid2_v, _acceleration_ecef(mid2_r, mid2_v)
        end_r, end_v = _add_scaled(position, k3_r, step), _add_scaled(velocity, k3_v, step)
        k4_r, k4_v = end_v, _acceleration_ecef(end_r, end_v)
        position = tuple(position[axis] + step / 6 * (k1_r[axis] + 2 * k2_r[axis] + 2 * k3_r[axis] + k4_r[axis])
                         for axis in range(3))
        velocity = tuple(velocity[axis] + step / 6 * (k1_v[axis] + 2 * k2_v[axis] + 2 * k3_v[axis] + k4_v[axis])
                         for axis in range(3))
    return position, velocity


def solve_velocity(previous_r, latest_r, gap_s):
    """Velocity at latest_r such that coasting back gap_s seconds lands on previous_r. Starts
    from the chord velocity with a half-step gravity correction, then shoots: each iteration
    propagates back and moves the guess by miss / gap_s. Converges to well under a metre in 2-3
    iterations for any gap up to MAX_SAMPLE_GAP_S."""
    chord = tuple((latest_r[axis] - previous_r[axis]) / gap_s for axis in range(3))
    velocity = _add_scaled(chord, _acceleration_ecef(latest_r, chord), gap_s / 2)
    for _iteration in range(VELOCITY_SOLVE_ITERATIONS):
        back_r, _back_v = propagate_ecef(latest_r, velocity, -gap_s)
        miss = tuple(back_r[axis] - previous_r[axis] for axis in range(3))
        velocity = _add_scaled(velocity, miss, 1.0 / gap_s)
        if math.sqrt(sum(component * component for component in miss)) < VELOCITY_SOLVE_TOLERANCE_M:
            break
    return velocity


# ---------------- per-vehicle state ----------------
class _VehicleTrack:
    def __init__(self, sample):
        self.previous_sample = None
        self.latest_sample = sample
        self.velocity = None            # at latest_sample; None until two usable samples exist
        self.handoff_offset = None      # old estimate - new estimate at handoff_unix_time
        self.handoff_unix_time = None

    def estimate(self, at_unix_time):
        """ECEF position at at_unix_time, or None (no velocity yet / beyond the horizon)."""
        if self.velocity is None:
            return None
        offset_s = at_unix_time - self.latest_sample.unix_time
        if abs(offset_s) > MAX_EXTRAPOLATION_S:
            return None
        position, _velocity = propagate_ecef(self.latest_sample.r_ecef, self.velocity, offset_s)
        if self.handoff_offset is not None:
            weight = 1.0 - (at_unix_time - self.handoff_unix_time) / HANDOFF_BLEND_S
            weight = min(1.0, max(0.0, weight))
            if weight > 0.0:
                position = _add_scaled(position, self.handoff_offset, weight)
        return position

    def ingest(self, sample, now_unix_time):
        """Returns a short reason string when the history had to be restarted, else None."""
        latest = self.latest_sample
        gap_s = sample.unix_time - latest.unix_time
        if abs(gap_s) < MIN_SAMPLE_GAP_S:
            return None  # same file generation (or the CDN served the same body again)
        restart_reason = None
        if sample.time_anchored != latest.time_anchored:
            restart_reason = "sample time base changed (ISS TLE anchoring on/off)"
        elif gap_s < 0:
            restart_reason = "sample time went backward"
        elif sample.mission_time < latest.mission_time:
            restart_reason = "mission clock reset"
        elif gap_s > MAX_SAMPLE_GAP_S:
            restart_reason = f"{gap_s:.0f}s since the previous sample"
        elif math.dist(sample.r_ecef, latest.r_ecef) / gap_s > MAX_PLAUSIBLE_SPEED_M_S:
            restart_reason = "implausible jump between samples"

        old_estimate = self.estimate(now_unix_time)
        self.previous_sample, self.latest_sample = latest, sample
        if restart_reason:
            self.previous_sample, self.velocity = None, None
            self.handoff_offset = None
            return restart_reason
        self.velocity = solve_velocity(latest.r_ecef, sample.r_ecef, gap_s)
        self.handoff_offset, self.handoff_unix_time = None, None
        new_estimate = self.estimate(now_unix_time)
        if old_estimate is not None and new_estimate is not None:
            self.handoff_offset = tuple(old_estimate[axis] - new_estimate[axis] for axis in range(3))
            self.handoff_unix_time = now_unix_time
        return None


class SpacexFleet:
    """Thread-safe store of every vehicle seen in the latest snapshot of each feed. Written by
    the fetch thread (ingest_feed), read by the position thread (available/estimate)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._tracks = {}  # label -> _VehicleTrack

    def ingest_feed(self, kind, samples, now_unix_time=None):
        """Replaces every vehicle of `kind` with this snapshot's set (a key missing from the new
        snapshot is gone). Returns [(label, restart_reason), ...] for histories that restarted."""
        now_unix_time = time.time() if now_unix_time is None else now_unix_time
        restarts = []
        with self._lock:
            present = set()
            for sample in samples:
                present.add(sample.label)
                track = self._tracks.get(sample.label)
                if track is None:
                    self._tracks[sample.label] = _VehicleTrack(sample)
                    continue
                reason = track.ingest(sample, now_unix_time)
                if reason:
                    restarts.append((sample.label, reason))
            for label in [label for label, track in self._tracks.items()
                          if track.latest_sample.kind == kind and label not in present]:
                del self._tracks[label]
        return restarts

    def available(self, now_unix_time=None):
        """[(label, kind, docked_to_iss, has_estimate)] for vehicles with a fresh sample,
        sorted Dragon first then by name."""
        now_unix_time = time.time() if now_unix_time is None else now_unix_time
        with self._lock:
            rows = [(label, track.latest_sample.kind, track.latest_sample.docked_to_iss,
                     track.velocity is not None)
                    for label, track in self._tracks.items()
                    if now_unix_time - track.latest_sample.unix_time <= SAMPLE_MAX_AGE_S]
        return sorted(rows, key=lambda row: (row[1] != KIND_DRAGON, row[0]))

    def estimate(self, label, at_unix_time, now_unix_time=None):
        """(r_ecef, sample_age_s) for a fresh vehicle at at_unix_time, else None."""
        now_unix_time = time.time() if now_unix_time is None else now_unix_time
        with self._lock:
            track = self._tracks.get(label)
            if track is None:
                return None
            sample_age_s = now_unix_time - track.latest_sample.unix_time
            if sample_age_s > SAMPLE_MAX_AGE_S:
                return None
            position = track.estimate(at_unix_time)
        return None if position is None else (position, sample_age_s)


# ---------------- sky position ----------------
_timescale = None


def get_radec_from_ecef(r_ecef_m, lat_deg, lon_deg, at_time, elevation_m=0.0):
    """Topocentric RA/DEC of an Earth-fixed position (metres) at at_time (timezone-aware
    datetime) - same frame and return shape as iss_tracker.get_current_radec:
    (ra_deg, dec_deg, alt_deg, az_deg, above_horizon). Raises ImportError without skyfield."""
    global _timescale
    from skyfield.api import load, wgs84  # optional dependency, imported lazily
    from skyfield.toposlib import ITRSPosition
    from skyfield.units import Distance
    if _timescale is None:
        _timescale = load.timescale()
    skyfield_time = _timescale.from_datetime(at_time)
    vehicle = ITRSPosition(Distance(m=list(r_ecef_m)))
    observer = wgs84.latlon(lat_deg, lon_deg, elevation_m)
    topocentric = (vehicle - observer).at(skyfield_time)
    ra, dec, _distance = topocentric.radec()
    alt, az, _distance = topocentric.altaz()
    return ra.hours * 15.0, dec.degrees, alt.degrees, az.degrees, alt.degrees > 0.0

"""
solar_system.py - real-time Sun/Moon sky position, apparent angular size, and lunar phase.

Uses skyfield (already a dependency for ISS tracking - see iss_tracker.py) with the JPL DE421
ephemeris, which skyfield downloads and caches automatically on first use (~17MB, one-time).
skyfield is imported lazily so the rest of the app still works without it installed.
"""

import os

# Real body radii (km) - used to convert distance into apparent angular diameter.
_MOON_RADIUS_KM = 1737.4
_SUN_RADIUS_KM = 696000.0

# skyfield's default Loader downloads into the CURRENT WORKING DIRECTORY, not a fixed location -
# anchoring it to this script's own directory instead means the ephemeris is found/cached in the
# same place regardless of what directory tracker_gui.py happens to be launched from.
_DATA_DIR = os.path.dirname(os.path.abspath(__file__))

_eph = None
_ts = None


def _load_ephemeris():
    global _eph, _ts
    if _eph is None:
        from skyfield.api import Loader  # optional dependency, imported lazily
        loader = Loader(_DATA_DIR)
        _eph = loader('de421.bsp')  # auto-downloaded + cached here on first call
        _ts = loader.timescale()
    return _eph, _ts


def _angular_diameter_deg(distance_km, radius_km):
    import math
    return math.degrees(2.0 * math.atan(radius_km / distance_km))


def get_sun_info(lat_deg, lon_deg, elevation_m=0.0):
    """Returns (ra_deg, dec_deg, angular_diameter_deg)."""
    from skyfield.api import wgs84
    eph, ts = _load_ephemeris()
    t = ts.now()
    earth, sun = eph['earth'], eph['sun']
    observer = earth + wgs84.latlon(lat_deg, lon_deg, elevation_m)
    astrometric = observer.at(t).observe(sun).apparent()
    ra, dec, distance = astrometric.radec()
    ang_diam = _angular_diameter_deg(distance.km, _SUN_RADIUS_KM)
    return ra.hours * 15.0, dec.degrees, ang_diam


def get_moon_info(lat_deg, lon_deg, elevation_m=0.0):
    """Returns (ra_deg, dec_deg, angular_diameter_deg, illuminated_fraction, phase_deg, waxing).

    phase_deg: 0=new, 90=first quarter, 180=full, 270=last quarter (standard "moon age angle").
    illuminated_fraction: 0 (new) to 1 (full), derived from phase_deg.
    waxing: True from new->full (phase_deg 0-180), False from full->new (180-360).
    """
    import math
    from skyfield.api import wgs84
    from skyfield import almanac
    eph, ts = _load_ephemeris()
    t = ts.now()
    earth, moon = eph['earth'], eph['moon']
    observer = earth + wgs84.latlon(lat_deg, lon_deg, elevation_m)
    astrometric = observer.at(t).observe(moon).apparent()
    ra, dec, distance = astrometric.radec()
    ang_diam = _angular_diameter_deg(distance.km, _MOON_RADIUS_KM)

    phase_deg = almanac.moon_phase(eph, t).degrees % 360.0
    illuminated_fraction = (1.0 - math.cos(math.radians(phase_deg))) / 2.0
    waxing = phase_deg < 180.0
    return ra.hours * 15.0, dec.degrees, ang_diam, illuminated_fraction, phase_deg, waxing


# DE421 target names - Mercury/Venus/Mars have their own direct segments, but the outer planets
# only have barycenter segments (no separate planet-center ephemeris, since their moons' offset
# from the barycenter isn't included in this kernel) - verified against skyfield's own documented
# list of de421.bsp segments before using these, rather than guessing the naming convention. All
# 8 IAU planets (Pluto excluded - reclassified as a dwarf planet in 2006, and DE421 doesn't carry
# it as a full segment anyway).
_PLANET_TARGETS = {
    "Mercury": "mercury",
    "Venus": "venus",
    "Mars": "mars",
    "Jupiter": "jupiter barycenter",
    "Saturn": "saturn barycenter",
    "Uranus": "uranus barycenter",
    "Neptune": "neptune barycenter",
}

# Equatorial radii (km) - same role as _MOON_RADIUS_KM/_SUN_RADIUS_KM above, used to convert
# distance into real apparent angular diameter so the GUI can draw these to true scale once
# zoomed in enough (values well below the arcsecond-to-arcminute range at typical distances, but
# real nonetheless - Jupiter and Saturn especially become clearly resolvable at high zoom).
_PLANET_RADII_KM = {
    "Mercury": 2440.5,
    "Venus": 6051.8,
    "Mars": 3396.2,
    "Jupiter": 71492.0,
    "Saturn": 60268.0,
    "Uranus": 25559.0,
    "Neptune": 24764.0,
}

# Saturn's ring system's outer edge (the A ring) - gives the rings their own real angular extent
# (about 2.27x the planet's own radius) alongside the disk itself, for the GUI to draw them at
# true relative scale once zoomed in enough, rather than an arbitrary fixed size.
_SATURN_RING_OUTER_KM = 136780.0


def get_planets_info(lat_deg, lon_deg, elevation_m=0.0):
    """Returns {name: (ra_deg, dec_deg, angular_diameter_deg, ring_angular_diameter_deg,
    illuminated_fraction)} for all 8 planets, all sampled at the same instant.
    ring_angular_diameter_deg is None for every planet except Saturn, where it's the ring system's
    own real angular diameter (see _SATURN_RING_OUTER_KM) - the GUI uses angular_diameter_deg the
    same way it already does for the Sun/Moon (draw to true scale once zoomed in enough that it
    exceeds a minimum marker size, a small fixed dot otherwise, since most of these are too small
    to resolve at low zoom).

    illuminated_fraction (0-1, via skyfield's own Apparent.fraction_illuminated()) is real, not
    approximated - Mercury and Venus show genuine, sometimes dramatic phases (inferior planets,
    same reason the Moon does), Mars a slight gibbous, and the outer planets essentially always
    full (their Sun-planet-Earth phase angle is tiny at any real distance) - the GUI applies the
    exact same phase-rendering technique used for the Moon uniformly to all of them, so this
    naturally comes out ~1.0 for the ones that don't show a visible phase rather than needing to
    special-case which planets bother to compute it."""
    from skyfield.api import wgs84
    eph, ts = _load_ephemeris()
    t = ts.now()
    sun = eph['sun']
    observer = eph['earth'] + wgs84.latlon(lat_deg, lon_deg, elevation_m)
    result = {}
    for name, target in _PLANET_TARGETS.items():
        astrometric = observer.at(t).observe(eph[target]).apparent()
        ra, dec, distance = astrometric.radec()
        ang_diam = _angular_diameter_deg(distance.km, _PLANET_RADII_KM[name])
        ring_ang_diam = _angular_diameter_deg(distance.km, _SATURN_RING_OUTER_KM) if name == "Saturn" else None
        illum_fraction = astrometric.fraction_illuminated(sun)
        result[name] = (ra.hours * 15.0, dec.degrees, ang_diam, ring_ang_diam, illum_fraction)
    return result

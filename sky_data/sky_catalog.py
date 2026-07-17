"""
sky_catalog.py - builds/updates the bundled night-sky object catalog (stars + Messier DSOs)
used by tracker_gui.py's sky visualization.

Data sources (all public, freely licensed):
  - Stars: AT-HYG v3.3, "reduced_m11" subset (871,139 stars: complete to mag +11.0, plus every
    star within 100 light-years regardless of magnitude), CC BY-SA 4.0.
    https://codeberg.org/astronexus/athyg
    Same author/lineage as the HYG database this used to pull from (astronexus), but built from
    Tycho-2 + Gaia DR3 + Hipparcos + Yale Bright Star + Gliese rather than HYG's narrower three-
    catalog compilation - HYG (even completely unfiltered by magnitude) turned out to still be
    missing the vast majority of real stars in the mag 8-13 range, since it's a curated
    compilation of specific catalogs rather than a magnitude-complete survey. Confirmed on real
    hardware: a 10s exposure through a 150mm scope showed far more stars than HYG had at any
    magnitude limit. AT-HYG's reduced_m11 subset is deep enough to match that, still a single
    plain-CSV file parseable with the standard library, and small enough to bundle (~70MB
    compressed download, well under the multi-hundred-MB-plus of the full AT-HYG catalog or
    catalogs like Tycho-2/UCAC4 in their native formats).
  - Deep-sky objects: OpenNGC, CC-BY-SA-4.0. https://github.com/mattiaverga/OpenNGC
    Filtered to Messier-numbered objects only (110 objects) - the well-known "highlight reel"
    of naked-eye/small-telescope targets, rather than the full ~14000-object NGC/IC catalog,
    to keep the bundled file small.
  - Constellation lines: Stellarium's "modern" skyculture, CC BY-SA 4.0.
    https://github.com/Stellarium/stellarium (skycultures/modern/index.json) - each line is a
    polyline of Hipparcos catalog numbers, which join directly against HYG's own "hip" column.

Run directly (`python sky_catalog.py`) to force a rebuild, or import ensure_catalog_current()
to build/update only if the local file is missing or stale (used at GUI startup).
"""

import csv
import gzip
import io
import json
import os
import time
import urllib.request

ATHYG_URL = "https://codeberg.org/astronexus/athyg/media/branch/main/data/subsets/athyg_33_reduced_m11.csv.gz"
NGC_URL = "https://raw.githubusercontent.com/mattiaverga/OpenNGC/master/database_files/NGC.csv"
ADDENDUM_URL = "https://raw.githubusercontent.com/mattiaverga/OpenNGC/master/database_files/addendum.csv"
CONSTELLATIONS_URL = "https://raw.githubusercontent.com/Stellarium/stellarium/master/skycultures/modern/index.json"

CATALOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sky_catalog.json")

# Stars fainter than this are dropped entirely. Set to 21.0 - comfortably past the faintest
# magnitude actually present in the AT-HYG reduced_m11 subset (verified directly against the
# source data: 20.1, from the "every star within 100ly regardless of magnitude" bonus inclusions
# past its main mag-11 cutoff) - so effectively NO stars get filtered out by magnitude here, the
# subset's own selection criteria are the only real limit. _star_mag_limit_for_zoom() scales how
# many of these actually get DRAWN at any given zoom level against this same constant.
STAR_MAG_LIMIT = 21.0

# (B-V color index, (R,G,B)) stops for a simple piecewise-linear color approximation - not a
# rigorous blackbody model, but a widely-used approximation good enough to render "this star
# looks blue-white/yellow/orange/red" at a glance, which is all the viz needs.
_BV_COLOR_STOPS = [
    (-0.4, (155, 176, 255)),
    (0.0, (202, 215, 255)),
    (0.2, (248, 247, 255)),
    (0.4, (255, 244, 234)),
    (0.6, (255, 229, 207)),
    (0.8, (255, 217, 178)),
    (1.0, (255, 199, 142)),
    (1.2, (255, 185, 116)),
    (1.4, (255, 166, 81)),
    (1.6, (255, 147, 41)),
    (2.0, (255, 101, 25)),
]


def bv_to_hex_color(bv):
    """Approximate a star's visual color from its B-V color index."""
    if bv is None:
        return "#ffffff"
    stops = _BV_COLOR_STOPS
    if bv <= stops[0][0]:
        r, g, b = stops[0][1]
        return f"#{r:02x}{g:02x}{b:02x}"
    if bv >= stops[-1][0]:
        r, g, b = stops[-1][1]
        return f"#{r:02x}{g:02x}{b:02x}"
    for (bv0, c0), (bv1, c1) in zip(stops, stops[1:]):
        if bv0 <= bv <= bv1:
            t = (bv - bv0) / (bv1 - bv0)
            r = round(c0[0] + (c1[0] - c0[0]) * t)
            g = round(c0[1] + (c1[1] - c0[1]) * t)
            b = round(c0[2] + (c1[2] - c0[2]) * t)
            return f"#{r:02x}{g:02x}{b:02x}"
    return "#ffffff"


def _parse_sexagesimal_ra(s):
    """OpenNGC RA is 'HH:MM:SS.s' - convert to degrees (0-360)."""
    h, m, sec = s.split(":")
    return (float(h) + float(m) / 60.0 + float(sec) / 3600.0) * 15.0


def _parse_sexagesimal_dec(s):
    """OpenNGC Dec is '+DD:MM:SS.s' - convert to degrees (-90..+90)."""
    sign = -1.0 if s.strip().startswith("-") else 1.0
    s = s.strip().lstrip("+-")
    d, m, sec = s.split(":")
    return sign * (float(d) + float(m) / 60.0 + float(sec) / 3600.0)


def _fetch_text(url, timeout=30):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read()


# HYG's "bayer" field uses these standard 3-letter Greek-letter abbreviations (sometimes suffixed
# "-1", "-2" etc. for multi-star systems sharing one letter, e.g. "Alp-1" for Alpha Centauri A) -
# expanded here purely to build a readable common name like "Tau Ceti" for stars that DO have a
# Bayer letter + constellation but DON'T have a curated "proper" name in HYG (which turns out to
# be most traditionally-named stars, including some as well-known as Tau Ceti itself - its
# "proper" field is blank, so without this it would only ever show up as the raw "52Tau Cet").
_GREEK = {
    "alp": "Alpha", "bet": "Beta", "gam": "Gamma", "del": "Delta", "eps": "Epsilon",
    "zet": "Zeta", "eta": "Eta", "the": "Theta", "iot": "Iota", "kap": "Kappa",
    "lam": "Lambda", "mu": "Mu", "nu": "Nu", "xi": "Xi", "omi": "Omicron", "pi": "Pi",
    "rho": "Rho", "sig": "Sigma", "tau": "Tau", "ups": "Upsilon", "phi": "Phi",
    "chi": "Chi", "psi": "Psi", "ome": "Omega",
}

# All 88 IAU constellations, abbreviation -> genitive form (the form used in traditional star
# names, e.g. "Tauri" in "Tau Tauri", "Orionis" in "Alpha Orionis") - standard, unchanging
# reference data, same spirit as the Messier catalog. Verified against HYG's own full set of
# "con" values (all 88 present in the data) rather than assumed from memory alone.
_CONSTELLATION_GENITIVE = {
    "And": "Andromedae", "Ant": "Antliae", "Aps": "Apodis", "Aql": "Aquilae", "Aqr": "Aquarii",
    "Ara": "Arae", "Ari": "Arietis", "Aur": "Aurigae", "Boo": "Bootis", "CMa": "Canis Majoris",
    "CMi": "Canis Minoris", "CVn": "Canum Venaticorum", "Cae": "Caeli", "Cam": "Camelopardalis",
    "Cap": "Capricorni", "Car": "Carinae", "Cas": "Cassiopeiae", "Cen": "Centauri",
    "Cep": "Cephei", "Cet": "Ceti", "Cha": "Chamaeleontis", "Cir": "Circini", "Cnc": "Cancri",
    "Col": "Columbae", "Com": "Comae Berenices", "CrA": "Coronae Australis",
    "CrB": "Coronae Borealis", "Crt": "Crateris", "Cru": "Crucis", "Crv": "Corvi",
    "Cyg": "Cygni", "Del": "Delphini", "Dor": "Doradus", "Dra": "Draconis", "Equ": "Equulei",
    "Eri": "Eridani", "For": "Fornacis", "Gem": "Geminorum", "Gru": "Gruis", "Her": "Herculis",
    "Hor": "Horologii", "Hya": "Hydrae", "Hyi": "Hydri", "Ind": "Indi", "LMi": "Leonis Minoris",
    "Lac": "Lacertae", "Leo": "Leonis", "Lep": "Leporis", "Lib": "Librae", "Lup": "Lupi",
    "Lyn": "Lyncis", "Lyr": "Lyrae", "Men": "Mensae", "Mic": "Microscopii", "Mon": "Monocerotis",
    "Mus": "Muscae", "Nor": "Normae", "Oct": "Octantis", "Oph": "Ophiuchi", "Ori": "Orionis",
    "PsA": "Piscis Austrini", "Pav": "Pavonis", "Peg": "Pegasi", "Per": "Persei",
    "Phe": "Phoenicis", "Pic": "Pictoris", "Psc": "Piscium", "Pup": "Puppis", "Pyx": "Pyxidis",
    "Ret": "Reticuli", "Scl": "Sculptoris", "Sco": "Scorpii", "Sct": "Scuti", "Ser": "Serpentis",
    "Sex": "Sextantis", "Sge": "Sagittae", "Sgr": "Sagittarii", "Tau": "Tauri", "Tel": "Telescopii",
    "TrA": "Trianguli Australis", "Tri": "Trianguli", "Tuc": "Tucanae", "UMa": "Ursae Majoris",
    "UMi": "Ursae Minoris", "Vel": "Velorum", "Vir": "Virginis", "Vol": "Volantis",
    "Vul": "Vulpeculae",
}


def _bayer_common_name(bayer, con):
    """Build a 'Tau Ceti'-style common name from HYG's raw bayer ('Tau', 'Alp-1', ...) and con
    ('Cet') fields, or None if either piece is missing/unrecognized."""
    if not bayer or not con:
        return None
    genitive = _CONSTELLATION_GENITIVE.get(con)
    if not genitive:
        return None
    base = bayer.split("-")[0].lower()
    suffix = bayer[len(base):] if bayer[:len(base)].lower() == base else ""
    greek = _GREEK.get(base)
    if not greek:
        return None
    return f"{greek}{suffix} {genitive}"


def _build_stars():
    raw = _fetch_text(ATHYG_URL)
    text = gzip.decompress(raw).decode("utf-8", errors="replace")
    reader = csv.DictReader(io.StringIO(text))
    stars = []
    for row in reader:
        try:
            # AT-HYG's Sun row ('Sol') is id=1 here (not 0, unlike HYG) and has dist=0.0 as a
            # placeholder - checking the name directly is exact and doesn't depend on id
            # numbering staying stable across AT-HYG versions.
            if row.get("proper") == "Sol":
                continue
            mag = float(row["mag"])
            if mag > STAR_MAG_LIMIT:
                continue
            ra_deg = float(row["ra"]) * 15.0
            dec_deg = float(row["dec"])
            ci = row.get("ci")
            bv = float(ci) if ci not in (None, "") else None
            hip = row.get("hip")

            proper = row.get("proper") or ""
            bayer_name = _bayer_common_name(row.get("bayer"), row.get("con"))
            # AT-HYG splits what HYG called "bf" (e.g. "3Alp Lyr") into separate bayer/flam
            # fields - reconstruct an equivalent raw designation fallback from Flamsteed number +
            # constellation genitive (e.g. "3 Lyrae") for stars with a Flamsteed number but no
            # curated proper name and no recognized Bayer letter.
            flam = row.get("flam") or ""
            con = row.get("con") or ""
            genitive = _CONSTELLATION_GENITIVE.get(con)
            flam_name = f"{flam} {genitive}" if flam and genitive else ""
            # Display name: prefer AT-HYG's curated proper name, then the constructed "Tau Ceti"
            # style common name, then the Flamsteed-based fallback. Left blank (not a synthesized
            # "HIP 12345"-style placeholder) for the ~750000 stars added beyond original HYG that
            # have none of these - _load_sky_catalog_from_disk's search index already skips
            # nameless stars, so leaving this blank keeps that index exactly as small as before
            # despite the much bigger total star count; they're still drawn on the viz, just not
            # individually searchable by name (their real catalog numbers are still in aliases).
            name = proper or bayer_name or flam_name

            # Search aliases: every name/id form someone might reasonably type, beyond the
            # primary display name above - so "Tau Ceti" finds a star whose display name is
            # "52 Ceti" (proper name takes priority for display, but the Bayer common name and
            # raw designation should still both be searchable), and catalog numbers work as
            # "official name" lookups too.
            aliases = set()
            if bayer_name and bayer_name != name:
                aliases.add(bayer_name)
            if flam_name and flam_name != name:
                aliases.add(flam_name)
            if hip:
                aliases.add(f"HIP {hip}")
            hd = row.get("hd")
            if hd:
                aliases.add(f"HD {hd}")
            gl = row.get("gl")
            if gl:
                aliases.add(gl)
            tyc = row.get("tyc")
            if tyc:
                aliases.add(f"TYC {tyc}")

            star = {
                "ra": round(ra_deg, 5),
                "dec": round(dec_deg, 5),
                "mag": round(mag, 2),
                "color": bv_to_hex_color(bv),
                "name": name,
            }
            if aliases:
                star["aliases"] = sorted(aliases)
            # hip links this star to constellation line data (see _build_constellation_lines) -
            # only kept when present so most star dicts stay compact.
            if hip:
                star["hip"] = int(hip)
            stars.append(star)
        except (ValueError, KeyError):
            continue
    return stars


def _format_ngc_ic_name(raw_name):
    """OpenNGC's primary 'Name' field is like 'NGC0224' or 'IC0001' - reformat to the
    conventional spaced, non-zero-padded form ('NGC 224', 'IC 1') people actually search for."""
    if not raw_name:
        return None
    prefix = "NGC" if raw_name.startswith("NGC") else "IC" if raw_name.startswith("IC") else None
    if not prefix:
        return raw_name
    number = raw_name[len(prefix):].lstrip("0") or "0"
    return f"{prefix} {number}"


def _build_dso():
    dso = []
    for url in (NGC_URL, ADDENDUM_URL):
        raw = _fetch_text(url).decode("utf-8", errors="replace")
        reader = csv.DictReader(io.StringIO(raw), delimiter=";")
        for row in reader:
            m = (row.get("M") or "").strip()
            if not m:
                continue  # Messier-numbered objects only - see module docstring
            try:
                ra_deg = _parse_sexagesimal_ra(row["RA"])
                dec_deg = _parse_sexagesimal_dec(row["Dec"])
            except (ValueError, KeyError):
                continue
            maj_ax = row.get("MajAx") or ""
            min_ax = row.get("MinAx") or ""
            pos_ang = row.get("PosAng") or ""
            mag = row.get("V-Mag") or row.get("B-Mag") or ""
            common = (row.get("Common names") or "").split(",")[0].strip()
            # Catalog/official identifier ("NGC 224" etc.) alongside the Messier number and any
            # common name - so a search for either the official designation or the popular name
            # finds the same object (e.g. "andromeda galaxy", "m31", and "ngc 224" should all work).
            aliases = set()
            catalog_name = _format_ngc_ic_name(row.get("Name"))
            if catalog_name:
                aliases.add(catalog_name)
            dso.append({
                "ra": round(ra_deg, 5),
                "dec": round(dec_deg, 5),
                "type": row.get("Type", ""),
                "mag": float(mag) if mag else None,
                # Apparent size in arcmin (major/minor axis) - used to draw an "accurate-ish"
                # ellipse instead of a plain point once zoomed in enough for it to matter.
                "size_maj_arcmin": float(maj_ax) if maj_ax else None,
                "size_min_arcmin": float(min_ax) if min_ax else None,
                # Position angle of the major axis, standard astronomical convention (degrees,
                # measured from North through East) - without this every ellipse was drawn
                # axis-aligned (major axis always horizontal), which is wrong for the ~2/3 of
                # galaxies/nebulae that aren't oriented that way on the sky.
                "pos_ang_deg": float(pos_ang) if pos_ang else 0.0,
                # int() strips OpenNGC's zero-padding ("031" -> 31) - left as-is, "M031" wouldn't
                # even substring-match a search for "M31", the exact official designation people
                # actually type.
                "messier": f"M{int(m)}",
                "name": common,
                "aliases": sorted(aliases),
            })
    dso.sort(key=lambda d: int(d["messier"][1:]))
    return dso


def _build_constellation_lines():
    """Flatten Stellarium's per-constellation polylines into a single list of [hip_a, hip_b]
    segments - simpler for the GUI to consume than the nested per-constellation structure, since
    all it needs is 'given this star's hip, what other hips is it directly linked to'."""
    data = json.loads(_fetch_text(CONSTELLATIONS_URL))
    segments = []
    for con in data.get("constellations", []):
        for line in con.get("lines", []):
            for a, b in zip(line, line[1:]):
                segments.append([a, b])
    return segments


def build_catalog(path=CATALOG_PATH):
    """Fetch fresh data from all sources and write the combined catalog JSON. Requires internet."""
    catalog = {
        "built_unix": time.time(),
        "stars": _build_stars(),
        "dso": _build_dso(),
        "const_lines": _build_constellation_lines(),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(catalog, f, separators=(",", ":"))
    return catalog


def ensure_catalog_current(path=CATALOG_PATH, max_age_days=60):
    """Build the catalog if missing, or refresh it if older than max_age_days. Safe to call
    from a background thread at GUI startup - does nothing (and raises nothing but a logged
    exception upstream) if there's no internet available and a catalog already exists."""
    if os.path.exists(path):
        age_days = (time.time() - os.path.getmtime(path)) / 86400.0
        if age_days < max_age_days:
            return False  # up to date, nothing to do
    build_catalog(path)
    return True


if __name__ == "__main__":
    print("Building sky catalog from AT-HYG + OpenNGC + Stellarium constellation lines ...")
    cat = build_catalog()
    print(f"Wrote {CATALOG_PATH}: {len(cat['stars'])} stars (mag <= {STAR_MAG_LIMIT}), "
          f"{len(cat['dso'])} Messier objects, {len(cat['const_lines'])} constellation line segments.")

"""SIH_AVIAN_FINAL -- terrain, river, banks, embankments, bank detail.

Written fresh rather than reusing AVIAN_ENVIRONMENT/source/terrain.py: that
module's build() hardcodes REV-C-scale absolute placements (a jetty at
y=180, a 620 m revetment) that make no sense on a 360 m x 150 m footprint.
The reusable IDEAS -- light deterministic noise, an edge-fade ring so the
Nishita sky never shows a void, link_dup instancing for rocks/trees -- are
kept; the numbers are this scene's own.

ASSUMPTIONS (working rule 5)
-----------------------------
* Structural members always place from `params_final.ground_z()` directly
  (bridge.py/metro.py already do this) -- `height()` here, with its light
  noise, is for DECORATIVE ground objects only (rocks, trees, riprap,
  distant skyline), never for anything a validation check measures against.
* "Waterline staining on every pier that meets water" (SPEC.md) is applied
  (in materials_final, Checkpoint B) to the piers nearest the river -- x=135
  and x=225, both structures -- even though the main span is sized so no
  pier stands IN the channel. At this proximity, splash and monsoon damp
  make staining physically honest without contradicting "no pier in the
  river".
"""
from __future__ import annotations
import math
import random

import meshlib as ML
import params_final as PF

_FIELD_SEED = PF.SEED


def _noise2(x, y, scale=60.0, amp=0.15):
    """Cheap deterministic value-noise -- light, decorative only."""
    def h(i, j):
        n = (i * 73856093) ^ (j * 19349663) ^ (_FIELD_SEED * 83492791)
        n = (n << 13) ^ n
        return (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589)
                       & 0x7fffffff) / 1073741824.0)
    u, v = x / scale, y / scale
    i, j = math.floor(u), math.floor(v)
    fu, fv = u - i, v - j
    su, sv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
    a = h(i, j) * (1 - su) + h(i + 1, j) * su
    b = h(i, j + 1) * (1 - su) + h(i + 1, j + 1) * su
    return (a * (1 - sv) + b * sv) * amp


def _edge_fade(x, y):
    m = 40.0
    fx = min((x - PF.GROUND_X0) / m, (PF.GROUND_X1 - x) / m, 1.0)
    fy = min((y - PF.GROUND_Y0) / m, (PF.GROUND_Y1 - y) / m, 1.0)
    f = max(0.0, min(1.0, min(fx, fy)))
    return f * f * (3.0 - 2.0 * f)


def height(x, y):
    """Decorative ground height: ideal profile + light noise + edge fade."""
    base = PF.ground_z(x, y)
    if base > PF.RIVER_WATER_Z + 0.1:      # only perturb dry land
        base = base + _noise2(x, y)
    f = _edge_fade(x, y)
    return PF.FAR_GROUND_Z + (base - PF.FAR_GROUND_Z) * f


def is_water(x, y):
    return PF.ground_z(x, y) < PF.RIVER_WATER_Z - 0.05


def build(colls, mats, log=print):
    rnd = random.Random(PF.SEED)

    # ---- distant ground ring (so the Nishita sky never shows a void) ------
    pad = PF.FAR_GROUND_PAD
    xs = [PF.GROUND_X0 - pad, PF.GROUND_X0, PF.GROUND_X1, PF.GROUND_X1 + pad]
    ys = [PF.GROUND_Y0 - pad, PF.GROUND_Y0, PF.GROUND_Y1, PF.GROUND_Y1 + pad]
    fv = [(x, y, PF.FAR_GROUND_Z) for x in xs for y in ys]
    ff = []
    for i in range(3):
        for j in range(3):
            if i == 1 and j == 1:
                continue
            a = i * 4 + j
            ff.append((a, a + 1, a + 5, a + 4))
    far = ML.mesh_obj("ENV_GROUND_FAR", fv, ff, colls["TERRAIN"],
                      mats["ground_far"])
    ML.set_custom(far, {"avi_kind": "distant_ground"})

    # ---- main terrain grid --------------------------------------------------
    ground = ML.grid("ENV_TERRAIN", PF.GROUND_X0, PF.GROUND_X1,
                     PF.GROUND_Y0, PF.GROUND_Y1,
                     PF.GROUND_RES_X, PF.GROUND_RES_Y, height,
                     colls["TERRAIN"], mats["ground"])
    ML.set_custom(ground, {"avi_kind": "terrain"})

    # ---- river water surface ------------------------------------------------
    half = PF.RIVER_WIDTH / 2.0
    water = ML.grid("ENV_RIVER_WATER",
                    PF.RIVER_CENTRE_X - half - 20.0,
                    PF.RIVER_CENTRE_X + half + 20.0,
                    PF.GROUND_Y0, PF.GROUND_Y1,
                    28, 30, lambda x, y: PF.RIVER_WATER_Z,
                    colls["TERRAIN"], mats["water"])
    ML.set_custom(water, {"avi_kind": "river_water",
                          "avi_width_m": PF.RIVER_WIDTH,
                          "avi_surface_z": PF.RIVER_WATER_Z})

    # ---- riprap at the abutments (rock armour on the embankment slopes) ---
    rproto = ML.cylinder("_ROCK_PROTO", 0.6, 0.7, (0, 0, 0), 7,
                         colls["TERRAIN"], mats["rock"], smooth=False)
    ML.displace_random(rproto, 0.22, seed=PF.SEED)
    rproto.hide_render = True
    rproto.hide_viewport = True
    n_rock = 0
    for end_x, sgn in ((0.0, -1.0), (PF.BRIDGE_LENGTH, 1.0)):
        for i in range(8):
            x = end_x + sgn * rnd.uniform(2.0, PF.EMBANKMENT_RUN - 2.0)
            y = rnd.uniform(-9.0, 9.0)
            gz = height(x, y)
            s = rnd.uniform(0.6, 1.4)
            ML.link_dup(rproto, f"ENV_RIPRAP_{n_rock+1:03d}",
                       (x, y, gz + 0.15 * s),
                       (rnd.uniform(0, 3.1), rnd.uniform(0, 3.1),
                        rnd.uniform(0, 6.28)),
                       (s, s, s * 0.7), colls["TERRAIN"])
            n_rock += 1
    # riprap at the river banks proper, either side of the channel
    for i in range(7):
        side = 1 if i % 2 == 0 else -1
        d = half + rnd.uniform(0.5, PF.RIVER_BANK_SLOPE * 2.0 - 1.0)
        x = PF.RIVER_CENTRE_X + side * d
        y = rnd.uniform(-25.0, 45.0)
        gz = height(x, y)
        s = rnd.uniform(0.6, 1.5)
        ML.link_dup(rproto, f"ENV_RIPRAP_{n_rock+1:03d}",
                   (x, y, gz + 0.15 * s),
                   (rnd.uniform(0, 3.1), rnd.uniform(0, 3.1),
                    rnd.uniform(0, 6.28)),
                   (s, s, s * 0.7), colls["TERRAIN"])
        n_rock += 1

    # ---- debris caught at the upstream (south, -X) pier faces --------------
    dproto = ML.box("_DEBRIS_PROTO", (0.9, 0.5, 0.35), (0, 0, 0),
                    colls["TERRAIN"], mats["rock"])
    ML.displace_random(dproto, 0.12, seed=PF.SEED + 3)
    dproto.hide_render = True
    dproto.hide_viewport = True
    n_debris = 0
    for x in (135.0, 225.0):                # piers flanking the main span
        for y in (0.0, 28.0):               # road pier, metro pier
            for k in range(2):
                dx = x - rnd.uniform(1.0, 2.0)       # upstream = -X face
                dy = y + rnd.uniform(-3.5, 3.5)
                gz = height(dx, dy)
                s = rnd.uniform(0.7, 1.3)
                ML.link_dup(dproto, f"ENV_DEBRIS_{n_debris+1:03d}",
                           (dx, dy, max(gz, PF.RIVER_WATER_Z) + 0.15 * s),
                           (0, 0, rnd.uniform(0, 6.28)),
                           (s, s, s), colls["TERRAIN"])
                n_debris += 1

    # ---- bank vegetation: ~20 trees ----------------------------------------
    vproto = ML.cylinder("_VEG_PROTO", 0.7, 1.8, (0, 0, 0.9), 5,
                         colls["TERRAIN"], mats["veg"], smooth=False)
    ML.displace_random(vproto, 0.28, seed=PF.SEED + 7)
    vproto.hide_render = True
    vproto.hide_viewport = True
    n_veg = 0
    for i in range(PF.RIVER_VEG_COUNT):
        side = 1 if i % 2 == 0 else -1
        d = half + rnd.uniform(6.0, 24.0)
        x = PF.RIVER_CENTRE_X + side * d
        y = rnd.uniform(PF.GROUND_Y0 + 5.0, PF.GROUND_Y1 - 5.0)
        if -12.0 < y < 40.0 and abs(x - PF.RIVER_CENTRE_X) < half + 12:
            continue                          # keep the under-bridge area clear
        gz = height(x, y)
        if gz < PF.RIVER_WATER_Z + 0.1:
            continue
        s = rnd.uniform(0.5, 1.6)
        ML.link_dup(vproto, f"ENV_BANKVEG_{n_veg+1:03d}", (x, y, gz),
                   (0, 0, rnd.uniform(0, 6.28)),
                   (s, s, s * rnd.uniform(0.7, 1.5)), colls["TERRAIN"])
        n_veg += 1

    # ---- 2-3 distant building silhouettes on the horizon, for scale --------
    # Locked pre-flight decision 3. Non-structural, CITY_-prefixed so the
    # collision exporter's EXCLUDE_PREFIXES keeps them out automatically.
    # Realism pass: routed through materials_final.silhouette() (_aerial(),
    # dim albedo) rather than a flat bright colour, and given a flat roof
    # line with a parapet lip so they read as buildings, not slabs.
    n_city = 0
    sky_x = [PF.GROUND_X0 + 30.0, PF.GROUND_X0 + 55.0, PF.GROUND_X1 - 40.0]
    for i, x in enumerate(sky_x[:PF.DISTANT_SKYLINE_COUNT]):
        y = (PF.GROUND_Y1 - 15.0) if i % 2 == 0 else (PF.GROUND_Y0 + 15.0)
        h = rnd.uniform(28.0, 55.0)
        w = rnd.uniform(14.0, 22.0)
        gz = height(x, y)
        ob = ML.box(f"CITY_SILHOUETTE_{i+1:02d}", (w, w * 0.8, h),
                   (x, y, gz + h / 2.0), colls["TERRAIN"], mats["bldg_far"])
        ML.set_custom(ob, {"avi_kind": "distant_silhouette"})
        parapet_h = h * 0.03 + 0.4
        ML.box(f"CITY_SILHOUETTE_{i+1:02d}_PARAPET",
              (w * 0.96, w * 0.8 * 0.96, parapet_h),
              (x, y, gz + h + parapet_h / 2.0), colls["TERRAIN"],
              mats["bldg_far"])
        n_city += 1

    log(f"  terrain : {PF.GROUND_RES_X}x{PF.GROUND_RES_Y} grid, "
        f"river {PF.RIVER_WIDTH:.0f} m wide (channel), water z={PF.RIVER_WATER_Z}")
    log(f"  terrain : {n_rock} riprap, {n_debris} debris, {n_veg} bank trees, "
        f"{n_city} distant silhouettes")
    return {"rocks": n_rock, "debris": n_debris, "veg": n_veg, "silhouettes": n_city}

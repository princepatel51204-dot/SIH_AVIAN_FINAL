"""Terrain, river channel, banks and riverside detail.

The river is not decoration. It is what stops a UAV treating the area under
the main span as ordinary ground: there is no emergency landing surface for
620 m, the piers stand in water, and the water reflects, which is a genuinely
difficult condition for downward-facing optical flow and for any depth sensor.
"""
from __future__ import annotations
import math
import random
import bpy

import params as P
import meshlib as ML


# ---------------------------------------------------------------------------
def _terrain_z(x, y, rnd_field):
    """Ground height with gentle rolling relief away from the channel.

    Relief is biased POSITIVE on dry land. A symmetric perturbation dips the
    terrain below the distant ground plane in places, and the distant plane
    then punches through as bright patches in aerial views.
    """
    z = P.ground_z(x, y)
    if z > -0.05:                       # only perturb dry land
        z = max(0.05, z + rnd_field(x, y) + 2.0)
    return z


def _make_field(seed):
    """Cheap deterministic value-noise, so terrain is reproducible."""
    def h(i, j):
        n = (i * 73856093) ^ (j * 19349663) ^ (seed * 83492791)
        n = (n << 13) ^ n
        return (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589)
                       & 0x7fffffff) / 1073741824.0)

    def f(x, y, scale=340.0, amp=2.6):
        u, v = x / scale, y / scale
        i, j = math.floor(u), math.floor(v)
        fu, fv = u - i, v - j
        su = fu * fu * (3 - 2 * fu)
        sv = fv * fv * (3 - 2 * fv)
        a = h(i, j) * (1 - su) + h(i + 1, j) * su
        b = h(i, j + 1) * (1 - su) + h(i + 1, j + 1) * su
        base = (a * (1 - sv) + b * sv) * amp
        # a second, finer octave
        u2, v2 = x / 95.0, y / 95.0
        i2, j2 = math.floor(u2), math.floor(v2)
        fu2, fv2 = u2 - i2, v2 - j2
        s2u = fu2 * fu2 * (3 - 2 * fu2)
        s2v = fv2 * fv2 * (3 - 2 * fv2)
        a2 = h(i2 + 40, j2) * (1 - s2u) + h(i2 + 41, j2) * s2u
        b2 = h(i2 + 40, j2 + 1) * (1 - s2u) + h(i2 + 41, j2 + 1) * s2u
        return base + (a2 * (1 - s2v) + b2 * s2v) * amp * 0.22
    return f


_FIELD = None


def _edge_fade(x, y):
    """1.0 well inside the detailed terrain, 0.0 at its boundary.

    Without this the detailed terrain sits as a raised slab on the distant
    ground with a visible step all the way round it, which is obvious from
    any aerial view.
    """
    m = 320.0
    fx = min((x - P.GROUND_X0) / m, (P.GROUND_X1 - x) / m, 1.0)
    fy = min((y - P.GROUND_Y0) / m, (P.GROUND_Y1 - y) / m, 1.0)
    f = max(0.0, min(1.0, min(fx, fy)))
    return f * f * (3.0 - 2.0 * f)


def height(x, y):
    """Terrain height at a point -- THE authoritative ground level.

    Everything that sits on the ground (buildings, trees, vehicles, lamp
    posts, rocks) must use this, not params.ground_z. ground_z is the ideal
    channel profile; this adds the rolling relief, the flattened corridor and
    the edge fade, and an object placed with the wrong one floats or sinks.
    """
    global _FIELD
    if _FIELD is None:
        _FIELD = _make_field(P.SEED)
    base = _terrain_z(x, y, _FIELD)
    d = abs(y)
    if d < 90.0 and P.ground_z(x, y) > -0.05:
        t = min(1.0, max(0.0, (d - 40.0) / 50.0))
        t = t * t * (3 - 2 * t)
        base = base * t
    if P.ground_z(x, y) > -0.05:
        f = _edge_fade(x, y)
        base = P.FAR_GROUND_Z + (base - P.FAR_GROUND_Z) * f
    return base


def is_water(x, y):
    return P.ground_z(x, y) < P.RIVER_WATER_Z - 0.05


def build(colls, mats, log=print):
    rnd = random.Random(P.SEED)
    z = height

    # Distant ground. The Nishita sky renders BLACK below the horizon -- it
    # is a sky model, not an environment -- so without this an aerial camera
    # sees a void wherever the detailed terrain ends. Low resolution: it is
    # only ever seen at range and through haze.
    # It is a RING, not a sheet. A full plane at FAR_GROUND_Z = -1.2 lies
    # ABOVE the river water at -2.0 and therefore covers the entire 620 m
    # river -- the water and the excavated channel simply never render, and
    # every view across the crossing shows dry ground where the river should
    # be. Nothing in the validation suite catches that, because the water
    # object is present and correct; it is just hidden.
    #
    # Nine cells with the middle one omitted, so the far ground starts exactly
    # where the detailed terrain ends and overlaps nothing.
    xs = [P.GROUND_X0 - P.FAR_GROUND_PAD, P.GROUND_X0,
          P.GROUND_X1, P.GROUND_X1 + P.FAR_GROUND_PAD]
    ys = [P.GROUND_Y0 - P.FAR_GROUND_PAD, P.GROUND_Y0,
          P.GROUND_Y1, P.GROUND_Y1 + P.FAR_GROUND_PAD]
    fv = [(x, y, P.FAR_GROUND_Z) for x in xs for y in ys]
    ff = []
    for i in range(3):
        for j in range(3):
            if i == 1 and j == 1:
                continue                      # the modelled terrain lives here
            a = i * 4 + j
            ff.append((a, a + 1, a + 5, a + 4))
    far = ML.mesh_obj("ENV_GROUND_FAR", fv, ff, colls["RIVER"],
                      mats["ground_far"])
    ML.set_custom(far, {"avi_kind": "distant_ground",
                        "avi_note": "ring around the modelled terrain; "
                                    "must not overlap the river"})

    ground = ML.grid("ENV_TERRAIN", P.GROUND_X0, P.GROUND_X1,
                     P.GROUND_Y0, P.GROUND_Y1,
                     P.GROUND_RES_X, P.GROUND_RES_Y, z,
                     colls["RIVER"], mats["ground"])
    ML.set_custom(ground, {"avi_kind": "terrain"})

    # ---- water surface ---------------------------------------------------
    half = P.RIVER_WIDTH / 2.0
    water = ML.grid("ENV_RIVER_WATER",
                    P.RIVER_CENTRE_X - half - 45.0,
                    P.RIVER_CENTRE_X + half + 45.0,
                    -P.RIVER_LENGTH / 2.0, P.RIVER_LENGTH / 2.0,
                    36, 60, lambda x, y: P.RIVER_WATER_Z,
                    colls["RIVER"], mats["water"])
    ML.set_custom(water, {"avi_kind": "river_water",
                          "avi_width_m": P.RIVER_WIDTH,
                          "avi_surface_z": P.RIVER_WATER_Z})

    # ---- riverbank rocks -------------------------------------------------
    # One base rock, instanced. A unique mesh per rock would cost 260 meshes
    # for something no sensor will ever inspect.
    proto = ML.cylinder("_ROCK_PROTO", 1.0, 1.2, (0, 0, 0), 7,
                        colls["RIVER"], mats["rock"], smooth=False)
    ML.displace_random(proto, 0.30, seed=P.SEED)
    proto.hide_render = True
    proto.hide_viewport = True
    n_rock = 0
    for i in range(P.RIVER_ROCK_COUNT):
        side = 1 if i % 2 == 0 else -1
        d = half + rnd.uniform(-8.0, 55.0)
        x = P.RIVER_CENTRE_X + side * d
        y = rnd.uniform(-P.RIVER_LENGTH * 0.42, P.RIVER_LENGTH * 0.42)
        gz = height(x, y)
        s = rnd.uniform(0.5, 2.4)
        ob = ML.link_dup(proto, f"ENV_ROCK_{i+1:03d}",
                         (x, y, gz + 0.25 * s),
                         (rnd.uniform(0, 3.1), rnd.uniform(0, 3.1),
                          rnd.uniform(0, 6.28)),
                         (s, s * rnd.uniform(0.6, 1.1), s * 0.62),
                         colls["RIVER"])
        n_rock += 1

    # ---- bank vegetation --------------------------------------------------
    vproto = ML.cylinder("_VEG_PROTO", 0.9, 2.2, (0, 0, 1.1), 5,
                         colls["RIVER"], mats["veg"], smooth=False)
    ML.displace_random(vproto, 0.35, seed=P.SEED + 7)
    vproto.hide_render = True
    vproto.hide_viewport = True
    n_veg = 0
    for i in range(P.RIVER_VEG_COUNT):
        side = 1 if i % 2 == 0 else -1
        d = half + rnd.uniform(4.0, 190.0)
        x = P.RIVER_CENTRE_X + side * d
        y = rnd.uniform(-P.RIVER_LENGTH * 0.46, P.RIVER_LENGTH * 0.46)
        if abs(y) < 55.0 and abs(x - P.RIVER_CENTRE_X) < half + 120:
            continue                      # keep the under-bridge area clear
        gz = height(x, y)
        if gz < P.RIVER_WATER_Z + 0.1:
            continue
        s = rnd.uniform(0.5, 1.8)
        ML.link_dup(vproto, f"ENV_BANKVEG_{i+1:04d}", (x, y, gz),
                    (0, 0, rnd.uniform(0, 6.28)),
                    (s, s, s * rnd.uniform(0.7, 1.6)), colls["RIVER"])
        n_veg += 1

    # ---- riverside infrastructure ----------------------------------------
    # A small jetty and a bank revetment on one side: real rivers under major
    # crossings have them, and they give the river-side inspection sector
    # something other than open water to navigate around.
    jx = P.RIVER_CENTRE_X - half + 12.0
    jetty = ML.box("ENV_JETTY_DECK", (26.0, 6.0, 0.5),
                   (jx, 180.0, P.RIVER_WATER_Z + 1.4),
                   colls["RIVER"], mats["concrete_low"])
    ML.set_custom(jetty, {"avi_kind": "jetty"})
    for k in range(6):
        ML.cylinder(f"ENV_JETTY_PILE_{k+1}", 0.35, 5.0,
                    (jx - 11 + k * 4.4, 180.0, P.RIVER_WATER_Z - 0.6), 8,
                    colls["RIVER"], mats["concrete_low"])
    for s in (1, -1):
        rev = ML.box(f"ENV_REVETMENT_{'W' if s > 0 else 'E'}",
                     (14.0, 620.0, 1.2),
                     (P.RIVER_CENTRE_X + s * (half + 5.0), 0.0,
                      P.RIVER_WATER_Z + 0.3),
                     colls["RIVER"], mats["rock"])
        ML.set_custom(rev, {"avi_kind": "revetment"})

    log(f"  terrain : {P.GROUND_RES_X}x{P.GROUND_RES_Y} grid")
    log(f"  river   : {P.RIVER_WIDTH:.0f} m wide, water z={P.RIVER_WATER_Z}, "
        f"{n_rock} rocks, {n_veg} bank plants")
    return {"rocks": n_rock, "veg": n_veg}

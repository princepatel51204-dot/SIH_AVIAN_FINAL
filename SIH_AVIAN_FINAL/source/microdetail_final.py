"""SIH_AVIAN_FINAL -- micro-detail closing the gap the first build's README
recorded: formwork lines, tie-hole recesses, construction-joint bleed,
honeycombing, chamfered arrises, parapet posts, and >3 mm crack relief
geometry. HIGH band only (the 4 piers inside the research zone, x=90/135/
225/270), added ADDITIVELY around the existing bridge.py/metro.py geometry
rather than editing those reused modules or their meshes.

Crack relief is the one exception that IS a boolean cut into the host --
that is what "relief" means (a raking light needs a real edge to cast a
shadow from) -- and it reuses damage.py's own `carve()` rather than
reimplementing booleans, the same way the hero defect does.
"""
from __future__ import annotations
import math
import random

import meshlib as ML

HIGH_PIER_X = (90.0, 135.0, 225.0, 270.0)


def _column_objects(prefix_list):
    import bpy
    return [o for o in bpy.data.objects
           if o.type == "MESH" and o.name.startswith(prefix_list)
           and "PIER_COL" in o.name]


def _cap_objects(prefix_list):
    import bpy
    return [o for o in bpy.data.objects
           if o.type == "MESH" and o.name.startswith(prefix_list)
           and "PIER_CAP" in o.name]


def formwork_and_ties(pier_columns, radius_fn, coll, mats, rnd, log=print):
    """Raised formwork board lines (ADDITIVE thin rings, not grooves -- a
    proud line under a low raking sun casts the same kind of shadow as a
    grooved one, without a boolean) and small flush tie-hole patches, on
    each HIGH-band pier column."""
    n_lines = n_ties = 0
    for ob in pier_columns:
        zs = sorted(v.co.z for v in ob.data.vertices)
        z0, z1 = zs[0], zs[-1]
        xs = sorted(set(round(v.co.x, 3) for v in ob.data.vertices))
        ys = sorted(set(round(v.co.y, 3) for v in ob.data.vertices))
        cx = (xs[0] + xs[-1]) / 2.0
        cy = (ys[0] + ys[-1]) / 2.0
        r = radius_fn(ob)
        if r <= 0:
            continue
        board_h = 0.6
        n = max(1, int((z1 - z0) / board_h))
        for i in range(1, n):
            z = z0 + i * board_h
            ring = ML.cylinder(f"_MD_{ob.name}_FORMLINE_{i:02d}", r + 0.006, 0.03,
                              (cx, cy, z), 20, coll, mats["concrete_pier"],
                              smooth=False)
            n_lines += 1
        # tie-hole patches: 3 rows around mid-height, 4 around the
        # circumference each -- the classic 4-per-panel former-tie grid
        for row in range(3):
            z = z0 + (row + 1) * (z1 - z0) / 4.0
            for k in range(4):
                a = k * (math.pi / 2.0) + rnd.uniform(-0.15, 0.15)
                px = cx + r * 0.985 * math.cos(a)
                py = cy + r * 0.985 * math.sin(a)
                ML.cylinder(f"_MD_{ob.name}_TIE_{row}{k}", 0.03, 0.012,
                          (px, py, z), 8, coll, mats["repair_patch"],
                          axis="Z", smooth=False)
                n_ties += 1
    log(f"  detail  : {n_lines} formwork lines, {n_ties} tie-hole patches "
        f"on {len(pier_columns)} HIGH-band columns")
    return n_lines + n_ties


def honeycombing(pier_columns, radius_fn, coll, mats, rnd, log=print):
    """Small irregular surface bumps near the base of each HIGH-band pier
    column -- reads as honeycombing/blowholes under grazing light."""
    n = 0
    for ob in pier_columns:
        zs = sorted(v.co.z for v in ob.data.vertices)
        z0 = zs[0]
        xs = sorted(set(round(v.co.x, 3) for v in ob.data.vertices))
        ys = sorted(set(round(v.co.y, 3) for v in ob.data.vertices))
        cx, cy = (xs[0] + xs[-1]) / 2.0, (ys[0] + ys[-1]) / 2.0
        r = radius_fn(ob)
        if r <= 0:
            continue
        for k in range(10):
            a = rnd.uniform(0, 2 * math.pi)
            z = z0 + rnd.uniform(0.2, 1.4)
            s = rnd.uniform(0.02, 0.05)
            px = cx + r * 1.0 * math.cos(a)
            py = cy + r * 1.0 * math.sin(a)
            ob2 = ML.cylinder(f"_MD_{ob.name}_HONEYCOMB_{k:02d}", s, s * 0.6,
                             (px, py, z), 6, coll, mats["concrete_low"],
                             axis="X" if abs(math.cos(a)) > abs(math.sin(a))
                             else "Y", smooth=False)
            ML.displace_random(ob2, s * 0.4, seed=int(z * 1000) + k)
            n += 1
    log(f"  detail  : {n} honeycomb/blowhole bumps near pier bases")
    return n


def chamfer_caps(pier_caps, coll, mats, log=print):
    """Small angled chamfer strips along each HIGH-band pier cap's four
    vertical arrises -- 'chamfered arrises', additive, no boolean."""
    n = 0
    for ob in pier_caps:
        xs = sorted(set(round(v.co.x, 3) for v in ob.data.vertices))
        ys = sorted(set(round(v.co.y, 3) for v in ob.data.vertices))
        zs = sorted(set(round(v.co.z, 3) for v in ob.data.vertices))
        if len(xs) < 2 or len(ys) < 2 or len(zs) < 2:
            continue
        x0, x1, y0, y1, z0, z1 = xs[0], xs[-1], ys[0], ys[-1], zs[0], zs[-1]
        c = 0.05
        for cx, cy, nm in ((x0, y0, "SW"), (x0, y1, "NW"),
                          (x1, y0, "SE"), (x1, y1, "NE")):
            v = [(cx, cy, z0), (cx, cy, z1),
                 (cx + (c if cx == x0 else -c), cy, z1),
                 (cx + (c if cx == x0 else -c), cy, z0)]
            ML.mesh_obj(f"_MD_{ob.name}_CHAMFER_{nm}", v, [(0, 1, 2, 3)], coll,
                       mats["concrete_pier"])
            n += 1
    log(f"  detail  : {n} chamfer strips on {len(pier_caps)} HIGH-band pier caps")
    return n


def parapet_posts(bridge_length, deck_top_z_fn, deck_width, parapet_thick,
                  coll, mats, spacing=3.5, log=print):
    """Thin vertical ribs on the parapet's outer face at regular intervals
    -- '360 m of one smooth extrusion' was the complaint. Purely additive."""
    n = 0
    x = spacing / 2.0
    half_w = deck_width / 2.0
    while x < bridge_length - spacing / 2.0:
        z = deck_top_z_fn(x)
        for s, nm in ((1, "L"), (-1, "R")):
            ML.box(f"_MD_BR_PARAPET_POST_{int(x*10):05d}_{nm}",
                  (0.10, 0.06, 0.85),
                  (x, s * (half_w - parapet_thick / 2.0), z + 0.55),
                  coll, mats["concrete_parapet"])
            n += 1
        x += spacing
    log(f"  detail  : {n} parapet posts at {spacing:.1f} m spacing")
    return n


def crack_relief(records, dmg_module, log=print):
    """For every CRACK_* record with width_mm > 3 mm, cut a shallow groove
    into the host along the SAME frame the decal already uses (read off the
    decal object's own matrix_world, not recomputed) -- so under the low
    raking sun a wide crack casts a real shadow. Reuses damage.py's own
    `carve()`, the same boolean pathway the hero defect and the random
    spall population already go through -- not a second implementation.
    Below 3 mm stays a decal, per the working rule against modelling a
    crack finer than any sensor could verify."""
    import bpy
    from mathutils import Vector, Matrix

    cutters = []
    n_candidates = 0
    for r in records:
        # road records key the crack family off "type" (CRACK_HAIRLINE, ...);
        # metro records key it off "base_type" (metro's own "type" is e.g.
        # WEB_SHEAR_CRACK) -- check whichever is present.
        family = r.get("base_type", r["type"])
        if not family.startswith("CRACK_"):
            continue
        if r.get("width_mm", 0.0) <= 3.0:
            continue
        ob = bpy.data.objects.get(r["defect_id"])
        host = bpy.data.objects.get(r["host_object"])
        if ob is None or host is None:
            continue
        n_candidates += 1
        m = ob.matrix_world
        L = r.get("length_m", 0.6)
        W = max(0.02, r.get("width_mm", 3.0) / 1000.0 * 6.0)
        depth = 0.006
        hl, hw = L / 2.0, W / 2.0
        # cutter in the decal's own local frame -- a thin box straddling
        # the surface, well into the host (top proud, bottom into it)
        cv = [(-hl, -hw, -depth * 1.4), (hl, -hw, -depth * 1.4),
             (hl, hw, -depth * 1.4), (-hl, hw, -depth * 1.4),
             (-hl, -hw, 0.03), (hl, -hw, 0.03),
             (hl, hw, 0.03), (-hl, hw, 0.03)]
        cf = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
        inv = host.matrix_world.inverted()
        local = [tuple(inv @ (m @ Vector(v))) for v in cv]
        cutters.append((host, local, cf, None))

    if not cutters:
        log("  detail  : 0 cracks over 3 mm -- no relief geometry needed")
        return {"candidates": n_candidates, "cut": 0}
    stats = dmg_module.carve(cutters, log)
    log(f"  detail  : crack relief cut into {len(cutters)} of {n_candidates} "
        f"cracks over 3 mm")
    return {"candidates": n_candidates, "cut": stats.get("cut", 0)}


def _x_centre(ob):
    xs = [v.co.x for v in ob.data.vertices]
    return (min(xs) + max(xs)) / 2.0


def _is_high_pier(ob, tol=1.0):
    cx = _x_centre(ob)
    return any(abs(cx - hx) < tol for hx in HIGH_PIER_X)


def build(colls, mats, params, log=print):
    """Top-level entry for the non-crack detail (formwork, tie-holes,
    honeycombing, chamfers, parapet posts), called from build_final.py
    right after bridge+metro exist. Crack relief is a separate function
    below (`crack_relief`), called after damage.build() once records exist."""
    rnd = random.Random(params.SEED + 4242)
    coll = colls["DETAILS"]

    def road_radius(ob):
        return params.PIER_COL_D / 2.0

    def metro_radius(ob):
        return 2.0 / 2.0     # MB.PIER_D, duplicated as a literal -- metro.py
                              # is not imported here to keep this module
                              # decoupled from the monkeypatch order

    road_cols = [o for o in _column_objects(("BR_",)) if _is_high_pier(o)]
    metro_cols = [o for o in _column_objects(("MB_",)) if _is_high_pier(o)]
    road_caps = [o for o in _cap_objects(("BR_",)) if _is_high_pier(o)]

    n = 0
    n += formwork_and_ties(road_cols, road_radius, coll, mats, rnd, log)
    n += formwork_and_ties(metro_cols, metro_radius, coll, mats, rnd, log)
    n += honeycombing(road_cols, road_radius, coll, mats, rnd, log)
    n += honeycombing(metro_cols, metro_radius, coll, mats, rnd, log)
    n += chamfer_caps(road_caps, coll, mats, log)
    n += parapet_posts(params.BRIDGE_LENGTH, params.deck_top_z,
                       params.DECK_WIDTH, params.PARAPET_THICK,
                       coll, mats, log=log)
    return {"objects": n}

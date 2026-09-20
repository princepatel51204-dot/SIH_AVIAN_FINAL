"""Indian-style metropolitan context.

A believable generic Indian metro, not a copy of any real city: dense low and
mid-rise blocks close in, taller commercial towers in the far field, flat roofs
crowded with water tanks and stair cores, and a scattering of informal
low-rise. Trees along the streets, vehicles on the roads.

The city exists to give the bridge context and to give a UAV a realistic
horizon, occlusion field and GPS-multipath environment. It deliberately does
NOT out-detail the bridge: everything here is instanced low-poly, and nothing
in it carries a defect.
"""
from __future__ import annotations
import math
import random
import bpy

import params as P
import meshlib as ML
import terrain as TR


# ---------------------------------------------------------------------------
def _building_proto(name, w, d, h, coll, mat, roof_mat, style, rnd):
    """One reusable building shell.

    Styles:
      SLAB    plain rectangular block, the default mid-rise
      SETBACK stepped, taller commercial
      LOWRISE small footprint, shallow, informal
    """
    parts = []
    if style == "SETBACK":
        levels = 3
        for i in range(levels):
            t = i / levels
            hw = w * (1.0 - 0.22 * t)
            hd = d * (1.0 - 0.22 * t)
            hh = h / levels
            parts.append(ML.box(f"{name}_L{i}", (hw, hd, hh),
                                (0, 0, hh * (i + 0.5)), coll, mat))
        # The roof sits on the TOP level, not the base footprint. Using the
        # base width here put a full-width slab on top of a tapered tower,
        # which reads from the air as an inverted pyramid.
        tw = w * (1.0 - 0.22 * (levels - 1) / levels)
        td = d * (1.0 - 0.22 * (levels - 1) / levels)
    else:
        parts.append(ML.box(f"{name}_B", (w, d, h), (0, 0, h / 2),
                            coll, mat))
        tw, td = w, d

    # flat roof parapet
    parts.append(ML.box(f"{name}_PAR", (tw, td, 0.9), (0, 0, h + 0.45),
                        coll, roof_mat))
    # roof clutter: water tanks and a stair core. Ubiquitous, and it is what
    # makes an Indian skyline read correctly from the air.
    if style != "LOWRISE":
        parts.append(ML.box(f"{name}_CORE",
                            (tw * 0.22, td * 0.22, 2.6),
                            (tw * 0.2, -td * 0.2, h + 1.3), coll, roof_mat))
        for k in range(rnd.randint(1, 3)):
            r = rnd.uniform(0.55, 0.95)
            parts.append(ML.cylinder(
                f"{name}_TANK{k}", r, r * 1.5,
                (rnd.uniform(-tw * 0.3, tw * 0.3),
                 rnd.uniform(-td * 0.3, td * 0.3), h + r * 0.75 + 0.4),
                8, coll, roof_mat))

    ob = ML.join_objects(parts, name, coll)
    if ob is not None:
        ob.data.materials.clear()
        ob.data.materials.append(mat)
    return ob


def build(colls, mats, log=print):
    rnd = random.Random(P.SEED + 23)
    CB = colls["BUILDINGS"]
    CV = colls["VEHICLES"]
    CL = colls["STREET_LIGHTS"]
    CG = colls["VEGETATION"]

    bldg_mats = [mats["bldg_a"], mats["bldg_b"], mats["bldg_c"],
                 mats["bldg_d"]]

    # ---- prototypes -------------------------------------------------------
    protos = []
    for i in range(14):
        if i < 8:
            style = "SLAB"
            w = rnd.uniform(16, 30)
            d = rnd.uniform(14, 26)
            h = rnd.uniform(*P.CITY_MIDRISE_H)
        elif i < 11:
            style = "SETBACK"
            w = rnd.uniform(24, 38)
            d = rnd.uniform(22, 34)
            h = rnd.uniform(*P.CITY_HIGHRISE_H)
        else:
            style = "LOWRISE"
            w = rnd.uniform(8, 15)
            d = rnd.uniform(7, 13)
            h = rnd.uniform(4.5, 10.0)
        m = bldg_mats[i % len(bldg_mats)]
        ob = _building_proto(f"_BLD_PROTO_{i:02d}", w, d, h, CB, m,
                             mats["concrete_low"], style, rnd)
        ob.hide_render = True
        ob.hide_viewport = True
        protos.append((ob, w, d, h, style))

    # ---- plot the city ----------------------------------------------------
    n_b = 0
    n_high = 0
    x = -300.0
    while x < P.BRIDGE_LENGTH + 300.0:
        for s in (1, -1):
            for k in range(9):
                y = s * (P.CITY_SETBACK_Y + 26.0 + k * P.CITY_BLOCK)
                if abs(y) > P.CITY_BAND_Y:
                    continue
                # no city in the river corridor
                if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 90:
                    continue
                # ground_z is 0.0 on all dry land, so testing "> 0.2" here
                # rejected the entire city. Test against the water line.
                if TR.is_water(x, y):
                    continue
                if rnd.random() > P.CITY_DENSITY:
                    continue

                far = abs(y) > 380.0
                want_high = far and rnd.random() < P.CITY_HIGHRISE_FRACTION * 3
                cands = [p for p in protos
                         if (p[4] == "SETBACK") == bool(want_high)]
                if not cands:
                    cands = protos
                proto, w, d, h, style = rnd.choice(cands)

                jx = x + rnd.uniform(-16, 16)
                jy = y + rnd.uniform(-14, 14)
                gz = TR.height(jx, jy)
                sc = rnd.uniform(0.82, 1.28)
                rot = rnd.choice([0.0, math.pi / 2]) + \
                    rnd.uniform(-0.05, 0.05)
                n_b += 1
                if style == "SETBACK":
                    n_high += 1
                ML.link_dup(proto, f"CITY_BLD_{n_b:04d}", (jx, jy, gz),
                            (0, 0, rot), (sc, sc, sc * rnd.uniform(0.8, 1.3)),
                            CB)
        x += P.CITY_BLOCK

    # ---- trees ------------------------------------------------------------
    trunk = ML.cylinder("_TREE_TRUNK", 0.18, 3.2, (0, 0, 1.6), 6, CG,
                        mats["veg_dry"])
    canopy = ML.cylinder("_TREE_CANOPY", 2.4, 3.4, (0, 0, 4.6), 7, CG,
                         mats["veg"], smooth=False)
    ML.displace_random(canopy, 0.45, seed=P.SEED + 3)
    for o in (trunk, canopy):
        o.hide_render = True
        o.hide_viewport = True

    n_t = 0
    for i in range(P.CITY_TREE_COUNT):
        x = rnd.uniform(-300.0, P.BRIDGE_LENGTH + 300.0)
        y = rnd.choice([1, -1]) * rnd.uniform(P.CITY_SETBACK_Y - 22.0,
                                              P.CITY_BAND_Y)
        if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 60:
            continue
        if TR.is_water(x, y):
            continue
        gz = TR.height(x, y)
        s = rnd.uniform(0.65, 1.45)
        for proto, nm in ((trunk, "T"), (canopy, "C")):
            ML.link_dup(proto, f"CITY_TREE_{n_t+1:04d}_{nm}", (x, y, gz),
                        (0, 0, rnd.uniform(0, 6.28)), (s, s, s), CG)
        n_t += 1

    # ---- street lights in the city ---------------------------------------
    lp = ML.cylinder("_CITY_LAMP_PROTO", 0.10, 7.5, (0, 0, 3.75), 6, CL,
                     mats["steel_dark"])
    lh = ML.box("_CITY_LAMP_HEAD", (0.9, 0.22, 0.12), (0.45, 0, 7.5), CL,
                mats["steel_dark"])
    for o in (lp, lh):
        o.hide_render = True
        o.hide_viewport = True
    n_l = 0
    for k in range(1, 9):
        for s in (1, -1):
            y = s * (P.CITY_SETBACK_Y + k * P.CITY_BLOCK)
            if abs(y) > P.CITY_BAND_Y:
                continue
            x = -300.0
            while x < P.BRIDGE_LENGTH + 300.0:
                if abs(x - P.RIVER_CENTRE_X) > P.RIVER_WIDTH / 2 + 60 \
                        and not TR.is_water(x, y):
                    for proto, nm in ((lp, "P"), (lh, "H")):
                        ML.link_dup(proto, f"CITY_LAMP_{n_l+1:04d}_{nm}",
                                    (x, y, TR.height(x, y)),
                                    (0, 0, 0), (1, 1, 1), CL)
                    n_l += 1
                x += 68.0

    # ---- vehicles ---------------------------------------------------------
    # Deliberately crude: a car is a box with a cabin box. At the ranges a
    # UAV sees them from, more geometry buys nothing.
    vprotos = []
    specs = [("CAR", 4.3, 1.8, 1.45, 0.55), ("CAR2", 4.0, 1.7, 1.50, 0.55),
             ("BUS", 11.5, 2.6, 3.20, 0.85), ("TRUCK", 8.6, 2.5, 3.00, 0.60),
             ("AUTO", 2.7, 1.4, 1.70, 0.70)]
    vm = [mats["vehicle_a"], mats["vehicle_b"], mats["vehicle_c"],
          mats["vehicle_d"]]
    for i, (nm, L, W, H, cab) in enumerate(specs):
        parts = [ML.box(f"_VEH_{nm}_B", (L, W, H * 0.55),
                        (0, 0, H * 0.275 + 0.30), CV, vm[i % len(vm)])]
        parts.append(ML.box(f"_VEH_{nm}_C", (L * cab, W * 0.92, H * 0.45),
                            (-L * 0.06, 0, H * 0.55 + 0.30 + H * 0.225),
                            CV, vm[i % len(vm)]))
        for sx in (1, -1):
            for sy in (1, -1):
                parts.append(ML.cylinder(
                    f"_VEH_{nm}_W{sx}{sy}", 0.34, 0.22,
                    (sx * L * 0.33, sy * W * 0.48, 0.34), 8, CV,
                    mats["steel_dark"], axis="Y"))
        ob = ML.join_objects(parts, f"_VEH_PROTO_{nm}", CV)
        ob.hide_render = True
        ob.hide_viewport = True
        vprotos.append(ob)

    n_v = 0
    # on the bridge deck
    for i in range(P.VEHICLE_COUNT_BRIDGE):
        x = rnd.uniform(40.0, P.BRIDGE_LENGTH - 40.0)
        s = rnd.choice([1, -1])
        lane = rnd.choice([-1, 0, 1])
        y = s * (P.MEDIAN_WIDTH / 2 + 1.8 + lane * 3.5 + 1.75)
        z = P.deck_top_z(x)
        pr = rnd.choice(vprotos)
        ML.link_dup(pr, f"CITY_VEH_BR_{i+1:03d}", (x, y, z),
                    (0, 0, 0.0 if s > 0 else math.pi), (1, 1, 1), CV)
        n_v += 1
    # on the surface roads
    for i in range(P.VEHICLE_COUNT_CITY):
        if rnd.random() < 0.55:
            x = rnd.uniform(360.0, 4140.0)
            y = rnd.choice([1, -1]) * (19.0 + rnd.choice([-3.5, 0, 3.5]))
            rot = 0.0 if y > 0 else math.pi
        else:
            k = rnd.randint(1, 8)
            y = rnd.choice([1, -1]) * (P.CITY_SETBACK_Y + k * P.CITY_BLOCK)
            x = rnd.uniform(-250.0, P.BRIDGE_LENGTH + 250.0)
            rot = 0.0
        if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 60:
            continue
        if TR.is_water(x, y):
            continue
        pr = rnd.choice(vprotos)
        ML.link_dup(pr, f"CITY_VEH_GR_{i+1:03d}", (x, y, TR.height(x, y) + 0.08),
                    (0, 0, rot), (1, 1, 1), CV)
        n_v += 1

    log(f"  city    : {n_b} buildings ({n_high} high-rise), {n_t} trees, "
        f"{n_l} street lights, {n_v} vehicles")
    return {"buildings": n_b, "highrise": n_high, "trees": n_t,
            "lamps": n_l, "vehicles": n_v}

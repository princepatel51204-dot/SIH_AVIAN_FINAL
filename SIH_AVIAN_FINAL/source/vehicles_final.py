"""SIH_AVIAN_FINAL -- realism pass: real vehicle silhouettes.

Replaces the plain white boxes with a car (stepped bonnet/cabin/boot side
profile as one prism, raked windscreen, four wheels, glass), a truck (cab +
separate box body, six wheels), a bus (12 m, window band), and two or three
auto-rickshaws. Each vehicle is built in a LOCAL frame (+X = direction of
travel, ground at Z=0) as several small objects, then placed with
`ob.location`/`ob.rotation_euler` -- NOT baked into world-space vertices --
specifically so it can be rotated to face either direction of travel.
meshlib's box()/cylinder()/prism() all bake `centre` into the vertices and
leave the object's own transform at identity, which is fine for axis-aligned
structure but wrong for anything that needs to face a direction.
"""
from __future__ import annotations
import math
import random

import meshlib as ML

WHEEL_R = 0.30
WHEEL_W = 0.22


def _place(ob, loc, yaw_deg):
    ob.location = loc
    ob.rotation_euler = (0.0, 0.0, math.radians(yaw_deg))
    return ob


def _wheel(name, loc_local, coll, mats):
    return ML.cylinder(name, WHEEL_R, WHEEL_W, loc_local, 12, coll,
                       mats["vehicle_tyre"], axis="Y")


def _glass_quad(name, x0, z0, x1, z1, half_w, coll, mats):
    v = [(x0, -half_w, z0), (x0, half_w, z0),
         (x1, half_w, z1), (x1, -half_w, z1)]
    return ML.mesh_obj(name, v, [(0, 1, 2, 3)], coll, mats["glass"])


# ===========================================================================
# CAR
# ===========================================================================
_CAR_PROFILE = [
    (2.15, 0.30), (2.15, 0.55), (1.55, 0.92), (1.05, 0.95), (0.45, 1.45),
    (-0.85, 1.42), (-1.35, 0.90), (-1.70, 0.90), (-2.15, 0.55), (-2.15, 0.30),
]


def build_car(tag, coll, mats):
    parts = []
    body = ML.prism(f"VEH_CAR_{tag}_BODY", _CAR_PROFILE, 1.70, "Y",
                    (0, 0, 0), coll, mats["vehicle_body"])
    parts.append(body)
    parts.append(_glass_quad(f"VEH_CAR_{tag}_WINDSCREEN", 1.05, 0.95,
                             0.45, 1.45, 0.80, coll, mats))
    parts.append(_glass_quad(f"VEH_CAR_{tag}_REARGLASS", -0.85, 1.42,
                             -1.35, 0.90, 0.80, coll, mats))
    for s, nm in ((1, "L"), (-1, "R")):
        parts.append(ML.mesh_obj(
            f"VEH_CAR_{tag}_SIDE_{nm}",
            [(0.35, s * 0.845, 1.10), (-0.75, s * 0.845, 1.10),
             (-0.75, s * 0.845, 1.40), (0.35, s * 0.845, 1.40)],
            [(0, 1, 2, 3)], coll, mats["glass"]))
    for x, s, nm in ((1.3, 1, "FL"), (1.3, -1, "FR"),
                    (-1.3, 1, "RL"), (-1.3, -1, "RR")):
        parts.append(_wheel(f"VEH_CAR_{tag}_WHEEL_{nm}",
                            (x, s * 0.78, WHEEL_R), coll, mats))
    return parts


# ===========================================================================
# TRUCK -- cab + separate box body, six wheels
# ===========================================================================
def build_truck(tag, coll, mats):
    parts = []
    cab = ML.box(f"VEH_TRUCK_{tag}_CAB", (1.8, 2.2, 1.9), (2.7, 0, 1.35),
                coll, mats["vehicle_body"])
    parts.append(cab)
    parts.append(_glass_quad(f"VEH_TRUCK_{tag}_WINDSCREEN", 1.85, 1.55,
                             1.55, 2.25, 1.05, coll, mats))
    body = ML.box(f"VEH_TRUCK_{tag}_BODY", (5.6, 2.4, 2.1), (-0.6, 0, 1.55),
                 coll, mats["vehicle_trim"])
    parts.append(body)
    for x, s, nm in ((2.4, 1, "F1L"), (2.4, -1, "F1R"),
                    (-1.2, 1, "R1L"), (-1.2, -1, "R1R"),
                    (-2.6, 1, "R2L"), (-2.6, -1, "R2R")):
        parts.append(_wheel(f"VEH_TRUCK_{tag}_WHEEL_{nm}",
                            (x, s * 1.05, WHEEL_R * 1.15), coll, mats))
    return parts


# ===========================================================================
# BUS -- 12 m, window band along both sides, six wheels
# ===========================================================================
def build_bus(tag, coll, mats):
    parts = []
    body = ML.box(f"VEH_BUS_{tag}_BODY", (12.0, 2.5, 2.9), (0, 0, 1.55),
                 coll, mats["vehicle_trim"])
    parts.append(body)
    parts.append(ML.box(f"VEH_BUS_{tag}_ROOF", (11.6, 2.4, 0.25),
                        (0, 0, 3.0), coll, mats["vehicle_trim"]))
    for s, nm in ((1, "L"), (-1, "R")):
        parts.append(ML.mesh_obj(
            f"VEH_BUS_{tag}_WINDOWBAND_{nm}",
            [(5.6, s * 1.255, 2.0), (-5.6, s * 1.255, 2.0),
             (-5.6, s * 1.255, 2.55), (5.6, s * 1.255, 2.55)],
            [(0, 1, 2, 3)], coll, mats["glass"]))
    parts.append(_glass_quad(f"VEH_BUS_{tag}_WINDSCREEN", 5.85, 1.4,
                             5.85, 2.55, 1.0, coll, mats))
    for x, s, nm in ((4.6, 1, "F_L"), (4.6, -1, "F_R"),
                    (-3.6, 1, "R1L"), (-3.6, -1, "R1R"),
                    (-4.9, 1, "R2L"), (-4.9, -1, "R2R")):
        parts.append(_wheel(f"VEH_BUS_{tag}_WHEEL_{nm}",
                            (x, s * 1.13, WHEEL_R * 1.2), coll, mats))
    return parts


# ===========================================================================
# AUTO-RICKSHAW -- single front wheel, two rear, canvas hood
# ===========================================================================
_RICK_PROFILE = [
    (1.15, 0.20), (1.15, 0.55), (0.75, 0.95), (-0.30, 1.35),
    (-1.10, 1.35), (-1.10, 0.20),
]


def build_rickshaw(tag, coll, mats):
    parts = []
    body = ML.prism(f"VEH_RICK_{tag}_BODY", _RICK_PROFILE, 1.30, "Y",
                    (0, 0, 0), coll, mats["rickshaw_body"])
    parts.append(body)
    hood = ML.box(f"VEH_RICK_{tag}_HOOD", (1.9, 1.35, 0.06), (-0.30, 0, 1.35),
                 coll, mats["rickshaw_hood"])
    parts.append(hood)
    parts.append(_wheel(f"VEH_RICK_{tag}_WHEEL_FRONT",
                        (0.95, 0, WHEEL_R * 0.85), coll, mats))
    for s, nm in ((1, "RL"), (-1, "RR")):
        parts.append(_wheel(f"VEH_RICK_{tag}_WHEEL_{nm}",
                            (-0.85, s * 0.62, WHEEL_R * 0.85), coll, mats))
    return parts


# ===========================================================================
# ORCHESTRATION
# ===========================================================================
def build(colls, mats, params, log=print):
    """Places 13 cars, 2 trucks, 1 bus, 2 auto-rickshaws (18 vehicles total,
    matching SPEC.md's '~16' with the two extra rickshaws SPEC.md itself
    calls out as the fastest way to read as an Indian road)."""
    rnd = random.Random(params.SEED + 555)
    coll = colls["VEHICLES"]
    specs = (["car"] * 13) + ["truck", "truck", "bus", "rickshaw", "rickshaw"]
    rnd.shuffle(specs)

    lane_ys = (-5.25, -1.75, 1.75, 5.25)
    xs = [20.0 + i * (params.BRIDGE_LENGTH - 40.0) / max(1, len(specs) - 1)
          for i in range(len(specs))]
    rnd.shuffle(xs)

    counts = {"car": 0, "truck": 0, "bus": 0, "rickshaw": 0}
    n_obj = 0
    for i, kind in enumerate(specs):
        x = max(3.0, min(params.BRIDGE_LENGTH - 3.0,
                         xs[i] + rnd.uniform(-3, 3)))
        lane = lane_ys[i % 4]
        heading = 0.0 if lane < 0 else 180.0     # opposing lanes face away
        z = params.deck_top_z(x)
        counts[kind] += 1
        tag = f"{counts[kind]:03d}"
        if kind == "car":
            parts = build_car(tag, coll, mats)
        elif kind == "truck":
            parts = build_truck(tag, coll, mats)
        elif kind == "bus":
            parts = build_bus(tag, coll, mats)
        else:
            parts = build_rickshaw(tag, coll, mats)
        for p in parts:
            _place(p, (x, lane, z), heading)
            p["avi_kind"] = "vehicle"
            p["avi_vehicle_type"] = kind
        n_obj += len(parts)

    log(f"  traffic : {sum(counts.values())} vehicles -> {counts}, "
        f"{n_obj} objects")
    return {"vehicles": dict(counts), "objects": n_obj}

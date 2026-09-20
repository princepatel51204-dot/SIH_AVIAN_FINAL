"""Dynamic objects and temporary obstacles, in switchable collections.

Everything built here is separated from structure on purpose. A UAV testbed
needs a clean answer to "is this thing part of the asset I am inspecting, or
is it something that happened to be there today", and the answer has to be
machine-readable, because the digital twin should contain the bridge and not
the scaffolding.

  AVI_DYNAMIC_VEHICLES      road traffic, on the deck and the surface roads
  AVI_DYNAMIC_BOATS         river craft and moored floats
  AVI_DYNAMIC_PEDESTRIANS   people, as coarse simulation proxies
  AVI_DYNAMIC_OBSTACLES     scaffolding, platforms, crane, nets, barriers

The obstacles collection is OFF in every preset except DYNAMIC_OBSTACLE. It
sits inside the research zone and deliberately intrudes into inspection
airspace: its whole function is to be an obstacle the planner has not been
told about.

Pedestrians are three boxes. At the ranges a UAV sees them from, a detailed
human mesh buys nothing and costs a lot; what matters is that something
person-sized and person-warm is standing there, and the metadata says which.
"""
from __future__ import annotations
import math
import random

import bpy

import params as P
import meshlib as ML
import terrain as TR


# ---------------------------------------------------------------------------
def _tag(ob, kind, extra=None):
    d = {"avi_kind": kind,
         "avi_object_type": "DYNAMIC",
         "avi_object_id": ob.name,
         "avi_is_structure": False,
         "avi_in_digital_twin": False}
    if extra:
        d.update(extra)
    ML.set_custom(ob, d)
    return ob


def build_boats(coll, mats, log=print):
    """River craft: two working boats, a barge and moored floats."""
    rnd = random.Random(P.SEED + 411)
    half = P.RIVER_WIDTH / 2.0
    made = 0

    def hull(name, L, W, H, pos, rot, mat):
        parts = [ML.taper_box(f"{name}_H", (L, W), (L * 0.82, W * 0.6), H,
                              (0, 0, H / 2), coll, mat),
                 ML.box(f"{name}_C", (L * 0.30, W * 0.72, H * 0.9),
                        (-L * 0.12, 0, H * 1.35), coll, mat)]
        ob = ML.join_objects(parts, name, coll)
        ob.location = pos
        ob.rotation_euler = (0, 0, rot)
        return ob

    specs = [
        ("AVI_BOAT_WORK_01", 9.5, 2.8, 1.1,
         (P.RIVER_CENTRE_X - half + 40.0, 165.0), 0.25, "vehicle_b",
         "small working boat moored at the jetty"),
        ("AVI_BOAT_WORK_02", 8.0, 2.5, 1.0,
         (P.RIVER_CENTRE_X + 60.0, -240.0), 1.9, "vehicle_c",
         "river craft transiting the navigation channel"),
        ("AVI_BOAT_BARGE_01", 26.0, 7.0, 1.8,
         (P.RIVER_CENTRE_X - 20.0, 430.0), 0.05, "vehicle_a",
         "loaded barge under the main span"),
    ]
    for name, L, W, H, (bx, by), rot, mkey, note in specs:
        ob = hull(name, L, W, H, (bx, by, P.RIVER_WATER_Z - 0.25), rot,
                  mats[mkey])
        _tag(ob, "boat", {"avi_dynamic_class": "BOAT",
                          "avi_length_m": L, "avi_beam_m": W,
                          "avi_note": note,
                          "avi_draft_m": 0.25})
        made += 1

    # small moored floats -- clutter a downward sensor has to reject
    proto = ML.cylinder("_FLOAT_PROTO", 0.55, 0.45, (0, 0, 0), 8, coll,
                        mats["vehicle_d"], smooth=False)
    proto.hide_render = True
    proto.hide_viewport = True
    for i in range(14):
        side = 1 if i % 2 == 0 else -1
        bx = P.RIVER_CENTRE_X + side * rnd.uniform(30.0, half - 25.0)
        by = rnd.uniform(-700.0, 700.0)
        ob = ML.link_dup(proto, f"AVI_FLOAT_{i+1:02d}",
                         (bx, by, P.RIVER_WATER_Z + 0.05),
                         (0, 0, rnd.uniform(0, 6.28)), (1, 1, 1), coll)
        _tag(ob, "float", {"avi_dynamic_class": "FLOATING_OBJECT"})
        made += 1

    log(f"  boats   : {made} river craft and floats")
    return made


def build_pedestrians(coll, mats, log=print):
    """Coarse person proxies on footways and near the service areas."""
    rnd = random.Random(P.SEED + 523)
    parts = [
        ML.cylinder("_PED_L", 0.16, 0.85, (0, 0, 0.43), 6, coll,
                    mats["vehicle_c"]),
        ML.box("_PED_T", (0.34, 0.22, 0.62), (0, 0, 1.17), coll,
               mats["vehicle_a"]),
        ML.cylinder("_PED_H", 0.11, 0.24, (0, 0, 1.60), 6, coll,
                    mats["vehicle_b"]),
    ]
    proto = ML.join_objects(parts, "_PED_PROTO", coll)
    proto.hide_render = True
    proto.hide_viewport = True

    made = 0
    # on the footways beside the surface highway, off the river
    for i in range(120):
        x = rnd.uniform(-200.0, P.BRIDGE_LENGTH + 200.0)
        if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 60:
            continue
        y = rnd.choice([1, -1]) * rnd.uniform(24.0, 30.0)
        if TR.is_water(x, y):
            continue
        s = rnd.uniform(0.90, 1.08)
        ob = ML.link_dup(proto, f"AVI_PED_{made+1:03d}",
                         (x, y, TR.height(x, y)),
                         (0, 0, rnd.uniform(0, 6.28)), (s, s, s), coll)
        _tag(ob, "pedestrian", {"avi_dynamic_class": "PEDESTRIAN",
                                "avi_height_m": round(1.72 * s, 2),
                                "avi_fidelity": "coarse proxy: three "
                                                "primitives, no articulation"})
        made += 1
    log(f"  people  : {made} pedestrian proxies")
    return made


def build_obstacles(coll, mats, log=print):
    """Temporary works inside the research zone. OFF unless asked for.

    Placed to intrude on the inspection airspace of SECTOR_B and SECTOR_E --
    the two sectors whose defined challenge is occlusion and confinement --
    so enabling this collection actually invalidates a plan rather than just
    adding scenery somewhere harmless.
    """
    made = 0
    x_b = P.RESEARCH_X0 + 1.5 * P.SECTOR_LENGTH
    x_e = P.RESEARCH_X0 + 4.5 * P.SECTOR_LENGTH
    half_w = P.DECK_WIDTH / 2.0

    # ---- scaffolding tower against a pier, SECTOR_B ---------------------
    ps = [x for x, k in P.pier_stations()
          if abs(x - x_b) < P.SECTOR_LENGTH / 2]
    px = ps[len(ps) // 2] if ps else x_b
    gz = P.ground_z(px, 0.0)
    top = P.soffit_z(px) - 1.2
    lifts = max(2, int((top - gz) / 2.0))
    parts = []
    for lv in range(lifts + 1):
        z = gz + lv * (top - gz) / lifts
        parts.append(ML.box(f"_SC_D{lv}", (3.4, 3.4, 0.06), (0, 0, z),
                            coll, mats["steel"]))
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(ML.cylinder(f"_SC_P{sx}{sy}", 0.045, top - gz,
                                     (sx * 1.6, sy * 1.6,
                                      gz + (top - gz) / 2), 6, coll,
                                     mats["steel"]))
    sc_ob = ML.join_objects(parts, "AVI_OBST_SCAFFOLD_TOWER", coll)
    sc_ob.location = (px + 3.2, half_w - 4.0, 0.0)
    _tag(sc_ob, "scaffolding", {
        "avi_dynamic_class": "TEMPORARY_STRUCTURE",
        "avi_obstacle_type": "SCAFFOLD_TOWER",
        "avi_sector": P.sector_at(px) or "SECTOR_B",
        "avi_height_m": round(top - gz, 1),
        "avi_intrudes_airspace": "AVI_AIRSPACE_INSPECTION",
        "avi_note": "not on the digital twin; a planner working from the "
                    "twin alone will fly into this"})
    made += 1

    # ---- suspended maintenance platform under the deck, SECTOR_E --------
    sz = P.soffit_z(x_e) - 1.6
    parts = [ML.box("_MP_DECK", (6.0, 2.2, 0.10), (0, 0, 0), coll,
                    mats["steel"]),
             ML.box("_MP_RAIL_A", (6.0, 0.06, 1.0), (0, 1.05, 0.55), coll,
                    mats["steel"]),
             ML.box("_MP_RAIL_B", (6.0, 0.06, 1.0), (0, -1.05, 0.55), coll,
                    mats["steel"])]
    for sx in (-1, 1):
        parts.append(ML.cylinder(f"_MP_H{sx}", 0.02, 1.5,
                                 (sx * 2.6, 0, 0.80), 5, coll,
                                 mats["steel_dark"]))
    mp = ML.join_objects(parts, "AVI_OBST_MAINT_PLATFORM", coll)
    mp.location = (x_e, -half_w + 3.0, sz)
    _tag(mp, "maintenance_platform", {
        "avi_dynamic_class": "TEMPORARY_STRUCTURE",
        "avi_obstacle_type": "SUSPENDED_PLATFORM",
        "avi_sector": P.sector_at(x_e) or "SECTOR_E",
        "avi_intrudes_airspace": "AVI_AIRSPACE_UNDERBRIDGE",
        "avi_note": "hangs INSIDE the under-deck working volume"})
    made += 1

    # ---- safety netting under an adjacent bay --------------------------
    net = ML.box("AVI_OBST_SAFETY_NET", (18.0, 9.0, 0.05),
                 (x_e + 26.0, 0.0, P.soffit_z(x_e + 26.0) - 2.4),
                 coll, mats["veg_dry"])
    _tag(net, "safety_net", {
        "avi_dynamic_class": "TEMPORARY_STRUCTURE",
        "avi_obstacle_type": "SAFETY_NET",
        "avi_sector": P.sector_at(x_e + 26.0) or "SECTOR_E",
        "avi_note": "occludes the soffit it hangs beneath; a defect above "
                    "it is unobservable while it is in place"})
    made += 1

    # ---- mobile crane on the bank --------------------------------------
    cx = P.RESEARCH_X1 - 40.0
    cy = -half_w - 34.0
    cz = TR.height(cx, cy)
    parts = [ML.box("_CR_BASE", (7.0, 3.2, 1.1), (0, 0, 0.55), coll,
                    mats["vehicle_a"]),
             ML.box("_CR_CAB", (2.6, 2.6, 2.2), (-1.6, 0, 2.2), coll,
                    mats["vehicle_a"])]
    parts.append(ML.box("_CR_BOOM", (26.0, 0.9, 0.9), (9.0, 0, 9.5), coll,
                        mats["steel"]))
    cr = ML.join_objects(parts, "AVI_OBST_MOBILE_CRANE", coll)
    cr.location = (cx, cy, cz)
    cr.rotation_euler = (0, math.radians(-22.0), math.radians(90.0))
    _tag(cr, "crane", {
        "avi_dynamic_class": "TEMPORARY_STRUCTURE",
        "avi_obstacle_type": "MOBILE_CRANE",
        "avi_sector": P.sector_at(cx) or "SECTOR_F",
        "avi_boom_length_m": 26.0,
        "avi_intrudes_airspace": "AVI_AIRSPACE_SAFE",
        "avi_note": "the boom reaches into the transit band, which is the "
                    "one volume a planner is entitled to assume is empty"})
    made += 1

    # ---- ground works barriers -----------------------------------------
    bproto = ML.box("_BAR_PROTO", (2.0, 0.35, 1.0), (0, 0, 0.5), coll,
                    mats["paint_yellow"])
    bproto.hide_render = True
    bproto.hide_viewport = True
    for i in range(10):
        bx = cx - 24.0 + i * 2.3
        ob = ML.link_dup(bproto, f"AVI_OBST_BARRIER_{i+1:02d}",
                         (bx, cy + 7.0, TR.height(bx, cy + 7.0)),
                         (0, 0, 0), (1, 1, 1), coll)
        _tag(ob, "barrier", {"avi_dynamic_class": "TEMPORARY_STRUCTURE",
                             "avi_obstacle_type": "WORKS_BARRIER"})
        made += 1

    log(f"  obstacle: {made} temporary works objects (off by default)")
    return made


def build(colls, mats, log=print):
    n_b = build_boats(colls["AVI_DYNAMIC_BOATS"], mats, log)
    n_p = build_pedestrians(colls["AVI_DYNAMIC_PEDESTRIANS"], mats, log)
    n_o = build_obstacles(colls["AVI_DYNAMIC_OBSTACLES"], mats, log)
    return {"boats": n_b, "pedestrians": n_p, "obstacles": n_o}

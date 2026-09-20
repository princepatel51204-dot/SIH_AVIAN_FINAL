"""REV-B secondary detail: bridge services and city variety.

GROUND-TRUTH SAFETY
-------------------
Every object built here is named with the `AVI_DET_` prefix, and damage.py's
host ray-cast skips that prefix. This is not cosmetic tidiness: the defect
positions in the ground truth are produced by firing a ray at the structure and
snapping the defect to whatever it hits. Adding a cable tray 200 mm under a
girder soffit would silently intercept that ray, move the defect onto the tray,
and rewrite 30-odd rows of a ground truth that is supposed to be frozen. The
prefix is the guarantee that cannot happen.

The detail is concentrated where it is useful: the bridge UNDERSIDE, which is
the primary inspection region and was the least furnished part of REV-A. Under
a real viaduct there is a great deal of secondary steelwork, and every piece of
it is something a UAV has to fly around and a perception system has to not
mistake for a defect.
"""
from __future__ import annotations
import math
import random

import bpy

import params as P
import meshlib as ML
import terrain as TR


def _tag(ob, kind, extra=None):
    d = {"avi_kind": kind,
         "avi_object_type": "SECONDARY_DETAIL",
         "avi_object_id": ob.name,
         "avi_is_structure": False,
         "avi_carries_defects": False,
         "avi_lod": "HIGH"}
    if extra:
        d.update(extra)
    ML.set_custom(ob, d)
    return ob


# ---------------------------------------------------------------------------
def build_bridge_services(coll, mats, log=print):
    """Cable trays, conduit, brackets, downpipes and access platforms.

    HIGH-detail band only. In the MED and LOW bands this is invisible at any
    range a camera will see them from and would cost 20k objects for nothing.
    """
    rnd = random.Random(P.SEED + 731)
    made = 0
    half_w = P.DECK_WIDTH / 2.0
    y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0

    ps = [x for x, k in P.pier_stations()
          if P.RESEARCH_X0 - 20 <= x <= P.RESEARCH_X1 + 20]

    # ---- prototypes, instanced along the corridor -----------------------
    tray_p = ML.box("_DET_TRAY_PROTO", (6.0, 0.34, 0.10), (0, 0, 0),
                    coll, mats["steel_dark"])
    brkt_p = ML.box("_DET_BRKT_PROTO", (0.06, 0.42, 0.30), (0, 0, 0),
                    coll, mats["steel"])
    cond_p = ML.cylinder("_DET_COND_PROTO", 0.055, 6.0, (0, 0, 0), 6,
                         coll, mats["steel_dark"], axis="X")
    for p in (tray_p, brkt_p, cond_p):
        p.hide_render = True
        p.hide_viewport = True

    for i in range(len(ps) - 1):
        x0, x1 = ps[i], ps[i + 1]
        xm = 0.5 * (x0 + x1)
        if P.detail_at(xm) != P.DETAIL_HIGH:
            continue
        span = x1 - x0
        depth = P.girder_depth_for_span(span)
        z_soffit = P.soffit_z(xm)
        z_gird_top = z_soffit + depth

        # cable tray runs under the deck on both cantilevers, in 6 m lengths
        n_seg = max(1, int(span / 6.0))
        for s in (1, -1):
            y = s * (half_w - 2.4)
            for k in range(n_seg):
                sx = x0 + (k + 0.5) * span / n_seg
                ob = ML.link_dup(
                    tray_p,
                    f"AVI_DET_CABLETRAY_{i+1:03d}_{k+1:02d}"
                    f"{'L' if s > 0 else 'R'}",
                    (sx, y, z_gird_top - 0.62), (0, 0, 0),
                    (span / n_seg / 6.0, 1, 1), coll)
                _tag(ob, "cable_tray",
                     {"avi_service": "power and signal cabling",
                      "avi_note": "a UAV has to fly under or around this; "
                                  "it also casts a hard shadow line across "
                                  "the soffit that is easy to mistake for a "
                                  "crack"})
                made += 1
            # supporting brackets every 2 m
            for k in range(max(1, int(span / 2.0))):
                sx = x0 + (k + 0.5) * 2.0
                if sx > x1:
                    break
                ob = ML.link_dup(
                    brkt_p,
                    f"AVI_DET_BRACKET_{i+1:03d}_{k+1:02d}"
                    f"{'L' if s > 0 else 'R'}",
                    (sx, y - s * 0.20, z_gird_top - 0.47), (0, 0, 0),
                    (1, 1, 1), coll)
                _tag(ob, "bracket", {"avi_service": "tray support"})
                made += 1

        # utility conduit pair running with the girders
        for s in (1, -1):
            y = s * (half_w - 3.1)
            n_c = max(1, int(span / 6.0))
            for k in range(n_c):
                sx = x0 + (k + 0.5) * span / n_c
                ob = ML.link_dup(
                    cond_p,
                    f"AVI_DET_CONDUIT_{i+1:03d}_{k+1:02d}"
                    f"{'L' if s > 0 else 'R'}",
                    (sx, y, z_gird_top - 0.80), (0, 0, 0),
                    (span / n_c / 6.0, 1, 1), coll)
                _tag(ob, "conduit", {"avi_service": "utility duct"})
                made += 1

        # drainage downpipes, dropping from the deck outlets toward ground
        n_d = max(1, int(span / P.DRAIN_SPACING))
        for d in range(n_d):
            dx = x0 + span * (d + 0.5) / n_d
            for s in (1, -1):
                y = s * (half_w - P.PARAPET_THICK - 0.30)
                drop = min(6.0, max(1.5, z_soffit
                                    - max(P.ground_z(dx, 0.0),
                                          P.RIVER_WATER_Z) - 1.0))
                ob = ML.cylinder(
                    f"AVI_DET_DOWNPIPE_{i+1:03d}_{d+1}"
                    f"{'L' if s > 0 else 'R'}",
                    P.DRAIN_PIPE_D / 2 * 0.9, drop,
                    (dx, y, z_soffit - drop / 2 + 0.2), 8, coll,
                    mats["steel_dark"])
                _tag(ob, "downpipe",
                     {"avi_service": "deck drainage",
                      "avi_note": "chronic leakage path; the concrete "
                                  "around its head is the most reliably "
                                  "deteriorated on the deck"})
                made += 1

    # ---- inspection access platforms on pier caps -----------------------
    for i, x in enumerate(ps):
        if P.detail_at(x) != P.DETAIL_HIGH:
            continue
        cap_top = P.soffit_z(x) - P.BEARING_H
        for s in (1, -1):
            ob = ML.box(f"AVI_DET_ACCESS_PLATFORM_{i+1:03d}"
                        f"{'L' if s > 0 else 'R'}",
                        (1.6, 2.4, 0.08),
                        (x + s * (P.PIER_CAP_W / 2 + 0.85), 0.0,
                         cap_top - 0.55), coll, mats["steel"])
            _tag(ob, "access_platform",
                 {"avi_service": "maintenance access to the bearing seats",
                  "avi_note": "permanent works, unlike the temporary "
                              "platform in AVI_DYNAMIC_OBSTACLES"})
            made += 1
            rail = ML.box(f"AVI_DET_ACCESS_RAIL_{i+1:03d}"
                          f"{'L' if s > 0 else 'R'}",
                          (1.6, 0.05, 0.95),
                          (x + s * (P.PIER_CAP_W / 2 + 0.85),
                           s * 1.15, cap_top - 0.08), coll,
                          mats["steel_dark"])
            _tag(rail, "access_rail")
            made += 1

    log(f"  services: {made} bridge service and access objects (HIGH band)")
    return made


# ---------------------------------------------------------------------------
def build_city_variety(coll, mats, log=print):
    """Parking structures, warehouses and industrial blocks.

    REV-A's city was towers and low-rise. Real metropolitan periphery is
    mostly neither: it is sheds, yards and multi-storey parking, and those
    have flat wide roofs, which changes the horizon a UAV sees and gives the
    far field a different silhouette from the core.
    """
    rnd = random.Random(P.SEED + 829)
    made = 0

    def deck_park(name, w, d, levels, pos):
        parts = []
        for lv in range(levels):
            parts.append(ML.box(f"{name}_L{lv}", (w, d, 0.30),
                                (0, 0, lv * 3.1 + 0.15), coll,
                                mats["concrete_low"]))
            for sx in (-1, 1):
                parts.append(ML.box(f"{name}_E{lv}{sx}",
                                    (w, 0.25, 1.05),
                                    (0, sx * d / 2, lv * 3.1 + 0.85),
                                    coll, mats["concrete_low"]))
        for sx in (-1, 1):
            for sy in (-1, 1):
                parts.append(ML.box(f"{name}_C{sx}{sy}", (0.5, 0.5,
                                                          levels * 3.1),
                                    (sx * (w / 2 - 0.6), sy * (d / 2 - 0.6),
                                     levels * 3.1 / 2), coll,
                                    mats["concrete_low"]))
        ob = ML.join_objects(parts, name, coll)
        ob.location = pos
        return ob

    def shed(name, w, d, h, pos, mat):
        parts = [ML.box(f"{name}_B", (w, d, h), (0, 0, h / 2), coll, mat),
                 ML.taper_box(f"{name}_R", (w, d), (w * 0.98, d * 0.5),
                              1.6, (0, 0, h + 0.8), coll,
                              mats["steel_dark"])]
        ob = ML.join_objects(parts, name, coll)
        ob.location = pos
        return ob

    # multi-storey parking near the corridor, both sides
    for i, (bx, by) in enumerate([(760.0, 168.0), (1180.0, -196.0),
                                  (3010.0, 182.0), (3620.0, -174.0)]):
        if TR.is_water(bx, by):
            continue
        ob = deck_park(f"AVI_DET_PARKING_{i+1:02d}",
                       rnd.uniform(42, 58), rnd.uniform(34, 46),
                       rnd.randint(4, 7), (bx, by, TR.height(bx, by)))
        _tag(ob, "parking_structure",
             {"avi_object_type": "CITY_STRUCTURE", "avi_lod": "MED",
              "avi_note": "open-deck parking: a large flat multi-level "
                          "void, and a genuinely difficult GNSS multipath "
                          "environment"})
        made += 1

    # warehouse / industrial belt in the far field
    wmats = [mats["bldg_b"], mats["bldg_c"], mats["bldg_d"]]
    for i in range(26):
        bx = rnd.uniform(-200.0, P.BRIDGE_LENGTH + 200.0)
        if abs(bx - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 120:
            continue
        by = rnd.choice([1, -1]) * rnd.uniform(560.0, 1080.0)
        if TR.is_water(bx, by):
            continue
        ob = shed(f"AVI_DET_WAREHOUSE_{i+1:02d}",
                  rnd.uniform(40, 78), rnd.uniform(26, 44),
                  rnd.uniform(7.5, 12.0), (bx, by, TR.height(bx, by)),
                  wmats[i % len(wmats)])
        _tag(ob, "warehouse",
             {"avi_object_type": "CITY_STRUCTURE", "avi_lod": "LOW"})
        made += 1

    # utility / substation blocks close in
    for i in range(8):
        bx = rnd.uniform(200.0, P.BRIDGE_LENGTH - 200.0)
        if abs(bx - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 90:
            continue
        by = rnd.choice([1, -1]) * rnd.uniform(96.0, 150.0)
        if TR.is_water(bx, by):
            continue
        ob = ML.box(f"AVI_DET_UTILITY_{i+1:02d}",
                    (rnd.uniform(10, 18), rnd.uniform(9, 14),
                     rnd.uniform(4.0, 6.5)),
                    (0, 0, 0), coll, mats["concrete_low"])
        gz = TR.height(bx, by)
        ob.location = (bx, by, gz + 2.6)
        _tag(ob, "utility_building",
             {"avi_object_type": "CITY_STRUCTURE", "avi_lod": "LOW"})
        made += 1

    log(f"  cityvar : {made} parking, warehouse and utility structures")
    return made


def build(colls, mats, log=print):
    n_s = build_bridge_services(colls["DETAILS"], mats, log)
    n_c = build_city_variety(colls["BUILDINGS"], mats, log)
    return {"bridge_services": n_s, "city_structures": n_c}

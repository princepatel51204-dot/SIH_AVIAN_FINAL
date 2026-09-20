"""The bridge: 4.5 km hybrid highway viaduct and river crossing.

Built span by span so that any one span can be edited, replaced or examined
independently. Detail follows params.detail_at(x):

  HIGH  research zone   real I-girders with tapered flanges, diaphragms,
                        bearings, chamfered pier caps, drainage, service duct
  MED   +/-500 m        simplified I-girders, plain pier caps
  LOW   remainder       a single closed deck box and simple piers, merged
                        into one object per section

That banding is the whole reason the corridor stays under a few million
triangles while the region the sensors actually work in is fully modelled.
"""
from __future__ import annotations
import math
import bpy

import params as P
import meshlib as ML


# ---------------------------------------------------------------------------
def girder_profile(depth, detail):
    """I-section in the (y, z) plane, origin at the TOP of the girder.

    The web tapers into both flanges via fillets in HIGH detail. Those fillets
    matter more than they look: the web-to-flange junction is where shear
    cracking shows up, so it is a surface the inspection has to be able to see
    properly.
    """
    tw = P.GIRDER_TOP_FLANGE_W / 2
    bw = P.GIRDER_BOT_FLANGE_W / 2
    ww = P.GIRDER_WEB_T / 2
    ft = P.GIRDER_FLANGE_T
    z0 = 0.0
    z1 = -ft
    z3 = -depth
    z2 = z3 + ft

    if detail == P.DETAIL_HIGH:
        fil = 0.16
        return [
            (-tw, z0), (tw, z0), (tw, z1), (ww + fil, z1 - fil * 0.6),
            (ww, z1 - fil), (ww, z2 + fil), (bw - 0.02, z2 + fil * 0.5),
            (bw, z2), (bw, z3), (-bw, z3), (-bw, z2),
            (-bw + 0.02, z2 + fil * 0.5), (-ww, z2 + fil), (-ww, z1 - fil),
            (-ww - fil, z1 - fil * 0.6), (-tw, z1),
        ]
    if detail == P.DETAIL_MED:
        return [(-tw, z0), (tw, z0), (tw, z1), (ww, z1), (ww, z2),
                (bw, z2), (bw, z3), (-bw, z3), (-bw, z2), (-ww, z2),
                (-ww, z1), (-tw, z1)]
    return [(-tw, z0), (tw, z0), (tw, z3), (-tw, z3)]


def parapet_profile():
    """New-Jersey style safety barrier, origin at deck top."""
    t = P.PARAPET_THICK
    h = P.PARAPET_HEIGHT
    return [(0.0, 0.0), (t, 0.0), (t, h), (t - 0.10, h),
            (t - 0.14, h * 0.55), (0.055, h * 0.30), (0.0, 0.22)]


# ---------------------------------------------------------------------------
def build_span(x0, x1, kind, colls, mats, idx):
    """One span: girders, deck slab, wearing course, diaphragms, parapets."""
    span = x1 - x0
    xm = 0.5 * (x0 + x1)
    detail = P.detail_at(xm)
    depth = P.girder_depth_for_span(span)
    ztop = P.deck_top_z(xm)
    z_slab_top = ztop - P.WEARING_COURSE_T
    z_gird_top = z_slab_top - P.DECK_SLAB_T
    made = []

    tag = f"{idx:03d}"
    lodtag = detail[0]

    # ---- deck slab --------------------------------------------------------
    slab = ML.box(f"BR_DECK_SLAB_{tag}",
                  (span, P.DECK_WIDTH, P.DECK_SLAB_T),
                  (xm, 0.0, z_gird_top + P.DECK_SLAB_T / 2),
                  colls["DECK"], mats["concrete_deck"])
    ML.set_custom(slab, {"avi_kind": "deck_slab", "avi_span": idx,
                         "avi_lod": detail, "avi_x0": x0, "avi_x1": x1})
    made.append(slab)

    # ---- wearing course (the visible road surface) -----------------------
    road_w = P.DECK_WIDTH - 2 * P.PARAPET_THICK
    wc = ML.box(f"BR_WEARING_{tag}",
                (span, road_w, P.WEARING_COURSE_T),
                (xm, 0.0, z_slab_top + P.WEARING_COURSE_T / 2),
                colls["DECK"], mats["asphalt"])
    made.append(wc)

    # ---- girders ----------------------------------------------------------
    if detail == P.DETAIL_LOW:
        # one closed box instead of six girders
        gb = ML.box(f"BR_DECKBOX_{tag}",
                    (span, P.GIRDER_SPACING * (P.GIRDER_COUNT - 1)
                     + P.GIRDER_TOP_FLANGE_W, depth),
                    (xm, 0.0, z_gird_top - depth / 2),
                    colls["BEAMS"], mats["concrete_low"])
        ML.set_custom(gb, {"avi_kind": "deck_box", "avi_lod": detail})
        made.append(gb)
    else:
        prof = girder_profile(depth, detail)
        y0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0
        for g in range(P.GIRDER_COUNT):
            y = y0 + g * P.GIRDER_SPACING
            ob = ML.prism(f"BR_GIRDER_{tag}_G{g+1}", prof, span, "X",
                          (xm, y, z_gird_top), colls["BEAMS"],
                          mats["concrete_girder"])
            ML.set_custom(ob, {"avi_kind": "girder", "avi_span": idx,
                               "avi_girder": g + 1, "avi_lod": detail,
                               "avi_depth": depth, "avi_y": y})
            made.append(ob)

        # ---- transverse diaphragms ---------------------------------------
        # At both ends and at intermediate points. The gaps between them are
        # the confined bays a UAV has to fly into to inspect a girder web.
        n_dia = max(2, int(span / 12.0) + 1)
        dw = P.GIRDER_SPACING * (P.GIRDER_COUNT - 1)
        for d in range(n_dia):
            dx = x0 + span * d / (n_dia - 1)
            dx = min(max(dx, x0 + P.DIAPHRAGM_T), x1 - P.DIAPHRAGM_T)
            dd = min(P.DIAPHRAGM_DEPTH, depth - 0.5)
            ob = ML.box(f"BR_DIAPHRAGM_{tag}_{d+1}",
                        (P.DIAPHRAGM_T, dw, dd),
                        (dx, 0.0, z_gird_top - P.GIRDER_FLANGE_T - dd / 2),
                        colls["BEAMS"], mats["concrete_girder"])
            ML.set_custom(ob, {"avi_kind": "diaphragm", "avi_span": idx,
                               "avi_lod": detail})
            made.append(ob)

    # ---- parapets ---------------------------------------------------------
    pp = parapet_profile()
    for s, nm in ((1, "L"), (-1, "R")):
        prof = [(s * (P.DECK_WIDTH / 2 - a), b) for a, b in pp]
        if s < 0:
            prof = prof[::-1]
        ob = ML.prism(f"BR_PARAPET_{tag}_{nm}", prof, span, "X",
                      (xm, 0.0, z_slab_top), colls["BARRIERS"],
                      mats["concrete_parapet"])
        ML.set_custom(ob, {"avi_kind": "parapet", "avi_side": nm,
                           "avi_lod": detail})
        made.append(ob)

    # ---- median barrier ---------------------------------------------------
    med = ML.taper_box(f"BR_MEDIAN_{tag}", (span, P.MEDIAN_WIDTH),
                       (span, P.MEDIAN_WIDTH * 0.45), 0.95,
                       (xm, 0.0, z_slab_top + 0.475),
                       colls["BARRIERS"], mats["concrete_parapet"])
    made.append(med)

    # ---- drainage + service duct (HIGH only) ------------------------------
    if detail == P.DETAIL_HIGH:
        n = max(1, int(span / P.DRAIN_SPACING))
        for d in range(n):
            dx = x0 + span * (d + 0.5) / n
            for s in (1, -1):
                y = s * (P.DECK_WIDTH / 2 - P.PARAPET_THICK - 0.30)
                ob = ML.cylinder(f"BR_DRAIN_{tag}_{d+1}{'L' if s>0 else 'R'}",
                                 P.DRAIN_PIPE_D / 2, 1.30,
                                 (dx, y, z_gird_top - 0.55), 10,
                                 colls["DETAILS"], mats["steel_dark"])
                ML.set_custom(ob, {"avi_kind": "drain"})
                made.append(ob)
        for s in (1, -1):
            y = s * (P.DECK_WIDTH / 2 - P.PARAPET_THICK - 0.75)
            ob = ML.cylinder(f"BR_SERVICE_DUCT_{tag}_{'L' if s>0 else 'R'}",
                             P.SERVICE_DUCT_D / 2, span,
                             (xm, y, z_gird_top - 0.42), 8,
                             colls["DETAILS"], mats["steel_dark"], axis="X")
            ML.set_custom(ob, {"avi_kind": "service_duct"})
            made.append(ob)

    return made


# ---------------------------------------------------------------------------
def build_pier(x, kind, colls, mats, idx):
    """Twin-column bent: footing, columns, pier cap, bearings."""
    detail = P.detail_at(x)
    gz = P.ground_z(x, 0.0)
    span = P.span_at(x)
    depth = P.girder_depth_for_span(span)
    z_gird_bot = (P.deck_top_z(x) - P.WEARING_COURSE_T - P.DECK_SLAB_T
                  - depth)
    cap_top = z_gird_bot - P.BEARING_H
    cap_bot = cap_top - P.PIER_CAP_H
    made = []
    tag = f"{idx:03d}"
    in_river = kind == "river" and gz < P.RIVER_WATER_Z - 0.5
    col_d = P.PIER_COL_D_RIVER if in_river else P.PIER_COL_D
    seg = 20 if detail == P.DETAIL_HIGH else (12 if detail == P.DETAIL_MED
                                              else 8)

    # ---- footing ----------------------------------------------------------
    fz = gz - P.PIER_FOOTING_H * 0.35
    for c in range(P.PIER_COLS_PER_BENT):
        y = (c - (P.PIER_COLS_PER_BENT - 1) / 2.0) * P.PIER_COL_SPACING
        if detail != P.DETAIL_LOW:
            ob = ML.box(f"BR_PIER_FOOTING_{tag}_{c+1}",
                        (P.PIER_FOOTING_L, P.PIER_FOOTING_W,
                         P.PIER_FOOTING_H),
                        (x, y, fz), colls["PIERS"], mats["concrete_pier"])
            ML.set_custom(ob, {"avi_kind": "pier_footing", "avi_pier": idx})
            made.append(ob)

        # ---- column -------------------------------------------------------
        h = cap_bot - (fz + P.PIER_FOOTING_H / 2)
        if h <= 0.5:
            continue
        cz = fz + P.PIER_FOOTING_H / 2 + h / 2
        ob = ML.cylinder(f"BR_PIER_COL_{tag}_{c+1}", col_d / 2, h,
                         (x, y, cz), seg, colls["PIERS"],
                         mats["concrete_pier"])
        ML.set_custom(ob, {"avi_kind": "pier_column", "avi_pier": idx,
                           "avi_lod": detail, "avi_height": h,
                           "avi_in_river": in_river, "avi_x": x, "avi_y": y})
        made.append(ob)

        # ---- pier collar at the waterline (scour protection) --------------
        if in_river and detail != P.DETAIL_LOW:
            ob = ML.cylinder(f"BR_PIER_COLLAR_{tag}_{c+1}",
                             col_d / 2 + 0.45, 1.6,
                             (x, y, P.RIVER_WATER_Z - 0.2), seg,
                             colls["PIERS"], mats["concrete_pier"])
            made.append(ob)

    # ---- pier cap ---------------------------------------------------------
    if detail == P.DETAIL_HIGH:
        cap = ML.taper_box(f"BR_PIER_CAP_{tag}",
                           (P.PIER_CAP_W, P.PIER_CAP_L * 0.88),
                           (P.PIER_CAP_W, P.PIER_CAP_L),
                           P.PIER_CAP_H,
                           (x, 0.0, cap_bot + P.PIER_CAP_H / 2),
                           colls["PIERS"], mats["concrete_pier"])
    else:
        cap = ML.box(f"BR_PIER_CAP_{tag}",
                     (P.PIER_CAP_W, P.PIER_CAP_L, P.PIER_CAP_H),
                     (x, 0.0, cap_bot + P.PIER_CAP_H / 2),
                     colls["PIERS"], mats["concrete_pier"])
    ML.set_custom(cap, {"avi_kind": "pier_cap", "avi_pier": idx,
                        "avi_lod": detail, "avi_x": x})
    made.append(cap)

    # ---- bearings ---------------------------------------------------------
    if detail != P.DETAIL_LOW:
        y0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0
        for g in range(P.GIRDER_COUNT):
            y = y0 + g * P.GIRDER_SPACING
            if abs(y) > P.PIER_CAP_L / 2 - 0.4:
                continue
            ob = ML.box(f"BR_BEARING_{tag}_G{g+1}",
                        (P.BEARING_L, P.BEARING_W, P.BEARING_H),
                        (x, y, cap_top - P.BEARING_H / 2),
                        colls["DETAILS"], mats["bearing"])
            ML.set_custom(ob, {"avi_kind": "bearing", "avi_pier": idx,
                               "avi_girder": g + 1})
            made.append(ob)

    return made


# ---------------------------------------------------------------------------
def build_joints(colls, mats):
    """Expansion joints -- a visible gap plus the steel nosing either side.

    Placed every JOINT_EVERY_N_SPANS piers. These are a defect hot-spot in
    real structures, which is why the damage model targets them specifically.
    """
    made = []
    ps = P.pier_stations()
    k = 0
    for i, (x, kind) in enumerate(ps):
        if i == 0 or i == len(ps) - 1:
            continue
        if i % P.JOINT_EVERY_N_SPANS != 0:
            continue
        k += 1
        z = P.deck_top_z(x)
        detail = P.detail_at(x)
        w = P.DECK_WIDTH - 2 * P.PARAPET_THICK
        gap = ML.box(f"BR_JOINT_{k:03d}_GAP", (P.JOINT_GAP, w, 0.12),
                     (x, 0.0, z - 0.05), colls["JOINTS"], mats["steel_dark"])
        ML.set_custom(gap, {"avi_kind": "expansion_joint", "avi_joint": k,
                            "avi_x": x, "avi_lod": detail})
        made.append(gap)
        if detail != P.DETAIL_LOW:
            for s in (-1, 1):
                nose = ML.box(f"BR_JOINT_{k:03d}_NOSE_{'A' if s<0 else 'B'}",
                              (0.16, w, 0.10),
                              (x + s * 0.14, 0.0, z - 0.045),
                              colls["JOINTS"], mats["steel"])
                made.append(nose)
    return made


# ---------------------------------------------------------------------------
def build(colls, mats, log=print):
    """Whole corridor."""
    ps = P.pier_stations()
    stats = {"spans": 0, "piers": 0, "objects": 0}

    log(f"  piers   : {len(ps)}")
    for i, (x, kind) in enumerate(ps):
        objs = build_pier(x, kind, colls, mats, i + 1)
        stats["piers"] += 1
        stats["objects"] += len(objs)

    log(f"  spans   : {len(ps) - 1}")
    for i in range(len(ps) - 1):
        x0, k0 = ps[i]
        x1, k1 = ps[i + 1]
        objs = build_span(x0, x1, k0, colls, mats, i + 1)
        stats["spans"] += 1
        stats["objects"] += len(objs)

    j = build_joints(colls, mats)
    stats["objects"] += len(j)
    log(f"  joints  : {len(j)}")

    # abutments at both ends
    for x, nm in ((0.0, "SOUTH"), (P.BRIDGE_LENGTH, "NORTH")):
        z = P.deck_top_z(x)
        gz = P.ground_z(x, 0.0)
        h = max(1.0, z - gz)
        ob = ML.taper_box(f"BR_ABUTMENT_{nm}",
                          (7.0, P.DECK_WIDTH + 4.0),
                          (4.0, P.DECK_WIDTH + 1.0), h,
                          (x + (3.0 if x < 1 else -3.0), 0.0, gz + h / 2),
                          colls["PIERS"], mats["concrete_pier"])
        ML.set_custom(ob, {"avi_kind": "abutment", "avi_end": nm})
        stats["objects"] += 1
        # wing walls
        for s in (1, -1):
            w = ML.box(f"BR_WINGWALL_{nm}_{'L' if s>0 else 'R'}",
                       (14.0, 0.6, h * 0.8),
                       (x + (10.0 if x < 1 else -10.0),
                        s * (P.DECK_WIDTH / 2 + 1.6), gz + h * 0.4),
                       colls["PIERS"], mats["concrete_pier"])
            stats["objects"] += 1

    return stats

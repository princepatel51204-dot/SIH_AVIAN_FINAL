"""SIH_AVIAN_FINAL -- the drone base: two landing pads, SCANNER and REPAIRER.

THE COLLISION TRAP
-------------------
`AVIAN_UAV/simulation/export_bridge_collision.py`'s `EXCLUDE_PREFIXES`
contains `"AVI_HOME"` -- a pad literally named `AVI_HOME_*` would render
perfectly, the SDF would parse, the world would load, and the UAV would
fall straight through it on first spawn. Two independent things make this
scene's pads safe instead of merely lucky:

1. The pad names ("AVI_BASE_SCANNER", "AVI_BASE_REPAIRER") do not start
   with any string in EXCLUDE_PREFIXES -- checked against the literal list,
   not assumed.
2. Classification in that exporter does not depend on the name prefix at
   all for these: `avi_kind="landing_pad"` is now a key in
   `STRUCTURAL_KINDS` (mapped to "BOX", added directly to
   export_bridge_collision.py -- purely additive, inert for REV-C since it
   has no landing_pad objects). A primitive is included as structural if
   `kind in STRUCTURAL_KINDS` OR its name matches a structural prefix; the
   pad deck qualifies on kind alone, which is the robust path regardless of
   naming.

Only the DECK carries `avi_kind="landing_pad"` -- the kerb, corner markers,
cabin, mast and charging docks are decorative and are not meant to be
independent obstacles the way the deck (something a UAV lands ON) is.
"""
from __future__ import annotations
import math

import meshlib as ML

PAD_SIZE = 6.0
PAD_GAP_Y = 12.0
BASE_X = 20.0
BASE_Y = -30.0          # south of the road bridge (edge at y=-7), clear


def _pad(role, centre, coll, mats, log=print):
    x, y, gz = centre
    out = []
    deck = ML.box(f"AVI_BASE_{role}", (PAD_SIZE, PAD_SIZE, 0.25),
                 (x, y, gz + 0.125), coll, mats["concrete_low"])
    ML.set_custom(deck, {
        "avi_kind": "landing_pad",
        "avi_base_role": role,
        "avi_pad_centre_m": [round(x, 3), round(y, 3), round(gz + 0.25, 3)],
    })
    out.append(deck)

    # low kerb -- a raised border ring, four boxes
    kerb_h = 0.10
    half = PAD_SIZE / 2.0
    out.append(ML.box(f"AVI_BASE_{role}_KERB_N", (PAD_SIZE + 0.3, 0.15, kerb_h),
                      (x, y + half + 0.075, gz + kerb_h / 2), coll, mats["barrier"]))
    out.append(ML.box(f"AVI_BASE_{role}_KERB_S", (PAD_SIZE + 0.3, 0.15, kerb_h),
                      (x, y - half - 0.075, gz + kerb_h / 2), coll, mats["barrier"]))
    out.append(ML.box(f"AVI_BASE_{role}_KERB_E", (0.15, PAD_SIZE, kerb_h),
                      (x + half + 0.075, y, gz + kerb_h / 2), coll, mats["barrier"]))
    out.append(ML.box(f"AVI_BASE_{role}_KERB_W", (0.15, PAD_SIZE, kerb_h),
                      (x - half - 0.075, y, gz + kerb_h / 2), coll, mats["barrier"]))

    # 4 corner markers -- short reflective posts
    for cxs, cys, nm in ((1, 1, "NE"), (1, -1, "SE"), (-1, 1, "NW"), (-1, -1, "SW")):
        out.append(ML.cylinder(f"AVI_BASE_{role}_MARKER_{nm}", 0.05, 0.5,
                               (x + cxs * (half - 0.2), y + cys * (half - 0.2),
                                gz + 0.25 + 0.25), 6, coll, mats["sign"]))

    # painted H, circle border -- real geometry, thin and flush
    paint_z = gz + 0.26
    bar_w, bar_l, bar_t = 0.35, 2.4, 0.02
    for sx, nm in ((1, "L"), (-1, "R")):
        out.append(ML.box(f"AVI_BASE_{role}_H_{nm}", (bar_w, bar_l, bar_t),
                          (x + sx * 0.75, y, paint_z), coll, mats["paint_white"]))
    out.append(ML.box(f"AVI_BASE_{role}_H_BAR", (1.5 + bar_w, bar_w, bar_t),
                      (x, y, paint_z), coll, mats["paint_white"]))
    seg = 28
    r_out, r_in = half - 0.35, half - 0.55
    cv, cf = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        cv.append((x + r_in * math.cos(a), y + r_in * math.sin(a), paint_z))
    for i in range(seg):
        a = 2 * math.pi * i / seg
        cv.append((x + r_out * math.cos(a), y + r_out * math.sin(a), paint_z))
    for i in range(seg):
        j = (i + 1) % seg
        cf.append((i, j, seg + j, seg + i))
    out.append(ML.mesh_obj(f"AVI_BASE_{role}_CIRCLE", cv, cf, coll,
                           mats["paint_white"]))

    # charging dock -- a small pedestal at the pad's inboard edge
    dock_y = y + (half + 1.0) * (1 if role == "SCANNER" else -1)
    out.append(ML.box(f"AVI_BASE_{role}_DOCK", (1.2, 0.8, 0.6),
                      (x, dock_y, gz + 0.3), coll, mats["steel_dark"]))
    out.append(ML.box(f"AVI_BASE_{role}_DOCK_POST", (0.15, 0.15, 1.1),
                      (x, dock_y, gz + 0.6 + 0.55), coll, mats["steel_dark"]))

    log(f"  base    : pad {role} at ({x:.1f}, {y:.1f}, {gz:.2f}), "
        f"{len(out)} objects")
    return out


def build(colls, mats, terrain_height, log=print):
    coll = colls["VEHICLES"]      # decorative/base furniture, same as traffic
    y_scanner = BASE_Y
    y_repairer = BASE_Y + PAD_GAP_Y
    gz_s = terrain_height(BASE_X, y_scanner)
    gz_r = terrain_height(BASE_X, y_repairer)

    out = []
    out += _pad("SCANNER", (BASE_X, y_scanner, gz_s), coll, mats, log)
    out += _pad("REPAIRER", (BASE_X, y_repairer, gz_r), coll, mats, log)

    # equipment cabin, between the two pads
    cabin_y = (y_scanner + y_repairer) / 2.0
    cabin_x = BASE_X + PAD_SIZE / 2.0 + 3.0
    gz_c = terrain_height(cabin_x, cabin_y)
    out.append(ML.box("AVI_BASE_CABIN", (2.5, 3.0, 2.4),
                      (cabin_x, cabin_y, gz_c + 1.2), coll, mats["concrete_low"]))
    out.append(ML.box("AVI_BASE_CABIN_DOOR", (0.05, 0.9, 1.9),
                      (cabin_x - 1.26, cabin_y, gz_c + 0.95), coll,
                      mats["steel_dark"]))

    # mast + windsock, beside the cabin
    mast_x, mast_y = cabin_x + 1.8, cabin_y
    gz_m = terrain_height(mast_x, mast_y)
    out.append(ML.cylinder("AVI_BASE_MAST", 0.06, 5.0,
                           (mast_x, mast_y, gz_m + 2.5), 8, coll,
                           mats["steel_dark"]))
    sock_v = [
        (mast_x - 0.03, mast_y - 0.22, gz_m + 4.9),
        (mast_x - 0.03, mast_y + 0.22, gz_m + 4.9),
        (mast_x + 1.1, mast_y + 0.06, gz_m + 4.75),
        (mast_x + 1.1, mast_y - 0.06, gz_m + 4.75),
    ]
    out.append(ML.mesh_obj("AVI_BASE_WINDSOCK", sock_v, [(0, 1, 2, 3)],
                           coll, mats["sign"]))

    for ob in out:
        if "avi_kind" not in ob.keys():
            ob["avi_kind"] = "base_furniture"
    log(f"  base    : {len(out)} total objects (2 pads, cabin, mast+windsock, "
        f"docks)")
    return {"objects": len(out),
           "pads": {"SCANNER": [BASE_X, y_scanner, gz_s + 0.25],
                    "REPAIRER": [BASE_X, y_repairer, gz_r + 0.25]}}

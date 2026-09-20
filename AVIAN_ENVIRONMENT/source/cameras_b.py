"""The twelve REV-B named cameras.

The seven REV-A cameras are preserved with their original names and poses --
they are what the REV-A validation renders were shot with, and changing them
would invalidate the comparison. Five are added, and three of those are aimed
from the ground truth at build time rather than from hard coordinates, because
a camera pointed at a fixed point in space is pointed at whatever happens to be
there after the next change.
"""
from __future__ import annotations
import math

import bpy
from mathutils import Vector

import params as P
import meshlib as ML
import zones as Z


EXTRA_SPECS = [
    ("CAMERA_08_PIER_INSPECTION", "pier",
     "river pier at inspection standoff, waterline to pier cap", 35.0),
    ("CAMERA_09_GIRDER_INSPECTION", "girder",
     "along a girder bay: the confined under-deck corridor", 20.0),
    ("CAMERA_10_DIFFICULT_OCCLUSION", "occluded",
     "the most occluded defect in the ground truth, from the only "
     "direction that reaches it", 35.0),
    ("CAMERA_11_UAV_APPROACH", "approach",
     "the view from a UAV on final approach to a sector, at transit "
     "altitude looking down the corridor", 28.0),
    ("CAMERA_12_UAV_INSPECTION", "inspect",
     "over-the-sensor view at working standoff from the sensor rig pose",
     24.0),
]


def _cam(name, eye, target, lens, purpose, coll, extra=None):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start = 0.02
    cd.clip_end = 60000.0
    ob = bpy.data.objects.new(name, cd)
    coll.objects.link(ob)
    ob.location = eye
    ML.look_at(ob, target)
    d = {
        "avi_kind": "validation_camera",
        "avi_object_type": "CAMERA",
        "avi_object_id": name,
        "avi_purpose": purpose,
        "avi_focal_length_mm": lens,
        "avi_target_m": [round(v, 2) for v in target],
        "avi_range_m": round((Vector(eye) - Vector(target)).length, 1),
        "avi_sector": P.sector_at(eye[0]) or "OUTSIDE_RESEARCH_ZONE",
    }
    if extra:
        d.update(extra)
    ML.set_custom(ob, d)
    return ob


def build_extra(coll, records, log=print):
    """Add cameras 08-12. Three are aimed from the ground truth."""
    made = []
    x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1
    half_w = P.DECK_WIDTH / 2.0

    # ---- 08 river pier ---------------------------------------------------
    ps = [x for x, k in P.pier_stations()
          if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 - 40]
    px = min(ps, key=lambda x: abs(x - P.RIVER_CENTRE_X)) if ps else \
        P.RIVER_CENTRE_X
    made.append(_cam("CAMERA_08_PIER_INSPECTION",
                     (px + 16.0, -26.0, 8.0),
                     (px, -P.PIER_COL_SPACING / 2, 6.0),
                     35.0, EXTRA_SPECS[0][2], coll,
                     {"avi_pier_station_m": round(px, 1),
                      "avi_in_water": True}))

    # ---- 09 girder bay ---------------------------------------------------
    xg = x0r + 2.2 * P.SECTOR_LENGTH
    y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0
    bay_y = y_g0 + P.GIRDER_SPACING * 1.5
    zg = P.soffit_z(xg) + 1.1
    made.append(_cam("CAMERA_09_GIRDER_INSPECTION",
                     (xg - 14.0, bay_y, zg),
                     (xg + 30.0, bay_y, zg + 0.2),
                     20.0, EXTRA_SPECS[1][2], coll,
                     {"avi_bay_width_m": round(P.GIRDER_SPACING
                                               - P.GIRDER_TOP_FLANGE_W, 2)}))

    # ---- 10 the hardest defect in the model ------------------------------
    worst = None
    if records:
        scored = [r for r in records if "occlusion_measured" in r]
        if scored:
            worst = max(scored, key=lambda r: (
                r["occlusion_measured"],
                r.get("detection_difficulty_score", 0.0)))
    if worst is not None:
        p = Vector(worst["position_m"])
        d = Vector(worst.get("recommended_view_direction",
                             worst["surface_normal"]))
        eye = p + d * 1.6
        made.append(_cam("CAMERA_10_DIFFICULT_OCCLUSION", tuple(eye),
                         tuple(p), 35.0, EXTRA_SPECS[2][2], coll,
                         {"avi_defect_id": worst["defect_id"],
                          "avi_occlusion_measured":
                              worst["occlusion_measured"],
                          "avi_detection_difficulty":
                              worst.get("detection_difficulty", "?"),
                          "avi_note": "aimed from the measured visibility "
                                      "ground truth, not a fixed point"}))
    else:
        made.append(_cam("CAMERA_10_DIFFICULT_OCCLUSION",
                         (x0r + 300.0, -half_w - 2.0,
                          P.soffit_z(x0r + 300.0) + 1.0),
                         (x0r + 302.0, -half_w + 1.0,
                          P.soffit_z(x0r + 300.0) + 1.0),
                         35.0, EXTRA_SPECS[2][2], coll))

    # ---- 11 UAV approach -------------------------------------------------
    deck_max = max(P.deck_top_z(x) for x in range(int(x0r), int(x1r), 10))
    tz = max(P.AIRSPACE_SAFE_Z[0], deck_max + 20.0) + 6.0
    made.append(_cam("CAMERA_11_UAV_APPROACH",
                     (x0r - 190.0, -150.0, tz),
                     (x0r + 240.0, 0.0, P.deck_top_z(x0r + 240.0)),
                     28.0, EXTRA_SPECS[3][2], coll,
                     {"avi_altitude_m": round(tz, 1),
                      "avi_note": "the transit-band view a UAV actually "
                                  "has on approach"}))

    # ---- 12 from the sensor rig ------------------------------------------
    rig = bpy.data.objects.get("AVI_SENSOR_RIG")
    if rig is not None:
        rp = Vector(rig.matrix_world.translation)
        tgt = Vector((rp.x + 6.0, 0.0, rp.z + 1.0))
        made.append(_cam("CAMERA_12_UAV_INSPECTION", tuple(rp), tuple(tgt),
                         24.0, EXTRA_SPECS[4][2], coll,
                         {"avi_from_sensor_rig": True,
                          "avi_matches_frame": "AVI_SENSOR_RGB_FRONT"}))
    else:
        xm = x0r + 2.5 * P.SECTOR_LENGTH
        made.append(_cam("CAMERA_12_UAV_INSPECTION",
                         (xm, half_w + P.AIRSPACE_INSPECTION_OFFSET,
                          P.soffit_z(xm) - 2.0),
                         (xm + 6.0, 0.0, P.soffit_z(xm) - 1.0),
                         24.0, EXTRA_SPECS[4][2], coll))

    log(f"  cam-B   : {len(made)} additional cameras (08-12)")
    return made

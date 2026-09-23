"""SIH_AVIAN_FINAL -- Phase 2 Step 1: the UAV-camera detection dataset,
phase 1 of 2 -- prep (ray-cast negative sampling, instance IDs). Renders
nothing; see `dataset_render_final.py` for phase 2 (all rendering).

Prepares everything for a dataset covering every one of the 192 defects
(96 road + 20 metro + 76 steel) at its own measured `min_detect_range_m`
(the theoretical best case) and, where reachable, the 1.5 m
`UNDERSIDE_INSPECTION` survey standoff -- plus matched SOUND negatives, so
the dataset does not teach a detector that every bolt is loose.

THE CAMERA IS THE UAV'S CAMERA, NOT AN ARBITRARY ONE
-----------------------------------------------------
The whole resolvability argument (SIH_AVIAN_DETECTION_MASTER_PROMPT.md)
rests on 2.1478 mm/px @ 1 m. That number is `AVIAN_UAV/sensors/cameras.py`'s
`RGBCamera` (hfov=69 deg) at its own LOOP_RES=(640,480) -- read directly
from that module (not retyped) so this can never drift from the real sensor
spec, and cross-checked against that package's own measured test result
(S13 in `AVIAN_UAV/logs/phase1_sensors.json`: "GSD 2.1478 mm/px @1m").
`visibility.py`'s OWN `RGB_GSD_MM_AT_1M = 0.62` is a DIFFERENT sensor's
number, used only for that module's internal range calc -- reaching for it
here would be the exact "unlinked constant" mistake this project already
made once (Phase 3's Gazebo materials) and once more (resolvability's own
docstring records the same trap). The achieved GSD is measured from the
actual Blender camera angle and resolution used for each frame, then
asserted against 2.1478 (VD02) -- not assumed.

THIS IS PHASE 1 OF 2 -- PREP, NOT RENDER
-------------------------------------------
This script does the RAY-CASTING (negative-sample surface snapping via
`damage._find_host`) and instance-ID assignment, then SAVES and EXITS --
it renders nothing. `dataset_render_final.py` is phase 2: a FRESH Blender
process that opens what this one saved and does all four render passes
through CYCLES.

This split exists for exactly the reason the rest of this project already
splits build from measure: contrast_c.py's own docstring documents that a
Cycles render issued AFTER a `scene.ray_cast()` call in the SAME process
segfaults inside Embree's BVH build on this machine, and this script's own
negative sampling calls `damage._find_host()` (a ray-cast) extensively.
EEVEE was tried first specifically to avoid the two-process split, but
EEVEE's Object Index compositor output (the per-defect MASK -- VD01, the
check that matters most) silently produces no file at all in this Blender
version (4.0.2's legacy EEVEE), while Cycles produces it correctly --
confirmed by a direct side-by-side test, not assumed. So the split is back,
and Cycles is what phase 2 uses.

RESOLUTION SCOPE (a real design choice, stated rather than buried)
--------------------------------------------------------------------
"Render both [LOOP and CAPTURE] and record which is which" is read as: the
PRIMARY dataset (every positive at both standoffs, every negative) renders
at LOOP resolution -- "the detector's real-world input" -- and every
positive's min_detect_range_m shot ALSO gets a CAPTURE-resolution
companion, specifically to make the loop-vs-capture GSD gap measurable
where it matters most (the headline resolvability number), without
tripling the render budget for shots (negatives, survey-standoff) where
that comparison isn't the point. This is a chosen scope, logged here and in
the manifest, not a silent assumption.
"""
from __future__ import annotations
import csv
import inspect
import json
import math
import os
import random
import sys
import time

import bpy
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
ENV_SRC = os.environ.get(
    "AVIAN_ENV_SRC", os.path.join(REPO, "AVIAN_ENVIRONMENT", "source"))
UAV_DIR = os.environ.get("AVIAN_UAV_DIR", os.path.join(REPO, "AVIAN_UAV"))
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if UAV_DIR not in sys.path:
    sys.path.insert(0, UAV_DIR)

import damage as DMG
import damage_steel as DMGS
import params_final as PF
import sensors.cameras as UAVCAM   # the real sensor spec, read not retyped
from validate import Result   # same Result/row shape as every other _final check module

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
DATASET_DIR = os.path.join(ROOT, "dataset")
IMG_DIR = os.path.join(DATASET_DIR, "images")
MASK_DIR = os.path.join(DATASET_DIR, "masks")
DEPTH_DIR = os.path.join(DATASET_DIR, "depth")
NORMAL_DIR = os.path.join(DATASET_DIR, "normals")
SAMPLES_DIR = os.path.join(DATASET_DIR, "samples")

# ---------------------------------------------------------------------------
# sensor intrinsics -- read from AVIAN_UAV/sensors/cameras.py, not retyped
# ---------------------------------------------------------------------------
_sig = inspect.signature(UAVCAM.RGBCamera.__init__)
HFOV_DEG = float(_sig.parameters["hfov_deg"].default)
VFOV_DEG = float(_sig.parameters["vfov_deg"].default)
LOOP_RES = tuple(UAVCAM.LOOP_RES)
CAPTURE_RES = tuple(UAVCAM.CAPTURE_RES)


def _gsd_mm_at_1m(hfov_deg, width_px):
    """Same formula AVIAN_UAV/sensors/cameras.py's RGBCamera.read() uses for
    its own `gsd_mm_at_1m` field -- ground sample distance at 1 m range for
    a pinhole camera with this horizontal FOV and pixel width."""
    return 1000.0 * 2.0 * math.tan(math.radians(hfov_deg / 2.0)) / width_px


LOOP_GSD_MM_AT_1M = _gsd_mm_at_1m(HFOV_DEG, LOOP_RES[0])
CAPTURE_GSD_MM_AT_1M = _gsd_mm_at_1m(HFOV_DEG, CAPTURE_RES[0])

STEEL_DEFECT_TYPES_BOLT = ("BOLT_LOOSE", "BOLT_MISSING", "BOLT_CORRODED")
CONCRETE_DEFECT_TYPES = (
    "CRACK_HAIRLINE", "CRACK_LONGITUDINAL", "CRACK_TRANSVERSE",
    "CRACK_NETWORK", "SPALL", "DELAMINATION", "REBAR_EXPOSED",
    "CORROSION_STAIN", "JOINT_DETERIORATION",
    "SEGMENTAL_JOINT_LEAKAGE", "EFFLORESCENCE_JOINT", "BEARING_DISTRESS",
    "INTERNAL_SOFFIT_CRACK", "WEB_SHEAR_CRACK", "SOFFIT_TRANSVERSE_CRACK",
    "PIER_SPALL", "PIER_CORROSION_STAIN",
)

NEGATIVE_PLAN = [
    # (label, n, matched_types)
    ("SOUND_BOLT", 300, STEEL_DEFECT_TYPES_BOLT),
    ("SOUND_CONCRETE", 120, CONCRETE_DEFECT_TYPES),
    ("SOUND_WELD", 40, ("WELD_CRACK",)),
    ("SOUND_COATING", 40, ("COATING_FAILURE",)),
    ("SOUND_BEARING", 20, ("BEARING_SEIZED",)),
]


# ===========================================================================
# 1. load ground truth, assign instance ids
# ===========================================================================
def _load_records():
    with open(os.path.join(
            SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.json")) as f:
        records = json.load(f)["defects"]
    with open(os.path.join(
            SCENE_DIR, "AVIAN_metro_ground_truth_FINAL.json")) as f:
        mrecords = json.load(f)["defects"]
    with open(os.path.join(
            SCENE_DIR, "AVIAN_steel_ground_truth_FINAL.json")) as f:
        srecords = json.load(f)["defects"]
    for r in records:
        r["_host_structure"] = "ROAD"
        r["avi_condition"] = PF.ROAD_CONDITION
    for r in mrecords:
        r["_host_structure"] = "METRO"
        r["avi_condition"] = PF.METRO_CONDITION
    for r in srecords:
        host = r.get("host_object", "")
        if host.startswith("MB_"):
            r["_host_structure"] = "METRO"
            r["avi_condition"] = PF.METRO_CONDITION
        else:
            r["_host_structure"] = "TRUSS" if host.startswith("ST_") else "ROAD"
            r["avi_condition"] = PF.ROAD_CONDITION
    return records + mrecords + srecords


def assign_instance_ids(all_records, log=print):
    """One stable instance id per defect, 1..N, written to the Blender
    object's pass_index so the Object Index render pass IS the per-defect
    mask -- same mechanism dataset.py's own assign_indices() uses for
    REV-C, reimplemented here because that one only scans BR_-prefixed
    structural members and REV-C's own defect list, not this scene's
    combined road+metro+steel 192."""
    n = 0
    for i, r in enumerate(all_records, start=1):
        ob = bpy.data.objects.get(r["defect_id"])
        if ob is None:
            r["instance_id"] = None
            continue
        ob.pass_index = i
        ob["avi_instance_id"] = i
        r["instance_id"] = i
        n += 1
    log(f"  indices : {n}/{len(all_records)} defects assigned an instance id")
    return n




# ===========================================================================
# 2. negatives -- sound examples, matched to the positives they stand in for
# ===========================================================================
_ASSEMBLY_PREFIXES = ("GUSSET", "FLOOR_STRINGER", "BEARING", "JOINT_ANCHOR",
                     "WALKWAY_BRACKET", "CABLE_CLAMP", "HANDRAIL_BASE")


def _bolt_assembly(defect_id):
    for a in _ASSEMBLY_PREFIXES:
        if defect_id.startswith(f"FAST_{a}_"):
            return a
    return None


def _bolt_id_from_defect_id(defect_id):
    for suf in ("_MARK_NUT", "_NUT", "_HOLE"):
        if defect_id.endswith(suf):
            return defect_id[:-len(suf)]
    return defect_id


def sample_sound_bolts(bolt_defects, n_total, rnd, log=print):
    """Sound bolts near (same assembly as) the 36 loose/missing/corroded
    ones -- there are 1,164 of them (1,200 - 36), so the pool is never
    the constraint; diversity across joints is."""
    defect_bolt_ids = {_bolt_id_from_defect_id(r["defect_id"])
                       for r in bolt_defects}
    pool_by_assembly = {}
    for ob in bpy.data.objects:
        if not (ob.name.startswith("FAST_") and ob.name.endswith("_NUT")
                and not ob.name.endswith("_MARK_NUT")):
            continue
        bolt_id = ob.name[:-len("_NUT")]
        if bolt_id in defect_bolt_ids:
            continue
        assembly = _bolt_assembly(bolt_id)
        if assembly not in ("GUSSET", "FLOOR_STRINGER"):
            continue
        pool_by_assembly.setdefault(assembly, []).append(bolt_id)

    negatives = []
    per, extra = divmod(n_total, max(1, len(bolt_defects)))
    for i, r in enumerate(bolt_defects):
        assembly = _bolt_assembly(r["defect_id"]) or "GUSSET"
        pool = pool_by_assembly.get(assembly, [])
        k = per + (1 if i < extra else 0)
        k = min(k, len(pool))
        chosen = rnd.sample(pool, k) if k else []
        for bolt_id in chosen:
            pool.remove(bolt_id)
            ob = bpy.data.objects.get(f"{bolt_id}_NUT")
            if ob is None:
                continue
            pos = tuple(ob.matrix_world.translation)
            normal = tuple((ob.matrix_world.to_3x3()
                           @ Vector((0, 0, 1))).normalized())
            negatives.append({
                "label": "SOUND_BOLT", "matched_defect_id": r["defect_id"],
                "matched_type": r["type"], "position_m": list(pos),
                "surface_normal": list(normal),
                "min_detect_range_m": r["min_detect_range_m"],
                "avi_condition": r.get("avi_condition", PF.ROAD_CONDITION),
                "host_structure": "TRUSS", "host_object": bolt_id,
            })
    log(f"  negatives: {len(negatives)} SOUND_BOLT sampled "
        f"(target {n_total})")
    return negatives


def _perp_tangent(normal):
    n = Vector(normal).normalized()
    arb = Vector((0, 0, 1)) if abs(n.z) < 0.9 else Vector((1, 0, 0))
    return n.cross(arb).normalized()


# damage.py's own _RAY_SKIP_PREFIXES = ("DEFECT_", "AVI_", "_") does not
# catch this project's own steel defect objects -- they are named
# "SDEFECT_..." (an "S" prefix chosen in the detection pass, not "DEFECT_"),
# so a ray meant to find a clean structural surface could otherwise land on
# a defect's own decal instead of the member behind it.
_NEG_RAY_SKIP = ("DEFECT_", "AVI_", "_", "SDEFECT_")


def sample_offset_negatives(label, matched_records, n_total, rnd,
                            offset_range=(0.6, 1.4), log=print):
    """A point on the SAME kind of surface as each matched positive, offset
    sideways and snapped onto whatever real surface is actually there via
    the same ray-cast damage.py itself uses (`_find_host`) -- not just an
    arithmetic offset, since an offset with no snap could land in open air
    off the edge of a narrow member."""
    negatives = []
    per, extra = divmod(n_total, max(1, len(matched_records)))
    for i, r in enumerate(matched_records):
        k = per + (1 if i < extra else 0)
        pos = Vector(r["position_m"])
        normal = Vector(r["surface_normal"]).normalized()
        tangent = _perp_tangent(normal)
        made = 0
        attempts = 0
        while made < k and attempts < k * 6:
            attempts += 1
            side = rnd.choice((-1.0, 1.0))
            off = side * rnd.uniform(*offset_range)
            candidate = pos + tangent * off
            host, hit, face_n = DMG._find_host(candidate, normal,
                                               exclude_prefix=_NEG_RAY_SKIP)
            if host is None:
                continue
            negatives.append({
                "label": label, "matched_defect_id": r["defect_id"],
                "matched_type": r["type"],
                "position_m": [round(v, 4) for v in hit],
                "surface_normal": [round(v, 4) for v in face_n],
                "min_detect_range_m": r["min_detect_range_m"],
                "avi_condition": r.get("avi_condition", PF.ROAD_CONDITION),
                "host_structure": r.get("_host_structure", "ROAD"),
                "host_object": host.name,
            })
            made += 1
    log(f"  negatives: {len(negatives)} {label} sampled (target {n_total})")
    return negatives


def sample_sound_bearings(bearing_defects, n_total, rnd, log=print):
    """Other BEARING_SEAT bearings, not the 4 seized ones -- a bearing pad
    is a small discrete object, not a continuous surface, so an offset+
    snap (as for concrete/welds/coating) would usually just land back on
    the SAME pad; picking a genuinely different one is the right match
    here, same shape as sample_sound_bolts()."""
    used = {r["host_object"] for r in bearing_defects}
    pool = [o.name for o in bpy.data.objects
           if o.type == "MESH" and "BEARING" in o.name
           and o.name.startswith(("BR_BEARING_004", "BR_BEARING_005"))
           and o.name not in used]
    negatives = []
    per, extra = divmod(n_total, max(1, len(bearing_defects)))
    for i, r in enumerate(bearing_defects):
        k = per + (1 if i < extra else 0)
        k = min(k, len(pool))
        chosen = rnd.sample(pool, k) if k else []
        for name in chosen:
            pool.remove(name)
            ob = bpy.data.objects.get(name)
            verts = [ob.matrix_world @ v.co for v in ob.data.vertices]
            cx = sum(v.x for v in verts) / len(verts)
            cy = sum(v.y for v in verts) / len(verts)
            ztop = max(v.z for v in verts)
            negatives.append({
                "label": "SOUND_BEARING", "matched_defect_id": r["defect_id"],
                "matched_type": r["type"],
                "position_m": [round(cx, 4), round(cy, 4), round(ztop, 4)],
                "surface_normal": [0.0, 0.0, 1.0],
                "min_detect_range_m": r["min_detect_range_m"],
                "avi_condition": r.get("avi_condition", PF.ROAD_CONDITION),
                "host_structure": "ROAD", "host_object": name,
            })
    log(f"  negatives: {len(negatives)} SOUND_BEARING sampled "
        f"(target {n_total})")
    return negatives


def build_negatives(all_records, rnd, log=print):
    bolt_defects = [r for r in all_records
                   if r["type"] in STEEL_DEFECT_TYPES_BOLT]
    bearing_defects = [r for r in all_records if r["type"] == "BEARING_SEIZED"]
    weld_defects = [r for r in all_records if r["type"] == "WELD_CRACK"]
    coating_defects = [r for r in all_records if r["type"] == "COATING_FAILURE"]
    concrete_defects = [r for r in all_records
                       if r["type"] in CONCRETE_DEFECT_TYPES]

    negatives = []
    negatives += sample_sound_bolts(bolt_defects, 300, rnd, log=log)
    negatives += sample_offset_negatives("SOUND_CONCRETE", concrete_defects,
                                         120, rnd, log=log)
    negatives += sample_offset_negatives("SOUND_WELD", weld_defects, 40, rnd,
                                         offset_range=(0.3, 0.7), log=log)
    negatives += sample_offset_negatives("SOUND_COATING", coating_defects,
                                         40, rnd, offset_range=(0.2, 0.5),
                                         log=log)
    negatives += sample_sound_bearings(bearing_defects, 20, rnd, log=log)
    return negatives




# ===========================================================================
# 3. main -- prep only: ray-cast, assign ids, save. NO rendering here.
# ===========================================================================
NEGATIVES_JSON = os.path.join(os.path.dirname(BLEND_PATH), "..", "dataset",
                              "AVIAN_dataset_negatives_FINAL.json")
NEGATIVES_JSON = os.path.normpath(NEGATIVES_JSON)


def main():
    t_start = time.time()
    limit = None
    for a in sys.argv:
        if a.startswith("--limit="):
            limit = int(a.split("=", 1)[1])

    log("== SIH_AVIAN_FINAL :: dataset_final.py (Phase 2 step 1, prep) ==")
    log(f"  sensor  : hfov={HFOV_DEG} deg, LOOP={LOOP_RES} "
        f"({LOOP_GSD_MM_AT_1M:.4f} mm/px @1m), CAPTURE={CAPTURE_RES} "
        f"({CAPTURE_GSD_MM_AT_1M:.4f} mm/px @1m)")
    assert abs(LOOP_GSD_MM_AT_1M - PF.GSD_MM_PER_PX_AT_1M) < 0.001, (
        f"LOOP GSD {LOOP_GSD_MM_AT_1M} != stated sensor figure "
        f"{PF.GSD_MM_PER_PX_AT_1M} -- the resolvability argument and this "
        f"dataset would be measuring two different cameras")

    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(os.path.dirname(NEGATIVES_JSON), exist_ok=True)

    all_records = _load_records()
    if limit:
        all_records = all_records[:limit]
        log(f"  LIMIT   : truncated to {len(all_records)} defects for a "
            f"test batch (--limit={limit})")
    assign_instance_ids(all_records, log=log)

    rnd = random.Random(PF.SEED + 424242)
    negatives = build_negatives(all_records, rnd, log=log) if not limit \
        else []

    with open(NEGATIVES_JSON, "w") as f:
        json.dump({"negatives": negatives, "limit": limit}, f, indent=2)
    log(f"  negatives: {len(negatives)} total -> {NEGATIVES_JSON}")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH)
    log(f"  saved   : {BLEND_PATH} (instance ids persisted on defect objects)")

    stats = {"defects": len(all_records), "negatives": len(negatives),
             "wall_s": round(time.time() - t_start, 1)}
    with open(os.path.join(DATASET_DIR, "AVIAN_dataset_prep_stats_FINAL.json"),
             "w") as f:
        json.dump(stats, f, indent=2)
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_prep_build_log_FINAL.txt"), "w") as f:
        f.write("\n".join(LOG_LINES))

    log(f"== prep done in {stats['wall_s']} s -- now run "
        f"dataset_render_final.py in a FRESH process ==")
    print("FINAL_DATASET_PREP_COMPLETE")


if __name__ == "__main__":
    main()

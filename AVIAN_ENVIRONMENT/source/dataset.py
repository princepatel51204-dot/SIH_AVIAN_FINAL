"""Synthetic dataset generation support.

WHAT THIS DOES AND DOES NOT DO
------------------------------
It prepares the scene so that RGB, depth, surface-normal, object-index and
per-defect instance data can be rendered whenever they are wanted. It does not
render a dataset here: a full multi-condition sweep is hours of CPU Cycles, and
which conditions to sweep is a research decision, not a build-time one.

INSTANCE IDENTITY
-----------------
Every defect gets a unique `pass_index` in 1..192, matching the numeric part of
its ID, and structural members get indices in a separate high band. The Object
Index render pass then IS the instance mask -- no colour-matching, no
post-hoc association, no ambiguity when two defects of the same class sit on
the same girder. That last case is exactly where a naive class-mask dataset
silently merges two labels into one.

WHY OBJECT INDEX AND NOT CRYPTOMATTE
------------------------------------
Cryptomatte is better for coverage-accurate mattes of overlapping transparent
things, which is a real consideration for crack decals. But it needs a
manifest decode step and multiple layers, and the index pass is exact for the
opaque geometry defects and adequate for decals that do not overlap each other.
The trade is stated rather than assumed; `enable_cryptomatte` turns it on for
work where decal coverage matters.
"""
from __future__ import annotations
import json
import os

import bpy

import params as P


# pass_index bands. Kept far apart so a mask can be split by range alone.
DEFECT_INDEX_BASE = 0          # defects occupy 1..999
STRUCTURE_INDEX_BASE = 1000    # members occupy 1000..8999
DYNAMIC_INDEX_BASE = 9000      # dynamic content occupies 9000+

CLASS_INDEX = {
    "CRACK_HAIRLINE": 1, "CRACK_LONGITUDINAL": 2, "CRACK_TRANSVERSE": 3,
    "CRACK_DIAGONAL": 4, "CRACK_NETWORK": 5, "SPALL": 6,
    "DELAMINATION": 7, "REBAR_EXPOSED": 8, "CORROSION_STAIN": 9,
    "JOINT_DETERIORATION": 10, "REPAIR_PATCH": 11,
}


def assign_indices(records, log=print):
    """Give every defect, member and dynamic object a stable pass index."""
    n_def = n_str = n_dyn = 0
    used = set()

    for i, rec in enumerate(records, start=1):
        ob = bpy.data.objects.get(rec["defect_id"])
        if ob is None:
            continue
        idx = DEFECT_INDEX_BASE + i
        ob.pass_index = idx
        ob["avi_instance_id"] = idx
        ob["avi_class_index"] = CLASS_INDEX.get(rec["type"], 0)
        rec["instance_id"] = idx
        rec["class_index"] = CLASS_INDEX.get(rec["type"], 0)
        used.add(idx)
        n_def += 1
        # child rebar shares the parent's instance so a bar is not counted
        # as a separate defect
        for ch in bpy.data.objects:
            if ch.name.startswith(rec["defect_id"] + "_BAR"):
                ch.pass_index = idx
                ch["avi_instance_id"] = idx

    s = STRUCTURE_INDEX_BASE
    for ob in bpy.data.objects:
        if ob.type != "MESH" or not ob.name.startswith("BR_"):
            continue
        s += 1
        ob.pass_index = s
        ob["avi_instance_id"] = s
        n_str += 1

    d = DYNAMIC_INDEX_BASE
    for ob in bpy.data.objects:
        if ob.get("avi_object_type") not in ("DYNAMIC",):
            continue
        d += 1
        ob.pass_index = d
        ob["avi_instance_id"] = d
        n_dyn += 1

    log(f"  indices : {n_def} defect, {n_str} structural, {n_dyn} dynamic "
        f"instance IDs")
    return {"defects": n_def, "structure": n_str, "dynamic": n_dyn}


def enable_passes(cryptomatte=False, log=print):
    """Turn on the render passes a dataset needs."""
    vl = bpy.context.view_layer
    vl.use_pass_combined = True
    vl.use_pass_z = True
    vl.use_pass_normal = True
    vl.use_pass_object_index = True
    for attr in ("use_pass_position", "use_pass_diffuse_color",
                 "use_pass_mist"):
        if hasattr(vl, attr):
            setattr(vl, attr, True)
    if cryptomatte:
        for attr in ("use_pass_cryptomatte_object",
                     "use_pass_cryptomatte_asset"):
            if hasattr(vl, attr):
                setattr(vl, attr, True)
    # Mist is what makes a usable normalised depth preview; set its range to
    # the corridor rather than Blender's 25 m default, which would clip
    # everything past the next pier to solid white.
    w = bpy.context.scene.world
    if w is not None and hasattr(w, "mist_settings"):
        w.mist_settings.start = 1.0
        w.mist_settings.depth = 400.0

    on = [a for a in ("use_pass_combined", "use_pass_z", "use_pass_normal",
                      "use_pass_object_index", "use_pass_position",
                      "use_pass_mist")
          if getattr(vl, a, False)]
    log(f"  passes  : {len(on)} render passes enabled"
        + (" + cryptomatte" if cryptomatte else ""))
    return on


# ---------------------------------------------------------------------------
# CAPTURE CONFIGURATIONS
# The seven inspection geometries a dataset should cover, expressed as a
# standoff and an approach rule rather than as fixed coordinates, so they can
# be applied to any defect in the ground truth.
# ---------------------------------------------------------------------------
CAPTURE_CONFIGS = [
    {"name": "NORMAL_INSPECTION", "standoff_m": 2.5, "obliquity_deg": 15,
     "lens_mm": 24.0, "purpose": "nominal standoff, near-normal incidence"},
    {"name": "CLOSE_INSPECTION", "standoff_m": 0.8, "obliquity_deg": 10,
     "lens_mm": 35.0,
     "purpose": "the range at which a hairline crack spans enough pixels"},
    {"name": "OBLIQUE_INSPECTION", "standoff_m": 2.0, "obliquity_deg": 62,
     "lens_mm": 24.0,
     "purpose": "grazing view; foreshortened, but the only geometry "
                "available for some confined surfaces"},
    {"name": "UNDERSIDE_INSPECTION", "standoff_m": 1.6, "obliquity_deg": 25,
     "lens_mm": 20.0,
     "purpose": "looking UP at a soffit, which is where a UAV has no sensor"},
    {"name": "PIER_INSPECTION", "standoff_m": 3.0, "obliquity_deg": 20,
     "lens_mm": 28.0, "purpose": "column face at orbit radius"},
    {"name": "DIFFICULT_OCCLUSION", "standoff_m": 1.2, "obliquity_deg": 70,
     "lens_mm": 20.0,
     "purpose": "the narrow reachable cone into a bearing seat or bay"},
    {"name": "LONG_RANGE_SCREENING", "standoff_m": 25.0, "obliquity_deg": 30,
     "lens_mm": 85.0,
     "purpose": "triage pass; most defects are BELOW detection here and the "
                "dataset should contain those negatives"},
]


def export_manifest(records, path, index_stats, passes):
    """Everything a dataset generator needs, without generating one."""
    per_class = {}
    for r in records:
        per_class[r["type"]] = per_class.get(r["type"], 0) + 1
    diff = {}
    for r in records:
        k = r.get("detection_difficulty", "UNKNOWN")
        diff[k] = diff.get(k, 0) + 1

    out = {
        "note": "the scene is prepared for dataset generation; no dataset "
                "is rendered here. Sweeping conditions is a research "
                "decision and hours of CPU render time.",
        "instance_id_scheme": {
            "defects": f"{DEFECT_INDEX_BASE + 1}..{DEFECT_INDEX_BASE + 999}",
            "structure": f"{STRUCTURE_INDEX_BASE}..{DYNAMIC_INDEX_BASE - 1}",
            "dynamic": f"{DYNAMIC_INDEX_BASE}+",
            "mechanism": "Blender pass_index, read back through the Object "
                         "Index render pass",
        },
        "class_index": CLASS_INDEX,
        "counts": index_stats,
        "render_passes": passes,
        "available_outputs": [
            "RGB (Combined)", "Depth (Z)", "Surface normal (Normal)",
            "Object/instance ID (IndexOB)", "World position (Position)",
            "Normalised depth preview (Mist, 1-400 m)",
            "Per-defect binary mask (threshold IndexOB to one instance)",
            "Per-class mask (map IndexOB through class_index)",
            "2-D bounding box (extent of the thresholded instance mask)",
        ],
        "capture_configs": CAPTURE_CONFIGS,
        "lighting_conditions_available": [
            "DAY_CLEAR", "DAY_OVERCAST", "MORNING", "EVENING", "NIGHT",
            "UNDERBRIDGE", "DEEP_SHADOW"],
        "defects_per_class": per_class,
        "defects_per_difficulty": diff,
        "recommended_split_note":
            "split by SECTOR, never at random. Defects in one sector share "
            "host members, lighting and surface finish, so a random split "
            "leaks the test set into training through the background.",
        "defect_index": [
            {"defect_id": r["defect_id"], "instance_id": r.get("instance_id"),
             "class_index": r.get("class_index"), "type": r["type"],
             "severity": r["severity"],
             "sector": r["inspection_sector"],
             "detection_difficulty": r.get("detection_difficulty"),
             "expected_rgb_visibility": r.get("expected_rgb_visibility"),
             "max_useful_range_m": r.get("max_useful_range_m")}
            for r in records],
    }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return {"classes": len(CLASS_INDEX), "configs": len(CAPTURE_CONFIGS)}

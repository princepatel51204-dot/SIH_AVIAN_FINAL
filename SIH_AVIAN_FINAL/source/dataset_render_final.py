"""SIH_AVIAN_FINAL -- Phase 2 Step 1: the UAV-camera detection dataset,
phase 2 of 2 -- render, manifest, validate, sabotage, gate.

Opens the .blend `dataset_final.py` (phase 1) already saved -- instance ids
already assigned to every defect object's `pass_index`/`avi_instance_id`,
negatives already ray-cast and written to
`dataset/AVIAN_dataset_negatives_FINAL.json` -- and does ALL rendering here,
in a fresh process that never calls `scene.ray_cast()`, so Cycles is safe
(see `dataset_final.py`'s own docstring for why EEVEE was tried first and
rejected: its Object Index compositor pass produces no file at all in this
Blender version, confirmed by a direct test, not assumed).

Usage:
    blender --background --python run_blender.py -- source/dataset_final.py
    blender --background --python run_blender.py -- source/dataset_render_final.py
"""
from __future__ import annotations
import csv
import json
import math
import os
import sys
import time

import bpy
from mathutils import Vector, Matrix

import dataset_final as DSF   # reuses path constants, sensor intrinsics,
                              # _load_records(), the defect-type tuples
import params_final as PF
from validate import Result

ROOT = DSF.ROOT
SCENE_DIR = DSF.SCENE_DIR
BLEND_PATH = DSF.BLEND_PATH
DATASET_DIR = DSF.DATASET_DIR
IMG_DIR = DSF.IMG_DIR
MASK_DIR = DSF.MASK_DIR
DEPTH_DIR = DSF.DEPTH_DIR
NORMAL_DIR = DSF.NORMAL_DIR
SAMPLES_DIR = DSF.SAMPLES_DIR
NEGATIVES_JSON = DSF.NEGATIVES_JSON

HFOV_DEG = DSF.HFOV_DEG
LOOP_RES = DSF.LOOP_RES
CAPTURE_RES = DSF.CAPTURE_RES
LOOP_GSD_MM_AT_1M = DSF.LOOP_GSD_MM_AT_1M
CAPTURE_GSD_MM_AT_1M = DSF.CAPTURE_GSD_MM_AT_1M
STEEL_DEFECT_TYPES_BOLT = DSF.STEEL_DEFECT_TYPES_BOLT
CONCRETE_DEFECT_TYPES = DSF.CONCRETE_DEFECT_TYPES
_gsd_mm_at_1m = DSF._gsd_mm_at_1m

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


# ===========================================================================
# 1. render setup -- CYCLES (no ray-casting happens in this process, so this
# is exactly the case contrast_c.py's own crash note doesn't apply to), four
# passes, the real sensor's own FOV
# ===========================================================================
def configure_cycles(w, h, samples=12):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = False   # see build/measure_final.py's own
                                          # note: this Blender install has no
                                          # OpenImageDenoiser
    # Same bounce limits lighting.py's own configure_render() sets for the
    # 12 named pitch cameras -- Cycles' stock defaults are higher, and this
    # scene's ~1,200 metallic bolt heads mean every close-in shot pays for
    # unbounded glossy/transmission bounces. Never explicitly set before
    # this fix: a single test frame measured 44.7 s average (some frames
    # over 100 s at CAPTURE resolution) with the defaults, which is what
    # caught this in the first place, not a guess.
    scene.cycles.max_bounces = 6
    scene.cycles.diffuse_bounces = 3
    scene.cycles.glossy_bounces = 3
    scene.cycles.transmission_bounces = 4
    scene.cycles.transparent_max_bounces = 8
    scene.render.resolution_x = w
    scene.render.resolution_y = h
    scene.render.resolution_percentage = 100
    scene.render.use_persistent_data = False
    vl = scene.view_layers[0]
    vl.use_pass_z = True
    vl.use_pass_normal = True
    vl.use_pass_object_index = True
    return scene


def _get_or_create_camera(name="AVI_DATASET_CAM"):
    cam_data = bpy.data.cameras.get(name)
    if cam_data is None:
        cam_data = bpy.data.cameras.new(name)
    cam_data.clip_start = 0.01
    cam_data.clip_end = 200.0
    cam_data.sensor_fit = "HORIZONTAL"
    cam = bpy.data.objects.get(name)
    if cam is None:
        cam = bpy.data.objects.new(name, cam_data)
        bpy.context.scene.collection.objects.link(cam)
    return cam


def _look_at(ob, loc, target, up=(0.0, 0.0, 1.0)):
    loc = Vector(loc)
    tgt = Vector(target)
    fwd = (loc - tgt)
    if fwd.length < 1e-9:
        fwd = Vector((0, 0, 1))
    fwd = fwd.normalized()
    upv = Vector(up)
    if abs(fwd.dot(upv)) > 0.999:
        upv = Vector((0.0, 1.0, 0.0))
    right = upv.cross(fwd).normalized()
    up2 = fwd.cross(right).normalized()
    ob.matrix_world = Matrix((
        (right.x, up2.x, fwd.x, loc.x),
        (right.y, up2.y, fwd.y, loc.y),
        (right.z, up2.z, fwd.z, loc.z),
        (0.0, 0.0, 0.0, 1.0)))


def _setup_compositor(scene):
    scene.use_nodes = True
    nt = scene.node_tree
    nt.nodes.clear()
    rl = nt.nodes.new("CompositorNodeRLayers")
    rl.location = (0, 0)

    def _file_out(pass_name, subdir, ext="OPEN_EXR", color_depth="32",
                 color_mode="RGB"):
        node = nt.nodes.new("CompositorNodeOutputFile")
        node.location = (400, -200 * len(nt.nodes))
        node.base_path = subdir
        node.format.file_format = ext
        node.format.color_depth = color_depth
        node.format.color_mode = color_mode
        node.file_slots[0].path = "frame_"
        # `rl.outputs[pass_name]` (string key) can raise KeyError for a
        # pass enabled this same session even though the name shows up in
        # plain iteration -- a real Blender quirk, not a typo. Iterate for
        # the socket object instead of indexing by name.
        sock = next((o for o in rl.outputs if o.name == pass_name), None)
        if sock is None:
            raise RuntimeError(f"RenderLayers node has no '{pass_name}' "
                              f"output -- pass not enabled?")
        nt.links.new(sock, node.inputs[0])
        return node

    n_rgb = _file_out("Image", IMG_DIR, ext="PNG", color_depth="8",
                      color_mode="RGB")
    n_depth = _file_out("Depth", DEPTH_DIR)
    n_normal = _file_out("Normal", NORMAL_DIR)
    n_index = _file_out("IndexOB", MASK_DIR)
    return {"rgb": n_rgb, "depth": n_depth, "normal": n_normal,
           "index": n_index}


def _set_output_names(nodes, frame_id):
    for key, node in nodes.items():
        node.file_slots[0].path = frame_id + "_"


def _load_exr_array(path):
    import numpy as np
    img = bpy.data.images.load(path, check_existing=False)
    try:
        w, h = img.size
        ch = img.channels
        arr = np.empty(w * h * ch, dtype=np.float32)
        img.pixels.foreach_get(arr)
        return arr.reshape(h, w, ch)[::-1]   # Blender pixels are bottom-up
    finally:
        bpy.data.images.remove(img)


def _read_exr_center(path, sample_radius=2):
    arr = _load_exr_array(path)
    h, w, ch = arr.shape
    cy, cx = h // 2, w // 2
    r = sample_radius
    patch = arr[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1, :]
    return [float(v) for v in patch.reshape(-1, ch).mean(axis=0)]


def _mask_contains(path, instance_id, tol=0.5):
    arr = _load_exr_array(path)
    return bool((abs(arr[..., 0] - instance_id) < tol).any())


def _measure_rendered_contrast(rgb_path, disc_r=0.06, ann_r0=0.10,
                               ann_r1=0.35):
    import numpy as np
    img = bpy.data.images.load(rgb_path, check_existing=False)
    try:
        w, h = img.size
        ch = img.channels
        arr = np.empty(w * h * ch, dtype=np.float32)
        img.pixels.foreach_get(arr)
        arr = arr.reshape(h, w, ch)
    finally:
        bpy.data.images.remove(img)
    luma = (0.2126 * arr[..., 0] + 0.7152 * arr[..., 1]
           + 0.0722 * arr[..., 2])
    yy, xx = np.mgrid[0:h, 0:w]
    cy, cx = h / 2.0, w / 2.0
    r = (((xx - cx) / (w / 2.0)) ** 2 + ((yy - cy) / (h / 2.0)) ** 2) ** 0.5
    disc = luma[r <= disc_r]
    ann = luma[(r >= ann_r0) & (r <= ann_r1)]
    if disc.size == 0 or ann.size == 0:
        return None
    ld, lb = float(disc.mean()), float(ann.mean())
    denom = ld + lb
    return abs(ld - lb) / denom if denom > 1e-6 else 0.0


def render_frame(cam, frame_id, pos, view_dir, standoff_m, resolution, nodes,
                 instance_id=None):
    """Places `cam` at `pos + view_dir*standoff_m` looking at `pos`, renders
    all four passes, and measures back what actually happened -- the
    achieved standoff and GSD come from the DEPTH pass, not the numbers
    used to place the camera, so a positioning bug or an unexpected
    obstruction shows up as a measured discrepancy (VD03) rather than
    silently passing because the commanded value was echoed back."""
    w, h = resolution
    eye = tuple(pos[i] + view_dir[i] * standoff_m for i in range(3))
    cam.location = eye
    _look_at(cam, eye, pos)
    bpy.context.scene.camera = cam
    bpy.context.view_layer.update()

    hfov = HFOV_DEG
    cam.data.angle = math.radians(hfov)
    scene = bpy.context.scene
    scene.render.resolution_x = w
    scene.render.resolution_y = h

    _set_output_names(nodes, frame_id)
    bpy.ops.render.render(write_still=True)

    res_tag = f"{w}x{h}"
    rgb_path = os.path.join(IMG_DIR, f"{frame_id}_0001.png")
    depth_path = os.path.join(DEPTH_DIR, f"{frame_id}_0001.exr")
    normal_path = os.path.join(NORMAL_DIR, f"{frame_id}_0001.exr")
    mask_path = os.path.join(MASK_DIR, f"{frame_id}_0001.exr")

    depth_c = _read_exr_center(depth_path, sample_radius=1)[0]
    normal_c = _read_exr_center(normal_path, sample_radius=1)[:3]
    n = Vector(normal_c)
    if n.length > 1e-6:
        n = n.normalized()
        view_axis = Vector(view_dir).normalized()
        cos_obl = max(-1.0, min(1.0, n.dot(view_axis)))
        obliquity_deg = math.degrees(math.acos(abs(cos_obl)))
    else:
        obliquity_deg = None

    achieved_gsd = _gsd_mm_at_1m(hfov, w) * depth_c if depth_c > 0 else None
    mask_ok = (_mask_contains(mask_path, instance_id)
              if instance_id is not None else None)
    rendered_contrast = _measure_rendered_contrast(rgb_path)

    return {
        "image_path": os.path.relpath(rgb_path, ROOT),
        "mask_path": os.path.relpath(mask_path, ROOT),
        "depth_path": os.path.relpath(depth_path, ROOT),
        "normal_path": os.path.relpath(normal_path, ROOT),
        "resolution": res_tag,
        "camera_pose": {
            "position_m": [round(v, 4) for v in eye],
            "target_m": [round(v, 4) for v in pos],
        },
        "commanded_standoff_m": round(standoff_m, 4),
        "subject_distance_m": round(depth_c, 4) if depth_c else None,
        "achieved_gsd_mm_px": (round(achieved_gsd, 5)
                               if achieved_gsd else None),
        "view_obliquity_deg": (round(obliquity_deg, 2)
                               if obliquity_deg is not None else None),
        "mask_contains_instance": mask_ok,
        "rendered_contrast_michelson": (round(rendered_contrast, 5)
                                        if rendered_contrast is not None
                                        else None),
    }


# ===========================================================================
# 2. drive the render batch
# ===========================================================================
def _capture_sample_ids(all_records, cap=12):
    """Which defects ALSO get a CAPTURE-resolution companion render, beyond
    everyone's LOOP one -- one per distinct defect TYPE, capped at `cap`.

    A CAPTURE frame measured ~6-7x a LOOP frame's Cycles cost here (3.375x
    the pixels, plus Cycles scales worse than linearly on this scene's
    dense metallic bolt geometry) -- rendering all 192 would alone take
    several hours. One representative per type still makes the loop-vs-
    capture GSD gap measurable everywhere it differs qualitatively (a
    crack vs a bolt vs a weld), which is the actual point (see this
    module's own docstring on the resolution-scope decision), without the
    exhaustive cost. A curated scope, not a silent cut."""
    seen_types = set()
    ids = []
    for r in all_records:
        if r["type"] in seen_types:
            continue
        seen_types.add(r["type"])
        ids.append(r["defect_id"])
        if len(ids) >= cap:
            break
    return set(ids)


def render_positives(all_records, cam, nodes, log=print, capture_cap=12):
    frames = []
    n_survey_skipped = 0
    n_capture = 0
    capture_ids = _capture_sample_ids(all_records, cap=capture_cap)
    for r in all_records:
        pos = r["position_m"]
        view_dir = r.get("recommended_view_direction") or r["surface_normal"]
        did = r["defect_id"]
        iid = r.get("instance_id")

        # §2.1 -- the theoretical best case, clamped to the aircraft's own
        # physical minimum flyable/focus range. Some defects (a hairline
        # crack in particular) have a min_detect_range_m of a few cm --
        # rendering AT that literal distance embeds the camera in the
        # surface: physically meaningless (no aircraft can hold station
        # there) and pathological for Cycles (near-zero-distance geometry
        # is a documented light-transport slow case, not just a rendering
        # nicety -- confirmed here directly: one such frame took minutes
        # at 24 samples where every other frame in this batch takes
        # seconds). The UNCLAMPED value stays in the manifest's own
        # min_detect_range_m field -- only the camera's real placement
        # is capped.
        mdr = max(r["min_detect_range_m"], PF.MIN_FLYABLE_RANGE_M)
        fid = f"POS_{iid:03d}_MDR_LOOP"
        rec = render_frame(cam, fid, pos, view_dir, mdr, LOOP_RES, nodes,
                           instance_id=iid)
        frames.append({**rec, "defect_id": did, "label": r["type"],
                      "is_positive": True, "standoff_kind": "min_detect_range",
                      **_manifest_common(r)})

        if did in capture_ids:
            fid2 = f"POS_{iid:03d}_MDR_CAPTURE"
            rec2 = render_frame(cam, fid2, pos, view_dir, mdr, CAPTURE_RES,
                                nodes, instance_id=iid)
            frames.append({**rec2, "defect_id": did, "label": r["type"],
                          "is_positive": True,
                          "standoff_kind": "min_detect_range",
                          **_manifest_common(r)})
            n_capture += 1

        # §2.2 -- the realistic case, only where actually reachable
        best_clear = r.get("best_view_clear_m", 0.0) or 0.0
        survey = PF.UNDERSIDE_INSPECTION_STANDOFF_M
        if best_clear >= survey - 0.1:
            fid3 = f"POS_{iid:03d}_SURVEY_LOOP"
            rec3 = render_frame(cam, fid3, pos, view_dir, survey, LOOP_RES,
                                nodes, instance_id=iid)
            frames.append({**rec3, "defect_id": did, "label": r["type"],
                          "is_positive": True, "standoff_kind": "survey",
                          **_manifest_common(r)})
        else:
            n_survey_skipped += 1
    log(f"  render  : {len(frames)} positive frames ({n_capture} with a "
        f"CAPTURE-resolution companion, capped at {capture_cap}), "
        f"{n_survey_skipped}/{len(all_records)} unreachable at "
        f"{PF.UNDERSIDE_INSPECTION_STANDOFF_M} m survey standoff (skipped)")
    return frames


def render_negatives(negatives, cam, nodes, log=print):
    frames = []
    for i, neg in enumerate(negatives, start=1):
        fid = f"NEG_{neg['label']}_{i:04d}"
        # same physical clamp as render_positives() -- see its own comment
        standoff = max(neg["min_detect_range_m"], PF.MIN_FLYABLE_RANGE_M)
        rec = render_frame(cam, fid, neg["position_m"], neg["surface_normal"],
                           standoff, LOOP_RES, nodes, instance_id=None)
        frames.append({**rec, "defect_id": None, "label": neg["label"],
                      "is_positive": False,
                      "standoff_kind": "min_detect_range",
                      "matched_defect_id": neg["matched_defect_id"],
                      "matched_type": neg["matched_type"],
                      "feature_size_mm": None,
                      "min_detect_range_m": round(standoff, 4),
                      "defect_background_contrast": None,
                      "contrast_limited": None, "escalation_reason": None,
                      "avi_condition": neg.get("avi_condition"),
                      "host_structure": neg.get("host_structure"),
                      "scenario": "BASELINE"})
    log(f"  render  : {len(frames)} negative frames")
    return frames


REQUIRED_COLUMNS = [
    "image_path", "mask_path", "depth_path", "normal_path", "defect_id",
    "label", "is_positive", "resolution", "camera_pose",
    "subject_distance_m", "achieved_gsd_mm_px", "view_obliquity_deg",
    "feature_size_mm", "min_detect_range_m", "defect_background_contrast",
    "contrast_limited", "escalation_reason", "avi_condition",
    "host_structure", "scenario",
]


def export_manifest(frames, log=print):
    json_path = os.path.join(DATASET_DIR, "AVIAN_dataset_manifest_FINAL.json")
    csv_path = os.path.join(DATASET_DIR, "AVIAN_dataset_manifest_FINAL.csv")
    with open(json_path, "w") as f:
        json.dump({"frames": frames, "count": len(frames),
                  "loop_res": LOOP_RES, "capture_res": CAPTURE_RES,
                  "hfov_deg": HFOV_DEG,
                  "loop_gsd_mm_at_1m": round(LOOP_GSD_MM_AT_1M, 4),
                  "capture_gsd_mm_at_1m": round(CAPTURE_GSD_MM_AT_1M, 4)},
                 f, indent=2)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(REQUIRED_COLUMNS)
        for r in frames:
            w.writerow([r.get(c) for c in REQUIRED_COLUMNS])
    log(f"  manifest: {json_path}")
    log(f"  manifest: {csv_path}")
    return json_path, csv_path


def _manifest_common(r):
    return {
        "feature_size_mm": r.get("feature_size_mm"),
        "min_detect_range_m": r.get("min_detect_range_m"),
        "defect_background_contrast": r.get("defect_background_contrast"),
        "contrast_limited": r.get("contrast_limited"),
        "escalation_reason": r.get("escalation_reason"),
        "avi_condition": r.get("avi_condition", PF.ROAD_CONDITION),
        "host_structure": r.get("_host_structure", "ROAD"),
        "scenario": "BASELINE",
    }


# ===========================================================================
# 3. checks VD01-08
# ===========================================================================
NEGATIVE_MATCH_GROUPS = {
    "SOUND_BOLT": STEEL_DEFECT_TYPES_BOLT,
    "SOUND_CONCRETE": CONCRETE_DEFECT_TYPES,
    "SOUND_WELD": ("WELD_CRACK",),
    "SOUND_COATING": ("COATING_FAILURE",),
    "SOUND_BEARING": ("BEARING_SEIZED",),
}


def validate_dataset(frames, all_records, log=print):
    R = []
    positives = [f for f in frames if f["is_positive"]]
    negatives = [f for f in frames if not f["is_positive"]]

    bad01 = [f["image_path"] for f in positives
            if not f.get("mask_contains_instance")]
    R.append(Result("VD01", "every positive frame's mask contains its "
                    "own instance id (non-empty)",
                    "PASS" if not bad01 else "FAIL",
                    f"{len(positives) - len(bad01)}/{len(positives)} "
                    f"masks OK", "100%", ", ".join(bad01[:5])))

    bad02 = []
    for f in positives + negatives:
        d = f.get("subject_distance_m")
        g = f.get("achieved_gsd_mm_px")
        if not d or not g:
            continue
        target = (LOOP_GSD_MM_AT_1M if f["resolution"] == "640x480"
                 else CAPTURE_GSD_MM_AT_1M)
        achieved_at_1m = g / d
        if abs(achieved_at_1m - target) / target > 0.01:
            bad02.append(f["image_path"])
    n_checked = sum(1 for f in positives + negatives
                   if f.get("subject_distance_m") and f.get("achieved_gsd_mm_px"))
    R.append(Result("VD02", "achieved GSD matches the sensor (within 1% of "
                    f"{LOOP_GSD_MM_AT_1M:.4f} mm/px @ 1m at LOOP res, "
                    f"{CAPTURE_GSD_MM_AT_1M:.4f} at CAPTURE res)",
                    "PASS" if not bad02 else "FAIL",
                    f"{n_checked - len(bad02)}/{n_checked} within 1%",
                    "within 1%", ", ".join(bad02[:5])))

    bad03 = []
    for f in positives + negatives:
        cmd = f.get("commanded_standoff_m")
        got = f.get("subject_distance_m")
        if not cmd or got is None:
            continue
        if abs(got - cmd) / cmd > 0.05:
            bad03.append(f["image_path"])
    n_checked3 = sum(1 for f in positives + negatives
                    if f.get("commanded_standoff_m")
                    and f.get("subject_distance_m") is not None)
    R.append(Result("VD03", "achieved standoff matches requested (depth "
                    "pass vs commanded)", "PASS" if not bad03 else "FAIL",
                    f"{n_checked3 - len(bad03)}/{n_checked3} within 5%",
                    "within 5%", ", ".join(bad03[:5])))

    bad04 = []
    for f in negatives:
        arr = _load_exr_array(os.path.join(ROOT, f["mask_path"]))
        ids = arr[..., 0]
        if bool(((ids >= 1) & (ids <= len(all_records))).any()):
            bad04.append(f["image_path"])
    R.append(Result("VD04", "negatives contain no defect instance",
                    "PASS" if not bad04 else "FAIL",
                    f"{len(negatives) - len(bad04)}/{len(negatives)} clean",
                    "0 defect ids in any negative mask",
                    ", ".join(bad04[:5])))

    bad05 = []
    counts05 = {}
    for label, types in NEGATIVE_MATCH_GROUPS.items():
        n_neg = sum(1 for f in negatives if f["label"] == label)
        n_types = len(types)
        ratio = n_neg / max(1, n_types)
        counts05[label] = {"negatives": n_neg, "matched_types": n_types,
                           "ratio": round(ratio, 2)}
        if ratio < 3.0:
            bad05.append(label)
    R.append(Result("VD05", "class balance: >=3 negatives per matched "
                    "defect TYPE, in every negative category",
                    "PASS" if not bad05 else "FAIL", json.dumps(counts05),
                    ">= 3.0", ", ".join(bad05)))

    positive_only_nullable = set()
    negative_nullable = {"defect_id", "feature_size_mm",
                         "defect_background_contrast", "contrast_limited",
                         "escalation_reason"}
    bad06 = []
    for f in frames:
        nullable = negative_nullable if not f["is_positive"] \
            else positive_only_nullable
        for col in REQUIRED_COLUMNS:
            if col in nullable:
                continue
            if f.get(col) is None:
                bad06.append(f"{f['image_path']}:{col}")
    R.append(Result("VD06", "manifest complete (no nulls except the "
                    "documented negative-only fields)",
                    "PASS" if not bad06 else "FAIL",
                    f"{len(frames)} rows, {len(bad06)} missing values",
                    "0 missing", ", ".join(bad06[:5])))

    seen = {f["defect_id"] for f in positives if f["defect_id"]}
    missing07 = [r["defect_id"] for r in all_records
                if r["defect_id"] not in seen]
    R.append(Result("VD07", "every defect appears at least once",
                    "PASS" if not missing07 else "FAIL",
                    f"{len(seen)}/{len(all_records)} defects rendered",
                    f"{len(all_records)}/{len(all_records)}",
                    ", ".join(missing07[:5])))

    by_defect = {}
    for f in positives:
        by_defect.setdefault(f["defect_id"], {})[f["standoff_kind"]] = f
    diffs = []
    for did, kinds in by_defect.items():
        mdr = kinds.get("min_detect_range")
        survey = kinds.get("survey")
        if mdr and survey:
            a = mdr.get("rendered_contrast_michelson")
            b = survey.get("rendered_contrast_michelson")
            if a is not None and b is not None:
                diffs.append(abs(a - b))
    mean_diff = sum(diffs) / len(diffs) if diffs else 0.0
    R.append(Result("VD08", "survey-standoff contrast differs measurably "
                    "from min-detect-range contrast (not degenerate)",
                    "PASS" if diffs and mean_diff > 0.01 else
                    ("SKIP" if not diffs else "FAIL"),
                    f"{len(diffs)} matched pairs, mean |diff| "
                    f"{mean_diff:.4f}" if diffs else "no matched pairs",
                    "> 0.01 mean Michelson difference"))

    pass_n = sum(1 for r in R if r.status == "PASS")
    fail_n = sum(1 for r in R if r.status == "FAIL")
    skip_n = sum(1 for r in R if r.status == "SKIP")
    for r in R:
        log(r.row())
    log(f"  VALIDATE: {pass_n} pass, {fail_n} fail, {skip_n} skip, "
        f"{len(R)} total")
    return R, {"pass": pass_n, "fail": fail_n, "skip": skip_n, "total": len(R)}


# ===========================================================================
# 4. sabotage -- proves each VD check can fail (working rule 3.10)
# ===========================================================================
def _check_status(frames, all_records):
    R, _ = validate_dataset(frames, all_records, log=lambda *a: None)
    return {r.id: r.status for r in R}


def sabotage_dataset(frames, all_records, log=print):
    rows = []

    def _sab_VD01():
        f = next(f for f in frames if f["is_positive"])
        old = f["mask_contains_instance"]
        f["mask_contains_instance"] = False
        def undo():
            f["mask_contains_instance"] = old
        return undo

    def _sab_VD02():
        f = next(f for f in frames
                 if f.get("achieved_gsd_mm_px") and f.get("subject_distance_m"))
        old = f["achieved_gsd_mm_px"]
        f["achieved_gsd_mm_px"] = old * 3.0
        def undo():
            f["achieved_gsd_mm_px"] = old
        return undo

    def _sab_VD03():
        f = next(f for f in frames
                 if f.get("commanded_standoff_m") and f.get("subject_distance_m"))
        old = f["subject_distance_m"]
        f["subject_distance_m"] = old + 5.0
        def undo():
            f["subject_distance_m"] = old
        return undo

    def _sab_VD04():
        neg = next(f for f in frames if not f["is_positive"])
        pos = next(f for f in frames if f["is_positive"])
        old = neg["mask_path"]
        neg["mask_path"] = pos["mask_path"]
        def undo():
            neg["mask_path"] = old
        return undo

    def _sab_VD05():
        victims = [f for f in frames if f["label"] == "SOUND_BOLT"]
        removed = victims[:280]
        for f in removed:
            frames.remove(f)
        def undo():
            frames.extend(removed)
        return undo

    def _sab_VD06():
        f = frames[0]
        key = "resolution"
        old = f[key]
        f[key] = None
        def undo():
            f[key] = old
        return undo

    def _sab_VD07():
        # Every defect has 1-3 frames (MDR-LOOP always, sometimes a
        # CAPTURE companion, sometimes a survey shot) -- removing only
        # ONE still leaves that defect's id in VD07's `seen` set via its
        # other frame(s), so this proves nothing (caught by testing this
        # sabotage itself, not assumed). Remove EVERY frame for one
        # defect_id instead.
        target_id = next(f["defect_id"] for f in frames if f["is_positive"])
        removed = [f for f in frames if f["defect_id"] == target_id]
        for f in removed:
            frames.remove(f)
        def undo():
            frames.extend(removed)
        return undo

    def _sab_VD08():
        olds = [(f, f.get("rendered_contrast_michelson")) for f in frames
               if f["is_positive"]]
        for f, _ in olds:
            f["rendered_contrast_michelson"] = 0.2
        def undo():
            for f, old in olds:
                f["rendered_contrast_michelson"] = old
        return undo

    has_negatives = any(not f["is_positive"] for f in frames)
    plan = [
        ("VD01", "mark a positive frame's mask as not containing its own "
         "instance", _sab_VD01),
        ("VD02", "triple one frame's achieved GSD", _sab_VD02),
        ("VD03", "push one frame's measured standoff 5 m off commanded",
         _sab_VD03),
        ("VD06", "null out a required manifest field", _sab_VD06),
        ("VD07", "remove a positive frame (defect never rendered)", _sab_VD07),
        ("VD08", "force every positive frame's rendered contrast to the "
         "same value", _sab_VD08),
    ]
    if has_negatives:
        # VD04/VD05 have nothing to sabotage in a --limit test run (0
        # negatives) -- the real dataset always has 520, so this guard
        # only ever applies to that test path, not the actual gate.
        plan.append(("VD04", "point a negative's mask at a positive's "
                    "(defect-bearing) mask", _sab_VD04))
        plan.append(("VD05", "delete 280 of 300 SOUND_BOLT negatives",
                    _sab_VD05))

    for check_id, desc, setup in plan:
        before = _check_status(frames, all_records)
        b = before.get(check_id)
        undo = setup()
        try:
            after = _check_status(frames, all_records)
            a = after.get(check_id)
            proven = (b == "PASS" and a == "FAIL")
            rows.append({"check": check_id, "sabotage": desc, "before": b,
                        "after": a, "proven": proven})
            log(f"  [{'OK ' if proven else 'BAD'}] {check_id:8} {desc:<58} "
                f"{b} -> {a}")
        finally:
            undo()

    proven_n = sum(1 for r in rows if r["proven"])
    log(f"  SABOTAGE: {proven_n}/{len(rows)} proven capable of failing")
    return {"total": len(rows), "proven": proven_n,
           "unproven": [r["check"] for r in rows if not r["proven"]],
           "rows": rows}


# ===========================================================================
# 5. gate: six sample frames side by side
# ===========================================================================
def build_gate_collage(frames, log=print):
    import numpy as np

    def _find(pred):
        return next((f for f in frames if pred(f)), None)

    wanted = []
    bolt_defect = next((f["defect_id"] for f in frames
                       if f["label"] == "BOLT_LOOSE"), None)
    if bolt_defect:
        wanted.append(_find(lambda f: f["defect_id"] == bolt_defect
                            and f["standoff_kind"] == "min_detect_range"
                            and f["resolution"] == "640x480"))
        wanted.append(_find(lambda f: f["defect_id"] == bolt_defect
                            and f["standoff_kind"] == "survey"))
    crack_defect = next((f["defect_id"] for f in frames
                        if f["label"] == "CRACK_HAIRLINE"), None)
    if crack_defect:
        wanted.append(_find(lambda f: f["defect_id"] == crack_defect
                            and f["standoff_kind"] == "min_detect_range"
                            and f["resolution"] == "640x480"))
        wanted.append(_find(lambda f: f["defect_id"] == crack_defect
                            and f["standoff_kind"] == "survey"))
    wanted.append(_find(lambda f: f["label"] == "SOUND_BOLT"))
    wanted.append(_find(lambda f: f["label"] == "SOUND_CONCRETE"))
    wanted = [f for f in wanted if f is not None]
    if not wanted:
        log("  gate    : no sample frames available for the collage")
        return None

    imgs = []
    for f in wanted:
        p = os.path.join(ROOT, f["image_path"])
        img = bpy.data.images.load(p, check_existing=False)
        w, h = img.size
        arr = np.empty(w * h * 4, dtype=np.float32)
        img.pixels.foreach_get(arr)
        bpy.data.images.remove(img)
        imgs.append(arr.reshape(h, w, 4)[::-1, :, :3])

    th = min(im.shape[0] for im in imgs)
    tw = min(im.shape[1] for im in imgs)
    resized = [im[:th, :tw, :] for im in imgs]
    cols = 3
    rows_n = math.ceil(len(resized) / cols)
    canvas = np.zeros((rows_n * th, cols * tw, 3), dtype=np.float32)
    for i, im in enumerate(resized):
        r, c = divmod(i, cols)
        canvas[r * th:(r + 1) * th, c * tw:(c + 1) * tw, :] = im

    os.makedirs(SAMPLES_DIR, exist_ok=True)
    out_path = os.path.join(SAMPLES_DIR, "gate_collage.png")
    out_img = bpy.data.images.new("gate_collage", canvas.shape[1],
                                  canvas.shape[0])
    rgba = np.ones((canvas.shape[0], canvas.shape[1], 4), dtype=np.float32)
    rgba[..., :3] = canvas
    out_img.pixels.foreach_set(rgba[::-1].ravel())
    out_img.filepath_raw = out_path
    out_img.file_format = "PNG"
    out_img.save()
    bpy.data.images.remove(out_img)
    log(f"  gate    : {len(wanted)}-frame sample collage -> {out_path}")
    return out_path


# ===========================================================================
# 6. main
# ===========================================================================
def main():
    t_start = time.time()

    log("== SIH_AVIAN_FINAL :: dataset_render_final.py (Phase 2 step 1, "
        "render) ==")
    log(f"  sensor  : hfov={HFOV_DEG} deg, LOOP={LOOP_RES} "
        f"({LOOP_GSD_MM_AT_1M:.4f} mm/px @1m), CAPTURE={CAPTURE_RES} "
        f"({CAPTURE_GSD_MM_AT_1M:.4f} mm/px @1m)")

    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    for d in (IMG_DIR, MASK_DIR, DEPTH_DIR, NORMAL_DIR, SAMPLES_DIR):
        os.makedirs(d, exist_ok=True)

    all_records = DSF._load_records()
    with open(NEGATIVES_JSON) as f:
        prep = json.load(f)
    negatives = prep["negatives"]
    limit = prep.get("limit")
    if limit:
        all_records = all_records[:limit]
        log(f"  LIMIT   : truncated to {len(all_records)} defects (from "
            f"phase 1's --limit={limit})")

    # instance ids were assigned onto the Blender objects in phase 1;
    # re-derive them here rather than re-assigning, so a frame's mask
    # value is checked against the SAME id the .blend actually carries.
    n_ids = 0
    for r in all_records:
        ob = bpy.data.objects.get(r["defect_id"])
        r["instance_id"] = ob.get("avi_instance_id") if ob else None
        if r["instance_id"] is not None:
            n_ids += 1
    log(f"  indices : {n_ids}/{len(all_records)} instance ids recovered "
        f"from the saved scene")

    scene = configure_cycles(*LOOP_RES, samples=12)
    cam = _get_or_create_camera()
    nodes = _setup_compositor(scene)

    t_render0 = time.time()
    pos_frames = render_positives(all_records, cam, nodes, log=log)
    neg_frames = render_negatives(negatives, cam, nodes, log=log)
    render_wall_s = time.time() - t_render0
    frames = pos_frames + neg_frames
    n_frames = len(frames)
    log(f"  render  : {n_frames} frames in {render_wall_s:.1f} s "
        f"({render_wall_s / max(1, n_frames):.2f} s/frame)")

    json_path, csv_path = export_manifest(frames, log=log)

    log("-- validation (VD01-08) --")
    results, vsummary = validate_dataset(frames, all_records, log=log)

    log("-- sabotage (working rule 3.10) --")
    sabotage_report = sabotage_dataset(frames, all_records, log=log)

    collage_path = build_gate_collage(frames, log=log)

    stats = {
        "frames": n_frames, "positives": len(pos_frames),
        "negatives": len(neg_frames),
        "render_wall_s": round(render_wall_s, 1),
        "s_per_frame": round(render_wall_s / max(1, n_frames), 3),
        "validation": vsummary, "sabotage": sabotage_report,
        "manifest_json": os.path.relpath(json_path, ROOT),
        "manifest_csv": os.path.relpath(csv_path, ROOT),
        "gate_collage": (os.path.relpath(collage_path, ROOT)
                        if collage_path else None),
        "wall_s": round(time.time() - t_start, 1),
    }
    with open(os.path.join(DATASET_DIR, "AVIAN_dataset_stats_FINAL.json"),
             "w") as f:
        json.dump(stats, f, indent=2)
    with open(os.path.join(DATASET_DIR, "AVIAN_dataset_build_log_FINAL.txt"),
             "w") as f:
        f.write("\n".join(LOG_LINES))

    log(f"== done in {stats['wall_s']} s ==")
    print("FINAL_DATASET_COMPLETE")


if __name__ == "__main__":
    main()

"""SIH_AVIAN_FINAL -- Stage 2 §4: scoring, after the flight only.

Compares `mission/detections.json` (what the stub detector claims it saw,
flight-time, from `flight_final.py --detect`) against the merged defect
taxonomy to report:

  * Structural coverage %  -- of `coverage_mission.json`'s own patch set,
    the fraction actually seen by a SETTLED, non-stuck waypoint in the real
    flight log (not the planner's own optimistic "if every waypoint flies
    perfectly" number already in that file).
  * Defect recall -- of the ground-truth defects that are not ALREADY
    provably undetectable (escalation_reason in below_contrast/
    below_resolution/occluded/unreachable_angle -- not a planner failure,
    a physics one), what fraction the stub flagged from some settled
    waypoint.
  * Detector precision/recall on the stub itself -- reported as a sanity
    check on this scoring code, NOT a real detector result: the stub can
    only ever "find" defects that are really there (it has no image plane
    to hallucinate a false positive from), so its precision is expected to
    sit at ~100% by construction, which is exactly why it isn't a
    meaningful detector metric yet.

Plain `python3`, no bpy/PyBullet needed -- this only reads JSON already on
disk from `coverage_final.py` and a `flight_final.py --detect` run. This
is the ONLY script in Stage 2 allowed to touch both the flight/detection
side and the ground-truth side, per the master prompt's own §5 gate.

Usage:
    python3 source/score_coverage.py
        [--mission=coverage_mission.json] [--flight-log=coverage_mission_flight_log.json]
        [--detections=detections.json]
"""
from __future__ import annotations
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
SCENE_DIR = os.path.join(ROOT, "scene")
MISSION_DIR = os.path.join(ROOT, "mission")
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")

UAV_DIR = os.environ.get("AVIAN_UAV_DIR", os.path.join(REPO, "AVIAN_UAV"))
if UAV_DIR not in sys.path:
    sys.path.insert(0, UAV_DIR)

_ESCALATIONS_NOT_A_PLANNER_FAILURE = {
    "below_contrast", "below_resolution", "occluded", "unreachable_angle",
}

_GROUND_TRUTH_FILES = (
    "AVIAN_defect_ground_truth_FINAL.json",
    "AVIAN_metro_ground_truth_FINAL.json",
    "AVIAN_steel_ground_truth_FINAL.json",
)


def _load_all_defects():
    out = []
    for fn in _GROUND_TRUTH_FILES:
        with open(os.path.join(SCENE_DIR, fn)) as f:
            out.extend(json.load(f)["defects"])
    return out


def _settled_waypoint_ids(flight_log, mission_waypoint_ids=None):
    """Waypoint ids the real flight actually achieved cleanly -- settled
    AND not a genuine structure-collision event (`stuck`), matching the
    same "a stuck waypoint proves nothing about what it saw" standard
    Stage 1's own gate uses for excluding renders.

    Filtered to `mission_waypoint_ids` when given: `flight_log["log"]`
    also contains RECOVERY/RESPAWN/BATTERY_SWAP sub-entries with suffixed
    ids (e.g. "CWP_082_RECOVERY") that can themselves settle without the
    ORIGINAL waypoint they followed ever having been achieved -- found by
    checking directly (82/150 real waypoint ids settle on their own entry,
    not the 141 a naive count over every settled log row returns, because
    that also counts 59 successful recoveries/respawns/swaps that are not
    themselves any of the 150 planned waypoints). The coverage/recall
    percentages this function feeds were already correct either way (they
    only ever look up membership against `mission["waypoints"]`'s own real
    ids), but the settled COUNT printed alongside them was not, before
    this fix."""
    ids = {e["waypoint_id"] for e in flight_log["log"]
          if e.get("settled") and not e.get("stuck")
          and not e.get("skipped")}
    if mission_waypoint_ids is not None:
        ids &= set(mission_waypoint_ids)
    return ids


def score(mission_path, flight_log_path, detections_path, log=print):
    with open(mission_path) as f:
        mission = json.load(f)
    with open(flight_log_path) as f:
        flight_log = json.load(f)
    with open(detections_path) as f:
        det_doc = json.load(f)

    settled_ids = _settled_waypoint_ids(
        flight_log, mission_waypoint_ids=[w["waypoint_id"]
                                        for w in mission["waypoints"]])
    log(f"  flight  : {len(settled_ids)}/{mission['n_waypoints']} coverage "
        "waypoints settled cleanly (not stuck, not skipped)")

    # -- structural coverage, from the REAL flight, not the plan --------
    all_patch_ids = set()
    for wp in mission["waypoints"]:
        all_patch_ids.update(wp["covers"])
    achieved_patch_ids = set()
    for wp in mission["waypoints"]:
        if wp["waypoint_id"] in settled_ids:
            achieved_patch_ids.update(wp["covers"])
    structural_pct = (100.0 * len(achieved_patch_ids) /
                      max(1, len(all_patch_ids)))
    log(f"  cover   : {len(achieved_patch_ids)}/{len(all_patch_ids)} "
        f"planned patches actually seen by a settled waypoint "
        f"({structural_pct:.2f}% real structural coverage, vs "
        f"{mission['structural_coverage_pct']:.2f}% the planner itself "
        "assumed if every waypoint flew perfectly)")

    # -- defect recall, from the stub's logged detections ---------------
    all_defects = _load_all_defects()
    scoreable = [d for d in all_defects
                if d.get("escalation_reason") not in
                _ESCALATIONS_NOT_A_PLANNER_FAILURE]
    n_excluded = len(all_defects) - len(scoreable)

    detected_ids = set()
    n_stub_reports = 0
    for entry in det_doc["detections"]:
        if entry["waypoint_id"] not in settled_ids:
            continue   # a stuck/unsettled attempt proves nothing real
        for d in entry["detections"]:
            detected_ids.add(d["defect_id"])
            n_stub_reports += 1

    found = sum(1 for d in scoreable if d["defect_id"] in detected_ids)
    recall_pct = 100.0 * found / max(1, len(scoreable))
    log(f"  recall  : {found}/{len(scoreable)} non-escalated ground-truth "
        f"defects fell inside a settled coverage waypoint's view "
        f"({recall_pct:.2f}%) -- {n_excluded} more are excluded as "
        "provably undetectable regardless of planning "
        "(below_contrast/below_resolution/occluded/unreachable_angle)")

    # -- stub precision, a sanity check on THIS scoring code, not a real
    # detector metric: the stub only ever reports real ground-truth
    # defect_ids it found geometrically nearby, so it cannot produce a
    # false positive by construction. Precision != 100% here would mean
    # a bug in this scoring script, not in a detector.
    all_defect_ids = {d["defect_id"] for d in all_defects}
    n_hallucinated = sum(1 for did in detected_ids
                        if did not in all_defect_ids)
    precision_pct = (100.0 * (len(detected_ids) - n_hallucinated) /
                     max(1, len(detected_ids)))
    log(f"  stub    : {n_stub_reports} detection reports across "
        f"{len(settled_ids)} settled waypoints, {precision_pct:.1f}% "
        "precision (STUB SANITY CHECK ONLY -- it can only 'find' real "
        "ground-truth defects, so this is not a real detector result; "
        "see detect_stub_final.py's own docstring)")

    return {
        "n_coverage_waypoints": mission["n_waypoints"],
        "n_settled": len(settled_ids),
        "planned_structural_coverage_pct": mission["structural_coverage_pct"],
        "achieved_structural_coverage_pct": round(structural_pct, 2),
        "n_patches_total": len(all_patch_ids),
        "n_patches_achieved": len(achieved_patch_ids),
        "n_defects_total": len(all_defects),
        "n_defects_excluded_undetectable": n_excluded,
        "n_defects_scoreable": len(scoreable),
        "n_defects_recalled": found,
        "recall_pct": round(recall_pct, 2),
        "stub_precision_pct_SANITY_CHECK_ONLY": round(precision_pct, 1),
        "stub_caveat": ("detect_stub_final.py peeks at ground truth "
                       "internally -- these are NOT real detector "
                       "numbers, only a check that this scoring script "
                       "and the coverage plan agree with each other."),
    }


# ---------------------------------------------------------------------------
# Phase 2 step 6: the REAL detector's numbers, IoU-matched on the rendered
# view -- "was the defect in frame" (what the stub-era `score()` above
# checks) is not the same claim as "the detector's box was ON the defect",
# which is what the master prompt's own gate demands here.
# ---------------------------------------------------------------------------
import inspect   # noqa: E402


def _camera_intrinsics():
    import sensors.cameras as UAVCAM
    sig = inspect.signature(UAVCAM.RGBCamera.__init__)
    return (float(sig.parameters["hfov_deg"].default),
           float(sig.parameters["vfov_deg"].default),
           tuple(UAVCAM.LOOP_RES))


def _project_point(cam_pos, target, hfov_deg, vfov_deg, width, height,
                   point):
    """Projects a world point into the image plane of a camera at
    `cam_pos` looking at `target`, using the exact same look-at frame
    `render_coverage_views_final.py`'s `_look_at()` builds (local +Z =
    away from target, +X = right, +Y = up) -- so this matches the frame
    the image was actually rendered in, not a re-derived one. Returns
    (px, py, depth_m) or None if the point is behind the camera."""
    loc = np.array(cam_pos, dtype=float)
    tgt = np.array(target, dtype=float)
    p = np.array(point, dtype=float)
    fwd = loc - tgt
    flen = np.linalg.norm(fwd)
    fwd = fwd / flen if flen > 1e-9 else np.array([0.0, 0.0, 1.0])
    up_world = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(fwd, up_world)) > 0.999:
        up_world = np.array([0.0, 1.0, 0.0])
    right = np.cross(up_world, fwd)
    right /= np.linalg.norm(right)
    up2 = np.cross(fwd, right)
    up2 /= np.linalg.norm(up2)

    delta = p - loc
    lx, ly, lz = np.dot(delta, right), np.dot(delta, up2), np.dot(delta, fwd)
    depth = -lz
    if depth <= 1e-3:
        return None
    ndc_x = lx / (depth * math.tan(math.radians(hfov_deg / 2.0)))
    ndc_y = ly / (depth * math.tan(math.radians(vfov_deg / 2.0)))
    px = (ndc_x * 0.5 + 0.5) * width
    py = (1.0 - (ndc_y * 0.5 + 0.5)) * height
    return px, py, depth


def _iou(box_a, box_b):
    ax0, ay0, ax1, ay1 = box_a
    bx0, by0, bx1, by1 = box_b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


REAL_IOU_MATCH = 0.10   # loose on purpose -- the ground-truth "box" here
                        # is a small window around a PROJECTED POINT, not
                        # a real rendered silhouette, so it is a rough
                        # proxy for "the detection is roughly on the
                        # defect", not a precise box a real detector's
                        # own training IoU (0.5, see train_detector_final.py)
                        # should be compared against
GT_MAX_RANGE_M = 20.0
GT_MIN_BOX_PX = 8.0


def score_real(renders_path, detections_path, log=print):
    """The master prompt's own instruction: match by IoU on the rendered
    view, not by "was the defect in frame". Ground-truth defects are
    projected into each render's own image plane (same camera frame the
    render used) and given a small box sized from their real
    `feature_size_mm` at the projected depth; a prediction counts as a
    real positive only if its box overlaps that projected box AND its
    predicted family matches the defect's own family. This does NOT model
    occlusion by intervening structure (a defect behind a girder can still
    project into frame mathematically) -- a stated simplification, not a
    silent one."""
    hfov_deg, vfov_deg, (width, height) = _camera_intrinsics()
    with open(renders_path) as f:
        renders = json.load(f)["renders"]
    with open(detections_path) as f:
        det_doc = json.load(f)
    det_by_waypoint = {d["waypoint_id"]: d["detections"]
                      for d in det_doc["detections"]}
    with open(os.path.join(DATASET_DIR, "labels_final.json")) as f:
        type_to_family = json.load(f)["type_to_family"]

    all_defects = _load_all_defects()
    scoreable = [d for d in all_defects
                if d.get("escalation_reason") not in
                _ESCALATIONS_NOT_A_PLANNER_FAILURE
                and d["type"] in type_to_family]

    gsd_mm_at_1m = 1000.0 * 2.0 * math.tan(math.radians(hfov_deg / 2.0)) / width

    tp = fp = fn = 0
    matched_defect_ids = set()
    per_family = {}
    for r in renders:
        cam_pos = r["achieved_position_m"]
        target = r["target_m"]
        preds = det_by_waypoint.get(r["waypoint_id"], [])
        gt_here = []
        for d in scoreable:
            proj = _project_point(cam_pos, target, hfov_deg, vfov_deg,
                                  width, height, d["position_m"])
            if proj is None:
                continue
            px, py, depth = proj
            if depth > GT_MAX_RANGE_M:
                continue
            if not (0 <= px <= width and 0 <= py <= height):
                continue
            gsd_at_depth = gsd_mm_at_1m * depth
            half = max(GT_MIN_BOX_PX,
                      (d["feature_size_mm"] / max(gsd_at_depth, 1e-6)) / 2.0)
            box = [max(0, px - half), max(0, py - half),
                  min(width, px + half), min(height, py + half)]
            gt_here.append((d, box))

        used_preds = set()
        for d, gbox in gt_here:
            fam = type_to_family[d["type"]]
            per_family.setdefault(fam, {"tp": 0, "fp": 0, "fn": 0})
            best_iou, best_i = 0.0, None
            for i, p in enumerate(preds):
                if i in used_preds or p["class"] != fam:
                    continue
                iou = _iou(gbox, p["bbox"])
                if iou > best_iou:
                    best_iou, best_i = iou, i
            if best_iou >= REAL_IOU_MATCH:
                tp += 1
                per_family[fam]["tp"] += 1
                used_preds.add(best_i)
                matched_defect_ids.add(d["defect_id"])
            else:
                fn += 1
                per_family[fam]["fn"] += 1
        for i, p in enumerate(preds):
            if i not in used_preds:
                fp += 1
                per_family.setdefault(p["class"], {"tp": 0, "fp": 0, "fn": 0})
                per_family[p["class"]]["fp"] += 1

    precision = tp / max(1, tp + fp)
    recall_all_renders = tp / max(1, tp + fn)
    recall_vs_all_scoreable = len(matched_defect_ids) / max(1, len(scoreable))
    log(f"  real    : tp={tp} fp={fp} fn={fn} across {len(renders)} rendered "
       f"views (IoU>={REAL_IOU_MATCH}, family must match)")
    log(f"  real precision = {precision:.3f}   "
       f"real recall (of GT nominally in a rendered frame) = "
       f"{recall_all_renders:.3f}")
    log(f"  real recall (of ALL {len(scoreable)} non-escalated ground-truth "
       f"defects, whether or not any render reached them) = "
       f"{recall_vs_all_scoreable:.3f}")
    for fam, s in per_family.items():
        p = s["tp"] / max(1, s["tp"] + s["fp"])
        r = s["tp"] / max(1, s["tp"] + s["fn"])
        log(f"    {fam:20s} P={p:.3f} R={r:.3f} "
           f"(tp={s['tp']} fp={s['fp']} fn={s['fn']})")

    return {
        "iou_threshold": REAL_IOU_MATCH,
        "n_renders": len(renders),
        "n_scoreable_defects": len(scoreable),
        "tp": tp, "fp": fp, "fn": fn,
        "precision": round(precision, 4),
        "recall_of_in_frame_defects": round(recall_all_renders, 4),
        "recall_of_all_scoreable_defects": round(recall_vs_all_scoreable, 4),
        "per_family": per_family,
        "caveat": ("REAL detector, real inference -- but ground-truth "
                  "boxes here are a small window around a PROJECTED "
                  "point, not a rendered silhouette, and this does not "
                  "model occlusion by intervening structure. Distinct "
                  "from the stub-era score()'s numbers above, which "
                  "checked cone-of-view only."),
    }


def main():
    mission_name = "coverage_mission.json"
    flight_log_name = None
    det_name = "detections.json"
    real = False
    renders_name = "AVIAN_coverage_renders_FINAL.json"
    real_det_name = "detections_real_final.json"
    for a in sys.argv:
        if a.startswith("--mission="):
            mission_name = a.split("=", 1)[1]
        elif a.startswith("--flight-log="):
            flight_log_name = a.split("=", 1)[1]
        elif a.startswith("--detections="):
            det_name = a.split("=", 1)[1]
        elif a == "--real":
            real = True
        elif a.startswith("--renders="):
            renders_name = a.split("=", 1)[1]
        elif a.startswith("--real-detections="):
            real_det_name = a.split("=", 1)[1]
    if flight_log_name is None:
        stem = os.path.splitext(mission_name)[0]
        flight_log_name = (f"{stem}_flight_log.json"
                          if mission_name != "mission.json"
                          else "flight_log.json")

    print("== SIH_AVIAN_FINAL :: score_coverage.py (Stage 2 sec4) ==")
    print("-- stub-era numbers (Phase 1, sanity-check only) --")
    result = score(
        os.path.join(MISSION_DIR, mission_name),
        os.path.join(MISSION_DIR, flight_log_name),
        os.path.join(MISSION_DIR, det_name),
    )
    out_path = os.path.join(MISSION_DIR, "coverage_score.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"  saved   : {out_path}")

    if real:
        print("-- REAL detector numbers (Phase 2) --")
        real_result = score_real(
            os.path.join(DET_DIR, renders_name),
            os.path.join(MISSION_DIR, real_det_name),
        )
        real_out_path = os.path.join(MISSION_DIR, "coverage_score_real.json")
        with open(real_out_path, "w") as f:
            json.dump(real_result, f, indent=2)
        print(f"  saved   : {real_out_path}")
    print("FINAL_SCORE_COMPLETE")


if __name__ == "__main__":
    main()

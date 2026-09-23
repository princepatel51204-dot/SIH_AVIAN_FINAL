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

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
MISSION_DIR = os.path.join(ROOT, "mission")

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


def _settled_waypoint_ids(flight_log):
    """Waypoint ids the real flight actually achieved cleanly -- settled
    AND not a genuine structure-collision event (`stuck`), matching the
    same "a stuck waypoint proves nothing about what it saw" standard
    Stage 1's own gate uses for excluding renders."""
    return {e["waypoint_id"] for e in flight_log["log"]
           if e.get("settled") and not e.get("stuck")
           and not e.get("skipped")}


def score(mission_path, flight_log_path, detections_path, log=print):
    with open(mission_path) as f:
        mission = json.load(f)
    with open(flight_log_path) as f:
        flight_log = json.load(f)
    with open(detections_path) as f:
        det_doc = json.load(f)

    settled_ids = _settled_waypoint_ids(flight_log)
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


def main():
    mission_name = "coverage_mission.json"
    flight_log_name = None
    det_name = "detections.json"
    for a in sys.argv:
        if a.startswith("--mission="):
            mission_name = a.split("=", 1)[1]
        elif a.startswith("--flight-log="):
            flight_log_name = a.split("=", 1)[1]
        elif a.startswith("--detections="):
            det_name = a.split("=", 1)[1]
    if flight_log_name is None:
        stem = os.path.splitext(mission_name)[0]
        flight_log_name = (f"{stem}_flight_log.json"
                          if mission_name != "mission.json"
                          else "flight_log.json")

    print("== SIH_AVIAN_FINAL :: score_coverage.py (Stage 2 sec4) ==")
    result = score(
        os.path.join(MISSION_DIR, mission_name),
        os.path.join(MISSION_DIR, flight_log_name),
        os.path.join(MISSION_DIR, det_name),
    )
    out_path = os.path.join(MISSION_DIR, "coverage_score.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_SCORE_COMPLETE")


if __name__ == "__main__":
    main()

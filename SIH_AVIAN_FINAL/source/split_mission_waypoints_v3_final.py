"""SIH_AVIAN_FINAL -- Phase 2 Section 3 step 1: split the 82 settled
Stage 2 waypoints into MISSION-NEG / MISSION-VAL / MISSION-TEST, BEFORE
any v3 training happens.

Geometry-only, same as every waypoint-selection step in this project so
far: stratified by structure kind (`coverage_final.py`'s own `prim_kind`,
itself ground-truth-free), fixed seed, decided before any tile is scored
or any hard negative is mined. `grep ground_truth` on this file returns
nothing.

  MISSION-NEG  (~40%) -- mined for hard negatives in Step 2 ONLY. Any
                         tile in this split that turns out (on later,
                         separate ground-truth inspection) to show a real
                         defect is dropped from the negative pool, not
                         relabelled -- a defect must never enter training
                         through this route.
  MISSION-VAL  (~20%) -- picks the score threshold and the headline model.
  MISSION-TEST (~40%) -- scored once, at the end.

Usage:
    python3 source/split_mission_waypoints_v3_final.py
"""
from __future__ import annotations
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MISSION_DIR = os.path.join(ROOT, "mission")
DET_DIR = os.path.join(ROOT, "detection")

# Same bucket mapping as render_zoom_tiles_final.py's own _KIND_BUCKET,
# copied rather than imported: that module imports bpy at load time (it's
# a Blender script), and this one needs to run in plain python3.
_KIND_BUCKET = {
    "pier_column": "PIER", "pier_cap": "PIER", "pier_footing": "PIER",
    "abutment": "PIER", "bearing": "PIER",
    "girder": "TRUSS", "truss_chord": "TRUSS", "truss_diagonal": "TRUSS",
    "truss_vertical": "TRUSS", "gusset_plate": "TRUSS", "bracing": "TRUSS",
    "diaphragm": "TRUSS", "floor_beam": "TRUSS", "stringer": "TRUSS",
    "structure": "TRUSS",
    "deck_box": "DECK", "deck_slab": "DECK", "parapet": "DECK",
    "track_slab": "DECK", "rail": "DECK", "joint_gap": "DECK",
    "expansion_joint": "DECK", "drain": "DECK", "cable_trough": "DECK",
    "access_hatch": "DECK", "service_duct": "DECK",
    "catenary_mast": "METRO",
}

SEED = 20260927
FRACTIONS = {"MISSION-NEG": 0.40, "MISSION-VAL": 0.20, "MISSION-TEST": 0.40}


def main():
    with open(os.path.join(MISSION_DIR, "coverage_mission.json")) as f:
        mission = json.load(f)
    with open(os.path.join(MISSION_DIR,
                          "coverage_mission_flight_log.json")) as f:
        flight_log = json.load(f)
    mission_by_id = {w["waypoint_id"]: w for w in mission["waypoints"]}
    settled = {e["waypoint_id"] for e in flight_log["log"]
              if e.get("settled") and not e.get("stuck")
              and not e.get("skipped")}
    settled = sorted(settled & set(mission_by_id))
    print(f"== SIH_AVIAN_FINAL :: split_mission_waypoints_v3_final.py ==")
    print(f"  settled waypoints: {len(settled)}")

    buckets = {}
    for wid in settled:
        kind = mission_by_id[wid]["prim_kind"]
        bucket = _KIND_BUCKET.get(kind, "OTHER")
        buckets.setdefault(bucket, []).append(wid)
    print("  buckets: " + ", ".join(f"{k}={len(v)}" for k, v in
                                    sorted(buckets.items())))

    rng = random.Random(SEED)
    split = {"MISSION-NEG": [], "MISSION-VAL": [], "MISSION-TEST": []}
    for bucket, ids in sorted(buckets.items()):
        ids_sorted = sorted(ids)
        rng.shuffle(ids_sorted)
        n = len(ids_sorted)
        n_neg = round(n * FRACTIONS["MISSION-NEG"])
        n_val = round(n * FRACTIONS["MISSION-VAL"])
        # remainder to TEST so every waypoint lands somewhere even after
        # rounding, and no bucket silently loses one to double-rounding
        n_test = n - n_neg - n_val
        split["MISSION-NEG"].extend(ids_sorted[:n_neg])
        split["MISSION-VAL"].extend(ids_sorted[n_neg:n_neg + n_val])
        split["MISSION-TEST"].extend(ids_sorted[n_neg + n_val:])
        print(f"    {bucket:6s}: {n} -> NEG={n_neg} VAL={n_val} "
             f"TEST={n_test}")

    for k in split:
        split[k] = sorted(split[k])
    total = sum(len(v) for v in split.values())
    print(f"  total: NEG={len(split['MISSION-NEG'])} "
         f"VAL={len(split['MISSION-VAL'])} "
         f"TEST={len(split['MISSION-TEST'])} (sum={total}, "
         f"expected {len(settled)})")
    assert total == len(settled), "waypoint split lost or duplicated one"
    overlap = (set(split["MISSION-NEG"]) & set(split["MISSION-VAL"])
              | set(split["MISSION-VAL"]) & set(split["MISSION-TEST"])
              | set(split["MISSION-NEG"]) & set(split["MISSION-TEST"]))
    print(f"  overlap check: {len(overlap)} waypoints in more than one "
         f"split -- {'FAIL' if overlap else 'PASS'}")
    assert not overlap

    out = {"seed": SEED, "fractions": FRACTIONS,
          "buckets": {k: len(v) for k, v in buckets.items()},
          "split": split}
    out_path = os.path.join(DET_DIR, "mission_split_v3.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_MISSION_SPLIT_COMPLETE")


if __name__ == "__main__":
    main()

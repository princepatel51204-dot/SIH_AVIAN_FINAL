"""SIH_AVIAN_FINAL -- Phase 2 Section 3 step 2: mine hard negatives from
MISSION-NEG tiles only.

Ground truth is read HERE, after every tile is already rendered and the
waypoint split is already fixed (`detection/mission_split_v3.json`,
written before this script ever runs) -- purely to VERIFY a candidate
negative is actually clean, the same role VD04 plays for the synthetic
dataset's own negatives. This is not "choosing tiles by ground truth":
every MISSION-NEG tile was already selected by geometry; this script only
ever REMOVES a tile from the negative pool, never adds one because of
what it shows.

Usage:
    python3 source/mine_hard_negatives_v3_final.py
"""
from __future__ import annotations
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
from score_coverage import (_project_point, _load_all_defects,
                           _ESCALATIONS_NOT_A_PLANNER_FAILURE)

GT_MAX_RANGE_M = 12.0
LOOP_W, LOOP_H = 640, 480
SEED = 20260927


def main():
    n_positive_cap = None
    for a in sys.argv:
        if a.startswith("--cap="):
            n_positive_cap = int(a.split("=", 1)[1])

    with open(os.path.join(DET_DIR, "AVIAN_zoom_tiles_all_FINAL.json")) as f:
        all_tiles = json.load(f)["tiles"]
    with open(os.path.join(DET_DIR, "mission_split_v3.json")) as f:
        mission_split = json.load(f)["split"]
    neg_waypoints = set(mission_split["MISSION-NEG"])

    neg_tiles = [t for t in all_tiles if t["waypoint_id"] in neg_waypoints]
    print("== SIH_AVIAN_FINAL :: mine_hard_negatives_v3_final.py "
         "(Phase 2 sec3) ==")
    print(f"  MISSION-NEG tiles available: {len(neg_tiles)} across "
         f"{len(neg_waypoints)} waypoints")

    all_defects = _load_all_defects()
    scoreable = [d for d in all_defects
                if d.get("escalation_reason") not in
                _ESCALATIONS_NOT_A_PLANNER_FAILURE]

    clean = []
    n_dropped = 0
    for t in neg_tiles:
        cam_pos = t["achieved_position_m"]
        target = t["tile_target_m"]
        hfov, vfov = t["zoom_hfov_deg"], t["zoom_vfov_deg"]
        contaminated = False
        for d in scoreable:
            proj = _project_point(cam_pos, target, hfov, vfov, LOOP_W,
                                  LOOP_H, d["position_m"])
            if proj is None:
                continue
            px, py, depth = proj
            if depth > GT_MAX_RANGE_M:
                continue
            if 0 <= px <= LOOP_W and 0 <= py <= LOOP_H:
                contaminated = True
                break
        if contaminated:
            n_dropped += 1
        else:
            clean.append(t)
    print(f"  clean (no projected defect in frame): {len(clean)}/"
         f"{len(neg_tiles)} ({n_dropped} dropped as contaminated -- "
         "same role as VD04 for the synthetic dataset)")

    rng = random.Random(SEED)
    rng.shuffle(clean)
    if n_positive_cap is not None:
        clean = clean[:n_positive_cap]
        print(f"  capped to {len(clean)} (requested cap={n_positive_cap})")

    out = {
        "seed": SEED, "n_available": len(neg_tiles),
        "n_dropped_contaminated": n_dropped, "n_selected": len(clean),
        "tiles": clean,
    }
    out_path = os.path.join(DET_DIR, "AVIAN_hard_negatives_v3_FINAL.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_MINE_NEGATIVES_COMPLETE")


if __name__ == "__main__":
    main()

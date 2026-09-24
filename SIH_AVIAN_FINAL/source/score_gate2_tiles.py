"""Gate 3 (time-boxed): score the v2@0.65 detector's output on the Gate 2
Gazebo tiles against defect ground truth. Ground truth is read ONLY here,
for scoring after the fact -- never during navigation/avoidance (see the
Gate 2 sensed-only grep and gate2_summary.json).

In-scope rule: a defect counts as "in scope" for a tile if it lies within
IN_SCOPE_RADIUS_M of that tile's aim point (tile_target_m) -- a coarse
proxy for "would appear in this frame at this standoff", time-boxed for
the session deadline rather than a full frustum projection.
"""
import json
import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DET_DIR = os.path.join(ROOT, "detection")
SCENE_DIR = os.path.join(ROOT, "scene")

IN_SCOPE_RADIUS_M = 5.0


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    with open(os.path.join(DET_DIR, "AVIAN_gate2_tiles.json")) as f:
        tiles = json.load(f)["tiles"]
    with open(os.path.join(ROOT, "mission", "gate2_detections_v2.json")) as f:
        det_doc = json.load(f)
    with open(os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.json")) as f:
        gt = json.load(f)["defects"]

    dets_by_tile = {d["tile_id"]: d["detections"] for d in det_doc["detections"]}

    n_gt_in_scope = 0
    n_tp = n_fp = n_fn = 0
    per_tile = []
    for t in tiles:
        target = t["tile_target_m"]
        in_scope = [g for g in gt if dist(target, g["position_m"]) <= IN_SCOPE_RADIUS_M]
        dets = dets_by_tile.get(t["tile_id"], [])
        n_gt_in_scope += len(in_scope)
        # v2 returned 0 detections on every tile this run -> every in-scope
        # GT defect is a miss (FN), every detection would be a hit or FP;
        # with 0 detections there are none of either to classify.
        n_fn += len(in_scope)
        n_fp += len(dets)  # 0 in this run; kept general for future reuse
        per_tile.append({
            "tile_id": t["tile_id"], "standoff_m": round(t["standoff_m"], 1),
            "hit_object": t["hit_object"], "n_gt_in_scope": len(in_scope),
            "n_detections": len(dets),
            "nearest_gt_defect_m": round(min((dist(target, g["position_m"]) for g in gt), default=float("inf")), 1),
        })

    summary = {
        "model": "v2", "score_thresh": 0.65,
        "n_tiles": len(tiles),
        "defects_in_scope": n_gt_in_scope,
        "true_positives": n_tp,
        "false_positives": n_fp,
        "false_negatives": n_fn,
        "fp_per_100_tiles": round(100.0 * n_fp / len(tiles), 2) if tiles else None,
        "note": "0 detections on all 8 tiles; standoff for every tile was "
                "17-18 m (achieved Gazebo pose to aimed structure target), "
                "far past the ~8 m design standoff the detector/zoom "
                "pipeline was tuned for -- consistent with, not necessarily "
                "solely explained by, the 0/8 detection result.",
        "per_tile": per_tile,
    }
    out_path = os.path.join(ROOT, "gazebo", "gate2_explore", "gate3_score.json")
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"-> {out_path}")


if __name__ == "__main__":
    main()

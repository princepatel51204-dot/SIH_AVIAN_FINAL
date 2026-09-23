"""SIH_AVIAN_FINAL -- Phase 2 Section 2 step 2 (scoring): IoU-match BOTH
models' zoom-tile detections against projected ground truth.

Reuses `score_coverage.py`'s own `_project_point()` / `_iou()` /
`_load_all_defects()` verbatim (imported, not re-derived) so wide-view and
zoom-tile scoring agree on what "matches" means. The one real difference
from the wide-view path: each TILE carries its OWN `zoom_hfov_deg`/
`zoom_vfov_deg` (computed per waypoint from its own achieved standoff),
so the projection call takes those per-tile, not the aircraft's fixed
69/42 deg sensor constant `score_real()` uses for the wide view.

A single physical defect can legitimately be scored across more than one
tile (tiles overlap by `render_zoom_tiles_final.TILE_OVERLAP_FRAC`) --
each tile is one independent detection opportunity, same as each
waypoint was in the wide-view scoring. `n_defects_found_at_least_once` is
reported separately from raw TP count for exactly this reason.

Ground truth is read ONLY here, after every tile is already rendered and
both models' detections are already on disk -- confirmed by grep, same
boundary as `score_coverage.py`'s own docstring states.

Usage:
    python3 source/score_zoom_tiles_final.py
"""
from __future__ import annotations
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")
MISSION_DIR = os.path.join(ROOT, "mission")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
from score_coverage import (_project_point, _iou, _load_all_defects,
                           _ESCALATIONS_NOT_A_PLANNER_FAILURE)

GT_MAX_RANGE_M = 12.0   # BUG FOUND AND FIXED: an earlier version of this
                        # constant was 3.0 m, on the mistaken assumption
                        # that zooming in means flying closer. It doesn't
                        # -- "no re-flight" means the camera sits at
                        # exactly the same ~8 m standoff as the wide view,
                        # only the FOV narrows. At 3.0 m every one of the
                        # 1167 tiles found zero candidate ground-truth
                        # defects in range (fn=0 across the board, the
                        # tell that something was wrong, not a real
                        # result) -- 12 m comfortably covers the real
                        # ~7.9-8.1 m standoff with slack for a defect not
                        # sitting exactly at the tile's own look depth.
REAL_IOU_MATCH = 0.10   # same threshold score_coverage.py's own
                        # score_real() uses, for a like-for-like comparison
GT_MIN_BOX_PX = 8.0
LOOP_W, LOOP_H = 640, 480


def score_zoom(zoom_tiles_path, detections_path, log=print):
    with open(zoom_tiles_path) as f:
        zoom_doc = json.load(f)
    tiles = zoom_doc["tiles"]
    with open(detections_path) as f:
        det_doc = json.load(f)
    det_by_tile = {d["tile_id"]: d["detections"] for d in det_doc["detections"]}
    with open(os.path.join(DATASET_DIR, "labels_final.json")) as f:
        type_to_family = json.load(f)["type_to_family"]

    all_defects = _load_all_defects()
    scoreable = [d for d in all_defects
                if d.get("escalation_reason") not in
                _ESCALATIONS_NOT_A_PLANNER_FAILURE
                and d["type"] in type_to_family]

    tp = fp = fn = 0
    matched_defect_ids = set()
    considered_defect_ids = set()
    per_family = {}
    for t in tiles:
        cam_pos = t["achieved_position_m"]
        target = t["tile_target_m"]
        hfov, vfov = t["zoom_hfov_deg"], t["zoom_vfov_deg"]
        gsd_mm_at_1m = 1000.0 * 2.0 * math.tan(math.radians(hfov / 2.0)) / LOOP_W
        preds = det_by_tile.get(t["tile_id"], [])

        gt_here = []
        for d in scoreable:
            proj = _project_point(cam_pos, target, hfov, vfov, LOOP_W,
                                  LOOP_H, d["position_m"])
            if proj is None:
                continue
            px, py, depth = proj
            if depth > GT_MAX_RANGE_M:
                continue
            if not (0 <= px <= LOOP_W and 0 <= py <= LOOP_H):
                continue
            considered_defect_ids.add(d["defect_id"])
            gsd_at_depth = gsd_mm_at_1m * depth
            half = max(GT_MIN_BOX_PX,
                      (d["feature_size_mm"] / max(gsd_at_depth, 1e-6)) / 2.0)
            box = [max(0, px - half), max(0, py - half),
                  min(LOOP_W, px + half), min(LOOP_H, py + half)]
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
    recall = tp / max(1, tp + fn)
    n_waypoints = len({t["waypoint_id"] for t in tiles})
    log(f"  model {det_doc.get('model', '?'):3s}: tp={tp} fp={fp} fn={fn} "
       f"across {len(tiles)} tiles / {n_waypoints} waypoints "
       f"(IoU>={REAL_IOU_MATCH}, family must match)")
    log(f"    precision={precision:.3f} recall={recall:.3f}  "
       f"{len(matched_defect_ids)}/{len(considered_defect_ids)} distinct "
       "ground-truth defects found at least once "
       f"(of {len(scoreable)} total non-escalated, scoreable defects)")
    for fam, s in sorted(per_family.items()):
        p = s["tp"] / max(1, s["tp"] + s["fp"])
        r = s["tp"] / max(1, s["tp"] + s["fn"])
        log(f"      {fam:20s} P={p:.3f} R={r:.3f} "
           f"(tp={s['tp']} fp={s['fp']} fn={s['fn']})")

    return {
        "model": det_doc.get("model"),
        "model_label": det_doc.get("model_label"),
        "n_tiles": len(tiles), "n_waypoints": n_waypoints,
        "tp": tp, "fp": fp, "fn": fn,
        "precision": round(precision, 4), "recall": round(recall, 4),
        "n_scoreable_defects": len(scoreable),
        "n_defects_considered_in_scope": len(considered_defect_ids),
        "n_defects_found_at_least_once": len(matched_defect_ids),
        "per_family": per_family,
    }


def main():
    zoom_tiles_path = os.path.join(DET_DIR, "AVIAN_zoom_tiles_FINAL.json")
    print("== SIH_AVIAN_FINAL :: score_zoom_tiles_final.py (Phase 2 sec2) ==")
    results = {}
    for name in ("v1", "v2"):
        det_path = os.path.join(MISSION_DIR,
                                f"detections_zoom_{name}_final.json")
        results[name] = score_zoom(zoom_tiles_path, det_path)
    out_path = os.path.join(MISSION_DIR, "coverage_score_zoom.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_ZOOM_SCORE_COMPLETE")


if __name__ == "__main__":
    main()

"""SIH_AVIAN_FINAL -- Phase 2 Section 3 steps 3-4: headline selection on
MISSION-VAL, scoring on MISSION-TEST, for v1/v2/v3 together.

Headline rule (fixed before this script's own results exist, per the
user's course-correction pattern from Section 2): the headline model is
whichever of v1/v2/v3 gets the highest F1 on MISSION-VAL, each at ITS OWN
best threshold (swept, not guessed) on that same split. MISSION-TEST is
scored exactly once, after the headline is already chosen, using each
model's own MISSION-VAL-selected threshold -- never re-tuned on test.

Ground truth is read here only to score already-rendered tiles and
already-run detections against projected positions -- the same boundary
`score_coverage.py` and `score_zoom_tiles_final.py` already established.
Waypoint/tile selection happened earlier, in `render_zoom_tiles_all_final.py`
and `split_mission_waypoints_v3_final.py`, both ground-truth-free.

Usage:
    python3 source/evaluate_v3_mission_final.py
"""
from __future__ import annotations
import json
import math
import os
import sys
import time

import torch
from PIL import Image
from torchvision.models.detection import (
    fasterrcnn_mobilenet_v3_large_320_fpn, fasterrcnn_mobilenet_v3_large_fpn)
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as TF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")
MISSION_DIR = os.path.join(ROOT, "mission")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
from score_coverage import (_project_point, _iou, _load_all_defects,
                           _ESCALATIONS_NOT_A_PLANNER_FAILURE)

with open(os.path.join(DATASET_DIR, "labels_final.json")) as _f:
    _LABELS = json.load(_f)
FAMILIES = _LABELS["family_order"]
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1
LOW_THRESH = 0.05   # keep everything above the floor; sweep filters later
GT_MAX_RANGE_M = 12.0
IOU_MATCH = 0.10
LOOP_W, LOOP_H = 640, 480
THRESH_SWEEP = [round(0.05 + 0.05 * i, 2) for i in range(19)]   # .05-.95

MODELS = {
    "v1": {
        "weights": os.path.join(DET_DIR, "AVIAN_detector_weights_FINAL.pt"),
        "builder": lambda: fasterrcnn_mobilenet_v3_large_320_fpn(weights=None),
    },
    "v2": {
        "weights": os.path.join(DET_DIR,
                                "AVIAN_detector_weights_v2_FINAL.pt"),
        "builder": lambda: fasterrcnn_mobilenet_v3_large_fpn(
            weights=None, min_size=480, max_size=640),
    },
    "v3": {
        "weights": os.path.join(DET_DIR,
                                "AVIAN_detector_weights_v3_FINAL.pt"),
        "builder": lambda: fasterrcnn_mobilenet_v3_large_fpn(
            weights=None, min_size=480, max_size=640),
    },
}


def _load_model(name):
    spec = MODELS[name]
    m = spec["builder"]()
    in_f = m.roi_heads.box_predictor.cls_score.in_features
    m.roi_heads.box_predictor = FastRCNNPredictor(in_f, NUM_CLASSES)
    m.load_state_dict(torch.load(spec["weights"], map_location="cpu"))
    m.eval()
    return m


@torch.no_grad()
def _detect_raw(model, image_path):
    img = Image.open(image_path).convert("RGB")
    pred = model([TF.to_tensor(img)])[0]
    out = []
    for box, label, score in zip(pred["boxes"], pred["labels"],
                                 pred["scores"]):
        if float(score) < LOW_THRESH:
            continue
        fam = ID_TO_FAMILY.get(int(label))
        if fam is None:
            continue
        out.append({"class": fam, "confidence": round(float(score), 4),
                   "bbox": [round(float(v), 2) for v in box.tolist()]})
    return out


def run_all_detections(tiles):
    """Runs all 3 models once, at a low floor threshold, over every tile
    -- the threshold sweep below only ever re-filters this same raw
    output, so inference happens exactly once per model per tile."""
    raw = {}
    for name in ("v1", "v2", "v3"):
        t0 = time.time()
        model = _load_model(name)
        dets = {}
        for t in tiles:
            dets[t["tile_id"]] = _detect_raw(
                model, os.path.join(ROOT, t["image_path"]))
        raw[name] = dets
        n = sum(len(v) for v in dets.values())
        print(f"  {name}: {n} raw detections (>= {LOW_THRESH}) across "
             f"{len(tiles)} tiles in {time.time() - t0:.1f}s")
    return raw


def _gt_for_tile(t, scoreable):
    gt_here = []
    cam_pos, target = t["achieved_position_m"], t["tile_target_m"]
    hfov, vfov = t["zoom_hfov_deg"], t["zoom_vfov_deg"]
    gsd_mm_at_1m = 1000.0 * 2.0 * math.tan(math.radians(hfov / 2.0)) / LOOP_W
    for d in scoreable:
        proj = _project_point(cam_pos, target, hfov, vfov, LOOP_W, LOOP_H,
                              d["position_m"])
        if proj is None:
            continue
        px, py, depth = proj
        if depth > GT_MAX_RANGE_M or not (0 <= px <= LOOP_W and
                                          0 <= py <= LOOP_H):
            continue
        gsd_at_depth = gsd_mm_at_1m * depth
        half = max(8.0, (d["feature_size_mm"] / max(gsd_at_depth, 1e-6)) / 2.0)
        box = [max(0, px - half), max(0, py - half),
              min(LOOP_W, px + half), min(LOOP_H, py + half)]
        gt_here.append((d, box))
    return gt_here


def score_at_threshold(tiles, raw_dets, threshold, type_to_family,
                       scoreable, defect_filter=None):
    tp = fp = fn = 0
    matched_ids = set()
    for t in tiles:
        preds = [d for d in raw_dets.get(t["tile_id"], [])
                if d["confidence"] >= threshold]
        gt_here = _gt_for_tile(t, scoreable)
        if defect_filter is not None:
            gt_here = [(d, b) for d, b in gt_here
                      if d["defect_id"] in defect_filter]
        used = set()
        for d, gbox in gt_here:
            fam = type_to_family[d["type"]]
            best_iou, best_i = 0.0, None
            for i, p in enumerate(preds):
                if i in used or p["class"] != fam:
                    continue
                iou = _iou(gbox, p["bbox"])
                if iou > best_iou:
                    best_iou, best_i = iou, i
            if best_iou >= IOU_MATCH:
                tp += 1
                used.add(best_i)
                matched_ids.add(d["defect_id"])
            else:
                fn += 1
        fp += len(preds) - len(used)
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = (2 * precision * recall / max(1e-9, precision + recall)
         if (precision + recall) > 0 else 0.0)
    return {"threshold": threshold, "tp": tp, "fp": fp, "fn": fn,
           "precision": round(precision, 4), "recall": round(recall, 4),
           "f1": round(f1, 4), "matched_defect_ids": sorted(matched_ids)}


def main():
    print("== SIH_AVIAN_FINAL :: evaluate_v3_mission_final.py "
         "(Phase 2 sec3 steps3-4) ==")
    with open(os.path.join(DET_DIR, "AVIAN_zoom_tiles_all_FINAL.json")) as f:
        all_tiles = json.load(f)["tiles"]
    with open(os.path.join(DET_DIR, "mission_split_v3.json")) as f:
        mission_split = json.load(f)["split"]
    val_ids = set(mission_split["MISSION-VAL"])
    test_ids = set(mission_split["MISSION-TEST"])
    val_tiles = [t for t in all_tiles if t["waypoint_id"] in val_ids]
    test_tiles = [t for t in all_tiles if t["waypoint_id"] in test_ids]
    print(f"  tiles   : {len(all_tiles)} total, {len(val_tiles)} in "
         f"MISSION-VAL, {len(test_tiles)} in MISSION-TEST")

    with open(os.path.join(DATASET_DIR, "labels_final.json")) as f:
        type_to_family = json.load(f)["type_to_family"]
    all_defects = _load_all_defects()
    scoreable = [d for d in all_defects
                if d.get("escalation_reason") not in
                _ESCALATIONS_NOT_A_PLANNER_FAILURE
                and d["type"] in type_to_family]

    # -- unseen-instance set: never a TRAIN or VAL positive -------------
    with open(os.path.join(DATASET_DIR, "splits_final.json")) as f:
        synth_splits = json.load(f)["splits"]
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_filtered_FINAL.json")) as f:
        filtered_by_path = {r["image_path"]: r
                           for r in json.load(f)["frames"]}
    trained_or_val_ids = set()
    for split_name in ("train", "val"):
        for p in synth_splits[split_name]:
            f_ = filtered_by_path[p]
            if f_["is_positive"]:
                trained_or_val_ids.add(f_["defect_id"])
    unseen_ids = {d["defect_id"] for d in scoreable} - trained_or_val_ids
    print(f"  unseen  : {len(unseen_ids)}/{len(scoreable)} scoreable "
         "defects never a TRAIN or VAL positive")
    overlap_check = unseen_ids & trained_or_val_ids
    print(f"  unseen-instance assertion: {len(overlap_check)} unseen ids "
         f"also found in train/val -- {'FAIL' if overlap_check else 'PASS'}")
    assert not overlap_check

    print("  running detections (v1/v2/v3) on all MISSION-VAL + "
         "MISSION-TEST tiles...")
    scoring_tiles = val_tiles + test_tiles
    raw = run_all_detections(scoring_tiles)

    # -- threshold sweep on MISSION-VAL, pick each model's best F1 -------
    best = {}
    for name in ("v1", "v2", "v3"):
        sweeps = [score_at_threshold(val_tiles, raw[name], th,
                                     type_to_family, scoreable)
                 for th in THRESH_SWEEP]
        best[name] = max(sweeps, key=lambda s: s["f1"])
        print(f"  MISSION-VAL {name}: best threshold={best[name]['threshold']} "
             f"F1={best[name]['f1']} P={best[name]['precision']} "
             f"R={best[name]['recall']} (tp={best[name]['tp']} "
             f"fp={best[name]['fp']} fn={best[name]['fn']})")

    headline = max(best, key=lambda n: best[n]["f1"])
    print(f"  HEADLINE (highest MISSION-VAL F1): {headline} "
         f"(F1={best[headline]['f1']})")

    # -- MISSION-TEST, scored once, each model at its own chosen threshold
    test_results = {}
    n_test_tiles_per_waypoint = len(test_tiles)
    for name in ("v1", "v2", "v3"):
        th = best[name]["threshold"]
        all_score = score_at_threshold(test_tiles, raw[name], th,
                                       type_to_family, scoreable)
        unseen_score = score_at_threshold(test_tiles, raw[name], th,
                                          type_to_family, scoreable,
                                          defect_filter=unseen_ids)
        n_fp_per_100 = 100.0 * all_score["fp"] / max(1, len(test_tiles))
        test_results[name] = {
            "threshold": th, "all_defects": all_score,
            "unseen_defects": unseen_score,
            "fp_per_100_tiles": round(n_fp_per_100, 2),
        }
        print(f"  MISSION-TEST {name} (thr={th}): "
             f"unseen R={unseen_score['recall']} "
             f"({len(unseen_score['matched_defect_ids'])}/"
             f"{len(unseen_ids)}); all-defects R={all_score['recall']} "
             f"(optimistic); FP/100 tiles={n_fp_per_100:.1f}")

    out = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "n_val_tiles": len(val_tiles), "n_test_tiles": len(test_tiles),
        "n_scoreable_defects": len(scoreable),
        "n_unseen_defects": len(unseen_ids),
        "unseen_defect_ids": sorted(unseen_ids),
        "mission_val_sweep_best": best,
        "headline_model": headline,
        "mission_test": test_results,
    }
    out_path = os.path.join(MISSION_DIR, "coverage_score_mission_v3.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_EVALUATE_V3_COMPLETE")


if __name__ == "__main__":
    main()

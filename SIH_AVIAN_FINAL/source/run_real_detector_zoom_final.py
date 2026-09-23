"""SIH_AVIAN_FINAL -- Phase 2 Section 2: run BOTH detectors on the zoom
tiles before scoring either.

Per the explicit course-correction: scoring only the new (v2) model on
the tiles the retrain was motivated by would be cherry-picking. Both v1
(`_320`, the Section 1 headline -- test mAP@0.5 0.342, fixed as the
headline BEFORE either model's tile numbers are seen) and v2 (full-res)
run over every rendered tile here, unconditionally, in the same pass.

v1 is loaded exactly as `detect_stub_final.py` loads it (same weights,
same fasterrcnn_mobilenet_v3_large_320_fpn architecture) -- NOT by
importing that module's `detect()`, which takes a plain image path with
no way to report which model answered; this script needs both models'
raw outputs side by side, so both are loaded directly here instead.

Usage:
    python3 source/run_real_detector_zoom_final.py
"""
from __future__ import annotations
import json
import os
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

with open(os.path.join(DATASET_DIR, "labels_final.json")) as _f:
    _LABELS = json.load(_f)
FAMILIES = _LABELS["family_order"]
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1
SCORE_THRESH = 0.5

MODELS = {
    "v1": {
        "weights": os.path.join(DET_DIR, "AVIAN_detector_weights_FINAL.pt"),
        "builder": lambda: fasterrcnn_mobilenet_v3_large_320_fpn(weights=None),
        "label": "v1 (_320, Section 1 HEADLINE, test mAP@0.5=0.342)",
    },
    "v2": {
        "weights": os.path.join(DET_DIR,
                                "AVIAN_detector_weights_v2_FINAL.pt"),
        "builder": lambda: fasterrcnn_mobilenet_v3_large_fpn(
            weights=None, min_size=480, max_size=640),
        "label": "v2 (full-res, test mAP@0.5=0.182 -- reported in full, "
                "NOT the headline)",
    },
}


def _load_model(name):
    spec = MODELS[name]
    m = spec["builder"]()
    in_f = m.roi_heads.box_predictor.cls_score.in_features
    m.roi_heads.box_predictor = FastRCNNPredictor(in_f, NUM_CLASSES)
    state = torch.load(spec["weights"], map_location="cpu")
    m.load_state_dict(state)
    m.eval()
    return m


@torch.no_grad()
def _detect(model, image_path, score_thresh=SCORE_THRESH):
    img = Image.open(image_path).convert("RGB")
    tensor = TF.to_tensor(img)
    pred = model([tensor])[0]
    out = []
    for box, label, score in zip(pred["boxes"], pred["labels"],
                                 pred["scores"]):
        if float(score) < score_thresh:
            continue
        fam = ID_TO_FAMILY.get(int(label))
        if fam is None:
            continue
        out.append({"class": fam, "confidence": round(float(score), 4),
                   "bbox": [round(float(v), 2) for v in box.tolist()]})
    return out


def main():
    with open(os.path.join(DET_DIR, "AVIAN_zoom_tiles_FINAL.json")) as f:
        zoom_doc = json.load(f)
    tiles = zoom_doc["tiles"]
    print(f"== SIH_AVIAN_FINAL :: run_real_detector_zoom_final.py "
         f"({len(tiles)} tiles, both models) ==")

    for name in ("v1", "v2"):
        t0 = time.time()
        model = _load_model(name)
        detections = []
        n_dets = 0
        for t in tiles:
            img_path = os.path.join(ROOT, t["image_path"])
            dets = _detect(model, img_path)
            n_dets += len(dets)
            detections.append({
                "waypoint_id": t["waypoint_id"], "tile_id": t["tile_id"],
                "image_path": t["image_path"], "detections": dets,
            })
        out = {
            "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                          time.gmtime()),
            "model": name, "model_label": MODELS[name]["label"],
            "weights": os.path.relpath(MODELS[name]["weights"], ROOT),
            "n_tiles": len(tiles), "n_detections": n_dets,
            "detections": detections,
        }
        out_path = os.path.join(MISSION_DIR, f"detections_zoom_{name}_final.json")
        with open(out_path, "w") as f:
            json.dump(out, f, indent=2)
        print(f"  {name}: {n_dets} detections across {len(tiles)} tiles "
             f"in {time.time() - t0:.1f}s -> {out_path}")

    print("FINAL_ZOOM_DETECT_COMPLETE")


if __name__ == "__main__":
    main()

"""Gate 3: run the headline v2 @ 0.65 detector on the Gate 2 Gazebo tiles."""
import json
import os
import time

import torch
from PIL import Image
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as TF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")

with open(os.path.join(DATASET_DIR, "labels_final.json")) as _f:
    _LABELS = json.load(_f)
FAMILIES = _LABELS["family_order"]
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1
SCORE_THRESH = 0.65
WEIGHTS = os.path.join(DET_DIR, "AVIAN_detector_weights_v2_FINAL.pt")


def load_model():
    m = fasterrcnn_mobilenet_v3_large_fpn(weights=None, min_size=480, max_size=640)
    in_f = m.roi_heads.box_predictor.cls_score.in_features
    m.roi_heads.box_predictor = FastRCNNPredictor(in_f, NUM_CLASSES)
    state = torch.load(WEIGHTS, map_location="cpu")
    m.load_state_dict(state)
    m.eval()
    return m


@torch.no_grad()
def detect(model, image_path):
    img = Image.open(image_path).convert("RGB")
    tensor = TF.to_tensor(img)
    pred = model([tensor])[0]
    out = []
    for box, label, score in zip(pred["boxes"], pred["labels"], pred["scores"]):
        if float(score) < SCORE_THRESH:
            continue
        fam = ID_TO_FAMILY.get(int(label))
        if fam is None:
            continue
        out.append({"class": fam, "confidence": round(float(score), 4),
                   "bbox": [round(float(v), 2) for v in box.tolist()]})
    return out


def main():
    with open(os.path.join(DET_DIR, "AVIAN_gate2_tiles.json")) as f:
        tiles = json.load(f)["tiles"]
    print(f"== Gate 3: run_detector_gate2.py (v2 @ {SCORE_THRESH}, {len(tiles)} tiles) ==")
    t0 = time.time()
    model = load_model()
    detections = []
    n_dets = 0
    for t in tiles:
        img_path = os.path.join(ROOT, t["image_path"])
        dets = detect(model, img_path)
        n_dets += len(dets)
        detections.append({"waypoint_id": t["waypoint_id"], "tile_id": t["tile_id"],
                          "image_path": t["image_path"], "detections": dets})
    out = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "model": "v2", "score_thresh": SCORE_THRESH,
        "weights": os.path.relpath(WEIGHTS, ROOT),
        "n_tiles": len(tiles), "n_detections": n_dets,
        "detections": detections,
    }
    out_path = os.path.join(ROOT, "mission", "gate2_detections_v2.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  {n_dets} detections across {len(tiles)} tiles in {time.time()-t0:.1f}s -> {out_path}")


if __name__ == "__main__":
    main()

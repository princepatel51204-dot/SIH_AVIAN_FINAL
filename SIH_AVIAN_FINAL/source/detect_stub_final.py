"""SIH_AVIAN_FINAL -- Phase 2: the real detector.

The stub that used to peek at the defect taxonomy's own logged JSON is
GONE, not superseded-but-reachable: this module now loads
`detection/AVIAN_detector_weights_FINAL.pt`
(`fasterrcnn_mobilenet_v3_large_320_fpn`, trained by
`train_detector_final.py` on the filtered, instance-split dataset) and
runs real inference on a rendered image. This file makes no reads of that
taxonomy's logged JSON at all -- confirmed after this rewrite, not
assumed.

Signature kept stable on purpose (`detect(image) -> list[{class,
confidence, bbox}]`) so nothing downstream (`score_coverage.py`,
whatever eventually calls this at flight time) has to change shape, only
trust the numbers more.

`image` is a path to a rendered RGB frame (e.g. one of
`render_coverage_views_final.py`'s outputs). The FLIGHT-TIME pose-based
call this module used to answer (`detect(camera_pos, target, standoff)`,
answered by cheating against ground truth) has no equivalent here on
purpose -- a real detector needs an actual image, not a 3-tuple of
numbers, and Phase 2's own instruction was "no re-flight needed": the
real scoring pass renders offline first (`render_coverage_views_final.py`)
and classifies those renders, it does not re-fly with a detector in the
loop. Any caller still passing a pose instead of an image path will get a
clear TypeError from PIL, not a silently wrong answer.
"""
from __future__ import annotations
import json
import os

import torch
from PIL import Image
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as TF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")
WEIGHTS_PATH = os.path.join(DET_DIR, "AVIAN_detector_weights_FINAL.pt")

with open(os.path.join(DATASET_DIR, "labels_final.json")) as _f:
    _LABELS = json.load(_f)
FAMILIES = _LABELS["family_order"]
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1

DEFAULT_SCORE_THRESH = 0.5

_model = None   # lazy-loaded singleton -- one process may call detect()
                # many times (150 renders) and reloading the weights per
                # call would dominate the wall time for no reason


def _get_model():
    global _model
    if _model is None:
        m = fasterrcnn_mobilenet_v3_large_320_fpn(weights=None)
        in_features = m.roi_heads.box_predictor.cls_score.in_features
        m.roi_heads.box_predictor = FastRCNNPredictor(in_features,
                                                       NUM_CLASSES)
        state = torch.load(WEIGHTS_PATH, map_location="cpu")
        m.load_state_dict(state)
        m.eval()
        _model = m
    return _model


@torch.no_grad()
def detect(image_path, score_thresh=DEFAULT_SCORE_THRESH):
    """Real inference. Returns [{class, confidence, bbox}] with `bbox` as
    [x0, y0, x1, y1] in the rendered image's own pixel coordinates and
    `class` one of the 5 coarse families in `labels_final.json`
    (NUM_CLASSES - 1; class 0 is the model's internal background class
    and is filtered out before returning)."""
    model = _get_model()
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
        out.append({
            "class": fam,
            "confidence": round(float(score), 4),
            "bbox": [round(float(v), 2) for v in box.tolist()],
        })
    return out

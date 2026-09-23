"""SIH_AVIAN_FINAL -- Phase A item 2: real photos, test only, no
retraining.

Dataset: Ozgenel, C.F. (2018) "Concrete Crack Images for Classification",
Mendeley Data, v2, https://doi.org/10.17632/5y9wdsg2zt.2 -- 40,000 images
(20,000 positive/crack, 20,000 negative/no-crack), 227x227, RGB, collected
from METU Campus Buildings. **License: CC BY 4.0** (permissive,
attribution required) -- checked directly on the dataset's own Mendeley
page before use, not assumed.

This is a CLASSIFICATION dataset (whole-image crack/no-crack), not
bounding-box annotated, so this test measures PRESENCE/ABSENCE only, not
localization: a tile counts as "predicted crack" if the model returns any
CRACK-family detection at or above its own chosen threshold (from
`evaluate_v3_mission_final.py`'s MISSION-VAL sweep), regardless of where
in the frame. No retraining on this data -- inference only, on whichever
model won the MISSION-VAL headline vote.

Usage:
    python3 source/evaluate_real_photos_final.py --model=v1 --threshold=0.35 --dir=/tmp/concrete_crack
"""
from __future__ import annotations
import argparse
import json
import os
import random
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

with open(os.path.join(DATASET_DIR, "labels_final.json")) as _f:
    _LABELS = json.load(_f)
FAMILIES = _LABELS["family_order"]
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1
SEED = 20260924
N_PER_CLASS = 500

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
def _predicts_crack(model, image_path, threshold):
    img = Image.open(image_path).convert("RGB")
    pred = model([TF.to_tensor(img)])[0]
    for label, score in zip(pred["labels"], pred["scores"]):
        if float(score) < threshold:
            continue
        if ID_TO_FAMILY.get(int(label)) == "CRACK":
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, choices=list(MODELS))
    ap.add_argument("--threshold", type=float, required=True)
    ap.add_argument("--dir", default="/tmp/concrete_crack")
    ap.add_argument("--n", type=int, default=N_PER_CLASS)
    args = ap.parse_args()

    pos_dir = os.path.join(args.dir, "Positive")
    neg_dir = os.path.join(args.dir, "Negative")
    pos_files = sorted(os.listdir(pos_dir))
    neg_files = sorted(os.listdir(neg_dir))
    rng = random.Random(SEED)
    rng.shuffle(pos_files)
    rng.shuffle(neg_files)
    pos_sample = pos_files[:args.n]
    neg_sample = neg_files[:args.n]

    print(f"== SIH_AVIAN_FINAL :: evaluate_real_photos_final.py "
         f"(model={args.model}, threshold={args.threshold}) ==")
    print(f"  dataset: Ozgenel 2018 'Concrete Crack Images for "
         f"Classification', CC BY 4.0, https://doi.org/10.17632/5y9wdsg2zt.2")
    print(f"  sample : {len(pos_sample)} positive (crack) + "
         f"{len(neg_sample)} negative (no-crack), seed={SEED}")

    model = _load_model(args.model)
    t0 = time.time()
    tp = fn = 0
    for fn_name in pos_sample:
        if _predicts_crack(model, os.path.join(pos_dir, fn_name),
                          args.threshold):
            tp += 1
        else:
            fn += 1
    fp = tn = 0
    for fn_name in neg_sample:
        if _predicts_crack(model, os.path.join(neg_dir, fn_name),
                          args.threshold):
            fp += 1
        else:
            tn += 1
    dt = time.time() - t0

    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    accuracy = (tp + tn) / max(1, tp + tn + fp + fn)
    print(f"  tp={tp} fp={fp} fn={fn} tn={tn}  "
         f"({dt:.1f}s for {len(pos_sample) + len(neg_sample)} images)")
    print(f"  precision={precision:.4f} recall={recall:.4f} "
         f"accuracy={accuracy:.4f}")

    out = {
        "model": args.model, "threshold": args.threshold,
        "dataset": "Ozgenel 2018 Concrete Crack Images for Classification",
        "dataset_doi": "10.17632/5y9wdsg2zt.2", "license": "CC BY 4.0",
        "seed": SEED, "n_positive": len(pos_sample),
        "n_negative": len(neg_sample),
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "precision": round(precision, 4), "recall": round(recall, 4),
        "accuracy": round(accuracy, 4),
        "caveat": ("Classification-only ground truth (no bounding boxes) "
                  "-- measures crack presence/absence, not localization. "
                  "No retraining on this data, inference only."),
    }
    out_path = os.path.join(DET_DIR,
                            f"AVIAN_real_photo_eval_{args.model}_FINAL.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  saved   : {out_path}")
    print("FINAL_REAL_PHOTO_EVAL_COMPLETE")


if __name__ == "__main__":
    main()

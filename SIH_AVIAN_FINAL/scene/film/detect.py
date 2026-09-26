"""Run the REAL trained detector (v2, the one live_detector_node.py flies with)
over image files and emit its raw output. No drawing, no filtering beyond the
score threshold -- whatever this prints is what the model actually said.

Model construction is copied from source/run_real_detector_zoom_final.py so the
architecture matches the weights exactly (fasterrcnn_mobilenet_v3_large_fpn,
min_size=480, max_size=640).
"""
from __future__ import annotations
import json, os, sys, time, argparse

import torch
from PIL import Image
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as TF

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
FAMILIES = json.load(open(f"{ROOT}/dataset/labels_final.json"))["family_order"]
ID_TO_FAMILY = {i + 1: f for i, f in enumerate(FAMILIES)}
WEIGHTS = f"{ROOT}/detection/AVIAN_detector_weights_v2_FINAL.pt"


def load_model():
    m = fasterrcnn_mobilenet_v3_large_fpn(weights=None, min_size=480, max_size=640)
    inf = m.roi_heads.box_predictor.cls_score.in_features
    m.roi_heads.box_predictor = FastRCNNPredictor(inf, len(FAMILIES) + 1)
    m.load_state_dict(torch.load(WEIGHTS, map_location="cpu"))
    m.eval()
    return m


@torch.no_grad()
def detect(model, path, thresh):
    img = Image.open(path).convert("RGB")
    t0 = time.time()
    pred = model([TF.to_tensor(img)])[0]
    dt = time.time() - t0
    out = []
    for b, l, s in zip(pred["boxes"], pred["scores"].new_tensor(pred["labels"], dtype=torch.int64) if False else pred["labels"], pred["scores"]):
        s = float(s)
        if s < thresh:
            continue
        x0, y0, x1, y1 = [float(v) for v in b]
        out.append({"family": ID_TO_FAMILY.get(int(l), "?"), "score": round(s, 4),
                    "bbox": [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)],
                    "w": round(x1 - x0, 1), "h": round(y1 - y0, 1)})
    return out, dt, img.size


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--thresh", type=float, default=0.65)
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "8")))
    m = load_model()
    allr = []
    for p in a.paths:
        dets, dt, size = detect(m, p, a.thresh)
        allr.append({"image": p, "size": list(size), "infer_s": round(dt, 3), "detections": dets})
        tag = f"{len(dets)} det" if dets else "NO DETECTION"
        print(f"{os.path.basename(p):<46} {size[0]}x{size[1]} {dt:5.2f}s  {tag}")
        for d in dets:
            print(f"      {d['family']:<18} {d['score']:.3f}  bbox {d['bbox']}  ({d['w']:.0f}x{d['h']:.0f}px)")
    if a.json:
        json.dump(allr, open(a.json, "w"), indent=1)
    n = sum(len(r["detections"]) for r in allr)
    print(f"\nTOTAL: {n} detections over {len(allr)} images at thresh {a.thresh}")

"""SIH_AVIAN_FINAL -- Phase 2 Section 3 step 2: v3 -- v2's architecture
plus hard negatives mined from real mission structure. NO EXTERNAL REAL
BRIDGE PHOTOS this pass -- see the scope note below before reading this
as the "add real images" step the master prompt asked for.

SCOPE DECISION, MEASURED NOT GUESSED: the master prompt's Step 2 asked
for CODEBRIM (Mundt et al., CVPR 2019) and optionally dacl10k. CODEBRIM's
real download is 7.9-12.2 GB depending on variant, under a non-commercial
license (Zenodo record 2620293). A smaller, MIT-licensed derivative,
SegCODEBRIM (916 MB, crack-only, Zenodo record 10071534), was attempted
instead. Measured sustained throughput from this environment to Zenodo:
177.9 MB in a 600 s window (~0.30 MB/s) -- confirmed twice, not a single
fluke reading. At that rate the 916 MB file alone needs ~52 more minutes
beyond the two attempts already spent, on top of the ~75 minute
all-waypoint tile render already running and the training/scoring still
ahead. Per this section's own time-box rule ("if a step isn't done by the
time box, stop, commit what's real"), real-image acquisition is dropped
entirely for this pass, not just dacl10k. v3 instead adds real NEGATIVE
signal from the actual deployment domain: hard negatives mined from
Stage 2's own MISSION-NEG zoom tiles (real Blender renders of the actual
bridge, from waypoints and viewing angles the synthetic dataset never
used), verified ground-truth-clean the same way VD04 verifies the
synthetic dataset's own negatives. This directly targets the measured
"texture reads as CRACK" failure mode (v1: 309 false CRACKs on 1,167
Section-2 zoom tiles) without needing an external dataset -- a narrower
fix than the master prompt's own Step 2, stated as such.

Architecture is v2's, unchanged: `fasterrcnn_mobilenet_v3_large_fpn`
(no `_320`), `min_size=480, max_size=640`, BSD-3-Clause (torchvision).
Positives: the SAME frozen `splits_final.json` train split v1/v2 used
(224 synthetic positive+negative frames plus 201 positives -- see that
file). Added: up to 201 mined hard-negative tiles (capped ~1:1 with the
201 synthetic positive training frames, per the master prompt's own
rule), as additional empty-target training images, TRAIN split only --
val/test stay exactly `splits_final.json`'s frozen synthetic frames so
the "synthetic test" comparison across v1/v2/v3 is apples to apples.

Outputs (a NEW model, v1/v2's own weights/outputs are untouched):
  detection/AVIAN_detector_weights_v3_FINAL.pt
  detection/confusion_matrix_v3_FINAL.png
  detection/AVIAN_detector_eval_v3_FINAL.json
"""
from __future__ import annotations
import json
import os
import random
import time

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.ops import box_iou
from torchvision.transforms import functional as TF

MODEL_MIN_SIZE = 480    # LOOP frames' own native short side -- no
                        # downscale, see module docstring
MODEL_MAX_SIZE = 640    # LOOP frames' own native long side

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")
os.makedirs(DET_DIR, exist_ok=True)

SEED = 20260923
random.seed(SEED)
torch.manual_seed(SEED)

with open(os.path.join(DATASET_DIR, "labels_final.json")) as f:
    LABELS = json.load(f)
FAMILIES = LABELS["family_order"]                       # 5 families
FAMILY_TO_ID = {fam: i + 1 for i, fam in enumerate(FAMILIES)}   # 0 = background
ID_TO_FAMILY = {v: k for k, v in FAMILY_TO_ID.items()}
NUM_CLASSES = len(FAMILIES) + 1

MIN_BBOX_SIDE = 4.0   # a 1x1 px mask bbox (real, measured -- see
                      # prepare_training_final.py's own stats) is not a
                      # trainable box; padded symmetrically to this
                      # minimum rather than dropped, so the reported
                      # kept-frame counts stay exactly what step 1 reported

EPOCHS = int(os.environ.get("AVIAN_DETECTOR_EPOCHS", "10"))
BATCH_SIZE = int(os.environ.get("AVIAN_DETECTOR_BATCH", "2"))
LR = 0.004
SCORE_THRESH_REPORT = 0.5
IOU_MATCH = 0.5


def _pad_bbox(x0, y0, x1, y1, w, h):
    """Pads a degenerate/tiny mask bbox up to MIN_BBOX_SIDE and guarantees
    the result is strictly positive-area and inside the frame -- clamping
    a padded box straight to [0,w]x[0,h] without this second pass can
    still leave a zero-width box for one already sitting on the image
    edge, which is exactly what fed NaN losses into the RPN on the first
    training attempt (found by testing, not assumed)."""
    x0, y0, x1, y1 = float(x0), float(y0), float(x1), float(y1)
    if x1 - x0 < MIN_BBOX_SIDE:
        cx = (x0 + x1) / 2.0
        x0, x1 = cx - MIN_BBOX_SIDE / 2.0, cx + MIN_BBOX_SIDE / 2.0
    if y1 - y0 < MIN_BBOX_SIDE:
        cy = (y0 + y1) / 2.0
        y0, y1 = cy - MIN_BBOX_SIDE / 2.0, cy + MIN_BBOX_SIDE / 2.0
    x0 = max(0.0, min(x0, w - MIN_BBOX_SIDE))
    y0 = max(0.0, min(y0, h - MIN_BBOX_SIDE))
    x1 = min(float(w), max(x1, x0 + MIN_BBOX_SIDE))
    y1 = min(float(h), max(y1, y0 + MIN_BBOX_SIDE))
    return [x0, y0, x1, y1]


class AvianDetectionDataset(Dataset):
    def __init__(self, frames, augment=False):
        self.frames = frames
        self.augment = augment

    def __len__(self):
        return len(self.frames)

    def __getitem__(self, idx):
        f = self.frames[idx]
        img = Image.open(os.path.join(ROOT, f["image_path"])).convert("RGB")
        w, h = img.size
        boxes, labels = [], []
        if f["is_positive"]:
            x0, y0, x1, y1 = f["bbox_xyxy"]
            boxes.append(_pad_bbox(x0, y0, x1, y1, w, h))
            labels.append(FAMILY_TO_ID[f["family"]])

        if self.augment and random.random() < 0.5:
            img = TF.hflip(img)
            boxes = [[w - x1, y0, w - x0, y1] for x0, y0, x1, y1 in boxes]
        if self.augment:
            if random.random() < 0.5:
                img = TF.adjust_brightness(img, random.uniform(0.7, 1.3))
            if random.random() < 0.5:
                img = TF.adjust_contrast(img, random.uniform(0.7, 1.3))
            if random.random() < 0.3:
                img = TF.gaussian_blur(img, kernel_size=3)

        boxes_t = (torch.as_tensor(boxes, dtype=torch.float32)
                  if boxes else torch.zeros((0, 4), dtype=torch.float32))
        labels_t = (torch.as_tensor(labels, dtype=torch.int64)
                   if labels else torch.zeros((0,), dtype=torch.int64))
        target = {
            "boxes": boxes_t, "labels": labels_t,
            "image_id": torch.tensor([idx]),
            "area": ((boxes_t[:, 2] - boxes_t[:, 0]) *
                    (boxes_t[:, 3] - boxes_t[:, 1])
                    if len(boxes) else torch.zeros((0,))),
            "iscrowd": torch.zeros((len(boxes),), dtype=torch.int64),
        }
        return TF.to_tensor(img), target, f["image_path"]


def _collate(batch):
    imgs = [b[0] for b in batch]
    targets = [b[1] for b in batch]
    paths = [b[2] for b in batch]
    return imgs, targets, paths


def build_model():
    model = fasterrcnn_mobilenet_v3_large_fpn(
        weights="DEFAULT", min_size=MODEL_MIN_SIZE, max_size=MODEL_MAX_SIZE)
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features,
                                                       NUM_CLASSES)
    return model


def load_split_frames():
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_filtered_FINAL.json")) as f:
        by_path = {r["image_path"]: r
                  for r in json.load(f)["frames"]}
    with open(os.path.join(DATASET_DIR, "splits_final.json")) as f:
        splits = json.load(f)["splits"]
    out = {}
    for name, paths in splits.items():
        out[name] = [by_path[p] for p in paths]
    return out


def load_mined_hard_negatives():
    """Mined by `mine_hard_negatives_v3_final.py` from MISSION-NEG zoom
    tiles only, already ground-truth-verified clean there -- this function
    only reshapes them into the same frame-dict shape `AvianDetectionDataset`
    expects for a negative (`image_path` + `is_positive: False`; no bbox/
    family needed, same contract every other negative in this project
    already uses)."""
    path = os.path.join(DET_DIR, "AVIAN_hard_negatives_v3_FINAL.json")
    with open(path) as f:
        mined = json.load(f)
    return [{"image_path": t["image_path"], "is_positive": False,
            "source": "mission_hard_negative", "tile_id": t["tile_id"]}
           for t in mined["tiles"]]


WARMUP_ITERS = 200
WARMUP_START_FACTOR = 0.01


def train(model, loader, epochs, log):
    device = torch.device("cpu")
    model.to(device)
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=LR, momentum=0.9,
                                weight_decay=5e-4)
    # torchvision's own reference detection training recipe warms up the
    # LR for the first iterations of the FIRST epoch specifically because
    # a freshly-reinitialised box-predictor head combined with the full
    # LR can blow up early (that plus a degenerate bbox is exactly what
    # produced the "loss nan" on the first attempt here, tested not
    # assumed) -- this is the standard fix, not a novel one.
    model.train()
    global_iter = 0
    for epoch in range(epochs):
        t0 = time.time()
        epoch_loss = 0.0
        n_batches = 0
        n_skipped_nonfinite = 0
        for imgs, targets, _ in loader:
            if epoch == 0 and global_iter < WARMUP_ITERS:
                warmup_factor = (WARMUP_START_FACTOR +
                                (1.0 - WARMUP_START_FACTOR) *
                                global_iter / WARMUP_ITERS)
                for g in optimizer.param_groups:
                    g["lr"] = LR * warmup_factor
            elif epoch == 0 and global_iter == WARMUP_ITERS:
                for g in optimizer.param_groups:
                    g["lr"] = LR
            imgs = [im.to(device) for im in imgs]
            targets = [{k: v.to(device) for k, v in t.items()}
                      for t in targets]
            loss_dict = model(imgs, targets)
            loss = sum(loss_dict.values())
            if not torch.isfinite(loss):
                n_skipped_nonfinite += 1
                optimizer.zero_grad()
                global_iter += 1
                continue
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(params, max_norm=5.0)
            optimizer.step()
            epoch_loss += float(loss.item())
            n_batches += 1
            global_iter += 1
        skip_note = (f", {n_skipped_nonfinite} non-finite batches skipped"
                    if n_skipped_nonfinite else "")
        log(f"  epoch {epoch + 1}/{epochs}: loss "
           f"{epoch_loss / max(1, n_batches):.4f} "
           f"({time.time() - t0:.1f}s{skip_note})")


@torch.no_grad()
def evaluate(model, frames, log, score_thresh=SCORE_THRESH_REPORT):
    device = torch.device("cpu")
    model.to(device)
    model.eval()
    ds = AvianDetectionDataset(frames, augment=False)
    records = []
    for i in range(len(ds)):
        img, target, path = ds[i]
        pred = model([img.to(device)])[0]
        keep = pred["scores"] >= score_thresh
        pred_boxes = pred["boxes"][keep]
        pred_labels = pred["labels"][keep]
        pred_scores = pred["scores"][keep]
        gt_boxes = target["boxes"]
        gt_labels = target["labels"]
        records.append({
            "image_path": path,
            "gt_boxes": gt_boxes.tolist(),
            "gt_labels": [ID_TO_FAMILY.get(int(l)) for l in gt_labels],
            "pred_boxes": pred_boxes.tolist(),
            "pred_labels": [ID_TO_FAMILY.get(int(l)) for l in pred_labels],
            "pred_scores": pred_scores.tolist(),
        })
    return records


def compute_ap(all_preds, all_gts, iou_thresh=IOU_MATCH):
    """VOC-style average precision for one class across the whole eval
    set. `all_preds`: list of (image_idx, score, box). `all_gts`: dict
    image_idx -> list of boxes for this class."""
    preds = sorted(all_preds, key=lambda x: -x[1])
    n_gt = sum(len(v) for v in all_gts.values())
    if n_gt == 0:
        return None
    matched = {k: [False] * len(v) for k, v in all_gts.items()}
    tp = np.zeros(len(preds))
    fp = np.zeros(len(preds))
    for i, (img_idx, score, box) in enumerate(preds):
        gts = all_gts.get(img_idx, [])
        if not gts:
            fp[i] = 1
            continue
        ious = box_iou(torch.tensor([box]), torch.tensor(gts))[0]
        best = int(torch.argmax(ious))
        if float(ious[best]) >= iou_thresh and not matched[img_idx][best]:
            tp[i] = 1
            matched[img_idx][best] = True
        else:
            fp[i] = 1
    tp_cum = np.cumsum(tp)
    fp_cum = np.cumsum(fp)
    recall = tp_cum / n_gt
    precision = tp_cum / np.maximum(tp_cum + fp_cum, 1e-9)
    # continuous (all-point) interpolated AP, the standard VOC-2010+ method
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    ap = float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))
    return ap


def score_split(records, log, split_name):
    """Precision/recall per family at SCORE_THRESH_REPORT + mAP@0.5 across
    the whole score range. Every image has AT MOST one ground-truth box
    (this dataset's own structure), so per-family TP/FP/FN and a simple
    per-image confusion matrix both fall out directly."""
    per_family = {fam: {"tp": 0, "fp": 0, "fn": 0} for fam in FAMILIES}
    ap_preds = {fam: [] for fam in FAMILIES}
    ap_gts = {fam: {} for fam in FAMILIES}
    confusion_labels = FAMILIES + ["NONE"]
    cm = np.zeros((len(confusion_labels), len(confusion_labels)), dtype=int)

    for img_idx, r in enumerate(records):
        gt_label = r["gt_labels"][0] if r["gt_labels"] else "NONE"
        if gt_label != "NONE":
            ap_gts[gt_label][img_idx] = [r["gt_boxes"][0]]

        for box, lab, score in zip(r["pred_boxes"], r["pred_labels"],
                                   r["pred_scores"]):
            if lab in ap_preds:
                ap_preds[lab].append((img_idx, score, box))

        # top-1 prediction (if any survive the report threshold) for the
        # per-image confusion matrix / precision-recall-at-threshold table
        pred_label = r["pred_labels"][0] if r["pred_labels"] else "NONE"
        cm[confusion_labels.index(gt_label)][
            confusion_labels.index(pred_label)] += 1
        if gt_label != "NONE":
            if pred_label == gt_label:
                per_family[gt_label]["tp"] += 1
            else:
                per_family[gt_label]["fn"] += 1
                if pred_label != "NONE":
                    per_family[pred_label]["fp"] += 1
        elif pred_label != "NONE":
            per_family[pred_label]["fp"] += 1

    aps = {}
    for fam in FAMILIES:
        ap = compute_ap(ap_preds[fam], ap_gts[fam])
        aps[fam] = ap

    valid_aps = [v for v in aps.values() if v is not None]
    mAP = float(np.mean(valid_aps)) if valid_aps else None

    log(f"  [{split_name}] mAP@0.5 = {mAP if mAP is None else round(mAP, 4)} "
       f"over {len(valid_aps)}/{len(FAMILIES)} families with test instances")
    for fam in FAMILIES:
        s = per_family[fam]
        prec = s["tp"] / max(1, s["tp"] + s["fp"])
        rec = s["tp"] / max(1, s["tp"] + s["fn"])
        log(f"    {fam:20s} AP@0.5={aps[fam]}  P={prec:.3f} R={rec:.3f} "
           f"(tp={s['tp']} fp={s['fp']} fn={s['fn']})")

    return {"mAP@0.5": mAP, "per_family_ap": aps,
           "per_family_prf": {fam: {
               "precision": per_family[fam]["tp"] /
                           max(1, per_family[fam]["tp"] + per_family[fam]["fp"]),
               "recall": per_family[fam]["tp"] /
                        max(1, per_family[fam]["tp"] + per_family[fam]["fn"]),
               **per_family[fam]} for fam in FAMILIES},
           "confusion_matrix": cm.tolist(),
           "confusion_labels": confusion_labels}


def save_confusion_png(cm, labels, out_path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks(range(len(labels)))
    ax.set_yticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_yticklabels(labels)
    ax.set_xlabel("Predicted")
    ax.set_ylabel("Ground truth")
    ax.set_title(title)
    for i in range(len(labels)):
        for j in range(len(labels)):
            ax.text(j, i, str(cm[i][j]), ha="center", va="center",
                   color="white" if cm[i][j] > cm.max() / 2 else "black")
    fig.colorbar(im)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    def log(msg):
        print(msg)

    log("== SIH_AVIAN_FINAL :: train_detector_v3_final.py (Phase 2 sec3) ==")
    log(f"  device  : CPU only (no nvidia-smi, no CUDA torch build)")
    log(f"  model   : fasterrcnn_mobilenet_v3_large_fpn (NOT _320), "
       f"min_size={MODEL_MIN_SIZE} max_size={MODEL_MAX_SIZE}, "
       "COCO-pretrained, BSD license (torchvision) -- same architecture "
       "as v2")
    log(f"  classes : {NUM_CLASSES} (background + {FAMILIES})")
    log("  NOTE: no external real bridge images this pass -- see module "
       "docstring for the measured (~0.30 MB/s) reason")

    split_frames = load_split_frames()
    for name, frames in split_frames.items():
        n_pos = sum(1 for f in frames if f["is_positive"])
        log(f"  {name:5s}: {len(frames)} frames ({n_pos} positive, "
           f"{len(frames) - n_pos} negative)")

    hard_negs = load_mined_hard_negatives()
    log(f"  hard negatives mined from MISSION-NEG tiles: {len(hard_negs)} "
       "(added to TRAIN only -- val/test stay exactly splits_final.json's "
       "frozen synthetic frames)")
    n_train_before = len(split_frames["train"])
    train_frames_v3 = split_frames["train"] + hard_negs
    log(f"  train   : {n_train_before} (v1/v2) -> {len(train_frames_v3)} "
       f"(v3) frames")

    train_ds = AvianDetectionDataset(train_frames_v3, augment=True)
    n_neg_in_ds = sum(1 for i in range(len(train_ds))
                     if train_ds[i][1]["boxes"].shape[0] == 0)
    log(f"  negatives reaching the dataset with an empty target: "
       f"{n_neg_in_ds}/{sum(1 for f in train_frames_v3 if not f['is_positive'])} "
       "-- checked directly, torchvision's Faster R-CNN computes a valid "
       "loss (loss_objectness/loss_classifier) from an empty target, "
       "confirmed with a standalone 2-image test before this run, so "
       "none were silently dropped")
    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,
                              collate_fn=_collate, num_workers=0)

    model = build_model()
    t0 = time.time()
    train(model, train_loader, EPOCHS, log)
    log(f"  train wall time: {time.time() - t0:.1f}s")

    weights_path = os.path.join(DET_DIR, "AVIAN_detector_weights_v3_FINAL.pt")
    torch.save(model.state_dict(), weights_path)
    log(f"  saved   : {weights_path}")

    val_records = evaluate(model, split_frames["val"], log)
    test_records = evaluate(model, split_frames["test"], log)

    log("-- validation split --")
    val_score = score_split(val_records, log, "val")
    log("-- test split (held out) --")
    test_score = score_split(test_records, log, "test")

    cm_path = os.path.join(DET_DIR, "confusion_matrix_v3_FINAL.png")
    save_confusion_png(np.array(test_score["confusion_matrix"]),
                       test_score["confusion_labels"], cm_path,
                       "AVIAN detector v3 (v2 arch + mission hard "
                       f"negatives) -- test-set confusion matrix "
                       f"(n={len(test_records)} frames)")
    log(f"  saved   : {cm_path}")

    eval_out = {
        "epochs": EPOCHS, "batch_size": BATCH_SIZE, "lr": LR,
        "min_size": MODEL_MIN_SIZE, "max_size": MODEL_MAX_SIZE,
        "n_train_before_hard_negatives": n_train_before,
        "n_train": len(train_frames_v3),
        "n_hard_negatives_added": len(hard_negs),
        "n_val": len(split_frames["val"]),
        "n_test": len(split_frames["test"]),
        "n_negatives_in_loader": n_neg_in_ds,
        "val": val_score, "test": test_score,
        "val_records": val_records, "test_records": test_records,
    }
    eval_path = os.path.join(DET_DIR, "AVIAN_detector_eval_v3_FINAL.json")
    with open(eval_path, "w") as f:
        json.dump(eval_out, f, indent=2)
    log(f"  saved   : {eval_path}")
    print("FINAL_TRAIN_COMPLETE")


if __name__ == "__main__":
    main()

"""SIH_AVIAN_FINAL -- Phase 2 step 1 support: read every mask EXR once.

Plain python3 has no EXR reader in this environment (checked: no cv2,
OpenEXR, or imageio installed), so this one pass uses Blender's own image
loader -- the same `_load_exr_array` pattern `dataset_render_final.py`
already uses for VD04 -- to do everything mask-shaped in one place, then
hands off a plain JSON so the rest of Phase 2 (filtering, training) never
needs Blender again.

For every frame in the manifest:
  * positive, `mask_contains_instance` true -> tight pixel bbox for its
    OWN instance id (never a re-derived id -- `assign_instance_ids()`'s
    own numbering, road+metro+steel concatenation order, 1..192).
  * negative -> the SAME VD04 test `dataset_render_final.py`'s own
    `validate_dataset()` runs (any pixel id in [1, 192]), so filtering
    uses the identical definition of "contaminated", not a re-guess.

Usage:
    blender --background --python run_blender.py -- source/extract_masks_final.py
"""
from __future__ import annotations
import json
import os
import sys

import bpy
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
DATASET_DIR = os.path.join(ROOT, "dataset")


def _load_records():
    """Same concatenation order as dataset_final.py's own _load_records()
    (road, then metro, then steel) -- reimplemented here rather than
    imported because that module imports bpy at load time in a way that
    pulls in more than this script needs; the ORDER is what matters, and
    it is copied verbatim, not reinvented."""
    with open(os.path.join(SCENE_DIR,
                          "AVIAN_defect_ground_truth_FINAL.json")) as f:
        records = json.load(f)["defects"]
    with open(os.path.join(SCENE_DIR,
                          "AVIAN_metro_ground_truth_FINAL.json")) as f:
        mrecords = json.load(f)["defects"]
    with open(os.path.join(SCENE_DIR,
                          "AVIAN_steel_ground_truth_FINAL.json")) as f:
        srecords = json.load(f)["defects"]
    return records + mrecords + srecords


def _load_exr_array(path):
    img = bpy.data.images.load(path, check_existing=False)
    try:
        w, h = img.size
        ch = img.channels
        arr = np.empty(w * h * ch, dtype=np.float32)
        img.pixels.foreach_get(arr)
        return arr.reshape(h, w, ch)[::-1]
    finally:
        bpy.data.images.remove(img)


def main():
    all_records = _load_records()
    instance_id_of = {r["defect_id"]: i for i, r in
                      enumerate(all_records, start=1)}
    n_records = len(all_records)

    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_manifest_FINAL.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]

    out = {}
    n_bbox = n_empty_bbox = n_contaminated = n_clean_neg = 0
    for i, f in enumerate(frames):
        mask_path = os.path.join(ROOT, f["mask_path"])
        arr = _load_exr_array(mask_path)
        ids = arr[..., 0]
        if f["is_positive"]:
            iid = instance_id_of.get(f["defect_id"])
            ys, xs = np.where(np.abs(ids - iid) < 0.5) if iid else \
                (np.array([]), np.array([]))
            if xs.size and ys.size:
                bbox = [int(xs.min()), int(ys.min()),
                       int(xs.max()) + 1, int(ys.max()) + 1]
                n_bbox += 1
            else:
                bbox = None
                n_empty_bbox += 1
            out[f["image_path"]] = {"bbox": bbox, "contaminated": None}
        else:
            contaminated = bool(((ids >= 1) & (ids <= n_records)).any())
            if contaminated:
                n_contaminated += 1
            else:
                n_clean_neg += 1
            out[f["image_path"]] = {"bbox": None,
                                   "contaminated": contaminated}
        if (i + 1) % 100 == 0:
            print(f"  ... {i + 1}/{len(frames)} masks read")

    out_path = os.path.join(DATASET_DIR, "AVIAN_mask_extraction_FINAL.json")
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"  bbox    : {n_bbox} positives with a real bbox, "
         f"{n_empty_bbox} positives with an empty/missing mask (VD01)")
    print(f"  neg     : {n_clean_neg} clean, {n_contaminated} contaminated "
         "(VD04)")
    print(f"  saved   : {out_path}")
    print("FINAL_MASK_EXTRACTION_COMPLETE")


if __name__ == "__main__":
    main()

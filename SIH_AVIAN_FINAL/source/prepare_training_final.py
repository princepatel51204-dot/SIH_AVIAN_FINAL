"""SIH_AVIAN_FINAL -- Phase 2 step 1+3: filter the dataset, then split it.

Plain python3, no bpy -- reads the manifest, `AVIAN_mask_extraction_FINAL.json`
(pre-computed by `extract_masks_final.py`'s one Blender pass over every mask
EXR), and `labels_final.json`.

FILTERING (exclusion, not repair, per the master prompt):
  * VD01 -- drop positive frames whose own instance never showed up in
    their own mask (`bbox is None` in the mask-extraction file).
  * VD04 -- drop negative frames whose mask is contaminated with a real
    defect instance id.
  * VD03 -- KEPT, but tagged `standoff_drift_flagged` (>5% commanded vs
    achieved standoff) so any size-dependent step can exclude them without
    losing them for plain presence/class training.
  * VD06 -- the 9 missing `view_obliquity_deg` values are filled from the
    ground truth's own `recommended_view_obliquity_deg` for that exact
    defect_id (the same number the render pipeline should have copied in
    the first place), not fabricated.

SPLITTING (instance-level, not frame-level -- the leakage trap): every
frame of a given positive defect_id goes to exactly one split, and every
frame sharing a negative's `matched_defect_id` (its real "source member")
does too. Stratified by coarse family (positives) / sound-category label
(negatives). Fixed seed for reproducibility.
"""
from __future__ import annotations
import json
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
DATASET_DIR = os.path.join(ROOT, "dataset")

SPLIT_FRACTIONS = {"train": 0.70, "val": 0.15, "test": 0.15}
SEED = 20260923


def _load_ground_truth_obliquity():
    out = {}
    for fn in ("AVIAN_defect_ground_truth_FINAL.json",
              "AVIAN_metro_ground_truth_FINAL.json",
              "AVIAN_steel_ground_truth_FINAL.json"):
        with open(os.path.join(SCENE_DIR, fn)) as f:
            for r in json.load(f)["defects"]:
                if r.get("recommended_view_obliquity_deg") is not None:
                    out[r["defect_id"]] = r["recommended_view_obliquity_deg"]
    return out


def _split_groups(group_ids, seed=SEED):
    """Deterministic 70/15/15 split of a list of group ids."""
    ids = sorted(group_ids)   # sort first so shuffle is reproducible
                              # regardless of dict/set iteration order
    random.Random(seed).shuffle(ids)
    n = len(ids)
    n_train = round(n * SPLIT_FRACTIONS["train"])
    n_val = round(n * SPLIT_FRACTIONS["val"])
    train = set(ids[:n_train])
    val = set(ids[n_train:n_train + n_val])
    test = set(ids[n_train + n_val:])
    return train, val, test


def main():
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_manifest_FINAL.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_mask_extraction_FINAL.json")) as f:
        mask_info = json.load(f)
    with open(os.path.join(DATASET_DIR, "labels_final.json")) as f:
        labels = json.load(f)
    type_to_family = labels["type_to_family"]
    obliquity_fallback = _load_ground_truth_obliquity()

    # ---- VD06: fill, else this would be the only remaining null -------
    n_filled = 0
    for f in frames:
        if f.get("view_obliquity_deg") is None:
            fill = obliquity_fallback.get(f.get("defect_id"))
            if fill is not None:
                f["view_obliquity_deg"] = fill
                f["view_obliquity_deg_filled"] = True
                n_filled += 1

    # ---- VD01 / VD04 exclusion, VD03 tagging ---------------------------
    kept = []
    n_drop_vd01 = n_drop_vd04 = n_tag_vd03 = 0
    for f in frames:
        info = mask_info.get(f["image_path"], {})
        if f["is_positive"]:
            if info.get("bbox") is None:
                n_drop_vd01 += 1
                continue
            f["bbox_xyxy"] = info["bbox"]
        else:
            if info.get("contaminated"):
                n_drop_vd04 += 1
                continue
        cmd = f.get("commanded_standoff_m")
        got = f.get("subject_distance_m")
        drift_flagged = bool(cmd and got is not None
                            and abs(got - cmd) / cmd > 0.05)
        f["standoff_drift_flagged"] = drift_flagged
        if drift_flagged:
            n_tag_vd03 += 1
        kept.append(f)

    print("== SIH_AVIAN_FINAL :: prepare_training_final.py (Phase 2) ==")
    print(f"  VD06    : {n_filled} view_obliquity_deg values filled from "
         "ground truth's own recommended_view_obliquity_deg")
    print(f"  VD01    : dropped {n_drop_vd01} positive frames (mask did "
         "not contain its own instance)")
    print(f"  VD04    : dropped {n_drop_vd04} negative frames (mask "
         "contaminated with a real defect instance)")
    print(f"  VD03    : tagged {n_tag_vd03} kept frames "
         "standoff_drift_flagged (>5% drift) -- excluded from any "
         "size-dependent use, kept for presence/class training")
    print(f"  kept    : {len(kept)}/{len(frames)} frames "
         f"({sum(1 for f in kept if f['is_positive'])} positive, "
         f"{sum(1 for f in kept if not f['is_positive'])} negative)")

    # ---- instance-level grouping ---------------------------------------
    pos = [f for f in kept if f["is_positive"]]
    neg = [f for f in kept if not f["is_positive"]]
    for f in pos:
        f["family"] = type_to_family[f["label"]]

    pos_by_family = {}
    for f in pos:
        pos_by_family.setdefault(f["family"], set()).add(f["defect_id"])
    neg_by_label = {}
    for f in neg:
        neg_by_label.setdefault(f["label"], set()).add(f["matched_defect_id"])

    split_of_instance = {}   # defect_id / matched_defect_id -> split name
    for family, ids in pos_by_family.items():
        train, val, test = _split_groups(ids, seed=SEED)
        for iid in train:
            split_of_instance[iid] = "train"
        for iid in val:
            split_of_instance[iid] = "val"
        for iid in test:
            split_of_instance[iid] = "test"
    for label, ids in neg_by_label.items():
        train, val, test = _split_groups(ids, seed=SEED + 1)
        for iid in train:
            split_of_instance[iid] = "train"
        for iid in val:
            split_of_instance[iid] = "val"
        for iid in test:
            split_of_instance[iid] = "test"

    splits = {"train": [], "val": [], "test": []}
    for f in pos:
        splits[split_of_instance[f["defect_id"]]].append(f["image_path"])
    for f in neg:
        splits[split_of_instance[f["matched_defect_id"]]].append(
            f["image_path"])

    # ---- leakage check: no instance in more than one split -------------
    frame_by_path = {f["image_path"]: f for f in kept}
    instance_splits = {}
    for split_name, paths in splits.items():
        for p in paths:
            f = frame_by_path[p]
            iid = f["defect_id"] if f["is_positive"] else f["matched_defect_id"]
            instance_splits.setdefault(iid, set()).add(split_name)
    leaks = {iid: s for iid, s in instance_splits.items() if len(s) > 1}
    print(f"  leakage : {len(leaks)} instance ids appear in more than one "
         f"split (checked {len(instance_splits)} instances) "
         f"-- {'FAIL' if leaks else 'PASS'}")
    if leaks:
        print(f"            {list(leaks.items())[:10]}")
        raise SystemExit("leakage check failed -- refusing to write splits")

    # ---- report: frames kept per split, per type -----------------------
    print("  per-split frame counts by defect type:")
    from collections import Counter
    for split_name in ("train", "val", "test"):
        c = Counter(frame_by_path[p]["label"] for p in splits[split_name])
        n_pos = sum(1 for p in splits[split_name]
                   if frame_by_path[p]["is_positive"])
        n_neg = len(splits[split_name]) - n_pos
        print(f"    {split_name:5s}: {len(splits[split_name])} frames "
             f"({n_pos} positive, {n_neg} negative)")
        for t, n in sorted(c.items(), key=lambda kv: -kv[1]):
            print(f"        {n:4d}  {t}")

    print("  per-split frame counts by coarse family (positives only):")
    for split_name in ("train", "val", "test"):
        c = Counter(frame_by_path[p]["family"] for p in splits[split_name]
                   if frame_by_path[p]["is_positive"])
        print(f"    {split_name:5s}: {dict(c)}")

    out = {
        "seed": SEED,
        "fractions": SPLIT_FRACTIONS,
        "n_kept_total": len(kept),
        "n_dropped_vd01": n_drop_vd01,
        "n_dropped_vd04": n_drop_vd04,
        "n_tagged_vd03": n_tag_vd03,
        "n_filled_vd06": n_filled,
        "leakage_check": "PASS" if not leaks else "FAIL",
        "splits": splits,
    }
    out_path = os.path.join(DATASET_DIR, "splits_final.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  saved   : {out_path}")

    # also persist the filtered/enriched per-frame records for training
    filtered_path = os.path.join(DATASET_DIR,
                                 "AVIAN_dataset_filtered_FINAL.json")
    with open(filtered_path, "w") as f:
        json.dump({"frames": kept}, f, indent=2)
    print(f"  saved   : {filtered_path}")
    print("FINAL_PREPARE_COMPLETE")


if __name__ == "__main__":
    main()

# AVIAN detector report — Phase 2

## Model, library, license

`fasterrcnn_mobilenet_v3_large_320_fpn` (torchvision 0.29.0+cpu), COCO-pretrained,
fine-tuned. **torchvision is BSD-3-Clause** — chosen over Ultralytics YOLOv8
specifically because YOLOv8 is AGPL-3.0 and this project is meant to be
handed over, not just demoed; a BSD dependency carries no obligation on
the code that uses it. Checked `nvidia-smi` before picking a model: not
present, so this ran CPU-only throughout, which is also why the smallest
torchvision detector (320×320 internal resize) was picked over a larger
one.

## Dataset, after filtering (not the raw 798)

| filter | frames dropped/tagged | reason |
|---|---:|---|
| VD01 | −78 positives | mask did not contain its own labeled instance |
| VD04 | −121 negatives | mask contaminated with a real defect instance |
| VD03 | 123 tagged, kept | >5% standoff drift — kept for class training, excluded from any size estimate |
| VD06 | 9 filled | `view_obliquity_deg` filled from ground truth's own `recommended_view_obliquity_deg` |

**Kept: 599/798 frames (286 positive, 313 negative).** This matches the
master prompt's own predicted ceiling (~286 positive / ~313 negative)
exactly.

**Correction against the master prompt**: it estimated "192 defects
across 19 types." The real rendered dataset has **28 distinct positive
defect types**, not 19 — METRO alone contributes 8 types (PIER_SPALL,
PIER_CORROSION_STAIN, INTERNAL_SOFFIT_CRACK, WEB_SHEAR_CRACK,
SOFFIT_TRANSVERSE_CRACK, SEGMENTAL_JOINT_LEAKAGE, BEARING_DISTRESS,
EFFLORESCENCE_JOINT) beyond the road+steel taxonomy the "19" estimate was
based on. Checked directly against `dataset/AVIAN_dataset_manifest_FINAL.json`,
not assumed — see `dataset/labels_final.json`'s own docstring.

## Label set: 5 coarse families (`dataset/labels_final.json`)

28 types over 364 positive frames is ~7 instances/type on average, several
in the 3-6 range — a 28-class detector would overfit and report a
meaningless mAP. Grouped into 5 families (rationale for each grouping is
written into `labels_final.json` itself, not just here):

| family | unique instances | types |
|---|---:|---|
| CRACK | 69 | 8 crack-shaped types incl. WELD_CRACK (grouped by appearance, not substrate) |
| FASTENER | 47 | 6 hardware-connection types |
| SPALL_DELAM | 34 | 5 broad-area material-loss types |
| CORROSION_COATING | 22 | 4 surface-discoloration types |
| OTHER | 20 | 5 catch-all types (bearing/joint/conduit/leakage) |

The 28-type breakdown is reported below too, clearly as low-sample and
indicative only — not headlined.

## Split (instance-level, leakage-checked)

70/15/15, stratified by family (positives) / sound-category (negatives),
split on `defect_id` / `matched_defect_id` so every frame of one physical
defect (or one negative's real source member) lands in exactly one split.
Seed `20260923`, `source/prepare_training_final.py`.

| split | frames | positive | negative | unique positive instances | unique negative source members |
|---|---:|---:|---:|---:|---:|
| train | 415 | 201 | 214 | 111 | 74 |
| val | 94 | 44 | 50 | 25 | 15 |
| test | 90 | 41 | 49 | 24 | 17 |

**Leakage check** (`prepare_training_final.py`'s own output, reproduced verbatim):
```
leakage : 0 instance ids appear in more than one split (checked 170 instances) -- PASS
```

## Training

SGD, lr 0.004 with a 200-iteration warmup (a fresh box-predictor head at
full LR produced NaN losses on the first attempt — fixed, see
`train_detector_final.py`'s own comments), batch size 2, 15 epochs,
gradient clipping at norm 5.0. Bounding boxes are tight pixel boxes read
directly off the instance masks (`extract_masks_final.py`), never
hand-drawn or re-inferred. Augmentation: horizontal flip, brightness/
contrast jitter, occasional blur.

**Loss plateaued early**: 0.896 (epoch 1) → 0.418 (epoch 2), then
oscillated between 0.39 and 0.53 for the remaining 13 epochs with no
clear further descent. Checked before using the model, per instruction:
this is a real, reportable weakness, not a "stop and discard" case —
val mAP (0.1136) is low but not uniformly zero (CRACK shows real
precision/recall on val: P=0.526, R=0.667, on the family with the most
training instances), so the pipeline demonstrably learned *something*
real rather than nothing. It is reported here as a plateau, not
papered over as a clean training curve.

A handful of augmented batches (2-3 per epoch on some epochs) produced
non-finite loss and were skipped rather than allowed to corrupt the
running weights — almost certainly from an aggressive brightness/contrast
jitter draw on an already near-black or near-white LOOP-resolution frame;
harmless to the run (skipped, not crashed) but worth knowing about if
retraining.

## Test-set results (held out, never touched during training)

**Test set: 90 frames, 24 unique positive defect instances.**

**mAP@0.5 = 0.342** (this number is noisy at this sample size — see the
per-family column below before treating it as a single headline figure).

| family | AP@0.5 | precision | recall | tp | fp | fn | test instances |
|---|---:|---:|---:|---:|---:|---:|---:|
| CRACK | 0.293 | 0.409 | 0.450 | 9 | 13 | 11 | 20 |
| SPALL_DELAM | 0.500 | 1.000 | 0.375 | 3 | 0 | 5 | 8 |
| OTHER | 0.917 | 0.750 | 1.000 | 3 | 1 | 0 | 3 |
| CORROSION_COATING | 0.000 | 0.000 | 0.000 | 0 | 1 | 2 | 2 |
| FASTENER | 0.000 | 0.000 | 0.000 | 0 | 0 | 8 | 8 |

**Read this table with the instance counts, not just the percentages**:
OTHER's 0.917 AP comes from 3 test instances — a single lucky or unlucky
frame swings that number by 33 points. FASTENER's flat 0.000 despite
being the *second-largest* training family (47 unique instances, 43
train frames) is the one number here that is NOT a small-sample artifact
and is a genuine, specific failure (see failure mode 2 below).

Confusion matrix (per-image, true label × top-1 predicted label at
confidence ≥ 0.5, `NONE` = no ground truth / no detection above
threshold): **`detection/confusion_matrix_FINAL.png`**.

## Two named failure modes (real test errors, with example images)

1. **False-positive "CRACK" on plain concrete texture.** 10 of 49 test
   SOUND_CONCRETE negatives were flagged CRACK at confidence 0.52-0.70,
   with no crack present. Example: `dataset/images/NEG_SOUND_CONCRETE_0305_0001.png`
   (confidence 0.600), `dataset/images/NEG_SOUND_CONCRETE_0306_0001.png`
   (confidence 0.705). The model appears to have learned "textured grey
   surface" as a partial proxy for "crack" rather than the linear feature
   itself — consistent with the CRACK family's own false-positive count
   (13 on the test set, the highest of any family).

2. **Confuses CORROSION_COATING with CRACK, and misses FASTENER
   entirely.** `dataset/images/POS_081_MDR_LOOP_0001.png` and its SURVEY
   companion (ground truth: CORROSION_COATING) were both predicted CRACK
   (confidence 0.67 and 0.55) — a rust stain's mottled edge is apparently
   read as a linear feature at this resolution. Separately, FASTENER
   (bolts, anchors, handrail hardware) was never correctly detected on
   either val or test (0/9 val, 0/8 test) despite being the second-best-
   represented family in training — the small, discrete, metallic objects
   this family covers may need a different anchor-box scale than this
   model's defaults provide, not just more data.

## The headline finding: a domain gap the dataset never tested for

The numbers above are all measured on frames from the **same rendering
pipeline and standoff range** (0.3-1.5 m, MDR/SURVEY) the model trained
on. Stage 2's actual coverage-mission renders — the images this detector
would need to work on in the real autonomy pipeline — are shot from a
**structural-survey standoff of 8.0 m**, chosen by `coverage_final.py`
for a completely different reason (covering the whole bridge's surface
area is intractable at close range; see that module's own docstring) and
never reconciled with what the detector would need.

Running the trained model on all 82 settled Stage 2 waypoints' rendered
views, matched to projected ground-truth positions by IoU (≥0.10, family
must match — not "was the defect anywhere in frame"):

**Real precision = 0.000, real recall = 0.000 (0 true positives, 3 false
positives, 109 false negatives).**

This is not the same failure as the weak per-family test numbers above —
it is a near-total collapse, and the standoff gap is the most likely
single cause: a defect that occupies a meaningful fraction of the frame
at 0.3 m is a handful of pixels at 8.0 m (roughly (8.0/0.3)² ≈ 700× less
image area), which is a different visual problem than anything in the
training set. Stated plainly because it is the most decision-relevant
number in this report: **the detector, as trained, is not usable on
Stage 2's own mission imagery without either retraining on wide-standoff
frames or redesigning the coverage mission to fly closer once a candidate
region is flagged** (a two-tier survey-then-verify pattern, not built
this pass).

## Synthetic-to-real caveat

Trained and tested exclusively on Blender renders (`SIH_AVIAN_FINAL.blend`).
No real bridge-inspection imagery (e.g. CODE-BRIM) was run through this
model this pass — the stretch goal in the master prompt was not attempted
given the time already spent on the standoff-gap finding above, which is
the more decision-relevant result for this deck. A real-image evaluation,
if attempted later, should be read as measuring on top of both gaps
(synthetic-to-real AND standoff), not instead of the standoff one.

## Before / after (stub vs. real, same 82 settled Stage 2 waypoints)

| | stub (Phase 1, cheats via ground truth) | real (Phase 2, trained model) |
|---|---:|---:|
| precision | 100.0% (by construction — see caveat) | 0.0% |
| recall (of 73 non-escalated defects) | 53.4% | 0.0% |
| detections logged | 152 | 3 |

The stub's numbers were never a detector result (its own docstring said
so at the time); this table exists to make the before/after visible, per
the master prompt's own instruction, not to claim the stub was ever
competing fairly.

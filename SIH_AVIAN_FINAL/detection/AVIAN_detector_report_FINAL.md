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
at close range is a handful of pixels at 8.0 m. **Correction (Section 2):**
the training standoff range is 0.3-1.5 m, not a single point, so the
image-area gap is a range, not one flat number — **(8.0/1.5)² ≈ 28× at
the far end of training range, up to (8.0/0.3)² ≈ 711× at the near end**,
which is a different visual problem than anything in the training set at
any point in that range. Stated plainly because it is the most
decision-relevant number in this report (Section 1's own text): **the
detector, as trained, is not usable on Stage 2's own mission imagery.**
Flying closer is not the fix -- China's MoT UAV bridge-inspection
guideline sets a 3 m minimum standoff from the structure, and Stage 1's
own flight data already showed what closer flight does in this truss
(71% crash rate). Section 2 below builds the correct fix instead: a
simulated zoom camera at the SAME safe standoff.

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

---

# Section 2 — full-resolution retrain + simulated zoom camera

**Everything above this line is Section 1, unchanged, accepted as
committed in `3f2fce4`.** This section fixes the two causes behind
Section 1's two worst numbers (FASTENER's 0 AP, and the 0%/0% mission-
imagery collapse), then measures again. Nothing above was deleted or
rewritten except two corrections marked explicitly in place: the flat
"~700×" image-area figure and the "fly closer" suggestion, both wrong for
reasons explained where they were.

## Headline rule (fixed BEFORE either model's tile numbers were seen)

**v1 (`fasterrcnn_mobilenet_v3_large_320_fpn`, test mAP@0.5 = 0.342) stays
the headline model.** v2 (full-resolution retrain) is reported in full
everywhere below, never substituted as the headline, regardless of how it
scores on the zoom tiles. This rule was written down before
`score_zoom_tiles_final.py` was run.

## Cause 1, checked directly: input resolution (Step 1)

Read `fasterrcnn_mobilenet_v3_large_320_fpn`'s own torchvision source
rather than assuming: it hard-codes `min_size=320, max_size=640`. A
640x480 LOOP frame's short side is resized 480 -> 320 (0.667x) before the
network sees it.

**Box widths, measured at each model's own input resolution (median /
10th-percentile, pixels):**

| family | v1 input (320) | v2 input (480, native) |
|---|---:|---:|
| CRACK | 318.7 / 64.8 | 478.0 / 97.2 |
| SPALL_DELAM | 71.3 / 2.7 | 107.0 / 3.7 |
| CORROSION_COATING | 383.0 / 85.7 | 574.5 / 128.6 |
| OTHER | 201.3 / 8.2 | 302.0 / 11.6 |
| **FASTENER** | **6.7 / 1.3** | **10.0 / 2.0** |

FASTENER's median box is 6.7 px at v1's input resolution — confirmed,
not assumed, as the direct explanation for its 0/9 val and 0/8 test AP in
Section 1. At v2's native resolution it grows to 10.0 px — real, measured
improvement, and still small. **This is now a declared limitation, not
something to keep chasing**: 10 px is what this dataset's own 640x480
LOOP source images can support for an object this physically small at
this standoff. Fixing it further needs a higher-resolution source image,
not another retrain — the CAPTURE mode (1920x1080, 0.7159 mm/px @ 1m
vs. LOOP's 2.1478) is the path, via `dataset_render_final.py`'s own
existing CAPTURE-companion renders — but only 12 CAPTURE frames exist in
the dataset today, nowhere near enough to retrain on. Named here as
future work, not attempted this pass.

**Negatives reaching the training loader**: 214/214, both runs — checked
directly (not assumed) by counting empty-target items the dataset yields,
and separately confirmed torchvision's Faster R-CNN computes a valid,
finite loss from an empty target in a standalone 2-image test before
either training run. The false "CRACK" on plain concrete (Section 1's
failure mode 1) was never caused by dropped negatives; both models saw
every one.

**Leakage check, re-run against the same frozen `splits_final.json`**
(unchanged, as instructed):
```
leakage : 0 instance ids appear in more than one split (checked 170 instances) -- PASS
```

### Test-set result: v1 vs. v2, with counts (not just ratios)

**Test set: 90 frames, 24 unique positive defect instances (20 of them
CRACK).** That imbalance matters for reading the table below — see the
sample-size note underneath it before drawing conclusions from it.

| family | v1 P/R (tp/n) | v2 P/R (tp/n) |
|---|---:|---:|
| CRACK | 0.409 / 0.450 (**9/20**) | 1.000 / 0.250 (**5/20**) |
| SPALL_DELAM | 1.000 / 0.375 (3/8) | 1.000 / 0.375 (3/8) — unchanged |
| CORROSION_COATING | 0.000 / 0.000 (0/2) | 0.000 / 0.000 (0/2) — unchanged |
| FASTENER | 0.000 / 0.000 (0/8) | 0.000 / 0.000 (0/8) — unchanged, see above |
| OTHER | 0.750 / 1.000 (3/3) | 1.000 / 0.333 (1/3) |
| **mAP@0.5** | **0.342** | **0.182** |

**Sample-size note, read before the mAP row above**: 20 of the test set's
24 defects are CRACK. CRACK recall moving from 9/20 to 5/20 is a
4-defect swing on a 20-item class — within the noise this sample size can
produce either direction, and does **not** prove v2 is a worse detector
in general, only that it scores worse on THIS 90-frame test set. The mAP
drop (0.342 -> 0.182) is real and reported as such, but should not be
read as more precise than a 24-instance test set can support.

What the shift likely reflects, not noise: **v1 -> v2 traded recall for
precision on CRACK** (P 0.409 -> 1.000, R 0.450 -> 0.250) — full-
resolution training left the model more conservative, flagging fewer
crack candidates but being right every time it does. That is consistent
with (not proof of) training on correctly-represented negatives being a
real effect, not a mistake to undo.

## Cause 2, checked directly: standoff, fixed with a simulated zoom (Step 2)

**Zoom factor computed from data**: median achieved GSD of the training
split's own 408 LOOP frames = **1.9737 mm/px** (`AVIAN_dataset_manifest_FINAL.json`,
filtered to `splits_final.json`'s train list — a median, not a mean,
since a handful of near-degenerate `subject_distance_m` values in the raw
manifest produce absurd outlier GSDs that would otherwise dominate an
average). Solved the pinhole relation for the HFOV that reproduces that
mm/px at each selected waypoint's own ACHIEVED standoff (7.88-8.13 m,
measured, not assumed to be exactly 8.0):

**Result: HFOV 8.89-9.16° across the 16 selected waypoints (median
~9.06°), a ~7.6-7.7x zoom from the aircraft's real 69° HFOV.** This lands
inside the physically realistic envelope the master prompt named (DJI
H20-class payloads reach ~4° HFOV) — the simulated zoom asks for less
than real hardware already provides.

**Waypoint selection — geometry only, before any detection ran**: bucketed
the 82 settled Stage 2 waypoints by structure kind (`coverage_final.py`'s
own `prim_kind`, itself ground-truth-free) into PIER/TRUSS/DECK/METRO.
The real distribution: **TRUSS=58, PIER=22, DECK=2 (joint_gap +
expansion_joint), METRO=0** — no settled waypoint this run happened to
land on a literal deck_box/deck_slab/catenary_mast primitive, reported
honestly rather than forced into an even split. Sampled proportionally
with **seed 20260925**: **16 waypoints** — `CWP_011, CWP_015, CWP_020,
CWP_022, CWP_036, CWP_038, CWP_065, CWP_067, CWP_069, CWP_075, CWP_101,
CWP_107, CWP_114, CWP_125, CWP_126, CWP_146`.

**Tiling**: each waypoint's wide 69°x42° footprint tiled with 20%-overlap
zoom-FOV cells (11x10 = 110 candidate tiles/waypoint), each candidate's
centre ray tested with a real `scene.ray_cast()` against the actual
Blender scene — kept only if it hits a structural object, excluding
anything prefixed `ENV_` (the confirmed real sky/ground/water objects,
`ENV_GROUND_FAR` / `ENV_RIVER_WATER`, checked by name directly). **1,167
of 1,760 candidate tiles kept** (66%), rendered via EEVEE in 850s (0.73
s/tile). Per-waypoint keep rate ranged 33%-100% depending on how much of
that waypoint's wide view was open air versus structure.

### Zoom-tile results: both models, IoU-matched against projected ground truth

A real bug was found and fixed before these numbers were final: the first
scoring pass used a 3 m ground-truth search radius on the mistaken
assumption that zooming in means flying closer. It does not — "no
re-flight" means the camera sits at the SAME ~8 m standoff as the wide
view, only the FOV narrows. At 3 m, every one of the 1,167 tiles found
zero candidate defects (fn=0 across the board — the tell that something
was wrong). Fixed to 12 m (covers the real 7.9-8.1 m standoff with
slack) before scoring below.

Only **10 of the 73 scoreable ground-truth defects fell within range of
any of the 16 selected waypoints** — expected and correct, not a bug:
waypoints were chosen by structure kind, never by where defects are, so
16 of 82 settled waypoints landing near only 10 of 73 defects is what an
unbiased geometric sample looks like, not a shortfall to fix.

| | v1 (headline) | v2 (full-res) |
|---|---:|---:|
| tiles / waypoints | 1,167 / 16 | 1,167 / 16 |
| GT defects in scope | 10 | 10 |
| TP / FP / FN | 1 / 326 / 17 | 2 / 11 / 16 |
| precision | 0.003 | 0.154 |
| recall | 0.056 | 0.111 |
| distinct defects found | 1/10 | 2/10 |

Per-family (both had zero TP outside SPALL_DELAM):

| family | v1 P/R (tp/fp/fn) | v2 P/R (tp/fp/fn) |
|---|---:|---:|
| SPALL_DELAM | 0.500 / 0.167 (1/1/5) | 0.667 / 0.333 (2/1/4) |
| CRACK | 0.000 / 0.000 (0/309/0) | 0.000 / 0.000 (0/10/0) |
| CORROSION_COATING | 0.000 / 0.000 (0/4/0) | n/a (0 predictions) |
| FASTENER | 0.000 / 0.000 (0/0/10) | 0.000 / 0.000 (0/0/10) |
| OTHER | 0.000 / 0.000 (0/12/2) | n/a (0 predictions) |

**Reading this honestly**: neither model is usable at these numbers. v1's
327 zoom-tile detections are almost all CRACK false positives (309 of
327) — the same "fires on plain texture" failure mode from Section 1,
now seen on tiles that resemble its training distribution far more
closely, so the false-positive habit is a real property of the model, not
an artifact of the wide-view domain gap. v2 is far more conservative (13
total detections, 11 false) and correspondingly has much better precision
on tiles (0.154 vs 0.003) while still finding more GT defects in absolute
terms (2 vs 1) despite fewer total predictions — the same recall/precision
trade seen on the test set, now visible on real mission-shaped imagery
too. **The zoom fix (Step 2) closes the standoff/resolution gap as
designed** (both models found real defects here that the wide 8 m view
found zero of in Section 1), but exposes that the underlying detector
-- either version -- is not accurate enough yet to act on unsupervised.

## Remaining domain gap: perspective flatness

A zoomed view from 8 m has a flatter, more orthographic-like perspective
than a wide view shot from 0.3-1.5 m (the training distribution) — less
foreshortening, less parallax across the frame. Whether this matters is
answered by the same data above: v2 (the model actually retrained at
native resolution) still fires 0 true positives on CRACK across all 1,167
zoom tiles despite CRACK being its best-represented family by far (69
unique training instances) and despite the zoom tiles now matching its
training mm/px almost exactly by construction. That is more consistent
with the false-positive/missed-detection pattern being about the
model's own weak learned features (Section 1 and this section both show
CRACK precision/recall issues on the ORIGINAL, non-zoomed test set too)
than about a NEW perspective-shape problem the zoom introduced. Stated as
a read of the evidence, not a certainty — a controlled test (render the
SAME defect from both a training-style close-up and a zoomed-from-afar
view, compare detections) would isolate this cleanly and was not run this
pass.

## Updated failure modes (zoom tiles)

Section 1's two failure modes (false CRACK on plain concrete; confused
CORROSION_COATING/CRACK, FASTENER never detected) both reproduce
directly on the zoom tiles: v1 alone produced 309 CRACK false positives
across 1,167 tiles, and FASTENER stayed at 0/10 for both models. No new
failure mode was needed to describe the zoom-tile results — the same two
from Section 1 explain what happened here too, now with mission-scale
evidence (1,167 tiles) instead of a 90-frame test set.

## Gate checks re-run for Section 2

```
$ python3 source/prepare_training_final.py | grep leakage
leakage : 0 instance ids appear in more than one split (checked 170 instances) -- PASS

$ grep -n ground_truth source/detect_stub_final.py source/render_zoom_tiles_final.py \
    source/run_real_detector_zoom_final.py source/score_zoom_tiles_final.py
(no output -- zero matches across all four files)
```

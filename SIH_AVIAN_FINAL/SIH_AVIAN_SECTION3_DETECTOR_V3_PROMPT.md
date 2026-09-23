# SECTION 3 — Detector v3: real bridge data, hard negatives, a bigger honest test

**Time box: end of 27 Sep 2026.** The deck gets refreshed with final numbers
on 28–29 Sep and submitted by 30 Sep (a draft already exists in `deck/`, so
the deadline is safe whatever happens here). If a step isn't done by the time
box, stop, commit what's real, and write the handoff.

Section 2 is accepted as committed in `87b2965`. Nothing in it is rewritten;
v3 is added next to v1 and v2.

**Out of scope:** flight and autonomy code (Phase 1 numbers are frozen),
Gazebo and the dashboard. **Keep frozen:** `splits_final.json`.

---

## Why v1 and v2 fail, per the Section 1–2 reports

1. **Too little data.** 192 defects across 19 types is about 10 examples per
   type, and all of them are synthetic. Both models learned "rough texture
   means CRACK": v1 fired 309 false cracks on 1,167 zoom tiles.
2. **The mission test is too small to measure progress.** 10 defects were in
   scope, so "1 of 10" versus "2 of 10" is noise.
3. **A possible leak nobody has checked.** Mission-tile scoring counted
   defects whose other views may have been in the detector's *training*
   split. If so, the mission numbers are optimistic.

## Step 0 — Check the leak and fix one label (small, do first)

- For the 10 defects in scope in Section 2, report how many have an
  `instance_id` in the detector's train, val or test split. Add one line
  with that count to the Section 2 results in the report.
- In the report, correct the zoom factor. 69° → 9.06° HFOV is
  tan(34.5°)/tan(4.53°) ≈ **8.7× optical zoom**. The "~7.6–7.7×" figure is
  an angle ratio, not a zoom factor.

## Step 1 — A bigger mission test set, fixed before any training

- Render zoom tiles for **all** settled Stage 2 waypoints, not just 16. Use
  the Section 2 tiling code unchanged (0.73 s/tile, so roughly 6,000 tiles
  is about 75 minutes). Keep the same rule: tiles are chosen by geometry,
  never by ground truth.
- **Split the waypoints now, before training**, stratified by structure kind
  with a fixed seed. Write the split to `detection/mission_split_v3.json`
  and commit it:
  - **MISSION-NEG** (about 40%): used only to mine hard negatives in Step 2.
  - **MISSION-VAL** (about 20%): used to pick the score threshold and the
    headline model.
  - **MISSION-TEST** (about 40%): scored once at the end.
- **Split rule:** any tile that shows a ground-truth defect is dropped
  from MISSION-NEG, so no defect enters training through this route.
- **Instance rule:** at the end, report MISSION-TEST recall two ways.
  - **Unseen defects (headline):** instances in the detector's test split
    that never appeared as a training positive from any source.
  - **All defects in scope:** labelled "includes defects seen in training,
    so optimistic".
  - Write an assertion for the unseen set and paste its output.

## Step 2 — Train v3

Start from v2's architecture: `fasterrcnn_mobilenet_v3_large_fpn`,
`min_size=480`, BSD-3.

1. **Add real bridge images.** Use CODEBRIM (Mundt et al., CVPR 2019): real
   bridge photos with bounding boxes for crack, spallation, exposed bars,
   efflorescence and corrosion stain. Add dacl10k (WACV 2024) if CPU time
   allows.
   - **Check each dataset's license first.** Write it in the README. If it's
     non-commercial, say so; that's fine for a prototype but must be
     disclosed.
   - Map their classes onto `labels_final.json`: crack → CRACK;
     spallation / exposed bars → SPALL_DELAM; efflorescence / corrosion
     stain → CORROSION_COATING.
   - Split the real images 80/10/10 by image. If an image source groups
     images by bridge, split by bridge instead.
2. **Add hard negatives** from MISSION-NEG tiles only, as empty-target
   images. Cap them at about 1:1 with positive images so the model doesn't
   learn to predict nothing.
3. **Training order:** train on real + synthetic together. The option of
   pretraining on real data and then fine-tuning on synthetic is allowed
   only if the combined run plateaus. Report which order you used.
4. **Choose the score threshold on MISSION-VAL**, maximising F1. Never tune
   it on test.
5. **CPU budget.** If the full run would take more than about 5 hours, drop
   dacl10k before cutting resolution or epochs. Say what you dropped.

## Step 3 — Headline rule, fixed now, before any results

The headline model is the one with the highest **F1 on MISSION-VAL** among
v1, v2 and v3. v1 and v2 get their thresholds chosen on MISSION-VAL too, so
the comparison is fair. The other models are reported in full next to it.
Test sets are scored once, after the headline is chosen.

## Step 4 — Report

Add a v3 section to `detection/AVIAN_detector_report_FINAL.md`:

- **Real-image test** (held-out CODEBRIM/dacl10k): mAP@0.5 and per-family
  P/R, with counts. This is the first number measured on real photos; state
  it plainly whether it's good or bad.
- **Synthetic test** (the same 24 test defects as before): v1 / v2 / v3.
- **MISSION-TEST:**
  - precision and recall with TP/FP/FN counts
  - the number of waypoints, tiles and defects in scope
  - unseen-defects recall as the headline, all-defects recall labelled
    optimistic
  - v1, v2 and v3 side by side
- **False positives per 100 tiles** for each model. This is the number that
  shows whether the "texture = crack" habit is fixed.
- **FASTENER stays a declared limitation.** Neither dataset has bolts. Note
  that 1920×1080 capture would give about 0.66 mm/px at the same 9° field of
  view, roughly 3× more pixels per bolt. That's the next fix, not attempted
  here.
- **Updated failure modes,** read off real errors.

Re-run all gates:
- the dataset split leakage assertion
- the unseen-instance assertion
- a grep proving no ground truth is read in the detector, the tile
  selection, or the waypoint split code

Commit the work, not the raw tiles or the downloaded datasets (add them to
`.gitignore`). Put the headline numbers in the commit message.

---

## End with this block, and stop

```
HANDOFF — SECTION 3
Leak check (Section 2's 10 defects):   n in train / val / test split
Real-image test (N images, M boxes):   v3 mAP@0.5; per-family P/R
Synthetic test (24 defects):           v1 → v2 → v3 mAP@0.5
MISSION-TEST (W waypoints, T tiles):   headline model; P / R; TP/FP/FN;
                                       unseen-defect recall (n) | all-defect recall (n, optimistic)
False positives per 100 tiles:         v1 / v2 / v3
Threshold (chosen on MISSION-VAL):     value per model
Datasets + licenses:                   …
Zoom factor (corrected):               ≈ 8.7×
Still weak, plainly:                   …
Commit:                                …
```

Don't touch the deck, the dashboard or Gazebo.

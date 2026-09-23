# SECTION 2 — Make the detector work on the mission's own imagery

**Time box: finish by end of 25 Sep 2026.** The SIH deck (Section 3) starts
26 Sep and needs whatever numbers are real by then. If a step isn't done by
the time box, stop, commit what's real, and write the handoff anyway.

Section 1 is accepted as committed in `3f2fce4`. Its honesty was right, and
nothing in it gets deleted or rewritten. This section fixes the two causes
behind its two worst numbers, then measures again.

**Out of scope:** flight and autonomy code (the Phase 1 numbers are frozen),
the dashboard, and Gazebo. **Keep frozen:** `splits_final.json` and
`labels_final.json`.

---

## Why the numbers are bad (two separate causes)

1. **Test set, mAP@0.5 = 0.342, FASTENER at zero.** The model is
   `fasterrcnn_mobilenet_v3_large_320_fpn`. The `_320` variant is
   torchvision's low-resolution model: it resizes every input down to about
   320 px on its short side. A 640×480 frame is roughly halved before the
   network sees it. Bolts and hairline cracks are only a few pixels wide to
   begin with, so after that downscale they barely exist. Check this by
   reading the torchvision source for that constructor's `min_size` and
   `max_size`. Don't assume it.
2. **Mission imagery, 0% precision and 0% recall.** Physics, not the model.
   At 8.0 m with a 69° field of view, one pixel covers about 17 mm of
   concrete. The training images were taken at 0.3–1.5 m, where one pixel
   covers about 0.6–3.2 mm. No model finds a 2 mm crack in 17 mm pixels.
   Flying closer is also wrong: China's MoT UAV bridge-inspection guideline
   sets a 3 m minimum distance from the structure, and Stage 1 showed what
   close flight in this truss does (71% crash rate). Real inspection drones
   solve this with **optical zoom**: they stay at a safe standoff and zoom
   in.
   - Correct one number in the Section 1 report: the image-area gap is
     about **28× to 711×** (8/1.5 squared to 8/0.3 squared), not a flat
     "~700×".

---

## Step 1 — Retrain at full resolution (fixes cause 1)

- Switch to a full-resolution torchvision detector with the same BSD
  license, such as `fasterrcnn_mobilenet_v3_large_fpn` without `_320`. Set
  `min_size`/`max_size` so LOOP frames run at their native 480 px short
  side. Write down the values you used and why.
- **Check that negatives were actually trained on.** Some pipelines quietly
  drop images with no boxes. Count how many of the training set's negative
  frames reached the data loader. If the answer is zero or low, that
  explains the 10/49 false CRACKs on plain concrete; add them back as
  empty-target images, which torchvision supports.
- **Report box sizes per family** (median and 10th-percentile width in
  pixels, at the model's input resolution, before and after the change).
  If FASTENER boxes were under about 16 px at 320 input, that explains its
  zero.
- Training loss plateaued at epoch 2 last time. If CPU training at full
  resolution would take more than about 4 hours, cut epochs, not
  resolution.
- Use the same splits. Re-run the leakage assertion and paste its output.
- Report test-set mAP@0.5 and per-family P/R, **old vs. new side by side**,
  with instance counts.

## Step 2 — Simulated zoom camera, same flight (fixes cause 2)

No re-flight. The aircraft stays exactly where Phase 1's `flight_log.json`
says it was, and only the camera changes.

- **Compute the zoom from data, don't hardcode it.** Take the median
  mm/px of the training frames from the manifest. Choose the horizontal
  FOV that gives the same mm/px at each waypoint's measured standoff.
  Sanity check: this should come out around 8–9× zoom, a horizontal FOV of
  about 8–10°. Real inspection payloads reach about 4° (DJI H20-class), so
  this is physically realistic. Record the zoom factor and resulting
  mm/px in the report.
- **Tile only structure surfaces.** For each chosen waypoint, tile the full
  wide-view footprint with zoom frames, keeping only tiles whose centre ray
  hits a structural primitive. Skip sky, ground and water. Within a chosen
  waypoint, cover all of its structure, not a sample, so every defect in
  that view gets a fair chance.
- **Choose waypoints by geometry, never by ground truth.** A full run is
  roughly 82 waypoints × dozens of tiles × 6 s per render, which is too
  long. So:
  - Pick a subset of waypoints, stratified by structure kind (deck, pier,
    truss, metro), with a fixed random seed.
  - Pick them *before* running any detection.
  - Aim for about 1,000 renders total.
  - Record the seed, the list and the render count.
  - **Never choose waypoints or tiles by where the ground-truth defects
    are.** That would inflate recall. Ground truth is used only for
    scoring, the same rule as Stage 2's planner.
- Run the Step 1 model on every tile. Map tile detections back to the
  waypoint. Score with IoU against projected ground truth, using the same
  matching code as Section 1.

## Step 3 — Report and commit

Update `detection/AVIAN_detector_report_FINAL.md`. Add; don't replace.
The Section 1 results stay in as "before":

- a test-set table: old model vs. new model
- mission imagery: wide 8 m (0%/0%, Section 1) vs. zoom tiles (new). Give
  TP/FP/FN counts, the number of waypoints and tiles, and the number of
  ground-truth defects in scope.
- the zoom factor, the achieved mm/px, and the waypoint-subset seed
- the remaining domain gap: a zoomed view from 8 m has flatter perspective
  than a wide view from 1 m. Say whether the errors suggest it matters.
- the corrected image-area gap (28×–711×)
- updated failure modes if the errors changed

Re-run the gates: the leakage check, and grep proving `ground_truth` is
not read in the detector, tile selection or waypoint selection code. Then
commit, with the headline numbers in the commit message.

---

## End with this block, and stop

```
HANDOFF — SECTION 2
Test set (N frames / M unique defects):   old mAP@0.5 → new mAP@0.5; per-family P/R old → new
Mission imagery, wide 8 m:                0% / 0% (unchanged, Section 1)
Mission imagery, zoom tiles:              P / R, TP/FP/FN, W waypoints, T tiles, G GT defects in scope
Zoom:                                     factor, HFOV, achieved mm/px
Negatives in training:                    before → after
Model / license:                          …
Still weak, plainly:                      …
Commit:                                   …
```

Don't start the deck or the dashboard.

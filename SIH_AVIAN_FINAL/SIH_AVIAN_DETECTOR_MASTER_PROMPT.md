# SIH_AVIAN_FINAL — Phase 2 go-ahead: a real, trained defect detector

**Phase 1 gate passed. Start Phase 2 now.** Read
`SIH_AVIAN_REAL_AUTONOMY_MASTER_PROMPT.md` Phase 2 in full — this file adds
the decisions and traps found since it was written. Where they differ, this
file wins.

## Locked decisions (2026-09-23)

1. **Phase 1 follow-ups stay deferred.** The 201 m worst-case position
   error and the transit-clearance-resolution question
   (`TRANSIT_CLEARANCE_MARGIN_M` / segment sampling in `mission_final.py`)
   are named "future work" for the deck. Do not chase them in this pass.
2. **Phase 1 numbers are frozen as the autonomy result:** 150 waypoints,
   44.7% crash rate (67/150), 56.66% structural coverage, 53.42% defect
   recall (39/73 non-escalated), sensed avoidance in 239/239 log entries.
   Do not re-fly Stage 2 except in step 6 below.

## The dataset you're starting from

`dataset/AVIAN_dataset_stats_FINAL.json`: 798 frames (364 positive, 434
negative), LOOP 640×480 at 2.1478 mm/px @ 1 m, 12 CAPTURE 1920×1080
companions, instance masks in `dataset/masks/`, 192 defects across 19
types, 32 unreachable at 1.5 m survey standoff. The raw renders are
gitignored (commit `063115e`) — they exist on disk; don't delete them.

## Step 1 — Filter, and report the damage before training

- Drop VD01 failures (positive frame whose mask lacks its own instance)
  and VD04 failures (negative frame whose mask isn't empty). Exclusion,
  not repair.
- VD03 (205 frames, standoff drift >5%): keep them for detection-only
  training, but tag them in the split manifest. They must be excluded from
  any defect-size estimate.
- VD06 (9 nulls): fill `view_obliquity_deg` from the pose if derivable,
  else drop the row.
- **Report before training:** frames kept per split, per defect type.
  Expected ceiling is roughly 286 positives / 313 negatives. If any class
  ends up with fewer than ~10 usable positive frames, say so here —
  that decides step 2.

## Step 2 — Pick a label set the data can actually support

192 defects across 19 types is ~10 instances per type. A 19-class
detector trained on that will overfit and report a meaningless mAP.
Do this instead:

- **Primary model:** 5 coarse families mapped from the 19 types —
  `CRACK` (hairline/longitudinal/transverse), `SPALL_DELAM`
  (spall/delamination/section loss), `CORROSION_COATING`
  (corrosion stain/coating failure/efflorescence), `FASTENER`
  (bolt loose/corroded/missing, joint anchor, handrail), `OTHER`
  (bearing, joint, conduit, leakage). Write the mapping into a
  committed `labels_final.json` so it's auditable. Adjust groupings if
  step 1's counts argue for it — but justify each change in writing.
- **Report the 19-type result too,** clearly labelled as
  low-sample and indicative only. Don't hide it; don't headline it.

## Step 3 — Split by defect instance, not by frame (the leakage trap)

The same physical defect appears in several frames (MDR view, SURVEY
view, CAPTURE companion). A random frame-level split puts one view of a
crack in train and another view of the same crack in test — the test
score then measures memorisation, not detection. **Split on
`instance_id`**: every frame of a given defect goes to exactly one split.
Stratify by coarse family. Suggested 70/15/15. Negatives split by source
member so the same girder face doesn't straddle splits either. Write the
split to `dataset/splits_final.json` and commit it.

## Step 4 — Train

- Derive bounding boxes from the instance masks (tight bbox per
  instance id). Don't hand-draw or re-infer them.
- Start from COCO-pretrained weights; small model (e.g. YOLOv8n/s or
  torchvision RetinaNet/Faster R-CNN). Heavy augmentation is justified
  by the dataset size: flips, scale, brightness/contrast, blur, and
  synthetic noise.
- **License check before choosing:** Ultralytics YOLOv8 is AGPL-3.0;
  torchvision detectors are BSD. The research brief says judges ask
  about licenses and handover. Pick one, write the license in the
  README, and say why.
- Check what hardware is actually available (`nvidia-smi`). If CPU-only,
  pick the smallest model and say so in the report.

## Step 5 — Report like the research brief says judges want

In `detection/AVIAN_detector_report_FINAL.md`:
- Test-set size (frames **and** unique instances), mAP@0.5 and
  precision/recall on the coarse model, per-family breakdown.
- Confusion matrix (saved as a PNG).
- **At least two named failure modes** read off real test errors
  (e.g. "hairline cracks missed beyond X m standoff", "formwork lines
  flagged as CRACK"), each with one example image path.
- A **synthetic-to-real caveat** stated plainly: trained and tested on
  Blender renders only. Optional stretch, only if time allows: run the
  model on a few dozen real CODE-BRIM images and report the drop
  honestly — a measured domain gap is a stronger deck line than an
  unmeasured one.

No bare percentage anywhere without its test-set size next to it.

## Step 6 — Wire it in and re-score

- Replace `detect_stub_final.py`'s body with real inference, same
  signature: `detect(image_or_viewpoint_pose) -> list[{class, confidence,
  bbox}]`. Delete the ground-truth-reading branch — don't leave it as a
  fallback. Grep to prove it's gone.
- For each settled Stage 2 waypoint in the existing Phase 1
  `flight_log.json`, render the view at the **achieved** pose
  (`cameras_final.py`), run the detector, write `detections.json`.
  ~150 renders at ~6 s each is fine. No re-flight needed.
- Re-run `score_coverage.py`. Report precision **and** recall, now
  real, next to the Phase 1 stub numbers so the change is visible.
- Match detections to ground truth by IoU on the rendered view, not by
  "was the defect in frame" — a detection has to be on the defect.

## Gate

1. `labels_final.json` and `splits_final.json` committed; no
   `instance_id` appears in more than one split (write a check that
   asserts this and include its output).
2. Detector report exists with test-set size, per-family numbers,
   confusion matrix, two named failure modes, and the synthetic-to-real
   caveat.
3. `grep -n ground_truth source/detect_stub_final.py` returns nothing
   that executes.
4. `score_coverage.py` output shows real precision and recall from the
   trained model, labelled as such.
5. Commit everything (not the raw renders), with the numbers in the
   commit message.

Then stop and report. Don't start the dashboard until these numbers are
reviewed.

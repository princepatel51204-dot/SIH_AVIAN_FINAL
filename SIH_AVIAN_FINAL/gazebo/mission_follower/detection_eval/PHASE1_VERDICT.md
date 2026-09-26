# Phase 1 verdict — does the trained detector find modelled damage in Gazebo?

**Answer: No, not in a way that can be used as evidence.** Gazebo shows the detection pipeline running in the loop; it does not
show detection accuracy. Detector accuracy evidence is the textured Blender imagery (benchmark tiles and the inspection film).

## Rules (identical for every number below)
All 192 ground-truth defects (road 96, metro 20, steel 76), true camera pose from the simulator audit track at the detection
timestamp. A defect is *usable* in a frame when its centre is inside the 640x480 image, 3.5–10 m from the camera, with clear line
of sight. A box is a *true positive* when it contains the projected centre of at least one usable defect; a second count also
requires the detector family to match the defect's family. Threshold 0.65 everywhere, never lowered; no box dropped or drawn.
Scripts: `gazebo/mission_follower/eval_detection_gazebo.py`, `detection_box_targets.py`, `eval_snapshot_recall.py`.

## Baseline (flat-coloured 0.25 m defect spheres)
| Run | Boxes | Contain a usable defect | + class match | Box centres on | Recall (waypoint snapshots) | File |
|---|---|---|---|---|---|---|
| full_pass_05 (1,161 viewpoints) | 87 | **0 (0.0 %)** | 0 | bank vegetation 77, road surface 10 | **0 of 444** sightings (168 unique defects) | `fp05_precision_strict_3p5_10m_los.json`, `fp05_box_targets.json`, `fp05_snapshot_precision_recall.json` |
| columns_full (460 viewpoints) | 233 | 61 (26.2 %) | 52 | vegetation 169, pier 63, none 1 | 2 of 107 (1.9 %) | `columns_full_*.json` |

In columns_full every true positive is on one of two dark marker spheres (REBAR_EXPOSED_006, CRACK_HAIRLINE_009), and precision
*falls* as confidence rises (4 of 83 at >= 0.85, 0 of 56 at >= 0.90): the most confident boxes are on vegetation props. A
confidence floor would make it worse.

## Decal feasibility test (one decal, test world, camera 640x480 HFOV 80 deg at 3.5 / 4.5 / 6.0 m)
Textures were rendered from the Blender twin (all 86 defect materials are procedural; no image textures exist to export).
| Variant | Result | Frames |
|---|---|---|
| A spall (SPALL_007), default lighting | 0 of 3 | `frames/decal_test_A_*.jpg` |
| B spall, lighting fixed (lower ambient, frontal sun, wall matched) | 0 of 3 | `frames/decal_test_B_*.jpg` |
| C exposed rebar (REBAR_EXPOSED_001), fixed lighting | **1 of 3: SPALL_DELAM 0.798 at 3.5 m**, box on the patch; none at 4.5 / 6.0 m | `frames/decal_test_C_*.jpg` |
Diagnostic: the same textures scored as images fire at 0.968 (spall) and 0.960 (rebar). The texture is not the problem; the
**apparent size** is. At 3.5 m the 80-degree camera spans 5.9 m of wall, so a 0.55 m spall is ~59 px wide; the film framed the
same defects in ~1.5 m and the detector's training tiles use an ~8.7x simulated zoom (9 deg field of view).

## Converted: 10 defects on road piers RP03 and RP04 (decals wrapped onto the columns), same orbit flown again
| RP03+RP04 waypoints (61) | Boxes | Contain a defect | + class | Box centres on | Snapshot recall |
|---|---|---|---|---|---|
| Before (spheres; columns_full subset) | 64 | 61 (95.3 %) | 52 | pier (the dark rebar sphere) | 1 of 53 |
| After (decals; decals_rp0304) | 33 | 32 (97.0 %) | 29 | pier 33 of 33 | **0 of 55** |
All 33 boxes are on the REBAR_EXPOSED_006 decal (confidence 0.66–0.88; `frames/decal_flight_rebar006_boxes.jpg`); the
CRACK_HAIRLINE_009 count comes only from it lying inside the same box. Of the 14 defects in usable view, **one** was detected.
The two spall decals (SPALL_011, SPALL_009) were in usable view 4 and 3 times and never fired; the cracks and the corrosion
stain are not visible at this resolution. Precision at these waypoints was already high before (the detector fired on the dark
sphere at the same spot), so the decals did not measurably improve it, and recall did not improve.

**False-positive reduction (step 4) was not attempted:** its precondition (precision improved by the conversion) was not met, and
the demo columns had no vegetation false positives to reduce. The vegetation problem remains elsewhere in the world.

## Recommendation
Present Gazebo for autonomy, sensed-only avoidance, coverage and gimbal aiming, and show the detector running in the loop as a
*pipeline* demonstration (in a demo, the RP04 rebar decal is a real, clearly labelled example of the detector firing on textured
damage in-loop). Present detection accuracy only from the Blender benchmark and film. Do not quote any Gazebo precision or
recall as defect-detection performance. The credible path to Gazebo detection would be a narrow-FOV zoom inspection camera
(matching the ~9 deg field of view the detector was trained on) plus textured defects; it was not built here.
Budget note: this phase took ~70 min against a 45-min budget (the conversion flight alone was ~17 min wall).

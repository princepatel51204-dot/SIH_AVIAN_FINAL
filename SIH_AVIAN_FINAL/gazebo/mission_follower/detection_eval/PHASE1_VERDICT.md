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

## Phase 2 (2026-09-27): continuous column aim + narrow 16 deg inspection camera, flown

Both fixes recommended above were built (commits `3f7317d`, `16424ef`) and then flown for the first time as `aimnarrow_rp0304`,
the same RP03/RP04 plan and settings as `decals_rp0304` (61/61 waypoints, 0 contacts, ekf err mean 0.088 m / p95 0.128 m / max
0.149 m -- position hold and settle are unaffected by continuous yawing, if anything marginally tighter than before's 0.093 /
0.131 / 0.162 m). Scripts: `measure_orbit_aim.py`, `eval_detection_gazebo.py`, `eval_snapshot_recall.py`, `detection_box_targets.py`.

**Orbit aim, scored through the 16 deg lens both times (`aim_before_decals_rp0304_narrow16.json` vs
`aim_after_aimnarrow_rp0304_narrow16.json`):**
| Leg type | Before az-err mean / in-FOV | After az-err mean / in-FOV | Continuous aim applies? |
|---|---|---|---|
| ring | 11.03 deg / 62.1 % | 6.33 deg / 75.3 % | yes |
| link | 46.34 deg / 35.9 % | 3.17 deg / 96.7 % | yes |
| transit | 17.25 deg / 31.8 % | 17.24 deg / 31.9 % | no (different piers) |
| vertical | 0.46 deg / 98.8 % | 0.89 deg / 98.8 % | no (already aimed) |
| dwell | 0.85 deg / 91.3 % | 0.74 deg / 91.7 % | no (already aimed) |
| ring mid-transit only | -- / 43.7 % in-FOV | -- / 62.1 % in-FOV | yes |
| all orbit legs (ring+vertical+dwell) | -- / 66.8 % in-FOV | -- / 78.1 % in-FOV | mixed |

**Target ("the column stays in the 16 deg FOV through the whole ring") was NOT met.** 24.7 % of ring time and 37.9 % of ring
mid-transit time the column is still outside frame. Measured root cause: elevation error is unchanged by this fix, in every leg
type, to two decimal places (ring el-err mean 18.53 deg before vs 18.56 deg after). `continuous_aim` (3f7317d) only re-tracks
**yaw** every tick; gimbal pitch is still set once per leg from the start pose (`start_leg`) plus one "settle" refine near the end
(`mission_follower_node.py:568-579`, `rem < AIM_REFINE_DIST_M`), so pitch drifts through most of a ring leg. The narrow camera's
vertical FOV is only 12.03 deg (half = 6.0 deg), far smaller than the ~18.5 deg mean elevation error, so **pitch, not yaw, is now
the dominant reason the column leaves frame.** Continuous pitch tracking was not attempted here.

**Precision and recall, same rules as above (0.65 threshold, never lowered; no box hand-drawn or dropped; rmin 3.5 m / rmax
10 m / line-of-sight):**
| | Boxes | tp-location | tp-class | FPs | Box centres |
|---|---|---|---|---|---|
| Before (decals_rp0304, wide 80 deg, whole flight) | 33 | 32 (97.0 %) | 29 (87.9 %) | 1 | pier 33/33 |
| After (aimnarrow_rp0304, narrow 16 deg, whole flight) | 22 | 18 (81.8 %) | 18 (81.8 %) | 4 | pier 21, floorbeam 1 |

Before, every true positive was on `REBAR_EXPOSED_006`. After, every true positive is on a **different** defect,
`DEFECT_SPALL_011` (family SPALL_DELAM) -- 17 of the 18 true-positive boxes come from one dwell waypoint (`RP04_R2_10`, where the
detector re-fires every frame while stationary), the rest scattered. Precision fell as detections concentrated on this one new
defect; there are still zero vegetation false positives.

Waypoint-snapshot recall (`eval_snapshot_recall.py`, scored on the **same 55 wide-camera-usable sightings / 14 unique defects**
as the baseline, via `--also-wide-usable`, for a fair comparison):
| | Instances detected | Unique defects detected |
|---|---|---|
| Before (0 boxes at any snapshot) | 0 of 55 (0.0 %) | 0 of 14 (0.0 %) |
| After | 2 of 55 (3.6 %) | 1 of 12 (8.3 %) |

(The usable-unique count moved 14 -> 12 because the actual flown pose track differs slightly leg-to-leg once yaw is continuous;
a small, honestly-reported side effect, not an error, and it does not change the conclusion.) Scored against the narrow camera's
own much smaller usable set -- only 3 sightings / 2 unique defects were ever geometrically inside the 16 deg frame at a settled
waypoint at all -- recall there is 2 of 3 (66.7 %) / 1 of 2 (50 %); that number is **not** comparable to "before" and is reported
only for completeness, not as evidence the detector works.

**Verdict, updated:** the recall floor moved off zero for the first time (0.0 % -> 3.6 % on the same 55-sighting basis) and the
detector caught a defect family (spall) it had never caught before, but this is still not usable detection: precision dropped as
hits concentrated on one dwell point, and the narrow camera's tiny vertical FOV combined with pitch-only-aimed-once-per-leg means
most of the flight still never frames a defect for long enough, at the right elevation, to fire. **Do not claim Gazebo detection
accuracy improved in any general sense** -- one additional defect family was caught, under favourable (dwell, near-static)
conditions, nothing more. The credible next step is the one flagged above and never attempted here: track gimbal **pitch**
continuously the same way yaw now is.

Second-camera cost (real-time factor): before (decals_rp0304) sim 437.0 s / wall 980.8 s = RTF 0.446; after (aimnarrow_rp0304)
sim 440.1 s / wall 1016.4 s = RTF 0.433 -- about 3 % slower relative to real time from bridging, recording and running the
detector on the second (narrow) camera feed at 5 Hz.

Sensed-only navigation re-checked after this phase's follower change: `grep -nE 'covlib|geometry\(|collision|\.sdf|world_boxes'
mission_follower_node.py` prints nothing; the follower still subscribes only to the 4 PX4 estimator topics
(`vehicle_local_position`, `vehicle_status`, `vehicle_attitude`, `vehicle_land_detected`) plus LiDAR `PointCloud2`. Safety
constants unchanged (`YAW_RATE_MAX = 60 deg/s`, `SENSED_CLEARANCE_M = 3.0`, `CRUISE_MPS = 1.9`).

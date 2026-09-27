# AVIAN prototype — final status report

Team TRINETRA · SIH 2026 · PS SIH26201 · Charotar University of Science & Technology.
Written 2026-09-27. Every number below is copied from the file named next to it. Nothing here is pushed to GitHub.

## 1. Subsystem status

| Subsystem | Status | Measured number | Source file |
|---|---|---|---|
| Digital twin (Blender) | DONE | 360 m corridor, 192 ground-truth defects (road 96, metro 20, steel 76), 568 collision primitives | `scene/AVIAN_*_ground_truth_FINAL.json`, `scene/collision/avian_bridge_collision.json` |
| Coverage mission, Gazebo + PX4 (full_pass_05) | DONE | 1,161 of 1,162 waypoints reached (1 had no legal route, not attempted), 0 contact episodes, 9,050 m flown, 81.1 % measured surface coverage, 0.099 m mean EKF hold error | `gazebo/mission_follower/results/full_pass_05/*` (commit 68d71d6) |
| Sensed-only avoidance | DONE | brake test 1.943 m/s stopped in 1.132 m (budget 1.445 m); 3.0 m sensed ring, 1.9 m/s cruise, flat/vertical route pieces; grep check clean (section 6) | `full_pass_05/mission_log.json`, `mission_follower_node.py` |
| Column-orbit inspection flight (columns_full) | DONE | 460 of 460 viewpoints reached (428 ring + 32 look-up dwells) around all 19 columns, 0 contacts, true position error 0.122 m mean / 0.186 m p95, closest sensed return 3.26 m (never inside 3.0 m, avoidance never slowed the drone), 2,846 s sim in 6,717 s wall (RTF 0.42) | `gazebo/mission_follower/columns_flight_summary.json` |
| Column coverage | DONE (planner model) | 80.2 % mean whole-column coverage at the flown poses vs 80.1 % planned; road piers 86–91 %, metro MC04–MC06 58–61 % | same file; `mission/columns_plan_report.json` |
| Gimbal aiming from plan target | DONE | camera axis on a column at 427 of 428 ring viewpoints and on the cap/column at 32 of 32 dwells; aiming fix: target off-axis 62.5 deg -> 1.95 deg mean, 4 -> 8 of 8 targets in frame | `columns_flight_summary.json`, `gazebo/mission_follower/viz/evidence/centering_*.json` |
| Live 3D map in RViz (from the drone's own sensors) | DONE | 0.25 m voxel map, 261,517 voxels on an 8-waypoint flight; map node ~0.46–0.49 of one core | `viz/evidence/map_viz_stats_aim_after.json`, rehearsal load files |
| Detector (Blender benchmark) | PARTIAL | headline v2 at 0.65: MISSION-VAL F1 0.1818; MISSION-TEST unseen-defect recall 1 of 3, all-defect recall 5 of 20, 0.08 false positives / 100 tiles | `mission/coverage_score_mission_v3.json` |
| Detector on real photos | NOT DONE | 0 of 500 real crack photos detected, 0 of 500 clean photos flagged | `detection/AVIAN_real_photo_eval_v2_FINAL.json` |
| Detection in Gazebo | NOT USABLE AS EVIDENCE | full_pass_05: 0 of 87 boxes on a real defect, recall 0 of 444 sightings; textured decals: 1 of 10 converted defects fires; flown with continuous aim + 16 deg camera (aimnarrow_rp0304): recall off zero for the first time (2 of 55 sightings, 3.6 %) but precision fell (33/32/29 -> 22/18/18) and the column is still outside the 16 deg frame 24.7 % of ring time (pitch, not yaw, is now the limit) | `gazebo/mission_follower/detection_eval/PHASE1_VERDICT.md` |
| Inspection film (Blender) | DONE | 82.6 s, 8 beats, 1,337 detector records on the final frames, 1,011 drawn; per-beat median confidence 0.726–0.989 | `scene/film/detection_log.json`, `media/avian_inspection_film.mp4` |
| Dashboard | DONE | builds from committed files only; 8/8 stat tiles verified against their source; no "pending"; both videos play (82.6 s, 47.9 s, 1920x1080) | `dashboard/build_dashboard.py`, `dashboard/index.html` |
| Three-window demo | DONE, slow | works from cold start (3 runs, 0 contacts, detector fires in-loop on the RP04 rebar decal, best 0.89) but the simulation runs at 0.31x real time with all three windows | `gazebo/mission_follower/demo_evidence/` |

## 2. Detection — the honest summary

**Blender (textured imagery) — this is the only detection evidence.**
- Benchmark (zoom tiles, 8.7x simulated zoom, 9.0 deg field of view): headline v2 at threshold 0.65 found **5 of 20** defects on MISSION-TEST and 1 of 3 unseen defects, with 0.08 false positives per 100 tiles; MISSION-VAL F1 0.1818. Twenty test defects is a small sample. (`mission/coverage_score_mission_v3.json`)
- Real photos: **0 of 500** crack photos detected (and 0 of 500 clean photos flagged). The detector does not transfer to real imagery. (`detection/AVIAN_real_photo_eval_v2_FINAL.json`)
- The film's 8 beats show genuine detector output (1,011 boxes drawn, all logged), but the beats were **chosen at defects where the detector fires**, so the film demonstrates the pipeline, not recall. Per-beat median confidence 0.726 (beat 5) to 0.989. Beats 3 and 5 have dropouts (104 and 54 of the 144 post-approach frames boxed). (`scene/film/detection_log.json`)

**Gazebo — pipeline only, not evidence.** Full verdict: `gazebo/mission_follower/detection_eval/PHASE1_VERDICT.md`.
- full_pass_05: **0 of 87** boxes contained a usable ground-truth defect; 77 box centres on bank-vegetation props, 10 on the road surface; recall 0 of 444 defect sightings in the waypoint snapshots.
- columns_full: 61 of 233 boxes on a defect, all on two dark marker spheres; precision falls with confidence (0 of 56 at >= 0.90); recall 2 of 107 sightings (1.9 %).
- Textured decals (rendered from the Blender twin, 10 defects on RP03/RP04): the single-decal test fired on 1 of 9 frames (rebar, 0.798 at 3.5 m); in flight (wide 80 deg camera, fixed yaw at waypoints only) the detector fired only on the exposed-rebar decal (33 boxes, 32 on it; 1 of 14 defects in view detected; snapshot recall 0 of 55). The textures themselves score 0.96–0.998 as images: the Gazebo camera's 80 deg field of view makes defects about four times smaller than the detector's training imagery.
- **Flown fix (2026-09-27, `aimnarrow_rp0304`): continuous column-aim yaw (`3f7317d`) + a 16 deg inspection camera co-located on the gimbal (`16424ef`), same RP03/RP04 plan.** Orbit aim improved (ring column-in-frame 62.1 % → 75.3 %; link 35.9 % → 96.7 %) but the target — the column staying in frame through the whole ring — was **not met**: still out of frame 24.7 % of ring time. Root cause, measured: the fix only tracks yaw continuously; gimbal *pitch* is still set once per leg, so elevation error is unchanged (~18.5 deg mean, before and after, to two decimal places) and now exceeds the narrow camera's 12.03 deg vertical FOV. Detection: 22 boxes (81.8 % precision, down from 33/97.0 %), all true positives moved to a different, previously-undetected defect (a spall, mostly from one dwell waypoint firing repeatedly). Snapshot recall on the same 55-sighting basis moved off zero for the first time: 0 → 2 of 55 (3.6 %). Second camera costs ~3 % real-time factor (0.446 → 0.433). Full numbers and root cause: `gazebo/mission_follower/detection_eval/PHASE1_VERDICT.md` (Phase 2 section).

## 3. Three-window demo

Run from `/home/prince/avian_rev_c/SIH_AVIAN_FINAL` on AC power, one command per terminal, in this order
(start terminals 2 and 3 once terminal 1 prints `phase WAIT -> ARM`, about 35 s):

```bash
gazebo/mission_follower/run_demo.sh --minutes 7 --detect-hz 2 --map-hz 0.5
```
```bash
gazebo/mission_follower/demo_camera.sh
```
```bash
gazebo/mission_follower/demo_rviz.sh
```

What it does: PX4 + Gazebo (GUI camera following the drone), sensed-only follower flying road pier RP04 rings 2–3
(`mission/gazebo_demo_rp04.json`, cut from the columns plan and verified: min airframe clearance 3.57 m, all route pieces flat or vertical,
camera axis on a column at 22 of 22 viewpoints), textured decals on RP03/RP04 (`--no-decals` turns them off), live detector in the camera window,
voxel map / scans / trajectory / drone in RViz. As of the 2026-09-27 fixes, `launch_mission.sh` defaults to continuous column-aim yaw and feeds the
detector from the 16 deg inspection camera (`AVIAN_DETECT_CAM=wide` restores the pre-27-Sep 80 deg feed used in the R1–R3 rehearsals below); the demo
window in `demo_camera.sh` (`/detection/image_annotated`) follows whichever camera is feeding the detector automatically, no change needed there. The follower ping-pongs the plan until the 7-minute window ends, then flies home over the flown
route and lands; Ctrl-C does the same early, a second Ctrl-C lands in place. `run_demo.sh` wraps everything in systemd-inhibit.

Rehearsals (all from a cold start with 0 leftover processes; evidence in `gazebo/mission_follower/demo_evidence/`):

| Run | Settings | RTF (sim / wall) | Package temp mean / p95 / max | Detector | Contacts | What broke → fixed |
|---|---|---|---|---|---|---|
| R1 | unthrottled, 5-min window | 0.291 (180.7 / 621.1 s) | 84 / 89 / 95 °C | 297 % CPU, 0 detections (only 3 waypoints reached before the window closed) | 0 | GUI camera stuck on the world's default view → follow the drone; GUI could not resolve decal textures → resource path; camera window was native Wayland (not capturable) and survived cleanup → forced X11 + SIGKILL fallback; the 125 m transit ate the 5-min window → 7-min window |
| R2 | detector 2 Hz, map 0.5 Hz, 7-min window | **0.308** (239.7 / 777.9 s) | 84 / 88 / 94 °C | 186 % CPU, **19 detections, all on the RP04 rebar decal**, best 0.89 | 0 | all R1 fixes held; RViz showed a red status on the "Live up cone" display in one capture (not investigated) |
| R3 | as R2 but `--no-gui` (measurement only) | 0.369 (167.6 / 453.8 s) | 70 / 79 / 83 °C | 156 % CPU | 0 | — |

Measured cost of each throttle: detector 2 Hz saved ~110 % CPU but raised RTF only 0.291 → 0.308; dropping the Gazebo GUI raised RTF to 0.369 and cut mean package
temperature by 14 °C but removes one of the three windows. The limit is the Gazebo server's sensor rendering on the integrated GPU (the CPU is only ~54 % busy),
so no CPU throttle will make it real-time. **Chosen setting: detector 2 Hz, map 0.5 Hz, GUI on.** Consequence: on screen the drone moves at about a third of real speed;
a full run is ~14 min wall (≈35 s start-up, ≈2.5 min transit to RP04, 7-min orbit window, ≈6 min flight home). Package peaks of 94–95 °C were seen with the GUI on
(below the 100 °C limit; the load sampler's 97 °C abort never triggered).

## 4. What we can and cannot claim to judges

**Can claim (measured, traceable):**
- The drone flies long autonomous inspection missions in Gazebo with PX4 using sensed-only avoidance: 1,161/1,162 waypoints and 460/460 column viewpoints, 0 contact episodes, ~0.1 m position error.
- It inspects bridge columns systematically: stacked orbits around all 19 columns, gimbal aimed at the nearest column axis (427/428 viewpoints on a column), ~80 % planned-model coverage achieved at the flown poses.
- It builds a live 3D voxel map from its own LiDAR and range cones, shown in RViz.
- The trained detector runs in the loop on the drone's camera feed in simulation.
- On textured Blender imagery the detector produces real, logged detections (the film), with the benchmark numbers above.

**Cannot claim — do not say these:**
- **That the drone detects damage in Gazebo.** It does not in any measurable sense: 0 of 87 boxes on a real defect in full_pass_05; the one textured decal that fires is a demonstration, not a result.
- **That the detector is accurate.** Even on its own synthetic benchmark it finds 5 of 20 test defects, and on real photos it found 0 of 500 cracks.
- **That the film shows detection performance.** Its beats were chosen where the detector fires; beats 3 and 5 flicker; beat 8's staged camera is 2.30 m from a deck edge (below the 3.5 m rule it otherwise respects).
- **Real-time or hardware performance.** Everything is simulation; the demo runs at ~0.31x real time; nothing has flown on hardware.
- **Full coverage.** Cap top faces are not reachable (the deck is directly above them), cap end faces are limited by the 3.5 m clearance, metro columns MC04–MC06 reach only 58–61 %, and the steel truss had the lowest coverage in full_pass_05. Coverage figures come from the planner's own visibility model.
- **The 71.2 % Stage 1 figure.** It had no source; the traceable number is 68 of 104 stuck (65.4 %).

## 5. Commits (all local; nothing pushed)
Tonight and this morning, oldest first — `83fed75` Stage 1 doc fix · `1a90605` detection baseline tooling · `0b407c8` decal tooling · `bd314a0` column flight analysis ·
`d50d852` full_pass_05 snapshot recall · `4cec336` decals + AVIAN_DECALS switch + RP03/RP04 cut · `5adce75` the other session's dashboard work, preserved unchanged ·
`c46b42b` + `d4786d5` dashboard: autonomy chart, column section, gallery, detection caveat · `f06ce9c` Phase 1 verdict · `21adb47` + `680c290` dashboard decal result ·
`4e0bb52` RP04 demo cut and rehearsal driver · `601bb5f` rehearsal-1 fixes · `3c75f80` rehearsal evidence · `f1fa8f3` this report's first commit ·
`3f7317d` continuous column aim (not yet flown at the time) + orbit-aim script + narrow-FOV decal test · `16424ef` 16 deg inspection camera + before-aim numbers ·
plus the flown result and this update (aimnarrow_rp0304, Phase 2 verdict, this report).
Earlier in the series: `80c7a0b` nearest-column aiming, `86b12d1` look-up dwells, `a0c561c` live map + RViz + gimbal aiming (two sessions' work).

## 6. Rule checks
- Sensed-only navigation: `grep -nE 'covlib|geometry\(|collision|\.sdf|world_boxes' gazebo/mission_follower/mission_follower_node.py` prints nothing; the follower was not changed in this phase (last change `a0c561c`); safety constants unchanged (`SENSED_CLEARANCE_M = 3.0`, `CRUISE_MPS = 1.9`, `HORIZ_ELEV_MAX = 12 deg`).
- Decals are visual-only (no collision). The LiDAR renders visuals, so with decals on it sees 10 thin decals 12 mm off the column surface instead of 10 flat 0.25 m spheres; default runs are unchanged (`AVIAN_DECALS` defaults to 0 except in `run_demo.sh`).
- full_pass_05 (68d71d6) was only read, never modified. No detection threshold was lowered; no box was drawn by hand or dropped.

## 7. The single most valuable thing left to do
The narrow-field inspection camera (16 deg) and continuous yaw aim were built and flown (Section 2, `aimnarrow_rp0304`): they moved recall off zero for
the first time but did not turn Gazebo detection into evidence. The measured reason is now known: only **yaw** is tracked continuously; gimbal **pitch**
is still set once per leg, so elevation error is unchanged (~18.5 deg mean) and now exceeds the narrow camera's 12 deg vertical FOV — pitch, not yaw, is
the remaining limit on how much of the flight frames a defect at all. Tracking pitch continuously the same way yaw now is, is the next concrete step.
Retraining the detector on real imagery remains the other big gap (0 of 500 real crack photos).


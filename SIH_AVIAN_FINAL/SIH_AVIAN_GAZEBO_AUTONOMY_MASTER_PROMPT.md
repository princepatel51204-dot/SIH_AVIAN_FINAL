# MASTER PROMPT — GarudaNEX autonomy on the SIH bridge (Gazebo) + detection

**Gate: do not paste this into Claude Code until `git tag -l` shows
`sih-idea-submission` (Prompt 1's freeze) AND at least ~3 hours remain
before the 16:00 IST submission target.** If either isn't true when the
freeze lands, this becomes Grand Finale (Dec 2026) work instead, and the
frozen prototype is what gets submitted.

**Supersedes the earlier from-scratch version of this file** (X3 quad +
hand-built exploration node). This version reuses `~/GarudaNEX`, a
separate, already-proven autonomy stack on this machine — verified
directly, not taken on faith: `results/BEST_RUN/recorder_summary.json`
shows 671.15 m flown, 0 contact samples, 74.4% goals reached (58/78),
fully autonomous, and every file this prompt references
(`README.md`, `docs/RUNBOOK.md`, `docs/debugging.md`,
`garudanex_bringup/scripts/start.sh`, `garudanex_navigation/config/nav2_uav.yaml`,
`garudanex_sim/worlds/garudanex_facility_pro.sdf`) exists on disk. The
spawn point below (x≈20, y≈-30) matches this repo's own
`mission/coverage_mission.json` → `base_position_m` exactly. Adapting a
working stack to a new world is a smaller, lower-risk lift than building
flight + avoidance from nothing under this deadline.

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_HANDOVER_FOR_NEW_CHAT.md`
and the other `*_MASTER_PROMPT.md` files this project has used throughout).

---

## Goal
Fully autonomous exploration + obstacle avoidance of the SIH bridge corridor in
Gazebo, using the PROVEN GarudaNEX stack, then defect detection on what the drone saw.
Reuse, don't rebuild.

## Read first (both repos)
1. ~/GarudaNEX  (github.com/princepatel51204-dot/GarudaNEX): README.md, docs/RUNBOOK.md,
   docs/debugging.md (16 failure classes), src/garudanex_bringup/scripts/start.sh,
   src/garudanex_sim (worlds, how a world is added), src/garudanex_navigation/config/nav2_uav.yaml,
   src/garudanex_explore (smart_explorer, run_recorder).
   Proven: PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy, 16-ring 3D LiDAR, slam_toolbox,
   octomap_server, pointcloud_to_laserscan, Nav2 (NavFn + MPPI), frontier explorer.
   671 m flown, 0 collisions. Workspace: ~/GarudaNEX/ros2_ws.
2. /home/prince/avian_rev_c/SIH_AVIAN_FINAL: SIH_AVIAN_HANDOVER_FOR_NEW_CHAT.md,
   gazebo/worlds/sih_avian_final.sdf + gazebo/models, mission/coverage_mission.json,
   detection/AVIAN_detector_report_FINAL.md.

## Environment (every shell)
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
cd ~/GarudaNEX/ros2_ws && source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/prince/avian_rev_c/SIH_AVIAN_FINAL/gazebo/models
Follow RUNBOOK's 3 rules: never launch Nav2 twice; restart sim between runs; be
airborne before the explorer starts.

## Step 1 — Bridge world inside GarudaNEX (gate: drone hovers in bridge world)
- Add the SIH world to garudanex_sim the same way existing worlds are added
  (new world name: sih_bridge). Keep the SIH SDF geometry unchanged; add what
  GarudaNEX worlds need (PX4 spawn, physics, sun, ground plane) — copy from
  garudanex_facility_pro.
- Spawn the GarudaNEX drone at the SIH drone base (x≈20, y≈-30; see
  mission/coverage_mission.json base_position_m).
- start.sh sih_bridge → PX4 takeoff → offboard → hover. GATE 1, commit.

## Step 2 — Adapt exploration to an outdoor bridge
Differences from the indoor facility, handle each explicitly:
- Altitude: explore at an inspection layer below the deck (deck underside ~12–13 m,
  piers, truss). Start with one layer at ~5 m AGL (tune so the 2D scan slice hits
  piers/abutments, not open air); optionally a second layer. Document the chosen
  altitude(s) and why.
- Unbounded outdoors: frontiers run to infinity. Bound the map/exploration with a
  geofence polygon around the corridor (x −10…370, y −40…45 m; check the SDF) —
  frontiers outside it are ignored.
- Safety: keep min approach distance ≥ 3 m to structure for bridge work (China MoT
  UAV bridge guideline); set Nav2 inflation / MPPI obstacle critic accordingly.
  3D LiDAR + octomap for obstacles above/below the 2D slice (deck underside, truss).
- Avoidance must be sensed only: LiDAR/octomap. Do NOT read
  scene/collision/avian_bridge_collision.json or any ground truth in any flight code.
  Grep to prove it.
- Return-to-home + land on time/battery budget.
GATE 2: full autonomous run with no human input after launch. Report: distance
flown, area mapped (m²), goals reached/total, collisions (from Gazebo contacts),
min distance to structure, run time. Save run_recorder results + a 60–90 s screen
recording (gazebo/demo_flight.mp4). Commit.

## Step 3 — Inspection pass (optional, if Gate 2 passes with time left)
After exploration, fly the geometry-only inspection viewpoints
(mission/coverage_mission.json, 8 m standoff) through Nav2 goals on the same map,
same avoidance. Report attempted/reached/failed.

## Step 4 — Detection on what the drone actually saw
- Gazebo's own camera shows flat colours with no defect textures → the detector
  would be meaningless there. So: take the ACHIEVED poses from the Gazebo run
  (odometry at each held viewpoint), render them in Blender with the SIH zoom camera
  (source/render_zoom_tiles_final.py, ~9° HFOV / ~8.7× zoom), output to
  detection/renders_zoom_gazebo/ (gitignored).
- Run headline detector v2 (detection/AVIAN_detector_weights_v2_FINAL.pt, threshold
  0.65 — do NOT re-tune), score with the existing IoU code
  (source/score_zoom_tiles_final.py). Report TP/FP/FN, GT defects in scope, false
  positives per 100 tiles, unseen-defect count.
- Write a gazebo_flight_log.json in the same schema as the SIH PyBullet flight_log
  so source/score_coverage.py can report coverage % and recall n/73 (new output file).

## Rules
- Measured numbers only; unfinished = "not run". Never fabricate logs or metrics.
- No ground truth in flight/avoidance/viewpoint selection. Ground truth only for scoring.
- Never overwrite committed results. Commit each gate with numbers in the message
  (GarudaNEX repo for stack changes; SIH repo for results).
- If a failure matches docs/debugging.md, use its fix and say which class.

## Finish — post and stop
HANDOFF — GARUDANEX ON SIH BRIDGE
Gate 1 (hover in bridge world):   PASS/FAIL
Autonomous run:  distance m; area mapped m²; goals n/N; collisions; min distance to structure m; time
Altitude layer(s) + geofence:      …
Sensed-only grep:                  clean/hits
Inspection pass:                   attempted/reached or "not run"
Coverage (score_coverage):         % ; recall n/73
Detection (v2 @0.65, Blender at Gazebo poses): tiles, GT in scope, TP/FP/FN, FP/100, unseen n
Demo media / results dir:          …
Not done:                          …
Commits:                           …

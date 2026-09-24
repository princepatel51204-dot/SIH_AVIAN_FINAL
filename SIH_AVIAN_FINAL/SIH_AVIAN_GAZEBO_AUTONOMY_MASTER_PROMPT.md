# MASTER PROMPT — AVIAN: autonomous flight in Gazebo + defect detection

**Gate: do not paste this into Claude Code until `git tag -l` shows
`sih-idea-submission` (Prompt 1's freeze) AND at least ~3 hours remain
before the 16:00 IST submission target.** If either isn't true when the
freeze lands, this becomes Grand Finale (Dec 2026) work instead, and the
frozen prototype is what gets submitted.

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_HANDOVER_FOR_NEW_CHAT.md`
and the other `*_MASTER_PROMPT.md` files this project has used throughout).

---

## Context (you are new to this project; read this first)
Project: SIH 2026 prototype "AVIAN" — autonomous bridge-inspection drone.
Team @TRINETRA, PS SIH26201. Repo (git): /home/prince/avian_rev_c/SIH_AVIAN_FINAL
Read before coding (in order): SIH_AVIAN_HANDOVER_FOR_NEW_CHAT.md, README.md,
SIH_AVIAN_AUTONOMOUS_MASTER_PROMPT.md, mission/AVIAN_collision_diagnosis_FINAL.md,
detection/AVIAN_detector_report_FINAL.md.

What already exists:
- Blender digital twin: scene/SIH_AVIAN_FINAL.blend (360 m corridor, road bridge +
  steel truss + metro viaduct, 192 ground-truth defects).
- Collision model: scene/collision/avian_bridge_collision.json (568 primitives).
- Gazebo world (exported, never flown): gazebo/worlds/sih_avian_final.sdf,
  models in gazebo/models.
- Geometry-only coverage mission: mission/coverage_mission.json (150 waypoints,
  8 m standoff). Section 4 variant: mission/coverage_mission_s4.json.
- PyBullet flight results (reference): mission/*flight_log*.json.
- Detector: torchvision Faster R-CNN MobileNetV3. Headline = v2 weights
  detection/AVIAN_detector_weights_v2_FINAL.pt, threshold 0.65 (do NOT re-tune).
- Zoom-tile rendering + scoring: source/render_zoom_tiles_final.py,
  source/score_zoom_tiles_final.py, source/score_coverage.py.
  Blender scripts run as: blender --background --python run_blender.py -- source/<script>.py

## Design (keep it)
Gazebo = the drone flies, explores and avoids obstacles.
Blender = the camera: render views at the poses the drone ACTUALLY reached in
Gazebo, run the detector on those. Do NOT run the detector on Gazebo camera
images (flat colours, no defect textures — meaningless score).

## Environment setup (verified working)
Every shell:
  conda deactivate 2>/dev/null
  source /opt/ros/jazzy/setup.bash
  export GZ_SIM_RESOURCE_PATH=/home/prince/avian_rev_c/SIH_AVIAN_FINAL/gazebo/models
  export AVIAN_ENV_SRC=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source
  export AVIAN_COMMON_DIR=/home/prince/avian_rev_c/avian_common
Verified: `gz sim --version` → Gazebo Sim 8.15.0 (Harmonic), ROS 2 Jazzy.
Conda must stay deactivated (it shadows system python3). Check ros_gz_bridge
is installed (`ros2 pkg list | grep ros_gz`); if missing, tell the user the exact
`sudo apt install ros-jazzy-ros-gz` command — you have no sudo.

## Rules
1. Measured numbers only. Unfinished = "not run". Never fabricate a log.
2. Ground truth / the collision JSON must NOT be read by the flight or
   avoidance code — avoidance uses the drone's lidar only. Grep to prove it.
   Ground truth is used only for scoring.
3. Never overwrite committed results — new filenames for everything.
4. Choose any waypoint/tile subset by geometry with a fixed seed, never by
   where defects are.
5. Commit at each gate with the numbers in the message. Gitignore renders/bags.
6. Don't use PX4 SITL (too slow to set up) — name it as next step.

## Step 1 — Drone flying in the bridge world
- Multirotor using Gazebo's own MulticopterMotorModel + MulticopterVelocityControl
  plugins (e.g. the X3 quadcopter from gz examples), velocity-commanded.
- Sensors: gpu_lidar (360°, a few vertical layers) + odometry.
- ros_gz_bridge: cmd_vel, odometry, lidar scan, clock.
- Launch: gazebo/launch/avian_flight.launch.py (or gazebo/run_flight.sh).
- Spawn at the drone base (see mission file's base_position_m).
GATE 1: takeoff → hover 10 m for 10 s → land, pose logged. Commit.

## Step 2 — Autonomous exploration node
gazebo/avian_explorer_node.py (ROS 2 Python):
- Loads mission/coverage_mission.json; flies waypoint to waypoint
  (P-controller position→velocity), holds 2 s at each.
- Sensed avoidance (lidar only): if nearest return along travel < 3 m, stop,
  move toward the freest direction (sidestep/climb), retry; after 3 failures mark
  waypoint "stuck" and continue. Log every avoidance event.
- Geofence on corridor bounds; return-to-home on a time/battery budget; land.
- Output mission/gazebo_flight_log.json in the SAME schema as the PyBullet
  flight_log (waypoint_id, target, achieved position, settled/stuck, avoidance
  events, sim time) so existing scripts reuse it.
- Record demo: 60–90 s screen capture gazebo/demo_flight.mp4 (or a ros2 bag).
- If all 150 won't finish in reasonable time, fly a stratified subset (fixed
  seed) and state n.
GATE 2: report attempted / settled / stuck / avoidance events / flight time. Commit.

## Step 3 — Detection on Gazebo-flown poses
1. score_coverage.py --mission=coverage_mission.json --flight-log=gazebo_flight_log.json
   → write to a NEW output file; report coverage % and recall n/73.
2. Render zoom tiles in Blender at the Gazebo-achieved poses of settled waypoints
   (reuse render_zoom_tiles_final.py, same ~9° HFOV / ~8.7× zoom, output to
   detection/renders_zoom_gazebo/). Subset of ~10–20 waypoints by structure kind,
   fixed seed, is fine.
3. Run v2 @0.65 on the tiles; score with the existing IoU code. Report TP/FP/FN,
   GT defects in scope, false positives per 100 tiles, and which of the in-scope
   defects were unseen by the detector's training split.
4. Add a "gazebo" block to FINAL_RESULTS.json (new commit on top; don't move any
   existing tag) and rebuild the dashboard: python3 dashboard/build_dashboard.py
GATE 3: commit.

## Finish: post this block and stop
HANDOFF — GAZEBO + DETECTION
Stack:          gz sim 8.15.0, ROS 2 Jazzy, drone model, sensors, bridge topics
Gate 1:         takeoff/hover/land PASS/FAIL
Exploration:    attempted / settled / stuck; avoidance events; flight time; subset n of 150
Sensed-only:    grep result for collision JSON / ground truth in node
Coverage:       % ; recall n/73
Detection (v2 @0.65 on Gazebo poses): waypoints, tiles, GT in scope, TP/FP/FN, FP/100 tiles, unseen n
Demo media:     path
Not done:       …
Commits:        …

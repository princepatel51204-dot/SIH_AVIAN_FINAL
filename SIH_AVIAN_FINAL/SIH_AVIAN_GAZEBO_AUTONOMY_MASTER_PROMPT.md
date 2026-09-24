# MASTER PROMPT — Gazebo: get the drone flying, avoiding, and detecting. Nothing else.

**Status: ready to send now.** Deliberately run BEFORE the freeze (the earlier
"wait for the tag" gate on this file is superseded) — Gazebo has a real chance
of a working gate before the deadline; the freeze itself is comparatively
quick once Section 4's numbers exist. Diagnosed first, not assumed: as of
2026-09-24 11:0x IST, `gazebo/worlds/sih_avian_final.sdf` exists and has only
ever been scene-validated (`gz sim -v 4 ...`, loads + Play) — no drone has
ever been spawned in it, and no Claude Code process was running anything.
`~/GarudaNEX/ros2_ws` is a separate, verified, already-working autonomy stack
on this machine (671.15 m flown, 0 contact samples, 58/78 goals reached —
`results/BEST_RUN/recorder_summary.json`).

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_HANDOVER_FOR_NEW_CHAT.md`
and the other `*_MASTER_PROMPT.md` files this project has used throughout).

---

Ignore all other pending work (Section 4 refly, freeze, dashboard, deck) for now.
Single focus: a drone that exists in the Gazebo bridge environment, flies
autonomously, avoids obstacles using its own sensor, and gets scored by the
detector on what it actually saw. Work fast — report partial progress rather
than going silent.

Environment, every new shell:
  conda deactivate
  source /opt/ros/jazzy/setup.bash
  source ~/GarudaNEX/ros2_ws/install/setup.bash
Confirm `gz sim --version` and `ros2 --help` both work before doing anything else.

## Step 1 — World + drone exist and fly
- gazebo/worlds/sih_avian_final.sdf already exists and has been visually
  validated (loads, Play works) — but no drone has ever been spawned in it.
- Reuse GarudaNEX's proven drone model, LiDAR, and ROS2/PX4 stack — do not
  build a new airframe from scratch.
- Spawn the drone at a sensible start point clear of structure.
- GATE 1: gz sim launches the world, drone spawns, takeoff -> hover 10s -> land,
  pose logged. Report PASS/FAIL immediately.

## Step 2 — Autonomous exploration + real obstacle avoidance
- Reuse GarudaNEX's frontier-based explorer (smart_explorer) + Nav2 (MPPI) +
  octomap, fed by the drone's own LiDAR only.
- HARD RULE: no reading of collision JSON or ground-truth geometry for
  navigation/avoidance. Sensor data only. Prove with:
  grep -n "collision\|ground_truth" <avoidance/nav files>
  and paste the (should-be-empty) result.
- Minimum standoff 3m. Geofence to corridor bounds.
- Fully autonomous explore -> RTH -> land, no manual waypoints/joystick.
- GATE 2: report distance flown, area explored, goals n/N, collisions (real
  Gazebo contact sensor count), min standoff achieved, run time. Capture
  demo_flight.mp4 or a rosbag.

## Step 3 — Detection on what the drone actually saw
- Export achieved poses to a flight log, same schema as the existing PyBullet
  flight_log.json so existing scoring scripts work unchanged.
- Do NOT score on Gazebo's own camera feed (flat-shaded, no defect texture).
  Render Blender zoom tiles at the Gazebo-achieved poses instead (reuse the
  existing zoom-tile render script).
- Run the existing headline detector (v2 @ 0.65, unchanged) on those tiles
  with the existing IoU scoring code.
- GATE 3: report tiles rendered, defects in scope, TP/FP/FN, FP per 100 tiles.

## Rules
- Measured numbers only. Anything that can't finish: report exactly how far
  it got, mark the rest "not run" — never estimate.
- Commit after each gate, numbers in the commit message.
- Do not touch FINAL_RESULTS.json, the freeze tag, dashboard, or deck.

## Stop and post exactly this when done or blocked:
HANDOFF — GAZEBO CORE
Env check:     gz sim / ros2 versions, PASS/FAIL
Gate 1:        PASS/FAIL — takeoff/hover/land
Gate 2:        distance, area, goals n/N, collisions, min standoff, run time
Sensed-only:   grep result (must be empty)
Gate 3:        tiles, GT in scope, TP/FP/FN, FP/100
Blocked on:    …
Commit:        …

#!/usr/bin/env bash
# Gate 2: fully autonomous frontier exploration + real sensor-only obstacle
# avoidance + RTH + land, on the sih_avian_final corridor world.
#
# Reuses, unmodified except where noted: GarudaNEX's gz_bridge, robot
# description, odom/cmd_vel bridges, SLAM Toolbox, smart_explorer, and
# nav2_bringup -- with a SIH-tuned nav2 params file (3 m inflation/standoff,
# bigger local costmap, octomap-fed 3D-aware layer) and a small, additive,
# backward-compatible geofence patch to smart_explorer.py (default
# unbounded, so other GarudaNEX worlds are unaffected).
#
# Usage: ./launch_gate2.sh
set -euo pipefail

WS="${HOME}/GarudaNEX/ros2_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GZDIR="$(cd "${HERE}/.." && pwd)"
RESULTS="${HOME}/GarudaNEX/results/gate2_sih"
LOGDIR="/tmp/gate2_sih_logs"
mkdir -p "${RESULTS}" "${LOGDIR}"

CRUISE_ALT=8.0
SPAWN_X=20.0; SPAWN_Y=-30.0
# Corridor geofence, WORLD frame: a 100 x 48 m box starting at the spawn/
# base-pad area (20, -30) and reaching north across the structure
# centerline -- a tractable sub-span of the full corridor for one
# autonomous run, chosen to CONTAIN the spawn point (see below).
#
# smart_explorer's frontier coordinates are in the SLAM *map* frame, whose
# origin is the drone's spawn pose, not world/Gazebo coordinates -- so the
# world-frame box above is translated by -SPAWN before being passed in.
#
# First run of this script found two things the hard way:
#  1. dry 5/5, 0 goals: the world-frame box didn't overlap the map-frame
#     frontiers at all (forgot the frame translation).
#  2. dry 5/5 again after fixing (1): the box was correctly transformed but
#     still didn't CONTAIN the spawn point (it started ~70 m away) -- a
#     frontier explorer only ever proposes frontiers adjacent to already-
#     explored space, so a geofence must contain the start point or there
#     is nothing for it to expand from inside the fence.
# Widened after the cmd_vel_smoothed wiring fix (see navigation_sih.launch.py)
# made frontier exploration actually work: the first box exhausted its own
# frontiers in ~2 minutes (8 real goals reached, standoff down to 10.6 m,
# genuinely nearing the structure) -- covers spawn through the collision
# manifest's own "research_zone_m" [90, 270].
GF_MIN_X=$(python3 -c "print(0.0-${SPAWN_X})");    GF_MAX_X=$(python3 -c "print(280.0-${SPAWN_X})")
GF_MIN_Y=$(python3 -c "print(-33.0-${SPAWN_Y})");  GF_MAX_Y=$(python3 -c "print(20.0-${SPAWN_Y})")

# Target-zone bias (map frame), same -SPAWN translation as the geofence
# above: the prior run (872.85 m flown, 123 goals, fully self-terminated)
# never left world x<=80 -- it exhausted the wide-open approach corridor
# under pure information-gain scoring before ever reaching the collision
# manifest's research_zone_m [90, 270] (world), which starts just past
# where that run's OLD geofence ended. See smart_explorer.py's
# target_bias_* params: additive nudge in frontier utility scoring only,
# zero effect on CostCritic/costmap avoidance.
TB_MIN_X=$(python3 -c "print(90.0-${SPAWN_X})");   TB_MAX_X=$(python3 -c "print(270.0-${SPAWN_X})")

echo "=== [1/9] world + drone + DDS bridge ==="
"${GZDIR}/launch_sih_sitl.sh"

echo "=== [2/9] robot description (TF) ==="
ros2 launch garudanex_description description.launch.py use_sim_time:=true \
  > "${LOGDIR}/description.log" 2>&1 &

echo "=== [3/9] gz clock + native 2D scan bridge ==="
ros2 launch garudanex_sim gz_bridge.launch.py world:=sih_avian_final \
  model:=x500_lidar_2d_0 lidar_sensor:=garudanex_lidar_3d use_sim_time:=true \
  > "${LOGDIR}/gz_bridge.log" 2>&1 &

echo "=== [4/9] odom + cmd_vel bridges (offboard translation) ==="
sleep 3
# Run the two nodes directly (not via bridge.launch.py) so cruise_altitude
# can be set at construction: cmd_vel_bridge reads its params once in its
# constructor with no live-update callback, so a runtime `ros2 param set`
# after startup has no effect on it.
ros2 run garudanex_bridge odometry_bridge_node --ros-args \
  --params-file "${WS}/src/garudanex_bridge/config/bridge_params.yaml" \
  -r __node:=garudanex_odom_bridge -p use_sim_time:=true \
  > "${LOGDIR}/odom_bridge.log" 2>&1 &
# max_xy_velocity/max_yaw_rate below override bridge_params.yaml's shared
# 1.5 / 0.8 defaults (not edited in place -- that file is shared across
# other GarudaNEX worlds). Raised to cover nav2_sih.yaml's new MPPI limits
# (vx_max 1.95, vy_max 1.05 -> combined diagonal magnitude 2.21 m/s; wz_max
# 1.65 rad/s) so this bridge-level clamp doesn't become a silent second
# bottleneck the way cmd_vel_smoothed once was.
ros2 run garudanex_bridge cmd_vel_bridge_node --ros-args \
  --params-file "${WS}/src/garudanex_bridge/config/bridge_params.yaml" \
  -r __node:=garudanex_cmd_vel_bridge -p use_sim_time:=true \
  -p cruise_altitude:="${CRUISE_ALT}" \
  -p max_xy_velocity:=2.3 -p max_yaw_rate:=1.65 \
  > "${LOGDIR}/cmd_vel_bridge.log" 2>&1 &

echo "=== [5/9] SLAM Toolbox (localization; native 2D scan) ==="
sleep 2
ros2 launch garudanex_navigation slam.launch.py use_sim_time:=true \
  params_file:="${HERE}/slam_toolbox_sih.yaml" \
  > "${LOGDIR}/slam.log" 2>&1 &

echo "=== [6/9] Octomap: SKIPPED for the live mission ==="
echo "  (This machine runs PX4 + Gazebo rendering + SLAM + Nav2's MPPI all at"
echo "   once; adding octomap's point-cloud processing pushed a raw ROS2"
echo "   service RPC (planner_server's lifecycle configure() response) past"
echo "   its timeout, which stalled the whole lifecycle_manager activation"
echo "   chain -- bt_navigator etc. never got configured at all. Real"
echo "   avoidance for this mission runs on the scan-based obstacle_layer,"
echo "   proven reliable across every run. Octomap itself was verified"
echo "   working standalone earlier (gate2_explore/octomap_sih.launch.py,"
echo "   /projected_map publishing) -- see gate2_summary.json.)"

echo "=== [7/9] real Gazebo contact sensor bridge + counter ==="
ros2 run ros_gz_bridge parameter_bridge \
  "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/body_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts" \
  --ros-args -r "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/body_contact_sensor/contact:=/body_contact" \
  -p use_sim_time:=true \
  > "${LOGDIR}/contact_bridge.log" 2>&1 &
sleep 2
python3 "${HERE}/contact_counter_node.py" "${RESULTS}/contact_summary.json" \
  > "${LOGDIR}/contact_counter.log" 2>&1 &

echo "=== [7b/9] front camera bridge (live viewing only -- NOT the detection pipeline) ==="
ros2 run ros_gz_bridge parameter_bridge \
  "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args \
  -r "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/front_camera/image:=/camera/image_raw" \
  -r "/world/sih_avian_final/model/x500_lidar_2d_0/link/base_link/sensor/front_camera/camera_info:=/camera/camera_info" \
  -p use_sim_time:=true \
  > "${LOGDIR}/camera_bridge.log" 2>&1 &
echo "  live view: ros2 run rqt_image_view rqt_image_view /camera/image_raw"

echo "waiting for /map, /scan, /odom, /tf ..."
for i in $(seq 1 60); do
  T="$(ros2 topic list 2>/dev/null)"
  grep -qx '/scan' <<<"$T" && grep -qx '/odom' <<<"$T" && break
  sleep 1
done

echo "=== [8/9] Nav2 (MPPI controller, SIH-tuned params) ==="
ros2 launch "${HERE}/navigation_sih.launch.py" \
  params_file:="${HERE}/nav2_sih.yaml" use_sim_time:=true autostart:=true \
  > "${LOGDIR}/nav2.log" 2>&1 &

for i in $(seq 1 60); do
  ros2 node list 2>/dev/null | grep -q bt_navigator && break
  sleep 1
done
sleep 3

echo "=== arming + OFFBOARD ==="
python3 "${HERE}/gate2_arm.py" | tee "${LOGDIR}/arm.log"
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
  echo "=== ARM/OFFBOARD FAILED -- aborting run, not wasting time on a vehicle that can't be controlled ==="
  exit 1
fi

echo "=== run_recorder (distance/speed/coverage/min-standoff metrics) ==="
ros2 run garudanex_explore run_recorder --ros-args \
  -p results_dir:="${RESULTS}" -p collision_threshold:=1.0 \
  > "${LOGDIR}/recorder.log" 2>&1 &

echo "=== [9/9] smart_explorer (frontier exploration, geofenced, real sensor data only) ==="
ros2 run garudanex_explore smart_explorer --ros-args \
  -p cruise_alt:="${CRUISE_ALT}" \
  -p clearance_m:=3.0 \
  -p sensor_range:=8.0 \
  -p geofence_min_x:="${GF_MIN_X}" -p geofence_max_x:="${GF_MAX_X}" \
  -p geofence_min_y:="${GF_MIN_Y}" -p geofence_max_y:="${GF_MAX_Y}" \
  -p target_bias_x_min:="${TB_MIN_X}" -p target_bias_x_max:="${TB_MAX_X}" \
  -p target_bias_weight:=0.6 -p target_bias_falloff_m:=40.0 \
  -p land_on_finish:=false \
  -p goal_timeout:=120.0 \
  -p bootstrap_secs:=15.0 \
  -p dry_runs_to_finish:=30 \
  -p stuck_timeout:=40.0 \
  -p results_dir:="${RESULTS}" \
  2>&1 | tee "${LOGDIR}/explorer.log"
EXPLORER_EXIT=$?
echo "smart_explorer exited (code ${EXPLORER_EXIT}) -- proceeding to land"

echo "=== landing ==="
python3 "${HERE}/gate2_land.py" | tee "${LOGDIR}/land.log"

pkill -f run_recorder 2>/dev/null || true
pkill -f contact_counter_node 2>/dev/null || true

echo "=== GATE 2 RUN COMPLETE. Results in ${RESULTS}, logs in ${LOGDIR} ==="

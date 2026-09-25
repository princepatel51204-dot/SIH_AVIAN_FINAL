#!/usr/bin/env bash
# Gazebo Task 1: single-pass coverage mission on the sih_avian_final world.
#
# Reuses gate2_explore's PX4/Gazebo/DDS bring-up (launch_sih_sitl.sh), the
# real contact-sensor counter and the front_camera bridge. REPLACES
# smart_explorer + Nav2 + SLAM with mission_follower_node.py (waypoints in
# order from mission/gazebo_mission_plan.json, direct PX4 offboard velocity
# control, sensed-only safety layer). See mission_follower_node.py's
# docstring for why Nav2 is not used.
#
# Usage: ./launch_mission.sh <run_name> [max_waypoints (0=all)] [brake_test true|false]
#   env: conda deactivate; source /opt/ros/jazzy/setup.bash;
#        source ~/GarudaNEX/ros2_ws/install/setup.bash
set -uo pipefail

# Keep the laptop awake for the whole run. full_pass_04 was suspended twice
# mid-flight (~44 min and ~19 min wall-clock gaps). Re-exec this script
# under a logind block inhibitor (works without sudo for the active local
# user); it is released automatically when the script exits.
if [ -z "${AVIAN_INHIBITED:-}" ] && command -v systemd-inhibit >/dev/null; then
  if systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block true 2>/dev/null; then
    export AVIAN_INHIBITED=1
    exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block \
      --who=AVIAN --why="AVIAN mission ${1:-run}" "$0" "$@"
  fi
  echo "WARNING: systemd-inhibit refused -- the machine may suspend mid-run" >&2
fi

RUN="${1:?run name}"
MAXWP="${2:-0}"
BRAKE="${3:-true}"
WS="${HOME}/GarudaNEX/ros2_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GZDIR="$(cd "${HERE}/.." && pwd)"
ROOT="$(cd "${GZDIR}/.." && pwd)"
PLAN="${ROOT}/mission/gazebo_mission_plan.json"
OUT="${HERE}/results/${RUN}"
LOGDIR="${OUT}/logs"
mkdir -p "${OUT}" "${LOGDIR}"
W=sih_avian_final; M=x500_lidar_2d_0
PFX="/world/${W}/model/${M}/link"

cleanup() {
  pkill -f camera_recorder_node.py 2>/dev/null; sleep 3
  pkill -f pose_audit_node.py 2>/dev/null
  pkill -f contact_counter_node.py 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  "${WS}/src/garudanex_bringup/scripts/stop.sh" >/dev/null 2>&1 || true
  pkill -9 -f MicroXRCEAgent 2>/dev/null || true
}
trap cleanup EXIT

echo "=== [1/6] world + drone + DDS bridge ==="
"${GZDIR}/launch_sih_sitl.sh" || exit 1
cp /tmp/sih_px4_sitl.log "${LOGDIR}/px4_boot.log" 2>/dev/null

echo "=== [2/6] sensor bridges: clock, LiDAR cloud, up/down range cones, camera, contact ==="
ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "${PFX}/link/sensor/garudanex_lidar_3d/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "${PFX}/base_link/sensor/up_range/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "${PFX}/base_link/sensor/down_range/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "${PFX}/base_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "${PFX}/base_link/sensor/body_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts" \
  --ros-args \
  -r "${PFX}/link/sensor/garudanex_lidar_3d/scan/points:=/mission/lidar_points" \
  -r "${PFX}/base_link/sensor/up_range/scan/points:=/mission/up_points" \
  -r "${PFX}/base_link/sensor/down_range/scan/points:=/mission/down_points" \
  -r "${PFX}/base_link/sensor/front_camera/image:=/camera/image_raw" \
  -r "${PFX}/base_link/sensor/body_contact_sensor/contact:=/body_contact" \
  > "${LOGDIR}/bridge.log" 2>&1 &
sleep 3

echo "=== [3/6] real Gazebo contact counter ==="
python3 "${GZDIR}/gate2_explore/contact_counter_node.py" "${OUT}/contact_summary.json" \
  > "${LOGDIR}/contact_counter.log" 2>&1 &

echo "=== [4/6] camera recorder (whole flight) ==="
python3 "${HERE}/camera_recorder_node.py" "${OUT}/camera" > "${LOGDIR}/camera.log" 2>&1 &

echo "=== [5/6] pose audit (simulator truth, logging only; never feeds navigation) ==="
python3 "${HERE}/pose_audit_node.py" "${OUT}/pose_audit.json" "${W}" "${M}" \
  > "${LOGDIR}/pose_audit.log" 2>&1 &

echo "waiting for /mission/lidar_points and /fmu/out ..."
for i in $(seq 1 60); do
  T="$(ros2 topic list 2>/dev/null)"
  grep -qx '/mission/lidar_points' <<<"$T" && grep -q '/fmu/out/vehicle_local_position' <<<"$T" && break
  sleep 1
done
gz topic -l > "${LOGDIR}/gz_topics.txt" 2>&1
ros2 topic list > "${LOGDIR}/ros_topics.txt" 2>&1

echo "=== [6/6] mission follower ==="
python3 "${HERE}/mission_follower_node.py" --ros-args \
  -p use_sim_time:=true -p plan:="${PLAN}" -p results_dir:="${OUT}" \
  -p max_waypoints:="${MAXWP}" -p brake_test:="${BRAKE}" \
  2>&1 | tee "${LOGDIR}/follower.log"
echo "follower exited (${PIPESTATUS[0]})"
sleep 3
echo "=== RUN ${RUN} COMPLETE -> ${OUT} ==="

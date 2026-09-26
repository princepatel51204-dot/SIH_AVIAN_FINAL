#!/usr/bin/env bash
# Gazebo Task 1: single-pass coverage mission on the sih_avian_final world.
#
# Reuses gate2_explore's PX4/Gazebo/DDS bring-up (launch_sih_sitl.sh), the
# real contact-sensor counter and the front_camera bridge. REPLACES
# smart_explorer + Nav2 + SLAM with mission_follower_node.py (waypoints in
# order from the mission plan JSON, direct PX4 offboard velocity
# control, sensed-only safety layer). See mission_follower_node.py's
# docstring for why Nav2 is not used.
#
# Usage: ./launch_mission.sh <run_name> [max_waypoints (0=all)] [brake_test true|false] [plan.json]
#   plan.json defaults to mission/gazebo_mission_plan_v5.json (the coverage-driven
#   1162-waypoint plan flown in full_pass_05, which carries a per-waypoint
#   gimbal_pitch_rad). The older 150-waypoint plan (gazebo_mission_plan.json) is
#   still accepted as the 4th argument; it has no gimbal_pitch_rad, so the
#   follower derives the pitch from each waypoint's target_m.
#   env AVIAN_VIZ=1     also start the live 3D map + RViz support (viz/), default 0
#   env AVIAN_RVIZ=1    ...and open RViz on viz/avian_viz.rviz (needs AVIAN_VIZ=1)
#   env AVIAN_AIM_TARGET=false   do not aim the gimbal from target_m (old fixed camera)
#   env AVIAN_DETECT=0  skip the live detector
#   env AVIAN_GZ_GUI=1  also open the Gazebo GUI window on the running server (default 0 = headless)
#   env AVIAN_FOLLOWER_ARGS / AVIAN_DETECT_ARGS / AVIAN_MAP_ARGS   extra "-p name:=value ..." for the
#                       follower, the live detector and the map node (demo looping, throttles)
#   env AVIAN_FOLLOWER_LOG_FILTER   egrep applied to the follower lines shown on screen (full log is kept)
#   Ctrl-C: the follower returns home over the route already flown and lands; a second Ctrl-C lands
#           where it is. The script then cleans up.
#   env: conda deactivate; source /opt/ros/jazzy/setup.bash;
#        source ~/GarudaNEX/ros2_ws/install/setup.bash
set -uo pipefail

# Keep the laptop awake for the whole run. full_pass_04 was suspended twice
# mid-flight (~44 min and ~19 min wall-clock gaps). smoke_v5_gimbal (25 Sep,
# 08:55-11:44) proved a `systemd-inhibit --mode=block sleep:idle:...` lock is
# NOT enough: GNOME's own idle timer (org.gnome.settings-daemon.plugins.power
# sleep-inactive-battery-timeout, 900 s) called logind Suspend() directly and
# the box slept for 3.5 h anyway (DDS multicast never recovered after -- the
# whole run was dead from 11:44 on, still burning CPU, until this was found
# and killed at 16:46). Belt-and-suspenders: also disable GNOME's idle-sleep
# via gsettings for the run and restore it on exit, in addition to the
# logind inhibitor. Re-exec this script under the inhibitor (works without
# sudo for the active local user); it is released automatically when the
# script exits.
if [ -z "${AVIAN_INHIBITED:-}" ] && command -v systemd-inhibit >/dev/null; then
  if systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block true 2>/dev/null; then
    export AVIAN_INHIBITED=1
    exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block \
      --who=AVIAN --why="AVIAN mission ${1:-run}" "$0" "$@"
  fi
  echo "WARNING: systemd-inhibit refused -- the machine may suspend mid-run" >&2
fi
if command -v gsettings >/dev/null; then
  _AVIAN_GS_AC_TYPE="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 2>/dev/null)"
  _AVIAN_GS_BAT_TYPE="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 2>/dev/null)"
  _AVIAN_GS_IDLE="$(gsettings get org.gnome.desktop.session idle-delay 2>/dev/null)"
  if [ -n "${_AVIAN_GS_AC_TYPE}" ]; then
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing' 2>/dev/null
    gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null
    echo "GNOME idle-suspend disabled for this run (was ac=${_AVIAN_GS_AC_TYPE} battery=${_AVIAN_GS_BAT_TYPE} idle-delay=${_AVIAN_GS_IDLE})"
  fi
fi
restore_gnome_power() {
  if [ -n "${_AVIAN_GS_AC_TYPE:-}" ]; then
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type \
      "$(sed "s/^'//;s/'$//" <<<"${_AVIAN_GS_AC_TYPE}")" 2>/dev/null
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type \
      "$(sed "s/^'//;s/'$//" <<<"${_AVIAN_GS_BAT_TYPE}")" 2>/dev/null
    gsettings set org.gnome.desktop.session idle-delay "${_AVIAN_GS_IDLE##* }" 2>/dev/null
  fi
}
# ROS2 discovery: default range is SUBNET (multicast over wlp1s0). Everything
# in this pipeline is one machine talking to itself; force LOCALHOST so a
# Wi-Fi roam/drop (which killed the DDS graph in smoke_v5_gimbal) can no
# longer break it.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN="${1:?run name}"
MAXWP="${2:-0}"
BRAKE="${3:-true}"
WS="${HOME}/GarudaNEX/ros2_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GZDIR="$(cd "${HERE}/.." && pwd)"
ROOT="$(cd "${GZDIR}/.." && pwd)"
PLAN="${4:-${ROOT}/mission/gazebo_mission_plan_v5.json}"
AVIAN_VIZ="${AVIAN_VIZ:-0}"
AVIAN_RVIZ="${AVIAN_RVIZ:-0}"
AIM="${AVIAN_AIM_TARGET:-true}"
OUT="${HERE}/results/${RUN}"
LOGDIR="${OUT}/logs"
mkdir -p "${OUT}" "${LOGDIR}"
W=sih_avian_final; M=x500_lidar_2d_0
PFX="/world/${W}/model/${M}/link"

cleanup() {
  pkill -f camera_recorder_node.py 2>/dev/null; sleep 3
  pkill -f pose_audit_node.py 2>/dev/null
  pkill -f contact_counter_node.py 2>/dev/null
  pkill -f live_detector_node.py 2>/dev/null; sleep 2
  pkill -f map_viz_node.py 2>/dev/null; sleep 2     # writes its map + stats on SIGTERM
  pkill -f 'robot_state_publisher' 2>/dev/null
  pkill -f 'rviz2 -d' 2>/dev/null
  pkill -f rqt_image_view 2>/dev/null
  pkill -f 'gz sim -g' 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  "${WS}/src/garudanex_bringup/scripts/stop.sh" >/dev/null 2>&1 || true
  pkill -9 -f MicroXRCEAgent 2>/dev/null || true
  restore_gnome_power
}
trap cleanup EXIT

echo "=== [1/7] world + drone + DDS bridge ==="
"${GZDIR}/launch_sih_sitl.sh" || exit 1
cp /tmp/sih_px4_sitl.log "${LOGDIR}/px4_boot.log" 2>/dev/null
if [ "${AVIAN_GZ_GUI:-0}" = "1" ]; then
  echo "=== [1b] Gazebo GUI window (client of the running server) ==="
  GZ_SIM_RESOURCE_PATH="${GZDIR}/models:${WS}/src/garudanex_sim/models:${GZ_SIM_RESOURCE_PATH:-}" \
    gz sim -g > "${LOGDIR}/gz_gui.log" 2>&1 &
fi

echo "=== [2/7] sensor bridges: clock, LiDAR cloud, up/down range cones, gimbal camera + pitch command, contact ==="
# YAML bridge config, not positional args + "-r" remaps: the gimbal command
# topic's GZ name ends in ".../gimbal_pitch_joint/0/cmd_pos" -- the bare "0"
# path segment is illegal ROS2 topic syntax (a name token cannot start with a
# digit), so rcl's "-r" remap-rule parser throws ("Expecting token or
# wildcard") and kills the WHOLE parameter_bridge process before any topic
# bridges, including LiDAR -- which is why the follower silently never left
# WAIT (root cause of the smoke_v5_gimbal hang, found 25 Sep). The YAML
# config's gz_topic_name field is not passed through rcl's arg parser at all,
# so it accepts any Gazebo topic name.
BRIDGE_YAML="${OUT}/bridge_config.yaml"
cat > "${BRIDGE_YAML}" <<EOF
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "/mission/lidar_points"
  gz_topic_name: "${PFX}/link/sensor/garudanex_lidar_3d/scan/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
- ros_topic_name: "/mission/up_points"
  gz_topic_name: "${PFX}/base_link/sensor/up_range/scan/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
- ros_topic_name: "/mission/down_points"
  gz_topic_name: "${PFX}/base_link/sensor/down_range/scan/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "${PFX}/gimbal_cam_link/sensor/front_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
- ros_topic_name: "/mission/gimbal_cmd"
  gz_topic_name: "/model/${M}/joint/gimbal_pitch_joint/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
- ros_topic_name: "/body_contact"
  gz_topic_name: "${PFX}/base_link/sensor/body_contact_sensor/contact"
  ros_type_name: "ros_gz_interfaces/msg/Contacts"
  gz_type_name: "gz.msgs.Contacts"
  direction: GZ_TO_ROS
EOF
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${BRIDGE_YAML}" \
  > "${LOGDIR}/bridge.log" 2>&1 &
sleep 3
if ! kill -0 $! 2>/dev/null; then
  echo "FATAL: parameter_bridge died immediately, see ${LOGDIR}/bridge.log" >&2
  tail -n 30 "${LOGDIR}/bridge.log" >&2
  exit 1
fi

echo "=== [3/7] real Gazebo contact counter ==="
python3 "${GZDIR}/gate2_explore/contact_counter_node.py" "${OUT}/contact_summary.json" \
  > "${LOGDIR}/contact_counter.log" 2>&1 &

echo "=== [4/7] camera recorder (whole flight) ==="
python3 "${HERE}/camera_recorder_node.py" "${OUT}/camera" > "${LOGDIR}/camera.log" 2>&1 &

echo "=== [5/7] pose audit (simulator truth, logging only; never feeds navigation) ==="
python3 "${HERE}/pose_audit_node.py" "${OUT}/pose_audit.json" "${W}" "${M}" \
  > "${LOGDIR}/pose_audit.log" 2>&1 &

AVIAN_DETECT="${AVIAN_DETECT:-1}"
AVIAN_VENV_PY="${HOME}/avian_rev_c/.venv/bin/python3"
if [ "${AVIAN_DETECT}" = "1" ] && [ -x "${AVIAN_VENV_PY}" ]; then
  echo "=== [6/7] live crack detector (real trained weights, per-frame inference) ==="
  "${AVIAN_VENV_PY}" "${HERE}/live_detector_node.py" --ros-args \
    -p use_sim_time:=true -p out_dir:="${OUT}/detection" -p plan:="${PLAN}" ${AVIAN_DETECT_ARGS:-} \
    > "${LOGDIR}/live_detector.log" 2>&1 &
else
  echo "=== [6/7] live crack detector SKIPPED (AVIAN_DETECT=${AVIAN_DETECT}, venv found: $([ -x "${AVIAN_VENV_PY}" ] && echo yes || echo no)) ==="
fi

if [ "${AVIAN_VIZ}" = "1" ]; then
  echo "=== [6b] live 3D map + RViz support (visualisation only; feeds nothing the follower reads) ==="
  "${HERE}/viz/launch_viz.sh" "${OUT}" "${PLAN}" "${AVIAN_RVIZ}"
fi
cp "${PLAN}" "${OUT}/plan_used_$(basename "${PLAN}")" 2>/dev/null

echo "waiting for /mission/lidar_points and /fmu/out ..."
for i in $(seq 1 60); do
  T="$(ros2 topic list 2>/dev/null)"
  grep -qx '/mission/lidar_points' <<<"$T" && grep -q '/fmu/out/vehicle_local_position' <<<"$T" && break
  sleep 1
done
gz topic -l > "${LOGDIR}/gz_topics.txt" 2>&1
ros2 topic list > "${LOGDIR}/ros_topics.txt" 2>&1

echo "=== [7/7] mission follower ==="
# tee and grep ignore SIGINT so a Ctrl-C reaches only the follower, which handles it (return + land);
# if they died with it the follower would lose its log pipe mid-landing
python3 "${HERE}/mission_follower_node.py" --ros-args \
  -p use_sim_time:=true -p plan:="${PLAN}" -p results_dir:="${OUT}" \
  -p max_waypoints:="${MAXWP}" -p brake_test:="${BRAKE}" -p aim_from_target:="${AIM}" ${AVIAN_FOLLOWER_ARGS:-} \
  2>&1 | (trap '' INT; exec tee "${LOGDIR}/follower.log") \
       | (trap '' INT; exec grep --line-buffered -E "${AVIAN_FOLLOWER_LOG_FILTER:-.}")
echo "follower exited (${PIPESTATUS[0]})"
sleep 3
echo "=== RUN ${RUN} COMPLETE -> ${OUT} ==="

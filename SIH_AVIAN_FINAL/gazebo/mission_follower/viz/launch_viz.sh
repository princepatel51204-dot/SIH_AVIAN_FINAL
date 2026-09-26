#!/usr/bin/env bash
# RViz support for a mission run: joint-state bridge, robot_state_publisher, the
# 3D map node and (optionally) RViz. VISUALISATION ONLY -- nothing started here
# publishes anything the mission follower reads. Started by launch_mission.sh
# when AVIAN_VIZ=1; safe to run on its own against a live sim:
#   viz/launch_viz.sh <out_dir> <plan.json> [rviz 0|1]
set -uo pipefail
OUT="${1:?out dir}"; PLAN="${2:?plan json}"; RVIZ="${3:-0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="${OUT}/logs"; VIZ="${OUT}/viz"
mkdir -p "${LOGDIR}" "${VIZ}"
W=sih_avian_final; M=x500_lidar_2d_0
PX4_MESHES="${PX4_MESHES:-${HOME}/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes}"
[ -d "${PX4_MESHES}" ] || echo "WARNING: ${PX4_MESHES} not found; the drone model in RViz will have no meshes" >&2

sed "s#@PX4_MESHES@#${PX4_MESHES}#g" "${HERE}/x500_viz.urdf.in" > "${VIZ}/x500_viz.urdf"

# true gimbal angle: its own small bridge, so the mission's bridge config is untouched
cat > "${VIZ}/viz_bridge.yaml" <<YAML
- ros_topic_name: "/viz/gz_joint_states"
  gz_topic_name: "/world/${W}/model/${M}/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
YAML
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${VIZ}/viz_bridge.yaml" \
  > "${LOGDIR}/viz_bridge.log" 2>&1 &

# The URDF goes in through a params file: as a "-p robot_description:=<xml>" argument the
# multi-line XML aborts rcl's argument parser and robot_state_publisher never starts.
python3 - "${VIZ}/x500_viz.urdf" "${VIZ}/rsp_params.yaml" <<'PY'
import sys
urdf = open(sys.argv[1]).read().splitlines()
with open(sys.argv[2], 'w') as f:
    f.write('/**:\n  ros__parameters:\n    use_sim_time: true\n    robot_description: |\n')
    f.writelines('      ' + ln + '\n' for ln in urdf)
PY
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "${VIZ}/rsp_params.yaml" \
  > "${LOGDIR}/viz_rsp.log" 2>&1 &

python3 "${HERE}/map_viz_node.py" --ros-args -p use_sim_time:=true -p plan:="${PLAN}" -p out_dir:="${VIZ}" ${AVIAN_MAP_ARGS:-} \
  > "${LOGDIR}/viz_map.log" 2>&1 &

if [ "${RVIZ}" = "1" ]; then
  ros2 run rviz2 rviz2 -d "${HERE}/avian_viz.rviz" --ros-args -p use_sim_time:=true \
    > "${LOGDIR}/viz_rviz.log" 2>&1 &
  # Camera feeds live in their own windows: on this machine RViz2's Image display
  # aborts RViz (glibc mutex assertion, reproduced with one and with two Image
  # displays), while the same config without them runs cleanly.
  ros2 run rqt_image_view rqt_image_view /camera/image_raw > "${LOGDIR}/viz_cam.log" 2>&1 &
  if [ "${AVIAN_DETECT:-1}" = "1" ]; then
    ros2 run rqt_image_view rqt_image_view /detection/image_annotated > "${LOGDIR}/viz_cam_det.log" 2>&1 &
  fi
fi
echo "viz started: map node, robot_state_publisher, joint-state bridge$([ "${RVIZ}" = "1" ] && echo ', RViz + camera windows')"

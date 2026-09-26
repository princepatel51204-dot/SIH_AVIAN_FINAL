#!/usr/bin/env bash
# AVIAN demo -- TERMINAL 3: RViz on the saved config (3D voxel map, live scans, drone, trajectory,
# camera image, localized defects).
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "${HOME}/GarudaNEX/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec ros2 run rviz2 rviz2 -d "${HERE}/viz/avian_viz.rviz" --ros-args -p use_sim_time:=true

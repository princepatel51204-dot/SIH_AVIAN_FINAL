#!/usr/bin/env bash
# AVIAN demo -- TERMINAL 2: live camera window with the real detector's boxes drawn on it.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "${HOME}/GarudaNEX/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec ros2 run rqt_image_view rqt_image_view /detection/image_annotated

#!/usr/bin/env bash
# AVIAN demo -- TERMINAL 2: live camera window, natural wide view, with the real detector's boxes
# drawn on it.
#
# The detector runs on the 16 deg inspect camera (launch_mission.sh default) because that is what
# makes it fire (see detection_eval/PHASE1_VERDICT.md); on its own that view is an unrecognisable
# zoomed patch of concrete. This script instead starts wide_box_reproject_node.py, which redraws
# the SAME real detector boxes -- geometrically reprojected, never re-run or re-scored -- onto the
# natural 80 deg camera feed (proof the reprojection is exact:
# detection_eval/verify_wide_reprojection.py, 0.000000 px vs the codebase's own ray-projection
# method, and a real frame where the reprojected box lands on the ground-truth defect:
# detection_eval/frames_aim_after/wide_reprojection_check.png). set AVIAN_DETECT_CAM=wide here to
# match a flight run the same way (then no reprojection is applied -- the boxes are already wide).
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "${HOME}/GarudaNEX/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# X11 (Xwayland) window, like RViz and the Gazebo GUI, so it can be captured and managed the same way
export QT_QPA_PLATFORM=xcb
python3 "${HERE}/viz/wide_box_reproject_node.py" --ros-args -p detect_cam:="${AVIAN_DETECT_CAM:-narrow}" \
  > /tmp/sih_wide_box_reproject.log 2>&1 &
REPROJ_PID=$!
trap 'kill "${REPROJ_PID}" 2>/dev/null' EXIT
exec ros2 run rqt_image_view rqt_image_view /detection/image_annotated_wide

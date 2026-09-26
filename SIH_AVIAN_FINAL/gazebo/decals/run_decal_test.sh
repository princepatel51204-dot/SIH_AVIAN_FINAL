#!/usr/bin/env bash
# One decal feasibility run: headless gz with the test world, bridge the three cameras, save frames.
#   run_decal_test.sh <world.sdf> <outdir>
# Isolated: own GZ_PARTITION and ROS_DOMAIN_ID, so it never touches a mission graph.
set +u
WORLD="$1"; OUT="$2"; mkdir -p "$OUT"
export GZ_PARTITION=decaltest ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source "$HOME/GarudaNEX/ros2_ws/install/setup.bash" 2>/dev/null
HERE="$(cd "$(dirname "$0")" && pwd)"
gz sim -s -r --headless-rendering "$WORLD" > "$OUT/gz.log" 2>&1 &
GZ=$!
Y="$OUT/bridge.yaml"; : > "$Y"
for d in 350 450 600; do cat >> "$Y" <<YAML
- ros_topic_name: "/decal_cam_$d"
  gz_topic_name: "/decal_cam_$d"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
YAML
done
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$Y" > "$OUT/bridge.log" 2>&1 &
BR=$!
sleep 6
timeout 90 python3 "$HERE/capture_frames.py" "$OUT" /decal_cam_350 /decal_cam_450 /decal_cam_600
kill $BR $GZ 2>/dev/null; sleep 2; kill -9 $BR $GZ 2>/dev/null
wait 2>/dev/null
ls "$OUT"/*.png 2>/dev/null

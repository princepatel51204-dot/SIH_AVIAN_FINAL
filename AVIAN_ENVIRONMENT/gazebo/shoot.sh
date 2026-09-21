#!/usr/bin/env bash
# Render the four documented views of the world to PNG, headless.
#
#   ./shoot.sh            render all four views
#   ./shoot.sh 0          render just view 0
#
# HOW, AND WHY NOT THE OBVIOUS WAYS
# ---------------------------------
# A GUI screenshot is not usable here: this is a Wayland session, gz runs
# under Xwayland, and neither `import -window root` nor the /gui/screenshot
# service can capture it from a non-interactive shell.
#
# The camera sensor's own <save enabled="true"> element is also a dead end:
# gz-sim 8 advertises the image topic and renders through ogre2 correctly,
# but writes no files.
#
# What does work is the path Stage 5 needs anyway -- bridge the Gazebo
# image topic into ROS 2 with ros_gz_image and save a frame. Reproducible,
# needs no display, and exercises the real sensor pipeline.
#
# One arithmetic trap worth keeping: iterations x max_step_size must exceed
# the camera's first frame time. 40 iterations x 0.002 s = 0.08 s against a
# 10 Hz camera whose first frame lands at 0.1 s produced nothing at all,
# silently. The values below give ~2 s of sim time.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${HERE}/screenshots"
TMP="${HERE}/.shoot"
ITERATIONS=1000        # x 0.002 s = 2.0 s of sim time
CAM_HZ=5

if [ -f /opt/ros/jazzy/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
fi
export GZ_SIM_RESOURCE_PATH="${HERE}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export SDF_PATH="${HERE}/models${SDF_PATH:+:${SDF_PATH}}"

mkdir -p "${OUT}" "${TMP}"

# name | pose (x y z roll pitch yaw) in GAZEBO coordinates.
# Gazebo x = Blender x - 2100; see avian_gazebo_manifest.json.
VIEWS=(
  "01_corridor_from_above|0 -430 430 0 0.80 1.5708"
  "02_river_crossing|0 -300 26 0 0.06 1.5708"
  "03_under_the_deck|-30 0 14 0 0.10 0.0"
  "04_inter_structure|-70 30 30 0 0.10 0.55"
)

GZ_TOPIC="/world/shoot/model/shot_cam/link/link/sensor/cam/image"

shoot_one () {
    local name="$1" pose="$2"
    local world="${TMP}/shoot_${name}.sdf"
    local shots="${TMP}/${name}"
    rm -rf "${shots}"; mkdir -p "${shots}"

    cat > "${world}" <<SDF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="shoot">
    <physics name="default" type="dart">
      <max_step_size>0.002</max_step_size>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 300 0 0 0</pose>
      <diffuse>1.0 0.97 0.92 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.4 0.35 -0.85</direction>
    </light>
    <scene>
      <ambient>0.32 0.33 0.35 1</ambient>
      <background>0.70 0.76 0.84 1</background>
      <grid>false</grid>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane>
          <normal>0 0 1</normal><size>4000 4000</size>
        </plane></geometry></collision>
        <visual name="v"><geometry><plane>
          <normal>0 0 1</normal><size>4000 4000</size>
        </plane></geometry>
        <material><ambient>0.25 0.24 0.20 1</ambient>
        <diffuse>0.45 0.43 0.36 1</diffuse></material></visual>
      </link>
    </model>
    <include><uri>model://avian_bridge</uri></include>
    <include><uri>model://avian_metro</uri></include>
    <include><uri>model://avian_terrain</uri></include>
    <include><uri>model://avian_defects</uri></include>
    <model name="shot_cam">
      <static>true</static>
      <pose>${pose}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>${CAM_HZ}</update_rate>
          <camera>
            <horizontal_fov>1.15</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.5</near><far>6000</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
SDF

    echo "  rendering ${name} ..."
    ros2 run ros_gz_image image_bridge "${GZ_TOPIC}" \
        > "${TMP}/${name}_bridge.log" 2>&1 &
    local bridge=$!
    sleep 2

    ( cd "${shots}" && ros2 run image_view image_saver \
        --ros-args -r image:="${GZ_TOPIC}" \
        -p filename_format:="frame%04d.png" \
        > "${TMP}/${name}_saver.log" 2>&1 ) &
    local saver=$!
    sleep 2

    gz sim -s -r --iterations "${ITERATIONS}" "${world}" \
        > "${TMP}/${name}.log" 2>&1 || true
    sleep 3

    kill "${saver}" "${bridge}" 2>/dev/null || true
    wait "${saver}" "${bridge}" 2>/dev/null || true

    local last
    last="$(find "${shots}" -name '*.png' | sort | tail -1 || true)"
    if [ -n "${last}" ]; then
        cp "${last}" "${OUT}/avian_sic_${name}.png"
        echo "  -> screenshots/avian_sic_${name}.png"
    else
        echo "  !! no image for ${name}; see ${TMP}/${name}_saver.log" >&2
        return 1
    fi
}

if [ $# -gt 0 ]; then
    IFS='|' read -r n p <<< "${VIEWS[$1]}"
    shoot_one "$n" "$p"
else
    for v in "${VIEWS[@]}"; do
        IFS='|' read -r n p <<< "${v}"
        shoot_one "$n" "$p" || true
    done
fi
echo "done -> ${OUT}"

#!/usr/bin/env bash
# Render one screenshot of the SIH_AVIAN_FINAL world, headless.
#
# Same mechanism as AVIAN_ENVIRONMENT/gazebo/shoot.sh, adapted: this world
# has NO origin shift (ORIGIN=(0,0,0) -- the corridor is already only
# 360 m and close to the origin), so Gazebo coordinates equal Blender
# coordinates directly. A GUI screenshot is not usable in this session (see
# shoot.sh's own header for why); bridging the camera sensor's image topic
# into ROS 2 and saving a frame is the path that actually works here.
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

name="01_overview"
pose="180 -250 200 0 0.75 1.5708"
GZ_TOPIC="/world/shoot/model/shot_cam/link/link/sensor/cam/image"

world="${TMP}/shoot_${name}.sdf"
shots="${TMP}/${name}"
rm -rf "${shots}"; mkdir -p "${shots}"

cat > "${world}" <<SDF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="shoot">
    <physics name="default" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>180 0 200 0 0 0</pose>
      <diffuse>0.9 0.88 0.85 1</diffuse>
      <direction>-0.3 0.9 -0.35</direction>
    </light>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.55 0.56 0.58 1</background>
      <grid>false</grid>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane>
          <normal>0 0 1</normal><size>600 300</size>
        </plane></geometry></collision>
        <visual name="v"><geometry><plane>
          <normal>0 0 1</normal><size>600 300</size>
        </plane></geometry>
        <material><ambient>0.25 0.24 0.20 1</ambient>
        <diffuse>0.45 0.43 0.36 1</diffuse></material></visual>
      </link>
      <pose>180 15 -1.3 0 0 0</pose>
    </model>
    <include><uri>model://avian_final_road</uri></include>
    <include><uri>model://avian_final_metro</uri></include>
    <include><uri>model://avian_final_terrain</uri></include>
    <include><uri>model://avian_final_defects</uri></include>
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
            <clip><near>0.5</near><far>2000</far></clip>
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
bridge=$!
sleep 2

( cd "${shots}" && ros2 run image_view image_saver \
    --ros-args -r image:="${GZ_TOPIC}" \
    -p filename_format:="frame%04d.png" \
    > "${TMP}/${name}_saver.log" 2>&1 ) &
saver=$!
sleep 2

gz sim -s -r --iterations "${ITERATIONS}" "${world}" \
    > "${TMP}/${name}.log" 2>&1 || true
sleep 3

kill "${saver}" "${bridge}" 2>/dev/null || true
wait "${saver}" "${bridge}" 2>/dev/null || true

last="$(find "${shots}" -name '*.png' | sort | tail -1 || true)"
if [ -n "${last}" ]; then
    cp "${last}" "${OUT}/sih_avian_final_${name}.png"
    echo "  -> screenshots/sih_avian_final_${name}.png"
else
    echo "  !! no image for ${name}; see ${TMP}/${name}_saver.log" >&2
    exit 1
fi

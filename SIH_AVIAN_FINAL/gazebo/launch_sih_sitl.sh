#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo on the SIH_AVIAN_FINAL corridor world with the
# GarudaNEX x500_lidar_2d airframe, then bring up the uXRCE-DDS bridge and
# apply the one runtime param override this world needs.
#
#   ./launch_sih_sitl.sh [spawn_pose]     default: 20,-30,1.626,0,0,0
#
# Requires (see SIH_AVIAN_GAZEBO_AUTONOMY_MASTER_PROMPT.md):
#   conda deactivate
#   source /opt/ros/jazzy/setup.bash
#   source ~/GarudaNEX/ros2_ws/install/setup.bash
#
# Why NAV_DLL_ACT=0: this is a fully autonomous mission with no RC and no
# GCS/MAVLink ground station attached, so PX4's "no data-link" preflight
# check (which exists to stop an RC-less, GCS-less vehicle from arming)
# must be disabled -- there will never be a GCS here by design. Gate 1
# diagnosis: gazebo/gate1_flight/gate1_summary.json.
set -euo pipefail

WS="${HOME}/GarudaNEX/ros2_ws"
PX4_DIR="${HOME}/PX4-Autopilot"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPAWN="${1:-20,-30,1.626,0,0,0}"

echo "--- stopping any stale GarudaNEX/PX4/Gazebo processes ---"
"${WS}/src/garudanex_bringup/scripts/stop.sh" || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
sleep 1

echo "--- launching PX4 SITL + Gazebo (world: sih_avian_final) ---"
cd "${PX4_DIR}"
export GZ_SIM_RESOURCE_PATH="${HERE}/models:${WS}/src/garudanex_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=sih_avian_final PX4_GZ_MODEL_POSE="${SPAWN}" HEADLESS=1 \
  GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}" \
  make px4_sitl gz_x500_lidar_2d > /tmp/sih_px4_sitl.log 2>&1 \
  < <(tail -f /dev/null --pid="${PPID}") &
# stdin: PX4's interactive shell (pxh>) re-prints its prompt in a tight loop
# when stdin is at EOF, which grew this log to 16.7 GB during full_pass_04
# (and costs CPU). A pipe that stays open but silent keeps the shell idle;
# it closes by itself when the calling mission script exits.
PX4_PID=$!
echo "PX4/Gazebo launcher PID ${PX4_PID}, log: /tmp/sih_px4_sitl.log"

echo "--- waiting for Gazebo world + model spawn ---"
for i in $(seq 1 60); do
  grep -q "Startup script returned successfully" /tmp/sih_px4_sitl.log 2>/dev/null && break
  sleep 1
done

echo "--- starting MicroXRCEAgent (uXRCE-DDS bridge) ---"
MicroXRCEAgent udp4 -p 8888 > /tmp/sih_uxrce_agent.log 2>&1 &
echo "MicroXRCEAgent PID $!"
for i in $(seq 1 30); do
  [ "$(ros2 topic list 2>/dev/null | grep -c fmu)" -gt 0 ] && break
  sleep 1
done

echo "--- applying NAV_DLL_ACT=0 (no-GCS arming override) ---"
python3 - <<'PYEOF'
from pymavlink import mavutil
import time, sys
m = mavutil.mavlink_connection("udpout:127.0.0.1:14580")
t0 = time.time()
vehicle_sys = None
while time.time() - t0 < 20:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        vehicle_sys, vehicle_comp = msg.get_srcSystem(), msg.get_srcComponent()
        break
if vehicle_sys is None:
    print("FAIL: no vehicle heartbeat on MAVLink"); sys.exit(1)
m.target_system, m.target_component = vehicle_sys, vehicle_comp
m.mav.param_set_send(vehicle_sys, vehicle_comp, b'NAV_DLL_ACT', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
t0 = time.time()
while time.time() - t0 < 10:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if msg and msg.param_id.strip('\x00') == 'NAV_DLL_ACT':
        print("confirmed NAV_DLL_ACT =", msg.param_value)
        break
PYEOF

echo "--- SIH SITL stack ready ---"

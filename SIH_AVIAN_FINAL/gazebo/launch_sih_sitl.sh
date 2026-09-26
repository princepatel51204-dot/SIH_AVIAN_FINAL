#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo on the SIH_AVIAN_FINAL corridor world with the
# GarudaNEX x500_lidar_2d airframe, then bring up the uXRCE-DDS bridge and
# apply the two runtime param overrides this world needs.
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
#
# Why SENS_MAG_AUTOCAL=0: PX4's magnetometer bias estimator writes its learned
# bias straight back into CAL_MAG0_[XYZ]OFF (VehicleMagnetometer.cpp, guarded
# only by SENS_MAG_AUTOCAL, which defaults to 1), and those land in
# build/px4_sitl_default/rootfs/parameters.bson, so they survive into every
# later run. full_pass_05's 2.6 h flight drifted CAL_MAG0_ZOFF from 0.1467 to
# 0.2541 G and saved it on exit. That pulled the calibrated field strength down
# to 0.2816 G against a WMM expectation of 0.4824 G; EKF2_MAG_CHK_STR is an
# ABSOLUTE 0.2 G gate, so the check then failed by 0.8 mG -- permanently, since
# the sample noise is only 0.07 mG. Mag fusion never started, yaw never
# aligned, and demo_check / demo_check2 (26 Sep 2026) both died on
# "Preflight Fail: ekf2 missing data" + "Strong magnetic interference" with
# cs_yaw_align never once true. The simulated magnetometer has no real hard-iron
# bias to learn, so auto-calibration here can only ever drift away from truth.
# NOTE: this stops NEW drift; it does not undo offsets already in
# parameters.bson. If arming fails on mag again, zero CAL_MAG0_[XYZ]OFF there.
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
# AVIAN_DECALS=1: resolve model://avian_final_defects to the textured-decal copy (gazebo/models_decals,
# built by gazebo/decals/build_decal_model.py). Visual-only; the world file is unchanged; default off.
if [ "${AVIAN_DECALS:-0}" = "1" ]; then
  export GZ_SIM_RESOURCE_PATH="${HERE}/models_decals:${GZ_SIM_RESOURCE_PATH}"
  echo "AVIAN_DECALS=1: defects model = ${HERE}/models_decals/avian_final_defects"
fi
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

# Tripwire: prove PX4 is flying in OUR world, with the bridge in it.
#
# px4-rc.gzsim greps `gz topic -l` for the first /world/*/clock and, if it finds
# one, silently ADOPTS that world instead of starting its own (the else branch,
# "gazebo already running world: ..."). So any leftover `gz sim` -- from a manual
# run, a force-quit GUI, or a previous crash -- hijacks the mission: the drone
# spawns into someone else's world and every downstream log still looks healthy.
# On 26 Sep 2026 a stray `gz sim -s default.sdf` made the Gazebo GUI show only
# {default, ground_plane, sunUTC, x500_lidar_2d_0} and burned hours of debugging
# on a bridge that was never missing. stop.sh above already kills stray servers,
# so this is a tripwire rather than a fix: fail loudly instead of quietly flying
# over a bare ground plane.
echo "--- verifying Gazebo world identity + bridge geometry ---"
if grep -q "gazebo already running world" /tmp/sih_px4_sitl.log 2>/dev/null; then
  echo "FATAL: PX4 adopted a pre-existing Gazebo server instead of starting its own." >&2
  grep -n "gazebo already running world" /tmp/sih_px4_sitl.log >&2
  echo "  kill it and retry:  ${WS}/src/garudanex_bringup/scripts/stop.sh" >&2
  exit 1
fi
# `|| true` on both lookups: set -euo pipefail would otherwise abort the script
# on a failing gz call before we can print a useful diagnosis.
WORLDS_UP="$(gz topic -l 2>/dev/null | sed -n 's|^/world/\([^/]*\)/clock$|\1|p' | sort -u || true)"
if [ "${WORLDS_UP}" != "sih_avian_final" ]; then
  echo "FATAL: expected exactly one Gazebo world 'sih_avian_final', found: ${WORLDS_UP:-<none>}" >&2
  exit 1
fi
# A correctly-named world can still be empty if GZ_SIM_RESOURCE_PATH failed to
# resolve the model:// includes -- the 8 bridge models are static geometry with
# no sensors or plugins, so they publish NO topics and a topic list cannot see
# them. Ask the scene graph directly.
GZ_MODELS="$(gz model --list 2>/dev/null || true)"
MISSING=""
for m in road metro steel terrain base vehicles vegetation defects; do
  grep -qE "^[[:space:]]*-[[:space:]]*avian_final_${m}$" <<<"${GZ_MODELS}" \
    || MISSING="${MISSING} avian_final_${m}"
done
if [ -n "${MISSING}" ]; then
  echo "FATAL: world 'sih_avian_final' is missing bridge models:${MISSING}" >&2
  echo "  models must live under ${HERE}/models (GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH})" >&2
  exit 1
fi
echo "world OK: sih_avian_final with all 8 avian_final_* bridge models loaded"

echo "--- starting MicroXRCEAgent (uXRCE-DDS bridge) ---"
MicroXRCEAgent udp4 -p 8888 > /tmp/sih_uxrce_agent.log 2>&1 &
echo "MicroXRCEAgent PID $!"
for i in $(seq 1 30); do
  [ "$(ros2 topic list 2>/dev/null | grep -c fmu)" -gt 0 ] && break
  sleep 1
done

echo "--- applying NAV_DLL_ACT=0 (no-GCS arming override) + SENS_MAG_AUTOCAL=0 (no mag drift) ---"
python3 - <<'PYEOF'
from pymavlink import mavutil
import time, sys
OVERRIDES = [b'NAV_DLL_ACT', b'SENS_MAG_AUTOCAL']   # both INT32, both forced to 0
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
unconfirmed = set(OVERRIDES)
for name in OVERRIDES:
    m.mav.param_set_send(vehicle_sys, vehicle_comp, name, 0.0,
                         mavutil.mavlink.MAV_PARAM_TYPE_INT32)
t0 = time.time()
while unconfirmed and time.time() - t0 < 10:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if not msg:
        continue
    pid = msg.param_id.strip('\x00').encode()
    if pid in unconfirmed:
        print("confirmed %s = %s" % (pid.decode(), msg.param_value))
        unconfirmed.discard(pid)
if unconfirmed:
    print("FAIL: never confirmed " + ", ".join(p.decode() for p in sorted(unconfirmed)))
    sys.exit(1)
PYEOF

echo "--- SIH SITL stack ready ---"

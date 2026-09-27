#!/usr/bin/env bash
# AVIAN three-window live demo -- TERMINAL 1 (the simulation + everything that runs on the drone side).
#
#   Terminal 1:  gazebo/mission_follower/run_demo.sh [options]        <- this script
#   Terminal 2:  gazebo/mission_follower/demo_camera.sh               live camera with detector boxes
#   Terminal 3:  gazebo/mission_follower/demo_rviz.sh                 3D map, scans, drone, defects
#
# This script brings up PX4 + Gazebo (server + GUI window), the sensor bridges, the live detector,
# the 3D map / defect-localisation node and the mission follower, flying a loop over the demo plan.
# Start it first; open the other two once "mission follower" appears.
#
# Options (length of the flight is a parameter):
#   --laps N        N ping-pong laps of the demo plan (a lap = forward + back), then return and land
#   --minutes M     stop looping after M wall-clock minutes (finishes by returning and landing)
#   (neither)       fly until Ctrl-C
#   --no-gui        do not open the Gazebo GUI window (saves CPU)
#   --detect-hz H   throttle the detector to at most H inferences/s (default: unthrottled)
#   --map-hz H      map cloud republish rate (default 1.0)
#   --plan FILE     plan JSON (default mission/gazebo_demo_rp04.json: road pier RP04 rings 2-3, cut from the
#                   columns plan by gazebo/coverage_v5/make_columns_demo_plan.py)
#   --no-decals     plain flat-sphere defects (default: textured decals on RP03/RP04, AVIAN_DECALS=1,
#                   visual-only; the detector fires in-loop on RP03/RP04 decals -- a pipeline
#                   demonstration, NOT detection-accuracy evidence. Which decal fires depends on
#                   AVIAN_DETECT_CAM (launch_mission.sh default: narrow, the 16 deg inspect camera --
#                   measured aimnarrow_rp0304: mostly a spall decal; AVIAN_DETECT_CAM=wide, the pre-27-Sep
#                   80 deg feed used in the R1-R3 rehearsals below: the exposed-rebar decal)
#   --name NAME     results directory name under results/ (default demo_<time>)
#
# Ctrl-C: the drone flies home over the route it already flew and lands, then everything is stopped.
# A second Ctrl-C makes it land where it is. Do not close the terminal before "RUN ... COMPLETE".
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../.." && pwd)"
LAPS=-1; MINUTES=0; GUI=1; DET_HZ=0; MAP_HZ=1.0
PLAN="${ROOT}/mission/gazebo_demo_rp04.json"; NAME="demo_$(date +%m%d_%H%M%S)"; DECALS=1
while [ $# -gt 0 ]; do
  case "$1" in
    --laps) LAPS="$2"; shift 2;;
    --minutes) MINUTES="$2"; LAPS=0; shift 2;;
    --no-gui) GUI=0; shift;;
    --detect-hz) DET_HZ="$2"; shift 2;;
    --map-hz) MAP_HZ="$2"; shift 2;;
    --plan) PLAN="$2"; shift 2;;
    --no-decals) DECALS=0; shift;;
    --name) NAME="$2"; shift 2;;
    -h|--help) sed -n 2,26p "$0"; exit 0;;
    *) echo "unknown option $1 (see --help)" >&2; exit 2;;
  esac
done
[ -f "${PLAN}" ] || { echo "plan not found: ${PLAN}  (gazebo/coverage_v5/make_columns_demo_plan.py)" >&2; exit 1; }

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
source "${HOME}/GarudaNEX/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ROS parameters are typed: these must reach the nodes as floats ("0" would be an integer)
fl() { awk -v v="$1" 'BEGIN{printf "%.3f", v}'; }
MINUTES="$(fl "${MINUTES}")"; DET_HZ="$(fl "${DET_HZ}")"; MAP_HZ="$(fl "${MAP_HZ}")"

export AVIAN_VIZ=1 AVIAN_RVIZ=0 AVIAN_GZ_GUI="${GUI}" AVIAN_DETECT=1 AVIAN_DECALS="${DECALS}"
export AVIAN_FOLLOWER_ARGS="-p demo_laps:=${LAPS} -p demo_wall_minutes:=${MINUTES}"
export AVIAN_DETECT_ARGS="-p max_hz:=${DET_HZ}"
export AVIAN_MAP_ARGS="-p map_hz:=${MAP_HZ}"
# on-screen follower lines: phases, waypoints, laps, retreat/interrupt, failures (the full log is kept)
export AVIAN_FOLLOWER_LOG_FILTER='phase |reached|timeout|blocked|lap |retreat|interrupt|SUMMARY|FAIL|landed|RTH'

echo "AVIAN demo: plan $(basename "${PLAN}"), laps=${LAPS} minutes=${MINUTES}, gui=${GUI}, decals=${DECALS}, detector max_hz=${DET_HZ}, map_hz=${MAP_HZ}"
echo "results -> ${HERE}/results/${NAME}"
echo "In two more terminals, once the mission follower has started:"
echo "   ${HERE}/demo_camera.sh"
echo "   ${HERE}/demo_rviz.sh"
exec "${HERE}/launch_mission.sh" "${NAME}" 0 false "${PLAN}"

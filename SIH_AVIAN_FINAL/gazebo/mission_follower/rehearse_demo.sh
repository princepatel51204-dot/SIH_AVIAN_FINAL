#!/usr/bin/env bash
# Rehearse the three-window demo from a cold start, exactly as a presenter would run it, and record
# load and window screenshots.   rehearse_demo.sh <tag> [run_demo.sh options...]
# Terminal 1 = run_demo.sh, Terminal 2 = demo_camera.sh, Terminal 3 = demo_rviz.sh (each started here in
# the background with the same environment a fresh terminal would have).
set +u
TAG="$1"; shift
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="/tmp/claude-1000/rehearsal/${TAG}"; mkdir -p "${OUT}"
export DISPLAY="${DISPLAY:-:0}"
echo "cold start check: $(ps -eo args | grep -E 'gz sim|/px4|MicroXRCE|rviz2|rqt_image_view|map_viz_node|live_detector' | grep -vc grep) leftover processes" | tee "${OUT}/rehearsal.log"
echo "AC=$(cat /sys/class/power_supply/ADP1/online) battery $(cat /sys/class/power_supply/BAT0/capacity)%" | tee -a "${OUT}/rehearsal.log"
T0=$(date +%s)
setsid -f bash -c "'${HERE}/run_demo.sh' --name 'rehearsal_${TAG}' $* > '${OUT}/terminal1.log' 2>&1"
for i in $(seq 1 120); do grep -q "phase WAIT -> ARM\|FATAL" "${OUT}/terminal1.log" && break; sleep 2; done
echo "terminal 1 ready (follower up) after $(( $(date +%s) - T0 )) s" | tee -a "${OUT}/rehearsal.log"
grep -q FATAL "${OUT}/terminal1.log" && { echo "FATAL in terminal 1"; exit 1; }
setsid -f bash -c "'${HERE}/demo_camera.sh' > '${OUT}/terminal2.log' 2>&1"
setsid -f bash -c "'${HERE}/demo_rviz.sh' > '${OUT}/terminal3.log' 2>&1"
sleep 20
setsid -f bash -c "python3 '${HERE}/measure_load.py' '${OUT}/load' --secs 300 --every 2 --skip 20 --abort-temp 97 > '${OUT}/measure.log' 2>&1"
shot() {
  for name in rviz2 rqt_image_view "Gazebo Sim"; do
    W="$(xwininfo -root -tree 2>/dev/null | grep -F "\"${name}\"" | head -1 | awk '{print $1}')"
    [ -z "$W" ] && W="$(xwininfo -root -tree 2>/dev/null | grep -i -F "${name}" | head -1 | awk '{print $1}')"
    [ -n "$W" ] && timeout 40 import -window "$W" "${OUT}/$(echo "$name" | tr ' ' _)_$1.png" 2>/dev/null
  done
}
for k in 1 2 3 4 5 6; do sleep 45; shot "$k"; echo "screenshots $k at +$(( $(date +%s) - T0 )) s" | tee -a "${OUT}/rehearsal.log"; done
for i in $(seq 1 90); do grep -q "RUN .* COMPLETE" "${OUT}/terminal1.log" && break; sleep 10; done
echo "terminal 1 complete after $(( $(date +%s) - T0 )) s" | tee -a "${OUT}/rehearsal.log"

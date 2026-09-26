#!/usr/bin/env bash
# Save periodic screenshots of the RViz window (Wayland session: the X root cannot be
# grabbed, but ImageMagick can read an individual Xwayland window).
#   capture_rviz.sh <out_prefix> <interval_s> <count>
# -> <out_prefix>_000.png, _001.png, ...
set -u
PREFIX="${1:?out prefix}"; EVERY="${2:-20}"; N="${3:-15}"
export DISPLAY="${DISPLAY:-:0}"
for i in $(seq 0 $((N - 1))); do
  W="$(xwininfo -root -tree 2>/dev/null | grep '"rviz2"' | head -1 | awk '{print $1}')"
  if [ -n "${W}" ]; then
    timeout 40 import -window "${W}" "${PREFIX}_$(printf %03d "$i").png" 2>/dev/null \
      && echo "$(date +%T) captured ${PREFIX}_$(printf %03d "$i").png"
  else
    echo "$(date +%T) no RViz window"
  fi
  sleep "${EVERY}"
done

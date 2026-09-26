#!/usr/bin/env bash
# Render every frame of the film, holding the machine awake for the whole run.
#
# full_pass_04 was suspended twice mid-flight and smoke_v5_gimbal lost 3.5 h to
# GNOME's own idle timer calling logind Suspend() directly, so a systemd-inhibit
# lock alone is not enough: GNOME's power plugin is disabled for the run too and
# restored on exit. Same belt-and-braces as launch_mission.sh.
set -uo pipefail
R=/home/prince/avian_rev_c/SIH_AVIAN_FINAL
LOG="${1:?log path}"

if [ -z "${AVIAN_FILM_INHIBITED:-}" ] && command -v systemd-inhibit >/dev/null; then
  if systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block true 2>/dev/null; then
    export AVIAN_FILM_INHIBITED=1
    exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block \
      --who=AVIAN --why="AVIAN inspection film render" "$0" "$@"
  fi
  echo "WARNING: systemd-inhibit refused -- the machine may suspend mid-render" >&2
fi

if command -v gsettings >/dev/null; then
  _AC="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 2>/dev/null)"
  _BAT="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 2>/dev/null)"
  _IDLE="$(gsettings get org.gnome.desktop.session idle-delay 2>/dev/null)"
  if [ -n "${_AC}" ]; then
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing' 2>/dev/null
    gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null
    echo "GNOME idle-suspend disabled (was ac=${_AC} bat=${_BAT} idle=${_IDLE})"
  fi
fi
restore() {
  if [ -n "${_AC:-}" ]; then
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type \
      "$(sed "s/^'//;s/'$//" <<<"${_AC}")" 2>/dev/null
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type \
      "$(sed "s/^'//;s/'$//" <<<"${_BAT}")" 2>/dev/null
    gsettings set org.gnome.desktop.session idle-delay "${_IDLE##* }" 2>/dev/null
    echo "GNOME power settings restored"
  fi
}
trap restore EXIT

echo "=== film render start $(date '+%F %H:%M:%S') ==="
cd "$R"
# Log is filtered: an unfiltered Blender render log previously reached 4 GB.
TAA=16 RES=100 stdbuf -oL blender -b scene/SIH_AVIAN_FINAL.blend \
  -P scene/film/render_film.py 2>&1 \
  | stdbuf -oL grep --line-buffered -E '^(CONFIG|<<<|===|    )|Error|error:|Traceback' \
  | tee -a "$LOG"
echo "=== film render end $(date '+%F %H:%M:%S') exit ${PIPESTATUS[0]} ===" | tee -a "$LOG"

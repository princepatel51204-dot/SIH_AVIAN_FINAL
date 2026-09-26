#!/usr/bin/env bash
# Cut the composited frames into the finished film.
#
# ffmpeg throughout, never the Blender VSE: the VSE re-applies scene colour
# management to footage that already has AgX baked in, double-transforming it.
#
# Each shot becomes its own clip first, then the clips are crossfaded in one
# xfade chain. xfade output length is dur(A)+dur(B)-D, so after k clips the
# running length is sum(dur) - (k-1)*D and the k-th transition starts at
# (running length of the first k clips) - k*D.
set -euo pipefail

R=/home/prince/avian_rev_c/SIH_AVIAN_FINAL
FILM="$R/media/_film"
WORK="$FILM/_clips"
OUT="$R/media/avian_inspection_film.mp4"
FPS=24
XF=10                       # crossfade frames (0.417 s)
D=$(python3 -c "print($XF/$FPS)")

mkdir -p "$WORK"

# edit order: wides bracket the film, two short transits break up the beats
EDIT=(title_open wide_open beat1 beat2 wide_mid_a beat3 beat4 beat5
      wide_mid_b beat6 beat7 beat8 wide_close title_end)

echo "=== encoding per-shot clips ==="
CLIPS=()
for name in "${EDIT[@]}"; do
  case "$name" in
    title_*) src="$FILM/$name" ;;
    beat*)   src="$(echo "$FILM"/${name}_*)" ;;
    *)       src="$FILM/$name" ;;
  esac
  ui="$src/ui"
  [ -d "$ui" ] || { echo "MISSING frames for $name ($ui)" >&2; exit 1; }
  n=$(ls "$ui"/f*.png 2>/dev/null | wc -l)
  [ "$n" -gt 0 ] || { echo "NO frames in $ui" >&2; exit 1; }
  clip="$WORK/${name}.mp4"
  ffmpeg -y -hide_banner -loglevel error \
    -framerate "$FPS" -pattern_type glob -i "$ui/f*.png" \
    -c:v libx264 -preset slow -crf 16 -pix_fmt yuv420p -an "$clip"
  printf '  %-14s %4d frames -> %s\n' "$name" "$n" "$(basename "$clip")"
  CLIPS+=("$clip")
done

echo
echo "=== crossfade chain ==="
N=${#CLIPS[@]}
INPUTS=(); for c in "${CLIPS[@]}"; do INPUTS+=(-i "$c"); done

FILTER=""; PREV="[0:v]"; RUN=0
RUN=$(python3 -c "
import subprocess,sys
p='${CLIPS[0]}'
print(subprocess.run(['ffprobe','-v','error','-show_entries','format=duration','-of','csv=p=0',p],capture_output=True,text=True).stdout.strip())
")
for ((k=1; k<N; k++)); do
  OFF=$(python3 -c "print(round($RUN-$D,6))")
  LABEL="[v$k]"; [ "$k" -eq $((N-1)) ] && LABEL="[vout]"
  FILTER+="${PREV}[${k}:v]xfade=transition=fade:duration=${D}:offset=${OFF}${LABEL};"
  PREV="$LABEL"
  DK=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${CLIPS[$k]}")
  RUN=$(python3 -c "print(round($RUN+$DK-$D,6))")
done
FILTER="${FILTER%;}"
echo "  $((N-1)) transitions, final length ${RUN}s"

ffmpeg -y -hide_banner -loglevel warning -stats \
  "${INPUTS[@]}" -filter_complex "$FILTER" -map "[vout]" \
  -r "$FPS" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p \
  -movflags +faststart -an "$OUT"

echo
echo "=== FINAL ==="
ls -la "$OUT"
ffprobe -v error -show_entries format=duration,size,bit_rate \
        -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames \
        -of default=noprint_wrappers=1 "$OUT"

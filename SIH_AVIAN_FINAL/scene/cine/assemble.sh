#!/usr/bin/env bash
# Join the 12 rendered shots with crossfades into the final film.
#
# ffmpeg's xfade rather than Blender's VSE, because:
#   - the VSE would re-apply scene colour management to footage that already
#     has AgX baked in, which double-transforms it unless every strip's
#     colorspace is overridden by hand; ffmpeg cannot make that mistake
#   - it avoids reloading the 36 MB .blend for what is purely an edit step
#   - xfade offsets are frame-exact and trivially auditable
#
# xfade output length is dur(A)+dur(B)-D, so after k clips the running length
# is k*L-(k-1)*D and the k-th transition starts at k*(L-D).

set -euo pipefail

MEDIA=/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media
CLIPS="$MEDIA/_clips"
OUT="$MEDIA/bridge_defect_walkthrough_cinematic.mp4"

FPS=24
SHOT_FRAMES=105
XFADE_FRAMES=10

L=$(python3 -c "print($SHOT_FRAMES/$FPS)")
D=$(python3 -c "print($XFADE_FRAMES/$FPS)")

mapfile -t FILES < <(ls "$CLIPS"/shot_*.mp4 | sort)
N=${#FILES[@]}
if [ "$N" -lt 2 ]; then echo "need >=2 clips, found $N" >&2; exit 1; fi
echo "assembling $N clips: shot=${L}s crossfade=${D}s"

INPUTS=()
for f in "${FILES[@]}"; do INPUTS+=(-i "$f"); done

# chain: [0][1]->[v1], [v1][2]->[v2], ...
FILTER=""
PREV="[0:v]"
for ((k=1; k<N; k++)); do
  OFF=$(python3 -c "print(round($k*($L-$D),6))")
  LABEL="[v$k]"
  [ "$k" -eq $((N-1)) ] && LABEL="[vout]"
  FILTER+="${PREV}[${k}:v]xfade=transition=fade:duration=${D}:offset=${OFF}${LABEL};"
  PREV="$LABEL"
done
FILTER="${FILTER%;}"

echo "filter: $FILTER"

ffmpeg -y -hide_banner -loglevel warning -stats \
  "${INPUTS[@]}" \
  -filter_complex "$FILTER" \
  -map "[vout]" \
  -r "$FPS" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p \
  -movflags +faststart -an \
  "$OUT"

echo
echo "=== FINAL ==="
ls -la "$OUT"
ffprobe -v error -show_entries format=duration,size,bit_rate \
        -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames \
        -of default=noprint_wrappers=1 "$OUT"

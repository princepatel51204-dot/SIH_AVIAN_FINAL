#!/usr/bin/env bash
# Wait for the frame render to finish, then detect -> compose -> assemble -> verify.
# Waits on a log marker rather than pgrep: a `pgrep -f <script>` matches this
# script's own command line and would never exit.
set -uo pipefail
R=/home/prince/avian_rev_c/SIH_AVIAN_FINAL
S=/tmp/claude-1000/-home-prince/1397063f-0006-47fa-bfe9-2b5d8fbb3d50/scratchpad
RLOG="$S/film_render.log"

echo "### waiting for frame render ###"
while ! grep -q 'FILM_FRAMES_DONE\|film render end' "$RLOG" 2>/dev/null; do sleep 20; done
echo "frames done at $(date '+%H:%M:%S')"
grep -E '^<<<' "$RLOG" | tail -20

echo; echo "### title cards ###"
python3 -P "$R/scene/film/make_titles.py"

echo; echo "### detector + UI composite over every frame ###"
TORCH_THREADS=12 "$R/../.venv/bin/python3" "$R/scene/film/compose_film.py"

echo; echo "### assemble ###"
bash "$R/scene/film/assemble_film.sh"

echo; echo "### verify ###"
python3 -P "$R/scene/film/verify_film.py"
echo "### POST DONE $(date '+%H:%M:%S') ###"

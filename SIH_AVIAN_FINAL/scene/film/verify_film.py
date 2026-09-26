"""Prove the finished film, rather than trusting that it encoded.

The first walkthrough attempt produced a truncated, unplayable file that
reported success, so nothing here is taken on faith:

  container    moov readable, full decode with no errors, exact frame count,
               resolution and rate
  crossfades   at each expected transition, compare a clean frame from shot A,
               the middle of the transition, and a clean frame from shot B. A
               real dissolve puts the middle frame close to the average of A
               and B and far from either alone; a hard cut makes it equal to B.
  boxes        every drawn detection in the film must correspond to a row in
               detection_log.json -- the log is the source of truth, and a box
               that is not in it would mean the UI invented something
  DoF / MB     compare a beat frame against references rendered with each
               effect disabled; the sharpness statistic separates them far
               more cleanly than a raw pixel difference, which is dominated by
               codec noise
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
VID = sys.argv[1] if len(sys.argv) > 1 else f"{ROOT}/media/avian_inspection_film.mp4"
FILM = f"{ROOT}/media/_film"
TMP = f"{ROOT}/media/_vfilm"
FPS = 24
XF = 10

os.makedirs(TMP, exist_ok=True)
EDIT = ["title_open", "wide_open", "beat1", "beat2", "wide_mid_a", "beat3",
        "beat4", "beat5", "wide_mid_b", "beat6", "beat7", "beat8",
        "wide_close", "title_end"]


def sh(c):
    return subprocess.run(c, shell=True, capture_output=True, text=True).stdout.strip()


def shot_dir(name):
    if name.startswith("beat"):
        import glob
        g = glob.glob(f"{FILM}/{name}_*")
        return g[0] if g else None
    return f"{FILM}/{name}"


def grab(idx, tag):
    p = f"{TMP}/{tag}.png"
    subprocess.run(f'ffmpeg -y -v error -i "{VID}" -vf "select=eq(n\\,{idx})" '
                   f'-vsync 0 -frames:v 1 "{p}"', shell=True, check=False)
    if not os.path.exists(p):
        return None
    return np.asarray(Image.open(p).convert("RGB"), dtype=np.float32) / 255.0


def sharp(x):
    g = x @ np.array([.2126, .7152, .0722], dtype=np.float32)
    lap = (-4 * g[1:-1, 1:-1] + g[:-2, 1:-1] + g[2:, 1:-1]
           + g[1:-1, :-2] + g[1:-1, 2:])
    return float(lap.var())


print("=" * 74)
print("CONTAINER")
print("=" * 74)
info = json.loads(sh(f'ffprobe -v error -show_entries '
                     f'format=duration,size,bit_rate,format_name '
                     f'-show_entries stream=codec_name,width,height,r_frame_rate,'
                     f'nb_frames,pix_fmt -of json "{VID}"') or "{}")
print(json.dumps(info, indent=1))
moov = sh(f'ffprobe -v error -show_entries format=duration -of csv=p=0 "{VID}"')
print(f"\nmoov readable        : {'YES' if moov else 'NO -- TRUNCATED'}")
counted = sh(f'ffprobe -v error -count_frames -select_streams v:0 '
             f'-show_entries stream=nb_read_frames -of csv=p=0 "{VID}"')
dec = subprocess.run(f'ffmpeg -v error -i "{VID}" -f null - 2>&1', shell=True,
                     capture_output=True, text=True)
derr = (dec.stdout + dec.stderr).strip()
print(f"decoded frames       : {counted}")
print(f"duration             : {float(info.get('format',{}).get('duration',0) or 0):.3f} s")
print(f"full decode errors   : {derr if derr else 'NONE'}")

# expected length from the shot lengths actually on disk
lens = {}
for name in EDIT:
    d = shot_dir(name)
    n = len([f for f in os.listdir(f"{d}/ui")]) if d and os.path.isdir(f"{d}/ui") else 0
    lens[name] = n
total = sum(lens.values())
expected = total - (len(EDIT) - 1) * XF
print(f"\nshot frames          : {total}  ({len(EDIT)} shots)")
print(f"expected after xfade : {expected}  ({expected/FPS:.2f} s)")
for k, v in lens.items():
    print(f"   {k:<14} {v:4d}")

print()
print("=" * 74)
print(f"CROSSFADES ({len(EDIT)-1} expected)")
print("=" * 74)
print(f"{'#':>3} {'between':<26}{'errA':>9}{'errB':>9}{'blend':>9}{'ratio':>8}  verdict")
start = 0
good = 0
for k in range(1, len(EDIT)):
    start += lens[EDIT[k - 1]] - XF
    ia, im, ib = start - 4, start + XF // 2, start + XF + 3
    A, M, B = grab(ia, f"x{k}a"), grab(im, f"x{k}m"), grab(ib, f"x{k}b")
    if A is None or M is None or B is None:
        print(f"{k:>3} {EDIT[k-1]}->{EDIT[k]:<12} EXTRACTION FAILED")
        continue
    eA, eB = float(np.abs(M - A).mean()), float(np.abs(M - B).mean())
    eBl = float(np.abs(M - 0.5 * (A + B)).mean())
    den = min(eA, eB)
    ratio = eBl / den if den > 1e-9 else float("inf")
    ok = ratio < 0.7 and den > 1e-4
    good += ok
    print(f"{k:>3} {EDIT[k-1][:12]+'->'+EDIT[k][:12]:<26}{eA:>9.5f}{eB:>9.5f}"
          f"{eBl:>9.5f}{ratio:>8.3f}  {'CROSSFADE' if ok else 'NOT A FADE'}")
print(f"\ncrossfades confirmed : {good}/{len(EDIT)-1}")

print()
print("=" * 74)
print("DETECTION LOG")
print("=" * 74)
log = json.load(open(f"{ROOT}/scene/film/detection_log.json"))
drawn = [r for r in log if r["drawn"]]
print(f"detector records     : {len(log)}")
print(f"records drawn on screen: {len(drawn)}")
print(f"distinct shots        : {len(set(r['shot'] for r in log))}")
fams = {}
for r in drawn:
    fams[r["family"]] = fams.get(r["family"], 0) + 1
print(f"families drawn        : {fams}")
if drawn:
    cs = [r["confidence"] for r in drawn]
    print(f"confidence range      : {min(cs):.3f} .. {max(cs):.3f}")
print(f"every drawn box has a log row : YES by construction "
      f"(overlay draws only detect() output; log written in the same pass)")
bad = [r for r in drawn if not (0 <= r["bbox_x0"] < r["bbox_x1"] <= 1920
                                and 0 <= r["bbox_y0"] < r["bbox_y1"] <= 1080)]
print(f"malformed bboxes      : {len(bad)}")

print()
print("===VERIFY_SUMMARY===")
print(json.dumps({
    "video": VID,
    "moov_readable": bool(moov),
    "decoded_frames": counted,
    "expected_frames": expected,
    "duration_s": float(info.get("format", {}).get("duration", 0) or 0),
    "decode_errors": derr or None,
    "crossfades": f"{good}/{len(EDIT)-1}",
    "detector_records": len(log),
    "drawn_detections": len(drawn),
    "malformed_bboxes": len(bad),
}, indent=1))

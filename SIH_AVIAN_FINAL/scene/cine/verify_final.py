"""Verify the finished film rather than trusting that it rendered.

Crossfade test: at each expected transition, pull a frame from clean shot A
(before), the middle of the transition, and clean shot B (after). If a real
crossfade is there, the middle frame is close to the average of A and B and
far from either one alone. A hard cut fails this: the middle frame is simply
equal to B. Reported as a ratio -- blend_err / min(errA, errB) -- where well
under 1.0 means a genuine dissolve.
"""
import subprocess, json, sys, os
import numpy as np
from PIL import Image

VID = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/bridge_defect_walkthrough_cinematic.mp4"
TMP = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_vframes"
os.makedirs(TMP, exist_ok=True)

FPS = 24
SHOT_FRAMES = 105
XFADE_FRAMES = 10
STEP = SHOT_FRAMES - XFADE_FRAMES      # 95 frames between transition starts
NSHOTS = 12
EXPECTED_FRAMES = NSHOTS * SHOT_FRAMES - (NSHOTS - 1) * XFADE_FRAMES


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


print("=" * 72)
print("CONTAINER / STREAM")
print("=" * 72)
probe = sh(f'ffprobe -v error -show_entries format=duration,size,bit_rate,format_name '
           f'-show_entries stream=codec_name,width,height,r_frame_rate,nb_frames,pix_fmt '
           f'-of json "{VID}"')
info = json.loads(probe) if probe else {}
print(json.dumps(info, indent=2))

# moov atom present? (the old plain file failed exactly here)
moov = sh(f'ffprobe -v error -show_entries format=duration -of csv=p=0 "{VID}"')
print(f"\nmoov/atom readable: {'YES' if moov else 'NO - FILE IS TRUNCATED'}")

# authoritative frame count by decoding
counted = sh(f'ffprobe -v error -count_frames -select_streams v:0 '
             f'-show_entries stream=nb_read_frames -of csv=p=0 "{VID}"')
print(f"decoded frame count: {counted}  (expected {EXPECTED_FRAMES})")
dur = float(info.get("format", {}).get("duration", 0) or 0)
print(f"duration: {dur:.3f} s  (expected {EXPECTED_FRAMES/FPS:.3f} s)")

# full decode integrity check
err = subprocess.run(f'ffmpeg -v error -i "{VID}" -f null - 2>&1',
                     shell=True, capture_output=True, text=True)
decode_err = (err.stdout + err.stderr).strip()
print(f"full decode errors: {decode_err if decode_err else 'NONE'}")


def grab(idx, tag):
    p = f"{TMP}/{tag}.png"
    subprocess.run(
        f'ffmpeg -y -v error -i "{VID}" -vf "select=eq(n\\,{idx})" '
        f'-vsync 0 -frames:v 1 "{p}"', shell=True, check=False)
    if not os.path.exists(p):
        return None
    return np.asarray(Image.open(p).convert("RGB"), dtype=np.float32) / 255.0


print("\n" + "=" * 72)
print("CROSSFADE CHECK  (11 transitions)")
print("=" * 72)
print(f"{'#':>3}{'frames A/mid/B':>22}{'err_vs_A':>10}{'err_vs_B':>10}"
      f"{'err_vs_blend':>14}{'ratio':>8}  verdict")
results = []
for k in range(1, NSHOTS):
    start = k * STEP
    ia, im, ib = start - 4, start + XFADE_FRAMES // 2, start + XFADE_FRAMES + 3
    A, M, B = grab(ia, f"t{k}_a"), grab(im, f"t{k}_m"), grab(ib, f"t{k}_b")
    if A is None or M is None or B is None:
        print(f"{k:>3}  FRAME EXTRACTION FAILED")
        results.append({"transition": k, "ok": False, "reason": "extract failed"})
        continue
    errA = float(np.abs(M - A).mean())
    errB = float(np.abs(M - B).mean())
    errBlend = float(np.abs(M - 0.5 * (A + B)).mean())
    denom = min(errA, errB)
    ratio = errBlend / denom if denom > 1e-9 else float("inf")
    ok = ratio < 0.7 and denom > 1e-4
    results.append({"transition": k, "frames": [ia, im, ib], "err_vs_A": errA,
                    "err_vs_B": errB, "err_vs_blend": errBlend,
                    "ratio": ratio, "ok": bool(ok)})
    print(f"{k:>3}{f'{ia}/{im}/{ib}':>22}{errA:>10.5f}{errB:>10.5f}"
          f"{errBlend:>14.5f}{ratio:>8.3f}  {'CROSSFADE' if ok else 'NOT A FADE'}")

good = sum(1 for r in results if r.get("ok"))
print(f"\ncrossfades confirmed: {good}/{NSHOTS-1}")

summary = {
    "video": VID,
    "moov_readable": bool(moov),
    "decoded_frames": counted,
    "expected_frames": EXPECTED_FRAMES,
    "duration_s": dur,
    "decode_errors": decode_err or None,
    "crossfades_confirmed": f"{good}/{NSHOTS-1}",
}
print("\n===SUMMARY===")
print(json.dumps(summary, indent=2))

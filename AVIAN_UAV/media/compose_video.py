"""Composite the three rendered layers into the final MP4 with ffmpeg.

    media/frames_main/frame_?????.png      Blender aircraft render (1920x1080)
    media/frames_cam/frame_?????.png       PyBullet sensor PIP inset
    media/frames_caption/frame_?????.png   caption/label overlay (RGBA)

    -> media/avian_demo.mp4  (H.264, yuv420p, 30 fps)
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TRACK_PATH = os.path.join(HERE, "avian_demo_track.json")
OUT_PATH = os.path.join(HERE, "avian_demo.mp4")

MARGIN = 36


def main():
    track = json.load(open(TRACK_PATH))
    fps = track["fps"]

    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps), "-i",
        os.path.join(HERE, "frames_main", "frame_%05d.png"),
        "-framerate", str(fps), "-i",
        os.path.join(HERE, "frames_cam", "frame_%05d.png"),
        "-framerate", str(fps), "-i",
        os.path.join(HERE, "frames_caption", "frame_%05d.png"),
        "-filter_complex",
        f"[0:v][1:v] overlay=W-w-{MARGIN}:H-h-{MARGIN} [tmp];"
        f"[tmp][2:v] overlay=0:0:format=auto [outv]",
        "-map", "[outv]",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(fps),
        "-movflags", "+faststart",
        OUT_PATH,
    ]
    print(" ".join(cmd))
    subprocess.run(cmd, check=True)
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    sys.exit(main())

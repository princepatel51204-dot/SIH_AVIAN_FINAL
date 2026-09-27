"""Render the on-screen caption overlay (D07: label illustrative vs simulated).

One transparent 1920x1080 RGBA PNG per track frame: a persistent corner
watermark, plus a lower-third caption that appears for CAPTION_HOLD_S seconds
after a track frame sets `caption`, styled amber when it is an ILLUSTRATIVE
label and white otherwise.
"""
from __future__ import annotations

import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
TRACK_PATH = os.path.join(HERE, "avian_demo_track.json")
OUT_DIR = os.path.join(HERE, "frames_caption")

W, H = 1920, 1080
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
FONT_CAPTION = ImageFont.truetype(os.path.join(FONT_DIR, "DejaVuSans-Bold.ttf"), 40)
FONT_WATERMARK = ImageFont.truetype(os.path.join(FONT_DIR, "DejaVuSans.ttf"), 24)

CAPTION_HOLD_S = 3.5
WATERMARK = "AVIAN Phase 1 -- simulated (PyBullet + Blender), not flight footage"


def draw_caption(draw, text, illustrative):
    color = (255, 176, 32, 255) if illustrative else (255, 255, 255, 255)
    prefix = "ILLUSTRATIVE -- " if illustrative and not text.upper().startswith(
        "ILLUSTRATIVE") else ""
    full = prefix + text
    bbox = draw.textbbox((0, 0), full, font=FONT_CAPTION)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (W - tw) / 2.0
    y = H - 140
    pad = 14
    draw.rectangle([x - pad, y - pad, x + tw + pad, y + th + pad],
                   fill=(0, 0, 0, 150))
    draw.text((x, y), full, font=FONT_CAPTION, fill=color)


def main():
    track = json.load(open(TRACK_PATH))
    frames = track["frames"]
    fps = track["fps"]
    hold_frames = int(round(CAPTION_HOLD_S * fps))
    os.makedirs(OUT_DIR, exist_ok=True)

    active_text, active_illustrative, active_until = "", False, -1
    n = len(frames)
    for i, fr in enumerate(frames):
        cap = fr.get("caption") or ""
        if cap:
            active_text = cap
            active_illustrative = "ILLUSTRATIVE" in cap.upper()
            active_until = i + hold_frames

        img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.text((28, H - 44), WATERMARK, font=FONT_WATERMARK,
                  fill=(255, 255, 255, 200))
        if i <= active_until and active_text:
            draw_caption(draw, active_text, active_illustrative)

        img.save(os.path.join(OUT_DIR, f"frame_{i:05d}.png"))
        if i % 100 == 0 or i == n - 1:
            print(f"captions {i + 1}/{n}", flush=True)

    print(f"wrote {n} caption frames to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

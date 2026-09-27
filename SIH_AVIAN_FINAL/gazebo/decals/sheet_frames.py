#!/usr/bin/env python3
"""Contact sheet of one decal test run: the three camera frames side by side with the detector's boxes (from det.json,
written by detect_frames.py) drawn on them. Nothing is re-detected here.

Usage: sheet_frames.py <run_dir> <label> <out.jpg>
"""
import json, sys
from PIL import Image, ImageDraw

run, label, out = sys.argv[1:4]
det = json.load(open(f'{run}/det.json'))
s = Image.new('RGB', (640 * 3 + 8, 480), (20, 20, 20))
for i, d in enumerate((350, 450, 600)):
    k = f'decal_cam_{d}.png'
    im = Image.open(f'{run}/{k}').convert('RGB'); dr = ImageDraw.Draw(im)
    for b in det.get(k, []):
        dr.rectangle(b['bbox'], outline=(255, 0, 0), width=3)
        dr.rectangle([b['bbox'][0], b['bbox'][1], b['bbox'][0] + 150, b['bbox'][1] + 14], fill=(255, 0, 0))
        dr.text((b['bbox'][0] + 3, b['bbox'][1] + 1), f"{b['family']} {b['score']:.3f}", fill=(255, 255, 255))
    dr.rectangle([0, 0, 640, 14], fill=(0, 0, 0))
    dr.text((4, 1), f'{label}  {d / 100:.1f} m   boxes >= 0.65: {len(det.get(k, []))}', fill=(255, 255, 255))
    s.paste(im, (i * 644, 0))
s.save(out, quality=88)

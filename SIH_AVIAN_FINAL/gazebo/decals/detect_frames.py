#!/usr/bin/env python3
"""Run the REAL detector (weights v2, threshold 0.65 - never lowered) on image files; print every box.
Run with the venv:  /home/prince/avian_rev_c/.venv/bin/python3 detect_frames.py img.png [...] [--json out.json]"""
import sys, os, json
ROOT = '/home/prince/avian_rev_c/SIH_AVIAN_FINAL'; sys.path.insert(0, ROOT + '/scene/film')
from detect import load_model, detect
import torch; torch.set_num_threads(int(os.environ.get('TORCH_THREADS', '4')))
args = [x for x in sys.argv[1:] if not x.startswith('--json')]
out = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
if out: args = [x for x in args if x != out]
m = load_model(); res = {}
for p in args:
    d, dt, size = detect(m, p, 0.65)
    res[p] = [{'family': x['family'], 'score': x['score'], 'bbox': x['bbox']} for x in d]
    print(f"{os.path.basename(p):40} {size[0]}x{size[1]} {dt:.2f}s  " + ('NO DETECTION >= 0.65' if not d else ' | '.join(f"{x['family']} {x['score']:.3f} {x['bbox']}" for x in d)))
if out: json.dump(res, open(out, 'w'), indent=1)

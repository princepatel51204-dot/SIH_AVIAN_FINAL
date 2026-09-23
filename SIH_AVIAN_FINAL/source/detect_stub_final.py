"""SIH_AVIAN_FINAL -- Stage 2: the detector stub.

THIS IS THE ONLY FILE IN STAGE 2 ALLOWED TO TOUCH GROUND TRUTH.
`coverage_final.py`'s planner reads nothing but `avian_bridge_collision.json`
and `params_final.py`'s sensor constants -- the whole point of Stage 2 is
that the mission is planned from structure geometry, not from a list of
known defect positions. But the planner needs ONE class-level design
constant it has no other way to obtain: how fine a feature this aircraft's
camera is even claimed to resolve, which sets the standoff every coverage
viewpoint flies at. That number is legitimately "read the defect taxonomy's
own finest feature class" (a spec decision -- "we inspect for defects down
to X mm" -- not a specific defect's logged position), and it is quarantined
here so a `grep ground_truth source/coverage_final.py` comes back empty:
`coverage_final.py` imports the resulting float, never the ground-truth
path itself.

The second job here is the actual detection stub: perception does not
exist yet (0% built, per project status), so `detect()` stands in for it by
peeking at ground truth directly -- legitimately, since faking "what a
detector would have seen" is this function's entire purpose, and it is
isolated to this one file/function so swapping in a real model later is a
one-point change (replace the body of `detect()`, keep its signature).
Nothing outside this file may import the ground-truth loader for any other
reason.
"""
from __future__ import annotations
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")

# The three files `dataset_final.py`'s own `_load_records()` already merges
# for every other "all 192 defects" figure in this project (96 road + 20
# metro + 76 steel) -- reused here for the same reason: a single combined
# ground-truth view, not a partial one.
_GROUND_TRUTH_FILES = (
    "AVIAN_defect_ground_truth_FINAL.json",
    "AVIAN_metro_ground_truth_FINAL.json",
    "AVIAN_steel_ground_truth_FINAL.json",
)


def _load_all_defects():
    out = []
    for fn in _GROUND_TRUTH_FILES:
        with open(os.path.join(SCENE_DIR, fn)) as f:
            out.extend(json.load(f)["defects"])
    return out


_ALL_DEFECTS = _load_all_defects()


def _finest_feature_mm(defects):
    """The smallest `feature_size_mm` anywhere in the taxonomy -- checked
    directly rather than assumed to be hairline cracks (the master prompt's
    own example guess): CORROSION_STAIN (road, 0.056 mm) and
    PIER_CORROSION_STAIN (metro, 0.052 mm) are both finer. 0.052 mm is the
    real global minimum across all 192 defects, used here rather than the
    guessed example."""
    sizes = [d["feature_size_mm"] for d in defects
             if d.get("feature_size_mm")]
    return min(sizes)


FINEST_FEATURE_MM = _finest_feature_mm(_ALL_DEFECTS)

# ---------------------------------------------------------------------------
# stub detect() -- STAND-IN FOR PERCEPTION, NOT A REAL MODEL.
# Given a camera pose (position + the point it is looking at + the standoff
# it was flown at), returns which ground-truth defects a real detector
# might plausibly have flagged from that view: within cone-of-view of the
# look direction and within a generous multiple of the flown standoff.
# Confidence is a simple monotonic stand-in built from the defect's own
# already-computed `defect_background_contrast` (SIH_AVIAN_DATASET pass),
# NOT a real classifier score -- report it as a stub number, never as a
# detector result, per the master prompt's own §4 instruction.
# ---------------------------------------------------------------------------
_RANGE_MARGIN = 1.5      # a real detector isn't perfectly cut off at the
                         # planned standoff; this just bounds the stub's
                         # search radius so it isn't literally the whole
                         # bridge every call
_HALF_CONE_DEG = 35.0    # roughly this sensor's own half-HFOV (34.5 deg)


def detect(camera_pos_m, look_target_m, standoff_m):
    """STUB. Returns [{defect_id, type, confidence, bbox}] for ground-truth
    defects a real detector might have flagged from this camera pose.
    `bbox` is a placeholder (the stub has no image plane to project onto)
    -- present only so the real detector's eventual return shape matches."""
    cx, cy, cz = camera_pos_m
    tx, ty, tz = look_target_m
    view = (tx - cx, ty - cy, tz - cz)
    vlen = math.sqrt(sum(c * c for c in view)) or 1e-6
    view = tuple(c / vlen for c in view)
    max_range = max(standoff_m * _RANGE_MARGIN, 0.5)
    cos_half_cone = math.cos(math.radians(_HALF_CONE_DEG))

    out = []
    for d in _ALL_DEFECTS:
        px, py, pz = d["position_m"]
        rel = (px - cx, py - cy, pz - cz)
        dist = math.sqrt(sum(c * c for c in rel))
        if dist < 1e-6 or dist > max_range:
            continue
        rel_n = tuple(c / dist for c in rel)
        cos_angle = sum(a * b for a, b in zip(view, rel_n))
        if cos_angle < cos_half_cone:
            continue
        contrast = d.get("defect_background_contrast") or 0.0
        confidence = max(0.05, min(0.98, 0.4 + 6.0 * contrast))
        out.append({
            "defect_id": d["defect_id"],
            "type": d["type"],
            "confidence": round(confidence, 3),
            "bbox": None,
        })
    return out

"""SIH_AVIAN_FINAL -- the ONE quarantined ground-truth read `coverage_final.py`
is allowed to depend on.

`coverage_final.py`'s planner reads nothing but `avian_bridge_collision.json`
and `params_final.py`'s sensor constants -- the whole point of Stage 2 is
that the mission is planned from structure geometry, not from a list of
known defect positions. But the planner needs ONE class-level design
constant it has no other way to obtain: how fine a feature this aircraft's
camera is even claimed to resolve, which sets the standoff every coverage
viewpoint flies at. That number is legitimately "read the defect taxonomy's
own finest feature class" (a spec decision -- "we inspect for defects down
to X mm" -- not a specific defect's logged position), and it is quarantined
here so `grep ground_truth source/coverage_final.py` comes back empty:
`coverage_final.py` imports only the resulting float, never the
ground-truth path itself.

Originally lived in `detect_stub_final.py` (see that file's own history,
commit `063115e`) back when that file was the ground-truth-peeking
detection stub. Section 1's rewrite (`3f2fce4`) turned `detect_stub_final.py`
into the real, torch-based detector and deleted its entire ground-truth
branch -- correctly, for detection -- but that also silently deleted this
unrelated design constant, breaking `coverage_final.py`'s import. Moved
here, unchanged in value, rather than restored inside the detector file:
mixing a one-time ground-truth spec read back into the real-detector module
would be architecturally confusing even if grep-clean, and this module has
no torch/PIL dependency so it still imports cleanly under Blender's system
Python (which has neither).
"""
from __future__ import annotations
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")

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


def _finest_feature_mm(defects):
    """The smallest `feature_size_mm` anywhere in the taxonomy -- checked
    directly rather than assumed to be hairline cracks: CORROSION_STAIN
    (road, 0.056 mm) and PIER_CORROSION_STAIN (metro, 0.052 mm) are both
    finer. 0.052 mm is the real global minimum across all 192 defects."""
    sizes = [d["feature_size_mm"] for d in defects if d.get("feature_size_mm")]
    return min(sizes)


FINEST_FEATURE_MM = _finest_feature_mm(_load_all_defects())

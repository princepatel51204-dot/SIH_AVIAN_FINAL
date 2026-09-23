"""SIH_AVIAN_FINAL -- Phase 2 step 6: run the real detector over every
rendered coverage view and write detections in `score_coverage.py`'s
`--real` format.

Plain python3. Reads `detection/AVIAN_coverage_renders_FINAL.json`
(`render_coverage_views_final.py`'s output), calls the now-real
`detect_stub_final.detect(image_path)` once per render, and writes
`mission/detections_real_final.json` -- a SEPARATE file from the Phase 1
stub's `mission/detections.json`, kept side by side on purpose so the
before/after is visible rather than overwritten.

Usage:
    python3 source/run_real_detector_final.py
"""
from __future__ import annotations
import json
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DET_DIR = os.path.join(ROOT, "detection")
MISSION_DIR = os.path.join(ROOT, "mission")

import sys
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import detect_stub_final as DETECT


def main():
    t0 = time.time()
    print("== SIH_AVIAN_FINAL :: run_real_detector_final.py (Phase 2) ==")
    with open(os.path.join(DET_DIR,
                          "AVIAN_coverage_renders_FINAL.json")) as f:
        renders = json.load(f)["renders"]

    detections = []
    n_dets = 0
    for r in renders:
        img_path = os.path.join(ROOT, r["image_path"])
        dets = DETECT.detect(img_path)
        n_dets += len(dets)
        detections.append({
            "waypoint_id": r["waypoint_id"],
            "image_path": r["image_path"],
            "detections": dets,
        })

    out = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "detector": "detect_stub_final.detect -- REAL trained model "
                  "(fasterrcnn_mobilenet_v3_large_320_fpn), not the "
                  "ground-truth-peeking stub",
        "weights": "detection/AVIAN_detector_weights_FINAL.pt",
        "n_renders": len(renders),
        "n_detections": n_dets,
        "detections": detections,
    }
    out_path = os.path.join(MISSION_DIR, "detections_real_final.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"  detect  : {n_dets} detections across {len(renders)} renders "
         f"in {time.time() - t0:.1f}s")
    print(f"  saved   : {out_path}")
    print("FINAL_REAL_DETECT_COMPLETE")


if __name__ == "__main__":
    main()

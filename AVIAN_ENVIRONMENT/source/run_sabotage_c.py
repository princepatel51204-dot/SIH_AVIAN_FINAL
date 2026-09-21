"""Run the REV-C sabotage harness standalone, against the saved scene.

    blender -b --python run_blender.py -- source/run_sabotage_c.py

Separate from the build so the proof can be re-run in a minute without
repeating the 13-minute contrast sweep. It reads the finished ground truth
off disk rather than the Phase A handoff, because the handoff predates the
contrast pass and V38/V39 check fields that the contrast pass adds.
"""
from __future__ import annotations

import json
import os
import sys

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import sabotage_c as SAB          # noqa: E402

SCENE = os.path.join(os.path.dirname(HERE), "scene")


def _load(path):
    with open(path) as f:
        d = json.load(f)
    return d["defects"] if isinstance(d, dict) and "defects" in d else d


def main():
    records = _load(os.path.join(
        SCENE, "AVIAN_defect_ground_truth_REV_C.json"))
    mrecords = _load(os.path.join(
        SCENE, "AVIAN_metro_ground_truth_REV_C.json"))

    print(f"  road  : {len(records)} records x {len(records[0])} fields")
    print(f"  metro : {len(mrecords)} records x {len(mrecords[0])} fields")
    only_road = sorted(set(records[0]) - set(mrecords[0]))
    only_metro = sorted(set(mrecords[0]) - set(records[0]))
    print(f"  fields only on road : {only_road}")
    print(f"  fields only on metro: {only_metro}")

    bpy.ops.wm.open_mainfile(
        filepath=os.path.join(SCENE, "AVIAN_SIC_REV_C.blend"))
    man = os.path.join(SCENE, "collision",
                       "avian_bridge_collision_manifest.json")

    import validate_c as VDC
    print("")
    print("  VALIDATION  (REV-C V33-V43)")
    VDC.run(records, mrecords, log=print, collision_manifest=man)

    res = SAB.run(records, mrecords, man, log=print)
    out = os.path.join(SCENE, "AVIAN_sabotage_REV_C.json")
    with open(out, "w") as f:
        json.dump(res, f, indent=2)
    print(f"  written: {os.path.basename(out)}")
    print("REVC_PHASE_COMPLETE")


if __name__ == "__main__":
    main()

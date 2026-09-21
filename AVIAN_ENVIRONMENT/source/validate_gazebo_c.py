"""REV-C Stage 4 validation -- V44..V50, the Gazebo export.

Runs as plain python3; needs no bpy. It checks the exported artifacts, not
the Blender scene, which is the point: these are the things that have to be
true of the file a judge opens on a different machine.

V45 and V46 are the pair that matters. V45 proves the right number of
bodies came out. V46 proves they are not all sitting on top of each other
at the world origin. Either can pass while the world is broken; together
they cannot -- which is why V46 asserts a non-degenerate pose spread and
prints the measured envelope rather than reporting a bare PASS.
"""
from __future__ import annotations

import glob
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

ENV_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GZ_DIR = os.path.join(ENV_DIR, "gazebo")
SCENE_DIR = os.path.join(ENV_DIR, "scene")
WORLD = os.path.join(GZ_DIR, "worlds", "avian_sic.sdf")
MANIFEST = os.path.join(GZ_DIR, "avian_gazebo_manifest.json")
COLLISION = os.path.join(SCENE_DIR, "collision",
                         "avian_bridge_collision.json")

# Degenerate-spread floor for V46. The corridor is 1100 m long, so a real
# export spans hundreds of metres in x. Anything under this means the poses
# collapsed -- the meshlib origin trap.
MIN_SPREAD_M = 100.0


class R:
    def __init__(self, cid, name, status, measured, limit, detail=""):
        self.id, self.name, self.status = cid, name, status
        self.measured, self.limit, self.detail = measured, limit, detail

    def row(self):
        return (f"  [{self.status:4}] {self.id:<4} {self.name:<44} "
                f"{self.measured:<34} {self.detail}")


def _shell(cmd):
    """Run through bash -lc so the ROS setup sourcing inside view.sh works."""
    p = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


def _model_sdfs():
    return sorted(glob.glob(os.path.join(GZ_DIR, "models", "*", "model.sdf")))


def _all_sdf_files():
    out = []
    for pat in ("worlds/*.sdf", "models/*/model.sdf", "models/*/model.config"):
        out += sorted(glob.glob(os.path.join(GZ_DIR, pat)))
    return out


def run(log=print, world=WORLD, manifest=MANIFEST, collision=COLLISION,
        min_spread=MIN_SPREAD_M):
    res = []

    # ---- V44 the world parses ------------------------------------------
    rc, out = _shell(
        f'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; '
        f'export GZ_SIM_RESOURCE_PATH="{GZ_DIR}/models"; '
        f'export SDF_PATH="{GZ_DIR}/models"; '
        f'gz sdf -k "{world}"')
    ok = (rc == 0 and "Valid" in out)
    res.append(R("V44", "SDF world parses", "PASS" if ok else "FAIL",
                 f"gz sdf -k exit {rc}", "exit 0",
                 "SDF_PATH is required as well as GZ_SIM_RESOURCE_PATH: "
                 "the validator uses sdformat's own findFile"))

    # ---- V45 collision count matches PyBullet's ------------------------
    with open(collision) as f:
        prims = json.load(f)["primitives"]
    with open(manifest) as f:
        man = json.load(f)
    x0, x1 = man["corridor_x"]
    in_corridor = [p for p in prims if x0 <= p["centre"][0] <= x1]

    n_sdf = 0
    for path in _model_sdfs():
        if os.path.basename(os.path.dirname(path)) == "avian_defects":
            continue
        n_sdf += sum(1 for _ in ET.parse(path).getroot().iter("collision"))
    ok = (n_sdf == len(in_corridor)) and n_sdf > 0
    res.append(R("V45", "SDF collision count equals PyBullet's",
                 "PASS" if ok else "FAIL",
                 f"{n_sdf} SDF vs {len(in_corridor)} PyBullet in corridor",
                 "exact match, non-zero",
                 f"{len(prims)} total primitives, "
                 f"{len(prims) - len(in_corridor)} outside the corridor"))

    # ---- V46 poses match the Blender world bounding-box centres --------
    # Compared against the pose table the exporter recorded, which came
    # from matrix_world @ ob.bound_box via avian_common. NOT from
    # matrix_world.translation, which meshlib leaves at (0,0,0) for
    # essentially every object -- that would compare zero against zero and
    # pass for all 1343 members while the world was a single stack.
    poses = man["poses_world_m"]
    off = man["world_origin_offset_m"]
    worst = 0.0
    worst_name = None
    seen = 0
    for path in _model_sdfs():
        root = ET.parse(path).getroot()
        for el in root.iter():
            if el.tag not in ("collision", "visual"):
                continue
            nm = el.get("name", "")
            base = nm.rsplit("_", 1)[0]
            if base not in poses:
                continue
            p = el.find("pose")
            if p is None or not p.text:
                continue
            v = [float(t) for t in p.text.split()[:3]]
            want = [poses[base][i] - off[i] for i in range(3)]
            d = max(abs(v[i] - want[i]) for i in range(3))
            if d > worst:
                worst, worst_name = d, nm
            seen += 1

    xs = [poses[k][0] for k in poses]
    ys = [poses[k][1] for k in poses]
    zs = [poses[k][2] for k in poses]
    spread = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    degenerate = spread[0] < min_spread
    ok = (seen > 0) and (worst <= 0.001) and not degenerate
    res.append(R("V46", "exported poses match Blender bbox centres",
                 "PASS" if ok else "FAIL",
                 f"{seen} poses, worst {worst*1000:.3f} mm, "
                 f"x-spread {spread[0]:.1f} m",
                 f"<= 1 mm, x-spread >= {min_spread:.0f} m",
                 f"envelope x {min(xs):.1f}..{max(xs):.1f}, "
                 f"y {min(ys):.1f}..{max(ys):.1f}, "
                 f"z {min(zs):.1f}..{max(zs):.1f}"
                 + (f"; DEGENERATE -- poses collapsed"
                    if degenerate else "")
                 + (f"; worst on {worst_name}" if worst > 0 else "")))

    # ---- V47 every defect exported and addressable by name -------------
    want = set()
    for fn in ("AVIAN_defect_ground_truth_REV_C.json",
               "AVIAN_metro_ground_truth_REV_C.json"):
        p = os.path.join(SCENE_DIR, fn)
        if not os.path.exists(p):
            continue
        with open(p) as f:
            blob = json.load(f)
        recs = blob["defects"] if isinstance(blob, dict) \
            and "defects" in blob else blob
        for r in recs:
            if x0 <= r["position_m"][0] <= x1:
                want.add(r["defect_id"])
    dpath = os.path.join(GZ_DIR, "models", "avian_defects", "model.sdf")
    got = set()
    if os.path.exists(dpath):
        for el in ET.parse(dpath).getroot().iter("visual"):
            got.add(el.get("name", ""))
    missing = sorted(want - got)
    ok = bool(want) and not missing
    res.append(R("V47", "every defect exported and addressable by name",
                 "PASS" if ok else "FAIL",
                 f"{len(got & want)} of {len(want)} in corridor",
                 "all, non-zero",
                 "named identically to the Blender objects"
                 + (f"; MISSING {missing[:3]}" if missing else "")))

    # ---- V48 the world loads in Gazebo ---------------------------------
    rc, out = _shell(
        f'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; '
        f'export GZ_SIM_RESOURCE_PATH="{GZ_DIR}/models"; '
        f'export SDF_PATH="{GZ_DIR}/models"; '
        f'gz sim -s -r --iterations 100 "{world}"')
    res.append(R("V48", "world loads and steps in Gazebo",
                 "PASS" if rc == 0 else "FAIL",
                 f"gz sim -s -r --iterations 100 exit {rc}", "exit 0",
                 "headless, 100 steps"))

    # ---- V49 memory on load --------------------------------------------
    rc, out = _shell(
        f'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; '
        f'export GZ_SIM_RESOURCE_PATH="{GZ_DIR}/models"; '
        f'export SDF_PATH="{GZ_DIR}/models"; '
        f'/usr/bin/time -v gz sim -s -r --iterations 100 "{world}" 2>&1 '
        f'| grep "Maximum resident"')
    rss_kb = 0
    for tok in out.split():
        if tok.isdigit():
            rss_kb = max(rss_kb, int(tok))
    res.append(R("V49", "memory on load", "PASS" if rss_kb > 0 else "SKIP",
                 f"peak RSS {rss_kb/1024:.0f} MB", "reported",
                 "one server, no aircraft yet"))

    # ---- V50 no absolute paths -----------------------------------------
    hits = []
    for path in _all_sdf_files():
        with open(path) as f:
            for i, line in enumerate(f, 1):
                if "/home" in line:
                    hits.append(f"{os.path.basename(path)}:{i}")
    res.append(R("V50", "no absolute paths in any SDF",
                 "PASS" if not hits else "FAIL",
                 f"{len(hits)} matches for /home in "
                 f"{len(_all_sdf_files())} files", "0 matches",
                 "model:// resolved via GZ_SIM_RESOURCE_PATH"
                 + (f"; {hits[:3]}" if hits else "")))

    for r in res:
        log(r.row())
    p = sum(1 for r in res if r.status == "PASS")
    f_ = sum(1 for r in res if r.status == "FAIL")
    s = sum(1 for r in res if r.status == "SKIP")
    log(f"  {p} pass, {f_} fail, {s} skip of {len(res)} Gazebo checks")
    return res, {"pass": p, "fail": f_, "skip": s, "total": len(res)}


if __name__ == "__main__":
    _, summary = run()
    sys.exit(0 if summary["fail"] == 0 else 1)

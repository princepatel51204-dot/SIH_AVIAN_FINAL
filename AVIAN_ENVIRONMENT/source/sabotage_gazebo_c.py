"""Prove V44..V50 can fail. Working rule 3.10, applied to Stage 4.

Each sabotage writes a deliberately broken copy of an artifact into a temp
tree, points the check at it, and confirms the check goes red. Nothing in
gazebo/ is modified.

V46 gets the sabotage that matters: every pose collapsed to the world
origin, which is precisely what reaching for matrix_world.translation
instead of the bounding-box centre would produce. That world parses, loads
and steps -- V44 and V48 stay green through it -- so V46 is the only thing
standing between a stacked export and a green gate.
"""
from __future__ import annotations

import copy
import json
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET

import validate_gazebo_c as VG


def _status(res, cid):
    for r in res:
        if r.id == cid:
            return r.status
    return None


def _tree():
    """A throwaway copy of gazebo/ plus the paths the checks read."""
    tmp = tempfile.mkdtemp(prefix="avian_sab_gz_")
    shutil.copytree(VG.GZ_DIR, os.path.join(tmp, "gazebo"),
                    ignore=shutil.ignore_patterns(".shoot", "screenshots"))
    return tmp


def _run_against(tmp, collision=None):
    gz = os.path.join(tmp, "gazebo")
    old_gz, VG.GZ_DIR = VG.GZ_DIR, gz
    try:
        return VG.run(log=lambda *a: None,
                      world=os.path.join(gz, "worlds", "avian_sic.sdf"),
                      manifest=os.path.join(gz,
                                            "avian_gazebo_manifest.json"),
                      collision=collision or VG.COLLISION)[0]
    finally:
        VG.GZ_DIR = old_gz


def sab_v44(tmp):
    """Corrupt the world XML."""
    p = os.path.join(tmp, "gazebo", "worlds", "avian_sic.sdf")
    with open(p) as f:
        s = f.read()
    with open(p, "w") as f:
        f.write(s.replace("</world>", "<world>"))


def sab_v45(tmp):
    """Drop a model's collisions so the count no longer matches."""
    p = os.path.join(tmp, "gazebo", "models", "avian_metro", "model.sdf")
    root = ET.parse(p)
    r = root.getroot()
    for parent in r.iter("link"):
        for el in list(parent):
            if el.tag == "collision":
                parent.remove(el)
    root.write(p)


def sab_v46(tmp):
    """Collapse every pose to the world origin -- the meshlib origin trap."""
    for name in ("avian_bridge", "avian_metro", "avian_terrain"):
        p = os.path.join(tmp, "gazebo", "models", name, "model.sdf")
        if not os.path.exists(p):
            continue
        tree = ET.parse(p)
        for el in tree.getroot().iter():
            if el.tag in ("collision", "visual"):
                pose = el.find("pose")
                if pose is not None:
                    pose.text = "0 0 0 0 0 0"
        tree.write(p)


def sab_v47(tmp):
    """Remove a defect marker."""
    p = os.path.join(tmp, "gazebo", "models", "avian_defects", "model.sdf")
    tree = ET.parse(p)
    for link in tree.getroot().iter("link"):
        vis = [e for e in link if e.tag == "visual"]
        if vis:
            link.remove(vis[0])
            break
    tree.write(p)


def sab_v50(tmp):
    """Put an absolute path into a model SDF."""
    p = os.path.join(tmp, "gazebo", "models", "avian_bridge", "model.sdf")
    with open(p) as f:
        s = f.read()
    with open(p, "w") as f:
        f.write(s.replace("<static>true</static>",
                          "<static>true</static>\n"
                          "<!-- /home/prince/somewhere/absolute.dae -->", 1))


SABOTAGES = [
    ("V44", "corrupt the world XML", sab_v44),
    ("V45", "drop one model's collision geometry", sab_v45),
    ("V46", "collapse every pose to the world origin", sab_v46),
    ("V47", "remove a defect marker", sab_v47),
    ("V50", "put an absolute /home path in a model SDF", sab_v50),
]


def run(log=print):
    log("")
    log("  SABOTAGE  (working rule 3.10 -- Gazebo checks observed failing)")
    base = VG.run(log=lambda *a: None)[0]
    rows = []
    for cid, what, fn in SABOTAGES:
        before = _status(base, cid)
        tmp = _tree()
        try:
            fn(tmp)
            after = _status(_run_against(tmp), cid)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        good = (before == "PASS" and after == "FAIL")
        rows.append((cid, what, before, after, good))
        log(f"  [{'OK  ' if good else 'BAD '}] {cid}  {before} -> {after}"
            f"   sabotage: {what}")

    # V48/V49 are not sabotage-tested here: breaking them means breaking
    # the Gazebo install or the machine's memory, not an artifact. V48 has
    # been observed failing for real -- it is what reported exit 255 before
    # SDF_PATH was set. Stated rather than quietly skipped.
    log("  V48 observed failing for real during Stage 4 (exit 255 before "
        "SDF_PATH was set); V49 reports a measurement, not a limit")

    bad = [r for r in rows if not r[4]]
    log(f"  {len(rows) - len(bad)}/{len(rows)} Gazebo checks observed "
        f"failing under sabotage")
    if bad:
        log(f"  UNPROVEN: {', '.join(r[0] for r in bad)}")
    return {"total": len(rows), "proven": len(rows) - len(bad),
            "unproven": [r[0] for r in bad],
            "rows": [{"check": c, "sabotage": w, "before": b, "after": a,
                      "proven": g} for c, w, b, a, g in rows]}


if __name__ == "__main__":
    res = run()
    out = os.path.join(VG.SCENE_DIR, "AVIAN_sabotage_gazebo_REV_C.json")
    with open(out, "w") as f:
        json.dump(res, f, indent=2)
    print(f"  written: {os.path.basename(out)}")

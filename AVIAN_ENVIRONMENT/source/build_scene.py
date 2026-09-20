"""AVIAN Smart Infrastructure Inspection City -- master build script.

    python build_scene.py                 build, validate, save .blend
    python build_scene.py --render        also render the 9 validation views
    python build_scene.py --samples 48    override render samples

PHASE BOUNDARY
--------------
This script builds an ENVIRONMENT. It does not build a UAV, a flight
controller, a perception stack, a planner or a manipulator, and it does not
start a simulation. The UAV-related content here is airspace *definition*:
named volumes and markers a later system can read. Nothing in this file flies.

Runs headless against bpy as a module -- no Blender GUI required.
"""
from __future__ import annotations
import os
import sys
import time
import json

import bpy

import params as P
import materials as M
import meshlib as ML
import bridge as B
import terrain as TR
import roads as RD
import city as CT
import lighting as LT
import damage as DM
import zones as ZN
import validate as VD

OUT = os.path.dirname(os.path.abspath(__file__))
BLEND = os.path.join(OUT, "AVIAN_Smart_Infrastructure_City.blend")


def _log_to(lines):
    def log(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s, flush=True)
    return log


def make_collections(scene):
    """The collection tree. Documented in the README; validated in V20."""
    root = bpy.data.collections.new("AVIAN_ENVIRONMENT")
    scene.collection.children.link(root)
    colls = {}

    def group(parent, name, children=()):
        c = bpy.data.collections.new(name)
        parent.children.link(c)
        colls[name] = c
        for ch in children:
            cc = bpy.data.collections.new(ch)
            c.children.link(cc)
            colls[ch] = cc
        return c

    group(root, "BRIDGE", ("DECK", "PIERS", "BEAMS", "JOINTS",
                           "BARRIERS", "DETAILS"))
    group(root, "RIVER")
    group(root, "ROADS")
    group(root, "CITY", ("BUILDINGS", "VEHICLES", "STREET_LIGHTS",
                         "VEGETATION"))
    group(root, "STRUCTURAL_DAMAGE", ("CRACKS", "SPALLING", "REBAR",
                                      "CORROSION", "JOINT_DAMAGE"))
    group(root, "UAV_AIRSPACE")
    group(root, "INSPECTION_SECTORS")
    group(root, "INSPECTION_SCENARIOS")
    group(root, "CAMERAS")
    group(root, "LIGHTING_ENVIRONMENT")

    # aliases used by the zone builder
    colls["SECTORS"] = colls["INSPECTION_SECTORS"]
    colls["SCENARIOS"] = colls["INSPECTION_SCENARIOS"]
    colls["ROOT"] = root
    return root, colls


def build(log=print):
    t0 = time.time()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"

    root, colls = make_collections(scene)
    stats = {}

    log("BUILD")
    mats = M.build_library()
    log(f"  materials: {len(mats)} procedural material definitions")

    t = time.time()
    stats["bridge"] = B.build(colls, mats, log)
    log(f"           ({time.time()-t:.1f} s)")

    t = time.time()
    stats["terrain"] = TR.build(colls, mats, log)
    stats["roads"] = RD.build(colls, mats, log)
    stats["city"] = CT.build(colls, mats, log)
    log(f"           ({time.time()-t:.1f} s)")

    t = time.time()
    records, counts, n_dobj = DM.build(colls, mats, log)
    stats["damage"] = {"defects": len(records), "objects": n_dobj,
                       "by_type": counts}
    log(f"  damage  : {len(records)} defects in {n_dobj} objects "
        f"({time.time()-t:.1f} s)")

    LT.build(colls["LIGHTING_ENVIRONMENT"], log)
    stats["zones"] = ZN.build(colls, records, log)

    scene.camera = bpy.data.objects.get("CAMERA_04_RESEARCH_ZONE")
    LT.configure_render()

    log(f"  total   : {time.time()-t0:.1f} s, "
        f"{len(bpy.data.objects)} objects")
    return scene, colls, records, stats


def main():
    argv = sys.argv[1:]
    do_render = "--render" in argv
    samples = None
    if "--samples" in argv:
        samples = int(argv[argv.index("--samples") + 1])

    lines = []
    log = _log_to(lines)

    scene, colls, records, stats = build(log)

    # ---- exports ---------------------------------------------------------
    gt = DM.export_ground_truth(
        records,
        os.path.join(OUT, "AVIAN_defect_ground_truth.json"),
        os.path.join(OUT, "AVIAN_defect_ground_truth.csv"))
    zm = ZN.export_zone_manifest(
        os.path.join(OUT, "AVIAN_zone_manifest.json"))
    log(f"  export  : ground truth {gt['total_defects']} defects; "
        f"manifest {zm}")

    # ---- validation ------------------------------------------------------
    results, summary = VD.run(records, log)
    VD.export(results, os.path.join(OUT, "AVIAN_validation_report.json"))

    # ---- scene statistics ------------------------------------------------
    tris = sum(sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
               for o in bpy.data.objects
               if o.type == "MESH" and not o.hide_render)
    uniq = len(set(o.data.name for o in bpy.data.objects
                   if o.type == "MESH"))
    stats["scene"] = {
        "objects": len(bpy.data.objects),
        "unique_meshes": uniq,
        "triangles_rendered": tris,
        "materials": len(bpy.data.materials),
        "collections": len(bpy.data.collections),
        "validation": summary,
    }
    with open(os.path.join(OUT, "AVIAN_scene_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)

    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    mb = os.path.getsize(BLEND) / 1e6
    log(f"  saved   : {os.path.basename(BLEND)} ({mb:.1f} MB)")

    with open(os.path.join(OUT, "AVIAN_build_log.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")

    if do_render:
        import renders
        renders.run(scene, records, samples=samples, log=log)


if __name__ == "__main__":
    main()

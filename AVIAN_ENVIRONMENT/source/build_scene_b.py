"""AVIAN REV-B -- master build for the simulation-ready environment.

    python build_scene_b.py                 build, validate, export, save
    python build_scene_b.py --render        also render validation views
    python build_scene_b.py --scenario NIGHT

WHAT THIS IS
------------
An UPGRADE of the REV-A environment, not a rebuild. The REV-A modules
(params, materials, meshlib, bridge, terrain, roads, city, lighting, damage,
zones, validate) are imported and run UNCHANGED. REV-B adds modules alongside
them: sensors, mission, visibility, scenarios, dynamics, detail_b, zones_b,
cameras_b, dataset, validate_b.

The 192 defects are regenerated from the same seed by the same code, and check
V22 compares every ID and coordinate against the frozen REV-A baseline on disk.
If REV-B disturbed the ground truth, the build says so by name.

PHASE: SIMULATION-READY ENVIRONMENT.
No flight controller, no ROS, no PX4, no SLAM, no planner, no detector, no
manipulator. Interfaces and metadata for all of them; none of them implemented.
"""
from __future__ import annotations
import json
import os
import sys
import time

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

import sensors as SN
import mission as MI
import visibility as VIS
import scenarios as SC
import dynamics as DY
import detail_b as DET
import zones_b as ZB
import cameras_b as CB
import dataset as DS
import validate_b as VDB

OUT = os.path.dirname(os.path.abspath(__file__))
BLEND = os.path.join(OUT, "AVIAN_Smart_Infrastructure_City_REV_B.blend")


def _log_to(lines):
    def log(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s, flush=True)
    return log


def make_collections(scene):
    """The REV-B collection tree.

    REV-A's names are all preserved. The digital-twin groupings the revision
    asks for are added as ALIASES onto the same collections rather than as a
    second parallel hierarchy -- an object in two hierarchies is an object
    that gets exported twice and counted twice.
    """
    root = bpy.data.collections.new("AVIAN_WORLD")
    scene.collection.children.link(root)
    colls = {"ROOT": root}

    def group(parent, name, children=()):
        c = bpy.data.collections.new(name)
        parent.children.link(c)
        colls[name] = c
        for ch in children:
            cc = bpy.data.collections.new(ch)
            c.children.link(cc)
            colls[ch] = cc
        return c

    group(root, "AVIAN_BRIDGE", ("DECK", "PIERS", "BEAMS", "JOINTS",
                                 "BARRIERS", "DETAILS"))
    group(root, "AVIAN_RIVER")
    group(root, "AVIAN_ROADS")
    group(root, "AVIAN_CITY", ("BUILDINGS", "VEHICLES", "STREET_LIGHTS",
                               "VEGETATION"))
    group(root, "AVIAN_DEFECTS", ("CRACKS", "SPALLING", "REBAR",
                                  "CORROSION", "JOINT_DAMAGE"))
    group(root, "AVIAN_AIRSPACE", ("UAV_AIRSPACE", "UAV_AIRSPACE_REV_B",
                                   "INSPECTION_SECTORS"))
    group(root, "AVIAN_MISSION", ("MISSION_MARKERS",))
    group(root, "AVIAN_SENSORS")
    group(root, "AVIAN_SCENARIOS", ("INSPECTION_SCENARIOS",))
    group(root, "AVIAN_DYNAMIC", ("AVI_DYNAMIC_VEHICLES",
                                  "AVI_DYNAMIC_BOATS",
                                  "AVI_DYNAMIC_PEDESTRIANS",
                                  "AVI_DYNAMIC_OBSTACLES"))
    group(root, "CAMERAS")
    group(root, "LIGHTING_ENVIRONMENT")

    # REV-A module aliases -> the same collection objects
    colls["RIVER"] = colls["AVIAN_RIVER"]
    colls["ROADS"] = colls["AVIAN_ROADS"]
    colls["SECTORS"] = colls["INSPECTION_SECTORS"]
    colls["SCENARIOS"] = colls["INSPECTION_SCENARIOS"]
    colls["MISSION"] = colls["MISSION_MARKERS"]
    colls["SENSORS"] = colls["AVIAN_SENSORS"]
    # REV-A city vehicles live in the dynamic collection now, so traffic can
    # be switched off without touching the buildings around it
    colls["VEHICLES"] = colls["AVI_DYNAMIC_VEHICLES"]
    return root, colls


def build(scenario="BASELINE", log=print):
    t0 = time.time()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"

    root, colls = make_collections(scene)
    stats = {}

    log("BUILD  (REV-B, upgrade of REV-A -- geometry generators unchanged)")
    mats = M.build_library()
    log(f"  materials: {len(mats)} procedural material definitions")

    # ---- PRESERVED REV-A GENERATORS, RUN UNCHANGED ----------------------
    t = time.time()
    stats["bridge"] = B.build(colls, mats, log)
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
    stats["zones_rev_a"] = ZN.build(colls, records, log)

    # ---- REV-B ADDITIONS ------------------------------------------------
    log("REV-B")
    stats["detail"] = DET.build(colls, mats, log)
    stats["dynamics"] = DY.build(colls, mats, log)
    stats["airspace_rev_b"] = ZB.build_airspace_b(
        colls["UAV_AIRSPACE_REV_B"], log)
    stats["scenarios_rev_b"] = len(
        ZB.build_scenarios_b(colls["INSPECTION_SCENARIOS"], log))
    sens = SN.build(colls["AVIAN_SENSORS"], log)
    stats["sensors"] = len(sens["frames"])

    bpy.context.view_layer.update()
    stats["base_sites_cleared"] = MI.prepare_base_sites(log)
    bpy.context.view_layer.update()
    mk = MI.build(colls["MISSION_MARKERS"], log)
    graph = MI.build_graph(mk["nodes"], log)
    stats["mission"] = {"markers": sum(mk["markers"].values()),
                        "nodes": len(graph["nodes"]),
                        "edges": len(graph["edges"])}

    stats["cameras_rev_b"] = len(CB.build_extra(colls["CAMERAS"],
                                                records, log))

    # ---- measured visibility ground truth --------------------------------
    t = time.time()
    stats["visibility"] = VIS.compute(records, log=log)
    stats["visibility"]["correlation_with_authored_prior"] = \
        VIS.correlate_with_prior(records)
    log(f"           ({time.time()-t:.1f} s)")

    # ---- dataset readiness ----------------------------------------------
    stats["indices"] = DS.assign_indices(records, log)
    passes = DS.enable_passes(log=log)

    # ---- scenario --------------------------------------------------------
    LT.configure_render()
    cfg = SC.apply(scenario, log=log)
    scene.camera = bpy.data.objects.get("CAMERA_04_RESEARCH_ZONE")

    log(f"  total   : {time.time()-t0:.1f} s, "
        f"{len(bpy.data.objects)} objects")
    return scene, colls, records, graph, cfg, passes, stats


def main():
    argv = sys.argv[1:]
    do_render = "--render" in argv
    scenario = "BASELINE"
    if "--scenario" in argv:
        scenario = argv[argv.index("--scenario") + 1]
    samples = None
    if "--samples" in argv:
        samples = int(argv[argv.index("--samples") + 1])

    lines = []
    log = _log_to(lines)

    scene, colls, records, graph, cfg, passes, stats = build(scenario, log)

    # ---- exports ---------------------------------------------------------
    gt = DM.export_ground_truth(
        records,
        os.path.join(OUT, "AVIAN_defect_ground_truth_REV_B.json"),
        os.path.join(OUT, "AVIAN_defect_ground_truth_REV_B.csv"))
    zm = ZB.export_manifest(
        os.path.join(OUT, "AVIAN_zone_manifest_REV_B.json"))
    mm = MI.export_manifest(
        graph, stats["mission"]["markers"] if isinstance(
            stats["mission"]["markers"], dict) else {},
        os.path.join(OUT, "AVIAN_mission_manifest_REV_B.json"))
    sm = SN.export_manifest(
        os.path.join(OUT, "AVIAN_sensor_manifest_REV_B.json"))
    cm = SC.export_manifest(
        os.path.join(OUT, "AVIAN_scenario_manifest_REV_B.json"))
    dm = DS.export_manifest(
        records, os.path.join(OUT, "AVIAN_dataset_manifest_REV_B.json"),
        stats["indices"], passes)
    log(f"  export  : ground truth {gt['total_defects']}; zones {zm}; "
        f"mission {mm}; sensors {sm}; scenarios {cm}; dataset {dm}")

    # ---- validation ------------------------------------------------------
    log("")
    log("  VALIDATION  (REV-A V01-V21, then REV-B V22-V32)")
    res_a, sum_a = VD.run(records, log)
    res_b, sum_b = VDB.run(records, log, mission_graph=graph,
                           scenario_cfg=cfg)
    allres = res_a + res_b
    summary = {"pass": sum_a["pass"] + sum_b["pass"],
               "fail": sum_a["fail"] + sum_b["fail"],
               "skip": sum_a["skip"] + sum_b["skip"],
               "total": len(allres)}
    log(f"  {summary['pass']} pass, {summary['fail']} fail, "
        f"{summary['skip']} skip of {summary['total']} checks")
    VD.export(allres,
              os.path.join(OUT, "AVIAN_validation_report_REV_B.json"))

    # ---- scene statistics ------------------------------------------------
    tris = sum(sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
               for o in bpy.data.objects
               if o.type == "MESH" and not o.hide_render)
    uniq = len(set(o.data.name for o in bpy.data.objects
                   if o.type == "MESH"))
    stats["scene"] = {
        "revision": "REV_B",
        "objects": len(bpy.data.objects),
        "unique_meshes": uniq,
        "triangles_rendered": tris,
        "materials": len(bpy.data.materials),
        "collections": len(bpy.data.collections),
        "cameras": sum(1 for o in bpy.data.objects if o.type == "CAMERA"),
        "active_scenario": cfg,
        "validation": summary,
    }
    with open(os.path.join(OUT, "AVIAN_scene_stats_REV_B.json"), "w") as f:
        json.dump(stats, f, indent=2)

    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    mb = os.path.getsize(BLEND) / 1e6
    log(f"  saved   : {os.path.basename(BLEND)} ({mb:.1f} MB)")

    with open(os.path.join(OUT, "AVIAN_build_log_REV_B.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")

    if do_render:
        import renders
        renders.run(scene, records, samples=samples, log=log)


if __name__ == "__main__":
    main()

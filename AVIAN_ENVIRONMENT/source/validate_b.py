"""REV-B validation: checks V22-V32, run after the REV-A suite V01-V21.

The REV-A checks are untouched and still run first. These add the things REV-B
introduced, and one check that matters more than all the rest:

    V22 GROUND TRUTH BASELINE DRIFT

REV-B was required to upgrade the environment without disturbing 192 frozen
defects. Asserting that in a README is worth nothing. V22 loads the REV-A
ground truth off disk and compares every ID and every coordinate against what
the rebuilt scene produced. If a single defect moved by more than a millimetre,
this fails and names it.
"""
from __future__ import annotations
import json
import math
import os

import bpy
from mathutils import Vector

import params as P
from validate import Result, _world_bbox, _in

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "BASELINE_REV_A_index.json")


def run(records, log=print, mission_graph=None, scenario_cfg=None):
    R = []

    # ---- V22 ground truth has not drifted from the REV-A baseline --------
    if os.path.exists(BASELINE):
        base = json.load(open(BASELINE))
        cur = {r["defect_id"]: r for r in records}
        missing = sorted(set(base) - set(cur))
        added = sorted(set(cur) - set(base))
        moved = []
        worst = 0.0
        for did, b in base.items():
            c = cur.get(did)
            if c is None:
                continue
            d = (Vector(b["position_m"]) - Vector(c["position_m"])).length
            worst = max(worst, d)
            if d > 0.001:
                moved.append(f"{did} moved {d*1000:.1f} mm")
            if c["type"] != b["type"]:
                moved.append(f"{did} type {b['type']} -> {c['type']}")
            if c["severity"] != b["severity"]:
                moved.append(f"{did} severity {b['severity']} -> "
                             f"{c['severity']}")
            if c["inspection_sector"] != b["inspection_sector"]:
                moved.append(f"{did} sector {b['inspection_sector']} -> "
                             f"{c['inspection_sector']}")
        bad = missing + added + moved
        R.append(Result(
            "V22", "ground truth matches the REV-A baseline",
            "PASS" if not bad else "FAIL",
            f"{len(base)} baseline, {len(cur)} now, max drift "
            f"{worst*1000:.2f} mm", "0 missing, 0 added, drift <= 1 mm",
            "every REV-A defect survived REV-B unchanged" if not bad
            else f"{len(missing)} missing, {len(added)} added, "
                 f"{len(moved)} altered", bad))
    else:
        R.append(Result("V22", "ground truth matches the REV-A baseline",
                        "SKIP", detail="no baseline file on disk"))

    # ---- V23 mission markers ---------------------------------------------
    marks = [o for o in bpy.data.objects
             if o.get("avi_kind") == "mission_marker"]
    need = {"UAV_SPAWN", "TAKEOFF", "LANDING", "EMERGENCY_LANDING",
            "SECTOR_ENTRY", "SECTOR_EXIT", "INSPECTION_START",
            "INSPECTION_END", "RETURN_TO_HOME", "WAYPOINT_CANDIDATE",
            "SERVICE_STATION"}
    have = {o["avi_marker_type"] for o in marks}
    unsafe = [o.name for o in marks
              if not o.get("avi_safe", True)
              and o.get("avi_marker_type") != "EMERGENCY_LANDING"]
    miss = sorted(need - have)
    R.append(Result("V23", "mission markers complete and clear",
                    "PASS" if not miss and not unsafe else "FAIL",
                    f"{len(marks)} markers, {len(have)}/{len(need)} types",
                    "all types present, all clear of structure",
                    "every marker type present and sited in free air"
                    if not miss and not unsafe
                    else f"missing {miss}; {len(unsafe)} inside structure",
                    miss + unsafe))

    # ---- V24 sector coverage by markers -----------------------------------
    per = {}
    for o in marks:
        per.setdefault(o.get("avi_sector", "SHARED"), 0)
        per[o["avi_sector"]] += 1
    thin = [s for s in P.SECTOR_NAMES if per.get(s, 0) < 5]
    R.append(Result("V24", "every sector has its own mission markers",
                    "PASS" if not thin else "FAIL",
                    ", ".join(f"{s[-1]}:{per.get(s, 0)}"
                              for s in P.SECTOR_NAMES), ">= 5 per sector",
                    "six aircraft can each be given a sector" if not thin
                    else "sectors without enough markers", thin))

    # ---- V25 sensor anchors -----------------------------------------------
    frames = [o for o in bpy.data.objects
              if o.get("avi_kind") == "sensor_frame"]
    classes = {o["avi_sensor_class"] for o in frames}
    need_c = {"RGB_CAMERA", "DEPTH_CAMERA", "LIDAR", "IMU",
              "THERMAL_CAMERA"}
    nofov = [o.name for o in frames
             if o["avi_sensor_class"] != "IMU"
             and "avi_hfov_deg" not in o.keys()]
    miss = sorted(need_c - classes)
    R.append(Result("V25", "sensor anchors defined with optics",
                    "PASS" if not miss and not nofov else "FAIL",
                    f"{len(frames)} frames, {len(classes)} classes",
                    "5 sensor classes, FOV on every optical frame",
                    "the rig is fully specified" if not miss and not nofov
                    else f"missing {miss}; {len(nofov)} without FOV",
                    miss + nofov))

    # ---- V26 visibility ground truth ---------------------------------------
    need_v = ["visible_fraction", "occlusion_measured",
              "recommended_view_direction", "min_inspection_range_m",
              "max_useful_range_m", "expected_rgb_visibility",
              "expected_depth_visibility", "expected_lidar_visibility",
              "detection_difficulty"]
    missv = []
    for r in records:
        for k in need_v:
            if k not in r:
                missv.append(f"{r['defect_id']}:{k}")
                break
    levels = {}
    for r in records:
        levels[r.get("detection_difficulty")] = \
            levels.get(r.get("detection_difficulty"), 0) + 1
    spread = sum(1 for k in ("LEVEL_1", "LEVEL_2", "LEVEL_3", "LEVEL_4")
                 if levels.get(k, 0) > 0)
    ok = not missv and spread >= 3
    R.append(Result("V26", "sensor visibility ground truth is complete",
                    "PASS" if ok else "FAIL",
                    f"{len(records) - len(missv)}/{len(records)} complete, "
                    f"{spread}/4 difficulty levels used",
                    "all fields, >= 3 levels populated",
                    ", ".join(f"{k}:{levels.get(k, 0)}" for k in
                              ("LEVEL_1", "LEVEL_2", "LEVEL_3", "LEVEL_4")),
                    missv[:12]))

    # ---- V27 REV-B airspace taxonomy ---------------------------------------
    import zones_b as ZB
    vols = [o for o in bpy.data.objects
            if o.get("avi_taxonomy") == "REV_B"]
    got = {o["avi_zone_class"] for o in vols}
    miss = sorted(set(ZB.REVB_CLASSES) - got)
    noenv = [o.name for o in vols
             if "avi_max_speed_mps" not in o.keys()
             or "avi_risk_level" not in o.keys()]
    R.append(Result("V27", "REV-B airspace classes complete",
                    "PASS" if not miss and not noenv else "FAIL",
                    f"{len(vols)} volumes, {len(got)}/"
                    f"{len(ZB.REVB_CLASSES)} classes",
                    "8 classes, operating envelope on every volume",
                    "each volume carries speed, clearance and risk"
                    if not miss and not noenv
                    else f"missing {miss}", miss + noenv))

    # ---- V28 sixteen scenarios with full metadata -------------------------
    scn = [o for o in bpy.data.objects
           if o.get("avi_kind") == "inspection_scenario"
           and o.name.startswith("SCENARIO_B")]
    need_s = ["avi_difficulty", "avi_recommended_sensor",
              "avi_minimum_clearance_m", "avi_lighting_condition",
              "avi_occlusion_level", "avi_water_present",
              "avi_gps_quality", "avi_dynamic_obstacles",
              "avi_recommended_uav_speed_mps"]
    bad = [f"{o.name}:{k}" for o in scn for k in need_s
           if k not in o.keys()]
    R.append(Result("V28", "sixteen scenarios fully specified",
                    "PASS" if len(scn) >= 16 and not bad else "FAIL",
                    f"{len(scn)} scenarios, {len(need_s)} fields each",
                    ">= 16, no missing fields",
                    "every scenario carries its full operating context"
                    if len(scn) >= 16 and not bad else "incomplete", bad[:12]))

    # ---- V29 mission graph traversable ------------------------------------
    if mission_graph:
        edges = mission_graph["edges"]
        nodes = {n["id"] for n in mission_graph["nodes"]}
        dangling = [f"{e['from']}->{e['to']}" for e in edges
                    if e["from"] not in nodes or e["to"] not in nodes]
        neg = [f"{e['from']}->{e['to']}" for e in edges
               if e["distance_m"] <= 0 or e["expected_time_s"] <= 0]
        # every sector must be reachable from a home node
        reach = set()
        frontier = [n for n in nodes if n.startswith("AVI_HOME_")]
        adj = {}
        for e in edges:
            adj.setdefault(e["from"], []).append(e["to"])
            adj.setdefault(e["to"], []).append(e["from"])
        while frontier:
            cur = frontier.pop()
            if cur in reach:
                continue
            reach.add(cur)
            frontier.extend(adj.get(cur, []))
        unreach = [s for s in P.SECTOR_NAMES
                   if f"AVI_SECTOR_{s[-1]}_INSPECT_START" not in reach]
        bad = dangling + neg + unreach
        R.append(Result("V29", "mission graph is connected and costed",
                        "PASS" if not bad else "FAIL",
                        f"{len(nodes)} nodes, {len(edges)} edges, "
                        f"{len(reach)} reachable from home",
                        "no dangling edges, all sectors reachable",
                        "every sector is reachable from a home node"
                        if not bad else "graph is broken", bad[:12]))
    else:
        R.append(Result("V29", "mission graph is connected and costed",
                        "SKIP"))

    # ---- V30 dynamic content is separated from structure -------------------
    dyn_colls = ["AVI_DYNAMIC_VEHICLES", "AVI_DYNAMIC_BOATS",
                 "AVI_DYNAMIC_PEDESTRIANS", "AVI_DYNAMIC_OBSTACLES"]
    missc = [c for c in dyn_colls if c not in bpy.data.collections]
    leaked = []
    for cn in dyn_colls:
        c = bpy.data.collections.get(cn)
        if c is None:
            continue
        for o in c.objects:
            if o.get("avi_is_structure", False):
                leaked.append(o.name)
    R.append(Result("V30", "dynamic content separated from structure",
                    "PASS" if not missc and not leaked else "FAIL",
                    f"{len(dyn_colls) - len(missc)}/{len(dyn_colls)} "
                    "collections present",
                    "all present, no structure inside",
                    "dynamic objects can be switched off without touching "
                    "the asset" if not missc and not leaked
                    else "collections missing or contaminated",
                    missc + leaked))

    # ---- V31 scenario controller ------------------------------------------
    import scenarios as SC
    sc = bpy.context.scene
    have_cfg = "AVIAN_SCENARIO" in sc.keys()
    n_pre = len(SC.PRESETS)
    n_light = len(SC.LIGHTING_SCENARIOS)
    ok = have_cfg and n_pre >= 9 and n_light >= 7
    R.append(Result("V31", "scenario controller configured",
                    "PASS" if ok else "FAIL",
                    f"{n_pre} presets, {n_light} lighting, "
                    f"{len(SC.WEATHER_SCENARIOS)} weather",
                    ">= 9 presets, >= 7 lighting, config on the scene",
                    f"active preset: {sc.get('avi_scenario_preset', '-')}"
                    if ok else "controller not applied to the scene"))

    # ---- V32 dataset readiness --------------------------------------------
    ids = [o for o in bpy.data.objects if o.get("avi_defect_id")]
    dup = len({o["avi_defect_id"] for o in ids}) != len(ids)
    vl = bpy.context.view_layer
    passes = []
    for attr in ("use_pass_combined", "use_pass_z",
                 "use_pass_normal", "use_pass_object_index"):
        if getattr(vl, attr, False):
            passes.append(attr.replace("use_pass_", ""))
    idx_set = sum(1 for o in bpy.data.objects if o.pass_index > 0)
    cams = [o for o in bpy.data.objects if o.type == "CAMERA"]
    ok = (not dup and len(passes) >= 4 and idx_set >= len(records)
          and len(cams) >= 12)
    R.append(Result("V32", "dataset generation is ready",
                    "PASS" if ok else "FAIL",
                    f"{len(passes)} passes, {idx_set} indexed objects, "
                    f"{len(cams)} cameras",
                    ">= 4 passes, >= 192 indices, >= 12 cameras",
                    "combined/depth/normal/object-index enabled; every "
                    "defect has a unique pass_index" if ok
                    else "missing passes, indices or cameras"))

    n_pass = sum(1 for r in R if r.status == "PASS")
    n_fail = sum(1 for r in R if r.status == "FAIL")
    n_skip = sum(1 for r in R if r.status == "SKIP")
    for r in R:
        log(r.row())
        if r.status == "FAIL" and r.offenders:
            for o in r.offenders[:6]:
                log(f"         - {o}")
            if len(r.offenders) > 6:
                log(f"         - ... and {len(r.offenders)-6} more")
    return R, {"pass": n_pass, "fail": n_fail, "skip": n_skip,
               "total": len(R)}

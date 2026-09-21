"""AVIAN REV-C -- master build for the realism/metro/advanced-city upgrade.

    blender -b --python run_blender.py -- source/build_scene_c.py
    blender -b --python run_blender.py -- source/build_scene_c.py --render

WHAT THIS IS
------------
An UPGRADE of REV-B, the same relationship REV-B has to REV-A:
build_scene_b is imported and run UNCHANGED (and itself runs REV-A's
modules unchanged). REV-C adds modules alongside it -- Stage 1 adds
materials_c (South Mumbai facades, concrete/asphalt/water weathering) and
Stage 1b adds contrast_c (measured defect contrast).

TWO PHASES, TWO PROCESSES
-------------------------
Running with no --phase orchestrates both, in sequence:

  --phase geometry   build, run visibility, save the .blend and a handoff
                     JSON, exit. NEVER renders.
  --phase measure    fresh process: open the saved .blend, apply the
                     weathering sweep, measure contrast, validate, export,
                     render. NEVER ray-casts.

This is not defensive structuring for its own sake. visibility.py measures
occlusion with scene.ray_cast() against an evaluated depsgraph, which makes
Blender build a scene-wide raycast BVH; on this machine's Blender 4.0.2 a
Cycles render afterwards segfaults inside Embree 4.3.0's own BVH build
(rtcSetSharedGeometryBuffer). Reopening the .blend does not clear it,
because bpy.ops.wm.open_mainfile does not tear down the Cycles device --
that is a module-level singleton in the addon and survives the file load.
Only a fresh process clears it.

Running the phases NESTED rather than in sequence was also tried and was
killed by the OOM reaper (exit 137): two Blender processes each holding the
13,835-object scene do not fit. Sequential phases hold one at a time.

The split is the right architecture independently of the bug: Phase A's
output is exactly the expensive, cacheable part, which is what the
visibility cache already wants.
"""
from __future__ import annotations
import json
import os
import subprocess
import sys
import time

import bpy

import build_scene_b as B2
import contrast_c as CC
import materials as M
import materials_c as MC
import metro as MB
import metro_damage as MD
import params as P
import params_c as PC
import validate as VD
import validate_b as VDB
import validate_c as VDC
import viscache_c as VC
import visibility as VIS
import zones_c as ZC

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_DIR = os.path.join(os.path.dirname(SOURCE_DIR), "scene")
BLEND = os.path.join(SCENE_DIR, "AVIAN_SIC_REV_C.blend")
HANDOFF = os.path.join(SCENE_DIR, "_handoff_phase_a.json")
COLLISION_DIR = os.path.join(SCENE_DIR, "collision")
COLLISION_MANIFEST = os.path.join(
    COLLISION_DIR, "avian_bridge_collision_manifest.json")
LAUNCHER = os.path.join(os.path.dirname(SOURCE_DIR), "run_blender.py")


def _log_to(lines):
    def log(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s, flush=True)
    return log


def _metro_collections(colls):
    """REV-C's own collections, added alongside REV-B's tree.

    Separate collections rather than reusing the road bridge's, so the metro
    can be switched off, exported, or counted on its own -- the same reason
    REV-B split dynamic content out from structure.
    """
    root = colls["ROOT"]
    for name, parent in (("AVIAN_METRO", root),
                         ("AVIAN_METRO_DEFECTS", root),
                         ("AVIAN_METRO_AIRSPACE", root),
                         ("AVIAN_METRO_SECTORS", root),
                         ("AVIAN_METRO_MISSION", root)):
        if name in colls:
            continue
        c = bpy.data.collections.new(name)
        parent.children.link(c)
        colls[name] = c
    return colls


def _flags(argv):
    return {
        "render": "--render" in argv,
        "scenario": (argv[argv.index("--scenario") + 1]
                     if "--scenario" in argv else "BASELINE"),
        "samples": (int(argv[argv.index("--samples") + 1])
                    if "--samples" in argv else None),
        # The cache is opt-in. --no-vis-cache is the default behaviour and
        # is accepted explicitly so the documented flag does what it says.
        "vis_cache": ("--vis-cache" in argv
                      and "--no-vis-cache" not in argv),
        "sweep": "--no-sweep" not in argv,
        "weather": (float(argv[argv.index("--weather") + 1])
                    if "--weather" in argv else None),
    }


# ===========================================================================
# PHASE A -- geometry and visibility. Never renders.
# ===========================================================================
def phase_geometry(argv, log):
    f = _flags(argv)
    t0 = time.time()

    # visibility.compute() is the largest single cost in the build (~140 s
    # of ~220 s). A gate run recomputes and still refreshes the cache: a
    # validation report built on a cached measurement has not measured
    # anything, and working rule 3 has no exception for slow checks.
    _orig = VIS.compute
    VIS.compute = VC.wrap(_orig, force=not f["vis_cache"],
                          gate=not f["vis_cache"], log=log)
    try:
        scene, colls, records, graph, cfg, passes, stats = B2.build(
            f["scenario"], log)
    finally:
        VIS.compute = _orig
    stats["visibility_cached"] = bool(f["vis_cache"])

    # ---- Stage 2: the metro viaduct ------------------------------------
    # Built BEFORE the materials pass so the metro's concrete picks up the
    # same weathering, and before visibility would matter -- but AFTER
    # B2.build(), so not one road-bridge object is touched. Every defect on
    # the road bridge is anchored to its host by ray-cast, so perturbing a
    # BR_ member by a centimetre would move its defects and break V22.
    log("")
    log("REV-C Stage 2 (metro viaduct)")
    mats = M.build_library()
    _metro_collections(colls)
    stats["metro"] = MB.build(colls, mats, log)
    mrecords, mcounts = MD.build(colls, mats, log)
    stats["metro_defects"] = {"count": len(mrecords), "by_type": mcounts}
    stats["metro_zones"] = ZC.build(colls, log)

    log("")
    log("REV-C Stage 1 (materials, colour, micro-detail)")
    s = PC.CONCRETE_WEATHER_STRENGTH if f["weather"] is None \
        else float(f["weather"])
    stats["materials_c"] = MC.build_all(s, log)
    stats["weather_strength"] = s

    # Metro defects get the same measured visibility the road bridge's do.
    # A metro defect without it is a defect the planner cannot be honestly
    # scored on, so it runs through the identical 61-direction hemisphere.
    log("")
    log("  metro visibility")
    stats["metro_visibility"] = VIS.compute(mrecords, log=log)

    os.makedirs(SCENE_DIR, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    mb = os.path.getsize(BLEND) / 1e6
    log(f"  saved   : {os.path.basename(BLEND)} ({mb:.1f} MB)")

    with open(HANDOFF, "w") as fh:
        json.dump({"records": records, "mrecords": mrecords,
                   "graph": graph, "cfg": cfg, "stats": stats}, fh)
    log(f"  handoff : {len(records)} road + {len(mrecords)} metro records "
        f"-> {os.path.basename(HANDOFF)}")
    log(f"  phase A : {time.time()-t0:.1f} s, "
        f"{len(bpy.data.objects)} objects")


# ===========================================================================
# PHASE COLLISION -- re-export the PyBullet collision asset from REV-C
# ===========================================================================
def phase_collision(argv, log):
    """Run the UAV package's collision exporter against the REV-C scene.

    Its own process, because export() calls open_mainfile and would
    otherwise destroy whatever scene the calling phase is holding. It is
    also the one place MB_ has to show up: V37 reads the manifest this
    writes and compares the primitive count against REV-B's 677.

    Deliberately the UAV package's exporter, not a copy. Two decomposers
    are two sources of the same numbers.
    """
    root = os.environ.get("AVIAN_UAV_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(SOURCE_DIR)), "AVIAN_UAV")
    path = os.path.join(root, "simulation", "export_bridge_collision.py")
    if not os.path.exists(path):
        raise SystemExit(f"collision exporter not found at {path}")

    import importlib.util
    spec = importlib.util.spec_from_file_location("_avian_colx", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    os.makedirs(COLLISION_DIR, exist_ok=True)
    mod.export(blend=BLEND, out_dir=COLLISION_DIR, log=log)

    # Read the count back off the manifest rather than trusting export()'s
    # return value -- it does not return the count, and the first version of
    # this line reported "0 primitives" while the exporter's own log said
    # 1354. A number that can silently read zero needs to come from the
    # artifact, not from a convenient-looking variable.
    if not os.path.exists(COLLISION_MANIFEST):
        raise SystemExit(f"collision export wrote no manifest at "
                         f"{COLLISION_MANIFEST}")
    with open(COLLISION_MANIFEST) as fh:
        man = json.load(fh)
    n = man.get("collision_primitives", 0)
    if n <= 0:
        raise SystemExit(f"collision export produced {n} primitives")
    log(f"  collision: {n} primitives in the REV-C scene "
        f"(REV-B road bridge alone was 677)")


# ===========================================================================
# PHASE B -- contrast, validation, renders. Never ray-casts.
# ===========================================================================
def phase_measure(argv, log):
    f = _flags(argv)
    t0 = time.time()

    if not os.path.exists(HANDOFF) or not os.path.exists(BLEND):
        raise SystemExit(
            "REV-C phase measure: no Phase A output found. Run without "
            "--phase to orchestrate both, or --phase geometry first.")
    with open(HANDOFF) as fh:
        blob = json.load(fh)
    records, graph, cfg = blob["records"], blob["graph"], blob["cfg"]
    mrecords = blob.get("mrecords", [])
    stats = blob["stats"]
    strength = stats["weather_strength"]

    # This process starts on an empty scene -- Phase A's geometry only
    # exists on disk. Loading it here rather than on the blender command
    # line keeps the launcher invocation identical for both phases.
    bpy.ops.wm.open_mainfile(filepath=BLEND)
    scene = bpy.context.scene
    log(f"  loaded  : {len(records)} records, scene "
        f"{os.path.basename(BLEND)} ({len(bpy.data.objects)} objects)")

    # ---- Stage 1b: measured contrast ----------------------------------
    # visibility.py measures occlusion and resolving power. It does not
    # measure contrast, and weathering attacks contrast specifically, so
    # expected_rgb_visibility goes optimistic the moment the concrete gets
    # dirty. This re-derives it against a rendered albedo measurement.
    log("")
    log("REV-C Stage 1b (measured contrast)")
    if f["sweep"]:
        # Clean endpoint first, so the scene is left at the working
        # strength afterwards rather than at the sweep's last point.
        MC.set_concrete_weather(0.0, log)
        clean = CC.measure(records, strength_label="0.00", log=log)
        base = {r["defect_id"]: r.get("expected_rgb_visibility")
                for r in records}
        MC.set_concrete_weather(strength, log)
    else:
        clean, base = None, None

    dirty = CC.measure(records, strength_label=f"{strength:.2f}", log=log)
    stats["contrast"] = {"at_strength": dirty}

    # The metro's defects get the same measurement, at the working strength.
    # Not swept: the sweep exists to characterise the knob, and one
    # characterisation of it is enough.
    if mrecords:
        stats["metro_contrast"] = CC.measure(
            mrecords, strength_label=f"metro {strength:.2f}", log=log)
    if base is not None:
        flips = [r["defect_id"] for r in records
                 if base.get(r["defect_id"]) != r.get(
                     "expected_rgb_visibility")]
        stats["contrast"]["at_clean"] = clean
        stats["contrast"]["reclassified_vs_clean"] = {
            "count": len(flips), "ids": flips,
            "clean_strength": 0.0, "dirty_strength": strength}
        log(f"  contrast: {len(flips)} of {len(records)} defects change RGB "
            f"classification between weather 0.00 and {strength:.2f} "
            f"(mean contrast {clean['mean_contrast']:.4f} -> "
            f"{dirty['mean_contrast']:.4f}; contrast-limited "
            f"{clean['contrast_limited']} -> {dirty['contrast_limited']})")

    # ---- validation ----------------------------------------------------
    # The same 32 checks REV-B runs. V14 checks ground-truth completeness
    # against a field list; REV-C adds contrast fields to every record, so
    # V14 must require them too or they are the one part of the ground
    # truth that nothing checks.
    VD.EXTRA_REQUIRED_FIELDS = list(CC.FIELDS)
    log("")
    log("  VALIDATION  (REV-A V01-V21, then REV-B V22-V32)")
    log(f"  V14 field set extended by REV-C: +{len(CC.FIELDS)} contrast "
        f"fields ({', '.join(CC.FIELDS)})")
    if stats.get("visibility_cached"):
        log("  WARNING: visibility was restored from cache, not measured. "
            "This report is NOT gate-valid; rerun without --vis-cache.")
    res_a, sum_a = VD.run(records, log)
    res_b, sum_b = VDB.run(records, log, mission_graph=graph,
                           scenario_cfg=cfg)
    log("")
    log("  VALIDATION  (REV-C V33-V43, metro)")
    res_c, sum_c = VDC.run(records, mrecords, log,
                           collision_manifest=COLLISION_MANIFEST)

    # Working rule 3.10: a green check is evidence of nothing until it has
    # been observed going red. Break what each REV-C check guards and
    # confirm it fails.
    import sabotage_c as SAB
    stats["sabotage"] = SAB.run(records, mrecords, COLLISION_MANIFEST, log)
    allres = res_a + res_b + res_c
    summary = {"pass": sum_a["pass"] + sum_b["pass"] + sum_c["pass"],
               "fail": sum_a["fail"] + sum_b["fail"] + sum_c["fail"],
               "skip": sum_a["skip"] + sum_b["skip"] + sum_c["skip"],
               "total": len(allres)}
    log(f"  {summary['pass']} pass, {summary['fail']} fail, "
        f"{summary['skip']} skip of {summary['total']} checks")
    VD.export(allres, os.path.join(SCENE_DIR,
                                   "AVIAN_validation_report_REV_C.json"))

    # ---- statistics and exports ---------------------------------------
    tris = sum(sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
               for o in bpy.data.objects
               if o.type == "MESH" and not o.hide_render)
    uniq = len(set(o.data.name for o in bpy.data.objects
                   if o.type == "MESH"))
    stats["scene"] = {
        "revision": "REV_C_STAGE_1B",
        "gate_valid": not stats.get("visibility_cached", False),
        "objects": len(bpy.data.objects),
        "unique_meshes": uniq,
        "triangles_rendered": tris,
        "materials": len(bpy.data.materials),
        "collections": len(bpy.data.collections),
        "cameras": sum(1 for o in bpy.data.objects if o.type == "CAMERA"),
        "active_scenario": cfg,
        "validation": summary,
    }

    # REV-C ground truth, carrying the contrast fields. Written beside the
    # REV-C scene under its own name -- REV-B's exported ground truth is
    # the V22 baseline and is never written to from here.
    import damage as DM
    gt = DM.export_ground_truth(
        records,
        os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_REV_C.json"),
        os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_REV_C.csv"))
    stats["ground_truth_rev_c"] = gt
    log(f"  export  : REV-C ground truth {gt['total_defects']} defects, "
        f"{len(records[0]) if records else 0} fields/record")

    if mrecords:
        mgt = MD.export_ground_truth(
            mrecords,
            os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_REV_C.json"),
            os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_REV_C.csv"))
        stats["metro_ground_truth"] = mgt
        log(f"  export  : metro ground truth {mgt['total_defects']} "
            f"defects, {len(mrecords[0])} fields/record")

    with open(os.path.join(SCENE_DIR, "AVIAN_scene_stats_REV_C.json"),
              "w") as fh:
        json.dump(stats, fh, indent=2)

    if f["render"]:
        import renders
        import lighting as LT
        renders.OUT_DIR = os.path.join(SCENE_DIR, "renders")

        # ENVIRONMENT WORKAROUND, recorded per working rule 5 rather than
        # applied silently: this machine's Blender is built without
        # OpenImageDenoiser, so configure_render()'s use_denoising=True
        # makes bpy.ops.render.render() raise outright. It is a property of
        # the install, not of REV-C, and build_scene_b.py --render would hit
        # it equally here. Patched in this process, not in lighting.py,
        # which is a REV-B module REV-C runs unchanged.
        _configure = LT.configure_render

        def _configure_local(*a, **kw):
            _configure(*a, **kw)
            cy = bpy.context.scene.cycles
            if PC.RENDER_DISABLE_DENOISE:
                cy.use_denoising = False
            if PC.RENDER_FORCE_BVH2:
                cy.debug_bvh_layout = "BVH2"
        LT.configure_render = _configure_local

        renders.run(scene, records, samples=f["samples"], log=log)

        # REV-C's own diagnostic views. No standard camera gets within 50 m
        # of a building, so the Stage 1 facade work is invisible to the
        # nine validation views -- these are what make it checkable.
        import renders_c
        renders_c.OUT_DIR = renders.OUT_DIR
        renders_c.run(scene, samples=f["samples"], log=log)

    log(f"  phase B : {time.time()-t0:.1f} s")
    return summary


# ===========================================================================
# ORCHESTRATOR -- runs both phases, in sequence, and fails loudly
# ===========================================================================
DONE_MARKER = "REVC_PHASE_COMPLETE"
DURATIONS = os.path.join(SCENE_DIR, "_phase_durations.json")


def _durations(set_name=None, set_value=None):
    """Last successful wall-time per phase, for the duration floor."""
    d = {}
    if os.path.exists(DURATIONS):
        try:
            with open(DURATIONS) as fh:
                d = json.load(fh)
        except (ValueError, OSError):
            d = {}
    if set_name is not None:
        d[set_name] = round(float(set_value), 1)
        try:
            os.makedirs(SCENE_DIR, exist_ok=True)
            with open(DURATIONS, "w") as fh:
                json.dump(d, fh, indent=2)
        except OSError:
            pass
    return d


def _run_phase(name, argv):
    cmd = [bpy.app.binary_path, "--background", "--python", LAUNCHER, "--",
           os.path.join("source", "build_scene_c.py"), "--phase", name
           ] + [a for a in argv if a != "--phase"]
    print(f"\n=== REV-C phase {name} ===", flush=True)
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=os.path.dirname(SOURCE_DIR),
                          capture_output=True, text=True)
    dt = time.time() - t0
    print(proc.stdout, end="", flush=True)
    if proc.stderr.strip():
        print(proc.stderr, end="", flush=True)

    # Returncode alone is NOT sufficient. Blender in background mode exited
    # 0 on an unhandled Python exception in this exact pipeline -- the
    # collision phase died on a KeyError and still reported "OK in 4.6 s".
    # Each phase therefore prints a marker as its last act, and its absence
    # is a failure regardless of what the exit code claims. Same lesson as
    # the stale phase1_report: a success signal that can be produced
    # without the work happening is not a success signal.
    ok = (proc.returncode == 0) and (DONE_MARKER in proc.stdout)
    if not ok:
        raise SystemExit(
            f"REV-C phase {name} FAILED after {dt:.1f} s "
            f"(exit {proc.returncode}, marker "
            f"{'present' if DONE_MARKER in proc.stdout else 'ABSENT'}). "
            f"137 means the OOM reaper; 139/-11 means a segfault.")

    # Duration floor. Both phase failures this project has seen announced
    # themselves as implausibly fast before anything else gave them away:
    # 4.6 s and 2.3 s for jobs that must open a 65 MB scene. So each phase
    # remembers its own last good wall-time and a run far under it is called
    # out. Deliberately a loud warning and not a hard failure: a phase can
    # be legitimately fast (the collision export genuinely runs in ~2 s),
    # and a floor that blocks real work would get switched off, which is
    # worse than one that is read. The hard guards stay where they belong --
    # on the artifacts, which is why phase_collision asserts the manifest
    # exists and its count is non-zero.
    prev = _durations().get(name)
    if prev and dt < 0.25 * prev:
        print(f"!!! phase {name} finished in {dt:.1f} s against a previous "
              f"{prev:.1f} s -- SUSPECT. Verify its artifacts before "
              f"trusting this run.", flush=True)
    _durations(set_name=name, set_value=dt)
    print(f"=== phase {name} OK in {dt:.1f} s ===", flush=True)


def orchestrate(argv):
    t0 = time.time()
    _run_phase("geometry", argv)
    _run_phase("collision", argv)
    _run_phase("measure", argv)
    print(f"\nREV-C complete in {time.time()-t0:.1f} s "
          f"(both phases). Scene: {BLEND}", flush=True)


def main():
    argv = sys.argv[1:]
    if "--phase" not in argv:
        orchestrate(argv)
        return

    phase = argv[argv.index("--phase") + 1]
    lines = []
    log = _log_to(lines)
    if phase == "geometry":
        phase_geometry(argv, log)
        name = "AVIAN_build_log_REV_C_phase_a.txt"
    elif phase == "collision":
        phase_collision(argv, log)
        name = "AVIAN_build_log_REV_C_collision.txt"
    elif phase == "measure":
        phase_measure(argv, log)
        name = "AVIAN_build_log_REV_C_phase_b.txt"
    else:
        raise SystemExit(f"unknown --phase {phase!r}")

    os.makedirs(SCENE_DIR, exist_ok=True)
    with open(os.path.join(SCENE_DIR, name), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    # Last act of a phase that actually finished. _run_phase treats its
    # absence as failure even when Blender exits 0.
    print(DONE_MARKER, flush=True)


if __name__ == "__main__":
    main()

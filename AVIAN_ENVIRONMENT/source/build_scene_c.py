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
import materials_c as MC
import params_c as PC
import validate as VD
import validate_b as VDB
import viscache_c as VC
import visibility as VIS

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_DIR = os.path.join(os.path.dirname(SOURCE_DIR), "scene")
BLEND = os.path.join(SCENE_DIR, "AVIAN_SIC_REV_C.blend")
HANDOFF = os.path.join(SCENE_DIR, "_handoff_phase_a.json")
LAUNCHER = os.path.join(os.path.dirname(SOURCE_DIR), "run_blender.py")


def _log_to(lines):
    def log(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s, flush=True)
    return log


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

    log("")
    log("REV-C Stage 1 (materials, colour, micro-detail)")
    s = PC.CONCRETE_WEATHER_STRENGTH if f["weather"] is None \
        else float(f["weather"])
    stats["materials_c"] = MC.build_all(s, log)
    stats["weather_strength"] = s

    os.makedirs(SCENE_DIR, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    mb = os.path.getsize(BLEND) / 1e6
    log(f"  saved   : {os.path.basename(BLEND)} ({mb:.1f} MB)")

    with open(HANDOFF, "w") as fh:
        json.dump({"records": records, "graph": graph, "cfg": cfg,
                   "stats": stats}, fh)
    log(f"  handoff : {len(records)} records -> "
        f"{os.path.basename(HANDOFF)}")
    log(f"  phase A : {time.time()-t0:.1f} s, "
        f"{len(bpy.data.objects)} objects")


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
    allres = res_a + res_b
    summary = {"pass": sum_a["pass"] + sum_b["pass"],
               "fail": sum_a["fail"] + sum_b["fail"],
               "skip": sum_a["skip"] + sum_b["skip"],
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
def _run_phase(name, argv):
    cmd = [bpy.app.binary_path, "--background", "--python", LAUNCHER, "--",
           os.path.join("source", "build_scene_c.py"), "--phase", name
           ] + [a for a in argv if a != "--phase"]
    print(f"\n=== REV-C phase {name} ===", flush=True)
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=os.path.dirname(SOURCE_DIR))
    dt = time.time() - t0
    if proc.returncode != 0:
        raise SystemExit(
            f"REV-C phase {name} FAILED with exit {proc.returncode} "
            f"after {dt:.1f} s -- see the output above. "
            f"(137 means the OOM reaper; 139/-11 means a segfault.)")
    print(f"=== phase {name} OK in {dt:.1f} s ===", flush=True)


def orchestrate(argv):
    t0 = time.time()
    _run_phase("geometry", argv)
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
    elif phase == "measure":
        phase_measure(argv, log)
        name = "AVIAN_build_log_REV_C_phase_b.txt"
    else:
        raise SystemExit(f"unknown --phase {phase!r}")

    os.makedirs(SCENE_DIR, exist_ok=True)
    with open(os.path.join(SCENE_DIR, name), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()

"""AVIAN REV-C -- master build for the realism/metro/advanced-city upgrade.

    python build_scene_c.py                 build, validate, export, save
    python build_scene_c.py --render        also render validation views
    python build_scene_c.py --scenario NIGHT

WHAT THIS IS
------------
An UPGRADE of REV-B, the same relationship REV-B has to REV-A: build_scene_b
is imported and run UNCHANGED (which itself runs REV-A's own modules
unchanged). REV-C adds modules alongside it. Stage 1 adds exactly one:
materials_c, which rebuilds the four flat MAT_BUILDING_A..D shaders in place
into South Mumbai facade materials -- no other REV-C module exists yet.

SCENE FILE
----------
Written to AVIAN_ENVIRONMENT/scene/AVIAN_SIC_REV_C.blend (master brief v2
S2.7), NOT next to this script and NOT over
AVIAN_Smart_Infrastructure_City_REV_B.blend, which stays on disk as the
fallback and as what V22 is measured against.

VALIDATION
----------
Re-runs the same 32 checks (V01-V21 from validate.py, V22-V32 from
validate_b.py) against this build's own fresh in-memory records/graph/cfg --
not against anything read back off disk -- so a Stage 1 materials change
that somehow disturbed geometry would still be caught. No V33+ checks exist
yet; those start in Stage 2 (validate_c.py, metro-specific).
"""
from __future__ import annotations
import json
import os
import sys
import time

import bpy

import build_scene_b as B2
import materials_c as MC
import validate as VD
import validate_b as VDB

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_DIR = os.path.join(os.path.dirname(SOURCE_DIR), "scene")
BLEND = os.path.join(SCENE_DIR, "AVIAN_SIC_REV_C.blend")


def _log_to(lines):
    def log(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s, flush=True)
    return log


def build(scenario="BASELINE", log=print):
    t0 = time.time()
    scene, colls, records, graph, cfg, passes, stats = B2.build(scenario, log)

    log("")
    log("REV-C Stage 1 (materials, colour, micro-detail)")
    stats["materials_c"] = MC.build_city_materials(log)
    log(f"  total (incl. REV-C Stage 1): {time.time()-t0:.1f} s, "
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

    # ---- validation --------------------------------------------------
    # Same 32 checks REV-B runs, against this build's own fresh state.
    # REV-B's own exported manifests/ground-truth files are untouched --
    # Stage 1 has no new defects, zones or missions to export.
    log("")
    log("  VALIDATION  (REV-A V01-V21, then REV-B V22-V32 -- unchanged for "
        "REV-C Stage 1)")
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

    # ---- scene statistics ----------------------------------------------
    tris = sum(sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
               for o in bpy.data.objects
               if o.type == "MESH" and not o.hide_render)
    uniq = len(set(o.data.name for o in bpy.data.objects
                   if o.type == "MESH"))
    stats["scene"] = {
        "revision": "REV_C_STAGE_1",
        "objects": len(bpy.data.objects),
        "unique_meshes": uniq,
        "triangles_rendered": tris,
        "materials": len(bpy.data.materials),
        "collections": len(bpy.data.collections),
        "cameras": sum(1 for o in bpy.data.objects if o.type == "CAMERA"),
        "active_scenario": cfg,
        "validation": summary,
    }

    os.makedirs(SCENE_DIR, exist_ok=True)
    with open(os.path.join(SCENE_DIR, "AVIAN_scene_stats_REV_C.json"),
              "w") as f:
        json.dump(stats, f, indent=2)

    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    mb = os.path.getsize(BLEND) / 1e6
    log(f"  saved   : {os.path.basename(BLEND)} ({mb:.1f} MB) -> {SCENE_DIR}")

    with open(os.path.join(SCENE_DIR, "AVIAN_build_log_REV_C.txt"),
              "w") as f:
        f.write("\n".join(lines) + "\n")

    if do_render:
        import renders
        import lighting as LT
        # Keep REV-C's validation renders alongside the REV-C scene rather
        # than in source/renders/, which is REV-B's own output directory.
        renders.OUT_DIR = os.path.join(SCENE_DIR, "renders")

        # TWO ENVIRONMENT WORKAROUNDS, recorded per working rule 5 rather
        # than silently applied. Both are properties of this machine's
        # system Blender 4.0.2, not of REV-C, and both would equally affect
        # `build_scene_b.py --render` here -- that command was simply never
        # exercised on this machine before REV-C needed it.
        #
        # 1. use_denoising: this Blender was built without
        #    OpenImageDenoiser, so LT.configure_render()'s
        #    use_denoising=True makes bpy.ops.render.render() raise
        #    outright ("Build without OpenImageDenoiser").
        #
        # 2. debug_bvh_layout: rendering in the same process that just
        #    built the scene segfaults inside libembree4 4.3.0
        #    (rtcSetSharedGeometryBuffer, while building proto-object
        #    BVHs). Measured, not assumed: the same scene saved and
        #    reopened in a fresh process renders fine under EMBREE, and so
        #    does REV-B's, so this is about in-process state after a
        #    from-scratch build, not about the scene or the materials.
        #    BVH2 avoids the Embree path entirely; it is an acceleration
        #    structure, so the rendered image is unaffected.
        #
        # Patched here, scoped to this process, rather than editing
        # lighting.py -- a REV-B module REV-C runs unchanged.
        _configure = LT.configure_render
        def _configure_local(*a, **kw):
            _configure(*a, **kw)
            cy = bpy.context.scene.cycles
            cy.use_denoising = False
            cy.debug_bvh_layout = "BVH2"
        LT.configure_render = _configure_local

        renders.run(scene, records, samples=samples, log=log)


if __name__ == "__main__":
    main()

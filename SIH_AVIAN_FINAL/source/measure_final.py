"""SIH_AVIAN_FINAL -- contrast, cameras, renders, validation, sabotage,
ground truth export, baseline freeze. Phase 2 of 2.

Reopens the .blend build_final.py saved, in a fresh Blender process -- kept
separate from phase 1 because damage.py's ray-casting and contrast_c's EEVEE
rendering sharing one process is a documented crash risk (see build_final.py
and build_scene_c.py's own docstring).

Usage:
    blender --background --python run_blender.py -- source/measure_final.py [--render]
"""
from __future__ import annotations
import json
import os
import sys
import time

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ENV_SRC = os.environ.get(
    "AVIAN_ENV_SRC",
    os.path.join(os.path.dirname(ROOT), "AVIAN_ENVIRONMENT", "source"))
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import damage as DMG
import metro_damage as MBD
import contrast_c as CC
import lighting as LT
import params_c as PARAMS_C
import cameras_final as CF
import validate_final as VDF
import sabotage_final as SBF
import params_final as PF

# damage.py's export_ground_truth() reads P.RESEARCH_X0/X1/SECTOR_NAMES/
# REPAIR_ENVELOPE for the ground-truth file's metadata. This is a SEPARATE
# Blender process from build_final.py, so its own `import damage as DMG`
# starts unpatched (REV-C's real params.py) -- without this, the exported
# ground truth's "research_zone" block silently shows x=1650..2550, SECTOR_A
# ..SECTOR_F, instead of this scene's x=90..270, SECTOR_A..SECTOR_C.
DMG.P = PF

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
HANDOFF_PATH = os.path.join(SCENE_DIR, "_handoff_final.json")
BASELINE_PATH = os.path.join(SCENE_DIR, "BASELINE_FINAL_ground_truth.json")
RENDER_DIR = os.path.join(ROOT, "renders")


def main():
    render = "--render" in sys.argv

    log("== SIH_AVIAN_FINAL :: measure_final.py (phase 2: measure) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    with open(HANDOFF_PATH) as f:
        handoff = json.load(f)
    records = handoff["records"]
    mrecords = handoff["mrecords"]
    stats = handoff["stats"]
    log(f"  loaded  : {BLEND_PATH}")
    log(f"  records : {len(records)} road, {len(mrecords)} metro")

    log("-- cameras --")
    # Idempotent: measure_final.py can be re-run against the same .blend
    # (e.g. after a validate_final.py fix) without piling up duplicate
    # cameras each time.
    for o in list(bpy.data.objects):
        if o.type == "CAMERA" and o.name.startswith("CAM_"):
            cam_data = o.data
            bpy.data.objects.remove(o, do_unlink=True)
            if cam_data.users == 0:
                bpy.data.cameras.remove(cam_data)
    cam_coll = bpy.data.collections.get("CAMERAS")
    hero = next((r for r in records if r.get("type") == "REBAR_EXPOSED"
               and bpy.data.objects.get(r["defect_id"]) is not None
               and bpy.data.objects[r["defect_id"]].get("avi_hero")), None)
    hero_pos = tuple(hero["position_m"]) if hero else None
    CF.build(cam_coll, hero_pos=hero_pos, log=log)

    log("-- measured contrast (contrast_c.py, reused unchanged) --")
    contrast_summary = CC.measure(records, strength_label="1.00", log=log)
    stats["contrast"] = contrast_summary
    mcontrast_summary = CC.measure(mrecords, strength_label="metro 1.00",
                                   log=log)
    stats["metro_contrast"] = mcontrast_summary

    log("-- ground truth export --")
    gt_json = os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.json")
    gt_csv = os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.csv")
    DMG.export_ground_truth(records, gt_json, gt_csv)
    mgt_json = os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_FINAL.json")
    mgt_csv = os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_FINAL.csv")
    MBD.export_ground_truth(mrecords, mgt_json, mgt_csv)
    log(f"  exported: {gt_json}")
    log(f"  exported: {mgt_json}")

    log("-- baseline --")
    if not os.path.exists(BASELINE_PATH):
        with open(gt_json) as f:
            gt = json.load(f)
        with open(BASELINE_PATH, "w") as f:
            json.dump(gt, f, indent=2)
        log(f"  FROZEN new baseline: {BASELINE_PATH} ({len(records)} defects)")
        stats["baseline_frozen_this_run"] = True
    else:
        log(f"  baseline already frozen: {BASELINE_PATH}")
        stats["baseline_frozen_this_run"] = False

    log("-- validation (validate_final.py) --")
    results, vsummary = VDF.run(records, mrecords, log=log,
                                baseline_path=BASELINE_PATH)
    stats["validation"] = vsummary
    stats["validation_detail"] = [r.as_dict() for r in results]

    log("-- sabotage (working rule 3.10) --")
    sabotage_report = SBF.run(records, mrecords, BASELINE_PATH, log=log)
    stats["sabotage"] = sabotage_report

    # sabotage mutates `records`/`mrecords` transiently but always restores
    # them (finally: undo()) -- re-export so the files on disk reflect the
    # final, un-sabotaged state, not whatever the last mutation left behind.
    DMG.export_ground_truth(records, gt_json, gt_csv)
    MBD.export_ground_truth(mrecords, mgt_json, mgt_csv)

    if render:
        log("-- rendering the 9 named cameras --")
        os.makedirs(RENDER_DIR, exist_ok=True)

        # ENVIRONMENT WORKAROUND (same as build_scene_c.py's phase_measure,
        # working rule 5): this machine's Blender is built without
        # OpenImageDenoiser, so configure_render()'s use_denoising=True makes
        # bpy.ops.render.render() raise outright. Property of the install,
        # not of this scene -- patched here, not in lighting.py, which is
        # reused unchanged.
        _configure = LT.configure_render

        def _configure_local(*a, **kw):
            _configure(*a, **kw)
            cy = bpy.context.scene.cycles
            if PARAMS_C.RENDER_DISABLE_DENOISE:
                cy.use_denoising = False
            if PARAMS_C.RENDER_FORCE_BVH2:
                cy.debug_bvh_layout = "BVH2"
        LT.configure_render = _configure_local

        LT.configure_render(samples=PF.RENDER_SAMPLES, w=1280, h=720)
        scene = bpy.context.scene
        prev_cam = scene.camera
        cams = [o for o in bpy.data.objects if o.type == "CAMERA"
               and o.name.startswith("CAM_")]
        cams.sort(key=lambda o: o.name)
        for cam in cams:
            scene.camera = cam
            out = os.path.join(RENDER_DIR, f"{cam.name}.png")
            scene.render.filepath = out
            bpy.ops.render.render(write_still=True)
            log(f"  render  : {out}")
        scene.camera = prev_cam
        stats["renders"] = [c.name for c in cams]

    stats["wall_s_phase2"] = round(time.time() - T0, 1)
    with open(os.path.join(SCENE_DIR, "AVIAN_scene_stats_FINAL.json"), "w") as f:
        json.dump(stats, f, indent=2)
    with open(os.path.join(SCENE_DIR, "AVIAN_build_log_FINAL_phase2.txt"), "w") as f:
        f.write("\n".join(LOG_LINES))

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH)
    log(f"  saved   : {BLEND_PATH}")
    log(f"== done in {stats['wall_s_phase2']} s ==")
    print("FINAL_PHASE2_COMPLETE")


if __name__ == "__main__":
    main()

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
import resolvability_final as RESF

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
BASELINE_METRO_PATH = os.path.join(
    SCENE_DIR, "BASELINE_FINAL_metro_ground_truth.json")
BASELINE_STEEL_PATH = os.path.join(
    SCENE_DIR, "BASELINE_FINAL_steel_ground_truth.json")
RENDER_DIR = os.path.join(ROOT, "renders")


def main():
    render = "--render" in sys.argv

    log("== SIH_AVIAN_FINAL :: measure_final.py (phase 2: measure) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    with open(HANDOFF_PATH) as f:
        handoff = json.load(f)
    records = handoff["records"]
    mrecords = handoff["mrecords"]
    srecords = handoff["srecords"]
    stats = handoff["stats"]
    log(f"  loaded  : {BLEND_PATH}")
    log(f"  records : {len(records)} road, {len(mrecords)} metro, "
        f"{len(srecords)} steel")

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
    loose_certifiable = next(
        (r for r in srecords if r["type"] == "BOLT_LOOSE"
         and r.get("intended_difficulty_band") == "CERTIFIABLE"), None)
    loose_bolt = ((tuple(loose_certifiable["position_m"]),
                  tuple(loose_certifiable["surface_normal"]))
                 if loose_certifiable else None)
    CF.build(cam_coll, hero_pos=hero_pos, loose_bolt=loose_bolt, log=log)

    log("-- measured contrast (contrast_c.py, reused unchanged) --")
    contrast_summary = CC.measure(records, strength_label="1.00", log=log)
    stats["contrast"] = contrast_summary
    mcontrast_summary = CC.measure(mrecords, strength_label="metro 1.00",
                                   log=log)
    stats["metro_contrast"] = mcontrast_summary
    scontrast_summary = CC.measure(srecords, strength_label="steel 1.00",
                                   log=log)
    stats["steel_contrast"] = scontrast_summary

    log("-- escalation_reason (resolvability_final.py, SPEC S5) --")
    # Must run AFTER contrast_c.measure() above -- escalation_reason folds
    # in contrast_limited, which only exists once contrast has actually
    # been measured, so it cannot be finished in build_final.py (phase 1).
    esc_road = RESF.add_escalation(records, PF, log=log, label="road")
    esc_metro = RESF.add_escalation(mrecords, PF, log=log, label="metro")
    esc_steel = RESF.add_escalation(srecords, PF, log=log, label="steel")
    stats["escalation"] = {"road": esc_road, "metro": esc_metro,
                           "steel": esc_steel}

    log("-- resolvability breakdown (SPEC S5 -- the result) --")
    resolv_road = RESF.resolvability_breakdown(records, PF)
    resolv_metro = RESF.resolvability_breakdown(mrecords, PF)
    resolv_steel = RESF.resolvability_breakdown(srecords, PF)
    stats["resolvability"] = {"road": resolv_road, "metro": resolv_metro,
                              "steel": resolv_steel}
    log(f"  resolv  : road      {resolv_road['overall']}")
    log(f"  resolv  : metro     {resolv_metro['overall']}")
    log(f"  resolv  : steel     {resolv_steel['overall']}")
    for t, b in sorted(resolv_steel["by_type"].items()):
        log(f"  resolv  :   {t:<20} {b}")

    log("-- ground truth export --")
    gt_json = os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.json")
    gt_csv = os.path.join(SCENE_DIR, "AVIAN_defect_ground_truth_FINAL.csv")
    DMG.export_ground_truth(records, gt_json, gt_csv)
    mgt_json = os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_FINAL.json")
    mgt_csv = os.path.join(SCENE_DIR, "AVIAN_metro_ground_truth_FINAL.csv")
    MBD.export_ground_truth(mrecords, mgt_json, mgt_csv)
    sgt_json = os.path.join(SCENE_DIR, "AVIAN_steel_ground_truth_FINAL.json")
    sgt_csv = os.path.join(SCENE_DIR, "AVIAN_steel_ground_truth_FINAL.csv")
    DMG.export_ground_truth(srecords, sgt_json, sgt_csv)
    log(f"  exported: {gt_json}")
    log(f"  exported: {mgt_json}")
    log(f"  exported: {sgt_json}")

    log("-- baseline (concrete + metro + steel, SPEC S8) --")
    def _freeze(path, src_json, label, n):
        if not os.path.exists(path):
            with open(src_json) as f:
                gt = json.load(f)
            with open(path, "w") as f:
                json.dump(gt, f, indent=2)
            log(f"  FROZEN new baseline: {path} ({n} defects)")
            return True
        log(f"  baseline already frozen: {path}")
        return False

    stats["baseline_frozen_this_run"] = _freeze(
        BASELINE_PATH, gt_json, "road", len(records))
    stats["metro_baseline_frozen_this_run"] = _freeze(
        BASELINE_METRO_PATH, mgt_json, "metro", len(mrecords))
    stats["steel_baseline_frozen_this_run"] = _freeze(
        BASELINE_STEEL_PATH, sgt_json, "steel", len(srecords))

    log("-- validation (validate_final.py) --")
    results, vsummary = VDF.run(records, mrecords, log=log,
                                baseline_path=BASELINE_PATH,
                                srecords=srecords,
                                steel_baseline_path=BASELINE_STEEL_PATH)
    stats["validation"] = vsummary
    stats["validation_detail"] = [r.as_dict() for r in results]

    log("-- sabotage (working rule 3.10) --")
    sabotage_report = SBF.run(records, mrecords, BASELINE_PATH, log=log,
                              srecords=srecords,
                              steel_baseline_path=BASELINE_STEEL_PATH)
    stats["sabotage"] = sabotage_report

    # sabotage mutates `records`/`mrecords`/`srecords` transiently but
    # always restores them (finally: undo()) -- re-export so the files on
    # disk reflect the final, un-sabotaged state, not whatever the last
    # mutation left behind.
    DMG.export_ground_truth(records, gt_json, gt_csv)
    MBD.export_ground_truth(mrecords, mgt_json, mgt_csv)
    DMG.export_ground_truth(srecords, sgt_json, sgt_csv)

    if render:
        log("-- rendering the 12 named cameras (CAM_12 first) --")
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
        # Gate requirement: CAM_12_LOOSE_BOLT is the premise-demonstrating
        # shot and is shown/rendered first, ahead of the rest.
        cams.sort(key=lambda o: 0 if o.name == "CAM_12_LOOSE_BOLT" else 1)
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

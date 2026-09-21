"""SIH_AVIAN_FINAL -- geometry + materials + damage. Phase 1 of 2.

Builds the whole scene and saves it, plus a handoff JSON for measure_final.py.
Split from measurement (contrast_c's EEVEE renders) into a separate process
on purpose: damage.py's build() and visibility.py's compute() both ray-cast
heavily (Embree BVH), and build_scene_c.py already documents that doing heavy
ray-casting and EEVEE rendering in the same Blender process is a crash risk.
Same two-phase shape, just not split into OS subprocesses since this scene
is two orders of magnitude smaller than REV-C's.

Usage:
    blender --background --python run_blender.py -- source/build_final.py
"""
from __future__ import annotations
import json
import os
import random
import sys
import time

import bpy
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # SIH_AVIAN_FINAL/
ENV_SRC = os.environ.get(
    "AVIAN_ENV_SRC",
    os.path.join(os.path.dirname(ROOT), "AVIAN_ENVIRONMENT", "source"))
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)          # meshlib, materials, bridge, ... live here
if HERE not in sys.path:
    sys.path.insert(0, HERE)             # params_final, terrain_final

import meshlib as ML
import materials as MAT
import materials_c as MATC
import params_c as PARAMS_C
import bridge as BR
import metro as MB
import metro_damage as MBD
import damage as DMG
import visibility as VIS
import lighting as LT
import terrain_final as TF
import params_final as PF

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


# ===========================================================================
# 0. MONKEYPATCH -- reuse bridge.py / damage.py / metro.py / metro_damage.py /
# lighting.py byte-for-byte against THIS scene's parameters instead of
# REV-C's. See params_final.py's docstring for why this is safe: every one
# of these modules is a pure function of a `P`/module-global reference.
# ===========================================================================
BR.P = PF
DMG.P = PF
LT.P = PF
MBD.P = PF

MB.P = PF
MB.TR = TF                    # metro.py's pier footings use TR.height()
MB.Y = 28.0
MB.DECK_TOP_Z = 19.0
MB.X0, MB.X1 = 0.0, PF.BRIDGE_LENGTH
MB.SPAN = 30.0
MB.MAIN_X0, MB.MAIN_X1 = 135.0, 225.0
MB.BOX_D = 2.2                 # SPEC.md: matches 19.0 - 2.2 = 16.8 soffit
# ASSUMPTION: deeper section for the 90 m span. Measured, not assumed to be
# safe: at 4.5 m the main-span soffit (14.5) came out BELOW the road deck's
# own crown crest (14.6 at x=180) -- no actual collision (16.5 m apart in Y),
# but confusing and needlessly tight. 3.2 m keeps the metro soffit (15.8)
# clear of the road crest by >1 m while still deeper than the 2.2 m approach.
MB.BOX_D_MAIN = 3.2
MB.PIER_D = 2.0                # SPEC.md: single circular pier, Ø2.0 m
MB.CEILING_Z = 40.0            # ASSUMPTION: no station this scene (locked
                                # decision 2), so the tallest thing is the
                                # deck+parapet+mast; 40 m gives real margin

# Locked decision 2: no metro station. metro.py's build() calls
# build_station() unconditionally, so disable it rather than edit metro.py.
MB.build_station = lambda coll, mats, log=print: []

# metro_damage.py's msector_of() hardcodes a 6-letter (A..F) clamp, which is
# only safe if SECTOR_COUNT == 6. This scene uses 3 sectors -- rebind it so a
# defect exactly at RESEARCH_X1 cannot be mis-labelled MSECTOR_D.
def _msector_of(x):
    if not (PF.RESEARCH_X0 <= x <= PF.RESEARCH_X1):
        return "OUTSIDE_RESEARCH_ZONE"
    i = int((x - PF.RESEARCH_X0) / PF.SECTOR_LENGTH)
    i = max(0, min(PF.SECTOR_COUNT - 1, i))
    return f"MSECTOR_{PF.SECTOR_NAMES[i][-1]}"


MBD.msector_of = _msector_of

# Metro defect population, scaled down from REV-C's 60 for a viaduct with
# ~1/4 the piers/spans (ASSUMPTION -- SPEC.md gives no metro defect count).
MBD.COUNTS = {
    "SEGMENTAL_JOINT_LEAKAGE": 3,
    "EFFLORESCENCE_JOINT": 2,
    "BEARING_DISTRESS": 2,
    "INTERNAL_SOFFIT_CRACK": 3,
    "WEB_SHEAR_CRACK": 3,
    "SOFFIT_TRANSVERSE_CRACK": 3,
    "PIER_SPALL": 2,
    "PIER_CORROSION_STAIN": 2,
}


# ===========================================================================
# 1. SCENE + COLLECTIONS
# ===========================================================================
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.name = "SIH_AVIAN_FINAL"
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.scale_length = 1.0
    return sc


COLL_NAMES = [
    "TERRAIN", "DECK", "BEAMS", "BARRIERS", "DETAILS", "JOINTS", "PIERS",
    "AVIAN_METRO", "CRACKS", "SPALLING", "REBAR", "CORROSION",
    "JOINT_DAMAGE", "AVIAN_DEFECTS", "AVIAN_METRO_DEFECTS", "CAMERAS",
    "LIGHTING", "VEHICLES",
]


def build_collections(scene):
    root = bpy.data.collections.new("SIH_AVIAN_FINAL")
    scene.collection.children.link(root)
    colls = {"ROOT": root}
    for nm in COLL_NAMES:
        c = bpy.data.collections.new(nm)
        root.children.link(c)
        colls[nm] = c
    return colls


# ===========================================================================
# 2. BUILD
# ===========================================================================
def main():
    scene = reset_scene()
    colls = build_collections(scene)
    stats = {}

    log("== SIH_AVIAN_FINAL :: build_final.py (phase 1: geometry) ==")
    log(f"  seed    : {PF.SEED}")

    log("-- materials --")
    mats = MAT.build_library()
    # build_all() returns a STATS dict (booleans/counts), not Material
    # objects -- it mutates the node trees of the materials already in
    # `mats` (looked up by name) rather than replacing them. Keep the two
    # separate: merging it into `mats` clobbers e.g. mats["asphalt"] with a
    # bool.
    stats["materials_c"] = MATC.build_all(
        strength=PARAMS_C.CONCRETE_WEATHER_STRENGTH, log=log)
    mats["bldg_far"] = MAT.simple("MAT_BLDG_FAR_SILHOUETTE",
                                  (0.34, 0.36, 0.40), rough=0.75)
    stats["materials"] = len(bpy.data.materials)

    log("-- lighting (low morning sun, BASELINE) --")
    LT.build(colls["LIGHTING"], log)

    log("-- terrain / river / banks --")
    stats["terrain"] = TF.build(colls, mats, log)

    log("-- road bridge (bridge.py, reused unchanged) --")
    stats["bridge"] = BR.build(colls, mats, log)

    log("-- metro viaduct (metro.py, reused unchanged) --")
    stats["metro"] = MB.build(colls, mats, log)

    bpy.context.view_layer.update()

    log("-- road-bridge damage (damage.py, reused unchanged) --")
    records, counts, n_obj = DMG.build(colls, mats, log)
    stats["damage"] = {"defects": len(records), "objects": n_obj,
                        "by_type": counts}
    log(f"  damage  : {len(records)} road defects, {n_obj} objects")

    log("-- metro damage (metro_damage.py, reused unchanged) --")
    mrecords, mcounts = MBD.build(colls, mats, log)
    stats["metro_damage"] = {"defects": len(mrecords), "by_type": mcounts}

    log("-- hand-placed hero defect (SPEC S5.2) --")
    # "A large spall with three exposed, corroded rebars on a river pier at
    # z ~ 6 m." The random population (collect_sites/build in damage.py) has
    # no way to target a specific location -- earlier types (SPALL,
    # DELAMINATION) consume most PIER_COLUMN sites in the research zone
    # before REBAR_EXPOSED's turn, so this is genuinely hand-placed rather
    # than filtered for, per the working rule "make no unstated assumptions"
    # -- and per SPEC.md, deliberately so ("place, deliberately and
    # documented as such"). REBAR_EXPOSED's random target was set to 5 (not
    # 6) in params_final.py specifically to leave this slot, so the total
    # stays 96.
    #
    # Reuses damage.py's own maker (make_spall), ray-snap and carve exactly
    # as the random population does -- only the SITE is authored by hand.
    # Column "_1" of the twin bent at x=135 (pier 004, flanking the main
    # span), outboard face (away from the corridor centreline).
    hero_rnd = random.Random(PF.SEED + 9001)
    hero_col_y = -PF.PIER_COL_SPACING / 2.0
    hero_r = PF.PIER_COL_D / 2.0
    hero_site = DMG.Site(
        (135.0, hero_col_y - hero_r * 1.002, 6.0),
        (0, -1, 0), (0, 0, 1), "PIER_COLUMN", "BR_PIER_COL_004_1",
        "hand-placed hero defect for the pitch shot: large spall with "
        "exposed, corroded rebar, river-adjacent pier, outboard face",
        occlusion=0.15, u_max=2.2, v_max=1.8)
    hero_ob, hero_info, hero_extra, hero_cut = DMG.make_spall(
        6, "REBAR_EXPOSED", hero_site, 4, hero_rnd, colls["REBAR"], mats)

    hero_rec = {
        "defect_id": hero_ob.name,
        "type": "REBAR_EXPOSED",
        "severity": 4,
        "position_m": [round(v, 3) for v in hero_site.pos],
        "surface_normal": [round(v, 3) for v in hero_site.normal],
        "host_surface": hero_site.host,
        "host_object": hero_site.obj_name,
        "bridge_section": PF.section_at(hero_site.pos.x),
        "inspection_sector": PF.sector_at(hero_site.pos.x),
        "occlusion": round(hero_site.occlusion, 2),
        "placement_rationale": hero_site.reason,
    }
    hero_rec.update(hero_info)

    bpy.context.view_layer.update()
    host, hit, face_n = DMG._find_host(Vector(hero_rec["position_m"]),
                                       Vector(hero_rec["surface_normal"]))
    if host is not None:
        nrm = Vector(hero_rec["surface_normal"])
        if face_n.dot(nrm) < 0.0:
            face_n = -face_n
        if face_n.length > 0.5 and face_n.dot(nrm) > 0.30:
            q = nrm.rotation_difference(face_n)
            rot = q.to_matrix().to_4x4() @ hero_ob.matrix_world.to_3x3().to_4x4()
            nrm = face_n
        else:
            rot = hero_ob.matrix_world.to_3x3().to_4x4()
        eps = 0.002
        newp = hit + nrm * eps
        old_mw = hero_ob.matrix_world.copy()
        hero_ob.matrix_world = Matrix.Translation(newp) @ rot
        rel = hero_ob.matrix_world @ old_mw.inverted()
        for ch in hero_extra:
            ch.matrix_world = rel @ ch.matrix_world
        hero_rec["position_m"] = [round(v, 4) for v in hit]
        hero_rec["surface_normal"] = [round(v, 4) for v in nrm]
        hero_rec["host_object"] = host.name
        hero_rec["surface_offset_mm"] = round(eps * 1000.0, 1)
        if hero_cut is not None:
            cv, cf, _m = hero_cut
            mtx = hero_ob.matrix_world
            inv = host.matrix_world.inverted()
            local = [tuple(inv @ (mtx @ Vector(v))) for v in cv]
            DMG.carve([(host, local, cf, None)], log)
    else:
        hero_rec["surface_offset_mm"] = -1.0
        log("  hero    : WARNING no host surface found by ray-cast")

    ML.set_custom(hero_ob, {
        "avi_defect_id": hero_ob.name, "avi_type": "REBAR_EXPOSED",
        "avi_severity": 4, "avi_repairable": bool(hero_info["repairable"]),
        "avi_reason": hero_info["reason"], "avi_host_surface": hero_site.host,
        "avi_host_object": hero_rec["host_object"],
        "avi_section": hero_rec["bridge_section"],
        "avi_sector": hero_rec["inspection_sector"] or "OUTSIDE_RESEARCH_ZONE",
        "avi_occlusion": hero_site.occlusion,
        "avi_representation": hero_info["representation"],
        "avi_area_m2": hero_info["area_m2"],
        "avi_width_mm": hero_info["width_mm"],
        "avi_depth_mm": hero_info["depth_mm"],
        "avi_rationale": hero_site.reason,
        "avi_hero": True,
    })
    records.append(hero_rec)
    counts["REBAR_EXPOSED"] = counts.get("REBAR_EXPOSED", 0) + 1
    stats["hero_defect"] = hero_rec["defect_id"]
    log(f"  hero    : {hero_rec['defect_id']} on {hero_rec['host_object']} "
        f"z={hero_rec['position_m'][2]:.2f} area={hero_rec['area_m2']} m2 "
        f"({len(records)} road defects total)")

    log("-- measured visibility (visibility.py, reused unchanged) --")
    vis_summary = VIS.compute(records, log=log)
    stats["visibility"] = vis_summary
    mvis_summary = VIS.compute(mrecords, log=log)
    stats["metro_visibility"] = mvis_summary

    bpy.context.view_layer.update()

    log("-- traffic on the road deck (SPEC.md: ~16 vehicles) --")
    # Added AFTER visibility.compute() deliberately: these sit on the deck
    # TOP while every defect host is on the deck underside, girder webs,
    # diaphragms or piers, so they cannot legitimately occlude a defect --
    # but there is no reason to let them anywhere near the ray-casts that
    # matter, so they are built last.
    veh_rnd = random.Random(PF.SEED + 555)
    veh_specs = (["car"] * 13) + ["truck", "truck", "bus"]
    veh_rnd.shuffle(veh_specs)
    VEH_DIMS = {"car": (4.4, 1.8, 1.5), "truck": (8.0, 2.4, 3.0),
                "bus": (11.0, 2.5, 3.2)}
    VEH_MAT = {"truck": "vehicle_c", "bus": "vehicle_d"}
    car_mats = ["vehicle_a", "vehicle_b", "vehicle_c", "vehicle_d"]
    lane_ys = (-5.25, -1.75, 1.75, 5.25)
    xs = [20.0 + i * (PF.BRIDGE_LENGTH - 40.0) / max(1, len(veh_specs) - 1)
          for i in range(len(veh_specs))]
    veh_rnd.shuffle(xs)
    n_veh = 0
    for i, kind in enumerate(veh_specs):
        x = max(3.0, min(PF.BRIDGE_LENGTH - 3.0, xs[i] + veh_rnd.uniform(-3, 3)))
        lane = lane_ys[i % 4]
        L, W, H = VEH_DIMS[kind]
        z = PF.deck_top_z(x) + H / 2.0 + 0.02
        matk = VEH_MAT.get(kind, car_mats[i % 4])
        ob = ML.box(f"VEH_{kind.upper()}_{i+1:03d}", (L, W, H), (x, lane, z),
                   colls["VEHICLES"], mats[matk])
        ML.set_custom(ob, {"avi_kind": "vehicle", "avi_vehicle_type": kind})
        n_veh += 1
    stats["vehicles"] = n_veh
    log(f"  traffic : {n_veh} vehicles ({veh_specs.count('car')} cars, "
        f"{veh_specs.count('truck')} trucks, {veh_specs.count('bus')} bus)")

    log("-- metro train, parked mid-span, static (SPEC.md) --")
    train_car_l, gap = 22.0, 0.5
    total = 3 * train_car_l + 2 * gap
    x0 = 180.0 - total / 2.0
    rail_top_z = MB.DECK_TOP_Z + MB.SLAB_TRACK_T
    car_h, car_w = 3.6, 2.9
    for i in range(3):
        cx = x0 + i * (train_car_l + gap) + train_car_l / 2.0
        ob = ML.box(f"MB_TRAIN_CAR_{i+1}", (train_car_l, car_w, car_h),
                   (cx, MB.Y, rail_top_z + car_h / 2.0),
                   colls["VEHICLES"], mats["steel"])
        ML.set_custom(ob, {"avi_kind": "train_car", "avi_static": True})
    stats["train"] = {"cars": 3, "length_m": total, "centre_x": 180.0}
    log(f"  train   : 3 cars, {total:.1f} m overall, centred x=180")

    bpy.context.view_layer.update()

    log("-- triangle / object accounting --")
    n_obj_total = len(bpy.data.objects)
    n_tri = ML.scene_tris()
    stats["scene"] = {"objects": n_obj_total, "triangles": n_tri,
                      "materials": len(bpy.data.materials),
                      "collections": len(bpy.data.collections)}
    log(f"  objects : {n_obj_total}")
    log(f"  triangles: {n_tri}")

    log("-- saving --")
    scene_dir = os.path.join(ROOT, "scene")
    os.makedirs(scene_dir, exist_ok=True)
    blend_path = os.path.join(scene_dir, "SIH_AVIAN_FINAL.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    log(f"  blend   : {blend_path}")

    handoff = {
        "records": records,
        "mrecords": mrecords,
        "stats": stats,
        "blend_path": blend_path,
    }
    handoff_path = os.path.join(scene_dir, "_handoff_final.json")
    with open(handoff_path, "w") as f:
        json.dump(handoff, f)
    log(f"  handoff : {handoff_path}")

    stats["wall_s"] = round(time.time() - T0, 1)
    with open(os.path.join(scene_dir, "AVIAN_scene_stats_FINAL.json"), "w") as f:
        json.dump(stats, f, indent=2)
    with open(os.path.join(scene_dir, "AVIAN_build_log_FINAL_phase1.txt"), "w") as f:
        f.write("\n".join(LOG_LINES))

    log(f"== done in {stats['wall_s']} s ==")
    print("FINAL_PHASE1_COMPLETE")


if __name__ == "__main__":
    main()

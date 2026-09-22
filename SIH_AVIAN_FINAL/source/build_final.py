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
import materials_final as MATF
import vehicles_final as VEHF
import train_final as TRAINF
import base_final as BASEF
import microdetail_final as MDF
import steel_final as STEELF
import fasteners_final as FASTF
import damage_steel as DMGS
import resolvability_final as RESF

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
    "LIGHTING", "VEHICLES", "STEEL", "FASTENERS",
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
    # Realism pass: silhouette routed through _aerial() (replaces the flat
    # MAT.simple() call), vehicle body/tyre/trim, rickshaw, train livery.
    mats["bldg_far"] = MATF.silhouette()
    mats["vehicle_body"] = MATF.vehicle_body()
    mats["vehicle_tyre"] = MATF.vehicle_tyre()
    mats["vehicle_trim"] = MATF.vehicle_trim()
    mats["rickshaw_body"] = MATF.rickshaw_body()
    mats["rickshaw_hood"] = MATF.rickshaw_hood()
    mats["train_body"] = MATF.train_body()
    mats["train_stripe"] = MATF.train_stripe()
    mats["train_roof"] = MATF.train_roof()
    mats["train_bogie"] = MATF.train_bogie()
    # Detection pass: steel truss / gusset / bolt materials.
    mats["steel_struct"] = MATF.steel_struct()
    mats["steel_gusset"] = MATF.steel_gusset()
    mats["steel_bolt"] = MATF.steel_bolt()
    mats["steel_bolt_corroded"] = MATF.steel_bolt_corroded()
    mats["weld_crack"] = MATF.weld_crack_mat()
    mats["rust_patch"] = MATF.rust_patch_mat()
    mats["coating_failure"] = MATF.coating_failure_mat()
    stats["materials"] = len(bpy.data.materials)

    log("-- lighting (low morning sun, BASELINE) --")
    LT.build(colls["LIGHTING"], log)

    log("-- terrain / river / banks --")
    stats["terrain"] = TF.build(colls, mats, log)

    log("-- road bridge (bridge.py, reused unchanged) --")
    stats["bridge"] = BR.build(colls, mats, log)
    tri_before_steel = ML.scene_tris()

    log("-- detection pass: replacing the concrete main span (x=135-225) "
        "with a steel truss --")
    # Main span is bridge.py's span idx=4 (tag "004") -- the span between
    # pier stations x=135/225 (locked decision: replace, not keep as a
    # second structure). Delete only the girder-line superstructure that a
    # truss actually replaces; the deck slab/wearing/parapet/median and both
    # flanking piers/bearings are bridge.py's own concrete, UNCHANGED, so the
    # truss carries the same deck a girder span would have.
    _MAIN_SPAN_DELETE_PREFIXES = (
        "BR_GIRDER_004_", "BR_DIAPHRAGM_004_", "BR_DRAIN_004_",
        "BR_SERVICE_DUCT_004_",
    )
    _to_delete = [o for o in bpy.data.objects
                 if o.name.startswith(_MAIN_SPAN_DELETE_PREFIXES)]
    n_deleted = len(_to_delete)
    for o in _to_delete:
        md = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if md is not None and md.users == 0:
            bpy.data.meshes.remove(md)
    log(f"  removed : {n_deleted} old main-span girder/diaphragm/drain/duct "
        f"objects (span tag 004)")
    stats["main_span_removed"] = n_deleted

    log("-- steel through-truss main span (steel_final.py) --")
    steel_result = STEELF.build(PF, colls, mats, log=log)
    stats["steel"] = steel_result["stats"]

    log("-- fasteners: bolt manifest, pass 1 (all SOUND) --")
    bolt_manifest = FASTF.build(PF, colls, mats, steel_result["joints"],
                                records_state=None, log=log)
    n_fasteners_objects_pass1 = len(colls["FASTENERS"].objects)
    stats["fasteners_pass1"] = {"count": len(bolt_manifest),
                                "objects": n_fasteners_objects_pass1}

    log("-- steel defects: select bolt-manifest-backed defects (40) --")
    steel_bolt_records, bolt_records_state = DMGS.select_bolt_defects(
        PF, bolt_manifest, log=log)

    log("-- fasteners: bolt manifest, pass 2 (defect states applied) --")
    # PASS 1's geometry (all SOUND, including the WALKWAY_BRACKET/
    # CABLE_CLAMP/HANDRAIL_BASE fittings) is cleared and rebuilt from
    # scratch rather than patched in place -- fasteners_final.build() is a
    # pure function of (params, truss_joints, records_state), so re-running
    # it is simpler and less error-prone than mutating ~4,800 objects by
    # hand, and it is deterministic: pass 2 recreates the exact same
    # objects at the exact same positions, just with the selected states.
    FASTF.clear(colls["FASTENERS"])
    bolt_manifest = FASTF.build(PF, colls, mats, steel_result["joints"],
                                records_state=bolt_records_state, log=log)
    stats["fasteners"] = {"count": len(bolt_manifest),
                          "objects": len(colls["FASTENERS"].objects)}

    log("-- steel defects: bespoke placements (36) --")
    # Runs AFTER pass 2 so it can reference the freshly-rebuilt bracket/
    # clamp/handrail objects (CONDUIT_DETACHED, HANDRAIL_LOOSE) and the
    # untouched gusset plates (GUSSET_DISTORTION) by their real geometry.
    steel_bespoke_records = DMGS.build_bespoke(PF, colls, mats, steel_result,
                                               log=log)
    srecords = steel_bolt_records + steel_bespoke_records
    assert len(srecords) == sum(PF.STEEL_DEFECT_TARGETS.values()), (
        f"steel defect count {len(srecords)} != "
        f"{sum(PF.STEEL_DEFECT_TARGETS.values())}")
    stats["steel_damage"] = {"defects": len(srecords)}
    log(f"  steel   : {len(srecords)} steel defects total (target "
        f"{sum(PF.STEEL_DEFECT_TARGETS.values())})")

    log("-- metro condition: separate GOOD/5yr weathering from road's "
        "POOR/40yr (SPEC S6) --")
    # metro.py consumes the SAME named concrete materials bridge.py/
    # steel_final.py use (mats["concrete_pier"] etc, already weathered at
    # ROAD_WEATHER_STRENGTH above) -- duplicating them and retuning the
    # copies via materials_c's own idempotent set_concrete_weather() (meant
    # for a strength sweep) gives metro its own, independently-weathered
    # instances without touching bridge.py/steel_final.py/metro.py at all,
    # each of which stays a pure function of whatever `mats` dict it is
    # handed.
    _METRO_CONCRETE_KEYS = ("concrete_pier", "concrete_girder",
                            "concrete_parapet", "concrete_low")
    mats_metro = dict(mats)
    for key in _METRO_CONCRETE_KEYS:
        src = mats.get(key)
        if src is None:
            continue
        dup = src.copy()
        dup.name = src.name + "_METRO"
        mats_metro[key] = dup
    _orig_hosts = MATC._CONCRETE_HOSTS
    MATC._CONCRETE_HOSTS = tuple(
        mats_metro[k].name for k in _METRO_CONCRETE_KEYS if k in mats_metro)
    MATC.set_concrete_weather(PF.METRO_WEATHER_STRENGTH, log=log)
    MATC._CONCRETE_HOSTS = _orig_hosts

    log("-- metro viaduct (metro.py, reused unchanged) --")
    stats["metro"] = MB.build(colls, mats_metro, log)

    bpy.context.view_layer.update()

    log("-- waterline staining (river-adjacent piers) --")
    # SPEC.md: "Waterline staining on every pier that meets water." Piers
    # 004/005 (road) flank the main span at x=135/225; the metro's own
    # piers nearest those x-values are the equivalent river-adjacent ones.
    n_stain = MATF.apply_waterline_staining(
        ("BR_PIER_COL_004", "BR_PIER_COL_005"), log=log)
    metro_river_piers = tuple(
        o.name for o in bpy.data.objects
        if o.name.startswith("MB_PIER_COL_") and o.type == "MESH"
        and min(v.co.x for v in o.data.vertices) < 226.0
        and max(v.co.x for v in o.data.vertices) > 134.0)
    # metro's own condition (GOOD/5yr) gets its own, less-weathered stain
    # variant built from its own MAT_CONCRETE_PIER_METRO base, not the
    # road's POOR/40yr one.
    n_stain += MATF.apply_waterline_staining(
        metro_river_piers, log=log,
        name="MAT_CONCRETE_PIER_WATERLINE_METRO",
        base_name=mats_metro["concrete_pier"].name)
    stats["waterline_staining"] = n_stain

    log("-- micro-detail: formwork, tie-holes, honeycombing, chamfers, "
        "parapet posts (HIGH band) --")
    stats["microdetail"] = MDF.build(colls, mats, PF, log=log)

    log("-- road-bridge damage (damage.py, reused unchanged) --")
    # collect_sites() enumerates candidate GIRDER_WEB/GIRDER_BOTTOM_FLANGE/
    # DIAPHRAGM positions from params_final's span geometry alone -- it has
    # no way to know the main span's own girders/diaphragms were just
    # deleted (checkpoint A). Pre-filtering those ~31 candidates out of the
    # pool BEFORE selection was tried and rejected: it starves the shared
    # fallback pool build()'s own per-type loop draws from, and the total
    # achievable count drops to 87 of the required 96 -- worse than the
    # problem it solves. Left unfiltered, ~17 of the 96 land on a host that
    # no longer exists and fail the post-placement ray-cast as orphans --
    # a real, measured, and fully explained consequence of replacing one of
    # seven spans' girders with a truss (see VF11's own truss-span carve-out
    # in validate_final.py), not a bug to paper over here.
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

    bpy.context.view_layer.update()

    log("-- crack relief geometry, >3 mm (reuses damage.py's carve()) --")
    stats["crack_relief"] = MDF.crack_relief(records, DMG, log=log)
    stats["metro_crack_relief"] = MDF.crack_relief(mrecords, DMG, log=log)

    log("-- measured visibility (visibility.py, reused unchanged) --")
    vis_summary = VIS.compute(records, log=log)
    stats["visibility"] = vis_summary
    mvis_summary = VIS.compute(mrecords, log=log)
    stats["metro_visibility"] = mvis_summary
    svis_summary = VIS.compute(srecords, log=log)
    stats["steel_visibility"] = svis_summary

    log("-- resolvability: feature_size_mm -> min_detect_range_m (SPEC S5) --")
    # feature_size_mm was just written by VIS.compute() above, using its OWN
    # (sensor-independent) geometric measurement -- min_detect_range_m
    # applies THIS project's stated sensor figure to it. escalation_reason
    # cannot be finished here: it also needs contrast_c.py's
    # contrast_limited, which is only measured in measure_final.py (phase
    # 2) -- see resolvability_final.py's own docstring.
    RESF.add_min_detect_range(records, PF, log=log)
    RESF.add_min_detect_range(mrecords, PF, log=log)
    RESF.add_min_detect_range(srecords, PF, log=log)

    bpy.context.view_layer.update()

    log("-- traffic on the road deck: real car/truck/bus/rickshaw geometry --")
    # Added AFTER visibility.compute() deliberately: these sit on the deck
    # TOP while every defect host is on the deck underside, girder webs,
    # diaphragms or piers, so they cannot legitimately occlude a defect --
    # but there is no reason to let them anywhere near the ray-casts that
    # matter, so they are built last.
    stats["vehicles"] = VEHF.build(colls, mats, PF, log=log)

    log("-- metro train: real 3-car EMU, parked mid-span, static --")
    rail_top_z = MB.DECK_TOP_Z + MB.SLAB_TRACK_T
    stats["train"] = TRAINF.build(colls, mats, 180.0, MB.Y, rail_top_z,
                                  log=log)

    log("-- drone base: two landing pads, SCANNER + REPAIRER --")
    stats["base"] = BASEF.build(colls, mats, TF.height, log=log)

    log("-- condition gradient: avi_condition / avi_age_years (SPEC S6) --")
    # Run LAST, after every BR_/ST_/MB_ object in the scene exists (the
    # parked EMU is MB_TRAIN_*, built above, after metro.py's own viaduct
    # geometry) -- tagging earlier would silently miss it. BR_/ST_ (road
    # bridge + the steel span that replaced its main-span girders) are one
    # condition; MB_ (metro, including its own train) is the other.
    # Micro-detail's _MD_-prefixed decoration is deliberately left
    # untagged -- it is not a structural member in its own right, and
    # damage.py's own _RAY_SKIP_PREFIXES already keeps it out of every
    # other ground-truth process for the same reason.
    n_road_cond = n_metro_cond = 0
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        if o.name.startswith(("BR_", "ST_")):
            o["avi_condition"] = PF.ROAD_CONDITION
            o["avi_age_years"] = PF.ROAD_AGE_YEARS
            n_road_cond += 1
        elif o.name.startswith("MB_"):
            o["avi_condition"] = PF.METRO_CONDITION
            o["avi_age_years"] = PF.METRO_AGE_YEARS
            n_metro_cond += 1
    stats["condition_gradient"] = {
        "road_and_steel_members": n_road_cond, "road_condition": PF.ROAD_CONDITION,
        "road_age_years": PF.ROAD_AGE_YEARS, "metro_members": n_metro_cond,
        "metro_condition": PF.METRO_CONDITION,
        "metro_age_years": PF.METRO_AGE_YEARS}
    log(f"  condition: {n_road_cond} road/steel members "
        f"{PF.ROAD_CONDITION}/{PF.ROAD_AGE_YEARS}y, {n_metro_cond} metro "
        f"members {PF.METRO_CONDITION}/{PF.METRO_AGE_YEARS}y")

    bpy.context.view_layer.update()

    log("-- triangle / object accounting --")
    n_obj_total = len(bpy.data.objects)
    n_tri = ML.scene_tris()
    stats["scene"] = {"objects": n_obj_total, "triangles": n_tri,
                      "materials": len(bpy.data.materials),
                      "collections": len(bpy.data.collections),
                      "triangles_before_steel": tri_before_steel,
                      "triangles_after_steel": n_tri,
                      "triangles_added_by_steel_pass": n_tri - tri_before_steel}
    log(f"  objects : {n_obj_total}")
    log(f"  triangles: {n_tri} ({tri_before_steel} before the steel/"
        f"fastener/defect pass, +{n_tri - tri_before_steel})")

    log("-- saving --")
    scene_dir = os.path.join(ROOT, "scene")
    os.makedirs(scene_dir, exist_ok=True)
    blend_path = os.path.join(scene_dir, "SIH_AVIAN_FINAL.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    log(f"  blend   : {blend_path}")

    handoff = {
        "records": records,
        "mrecords": mrecords,
        "srecords": srecords,
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

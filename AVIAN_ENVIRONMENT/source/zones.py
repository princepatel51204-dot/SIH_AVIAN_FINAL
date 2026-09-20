"""Inspection sectors, UAV airspace volumes, scenario markers and cameras.

NOTHING IN THIS MODULE IS A UAV.
This phase is the environment only. What is built here is the *airspace
definition* a later flight system would read: named, measurable, non-rendering
volumes with machine-readable bounds attached as custom properties. No
aircraft, no flight controller, no planner.

Why volumes and not just numbers in a config file: a volume in the scene can be
checked against the geometry that actually exists. A number in a file cannot.
The validation pass in validate.py tests every airspace volume for intersection
with real structure, which is how a clearance error gets caught before a
simulator ever runs.

VOLUME CLASSES
--------------
  AVI_AIRSPACE_SAFE          transit band well above the deck. Free flight.
  AVI_AIRSPACE_INSPECTION    working shell around the superstructure, one per
                             sector, standoff AIRSPACE_INSPECTION_OFFSET.
  AVI_AIRSPACE_UNDERBRIDGE   under-deck volume between soffit and the clearance
                             floor. GNSS-degraded, confined, one per sector.
  AVI_AIRSPACE_RIVER         over-water working band. No landing surface.
  AVI_AIRSPACE_RESTRICTED    keep-out shell around each pier bent and around
                             the live carriageway.
  AVI_AIRSPACE_RETURN        launch and recovery corridors at each end of the
                             research zone, ground to transit altitude.

All volumes: hide_render = True, wireframe display, and every bound written to
custom properties so they can be extracted without parsing meshes.
"""
from __future__ import annotations
import math
import bpy
from mathutils import Vector

import params as P
import meshlib as ML
import terrain as TR


ZONE_CLASSES = [
    "AVI_AIRSPACE_SAFE",
    "AVI_AIRSPACE_INSPECTION",
    "AVI_AIRSPACE_UNDERBRIDGE",
    "AVI_AIRSPACE_RIVER",
    "AVI_AIRSPACE_RESTRICTED",
    "AVI_AIRSPACE_RETURN",
]


# ---------------------------------------------------------------------------
def _volume(name, x0, x1, y0, y1, z0, z1, coll, zone_class, props=None):
    """An axis-aligned airspace box. Never rendered, always measurable."""
    ob = ML.box(name,
                (x1 - x0, y1 - y0, z1 - z0),
                (0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
                coll)
    ob.display_type = "WIRE"
    ob.hide_render = True
    ob.visible_camera = False
    ob.visible_shadow = False
    ob.visible_diffuse = False
    ob.visible_glossy = False
    ob.visible_transmission = False
    ob.visible_volume_scatter = False
    d = {
        "avi_kind": "airspace_volume",
        "avi_zone_class": zone_class,
        "avi_x_min_m": round(x0, 3), "avi_x_max_m": round(x1, 3),
        "avi_y_min_m": round(y0, 3), "avi_y_max_m": round(y1, 3),
        "avi_z_min_m": round(z0, 3), "avi_z_max_m": round(z1, 3),
        "avi_volume_m3": round((x1 - x0) * (y1 - y0) * (z1 - z0), 1),
    }
    if props:
        d.update(props)
    ML.set_custom(ob, d)
    return ob


def _marker(name, pos, coll, props):
    """A point of interest. An EMPTY: zero geometry, zero render cost."""
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_type = "SPHERE"
    ob.empty_display_size = 2.5
    ob.location = pos
    coll.objects.link(ob)
    ML.set_custom(ob, props)
    return ob


# ---------------------------------------------------------------------------
def build_sectors(coll, log=print):
    """Six inspection sectors across the research zone.

    Each sector gets a bounding volume covering the full structural envelope
    over its 150 m of chainage -- soffit-minus-clearance up to transit height,
    deck width plus working standoff. This is the region one aircraft owns.
    """
    out = []
    half_w = P.DECK_WIDTH / 2.0 + P.AIRSPACE_INSPECTION_OFFSET
    for i, nm in enumerate(P.SECTOR_NAMES):
        x0 = P.RESEARCH_X0 + i * P.SECTOR_LENGTH
        x1 = x0 + P.SECTOR_LENGTH
        xm = 0.5 * (x0 + x1)
        # lowest structure in the sector: the deepest pier foundation level
        gz = min(P.ground_z(x, 0.0) for x in (x0, xm, x1))
        soffit = min(P.soffit_z(x) for x in (x0, xm, x1))
        deck = max(P.deck_top_z(x) for x in (x0, xm, x1))
        ob = _volume(f"SECTOR_{nm[-1]}_VOLUME", x0, x1,
                     -half_w, half_w,
                     gz - 2.0, deck + P.AIRSPACE_SAFE_Z[1] - deck + 5.0,
                     coll, "SECTOR",
                     {"avi_kind": "inspection_sector",
                      "avi_sector": nm,
                      "avi_sector_index": i + 1,
                      "avi_chainage_start_m": round(x0, 1),
                      "avi_chainage_end_m": round(x1, 1),
                      "avi_length_m": round(P.SECTOR_LENGTH, 1),
                      "avi_deck_top_z_m": round(deck, 2),
                      "avi_soffit_z_m": round(soffit, 2),
                      "avi_ground_z_m": round(gz, 2),
                      "avi_section": P.section_at(xm),
                      "avi_over_water": bool(
                          abs(xm - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2)})
        _marker(f"SECTOR_{nm[-1]}_ORIGIN", (x0, 0.0, deck + 2.0), coll,
                {"avi_kind": "sector_origin", "avi_sector": nm})
        out.append(ob)
    log(f"  sectors : {len(out)} x {P.SECTOR_LENGTH:.0f} m "
        f"({P.RESEARCH_X0:.0f} - {P.RESEARCH_X1:.0f})")
    return out


def build_airspace(coll, log=print):
    """Named UAV airspace volumes. Definitions only -- no aircraft."""
    n = {c: 0 for c in ZONE_CLASSES}
    half_w = P.DECK_WIDTH / 2.0 + P.AIRSPACE_INSPECTION_OFFSET
    x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1

    # ---- 1. SAFE transit band -------------------------------------------
    # Sits above the tallest structure in the zone by a clear margin. A UAV in
    # here cannot hit the bridge whatever it does.
    deck_max = max(P.deck_top_z(x) for x in
                   range(int(x0r) - 200, int(x1r) + 200, 10))
    z0 = max(P.AIRSPACE_SAFE_Z[0], deck_max + 20.0)
    _volume("AVI_AIRSPACE_SAFE_TRANSIT", x0r - 200, x1r + 200,
            -260.0, 260.0, z0, max(P.AIRSPACE_SAFE_Z[1], z0 + 40.0),
            coll, "AVI_AIRSPACE_SAFE",
            {"avi_rule": "free transit, no structure within the volume",
             "avi_gnss": "nominal",
             "avi_min_clearance_above_deck_m": round(z0 - deck_max, 1)})
    n["AVI_AIRSPACE_SAFE"] += 1

    # ---- 2. INSPECTION shells, one per sector ---------------------------
    for i, nm in enumerate(P.SECTOR_NAMES):
        a = x0r + i * P.SECTOR_LENGTH
        b = a + P.SECTOR_LENGTH
        xm = 0.5 * (a + b)
        soffit = min(P.soffit_z(x) for x in (a, xm, b))
        deck = max(P.deck_top_z(x) for x in (a, xm, b))
        _volume(f"AVI_AIRSPACE_INSPECTION_{nm}", a, b,
                -half_w, half_w,
                soffit - P.AIRSPACE_UNDER_CLEAR - 6.0,
                deck + P.AIRSPACE_INSPECTION_OFFSET,
                coll, "AVI_AIRSPACE_INSPECTION",
                {"avi_sector": nm,
                 "avi_rule": "sensor standoff work, structure inside volume",
                 "avi_standoff_m": P.AIRSPACE_INSPECTION_OFFSET,
                 "avi_gnss": "degraded near structure"})
        n["AVI_AIRSPACE_INSPECTION"] += 1

        # ---- 3. UNDERBRIDGE, one per sector -----------------------------
        # The hard one. Between the girder soffit and the clearance floor,
        # roofed by structure: no sky, no GNSS, and the only light is bounce.
        floor = max(P.ground_z(xm, 0.0), P.RIVER_WATER_Z) + 2.0
        top = soffit - P.AIRSPACE_UNDER_CLEAR
        if top > floor + 2.0:
            _volume(f"AVI_AIRSPACE_UNDERBRIDGE_{nm}", a, b,
                    -P.DECK_WIDTH / 2.0 - 4.0, P.DECK_WIDTH / 2.0 + 4.0,
                    floor, top, coll, "AVI_AIRSPACE_UNDERBRIDGE",
                    {"avi_sector": nm,
                     "avi_rule": "confined under-deck volume",
                     "avi_clearance_below_soffit_m": P.AIRSPACE_UNDER_CLEAR,
                     "avi_headroom_m": round(top - floor, 2),
                     "avi_gnss": "denied",
                     "avi_lighting": "indirect bounce only"})
            n["AVI_AIRSPACE_UNDERBRIDGE"] += 1

    # ---- 4. RIVER working band ------------------------------------------
    half_r = P.RIVER_WIDTH / 2.0
    _volume("AVI_AIRSPACE_RIVER_WORKING",
            max(x0r, P.RIVER_CENTRE_X - half_r),
            min(x1r, P.RIVER_CENTRE_X + half_r),
            -300.0, 300.0,
            P.RIVER_WATER_Z + P.AIRSPACE_RIVER_Z[0],
            P.RIVER_WATER_Z + P.AIRSPACE_RIVER_Z[1],
            coll, "AVI_AIRSPACE_RIVER",
            {"avi_rule": "over open water, no emergency landing surface",
             "avi_water_surface_z_m": P.RIVER_WATER_Z,
             "avi_hazard": "specular reflection degrades optical flow and "
                           "downward depth sensing",
             "avi_gnss": "nominal, multipath from water"})
    n["AVI_AIRSPACE_RIVER"] += 1

    # ---- 5. RESTRICTED keep-out shells ----------------------------------
    # Around every pier bent in the research zone, and over the live
    # carriageway. These are the volumes a planner must treat as solid.
    for i, (x, kind) in enumerate(P.pier_stations()):
        if not (x0r - 10 <= x <= x1r + 10):
            continue
        gz = P.ground_z(x, 0.0)
        pad = P.AIRSPACE_RESTRICTED_PAD
        r = (P.PIER_COL_D_RIVER if kind == "river" and gz < P.RIVER_WATER_Z
             else P.PIER_COL_D) / 2.0
        hw = P.PIER_COL_SPACING / 2.0 + r + pad
        cap_top = P.soffit_z(x) - P.BEARING_H
        _volume(f"AVI_AIRSPACE_RESTRICTED_PIER_{i+1:03d}",
                x - P.PIER_CAP_W / 2 - pad, x + P.PIER_CAP_W / 2 + pad,
                -max(hw, P.PIER_CAP_L / 2 + pad),
                max(hw, P.PIER_CAP_L / 2 + pad),
                min(gz, P.RIVER_WATER_Z) - 2.0, cap_top + pad,
                coll, "AVI_AIRSPACE_RESTRICTED",
                {"avi_rule": "keep-out shell around pier bent",
                 "avi_pad_m": pad,
                 "avi_pier_station_m": round(x, 1),
                 "avi_pier_kind": kind,
                 "avi_sector": P.sector_at(x) or "OUTSIDE_RESEARCH_ZONE"})
        n["AVI_AIRSPACE_RESTRICTED"] += 1

    # live carriageway: traffic, and the one place a fall is unacceptable
    _volume("AVI_AIRSPACE_RESTRICTED_CARRIAGEWAY", x0r, x1r,
            -P.DECK_WIDTH / 2.0, P.DECK_WIDTH / 2.0,
            min(P.deck_top_z(x) for x in (x0r, x1r)) - 1.0,
            max(P.deck_top_z(x) for x in (x0r, x1r)) + 8.0,
            coll, "AVI_AIRSPACE_RESTRICTED",
            {"avi_rule": "live traffic, overflight prohibited",
             "avi_hazard": "vehicles, and any dropped mass is unacceptable"})
    n["AVI_AIRSPACE_RESTRICTED"] += 1

    # ---- 6. RETURN corridors --------------------------------------------
    # Launch and recovery at each end of the research zone, on dry land clear
    # of the structure, running ground to transit altitude.
    # Offset well off the centreline: a corridor centred on the alignment runs
    # straight through the viaduct, which the airspace validation catches as an
    # intersection with the bearings. A launch lane has to be beside the
    # structure, not under it.
    for tag, xs, ys in (("SOUTH", x0r - 120.0, -95.0),
                        ("NORTH", x1r + 120.0, 95.0)):
        gz = TR.height(xs, ys)
        _volume(f"AVI_AIRSPACE_RETURN_{tag}", xs - 30.0, xs + 30.0,
                ys - 30.0, ys + 30.0, gz + 1.0, z0 + 10.0,
                coll, "AVI_AIRSPACE_RETURN",
                {"avi_rule": "launch and recovery corridor, ground to transit",
                 "avi_ground_z_m": round(gz, 2),
                 "avi_end": tag,
                 "avi_centreline_offset_m": abs(ys),
                 "avi_over_water": bool(
                     abs(xs - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2)})
        _marker(f"AVI_LANDING_PAD_{tag}", (xs, ys, gz + 0.2), coll,
                {"avi_kind": "landing_pad", "avi_end": tag,
                 "avi_surface": "prepared hardstanding"})
        n["AVI_AIRSPACE_RETURN"] += 1

    total = sum(n.values())
    log(f"  airspace: {total} volumes across {len(ZONE_CLASSES)} classes "
        + ", ".join(f"{k.split('_')[-1]}={v}" for k, v in n.items()))
    return n


# ---------------------------------------------------------------------------
# NINE DIFFICULT-INSPECTION SCENARIOS
# ---------------------------------------------------------------------------
def build_scenarios(coll, log=print):
    """Nine marked locations where inspection is genuinely hard.

    Each is a real place in this model with a real reason, not a label dropped
    on open air. They exist so a later system can be tested on the cases that
    break naive approaches rather than only on the easy open faces.
    """
    xm_river = P.RIVER_CENTRE_X
    ps = [x for x, k in P.pier_stations()
          if P.RESEARCH_X0 <= x <= P.RESEARCH_X1]
    # tallest pier in the zone
    tall_x = max(ps, key=lambda x: P.deck_top_z(x) - P.ground_z(x, 0.0))
    # a mid-span point over deep water
    mid_x = min(ps, key=lambda x: abs(x - xm_river))
    y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0

    S = []

    def add(idx, name, pos, difficulty, why, sensing, mitigation):
        nm = f"SCENARIO_{idx:02d}_{name}"
        _marker(nm, pos, coll, {
            "avi_kind": "inspection_scenario",
            "avi_scenario_id": idx,
            "avi_scenario": name,
            "avi_difficulty": difficulty,        # 1..5
            "avi_challenge": why,
            "avi_sensing_problem": sensing,
            "avi_suggested_mitigation": mitigation,
            "avi_sector": P.sector_at(pos[0]) or "OUTSIDE_RESEARCH_ZONE",
            "avi_section": P.section_at(pos[0]),
        })
        S.append(nm)

    soff = P.soffit_z
    add(1, "CONFINED_GIRDER_BAY",
        (ps[2] + 18.0, y_g0 + P.GIRDER_SPACING * 2.5,
         soff(ps[2] + 18.0) + 0.8),
        5,
        "bay between two 4 m girders closed at both ends by diaphragms; "
        "clear width 3.8 m, roofed by the deck slab",
        "no sky view, no GNSS, propeller downwash recirculates off three "
        "surfaces, and the walls are inside the minimum focus range of a "
        "long-baseline stereo rig",
        "short-baseline stereo or structured light, wall-referenced "
        "odometry, reduced thrust profile")

    add(2, "GNSS_DENIED_UNDERDECK",
        (mid_x + 45.0, 0.0, soff(mid_x + 45.0) - 4.0),
        4,
        "mid-channel under-deck volume, 24 m of concrete overhead and open "
        "water below",
        "GNSS fully occluded by the deck, no ground texture for downward "
        "optical flow -- the water moves",
        "visual-inertial odometry referenced upward to the soffit, not "
        "downward")

    add(3, "WATERLINE_REFLECTION_BAND",
        (mid_x, P.PIER_COL_SPACING / 2.0 + 2.5, P.RIVER_WATER_Z + 1.2),
        4,
        "pier waterline band, the highest-value defect zone on a river pier",
        "specular sun glint off moving water saturates the sensor and "
        "produces phantom depth returns; the surface is also wet and dark",
        "polarising filter, oblique approach away from the specular lobe, "
        "exposure bracketing")

    add(4, "TALL_PIER_WIND_EXPOSURE",
        (tall_x, P.PIER_COL_SPACING / 2.0 + 3.0,
         P.ground_z(tall_x, 0.0) + 0.72 *
         (P.deck_top_z(tall_x) - P.ground_z(tall_x, 0.0))),
        4,
        f"column face at {0.72*(P.deck_top_z(tall_x)-P.ground_z(tall_x,0.0)):.0f} m "
        "above ground on the tallest pier in the zone",
        "channelled wind across the valley plus vortex shedding off the "
        "column; station-keeping error directly becomes image blur",
        "windward standoff increase, gust-tolerant position hold, shorter "
        "exposure with higher gain")

    add(5, "OCCLUDED_BEARING_SEAT",
        (ps[3], y_g0 + 0.0, soff(ps[3]) - 0.15),
        5,
        "bearing seat: a 0.7 m recess enclosed by the pier cap below, the "
        "girder above and the diaphragm behind",
        "reachable from a single narrow cone of viewpoints; a UAV that "
        "cannot achieve that cone returns no data at all rather than bad data",
        "articulated or gimbal-forward sensor, approach along the cap axis")

    add(6, "LOW_CONTRAST_SHADOW_CRACK",
        (ps[4] + 26.0, y_g0 + P.GIRDER_SPACING * 1.0 + 0.15,
         soff(ps[4] + 26.0) + 1.4),
        5,
        "hairline cracking, 0.05-0.20 mm, on a web face that is in permanent "
        "shadow under the deck",
        "the defect signal is a few grey levels wide against aggregate "
        "mottle of similar amplitude -- it is a contrast problem, not a "
        "resolution problem",
        "on-board illumination at grazing incidence, HDR capture, and "
        "detection thresholds tuned per illumination condition")

    jx = ps[min(4, len(ps) - 1)]
    add(7, "EXPANSION_JOINT_UNDERSIDE",
        (jx + 0.4, P.DECK_WIDTH / 2.0 - 2.0, soff(jx) + 0.5),
        3,
        "underside of an expansion joint: chronic leakage path, so the "
        "surrounding concrete is the most reliably deteriorated on the deck",
        "wet and stained surface changes the reflectance model; the joint gap "
        "itself reads as an infinite-depth void to a depth sensor",
        "treat gaps as no-data rather than as geometry, and classify staining "
        "separately from cracking")

    add(8, "OVERHEAD_DECK_SOFFIT",
        (ps[1] + 30.0, y_g0 + P.GIRDER_SPACING * 1.5,
         soff(ps[1] + 30.0) + 1.8),
        4,
        "flat deck soffit directly overhead, 0.9 m above the aircraft",
        "the inspection surface is where a UAV has no sensor and where "
        "downwash reflects straight back; ground effect against a ceiling is "
        "destabilising in the opposite sense to ground effect below",
        "upward-facing sensor, ceiling-effect-aware controller, fixed "
        "standoff hold")

    add(9, "PARAPET_EDGE_TRAFFIC_EXPOSURE",
        (ps[2] + 60.0, P.DECK_WIDTH / 2.0 + 1.2,
         P.deck_top_z(ps[2] + 60.0) + 0.6),
        3,
        "outer parapet face alongside live traffic on a crest",
        "vehicle-induced gusts arrive at traffic speed and are not "
        "predictable from ambient wind; a lateral excursion of 1.2 m puts the "
        "aircraft over the carriageway",
        "work from the outboard side only, hard lateral geofence at the "
        "parapet line, abort on gust threshold")

    log(f"  scenarios: {len(S)} difficult-inspection cases marked")
    return S


# ---------------------------------------------------------------------------
# CAMERAS
# ---------------------------------------------------------------------------
CAMERA_SPECS = [
    # (name, eye, target, lens_mm, purpose)
    ("CAMERA_01_GLOBAL_CITY",
     (-1500.0, -3100.0, 1250.0), (2100.0, 0.0, 40.0), 38.0,
     "whole city and corridor in context, from the south-east at altitude"),
    ("CAMERA_02_BRIDGE_FULL_LENGTH",
     (-700.0, -520.0, 300.0), (2400.0, 0.0, 20.0), 55.0,
     "the full 4.5 km corridor foreshortened along its length"),
    ("CAMERA_03_RIVER_CROSSING",
     (2100.0, -620.0, 42.0), (2100.0, 0.0, 14.0), 50.0,
     "the 90 m main span and river piers broadside from the water"),
    ("CAMERA_04_RESEARCH_ZONE",
     (2100.0, -700.0, 330.0), (2100.0, 0.0, 18.0), 40.0,
     "the 900 m high-detail research zone and its six sectors"),
    ("CAMERA_05_UNDERBRIDGE",
     (1880.0, 9.5, 9.0), (2080.0, -2.0, 12.5), 24.0,
     "under-deck view along the girder bays: the primary inspection volume"),
    ("CAMERA_06_DEFECT_CLOSEUP",
     (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 50.0,
     "inspection standoff on a representative defect; placed at build time "
     "from the ground truth"),
    ("CAMERA_07_FULL_INFRASTRUCTURE",
     (-2600.0, -2600.0, 900.0), (2300.0, 0.0, 30.0), 28.0,
     "everything: corridor, river, city and horizon in one frame"),
]


def build_cameras(coll, defect_records=None, log=print):
    """Seven named cameras. Camera 06 is aimed from the ground truth.

    clip_end is set explicitly on every camera. Blender's default is 1000 m,
    which silently cuts a 4.5 km corridor off at the first kilometre and looks
    convincingly like atmospheric haze. That cost several renders here before
    it was spotted.
    """
    cams = []
    for name, eye, tgt, lens, purpose in CAMERA_SPECS:
        if name == "CAMERA_06_DEFECT_CLOSEUP":
            eye, tgt, lens = _closeup_from(defect_records)
        cd = bpy.data.cameras.new(name)
        cd.lens = lens
        cd.clip_start = 0.05
        cd.clip_end = 60000.0
        ob = bpy.data.objects.new(name, cd)
        coll.objects.link(ob)
        ob.location = eye
        ML.look_at(ob, tgt)
        ML.set_custom(ob, {
            "avi_kind": "validation_camera",
            "avi_purpose": purpose,
            "avi_focal_length_mm": lens,
            "avi_target_m": [round(v, 2) for v in tgt],
            "avi_range_m": round((Vector(eye) - Vector(tgt)).length, 1),
        })
        cams.append(ob)
    log(f"  cameras : {len(cams)} named views")
    return cams


def _closeup_from(records):
    """Aim the close-up camera at a real, interesting defect.

    Preference order: exposed rebar (the most visually distinctive), then any
    severity-4 geometric defect, then anything at all. Falling back to a fixed
    coordinate would put the camera at a location that may hold nothing.
    """
    default = ((1900.0, 3.0, 12.0), (1902.0, 0.0, 11.5), 50.0)
    if not records:
        return default
    cand = ([r for r in records if r["type"] == "REBAR_EXPOSED"]
            or [r for r in records
                if r["severity"] == 4 and r["representation"] == "GEOMETRY"]
            or [r for r in records if r["severity"] >= 3]
            or records)
    r = cand[0]
    p = Vector(r["position_m"])
    n = Vector(r["surface_normal"])
    side = n.cross(Vector((0, 0, 1)))
    if side.length < 1e-3:
        side = Vector((1, 0, 0))
    side.normalize()
    eye = p + n * 1.35 + side * 0.55 - Vector((0.0, 0.0, 0.30))
    return (tuple(eye), tuple(p), 50.0)


# ---------------------------------------------------------------------------
def build(colls, defect_records=None, log=print):
    sec = build_sectors(colls["SECTORS"], log)
    air = build_airspace(colls["UAV_AIRSPACE"], log)
    scn = build_scenarios(colls["SCENARIOS"], log)
    cam = build_cameras(colls["CAMERAS"], defect_records, log)
    return {"sectors": len(sec), "airspace": air,
            "scenarios": len(scn), "cameras": len(cam)}


def export_zone_manifest(path):
    """Dump every airspace volume, sector, scenario and camera to JSON."""
    import json
    out = {"sectors": [], "airspace": [], "scenarios": [], "cameras": []}
    for ob in bpy.data.objects:
        k = ob.get("avi_kind")
        if k is None:
            continue
        rec = {key: (list(ob[key]) if hasattr(ob[key], "__len__")
                     and not isinstance(ob[key], str) else ob[key])
               for key in ob.keys() if key.startswith("avi_")}
        rec["name"] = ob.name
        rec["location_m"] = [round(v, 3) for v in ob.location]
        if k == "inspection_sector":
            out["sectors"].append(rec)
        elif k == "airspace_volume":
            out["airspace"].append(rec)
        elif k == "inspection_scenario":
            out["scenarios"].append(rec)
        elif k == "validation_camera":
            out["cameras"].append(rec)
    for v in out.values():
        v.sort(key=lambda r: r["name"])
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return {k: len(v) for k, v in out.items()}

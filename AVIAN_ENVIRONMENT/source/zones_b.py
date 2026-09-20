"""REV-B additions to the zone system.

The REV-A airspace (zones.py) is preserved untouched and still built. This
module ADDS the eight-class REV-B airspace taxonomy alongside it, and replaces
the nine scenario markers with the sixteen the revision requires -- superset,
not substitute: every REV-A scenario survives with its original name and ID.

Why a second module rather than an edit: zones.py is what produced the
validated REV-A scene, and the cheapest way to guarantee REV-A behaviour is
preserved is to not touch the code that produced it.
"""
from __future__ import annotations
import math

import bpy
from mathutils import Vector

import params as P
import meshlib as ML
import zones as Z


# REV-B airspace taxonomy. Each carries the operating envelope a planner
# needs, not just a bounding box.
#   speed        m/s ceiling inside the volume
#   standoff     recommended sensor working distance from structure
#   clearance    hard minimum to any structure
#   risk         LOW / MEDIUM / HIGH / CRITICAL
REVB_CLASSES = {
    "TRANSIT_AIRSPACE": {
        "speed": 8.0, "standoff": None, "clearance": 15.0, "risk": "LOW",
        "gps": "HIGH",
        "rule": "free transit above all structure; no inspection work"},
    "DECK_INSPECTION_AIRSPACE": {
        "speed": 2.5, "standoff": 6.0, "clearance": 4.0, "risk": "MEDIUM",
        "gps": "HIGH",
        "rule": "deck surface, parapet and expansion joint work from above"},
    "UNDERSIDE_INSPECTION_AIRSPACE": {
        "speed": 1.2, "standoff": 2.5, "clearance": 1.5, "risk": "HIGH",
        "gps": "LOW",
        "rule": "soffit and girder work; sky occluded by the deck"},
    "PIER_INSPECTION_AIRSPACE": {
        "speed": 1.5, "standoff": 3.0, "clearance": 2.0, "risk": "MEDIUM",
        "gps": "MEDIUM",
        "rule": "column and pier-cap circumnavigation"},
    "RIVER_INSPECTION_AIRSPACE": {
        "speed": 2.0, "standoff": 3.5, "clearance": 2.5, "risk": "CRITICAL",
        "gps": "MEDIUM",
        "rule": "over water; no landing surface, specular reflection"},
    "CONFINED_INSPECTION_AIRSPACE": {
        "speed": 0.6, "standoff": 1.2, "clearance": 0.8, "risk": "CRITICAL",
        "gps": "DENIED",
        "rule": "inside a girder bay closed by diaphragms on both ends"},
    "EMERGENCY_RETURN_AIRSPACE": {
        "speed": 6.0, "standoff": None, "clearance": 8.0, "risk": "LOW",
        "gps": "HIGH",
        "rule": "reserved corridor, kept clear for abort and recovery"},
    "MULTI_UAV_SEPARATION_ZONE": {
        "speed": 1.0, "standoff": None, "clearance": 10.0, "risk": "HIGH",
        "gps": "HIGH",
        "rule": "buffer between adjacent sector workspaces; one aircraft at "
                "a time, by protocol"},
}


def _vol(name, x0, x1, y0, y1, z0, z1, coll, cls, extra=None):
    spec = REVB_CLASSES[cls]
    props = {
        "avi_kind": "airspace_volume",
        "avi_object_type": "AIRSPACE",
        "avi_object_id": name,
        "avi_zone_class": cls,
        "avi_taxonomy": "REV_B",
        "avi_max_speed_mps": spec["speed"],
        "avi_recommended_standoff_m": spec["standoff"],
        "avi_min_structural_clearance_m": spec["clearance"],
        "avi_risk_level": spec["risk"],
        "avi_gps_quality": spec["gps"],
        "avi_rule": spec["rule"],
        "avi_max_altitude_m": round(z1, 2),
    }
    if extra:
        props.update(extra)
    return Z._volume(name, x0, x1, y0, y1, z0, z1, coll, cls, props)


# ---------------------------------------------------------------------------
def build_airspace_b(coll, log=print):
    """The eight-class REV-B airspace, built alongside the REV-A volumes."""
    n = {c: 0 for c in REVB_CLASSES}
    x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1
    half_w = P.DECK_WIDTH / 2.0
    stand = half_w + P.AIRSPACE_INSPECTION_OFFSET
    deck_max = max(P.deck_top_z(x) for x in range(int(x0r), int(x1r), 10))
    transit_z0 = max(P.AIRSPACE_SAFE_Z[0], deck_max + 20.0)

    _vol("AVI_TRANSIT_AIRSPACE_MAIN", x0r - 220, x1r + 220, -280, 280,
         transit_z0, transit_z0 + 45.0, coll, "TRANSIT_AIRSPACE",
         {"avi_clearance_above_deck_m": round(transit_z0 - deck_max, 1)})
    n["TRANSIT_AIRSPACE"] += 1

    for i, nm in enumerate(P.SECTOR_NAMES):
        a = x0r + i * P.SECTOR_LENGTH
        b = a + P.SECTOR_LENGTH
        xm = 0.5 * (a + b)
        L = nm[-1]
        soffit = min(P.soffit_z(x) for x in (a, xm, b))
        deck = max(P.deck_top_z(x) for x in (a, xm, b))
        gz = P.ground_z(xm, 0.0)
        over_water = abs(xm - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2.0

        _vol(f"AVI_DECK_INSPECTION_{nm}", a, b, -stand, stand,
             deck + 1.5, deck + 14.0, coll, "DECK_INSPECTION_AIRSPACE",
             {"avi_sector": nm})
        n["DECK_INSPECTION_AIRSPACE"] += 1

        floor = max(gz, P.RIVER_WATER_Z) + 2.0
        top = soffit - P.AIRSPACE_UNDER_CLEAR
        if top > floor + 2.0:
            _vol(f"AVI_UNDERSIDE_INSPECTION_{nm}", a, b,
                 -half_w - 4.0, half_w + 4.0, floor, top, coll,
                 "UNDERSIDE_INSPECTION_AIRSPACE",
                 {"avi_sector": nm,
                  "avi_headroom_m": round(top - floor, 2),
                  "avi_over_water": over_water})
            n["UNDERSIDE_INSPECTION_AIRSPACE"] += 1

        # Confined bays: the volume BETWEEN two girders, closed by
        # diaphragms. Narrow on purpose -- this is the hardest airspace in
        # the model and its dimensions come from the real girder spacing.
        y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0
        bay_y = y_g0 + P.GIRDER_SPACING * (1.5 + (i % 3))
        depth = P.girder_depth_for_span(P.span_at(xm))
        _vol(f"AVI_CONFINED_BAY_{nm}", xm - 9.0, xm + 9.0,
             bay_y - (P.GIRDER_SPACING - P.GIRDER_TOP_FLANGE_W) / 2 + 0.2,
             bay_y + (P.GIRDER_SPACING - P.GIRDER_TOP_FLANGE_W) / 2 - 0.2,
             soffit + 0.4, soffit + max(0.8, depth - 0.6), coll,
             "CONFINED_INSPECTION_AIRSPACE",
             {"avi_sector": nm,
              "avi_bay_width_m": round(P.GIRDER_SPACING
                                       - P.GIRDER_TOP_FLANGE_W, 2),
              "avi_note": "closed above by the slab and at both ends by "
                          "diaphragms; propeller downwash recirculates"})
        n["CONFINED_INSPECTION_AIRSPACE"] += 1

        # Multi-UAV separation buffer at each internal sector boundary
        if i > 0:
            _vol(f"AVI_SEPARATION_{P.SECTOR_NAMES[i-1][-1]}_{L}",
                 a - 10.0, a + 10.0, -stand, stand,
                 max(gz, P.RIVER_WATER_Z), transit_z0 + 45.0, coll,
                 "MULTI_UAV_SEPARATION_ZONE",
                 {"avi_between": [P.SECTOR_NAMES[i - 1], nm],
                  "avi_buffer_width_m": 20.0})
            n["MULTI_UAV_SEPARATION_ZONE"] += 1

    # Pier inspection shells, research zone only
    for i, (x, kind) in enumerate(P.pier_stations()):
        if not (x0r <= x <= x1r):
            continue
        gz = P.ground_z(x, 0.0)
        r = (P.PIER_COL_D_RIVER if kind == "river" and gz < P.RIVER_WATER_Z
             else P.PIER_COL_D) / 2.0
        pad = 3.0
        hw = P.PIER_COL_SPACING / 2.0 + r + pad
        _vol(f"AVI_PIER_INSPECTION_{i+1:03d}",
             x - P.PIER_CAP_W / 2 - pad, x + P.PIER_CAP_W / 2 + pad,
             -hw, hw, max(gz, P.RIVER_WATER_Z) + 0.5,
             P.soffit_z(x) - 0.5, coll, "PIER_INSPECTION_AIRSPACE",
             {"avi_pier_station_m": round(x, 1),
              "avi_pier_kind": kind,
              "avi_sector": P.sector_at(x) or "OUTSIDE_RESEARCH_ZONE",
              "avi_in_water": bool(gz < P.RIVER_WATER_Z)})
        n["PIER_INSPECTION_AIRSPACE"] += 1

    # River working band
    half_r = P.RIVER_WIDTH / 2.0
    _vol("AVI_RIVER_INSPECTION_MAIN",
         max(x0r, P.RIVER_CENTRE_X - half_r),
         min(x1r, P.RIVER_CENTRE_X + half_r), -320.0, 320.0,
         P.RIVER_WATER_Z + P.AIRSPACE_RIVER_Z[0],
         P.RIVER_WATER_Z + P.AIRSPACE_RIVER_Z[1], coll,
         "RIVER_INSPECTION_AIRSPACE",
         {"avi_water_surface_z_m": P.RIVER_WATER_Z,
          "avi_hazard": "specular reflection; no emergency landing surface "
                        "for 620 m"})
    n["RIVER_INSPECTION_AIRSPACE"] += 1

    # Emergency return corridors, offset off the centreline on dry land
    for tag, xs, ys in (("SOUTH", x0r - 120.0, -95.0),
                        ("NORTH", x1r + 120.0, 95.0)):
        gz = P.ground_z(xs, ys)
        _vol(f"AVI_EMERGENCY_RETURN_{tag}", xs - 30.0, xs + 30.0,
             ys - 30.0, ys + 30.0, gz + 1.0, transit_z0 + 12.0, coll,
             "EMERGENCY_RETURN_AIRSPACE",
             {"avi_end": tag, "avi_ground_z_m": round(gz, 2),
              "avi_reserved": True})
        n["EMERGENCY_RETURN_AIRSPACE"] += 1

    total = sum(n.values())
    log(f"  airsp-B : {total} volumes across {len(REVB_CLASSES)} REV-B "
        "classes " + ", ".join(f"{k.split('_')[0]}={v}"
                               for k, v in n.items() if v))
    return n


# ---------------------------------------------------------------------------
# SIXTEEN INSPECTION SCENARIOS
# ---------------------------------------------------------------------------
def build_scenarios_b(coll, log=print):
    """The sixteen required scenario types, each at a real location.

    The nine REV-A scenarios keep their identity and are re-expressed here
    with the fuller metadata schema; seven are new. Every one is anchored to
    geometry that exists -- a specific pier, a specific bay -- rather than to
    a coordinate chosen for convenience.
    """
    x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1
    ps = [x for x, k in P.pier_stations() if x0r <= x <= x1r]
    half_w = P.DECK_WIDTH / 2.0
    y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0
    river_x = min(ps, key=lambda x: abs(x - P.RIVER_CENTRE_X))
    tall_x = max(ps, key=lambda x: P.deck_top_z(x) - P.ground_z(x, 0.0))
    out = []

    def add(idx, name, pos, **kw):
        # B-prefixed so the nine REV-A markers keep their own names and
        # both sets can coexist without a name collision.
        nm = f"SCENARIO_B{idx:02d}_{name}"
        props = {
            "avi_kind": "inspection_scenario",
            "avi_object_type": "SCENARIO",
            "avi_object_id": nm,
            "avi_scenario_id": idx,
            "avi_scenario": name,
            "avi_scenario_type": kw.get("stype", name),
            "avi_difficulty": kw["difficulty"],
            "avi_recommended_sensor": kw["sensor"],
            "avi_minimum_clearance_m": kw["clearance"],
            "avi_lighting_condition": kw["lighting"],
            "avi_occlusion_level": kw["occlusion"],
            "avi_water_present": kw.get("water", False),
            "avi_gps_quality": kw["gps"],
            "avi_dynamic_obstacles": kw.get("dynamic", False),
            "avi_recommended_uav_speed_mps": kw["speed"],
            "avi_challenge": kw["challenge"],
            "avi_sensing_problem": kw["sensing"],
            "avi_suggested_mitigation": kw["mitigation"],
            "avi_sector": P.sector_at(pos[0]) or "OUTSIDE_RESEARCH_ZONE",
            "avi_section": P.section_at(pos[0]),
            "avi_chainage_m": round(pos[0], 1),
        }
        ob = bpy.data.objects.new(nm, None)
        ob.empty_display_type = "SPHERE"
        ob.empty_display_size = 2.5
        ob.location = pos
        coll.objects.link(ob)
        ML.set_custom(ob, props)
        out.append(nm)

    soff = P.soffit_z

    add(1, "OPEN_DAYLIGHT_INSPECTION",
        (ps[1] + 40.0, half_w + 2.0, P.deck_top_z(ps[1] + 40.0) - 1.0),
        difficulty=1, sensor="RGB", clearance=3.0, lighting="DIRECT_SUN",
        occlusion=0.05, gps="HIGH", speed=2.5,
        challenge="outer parapet and deck edge in full sun, reachable from "
                  "any direction",
        sensing="none in particular -- this is the control case, and a "
                "detector that cannot find defects HERE has no chance "
                "anywhere else",
        mitigation="baseline condition; use it to calibrate before "
                   "attempting anything harder")

    add(2, "UNDERBRIDGE_INSPECTION",
        (ps[2] + 25.0, 0.0, soff(ps[2] + 25.0) - 2.5),
        difficulty=3, sensor="RGB+DEPTH", clearance=1.5,
        lighting="INDIRECT_ONLY", occlusion=0.55, gps="LOW", speed=1.2,
        challenge="general under-deck volume beneath the slab",
        sensing="illumination is bounce only; exposure differs by four "
                "stops from the sky visible at the deck edge",
        mitigation="onboard illumination, HDR capture, exposure lock")

    add(3, "DEEP_UNDERSIDE_INSPECTION",
        (ps[3] + 14.0, y_g0 + P.GIRDER_SPACING * 2.5,
         soff(ps[3] + 14.0) + 1.6),
        difficulty=5, sensor="RGB+DEPTH", clearance=0.8,
        lighting="DEEP_SHADOW", occlusion=0.85, gps="DENIED", speed=0.6,
        challenge="between two girders with the slab overhead, 3.8 m clear",
        sensing="no sky, no GNSS, downwash recirculates off three surfaces, "
                "and the walls sit inside a long-baseline stereo minimum "
                "range",
        mitigation="short-baseline stereo or structured light, "
                   "wall-referenced odometry, reduced thrust")

    add(4, "PIER_INSPECTION",
        (ps[2], P.PIER_COL_SPACING / 2.0 + 3.0,
         P.ground_z(ps[2], 0.0) + 8.0),
        difficulty=2, sensor="RGB", clearance=2.0, lighting="MIXED",
        occlusion=0.20, gps="MEDIUM", speed=1.5,
        challenge="land pier column, circumnavigation required",
        sensing="the column is a curved surface, so a single standoff "
                "pass leaves the flanks at grazing incidence",
        mitigation="orbit the column rather than translating past it")

    add(5, "RIVER_PIER_INSPECTION",
        (river_x, P.PIER_COL_SPACING / 2.0 + 2.5, P.RIVER_WATER_Z + 1.2),
        difficulty=4, sensor="RGB", clearance=2.5, lighting="HIGH_GLARE",
        occlusion=0.25, water=True, gps="MEDIUM", speed=1.2,
        challenge="waterline band on a river pier -- the highest-value "
                  "defect zone on the whole structure",
        sensing="sun glint off moving water saturates the sensor and "
                "produces phantom depth returns; a ditching is unrecoverable",
        mitigation="polarising filter, approach off the specular lobe, "
                   "exposure bracketing, hard abort altitude")

    add(6, "STRONG_OCCLUSION",
        (ps[3], y_g0 + 0.0, soff(ps[3]) - 0.15),
        difficulty=5, sensor="RGB", clearance=0.8, lighting="DEEP_SHADOW",
        occlusion=0.92, gps="DENIED", speed=0.5,
        challenge="bearing seat: a 0.7 m recess enclosed by the pier cap "
                  "below, the girder above and the diaphragm behind",
        sensing="reachable from a single narrow cone of viewpoints; miss "
                "the cone and you return NO data rather than bad data",
        mitigation="gimbal-forward or articulated sensor, approach along "
                   "the cap axis")

    add(7, "NARROW_STRUCTURAL_CORRIDOR",
        (ps[4] - 20.0, y_g0 + P.GIRDER_SPACING * 1.0,
         soff(ps[4] - 20.0) + 1.0),
        difficulty=4, sensor="DEPTH", clearance=0.8, lighting="DEEP_SHADOW",
        occlusion=0.80, gps="DENIED", speed=0.6,
        challenge="translating the length of a bay between girder webs "
                  "with under a metre either side",
        sensing="lateral clearance is below the depth camera's minimum "
                "range, so the walls are invisible exactly when they matter",
        mitigation="side-facing short-range sensing, or fly the bay "
                   "centreline on wall-referenced odometry")

    add(8, "LOW_LIGHT_INSPECTION",
        (ps[4] + 30.0, -half_w - 2.0, soff(ps[4] + 30.0) + 1.2),
        difficulty=4, sensor="RGB+LIGHT", clearance=1.5, lighting="NIGHT",
        occlusion=0.45, gps="MEDIUM", speed=0.8,
        challenge="the same surfaces worked under the NIGHT scenario",
        sensing="ambient contributes nothing; every photon is the "
                "aircraft's own, so illumination falls off as 1/r^2 and "
                "exposure couples to standoff",
        mitigation="constant-standoff flight so illumination is constant, "
                   "calibrated light, longer dwell")

    add(9, "PARTIAL_SHADOW",
        (ps[1] + 60.0, half_w + 1.2, P.deck_top_z(ps[1] + 60.0) + 0.6),
        difficulty=3, sensor="RGB", clearance=2.0, lighting="HIGH_CONTRAST",
        occlusion=0.15, gps="HIGH", speed=1.5,
        challenge="the shadow line of the parapet crossing the surface "
                  "under inspection",
        sensing="a single frame cannot hold both sides of the shadow edge; "
                "a crack crossing it appears twice with different contrast",
        mitigation="HDR bracket, or work the surface at a time of day when "
                   "the shadow line is elsewhere")

    add(10, "REFLECTIVE_WATER_ENVIRONMENT",
        (river_x + 45.0, 0.0, P.RIVER_WATER_Z + 6.0),
        difficulty=4, sensor="DEPTH+RGB", clearance=2.5,
        lighting="HIGH_GLARE", occlusion=0.10, water=True,
        gps="MEDIUM", speed=1.5,
        challenge="open water beneath, structure above",
        sensing="downward optical flow fails over moving water; depth "
                "returns from the surface are unreliable and intermittent",
        mitigation="reference odometry UPWARD to the soffit, treat "
                   "downward depth as no-data")

    add(11, "LONG_RANGE_DETECTION",
        (ps[0] - 25.0, -half_w - 45.0, P.deck_top_z(ps[0]) + 6.0),
        difficulty=4, sensor="RGB", clearance=8.0, lighting="DIRECT_SUN",
        occlusion=0.05, gps="HIGH", speed=3.0,
        challenge="screening a whole span from 45 m standoff to decide "
                  "where to go close",
        sensing="at 45 m the ground sample distance is 28 mm, so anything "
                "under severity 3 is below the noise floor; the honest "
                "output is a coarse triage map, not a detection",
        mitigation="use it to PRIORITISE, never to clear a surface")

    add(12, "CLOSE_RANGE_INSPECTION",
        (ps[2] + 18.0, -half_w - 1.0, soff(ps[2] + 18.0) + 1.5),
        difficulty=3, sensor="RGB+DEPTH", clearance=0.5,
        lighting="ONBOARD", occlusion=0.60, gps="LOW", speed=0.4,
        challenge="0.5-1.0 m standoff, where a hairline crack finally "
                  "spans enough pixels to be detected",
        sensing="depth of field is shallow, downwash is close enough to "
                "disturb loose material, and station-keeping error is a "
                "large fraction of the standoff",
        mitigation="mechanical or visual standoff reference, short "
                   "exposure, accept a small field of view")

    add(13, "HIGH_WIND_CONCEPTUAL",
        (tall_x, P.PIER_COL_SPACING / 2.0 + 3.0,
         P.ground_z(tall_x, 0.0) + 0.72 *
         (P.deck_top_z(tall_x) - P.ground_z(tall_x, 0.0))),
        difficulty=4, sensor="RGB", clearance=4.0, lighting="DIRECT_SUN",
        occlusion=0.15, gps="HIGH", speed=1.0,
        challenge=f"column face high on the tallest pier in the zone "
                  f"({P.deck_top_z(tall_x) - P.ground_z(tall_x, 0.0):.0f} m)",
        sensing="CONCEPTUAL ONLY -- no wind field is simulated. The marker "
                "records where channelled wind and vortex shedding would "
                "make station-keeping error into image blur",
        mitigation="increase windward standoff, gust-tolerant hold, "
                   "shorter exposure at higher gain")

    add(14, "GPS_DENIED_CONCEPTUAL",
        (river_x + 45.0, 0.0, soff(river_x + 45.0) - 4.0),
        difficulty=4, sensor="DEPTH+LIDAR", clearance=1.5,
        lighting="INDIRECT_ONLY", occlusion=0.75, water=True,
        gps="DENIED", speed=0.8,
        challenge="mid-channel under-deck, 24 m of concrete overhead and "
                  "open water below",
        sensing="CONCEPTUAL ONLY -- no GNSS physics is simulated; the "
                "avi_gps_quality field is metadata a navigation stack "
                "consumes. Geometrically, the sky really is fully occluded "
                "here and there really is no ground texture below",
        mitigation="visual-inertial odometry referenced upward to the "
                   "soffit; do not expect a position fix")

    add(15, "MULTI_UAV_SHARED_WORKSPACE",
        (P.RESEARCH_X0 + 3.0 * P.SECTOR_LENGTH, -half_w - 16.0,
         P.deck_top_z(P.RESEARCH_X0 + 3.0 * P.SECTOR_LENGTH) + 10.0),
        difficulty=3, sensor="RGB", clearance=10.0, lighting="DIRECT_SUN",
        occlusion=0.10, gps="HIGH", speed=1.0,
        challenge="the SECTOR_C / SECTOR_D boundary, where two aircraft "
                  "working adjacent sectors come closest",
        sensing="each aircraft is a moving obstacle the other's planner "
                "did not place, and both are inside the other's "
                "inspection standoff",
        mitigation="the 20 m separation buffer volume, plus a protocol "
                   "that gives one aircraft the boundary at a time")

    add(16, "DYNAMIC_OBSTACLE",
        (P.RESEARCH_X0 + 4.5 * P.SECTOR_LENGTH, -half_w + 3.0,
         P.soffit_z(P.RESEARCH_X0 + 4.5 * P.SECTOR_LENGTH) - 1.6),
        difficulty=4, sensor="DEPTH", clearance=1.5,
        lighting="INDIRECT_ONLY", occlusion=0.70, gps="LOW", speed=0.6,
        dynamic=True,
        challenge="a suspended maintenance platform hanging inside the "
                  "under-deck working volume",
        sensing="the obstacle is NOT on the digital twin. A planner working "
                "from the twin alone flies into it; only live depth sensing "
                "prevents that",
        mitigation="enable AVI_DYNAMIC_OBSTACLES, plan from the twin, and "
                   "require the aircraft to detect and re-route")

    log(f"  scen-B  : {len(out)} inspection scenarios with full metadata")
    return out


def export_manifest(path):
    """Zone manifest covering REV-A and REV-B volumes together."""
    import json
    out = {"sectors": [], "airspace_rev_a": [], "airspace_rev_b": [],
           "scenarios": [], "cameras": []}
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
            if rec.get("avi_taxonomy") == "REV_B":
                out["airspace_rev_b"].append(rec)
            else:
                out["airspace_rev_a"].append(rec)
        elif k == "inspection_scenario":
            out["scenarios"].append(rec)
        elif k == "validation_camera":
            out["cameras"].append(rec)
    for v in out.values():
        v.sort(key=lambda r: r["name"])
    out["rev_b_classes"] = REVB_CLASSES
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return {k: len(v) for k, v in out.items() if isinstance(v, list)}

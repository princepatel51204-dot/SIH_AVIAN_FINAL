"""REV-C Stage 2 -- metro airspace, sectors, and the inter-structure corridor.

The interesting one is INTER_STRUCTURE_CORRIDOR: the volume between the road
bridge and the metro viaduct. REV-B has no equivalent. It is bounded on both
sides by structure, it sits under the transit corridor, and it is the only
place in the model where two inspection targets face each other across a
flyable gap -- a confined, GNSS-degraded, multi-aircraft separation problem.

METRO_INTERIOR is the second new condition: the inside of the box girder,
reachable only through the access hatches in the bottom slab. 1.82 m of
headroom between the slabs, which is genuinely confined rather than
nominally so.

Measured clearances this is built against (not assumed -- see the Stage 2
report): BR_ reaches y = +12.00, the metro box spans y in [40.70, 49.30],
so the gap is 28.7 m wide. SAFE_TRANSIT and the REV-B transit corridor both
floor at z = 55, which is why nothing here goes above it.
"""
from __future__ import annotations

import bpy

import metro as MB
import params as P
import zones as Z


# Same shape as zones_b.REVB_CLASSES: every volume carries its own operating
# rule, so a planner reads the constraint off the volume rather than
# inferring it from the name.
METRO_CLASSES = {
    "METRO_DECK_INSPECTION": {
        "speed": 2.5, "standoff": 3.0, "clearance": 2.0, "risk": "MEDIUM",
        "gps": "HIGH",
        "rule": "above the metro deck, clear of the catenary masts"},
    "METRO_UNDERSIDE_INSPECTION": {
        "speed": 1.2, "standoff": 1.5, "clearance": 1.0, "risk": "HIGH",
        "gps": "DEGRADED",
        "rule": "under the box girder; soffit above, ground below"},
    "METRO_PIER_INSPECTION": {
        "speed": 1.2, "standoff": 1.5, "clearance": 1.0, "risk": "HIGH",
        "gps": "DEGRADED",
        "rule": "around a single circular pier; orbit, do not hover upwind"},
    "METRO_INTERIOR": {
        "speed": 0.6, "standoff": 0.5, "clearance": 0.4, "risk": "CRITICAL",
        "gps": "DENIED",
        "rule": "inside the box girder, entered through a bottom-slab hatch; "
                "no GNSS, no return-to-home, tethered or dead-reckoned only"},
    "INTER_STRUCTURE_CORRIDOR": {
        "speed": 2.0, "standoff": 2.5, "clearance": 2.0, "risk": "HIGH",
        "gps": "DEGRADED",
        "rule": "between the road bridge and the metro viaduct; structure "
                "both sides, multi-aircraft separation by protocol"},
}


def _vol(name, x0, x1, y0, y1, z0, z1, coll, cls, extra=None):
    spec = METRO_CLASSES[cls]
    props = {
        "avi_kind": "airspace_volume",
        "avi_object_type": "AIRSPACE",
        "avi_object_id": name,
        "avi_zone_class": cls,
        "avi_taxonomy": "REV_C",
        "avi_structure": "METRO",
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


def build_airspace_c(coll, log=print):
    n = {c: 0 for c in METRO_CLASSES}
    half = MB.BOX_W / 2.0
    made = []

    for i in range(P.SECTOR_COUNT):
        s = "ABCDEF"[i]
        x0 = P.RESEARCH_X0 + i * P.SECTOR_LENGTH
        x1 = x0 + P.SECTOR_LENGTH

        # above the deck, stopping below the catenary mast tops
        made.append(_vol(f"AVI_METRO_DECK_{s}", x0, x1,
                         MB.Y - half - 3.0, MB.Y + half + 3.0,
                         MB.DECK_TOP_Z + MB.MAST_H + 1.0,
                         MB.DECK_TOP_Z + MB.MAST_H + 9.0,
                         coll, "METRO_DECK_INSPECTION",
                         {"avi_sector": f"MSECTOR_{s}"}))
        n["METRO_DECK_INSPECTION"] += 1

        # under the box girder
        made.append(_vol(f"AVI_METRO_UNDER_{s}", x0, x1,
                         MB.Y - half - 1.5, MB.Y + half + 1.5,
                         8.0, MB.DECK_TOP_Z - MB.BOX_D_MAIN - 1.0,
                         coll, "METRO_UNDERSIDE_INSPECTION",
                         {"avi_sector": f"MSECTOR_{s}"}))
        n["METRO_UNDERSIDE_INSPECTION"] += 1

        # the box interior -- between the two webs and the two slabs
        made.append(_vol(f"AVI_METRO_INTERIOR_{s}", x0, x1,
                         MB.Y - MB.WEB_GAUGE / 2.0 + MB.WEB_T / 2.0,
                         MB.Y + MB.WEB_GAUGE / 2.0 - MB.WEB_T / 2.0,
                         MB.DECK_TOP_Z - MB.BOX_D + MB.BOT_SLAB_T,
                         MB.DECK_TOP_Z - MB.TOP_SLAB_T,
                         coll, "METRO_INTERIOR",
                         {"avi_sector": f"MSECTOR_{s}",
                          "avi_entry": "bottom-slab access hatch"}))
        n["METRO_INTERIOR"] += 1

    # one pier-inspection volume per pier inside the research zone
    for i, x in enumerate(MB.pier_stations()):
        if not (P.RESEARCH_X0 <= x <= P.RESEARCH_X1):
            continue
        made.append(_vol(f"AVI_METRO_PIER_{i+1:03d}", x - 5.5, x + 5.5,
                         MB.Y - 5.5, MB.Y + 5.5, 2.0,
                         MB.DECK_TOP_Z - MB.BOX_D - 1.0,
                         coll, "METRO_PIER_INSPECTION", {"avi_pier": i + 1}))
        n["METRO_PIER_INSPECTION"] += 1

    # the corridor between the two structures, one per sector
    y0 = 12.0 + 2.0                       # BR_ reaches y = +12.00, measured
    y1 = MB.Y - MB.BOX_W / 2.0 - 2.0      # metro box near face
    for i in range(P.SECTOR_COUNT):
        s = "ABCDEF"[i]
        x0 = P.RESEARCH_X0 + i * P.SECTOR_LENGTH
        made.append(_vol(f"AVI_INTER_STRUCTURE_{s}", x0,
                         x0 + P.SECTOR_LENGTH, y0, y1, 12.0, 40.0,
                         coll, "INTER_STRUCTURE_CORRIDOR",
                         {"avi_sector": f"MSECTOR_{s}",
                          "avi_width_m": round(y1 - y0, 2)}))
        n["INTER_STRUCTURE_CORRIDOR"] += 1

    log("  airsp-C : " + f"{len(made)} volumes across {len(n)} REV-C classes "
        + ", ".join(f"{k.replace('METRO_', '').replace('_INSPECTION', '')}"
                    f"={v}" for k, v in n.items()))
    log(f"  corridor: inter-structure gap y[{y0:.1f},{y1:.1f}] "
        f"= {y1 - y0:.2f} m wide, z[12.0,40.0] = 28.0 m tall")
    return {"volumes": len(made), "classes": n,
            "corridor_width_m": round(y1 - y0, 2)}


def build_sectors_c(coll, log=print):
    """MSECTOR_A..F -- named so they can never collide with SECTOR_*."""
    out = []
    half = MB.BOX_W / 2.0
    for i in range(P.SECTOR_COUNT):
        s = "ABCDEF"[i]
        x0 = P.RESEARCH_X0 + i * P.SECTOR_LENGTH
        # The chainage fields are NOT optional decoration. The collision
        # exporter derives the research corridor as
        #     min(o["avi_chainage_start_m"] for o in sectors)
        # over every object with avi_kind == "inspection_sector" -- so a
        # metro sector carrying that kind but missing the field raises
        # KeyError and takes the whole export down. Found exactly that way.
        # The metro sectors share the road bridge's x bands, so including
        # them leaves the derived corridor unchanged.
        ob = Z._volume(f"MSECTOR_{s}_VOLUME", x0, x0 + P.SECTOR_LENGTH,
                       MB.Y - half - 6.0, MB.Y + half + 6.0,
                       0.0, MB.DECK_TOP_Z + MB.MAST_H + 10.0,
                       coll, "METRO_INSPECTION_SECTOR",
                       {"avi_kind": "inspection_sector",
                        "avi_object_type": "SECTOR",
                        "avi_structure": "METRO",
                        "avi_sector": f"MSECTOR_{s}",
                        "avi_sector_index": i + 1,
                        "avi_chainage_start_m": round(x0, 1),
                        "avi_chainage_end_m": round(x0 + P.SECTOR_LENGTH, 1),
                        "avi_length_m": round(P.SECTOR_LENGTH, 1),
                        "avi_deck_top_z_m": round(MB.DECK_TOP_Z, 2),
                        "avi_soffit_z_m": round(MB.DECK_TOP_Z - MB.BOX_D, 2)})
        out.append(ob)
    log(f"  msectors: {len(out)} x {P.SECTOR_LENGTH:.0f} m "
        f"({P.RESEARCH_X0:.0f} - {P.RESEARCH_X1:.0f}) at y={MB.Y:.1f}")
    return out


def build_markers_c(coll, log=print):
    """Mission markers for the metro, so V43 has something to reach.

    Deliberately mirrors the road bridge's per-sector entry/exit/inspection
    triplet rather than inventing a second scheme, so one planner can drive
    both structures.
    """
    import meshlib as ML
    made = {}
    half = MB.BOX_W / 2.0
    for i in range(P.SECTOR_COUNT):
        s = "ABCDEF"[i]
        x0 = P.RESEARCH_X0 + i * P.SECTOR_LENGTH
        x1 = x0 + P.SECTOR_LENGTH
        pts = {
            "MSECTOR_ENTRY": (x0 + 4.0, MB.Y - half - 5.0,
                              MB.DECK_TOP_Z + 12.0),
            "MSECTOR_EXIT": (x1 - 4.0, MB.Y - half - 5.0,
                             MB.DECK_TOP_Z + 12.0),
            "MINSPECTION_START": (x0 + 10.0, MB.Y - half - 2.5,
                                  MB.DECK_TOP_Z - MB.BOX_D - 2.5),
            "MINSPECTION_END": (x1 - 10.0, MB.Y - half - 2.5,
                                MB.DECK_TOP_Z - MB.BOX_D - 2.5),
        }
        for kind, p in pts.items():
            name = f"AVI_{kind}_{s}"
            ob = ML.box(name, (0.6, 0.6, 0.6), p, coll)
            ob.display_type = "WIRE"
            ob.hide_render = True
            ML.set_custom(ob, {
                "avi_kind": "mission_marker",
                "avi_object_type": "MISSION_MARKER",
                "avi_marker_type": kind,
                "avi_structure": "METRO",
                "avi_sector": f"MSECTOR_{s}",
                "avi_position_m": [round(v, 3) for v in p]})
            made[kind] = made.get(kind, 0) + 1
    log("  mmarkers: " + ", ".join(f"{k}={v}" for k, v in made.items()))
    return made


def build(colls, log=print):
    coll = colls.get("AVIAN_METRO_AIRSPACE") or colls["UAV_AIRSPACE_REV_B"]
    scoll = colls.get("AVIAN_METRO_SECTORS") or colls["INSPECTION_SECTORS"]
    mcoll = colls.get("AVIAN_METRO_MISSION") or colls["MISSION_MARKERS"]
    air = build_airspace_c(coll, log)
    secs = build_sectors_c(scoll, log)
    marks = build_markers_c(mcoll, log)
    return {"airspace": air, "sectors": len(secs), "markers": marks}

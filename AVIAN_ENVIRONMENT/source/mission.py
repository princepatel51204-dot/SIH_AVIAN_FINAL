"""UAV mission markers, corridors and the environment-side mission graph.

THIS IS NOT A PATH PLANNER.
It is the data a path planner will be given: where an aircraft may start, where
it may land, where a sector begins and ends, which waypoints are reachable from
which, and what each edge costs in distance, clearance and risk. The planner
that consumes this does not exist yet and nothing here decides a route.

Why the graph lives in the environment rather than in the planner: the costs on
these edges are properties of the *world* -- how much clearance there actually
is between two points, whether the straight line between them passes through a
girder -- and the environment is the only thing that can measure them. A planner
that computed its own clearance would be re-deriving what the scene already
knows, and would disagree with it the moment either changed.

Every edge is checked against real geometry by ray-cast. An edge that intersects
structure is not emitted, so the graph is traversable by construction.
"""
from __future__ import annotations
import json
import math

import bpy
from mathutils import Vector

import params as P
import meshlib as ML
import terrain as TR


MARKER_TYPES = [
    "UAV_SPAWN", "TAKEOFF", "LANDING", "EMERGENCY_LANDING",
    "SECTOR_ENTRY", "SECTOR_EXIT", "INSPECTION_START", "INSPECTION_END",
    "RETURN_TO_HOME", "WAYPOINT_CANDIDATE", "SERVICE_STATION",
]


# ---------------------------------------------------------------------------
def _marker(name, pos, coll, props, size=2.0, display="SPHERE"):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_type = display
    ob.empty_display_size = size
    ob.location = pos
    coll.objects.link(ob)
    ML.set_custom(ob, props)
    return ob


def _push_clear(pos, want=2.2, step=0.6, tries=8):
    """Nudge a marker outboard until it has real clearance.

    A waypoint the aircraft cannot occupy is not a waypoint. Rather than
    relaxing the check until the numbers pass, the marker is moved: outboard
    in -Y first, then down, because those are the two directions that lead
    away from a bridge deck.
    """
    if _clear_at(pos) >= want:
        return tuple(pos)
    # Try each escape direction independently rather than committing to one.
    # A waypoint under the deck has nowhere to go in -Y (that is further
    # under); it has to drop instead.
    outboard = -1.0 if pos[1] <= 0 else 1.0
    for axis, sign in ((1, outboard), (2, -1.0), (2, 1.0), (1, -outboard)):
        p = list(pos)
        for _ in range(tries):
            p[axis] += sign * step
            if _clear_at(p) >= want:
                return tuple(p)
    return tuple(pos)


def _clear_at(pos, radius=3.0, ignore_down=False):
    """Shortest distance from a point to renderable structure, capped.

    Casts along the six axes. Crude on purpose: this is a marker sanity check
    that runs 120-odd times, not a clearance field. Anything that reports the
    full cap is 'at least this clear', which is all a spawn point needs.

    `ignore_down` drops the -Z ray. A landing pad 0.4 m above its own
    hardstanding is not obstructed by the hardstanding -- resting on it is
    the entire purpose -- and counting the ground as an obstruction reported
    every pad in the model as unsafe.
    """
    dg = bpy.context.evaluated_depsgraph_get()
    best = radius
    dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1)]
    if not ignore_down:
        dirs.append((0, 0, -1))
    for d in dirs:
        org = Vector(pos)
        rem = radius
        for _ in range(6):
            hit, loc, nrm, idx, ob, mw = bpy.context.scene.ray_cast(
                dg, org, Vector(d), distance=rem)
            if not hit or ob is None:
                break
            if not ob.hide_render:
                best = min(best, (Vector(loc) - Vector(pos)).length)
                break
            step = (Vector(loc) - org).length + 1e-3
            rem -= step
            if rem <= 0:
                break
            org = Vector(loc) + Vector(d) * 1e-3
    return best


def _segment_clear(a, b, margin=1.5):
    """True if the straight line a->b misses all renderable structure.

    An edge that clips a girder is not an edge. Emitting one and leaving the
    planner to discover the problem at flight time is exactly the sort of
    thing that makes a simulation result meaningless.
    """
    dg = bpy.context.evaluated_depsgraph_get()
    a, b = Vector(a), Vector(b)
    d = b - a
    dist = d.length
    if dist < 1e-6:
        return True, dist
    d = d / dist
    # Trim both ends. An edge that lands on a pad necessarily terminates at
    # the ground, and an edge that starts 1.5 m above it necessarily starts
    # near it; testing the raw segment reported both as "blocked by the
    # terrain" and left every landing node orphaned in the graph.
    trim = min(margin, dist * 0.35)
    org = a + d * trim
    rem = max(0.0, dist - 2.0 * trim)
    if rem <= 0.0:
        return True, dist
    for _ in range(10):
        hit, loc, nrm, idx, ob, mw = bpy.context.scene.ray_cast(
            dg, org, d, distance=rem)
        if not hit or ob is None:
            return True, dist
        if not ob.hide_render:
            return False, dist
        step = (Vector(loc) - org).length + 1e-3
        rem -= step
        if rem <= 0:
            return True, dist
        org = Vector(loc) + d * 1e-3
    return True, dist


BASE_SITES = [("SOUTH", -120.0, -95.0), ("NORTH", 120.0, 95.0)]
BASE_CLEAR_RADIUS = 48.0


def prepare_base_sites(log=print):
    """Clear the two UAV base areas of city clutter.

    A base is a prepared site. Dropping the home markers into the middle of
    the procedural city left them 0.6 m from a tree, which the clearance
    check correctly reported as unsafe -- and the right response to that is
    to prepare the ground, not to lower the threshold until it passes.

    Only city dressing and secondary detail is removed. Nothing structural,
    nothing in the river, and nothing carrying a defect can be touched: the
    filter is by name prefix and it is deliberately narrow.
    """
    removable = ("CITY_BLD_", "CITY_TREE_", "CITY_LAMP_", "CITY_VEH_GR_",
                 "AVI_DET_", "AVI_PED_")
    centres = [(P.RESEARCH_X0 + dx if tag == "SOUTH"
                else P.RESEARCH_X1 + dx, dy)
               for tag, dx, dy in BASE_SITES]
    doomed = []
    for ob in bpy.data.objects:
        if ob.type != "MESH" or not ob.name.startswith(removable):
            continue
        for cx, cy in centres:
            if ((ob.location.x - cx) ** 2
                    + (ob.location.y - cy) ** 2) < BASE_CLEAR_RADIUS ** 2:
                doomed.append(ob)
                break
    for ob in doomed:
        bpy.data.objects.remove(ob, do_unlink=True)
    log(f"  basesite: cleared {len(doomed)} city objects from the two "
        f"{BASE_CLEAR_RADIUS:.0f} m base areas")
    return len(doomed)


# ---------------------------------------------------------------------------
def build(coll, log=print):
    """Mission markers for all six sectors plus the shared home area."""
    made = {t: 0 for t in MARKER_TYPES}
    x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1
    deck_max = max(P.deck_top_z(x) for x in range(int(x0r), int(x1r), 10))
    transit_z = max(P.AIRSPACE_SAFE_Z[0], deck_max + 20.0) + 6.0

    nodes = []

    GROUND_KINDS = {"UAV_SPAWN", "TAKEOFF", "LANDING", "SERVICE_STATION",
                    "EMERGENCY_LANDING"}

    def node(name, pos, kind, sector, extra=None):
        on_ground = kind in GROUND_KINDS
        d = {
            "avi_kind": "mission_marker",
            "avi_object_type": "MISSION_MARKER",
            "avi_object_id": name,
            "avi_marker_type": kind,
            "avi_sector": sector or "SHARED",
            "avi_chainage_m": round(pos[0], 1),
            "avi_position_m": [round(v, 2) for v in pos],
            "avi_clearance_m": round(
                _clear_at(pos, ignore_down=on_ground), 2),
            "avi_clearance_excludes_ground": on_ground,
        }
        if extra:
            d.update(extra)
        d["avi_safe"] = bool(d["avi_clearance_m"] >= 2.0)
        ob = _marker(name, pos, coll, d,
                     size=3.0 if kind in ("TAKEOFF", "LANDING") else 1.8,
                     display="CUBE" if kind in ("TAKEOFF", "LANDING",
                                                "SERVICE_STATION")
                     else "SPHERE")
        made[kind] = made.get(kind, 0) + 1
        nodes.append({"id": name, "type": kind,
                      "sector": sector or "SHARED",
                      "position_m": [round(v, 2) for v in pos],
                      "clearance_m": d["avi_clearance_m"],
                      "safe": d["avi_safe"]})
        return ob

    # ---- shared home area, off the corridor on dry land -------------------
    for tag, xs, ys in (("SOUTH", x0r - 120.0, -95.0),
                        ("NORTH", x1r + 120.0, 95.0)):
        # terrain.height, NOT params.ground_z. ground_z is the ideal channel
        # profile; the actual terrain carries up to 2 m of rolling relief on
        # top of it, and a pad placed with the wrong one sits underground.
        # This is the same error V03 exists to catch on city objects.
        gz = TR.height(xs, ys)
        node(f"AVI_HOME_{tag}", (xs, ys, gz + 0.4), "UAV_SPAWN", None,
             {"avi_recommended_heading_deg": 0.0 if tag == "SOUTH" else 180.0,
              "avi_required_clearance_m": 5.0,
              "avi_surface": "prepared hardstanding"})
        node(f"AVI_TAKEOFF_{tag}", (xs, ys, gz + 1.5), "TAKEOFF", None,
             {"avi_climb_to_z_m": round(transit_z, 1),
              "avi_required_clearance_m": 5.0})
        node(f"AVI_LANDING_{tag}", (xs + 14.0, ys,
                                    TR.height(xs + 14.0, ys) + 0.4),
             "LANDING", None, {"avi_required_clearance_m": 5.0})
        node(f"AVI_RTH_{tag}", (xs, ys, transit_z), "RETURN_TO_HOME", None,
             {"avi_required_clearance_m": 8.0})
        node(f"AVI_SERVICE_STATION_{tag}", (xs - 16.0, ys,
                                            TR.height(xs - 16.0, ys) + 0.6),
             "SERVICE_STATION", None,
             {"avi_battery_swap": True, "avi_bays": 3})

    # ---- per-sector markers ----------------------------------------------
    half_w = P.DECK_WIDTH / 2.0
    for i, nm in enumerate(P.SECTOR_NAMES):
        a = x0r + i * P.SECTOR_LENGTH
        b = a + P.SECTOR_LENGTH
        xm = 0.5 * (a + b)
        letter = nm[-1]
        soffit = min(P.soffit_z(x) for x in (a, xm, b))
        gz = max(P.ground_z(xm, 0.0), P.RIVER_WATER_Z)
        over_water = abs(xm - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2.0
        stand = half_w + P.AIRSPACE_INSPECTION_OFFSET

        node(f"AVI_SECTOR_{letter}_ENTRY", (a + 6.0, -stand, transit_z),
             "SECTOR_ENTRY", nm,
             {"avi_recommended_heading_deg": 0.0,
              "avi_required_clearance_m": 8.0})
        node(f"AVI_SECTOR_{letter}_EXIT", (b - 6.0, -stand, transit_z),
             "SECTOR_EXIT", nm,
             {"avi_recommended_heading_deg": 0.0,
              "avi_required_clearance_m": 8.0})
        node(f"AVI_SECTOR_{letter}_INSPECT_START",
             (a + 12.0, -stand, soffit + 4.0), "INSPECTION_START", nm,
             {"avi_recommended_heading_deg": 90.0,
              "avi_required_clearance_m": 3.0})
        node(f"AVI_SECTOR_{letter}_INSPECT_END",
             (b - 12.0, -stand, soffit + 4.0), "INSPECTION_END", nm,
             {"avi_recommended_heading_deg": 90.0,
              "avi_required_clearance_m": 3.0})

        # Emergency landing: only where there IS a surface to land on. Over
        # the navigation channel there is not, and saying so is the point.
        if over_water:
            node(f"AVI_SECTOR_{letter}_EMERG",
                 (xm, -half_w - 60.0, transit_z), "EMERGENCY_LANDING", nm,
                 {"avi_landable": False,
                  "avi_reason": "over open water for 620 m; no landing "
                                "surface within glide range",
                  "avi_ditch_only": True,
                  "avi_required_clearance_m": 10.0})
        else:
            node(f"AVI_SECTOR_{letter}_EMERG",
                 (xm, -half_w - 55.0,
                  TR.height(xm, -half_w - 55.0) + 0.4),
                 "EMERGENCY_LANDING", nm,
                 {"avi_landable": True,
                  "avi_surface": "open ground clear of the viaduct",
                  "avi_required_clearance_m": 6.0})

        # Waypoint lattice: three STATIONS, each a straight run along the
        # sector at constant y and z. The tour visits one station at a time
        # and changes station at a single chainage, which is what keeps
        # consecutive edges from cutting through the girders -- the first
        # version interleaved DECK/WEB/UNDER at every station and produced a
        # graph in which almost every edge was rejected as blocked.
        under_z = max(gz + 3.0, P.soffit_z(xm) - 4.0)
        for k in range(4):
            wx = a + (k + 0.5) * P.SECTOR_LENGTH / 4.0
            for tagz, wz, wy in (
                    ("DECK", P.deck_top_z(wx) + 8.0, -stand),
                    ("WEB", P.soffit_z(wx) + 2.2, -half_w - 3.2),
                    ("UNDER", under_z, 0.0)):
                node(f"AVI_WP_{letter}_{k+1}_{tagz}",
                     _push_clear((wx, wy, wz), 2.2),
                     "WAYPOINT_CANDIDATE", nm,
                     {"avi_station": tagz,
                      "avi_required_clearance_m": 2.0})
        # Station-change nodes, outboard of the deck edge at the two heights
        # a transition needs. Without these a WEB->UNDER move is a diagonal
        # straight through a girder.
        node(f"AVI_WP_{letter}_XFER_WEB", _push_clear(
            (a + 4.0, -half_w - 3.2, P.soffit_z(a + 4.0) + 2.2), 2.2),
            "WAYPOINT_CANDIDATE", nm,
            {"avi_station": "TRANSFER", "avi_required_clearance_m": 2.0})
        node(f"AVI_WP_{letter}_XFER_UNDER", _push_clear(
            (a + 4.0, -half_w - 3.2, under_z), 2.2),
            "WAYPOINT_CANDIDATE", nm,
            {"avi_station": "TRANSFER", "avi_required_clearance_m": 2.0})

    log(f"  mission : {sum(made.values())} markers "
        + ", ".join(f"{k}={v}" for k, v in made.items() if v))
    return {"markers": made, "nodes": nodes,
            "transit_z": transit_z}


# ---------------------------------------------------------------------------
def build_graph(nodes, log=print):
    """Edges between mission nodes, costed and geometry-checked.

    Only edges that are actually traversable are emitted. `risk` is a 0-1
    composite of how tight the corridor is and whether it is over water,
    because those are the two things in this environment that turn a control
    error into a lost aircraft.
    """
    by_id = {n["id"]: n for n in nodes}

    def add(edges, a, b, kind):
        if a not in by_id or b not in by_id:
            return
        na, nb = by_id[a], by_id[b]
        ok, dist = _segment_clear(na["position_m"], nb["position_m"])
        if not ok:
            return
        clear = min(na["clearance_m"], nb["clearance_m"])
        mx = 0.5 * (na["position_m"][0] + nb["position_m"][0])
        over_water = abs(mx - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2.0
        risk = min(1.0, max(0.0, (3.0 - clear) / 3.0) * 0.7
                   + (0.3 if over_water else 0.0))
        # 4 m/s transit, 1.2 m/s in the tight stuff
        speed = 4.0 if kind in ("TRANSIT", "CLIMB", "RETURN") else 1.2
        edges.append({
            "from": a, "to": b, "edge_type": kind,
            "distance_m": round(dist, 2),
            "min_clearance_m": round(clear, 2),
            "risk": round(risk, 3),
            "over_water": over_water,
            "assumed_speed_mps": speed,
            "expected_time_s": round(dist / speed, 1),
            "sector": nb["sector"],
        })

    edges = []
    for tag in ("SOUTH", "NORTH"):
        add(edges, f"AVI_HOME_{tag}", f"AVI_TAKEOFF_{tag}", "GROUND")
        add(edges, f"AVI_TAKEOFF_{tag}", f"AVI_RTH_{tag}", "CLIMB")
        add(edges, f"AVI_RTH_{tag}", f"AVI_LANDING_{tag}", "RETURN")
        add(edges, f"AVI_SERVICE_STATION_{tag}", f"AVI_HOME_{tag}", "GROUND")

    for i, nm in enumerate(P.SECTOR_NAMES):
        L = nm[-1]
        home = "SOUTH" if i < 3 else "NORTH"
        add(edges, f"AVI_RTH_{home}", f"AVI_SECTOR_{L}_ENTRY", "TRANSIT")
        add(edges, f"AVI_SECTOR_{L}_ENTRY", f"AVI_SECTOR_{L}_INSPECT_START",
            "DESCENT")
        # One station at a time: deck run out, web run back, transfer down
        # outboard of the deck edge, under-deck run out.
        prev = f"AVI_SECTOR_{L}_INSPECT_START"
        for k in range(4):
            nid = f"AVI_WP_{L}_{k+1}_DECK"
            add(edges, prev, nid, "INSPECTION")
            prev = nid
        add(edges, prev, f"AVI_WP_{L}_XFER_WEB", "DESCENT")
        prev = f"AVI_WP_{L}_XFER_WEB"
        for k in range(4):
            nid = f"AVI_WP_{L}_{k+1}_WEB"
            add(edges, prev, nid, "INSPECTION")
            prev = nid
        add(edges, prev, f"AVI_WP_{L}_XFER_UNDER", "DESCENT")
        prev = f"AVI_WP_{L}_XFER_UNDER"
        for k in range(4):
            nid = f"AVI_WP_{L}_{k+1}_UNDER"
            add(edges, prev, nid, "INSPECTION")
            prev = nid
        add(edges, prev, f"AVI_SECTOR_{L}_INSPECT_END", "INSPECTION")
        add(edges, f"AVI_SECTOR_{L}_INSPECT_END", f"AVI_SECTOR_{L}_EXIT",
            "CLIMB")
        add(edges, f"AVI_SECTOR_{L}_EXIT", f"AVI_RTH_{home}", "RETURN")
        # Abort from either end of the sector; whichever line is clear is
        # the one that gets emitted.
        add(edges, f"AVI_SECTOR_{L}_INSPECT_START", f"AVI_SECTOR_{L}_EMERG",
            "ABORT")
        add(edges, f"AVI_SECTOR_{L}_EXIT", f"AVI_SECTOR_{L}_EMERG", "ABORT")
        add(edges, f"AVI_RTH_{home}", f"AVI_SECTOR_{L}_EMERG", "ABORT")

    reach = {e["from"] for e in edges} | {e["to"] for e in edges}
    orphan = [n["id"] for n in nodes if n["id"] not in reach]
    log(f"  graph   : {len(nodes)} nodes, {len(edges)} traversable edges"
        + (f", {len(orphan)} unconnected" if orphan else ""))
    return {"nodes": nodes, "edges": edges, "unconnected": orphan}


def export_manifest(graph, markers, path):
    import json
    tot = sum(e["distance_m"] for e in graph["edges"])
    out = {
        "note": "environment-side mission graph. Costs are measured against "
                "real geometry; no route is chosen here.",
        "marker_counts": markers,
        "node_count": len(graph["nodes"]),
        "edge_count": len(graph["edges"]),
        "unconnected_nodes": graph["unconnected"],
        "total_edge_length_m": round(tot, 1),
        "edge_types": sorted({e["edge_type"] for e in graph["edges"]}),
        "nodes": graph["nodes"],
        "edges": graph["edges"],
    }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return {"nodes": len(graph["nodes"]), "edges": len(graph["edges"])}

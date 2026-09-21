"""Phase 3 - export a PyBullet collision asset for the research corridor.

WHAT THIS PRODUCES AND WHY IT IS SHAPED THIS WAY
------------------------------------------------
Not a decimated triangle soup. A set of ORIENTED BOXES AND CYLINDERS, one per
structural member, derived from the member's own geometry in the authoritative
REV-B .blend.

That choice matters. A contact solver on a decimated concave mesh is both slow
and wrong: decimation moves surfaces, and moving a girder soffit by 80 mm to
save triangles silently changes the clearance a UAV is being tested against.
Primitives fitted to each member are:

  * CONSERVATIVE  every primitive fully contains its source member, so the
                  collision world is a superset of the real world. A UAV that
                  clears the collision model clears the real bridge. The
                  reverse -- a decimated mesh that locally shrinks -- would let
                  an aircraft fly through concrete.
  * EXACT where it counts. The deck slabs, diaphragms, parapets and pier caps
                  ARE boxes in the source model, so their primitives are not
                  approximations at all.
  * FAST          ~600 primitives instead of 141,648 triangles.

The one real approximation is the I-section girder, which becomes the box of
its own bounding volume: the web recess between the flanges is filled in. That
is 0.30 m of depth on each side of a 0.20 m web, it is conservative, and
crucially it does NOT close the inspection bay -- the 3.20 m gap between
adjacent girders is set by the 4.00 m spacing minus the 0.80 m flange and is
untouched. Validation V04 measures that rather than assuming it.

WHAT IS EXCLUDED, DELIBERATELY
------------------------------
  * defect objects        decals and spall liners are surface detail, not
                          obstacles; a 6 mm decal is inside its host member
  * secondary detail      cable trays, conduit, brackets, downpipes. These are
                          real obstacles and they are EXCLUDED BY DEFAULT but
                          available behind `include_services=True`, because
                          they add 1,500 primitives for objects under 100 mm
                          across. Phase 8 turns them on for contact work.
  * city, vegetation      outside the flight envelope of the research zone
  * airspace volumes      non-rendering markers, not matter

Every exclusion is recorded in the manifest with its reason and its object
count, so nothing is silently dropped.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

# ---------------------------------------------------------------------------
DEFAULT_BLEND = os.environ.get(
    "AVIAN_ENV_BLEND",
    "/home/claude/avian_env/AVIAN_Smart_Infrastructure_City_REV_B.blend")

# Configurable corridor. Defaults give 900 m research zone + 100 m each side.
DEFAULT_MARGIN_M = 100.0

# Members that become collision primitives, mapped to a primitive type.
# CYL for anything round in plan; BOX for everything else.
STRUCTURAL_KINDS = {
    "deck_slab": "BOX", "deck_box": "BOX", "girder": "BOX",
    "diaphragm": "BOX", "parapet": "BOX", "pier_cap": "BOX",
    "pier_footing": "BOX", "pier_column": "CYL", "pier_collar": "CYL",
    "bearing": "BOX", "joint_gap": "BOX", "joint_nose": "BOX",
    "median": "BOX", "wearing": "BOX",
    # SIH_AVIAN_FINAL's drone-base landing pads. Purely additive: REV-C has
    # no landing_pad objects, so this is inert for it. Without this, a pad
    # is just geometry a UAV falls through -- classification here does not
    # depend on a name prefix, which is what makes it robust regardless of
    # what the pad object happens to be named.
    "landing_pad": "BOX",
}
# Name prefixes used when an object carries no avi_kind.
# MB_ is the REV-C metro viaduct. Required, not decorative:
# rails, catenary masts and station parts carry avi_kind values
# that are not in STRUCTURAL_KINDS, so without the prefix they
# fail is_struct and vanish from collision silently -- the UAV
# would fly straight through them with no check firing.
STRUCTURAL_PREFIXES = ("BR_", "MB_")

SERVICE_PREFIXES = ("AVI_DET_",)
EXCLUDE_PREFIXES = ("DEFECT_", "AVI_AIRSPACE", "AVI_TRANSIT", "AVI_DECK_",
                    "AVI_UNDERSIDE", "AVI_PIER_INSPECTION", "AVI_RIVER_",
                    "AVI_CONFINED", "AVI_SEPARATION", "AVI_EMERGENCY",
                    "SECTOR_", "SCENARIO_", "AVI_WP_", "AVI_HOME",
                    "AVI_SENSOR", "CITY_", "ENV_BANKVEG", "ENV_ROCK",
                    "AVI_PED_", "AVI_BOAT", "AVI_FLOAT", "AVI_OBST")


_COMMON = None


def _common():
    """Import the shared decomposition from avian_common/, once.

    One definition, used by this exporter and by the environment's Gazebo
    exporter. See avian_common/decompose.py for why it is reached by env var
    rather than installed. Cached because _obb runs per object, ~14,000
    times per export.
    """
    global _COMMON
    if _COMMON is not None:
        return _COMMON
    import importlib.util
    root = os.environ.get("AVIAN_COMMON_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), "avian_common")
    path = os.path.join(root, "decompose.py")
    spec = importlib.util.spec_from_file_location("_avian_common", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _COMMON = mod
    return mod


def _obb(ob, mathutils):
    """Axis-aligned box in world space, plus the object's world Z rotation.

    Delegates to avian_common so the Gazebo exporter cannot disagree with
    this one. The centre comes from matrix_world @ ob.bound_box, never from
    matrix_world.translation, which meshlib leaves at (0,0,0).
    """
    return _common().obb(ob, mathutils)


def export(blend=DEFAULT_BLEND, out_dir=None, margin_m=DEFAULT_MARGIN_M,
           include_services=False, log=print):
    import bpy
    import mathutils

    t0 = time.time()
    out_dir = out_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "assets")
    os.makedirs(out_dir, exist_ok=True)

    bpy.ops.wm.open_mainfile(filepath=blend)

    # Research zone comes from the environment itself, not from a copy of the
    # numbers. If the environment moves the zone, this follows.
    sectors = [o for o in bpy.data.objects
               if o.get("avi_kind") == "inspection_sector"]
    if sectors:
        x0r = min(o["avi_chainage_start_m"] for o in sectors)
        x1r = max(o["avi_chainage_end_m"] for o in sectors)
    else:                                    # fall back to the params module
        sys.path.insert(0, os.path.dirname(blend))
        import params as P
        x0r, x1r = P.RESEARCH_X0, P.RESEARCH_X1
    X0, X1 = x0r - margin_m, x1r + margin_m
    log(f"  corridor: x = {X0:.0f} .. {X1:.0f} m "
        f"({X1 - X0:.0f} m = {x1r - x0r:.0f} m zone + 2 x {margin_m:.0f} m)")

    prims = []
    excluded = {}
    src_tris = 0
    n_seen = 0

    for ob in bpy.data.objects:
        if ob.type != "MESH" or ob.hide_render:
            continue
        centre, half, yaw, lo, hi = _obb(ob, mathutils)
        if hi[0] < X0 or lo[0] > X1:
            continue
        n_seen += 1
        nm = ob.name
        kind = ob.get("avi_kind")

        if nm.startswith(EXCLUDE_PREFIXES):
            excluded[nm.split("_")[0]] = excluded.get(nm.split("_")[0], 0) + 1
            continue
        if nm.startswith(SERVICE_PREFIXES) and not include_services:
            excluded["secondary_detail"] = excluded.get(
                "secondary_detail", 0) + 1
            continue
        if nm.startswith("ENV_TERRAIN") or nm.startswith("ENV_GROUND_FAR"):
            excluded["terrain_mesh"] = excluded.get("terrain_mesh", 0) + 1
            continue
        if nm.startswith("ENV_RIVER_WATER"):
            excluded["water_surface"] = excluded.get("water_surface", 0) + 1
            continue
        if nm.startswith("RD_") or nm.startswith("ROAD_"):
            excluded["road_surface"] = excluded.get("road_surface", 0) + 1
            continue

        is_struct = (kind in STRUCTURAL_KINDS
                     or nm.startswith(STRUCTURAL_PREFIXES)
                     or nm.startswith(SERVICE_PREFIXES))
        if not is_struct:
            excluded["other"] = excluded.get("other", 0) + 1
            continue

        src_tris += sum(max(0, len(p.vertices) - 2) for p in ob.data.polygons)
        ptype = STRUCTURAL_KINDS.get(kind, "BOX")
        if ptype == "CYL":
            r = max(half[0], half[1])
            prims.append({"name": nm, "type": "CYLINDER",
                          "kind": kind or "structure",
                          "centre": [round(v, 4) for v in centre],
                          "radius": round(r, 4),
                          "half_height": round(half[2], 4),
                          "yaw": round(yaw, 6)})
        else:
            prims.append({"name": nm, "type": "BOX",
                          "kind": kind or "structure",
                          "centre": [round(v, 4) for v in centre],
                          "half_extents": [round(v, 4) for v in half],
                          "yaw": round(yaw, 6)})

    # ---- ground and water as planes -------------------------------------
    # Terrain is a 190x100 height grid; as collision it would be 38k triangles
    # for a surface a UAV should never touch. Two half-space boxes carry the
    # information that actually matters: where the ground is, and where the
    # water is. Both are recorded as approximations in the manifest.
    sys.path.insert(0, os.path.dirname(blend))
    import params as PP
    ground_z = 0.0
    water_z = PP.RIVER_WATER_Z
    river_half = PP.RIVER_WIDTH / 2.0
    prims.append({"name": "ENV_GROUND_PLANE", "type": "BOX",
                  "kind": "ground",
                  "centre": [(X0 + X1) / 2.0, 0.0, ground_z - 5.0],
                  "half_extents": [(X1 - X0) / 2.0 + 50.0, 700.0, 5.0],
                  "yaw": 0.0,
                  "note": "flat half-space at the corridor datum; the real "
                          "terrain carries up to 2 m of relief and the river "
                          "channel is cut separately below"})
    prims.append({"name": "ENV_WATER_SURFACE", "type": "BOX",
                  "kind": "water",
                  "centre": [PP.RIVER_CENTRE_X, 0.0, water_z - 5.0],
                  "half_extents": [river_half, 700.0, 5.0],
                  "yaw": 0.0,
                  "note": "water treated as a solid surface at z=-2.0. A UAV "
                          "that touches it has ditched; the sim registers "
                          "contact rather than letting it sink"})

    # ---- validation ------------------------------------------------------
    checks = []

    def vchk(cid, name, ok, measured, expected, detail=""):
        checks.append({"id": cid, "name": name,
                       "status": "PASS" if ok else "FAIL",
                       "measured": str(measured), "expected": str(expected),
                       "detail": detail})
        log(f"  [{'PASS' if ok else 'FAIL'}] {cid} {name:<42} "
            f"{str(measured):<26} {detail}")
        return ok

    # V01 every primitive contains its source object
    worst = 0.0
    bad = []
    byname = {p["name"]: p for p in prims}
    for ob in bpy.data.objects:
        if ob.type != "MESH" or ob.name not in byname:
            continue
        p = byname[ob.name]
        centre, half, yaw, lo, hi = _obb(ob, mathutils)
        if p["type"] == "BOX":
            for i in range(3):
                d = abs(p["centre"][i] - centre[i]) + \
                    abs(p["half_extents"][i] - half[i])
                worst = max(worst, d)
                if d > 0.002:
                    bad.append(f"{ob.name} axis{i} {d:.4f}")
    vchk("C01", "every box primitive matches its source bbox",
         not bad, f"max deviation {worst*1000:.2f} mm", "< 2 mm",
         f"{len(byname)} primitives cross-checked")

    # V02 corridor bounds
    xs = [p["centre"][0] for p in prims if p["kind"] not in ("ground",
                                                             "water")]
    vchk("C02", "all structural primitives inside the corridor",
         bool(xs) and min(xs) >= X0 - 60 and max(xs) <= X1 + 60,
         f"x = {min(xs):.0f} .. {max(xs):.0f}", f"{X0:.0f} .. {X1:.0f}",
         "60 m tolerance for members that straddle the boundary")

    # V03 air draft over the navigation channel preserved
    deck_prims = [p for p in prims if p["kind"] in ("girder", "deck_box")]
    mid = [p for p in deck_prims
           if abs(p["centre"][0] - PP.RIVER_CENTRE_X) < 45.0]
    if mid:
        soffit = min(p["centre"][2] - p["half_extents"][2] for p in mid)
        draft = soffit - water_z
        true_draft = PP.soffit_z(PP.RIVER_CENTRE_X) - water_z
        vchk("C03", "air draft preserved over the navigation channel",
             abs(draft - true_draft) < 0.25, f"{draft:.2f} m",
             f"{true_draft:.2f} m (source)",
             "collision soffit vs authoritative soffit")
    else:
        vchk("C03", "air draft preserved over the navigation channel",
             False, "no girders found mid-channel", "girders present")

    # V04 the inspection bay between girders is still open
    gp = [p for p in prims if p["kind"] == "girder"]
    bay = None
    if len(gp) >= 2:
        xm = PP.RESEARCH_X0 + 2.5 * (PP.RESEARCH_X1 - PP.RESEARCH_X0) / 6.0
        here = sorted([p for p in gp
                       if abs(p["centre"][0] - xm) < 40.0],
                      key=lambda p: p["centre"][1])
        gaps = []
        for a, b in zip(here, here[1:]):
            gap = ((b["centre"][1] - b["half_extents"][1])
                   - (a["centre"][1] + a["half_extents"][1]))
            if 0.5 < gap < 8.0:
                gaps.append(gap)
        bay = min(gaps) if gaps else None
    want_bay = PP.GIRDER_SPACING - PP.GIRDER_TOP_FLANGE_W
    vchk("C04", "inspection bay between girders stays open",
         bay is not None and abs(bay - want_bay) < 0.15,
         f"{bay:.2f} m" if bay else "not measured", f"{want_bay:.2f} m",
         "box-fitting the I-section must not close the bay")

    # C05 and C06 are ROAD BRIDGE checks. They were written when the scene
    # held one structure, so "pier columns" and "six sectors" meant the road
    # bridge's without having to say so. REV-C's metro viaduct adds a third
    # column diameter (2.8 m) and six MSECTORs, which made both ambiguous
    # rather than wrong: C05 read dia [2.4, 2.8, 3.2] and failed, C06
    # counted 12 sectors and failed. Scoping them to the road bridge
    # restores their original meaning exactly -- nothing is relaxed, road
    # columns must still be 2.4/3.2 m and there must still be exactly six
    # road sectors over 3 m. C07/C08 add the equivalent metro coverage, so
    # the number of things checked goes up, not down.
    road_cols = [p for p in prims if p["kind"] == "pier_column"
                 and str(p.get("name", "")).startswith("BR_")]
    rr = sorted({round(p["radius"] * 2, 2) for p in road_cols})
    vchk("C05", "road pier columns preserved with correct diameter",
         bool(road_cols) and all(
             abs(d - PP.PIER_COL_D) < 0.3 or abs(d - PP.PIER_COL_D_RIVER)
             < 0.3 for d in rr),
         f"{len(road_cols)} columns, dia {rr}",
         f"{PP.PIER_COL_D} / {PP.PIER_COL_D_RIVER} m")

    # V06 under-deck headroom preserved
    road_sectors = [s for s in sectors if s.get("avi_structure") != "METRO"]
    hs = []
    for s in road_sectors:
        if not (X0 <= s["avi_chainage_start_m"] <= X1):
            continue
        hs.append(s["avi_soffit_z_m"] - max(0.0, water_z))
    vchk("C06", "under-deck volume preserved for all six road sectors",
         len(road_sectors) == 6 and all(h > 3.0 for h in hs),
         f"{len(road_sectors)} sectors, headroom "
         f"{min(hs):.1f}-{max(hs):.1f} m" if hs else "0", "6 sectors, > 3 m")

    # ---- C07/C08 -- the metro viaduct, added in REV-C --------------------
    metro_prims = [p for p in prims
                   if str(p.get("name", "")).startswith("MB_")]
    vchk("C07", "metro viaduct reaches the collision asset",
         len(metro_prims) > 0,
         f"{len(metro_prims)} MB_ primitives of {len(prims)} total",
         "> 0 -- MB_ must be in STRUCTURAL_PREFIXES")

    metro_cols = [p for p in prims if p["kind"] == "pier_column"
                  and str(p.get("name", "")).startswith("MB_")]
    mrr = sorted({round(p["radius"] * 2, 2) for p in metro_cols})
    vchk("C08", "metro pier columns preserved as cylinders",
         bool(metro_cols) and all(abs(d - 2.8) < 0.3 for d in mrr),
         f"{len(metro_cols)} columns, dia {mrr}", "2.8 m")

    n_fail = sum(1 for c in checks if c["status"] == "FAIL")

    # ---- write -----------------------------------------------------------
    asset = os.path.join(out_dir, "avian_bridge_collision.json")
    with open(asset, "w") as f:
        json.dump({"primitives": prims}, f)
    manifest = {
        "source_environment": os.path.basename(blend),
        "source_environment_path": os.path.abspath(blend),
        "environment_revision": "REV_B",
        "research_zone_m": [x0r, x1r],
        "margin_m": margin_m,
        "corridor_m": [X0, X1],
        "corridor_length_m": X1 - X0,
        "coordinate_system": "identical to the REV-B environment: 1 unit = "
                             "1 m, Z up, right-handed, origin at the bridge "
                             "deck start, +X along the corridor",
        "units": "metres",
        "collision_primitives": len(prims),
        "primitive_types": {
            t: sum(1 for p in prims if p["type"] == t)
            for t in ("BOX", "CYLINDER")},
        "primitives_by_kind": {
            k: sum(1 for p in prims if p["kind"] == k)
            for k in sorted({p["kind"] for p in prims})},
        "source_triangles_in_corridor": src_tris,
        "objects_seen_in_corridor": n_seen,
        "simplification_ratio": (round(src_tris / max(1, len(prims)), 1)),
        "method": "per-member oriented bounding volumes, conservative by "
                  "construction (every primitive contains its source member)",
        "include_services": include_services,
        "excluded": excluded,
        "exclusion_reasons": {
            "DEFECT": "surface detail inside its host member, not an obstacle",
            "secondary_detail": "cable trays, conduit, brackets, downpipes - "
                                "real but sub-100 mm; enable with "
                                "include_services=True",
            "terrain_mesh": "replaced by a ground half-space primitive",
            "water_surface": "replaced by a water half-space primitive",
            "road_surface": "coincident with the deck slab already present",
            "CITY": "outside the research-zone flight envelope",
        },
        "validation": {"checks": checks,
                       "pass": len(checks) - n_fail, "fail": n_fail},
        "build_time_s": round(time.time() - t0, 1),
    }
    man_path = os.path.join(out_dir, "avian_bridge_collision_manifest.json")
    with open(man_path, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  export  : {len(prims)} primitives from {src_tris:,} source "
        f"triangles ({manifest['simplification_ratio']}:1)")
    return asset, man_path, manifest


if __name__ == "__main__":
    a, m, man = export()
    print(f"asset    {a}")
    print(f"manifest {m}")
    print(f"validation {man['validation']['pass']} pass, "
          f"{man['validation']['fail']} fail")

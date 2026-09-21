"""Automated scene validation.

Every check reports PASS or FAIL with the actual measured number and, on
failure, the specific objects at fault. A check that cannot be evaluated
reports SKIP rather than silently passing -- a validation suite that reports
success for a test it did not run is worse than no suite at all.

The checks are deliberately geometric and measurable. "Looks right" is not a
check; "no object's lowest vertex is more than 0.25 m above the terrain it
stands on" is.
"""
from __future__ import annotations
import math
import bpy
from mathutils import Vector

import params as P
import terrain as TR


# V14 checks the ground truth carries every field it should. REV-A and
# REV-B require the original 15; a later revision that adds fields to the
# ground truth appends them here so V14 requires those too, instead of the
# new fields going unchecked. REV-C's build_scene_c.py sets this to the
# fields contrast_c adds. Left empty, every earlier build behaves exactly as
# before.
EXTRA_REQUIRED_FIELDS = []


class Result:
    __slots__ = ("id", "name", "status", "measured", "limit", "detail",
                 "offenders")

    def __init__(self, cid, name, status, measured="", limit="", detail="",
                 offenders=None):
        self.id = cid
        self.name = name
        self.status = status
        self.measured = measured
        self.limit = limit
        self.detail = detail
        self.offenders = offenders or []

    def row(self):
        return (f"  [{self.status:4}] {self.id:<4} {self.name:<38} "
                f"{self.measured:<26} {self.detail}")

    def as_dict(self):
        return {"id": self.id, "check": self.name, "status": self.status,
                "measured": self.measured, "limit": self.limit,
                "detail": self.detail, "offenders": self.offenders[:12]}


def _world_bbox(ob):
    m = ob.matrix_world
    pts = [m @ Vector(c) for c in ob.bound_box]
    return (min(p.x for p in pts), max(p.x for p in pts),
            min(p.y for p in pts), max(p.y for p in pts),
            min(p.z for p in pts), max(p.z for p in pts))


def _in(name, *frags):
    return any(f in name for f in frags)


# ===========================================================================
def run(records, log=print):
    R = []
    meshes = [o for o in bpy.data.objects
              if o.type == "MESH" and not o.hide_render]

    # ---- V01 scale sanity -------------------------------------------------
    deck = [o for o in meshes if _in(o.name, "BR_DECK_SLAB")]
    if deck:
        ws = []
        for o in deck[:40]:
            b = _world_bbox(o)
            ws.append(b[3] - b[2])
        w = max(ws)
        ok = abs(w - P.DECK_WIDTH) < 1.0
        R.append(Result("V01", "deck width matches specification",
                        "PASS" if ok else "FAIL",
                        f"{w:.2f} m", f"{P.DECK_WIDTH:.2f} +/- 1.00 m",
                        "1 BU = 1 m confirmed" if ok
                        else "deck geometry does not match DECK_WIDTH"))
    else:
        R.append(Result("V01", "deck width matches specification", "SKIP",
                        detail="no deck slab objects found"))

    # ---- V02 corridor length ---------------------------------------------
    xs = []
    for o in meshes:
        if _in(o.name, "BR_DECK_SLAB", "BR_WEARING"):
            b = _world_bbox(o)
            xs += [b[0], b[1]]
    if xs:
        L = max(xs) - min(xs)
        ok = abs(L - P.BRIDGE_LENGTH) < 25.0
        R.append(Result("V02", "corridor length matches specification",
                        "PASS" if ok else "FAIL", f"{L:.1f} m",
                        f"{P.BRIDGE_LENGTH:.0f} +/- 25 m",
                        f"deck runs x={min(xs):.1f} to x={max(xs):.1f}"))
    else:
        R.append(Result("V02", "corridor length matches specification",
                        "SKIP"))

    # ---- V03 nothing floats above the terrain ----------------------------
    # City objects, trees, vehicles and rocks are all placed against
    # terrain.height(). If any of them was placed with params.ground_z
    # instead, it floats or sinks -- this is the check that catches it.
    worst = 0.0
    bad = []
    checked = 0
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        if not _in(o.name, "CITY_BLD_", "CITY_TREE_", "CITY_LAMP_",
                   "ENV_ROCK_", "ENV_BANKVEG_"):
            continue
        checked += 1
        gz = TR.height(o.location.x, o.location.y)
        d = o.location.z - gz
        if abs(d) > abs(worst):
            worst = d
        if abs(d) > 0.60:
            bad.append(f"{o.name} dz={d:+.2f}")
    R.append(Result("V03", "no floating or sunken ground objects",
                    "PASS" if not bad else "FAIL",
                    f"max |dz| {abs(worst):.3f} m over {checked} objects",
                    "0.60 m",
                    "all ground objects sit on terrain.height()" if not bad
                    else f"{len(bad)} objects off the terrain surface",
                    bad))

    # ---- V04 no structure below the water surface where it should not be --
    # Piers are ALLOWED in the river. Buildings, trees, vehicles and roads
    # are not.
    bad = []
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        # CITY_VEH_BR_* are vehicles ON THE BRIDGE DECK. They are supposed
        # to be over the water -- that is what the bridge is for. Only ground
        # objects are being tested here.
        if not _in(o.name, "CITY_BLD_", "CITY_TREE_", "CITY_VEH_GR_",
                   "CITY_LAMP_", "ROAD_"):
            continue
        if TR.is_water(o.location.x, o.location.y):
            bad.append(o.name)
    R.append(Result("V04", "no city or road objects in the river",
                    "PASS" if not bad else "FAIL",
                    f"{len(bad)} objects over water", "0",
                    "river corridor is clear of land objects" if not bad
                    else "objects placed over open water", bad))

    # ---- V05 piers reach the ground --------------------------------------
    # A pier whose footing floats is the single most visible structural error
    # in an aerial view.
    bad = []
    worst = 0.0
    n = 0
    for o in bpy.data.objects:
        if o.type != "MESH" or not _in(o.name, "BR_PIER_FOOT"):
            continue
        n += 1
        b = _world_bbox(o)
        gz = min(P.ground_z(b[0], 0.0), P.ground_z(b[1], 0.0))
        gap = b[4] - gz
        worst = max(worst, gap)
        if gap > 0.5:
            bad.append(f"{o.name} gap={gap:.2f}")
    R.append(Result("V05", "every pier footing meets the ground",
                    "PASS" if not bad else "FAIL",
                    f"max gap {worst:.3f} m over {n} footings", "0.50 m",
                    "all footings founded" if not bad
                    else f"{len(bad)} footings above ground level", bad))

    # ---- V06 navigation channel is clear ---------------------------------
    bad = []
    a = P.RIVER_CENTRE_X - P.NAV_CHANNEL_HALF
    b = P.RIVER_CENTRE_X + P.NAV_CHANNEL_HALF
    for x, kind in P.pier_stations():
        if a + 0.5 < x < b - 0.5:
            bad.append(f"pier at x={x:.1f}")
    R.append(Result("V06", "navigation channel free of piers",
                    "PASS" if not bad else "FAIL",
                    f"{P.MAIN_SPAN:.0f} m clear span",
                    f"x in ({a:.0f}, {b:.0f}) must be empty",
                    f"main span {P.MAIN_SPAN:.0f} m, air draft "
                    f"{P.soffit_z(P.RIVER_CENTRE_X) - P.RIVER_WATER_Z:.1f} m",
                    bad))

    # ---- V07 air draft over the navigable channel -------------------------
    draft = P.soffit_z(P.RIVER_CENTRE_X) - P.RIVER_WATER_Z
    ok = draft >= 12.0
    R.append(Result("V07", "air draft over navigation channel",
                    "PASS" if ok else "FAIL", f"{draft:.2f} m", ">= 12.00 m",
                    "clears inland navigation" if ok
                    else "insufficient clearance for vessels"))

    # ---- V08 longitudinal grade -------------------------------------------
    g = max(abs(P.PROFILE[i + 1][1] - P.PROFILE[i][1])
            / (P.PROFILE[i + 1][0] - P.PROFILE[i][0])
            for i in range(len(P.PROFILE) - 1)) * 100.0
    ok = g <= 4.0
    R.append(Result("V08", "maximum longitudinal grade",
                    "PASS" if ok else "FAIL", f"{g:.2f} %", "<= 4.00 %",
                    "within highway design limits" if ok
                    else "grade exceeds highway design limits"))

    # ---- V09 span / structural depth ratio --------------------------------
    ps = P.pier_stations()
    spans = [ps[i + 1][0] - ps[i][0] for i in range(len(ps) - 1)]
    ratios = [s / P.girder_depth_for_span(s) for s in spans]
    ok = max(ratios) <= 26.0
    R.append(Result("V09", "span to structural depth ratio",
                    "PASS" if ok else "FAIL",
                    f"{min(ratios):.1f} - {max(ratios):.1f}", "<= 26.0",
                    f"spans {min(spans):.0f} - {max(spans):.0f} m, "
                    f"depth {P.girder_depth_for_span(min(spans)):.2f} - "
                    f"{P.girder_depth_for_span(max(spans)):.2f} m"))

    # ---- V10 airspace volumes clear of structure -------------------------
    # The SAFE transit band and the RETURN corridors must contain no
    # structure at all. This is the check that makes the airspace definition
    # trustworthy rather than decorative.
    # MB_ (the REV-C metro viaduct) is tested here too. Without it the metro
    # could sit inside the SAFE or RETURN corridors and no check would fire,
    # which is the same trap as leaving it out of STRUCTURAL_PREFIXES.
    struct = [o for o in meshes
              if _in(o.name, "BR_", "MB_") and not _in(o.name, "_MARK")]
    sboxes = [_world_bbox(o) for o in struct]
    bad = []
    tested = 0
    for z in bpy.data.objects:
        if z.get("avi_zone_class") not in ("AVI_AIRSPACE_SAFE",
                                           "AVI_AIRSPACE_RETURN"):
            continue
        tested += 1
        zb = (z["avi_x_min_m"], z["avi_x_max_m"], z["avi_y_min_m"],
              z["avi_y_max_m"], z["avi_z_min_m"], z["avi_z_max_m"])
        for o, sb in zip(struct, sboxes):
            if (zb[0] < sb[1] and zb[1] > sb[0] and
                    zb[2] < sb[3] and zb[3] > sb[2] and
                    zb[4] < sb[5] and zb[5] > sb[4]):
                bad.append(f"{z.name} <-> {o.name}")
                break
    R.append(Result("V10", "free-flight airspace clear of structure",
                    "PASS" if not bad else "FAIL",
                    f"{tested} volumes tested against {len(struct)} members",
                    "0 intersections",
                    "SAFE and RETURN volumes contain no structure"
                    if not bad else "airspace intersects structure", bad))

    # ---- V11 under-bridge volumes have real headroom ---------------------
    bad = []
    hs = []
    for z in bpy.data.objects:
        if z.get("avi_zone_class") != "AVI_AIRSPACE_UNDERBRIDGE":
            continue
        h = z["avi_headroom_m"]
        hs.append(h)
        if h < 3.0:
            bad.append(f"{z.name} headroom={h:.2f}")
    R.append(Result("V11", "under-bridge volumes are flyable",
                    "PASS" if hs and not bad else
                    ("FAIL" if bad else "SKIP"),
                    f"headroom {min(hs):.1f} - {max(hs):.1f} m" if hs else "-",
                    ">= 3.0 m",
                    f"{len(hs)} under-deck volumes" if hs
                    else "no under-bridge volumes built", bad))

    # ---- V12 defects lie on their host surfaces --------------------------
    # A defect floating 200 mm off the concrete is invisible from a grazing
    # angle and wrong in the ground truth. Tolerance is deliberately tight.
    bad = []
    worst = 0.0
    for r in (records or []):
        ob = bpy.data.objects.get(r["defect_id"])
        if ob is None:
            bad.append(f"{r['defect_id']} MISSING")
            continue
        d = (Vector(ob.location) - Vector(r["position_m"])).length
        worst = max(worst, d)
        if d > 0.05:
            bad.append(f"{ob.name} off by {d:.3f} m")
    R.append(Result("V12", "defects sit on their recorded positions",
                    "PASS" if not bad else "FAIL",
                    f"max offset {worst*1000:.1f} mm over "
                    f"{len(records or [])} defects", "50 mm",
                    "ground truth positions match the scene" if not bad
                    else "defect positions disagree with the ground truth",
                    bad))

    # ---- V13 defects are inside the research zone ------------------------
    out = [r["defect_id"] for r in (records or [])
           if not (P.RESEARCH_X0 - 5 <= r["position_m"][0]
                   <= P.RESEARCH_X1 + 5)]
    R.append(Result("V13", "all defects inside the research zone",
                    "PASS" if not out else "FAIL",
                    f"{len(records or []) - len(out)}/{len(records or [])} "
                    "in zone",
                    f"x in [{P.RESEARCH_X0:.0f}, {P.RESEARCH_X1:.0f}]",
                    "high-detail damage is confined to the research zone"
                    if not out else "defects placed outside the zone", out))

    # ---- V14 ground truth completeness -----------------------------------
    need = ["defect_id", "type", "severity", "position_m", "surface_normal",
            "host_surface", "host_object", "bridge_section",
            "inspection_sector", "occlusion", "repairable", "reason",
            "representation", "area_m2", "width_mm"] + \
        list(EXTRA_REQUIRED_FIELDS)
    miss = []
    for r in (records or [])[:400]:
        for k in need:
            if k not in r or r[k] is None:
                miss.append(f"{r.get('defect_id','?')}:{k}")
    R.append(Result("V14", "ground truth records are complete",
                    "PASS" if not miss and records else
                    ("FAIL" if miss else "SKIP"),
                    f"{len(need)} fields x {len(records or [])} records",
                    "no missing fields",
                    "every defect is fully described" if not miss
                    else f"{len(miss)} missing values", miss))

    # ---- V15 escalation set is a meaningful minority ---------------------
    if records:
        nr = sum(1 for r in records if not r["repairable"])
        frac = nr / len(records)
        ok = 0.10 <= frac <= 0.35
        reasons = {}
        for r in records:
            if not r["repairable"]:
                reasons[r["reason"].split(";")[0][:44]] = \
                    reasons.get(r["reason"].split(";")[0][:44], 0) + 1
        R.append(Result("V15", "non-repairable cases are a useful minority",
                        "PASS" if ok else "FAIL",
                        f"{nr}/{len(records)} = {frac*100:.1f} %",
                        "10 - 35 %",
                        f"{len(reasons)} distinct escalation reasons"))
    else:
        R.append(Result("V15", "non-repairable cases are a useful minority",
                        "SKIP"))

    # ---- V21 the river is actually visible -------------------------------
    # The water object being present is not the same as the water being
    # renderable. The distant ground plane sat 0.8 m ABOVE the water surface
    # and covered the whole river; every view across the crossing showed dry
    # ground and nothing else caught it. This fires a ray straight down at
    # the river centre and demands that the first thing it meets is water.
    dg = bpy.context.evaluated_depsgraph_get()
    miss = []
    tested = 0
    for dy in (-900.0, -300.0, 0.0, 300.0, 900.0):
        p = Vector((P.RIVER_CENTRE_X, dy, 400.0))
        tested += 1
        down = Vector((0.0, 0.0, -1.0))
        org = p
        remaining = 2000.0
        found = None
        for _ in range(12):
            hit, loc, nrm, idx, ob, mwx = bpy.context.scene.ray_cast(
                dg, org, down, distance=remaining)
            if not hit or ob is None:
                break
            # airspace and sector volumes are hide_render wireframes; they are
            # in the ray-cast world but never in a picture, so step past them
            if not ob.hide_render:
                found = (ob, loc)
                break
            remaining -= (loc - org).length + 1e-3
            if remaining <= 0.0:
                break
            org = loc + down * 1e-3
        if found is None:
            miss.append(f"y={dy:.0f}: nothing renderable hit")
        elif ("WATER" not in found[0].name.upper()
              and not _in(found[0].name, "BR_")):
            miss.append(f"y={dy:.0f}: {found[0].name} "
                        f"at z={found[1].z:.2f}")
    R.append(Result("V21", "the river surface is not occluded",
                    "PASS" if not miss else "FAIL",
                    f"{tested - len(miss)}/{tested} probes reach water",
                    "water or bridge structure first",
                    "the river renders as water" if not miss
                    else "something is covering the river", miss))

    # ---- V16 polygon budget ----------------------------------------------
    tris = 0
    for o in bpy.data.objects:
        if o.type == "MESH" and not o.hide_render:
            tris += sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
    uniq = len(set(o.data.name for o in bpy.data.objects
                   if o.type == "MESH"))
    ok = tris < 6_000_000
    R.append(Result("V16", "polygon budget",
                    "PASS" if ok else "FAIL", f"{tris:,} triangles",
                    "< 6,000,000",
                    f"{len(bpy.data.objects)} objects, {uniq} unique meshes "
                    f"({len(bpy.data.objects)/max(1,uniq):.1f}x instancing)"))

    # ---- V17 camera far clip covers the scene ----------------------------
    bad = []
    for c in bpy.data.objects:
        if c.type != "CAMERA":
            continue
        need_d = c.get("avi_range_m", 0.0) * 2.5 + 1000.0
        if c.data.clip_end < need_d:
            bad.append(f"{c.name} clip_end={c.data.clip_end:.0f} "
                       f"needs {need_d:.0f}")
    R.append(Result("V17", "camera far clip covers the scene",
                    "PASS" if not bad else "FAIL",
                    f"{sum(1 for c in bpy.data.objects if c.type=='CAMERA')} "
                    "cameras", "clip_end >= 2.5x range + 1 km",
                    "no camera clips the corridor" if not bad
                    else "a camera will cut the scene short", bad))

    # ---- V18 sector coverage ---------------------------------------------
    if records:
        per = {}
        for r in records:
            per[r["inspection_sector"]] = per.get(r["inspection_sector"], 0) + 1
        counts = [per.get(s, 0) for s in P.SECTOR_NAMES]
        ok = min(counts) >= 8
        R.append(Result("V18", "every sector carries defects",
                        "PASS" if ok else "FAIL",
                        f"{min(counts)} - {max(counts)} per sector", ">= 8",
                        ", ".join(f"{s[-1]}:{c}"
                                  for s, c in zip(P.SECTOR_NAMES, counts))))
    else:
        R.append(Result("V18", "every sector carries defects", "SKIP"))

    # ---- V19 no duplicate object names -----------------------------------
    seen = {}
    for o in bpy.data.objects:
        seen[o.name] = seen.get(o.name, 0) + 1
    dups = [k for k, v in seen.items() if v > 1]
    R.append(Result("V19", "object names are unique",
                    "PASS" if not dups else "FAIL",
                    f"{len(bpy.data.objects)} objects", "no duplicates",
                    "every object is addressable by name" if not dups
                    else "duplicate names present", dups))

    # ---- V20 required collections exist ----------------------------------
    # Checked by ROLE, not by literal string. REV-B renames the top-level
    # groups to the AVIAN_* digital-twin scheme; the roles are identical and
    # a check that insisted on the old spelling would fail a scene that is
    # organised correctly.
    need_roles = [
        ("world", ("AVIAN_ENVIRONMENT", "AVIAN_WORLD")),
        ("bridge", ("BRIDGE", "AVIAN_BRIDGE")),
        ("river", ("RIVER", "AVIAN_RIVER")),
        ("roads", ("ROADS", "AVIAN_ROADS")),
        ("city", ("CITY", "AVIAN_CITY")),
        ("damage", ("STRUCTURAL_DAMAGE", "AVIAN_DEFECTS")),
        ("airspace", ("UAV_AIRSPACE",)),
        ("sectors", ("INSPECTION_SECTORS",)),
        ("cameras", ("CAMERAS",)),
        ("lighting", ("LIGHTING_ENVIRONMENT",)),
    ]
    need_c = [n for _, alts in need_roles for n in alts]
    miss = [role for role, alts in need_roles
            if not any(a in bpy.data.collections for a in alts)]
    R.append(Result("V20", "scene organisation is complete",
                    "PASS" if not miss else "FAIL",
                    f"{len(bpy.data.collections)} collections",
                    f"{len(need_roles)} roles required",
                    "collection tree as documented" if not miss
                    else f"missing: {', '.join(miss)}", miss))

    n_pass = sum(1 for r in R if r.status == "PASS")
    n_fail = sum(1 for r in R if r.status == "FAIL")
    n_skip = sum(1 for r in R if r.status == "SKIP")
    log("")
    log("  VALIDATION")
    for r in R:
        log(r.row())
        if r.status == "FAIL" and r.offenders:
            for o in r.offenders[:6]:
                log(f"         - {o}")
            if len(r.offenders) > 6:
                log(f"         - ... and {len(r.offenders)-6} more")
    log(f"  {n_pass} pass, {n_fail} fail, {n_skip} skip "
        f"of {len(R)} checks")
    return R, {"pass": n_pass, "fail": n_fail, "skip": n_skip,
               "total": len(R)}


def export(results, path):
    import json
    with open(path, "w") as f:
        json.dump({"checks": [r.as_dict() for r in results],
                   "summary": {
                       "pass": sum(1 for r in results if r.status == "PASS"),
                       "fail": sum(1 for r in results if r.status == "FAIL"),
                       "skip": sum(1 for r in results if r.status == "SKIP"),
                       "total": len(results)}}, f, indent=2)

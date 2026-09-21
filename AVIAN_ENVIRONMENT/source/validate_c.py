"""REV-C validation -- V33 onward.

Same discipline as validate.py and validate_b.py: every check reports the
measured number against its limit, and a check that cannot be evaluated
reports SKIP rather than passing quietly.
"""
from __future__ import annotations

import json
import math
import os

import bpy
from mathutils import Vector

import metro as MB
import params as P
from validate import Result


def _structural_kinds():
    """Read STRUCTURAL_KINDS from the collision exporter, not a copy of it.

    The vocabulary has exactly one definition, in
    AVIAN_UAV/simulation/export_bridge_collision.py. Copying it here so V36
    could check against it would create a second source that drifts the
    first time a kind is added -- the same failure the generated URDF
    exists to prevent. Loaded by file path because the two packages have no
    shared import path yet; Stage 4 replaces this with avian_common/.
    """
    root = os.environ.get("AVIAN_UAV_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), "AVIAN_UAV")
    path = os.path.join(root, "simulation", "export_bridge_collision.py")
    if not os.path.exists(path):
        return None
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("_avian_colx", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return dict(mod.STRUCTURAL_KINDS)
    except Exception:
        return None


def _mb():
    return [o for o in bpy.data.objects
            if o.type == "MESH" and o.name.startswith("MB_")]


def _bb(ob):
    cs = [ob.matrix_world @ v.co for v in ob.data.vertices]
    if not cs:
        t = ob.matrix_world.translation
        return (t.x, t.x, t.y, t.y, t.z, t.z)
    return (min(c.x for c in cs), max(c.x for c in cs),
            min(c.y for c in cs), max(c.y for c in cs),
            min(c.z for c in cs), max(c.z for c in cs))


def _overlap(a, b, tol=0.0):
    return (a[0] < b[1] - tol and b[0] < a[1] - tol and
            a[2] < b[3] - tol and b[2] < a[3] - tol and
            a[4] < b[5] - tol and b[4] < a[5] - tol)


def run(records, mrecords, log=print, collision_manifest=None):
    R = []
    mb = _mb()

    # ---- V33 metro clears the navigation channel -----------------------
    # Bounding-box centre, not matrix_world.translation: meshlib.box() bakes
    # the centre into the vertices and leaves the origin at (0,0,0), so
    # translation.x is 0 for every member. Testing against it would find no
    # pier in the channel no matter where the piers actually were -- a check
    # that passes for the wrong reason, which is worse than one that fails.
    piers = [o for o in mb if "PIER" in o.name]
    inside = []
    for o in piers:
        b = _bb(o)
        cx = 0.5 * (b[0] + b[1])
        if MB.MAIN_X0 < cx < MB.MAIN_X1:
            inside.append(o.name)
    soffit = MB.DECK_TOP_Z - MB.BOX_D_MAIN
    draft = soffit - P.RIVER_WATER_Z
    ok33 = (not inside) and draft >= 12.0
    R.append(Result("V33", "metro clears the navigation channel",
                    "PASS" if ok33 else "FAIL",
                    f"air draft {draft:.2f} m, {len(inside)} piers in channel",
                    ">= 12.0 m, 0 piers",
                    f"main span {MB.MAIN_X0:.0f}-{MB.MAIN_X1:.0f} "
                    f"({MB.MAIN_X1 - MB.MAIN_X0:.0f} m clear)", inside))

    # ---- V34 metro and road bridge do not intersect; ceiling ------------
    br = [o for o in bpy.data.objects
          if o.type == "MESH" and o.name.startswith("BR_")]
    mb_bb = [(o.name, _bb(o)) for o in mb]
    br_bb = [(o.name, _bb(o)) for o in br]
    # only compare members whose x ranges overlap, else this is 40k x 2k
    hits = []
    min_gap = 1e9
    for mn, m in mb_bb:
        for bn, b in br_bb:
            if m[1] < b[0] or b[1] < m[0]:
                continue
            if _overlap(m, b):
                hits.append(f"{mn}|{bn}")
            gap = m[2] - b[3]          # metro y_min minus road y_max
            if gap < min_gap:
                min_gap = gap
    top = max((v[5] for _n, v in mb_bb), default=0.0)
    ok34 = (not hits) and top < MB.CEILING_Z
    R.append(Result("V34", "metro clears the road bridge and the ceiling",
                    "PASS" if ok34 else "FAIL",
                    f"{len(hits)} overlaps, min gap {min_gap:.2f} m, "
                    f"max z {top:.2f} m",
                    f"0 overlaps, max z < {MB.CEILING_Z:.1f} m",
                    f"ceiling is the SAFE_TRANSIT / TRANSIT floor at z=55",
                    hits[:10]))

    # ---- V35 inter-structure corridor is flyable ------------------------
    cor = [o for o in bpy.data.objects
           if o.name.startswith("AVI_INTER_STRUCTURE_")]
    worst = None
    if cor:
        dims = []
        for o in cor:
            b = _bb(o)
            dims.append((b[1] - b[0], b[3] - b[2], b[5] - b[4]))
        worst = min(min(d) for d in dims)
    ok35 = bool(cor) and worst is not None and worst >= 3.0
    R.append(Result("V35", "inter-structure corridor is flyable",
                    "PASS" if ok35 else ("FAIL" if cor else "SKIP"),
                    f"{len(cor)} volumes, min dimension "
                    f"{worst:.2f} m" if worst is not None else "no volumes",
                    ">= 3.0 m in every dimension",
                    "between the road bridge and the metro viaduct"))

    # ---- V36 every MB_ member carries a valid avi_kind ------------------
    kinds = _structural_kinds()
    if kinds is None:
        R.append(Result("V36", "every MB_ member carries a valid avi_kind",
                        "SKIP", "collision exporter not importable",
                        "0 invalid",
                        "set AVIAN_UAV_DIR so the vocabulary is read from "
                        "its one definition rather than copied here"))
        kinds = {}
    missing, vocab, extra = [], 0, 0
    for o in mb:
        k = o.get("avi_kind")
        if not k:
            missing.append(o.name)
        elif k in kinds:
            vocab += 1
        elif k in MB.EXTRA_KINDS:
            extra += 1
        else:
            missing.append(f"{o.name}:{k}")
    ok36 = not missing
    R.append(Result("V36", "every MB_ member carries a valid avi_kind",
                    "PASS" if ok36 else "FAIL",
                    f"{vocab} in STRUCTURAL_KINDS, {extra} metro-specific, "
                    f"{len(missing)} invalid of {len(mb)}",
                    "0 invalid",
                    "metro-specific kinds classify via the MB_ prefix, "
                    "which is why MB_ must be in STRUCTURAL_PREFIXES",
                    missing[:10]))

    # ---- V37 MB_ reaches the collision exporter -------------------------
    if collision_manifest and os.path.exists(collision_manifest):
        with open(collision_manifest) as f:
            man = json.load(f)
        total = man.get("collision_primitives", 0)
        bykind = man.get("primitives_by_kind", {})
        mb_prims = man.get("metro_primitives")
        ok37 = total > 677
        R.append(Result("V37", "MB_ reaches the collision exporter",
                        "PASS" if ok37 else "FAIL",
                        f"{total} primitives"
                        + (f", {mb_prims} from MB_" if mb_prims else ""),
                        "> 677 (the REV-B count)",
                        f"kinds: {', '.join(sorted(bykind)[:6])}"))
    else:
        R.append(Result("V37", "MB_ reaches the collision exporter", "SKIP",
                        "no collision manifest", "> 677",
                        "run --phase collision first"))

    # ---- V38 metro ground truth is complete -----------------------------
    # The requirement is a SUPERSET of the road bridge's field set, not a
    # fixed count. The metro legitimately carries two fields the road does
    # not (base_type, structure), so asserting a bare number would either
    # be wrong or have to be updated every time either side gains a field.
    # Comparing against the road's own record is self-maintaining.
    #
    # This is the check that would have caught the gap found during Stage 2:
    # the metro records were missing instance_id, class_index and
    # surface_offset_mm. Nothing was looking for that -- it surfaced from
    # the sabotage harness's field comparison. Without instance_id the
    # metro defects are absent from dataset.py's instance segmentation
    # entirely, so a detector could never be scored on them.
    # The comparison is against the road's GUARANTEED set -- the fields
    # present on every road record -- not against one sample record. The
    # ground truth is legitimately not uniform: make_crack returns length_m
    # and make_spall does not, so length_m exists only on crack-type
    # defects in both populations. Taking the field set from records[0] (a
    # crack) made V38 demand length_m from the metro's 12 spalls and fail
    # while simultaneously reporting "0 absent" -- a contradiction that is
    # how the bug was found.
    road_all = [set(r) for r in (records or [])]
    road_common = set.intersection(*road_all) if road_all else set()
    road_union = set.union(*road_all) if road_all else set()
    metro_all = [set(r) for r in (mrecords or [])]
    metro_common = set.intersection(*metro_all) if metro_all else set()

    miss = []
    for r in (mrecords or []):
        for k in sorted(road_common):
            if k not in r or r[k] is None:
                miss.append(f"{r.get('defect_id','?')}:{k}")
    absent = sorted(road_common - metro_common) if metro_all else []
    extra = sorted(metro_common - road_union) if metro_all else []
    nf = len(mrecords[0]) if mrecords else 0
    R.append(Result("V38", "metro ground truth records are complete",
                    "PASS" if (mrecords and not miss) else
                    ("FAIL" if mrecords else "SKIP"),
                    f"{nf} fields on record 1, {len(metro_common)} on every "
                    f"one of {len(mrecords or [])}, {len(absent)} of the "
                    f"road's {len(road_common)} guaranteed fields absent",
                    f"every field the road guarantees ({len(road_common)})",
                    (f"metro-specific: {', '.join(extra)}; "
                     if extra else "")
                    + f"road union is {len(road_union)} "
                    f"(length_m is crack-only in both populations)"
                    + (f"; MISSING {', '.join(absent)}" if absent else ""),
                    miss[:10]))

    # ---- V39 metro defects have measured visibility and contrast --------
    lv = {}
    nvis = ncon = 0
    for r in (mrecords or []):
        if r.get("visible_fraction") is not None:
            nvis += 1
        if r.get("defect_background_contrast") is not None:
            ncon += 1
        d = r.get("detection_difficulty")
        if d:
            lv[d] = lv.get(d, 0) + 1
    n = len(mrecords or [])
    ok39 = n > 0 and nvis == n and ncon == n and len(lv) >= 3
    R.append(Result("V39", "metro defects have measured visibility+contrast",
                    "PASS" if ok39 else ("FAIL" if n else "SKIP"),
                    f"{nvis}/{n} visibility, {ncon}/{n} contrast, "
                    f"{len(lv)}/4 difficulty levels",
                    "100 % both, >= 3 levels",
                    ", ".join(f"{k}:{v}" for k, v in sorted(lv.items()))))

    # ---- V40 the original 192 are untouched -----------------------------
    # V22 already compares against the frozen REV-A baseline. This restates
    # it on the REV-C record set: the metro must not have added, removed or
    # renamed anything in the road bridge's population.
    ids = [r["defect_id"] for r in (records or [])]
    bad = [i for i in ids if not i.startswith("DEFECT_")]
    mids = [r["defect_id"] for r in (mrecords or [])]
    collide = sorted(set(ids) & set(mids))
    ok40 = len(ids) == 192 and not bad and not collide
    R.append(Result("V40", "the original 192 defects are untouched",
                    "PASS" if ok40 else "FAIL",
                    f"{len(ids)} road defects, {len(mids)} metro, "
                    f"{len(collide)} id collisions",
                    "192, 0 collisions",
                    "separate namespace and separate list", collide[:10]))

    # ---- V43 metro sectors reachable ------------------------------------
    secs = sorted({o.get("avi_sector") for o in bpy.data.objects
                   if o.get("avi_sector", "").startswith("MSECTOR_")})
    marks = {}
    for o in bpy.data.objects:
        if o.get("avi_object_type") == "MISSION_MARKER" and \
                o.get("avi_structure") == "METRO":
            marks.setdefault(o.get("avi_sector"), []).append(o.name)
    dangling = [s for s in secs if len(marks.get(s, [])) < 2]
    ok43 = len(secs) == P.SECTOR_COUNT and not dangling
    R.append(Result("V43", "metro sectors reachable in the mission graph",
                    "PASS" if ok43 else "FAIL",
                    f"{len(secs)} MSECTORs, "
                    f"{sum(len(v) for v in marks.values())} markers",
                    f"{P.SECTOR_COUNT} sectors, >= 2 markers each",
                    ", ".join(f"{s.split('_')[-1]}:{len(marks.get(s, []))}"
                              for s in secs), dangling))

    n_pass = sum(1 for r in R if r.status == "PASS")
    n_fail = sum(1 for r in R if r.status == "FAIL")
    n_skip = sum(1 for r in R if r.status == "SKIP")
    for r in R:
        log(r.row())
    log(f"  {n_pass} pass, {n_fail} fail, {n_skip} skip of {len(R)} "
        f"REV-C checks")
    return R, {"pass": n_pass, "fail": n_fail, "skip": n_skip,
               "total": len(R)}

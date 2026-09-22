"""SIH_AVIAN_FINAL -- detection pass: the 76-defect steel taxonomy (SPEC S4).

Two families, built in two separate passes from build_final.py:

  1. FASTENER defects (BOLT_LOOSE 18, BOLT_MISSING 8, BOLT_CORRODED 10,
     JOINT_ANCHOR_LOOSE 4 -- 40 total): selected from fasteners_final.py's
     PASS-1 manifest (every bolt still SOUND) by `select_bolt_defects()`,
     which returns a `records_state` dict for fasteners_final.build()'s
     PASS 2 to actually build. Records are assembled here, from the
     manifest's own known position/normal/host -- pass 2 rebuilds the exact
     same bolt at the exact same place, just with a different state.

  2. BESPOKE defects (WELD_CRACK, SECTION_LOSS, COATING_FAILURE,
     GUSSET_DISTORTION, BEARING_SEIZED, CONDUIT_DETACHED, HANDRAIL_LOOSE --
     36 total): damage.py's collect_sites() is concrete-specific (girder
     bays, pier columns, deck soffit), so these are placed directly against
     steel_final.py's own joint list and fasteners_final.py's bracket/clamp/
     handrail geometry, via `build_bespoke()`, called AFTER fasteners pass 2
     (so it can also reference the freshly-rebuilt bracket/clamp/handrail
     objects, which pass 2 recreates from scratch).

DELIBERATE DIFFICULTY SPREAD
-----------------------------
The detection-pass brief is explicit: an escalation list that comes back
empty is a bug, not a success. Every defect here is placed into one of three
INTENDED bands, using real joint geometry rather than a random roll --

  CERTIFIABLE    top-chord gusset / open chord faces, mid-span: open sky,
                 a drone can hold a near-normal 1.5 m standoff.
  MARGINAL       bottom-chord gusset / diagonal faces, mid-span: under the
                 deck, partially shadowed, oblique from any reachable angle.
  MUST_ESCALATE  floor/stringer framing under the bottom chord, or a joint
                 near the end panels/portal bracing: boxed in on multiple
                 sides, no direction gets close to the surface normal.

`intended_difficulty_band` is written to every record so the eventual
visibility.py / contrast_c.py measurement can be checked against the
intent -- and reported honestly if it disagrees (that is a finding, not a
bug in this module).
"""
from __future__ import annotations
import math
import random

import bpy
from mathutils import Matrix

import meshlib as ML

SEVERITY = {
    "BOLT_LOOSE": 3, "BOLT_MISSING": 4, "BOLT_CORRODED": 2,
    "WELD_CRACK": 4, "SECTION_LOSS": 3, "COATING_FAILURE": 1,
    "GUSSET_DISTORTION": 3, "BEARING_SEIZED": 3, "JOINT_ANCHOR_LOOSE": 3,
    "CONDUIT_DETACHED": 2, "HANDRAIL_LOOSE": 1,
}

OCCLUSION_BY_BAND = {"CERTIFIABLE": 0.10, "MARGINAL": 0.45,
                     "MUST_ESCALATE": 0.80}


# ===========================================================================
# shared geometry helpers
# ===========================================================================
def _norm(v):
    l = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / l, v[1] / l, v[2] / l)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
           a[2] * b[0] - a[0] * b[2],
           a[0] * b[1] - a[1] * b[0])


def _orient_euler(normal, tangent):
    n = _norm(normal)
    t = _norm(tangent)
    b = _cross(n, t)
    t = _cross(b, n)
    m = Matrix((
        (t[0], b[0], n[0]),
        (t[1], b[1], n[1]),
        (t[2], b[2], n[2]),
    ))
    return m.to_euler("XYZ")


def _decal(name, pos, normal, tangent, w, h, thick, coll, mat):
    """A thin, flat, real box just proud of a host surface -- same
    convention damage.py itself uses for SHADER_DECAL-style defects: actual
    geometry, so it reads correctly from any camera angle rather than only
    face-on."""
    ob = ML.box(name, (w, h, thick), (0, 0, 0), coll, mat)
    ob.rotation_euler = _orient_euler(normal, tangent)
    ob.location = tuple(pos[i] + normal[i] * (thick / 2.0 + 0.002)
                        for i in range(3))
    return ob


def _world_verts(ob):
    return [ob.matrix_world @ v.co for v in ob.data.vertices]


def _world_centre_top(ob):
    """(centre_x, centre_y, top_z) in WORLD space, from the mesh's own
    vertices -- not `.location`/`.matrix_world.translation`, which meshlib's
    box()/cylinder() leave at identity while baking the real position into
    the vertices themselves (the "origin trap" this whole codebase has to
    keep working around). `_member_between()`-built members (diagonals,
    verticals) DO carry a real object-level transform, but this still gives
    the right answer for them too, since it reads the transformed vertices
    either way."""
    verts = _world_verts(ob)
    cx = sum(v.x for v in verts) / len(verts)
    cy = sum(v.y for v in verts) / len(verts)
    ztop = max(v.z for v in verts)
    return cx, cy, ztop


def _world_centroid(ob):
    verts = _world_verts(ob)
    n = len(verts)
    return (sum(v.x for v in verts) / n, sum(v.y for v in verts) / n,
           sum(v.z for v in verts) / n)


def _surface_point(ob, half_width):
    """A point on the member's OUTER (+/-Y, whichever faces the corridor
    edge) face, `half_width` out from its centreline -- so a decal built at
    this point and nudged further out by `_decal()`'s own small offset sits
    on the visible surface rather than embedded inside the member."""
    cx, cy, cz = _world_centroid(ob)
    sign = 1.0 if cy <= 0.0 else -1.0
    return (cx, cy + sign * half_width, cz), (0.0, sign, 0.0)


def _panel_idx(host_name):
    try:
        return int(host_name.rsplit("_", 1)[-1])
    except ValueError:
        return -1


def _record(defect_id, dtype, band, pos, normal, host_object, host_surface,
           params, reason, representation="GEOMETRY", **fields):
    x = pos[0]
    rec = {
        "defect_id": defect_id,
        "type": dtype,
        "severity": SEVERITY[dtype],
        "position_m": [round(v, 4) for v in pos],
        "surface_normal": [round(v, 4) for v in normal],
        "host_surface": host_surface,
        "host_object": host_object,
        "bridge_section": params.section_at(x),
        "inspection_sector": params.sector_at(x),
        "occlusion": OCCLUSION_BY_BAND[band],
        "placement_rationale": reason,
        "intended_difficulty_band": band,
        "repairable": False,        # detection-only scope: repair is out
        "reason": reason,
        "representation": representation,
    }
    rec.update(fields)
    return rec


def _host_surface_for(assembly, host_object):
    if assembly == "GUSSET":
        return "TRUSS_GUSSET_TOP" if "_TOP_" in host_object \
            else "TRUSS_GUSSET_BOTTOM"
    if assembly == "FLOOR_STRINGER":
        return "TRUSS_FLOOR_STRINGER"
    if assembly == "JOINT_ANCHOR":
        return "DECK_TOP_FITTING"
    if assembly == "BEARING":
        return "BEARING_SEAT"
    return "TRUSS_MEMBER"


# ===========================================================================
# 1. fastener-manifest-backed defects
# ===========================================================================
def select_bolt_defects(params, bolt_manifest, log=print):
    """Choose bolt ids -> (type, band, state, angle). Returns
    (records, records_state) -- records_state is what fasteners_final.py's
    PASS 2 needs; records is the ground truth for these 40 defects."""
    rnd = random.Random(params.SEED + 31415)
    by_assembly = {}
    for m in bolt_manifest:
        by_assembly.setdefault(m["assembly"], []).append(m)

    gusset = by_assembly.get("GUSSET", [])
    top_mid = [m for m in gusset if "_TOP_" in m["host_object"]
              and _panel_idx(m["host_object"]) in (3, 4, 5)]
    bot_mid = [m for m in gusset if "_BOT_" in m["host_object"]
              and _panel_idx(m["host_object"]) in (3, 4, 5)]
    floor_pool = list(by_assembly.get("FLOOR_STRINGER", []))
    anchor_pool = list(by_assembly.get("JOINT_ANCHOR", []))

    used = set()

    def take(pool, n):
        avail = [m for m in pool if m["id"] not in used]
        rnd.shuffle(avail)
        chosen = avail[:n]
        used.update(m["id"] for m in chosen)
        if len(chosen) < n:
            log(f"  WARNING steel defect pool exhausted: wanted {n}, "
                f"got {len(chosen)}")
        return chosen

    ang = lambda: round(rnd.uniform(*params.BOLT_LOOSE_ANGLE_RANGE_DEG), 1)

    picks = []   # (manifest_entry, dtype, band, state, angle)

    def add(entries, dtype, band, state, angle_fn):
        for m in entries:
            picks.append((m, dtype, band, state, angle_fn()))

    add(take(top_mid, 6), "BOLT_LOOSE", "CERTIFIABLE", "LOOSE", ang)
    add(take(bot_mid, 6), "BOLT_LOOSE", "MARGINAL", "LOOSE", ang)
    add(take(floor_pool, 6), "BOLT_LOOSE", "MUST_ESCALATE", "LOOSE", ang)

    add(take(top_mid, 3), "BOLT_MISSING", "CERTIFIABLE", "MISSING",
        lambda: 0.0)
    add(take(bot_mid, 3), "BOLT_MISSING", "MARGINAL", "MISSING", lambda: 0.0)
    add(take(floor_pool, 2), "BOLT_MISSING", "MUST_ESCALATE", "MISSING",
        lambda: 0.0)

    add(take(top_mid, 4), "BOLT_CORRODED", "CERTIFIABLE", "CORRODED",
        lambda: 0.0)
    add(take(bot_mid, 3), "BOLT_CORRODED", "MARGINAL", "CORRODED",
        lambda: 0.0)
    add(take(floor_pool, 3), "BOLT_CORRODED", "MUST_ESCALATE", "CORRODED",
        lambda: 0.0)

    add(take(anchor_pool, 2), "JOINT_ANCHOR_LOOSE", "CERTIFIABLE", "LOOSE",
        ang)
    add(take(anchor_pool, 2), "JOINT_ANCHOR_LOOSE", "MARGINAL", "LOOSE", ang)

    records = []
    records_state = {}
    for m, dtype, band, state, angle in picks:
        records_state[m["id"]] = (state, angle)
        host_surface = _host_surface_for(m["assembly"], m["host_object"])
        if state == "MISSING":
            defect_id = f"{m['id']}_HOLE"
            feat_mm = params.BOLT_HEAD_ACROSS_FLATS_MM
            reason = (f"bolt physically absent at {m['assembly']} joint "
                     f"{m['host_object']} -- intended {band.lower()}")
        elif state == "LOOSE":
            defect_id = f"{m['id']}_MARK_NUT"
            feat_mm = params.MATCH_MARK_W * 1000.0
            reason = (f"torque match-mark broken, nut rotated {angle:.1f} "
                     f"deg at {m['assembly']} joint {m['host_object']} -- "
                     f"intended {band.lower()}; feature is the mark's own "
                     f"line width, not the 36 mm bolt head")
        else:  # CORRODED
            defect_id = f"{m['id']}_NUT"
            feat_mm = params.BOLT_HEAD_ACROSS_FLATS_MM
            reason = (f"corroded bolt head/washer at {m['assembly']} joint "
                     f"{m['host_object']} -- intended {band.lower()}")
        rec = _record(defect_id, dtype, band, m["position"], m["normal"],
                     m["host_object"], host_surface, params, reason,
                     representation="GEOMETRY",
                     width_mm=round(feat_mm, 3), depth_mm=0.0)
        records.append(rec)

    log(f"  steel   : {len(records)} fastener-backed defects selected "
        f"(BOLT_LOOSE={sum(1 for r in records if r['type']=='BOLT_LOOSE')}, "
        f"BOLT_MISSING={sum(1 for r in records if r['type']=='BOLT_MISSING')}, "
        f"BOLT_CORRODED={sum(1 for r in records if r['type']=='BOLT_CORRODED')}, "
        f"JOINT_ANCHOR_LOOSE="
        f"{sum(1 for r in records if r['type']=='JOINT_ANCHOR_LOOSE')})")
    return records, records_state


# ===========================================================================
# 2. bespoke defects (no fastener manifest involved)
# ===========================================================================
def build_bespoke(params, colls, mats, steel_result, log=print):
    rnd = random.Random(params.SEED + 27182)
    coll = colls["STEEL"]
    joints = steel_result["joints"]
    gusset_joints = [j for j in joints if j[5] == "GUSSET"]
    used_gussets = set()
    records = []

    def pick_gussets(face, mid, n):
        pool = [j for j in gusset_joints
               if f"_{face}_" in j[0]
               and (_panel_idx(j[0]) in (3, 4, 5)) == mid
               and j[0] not in used_gussets]
        rnd.shuffle(pool)
        chosen = pool[:n]
        used_gussets.update(j[0] for j in chosen)
        return chosen

    n_made = {}

    def count(dtype):
        n_made[dtype] = n_made.get(dtype, 0) + 1
        return n_made[dtype]

    # ---- WELD_CRACK (6): gusset-to-chord weld toe, thin dark line ---------
    weld_mat = mats["weld_crack"]
    weld_plan = [("TOP", True, "CERTIFIABLE", 2),
                ("BOT", True, "MARGINAL", 2),
                ("BOT", False, "MUST_ESCALATE", 2)]
    for face, mid, band, n in weld_plan:
        for j in pick_gussets(face, mid, n):
            jname, centre, normal, tangent, size, _assembly = j
            idx = count("WELD_CRACK")
            width_mm = round(rnd.uniform(0.4, 1.2), 2)
            length_m = rnd.uniform(0.08, 0.16)
            edge = tuple(centre[i] + tangent[i] * (size[0] / 2.0 - 0.05)
                        for i in range(3))
            name = f"SDEFECT_WELD_CRACK_{idx:02d}"
            _decal(name, edge, normal, tangent, length_m,
                  width_mm / 1000.0, 0.002, coll, weld_mat)
            records.append(_record(
                name, "WELD_CRACK", band, edge, normal, jname,
                "TRUSS_GUSSET_TOP" if face == "TOP" else "TRUSS_GUSSET_BOTTOM",
                params,
                f"weld toe crack at gusset {jname} -- intended "
                f"{band.lower()}", representation="SHADER_DECAL",
                width_mm=width_mm, depth_mm=0.0, length_m=round(length_m, 3)))

    # ---- SECTION_LOSS (6): corroded/thinned patch on a diagonal/vertical --
    rust_mat = mats["rust_patch"]
    diag_vert = [o for o in steel_result["objects"]
                if o.get("avi_kind") in ("truss_diagonal", "truss_vertical")]
    rnd.shuffle(diag_vert)
    sl_plan = [("CERTIFIABLE", 2), ("MARGINAL", 2), ("MUST_ESCALATE", 2)]
    i = 0
    for band, n in sl_plan:
        for _ in range(n):
            ob = diag_vert[i % len(diag_vert)]
            i += 1
            idx = count("SECTION_LOSS")
            area_m2 = round(rnd.uniform(0.03, 0.09), 4)
            side = math.sqrt(area_m2)
            half_w = (params.TRUSS_VERTICAL_SIZE[0] / 2.0
                     if ob.get("avi_kind") == "truss_vertical"
                     else params.TRUSS_DIAGONAL_SIZE[0] / 2.0)
            pos, normal = _surface_point(ob, half_w)
            tangent = (1.0, 0.0, 0.0)
            name = f"SDEFECT_SECTION_LOSS_{idx:02d}"
            _decal(name, pos, normal, tangent, side, side, 0.003, coll,
                  rust_mat)
            records.append(_record(
                name, "SECTION_LOSS", band, pos, normal, ob.name,
                "TRUSS_DIAGONAL" if ob.get("avi_kind") == "truss_diagonal"
                else "TRUSS_MEMBER", params,
                f"corroded, thinned flange patch on {ob.name} -- intended "
                f"{band.lower()}", representation="SHADER_DECAL",
                area_m2=area_m2, depth_mm=round(rnd.uniform(2.0, 6.0), 1)))

    # ---- COATING_FAILURE (8): early-stage rust bloom through failed paint -
    coat_mat = mats["coating_failure"]
    chords = [o for o in steel_result["objects"]
             if o.get("avi_kind") == "truss_chord"]
    pool = chords + diag_vert
    rnd.shuffle(pool)
    cf_plan = [("CERTIFIABLE", 2), ("MARGINAL", 3), ("MUST_ESCALATE", 3)]
    i = 0
    for band, n in cf_plan:
        for _ in range(n):
            ob = pool[i % len(pool)]
            i += 1
            idx = count("COATING_FAILURE")
            area_m2 = round(rnd.uniform(0.02, 0.07), 4)
            side = math.sqrt(area_m2)
            kind = ob.get("avi_kind")
            half_w = (params.TRUSS_CHORD_SIZE[0] / 2.0 if kind == "truss_chord"
                     else params.TRUSS_VERTICAL_SIZE[0] / 2.0
                     if kind == "truss_vertical"
                     else params.TRUSS_DIAGONAL_SIZE[0] / 2.0)
            pos, normal = _surface_point(ob, half_w)
            tangent = (1.0, 0.0, 0.0)
            name = f"SDEFECT_COATING_FAILURE_{idx:02d}"
            _decal(name, pos, normal, tangent, side, side, 0.002, coll,
                  coat_mat)
            records.append(_record(
                name, "COATING_FAILURE", band, pos, normal, ob.name,
                "TRUSS_MEMBER", params,
                f"failed coating, early rust bloom on {ob.name} -- "
                f"intended {band.lower()}", representation="SHADER_DECAL",
                area_m2=area_m2, depth_mm=0.0))

    # ---- GUSSET_DISTORTION (3): rotate an existing gusset plate slightly --
    gd_plan = [("TOP", True, "CERTIFIABLE"), ("BOT", True, "MARGINAL"),
              ("BOT", False, "MUST_ESCALATE")]
    for face, mid, band in gd_plan:
        j = pick_gussets(face, mid, 1)
        if not j:
            continue
        jname, centre, normal, tangent, size, _assembly = j[0]
        ob = bpy.data.objects.get(jname)
        if ob is None:
            continue
        angle_deg = rnd.uniform(1.5, 3.0)
        t = _norm(tangent)
        ob.rotation_euler.rotate_axis(
            "X" if abs(t[0]) < 0.5 else "Y", math.radians(angle_deg))
        idx = count("GUSSET_DISTORTION")
        width_mm = round(2.0 * (size[0] / 2.0) * math.sin(math.radians(
            angle_deg)) * 1000.0, 1)
        records.append(_record(
            jname, "GUSSET_DISTORTION", band, centre, normal, jname,
            "TRUSS_GUSSET_TOP" if face == "TOP" else "TRUSS_GUSSET_BOTTOM",
            params,
            f"gusset plate warped {angle_deg:.1f} deg out of plane at "
            f"{jname} -- intended {band.lower()}",
            representation="GEOMETRY", width_mm=width_mm,
            depth_mm=round(width_mm, 1)))

    # ---- BEARING_SEIZED (4): rust streak on the main-span bearings --------
    bearings = [o for o in bpy.data.objects
               if o.type == "MESH" and "BEARING" in o.name
               and o.name.startswith(("BR_BEARING_004", "BR_BEARING_005"))]
    rnd.shuffle(bearings)
    bs_plan = [("CERTIFIABLE", 1), ("MARGINAL", 2), ("MUST_ESCALATE", 1)]
    i = 0
    for band, n in bs_plan:
        for _ in range(n):
            if not bearings:
                break
            ob = bearings[i % len(bearings)]
            i += 1
            idx = count("BEARING_SEIZED")
            cx, cy, ztop = _world_centre_top(ob)
            pos = (cx, cy, ztop)
            normal, tangent = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
            name = f"SDEFECT_BEARING_SEIZED_{idx:02d}"
            _decal(name, pos, normal, tangent, 0.20, 0.15, 0.003, coll,
                  mats["rust_patch"])
            records.append(_record(
                name, "BEARING_SEIZED", band, pos, normal, ob.name,
                "BEARING_SEAT", params,
                f"seized bearing, rust streak and debris on {ob.name} -- "
                f"intended {band.lower()}", representation="SHADER_DECAL",
                area_m2=0.03, depth_mm=0.0))

    # ---- CONDUIT_DETACHED (5): a cable sagging off its clamp --------------
    clamps = [o for o in bpy.data.objects
             if o.get("avi_kind") == "cable_clamp"]
    rnd.shuffle(clamps)
    cd_plan = [("CERTIFIABLE", 2), ("MARGINAL", 2), ("MUST_ESCALATE", 1)]
    i = 0
    for band, n in cd_plan:
        for _ in range(n):
            if not clamps:
                break
            ob = clamps[i % len(clamps)]
            i += 1
            idx = count("CONDUIT_DETACHED")
            gap_mm = round(rnd.uniform(30.0, 70.0), 1)
            cx, cy, ctop = _world_centre_top(ob)
            sag = (cx, cy + 0.02, ctop - gap_mm / 1000.0 - 0.03)
            name = f"SDEFECT_CONDUIT_DETACHED_{idx:02d}"
            ML.box(name, (0.08, 0.05, 0.05), sag, coll, mats["steel_dark"])
            normal, tangent = (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)
            records.append(_record(
                name, "CONDUIT_DETACHED", band, sag, normal, ob.name,
                "TRUSS_MEMBER", params,
                f"cable detached from clamp {ob.name}, sagging "
                f"{gap_mm:.0f} mm -- intended {band.lower()}",
                representation="GEOMETRY", width_mm=gap_mm, depth_mm=0.0))

    # ---- HANDRAIL_LOOSE (4): base plate lifted off the deck ---------------
    bases = [o for o in bpy.data.objects
            if o.get("avi_kind") == "handrail_base"]
    rnd.shuffle(bases)
    hl_plan = [("CERTIFIABLE", 2), ("MARGINAL", 2)]
    i = 0
    for band, n in hl_plan:
        for _ in range(n):
            if not bases:
                break
            ob = bases[i % len(bases)]
            i += 1
            idx = count("HANDRAIL_LOOSE")
            gap_mm = round(rnd.uniform(6.0, 22.0), 1)
            ob.location = (ob.location.x, ob.location.y,
                          ob.location.z + gap_mm / 1000.0)
            bpy.context.view_layer.update()
            cx, cy, ctop = _world_centre_top(ob)
            pos = (cx, cy, ctop)
            name = f"SDEFECT_HANDRAIL_LOOSE_{idx:02d}"
            normal, tangent = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
            _decal(name, pos, normal, tangent, 0.24, 0.24, 0.002, coll,
                  mats["rust_patch"])
            records.append(_record(
                name, "HANDRAIL_LOOSE", band, pos, normal, ob.name,
                "DECK_TOP_FITTING", params,
                f"handrail base lifted {gap_mm:.0f} mm off the deck at "
                f"{ob.name} -- intended {band.lower()}",
                representation="GEOMETRY", width_mm=gap_mm, depth_mm=0.0))

    bpy.context.view_layer.update()
    by_type = {}
    for r in records:
        by_type[r["type"]] = by_type.get(r["type"], 0) + 1
    log(f"  steel   : {len(records)} bespoke defects placed -> {by_type}")
    return records

"""SIH_AVIAN_FINAL -- detection pass: ~1,200 bolts and their torque match
marks.

WHY MATCH MARKS EXIST AT ALL
-----------------------------
An M24 head is ~36 mm across flats -- about 17 px at 1 m against this
sensor's 2.1478 mm/px GSD, comfortably visible. But "loose" is a 2-5 mm
gap, 1-2 px: physically unresolvable by this camera at any sane inspection
range. Real practice solves this by painting a line across nut, washer and
plate after torquing; if the nut backs off, the line visibly breaks. That
is what makes loose-bolt detection possible by camera at all, so it is
built as real geometry, not asserted.

A bolt is one hex "nut" (the visible, torqued element -- the true bolt head
is on the inaccessible far face, not modelled, since inspection only ever
sees this side) + a washer + a two-segment match mark: one segment fixed to
the PLATE at a reference angle, one fixed to the NUT's own current angle.
On a sound bolt the two segments are collinear (mark angle 0). On a loose
bolt the nut segment is rotated by the slip angle -- the visible break.

Placement across the 7 assemblies (SPEC S3.1): the two truss-specific ones
(GUSSET, FLOOR_STRINGER) use the real joint list steel_final.py returns;
the five general ones (BEARING, JOINT_ANCHOR, WALKWAY_BRACKET, CABLE_CLAMP,
HANDRAIL_BASE) run the length of the whole corridor, not just the truss,
and get their own small bracket/clamp/base-plate geometry here since none
of it exists yet.
"""
from __future__ import annotations
import math
import random

import bpy

import meshlib as ML

_NUT_SEG = 6     # hex


def _build_protos(coll, mats, P):
    head = ML.cylinder("_BOLT_NUT_PROTO", P.BOLT_HEAD_ACROSS_FLATS_MM
                       / 1000.0 / math.cos(math.pi / _NUT_SEG) / 2.0,
                       P.BOLT_HEAD_H, (0, 0, 0), _NUT_SEG, coll,
                       mats["steel_bolt"], axis="Z", smooth=False)
    head.hide_render = True
    head.hide_viewport = True
    washer = ML.cylinder("_BOLT_WASHER_PROTO", P.BOLT_WASHER_R,
                         P.BOLT_WASHER_T, (0, 0, 0), 12, coll,
                         mats["steel_bolt"], axis="Z", smooth=False)
    washer.hide_render = True
    washer.hide_viewport = True
    mark = ML.box("_MATCH_MARK_PROTO",
                  (P.MATCH_MARK_LEN, P.MATCH_MARK_W, P.MATCH_MARK_T),
                  (P.MATCH_MARK_LEN / 2.0, 0, 0), coll, mats["paint_white"])
    mark.hide_render = True
    mark.hide_viewport = True
    hole = ML.cylinder("_BOLT_HOLE_PROTO", P.BOLT_THREAD_R * 1.3, 0.02,
                       (0, 0, 0), 10, coll, mats["steel_dark"], axis="Z",
                       smooth=False)
    hole.hide_render = True
    hole.hide_viewport = True
    return head, washer, mark, hole


def _frame_matrix(normal, tangent):
    """3x3 rotation matrix placing local +Z along `normal` and local +X
    along `tangent`."""
    import mathutils
    n = _norm(normal)
    t = _norm(tangent)
    b = _cross(n, t)
    t = _cross(b, n)
    return mathutils.Matrix((
        (t[0], b[0], n[0]),
        (t[1], b[1], n[1]),
        (t[2], b[2], n[2]),
    ))


def _frame(normal, tangent, spin_deg=0.0):
    """Euler XYZ for the bolt's frame, with an additional rotation of
    `spin_deg` about the bolt's OWN local Z (its normal) -- composed as a
    matrix product, not `Euler.rotate_axis`, which rotates about a GLOBAL
    axis and would spin the match mark around the wrong axis entirely for
    any bolt whose normal isn't already world-Z (i.e. every vertical
    gusset/truss-member bolt in this scene)."""
    import mathutils
    m = _frame_matrix(normal, tangent)
    if spin_deg:
        m = m @ mathutils.Matrix.Rotation(math.radians(spin_deg), 3, "Z")
    return m.to_euler("XYZ")


def _norm(v):
    l = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / l, v[1] / l, v[2] / l)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
           a[2] * b[0] - a[0] * b[2],
           a[0] * b[1] - a[1] * b[0])


def _true_host_name(jname):
    """steel_final.py's FLOOR_STRINGER joint names carry a disambiguating
    suffix (`_S`/`_N` for a floor beam's two ends, `_J00` for a stringer's
    panel points) that keeps them unique as dict keys -- it is not part of
    the real Blender object's name. Strip it back off so `host_object`
    points at an object that actually exists."""
    if bpy.data.objects.get(jname) is not None:
        return jname
    if jname.endswith("_S") or jname.endswith("_N"):
        cand = jname[:-2]
        if bpy.data.objects.get(cand) is not None:
            return cand
    if "_J" in jname:
        cand = jname.rsplit("_J", 1)[0]
        if bpy.data.objects.get(cand) is not None:
            return cand
    return jname


def clear(coll):
    """Delete every object (and orphaned mesh data) currently in `coll`.

    Needed before a second `build()` pass: damage_steel.py selects which
    bolts become defects from PASS 1's manifest, then this module is called
    again to actually build those states -- without clearing first, pass 2
    would create a second, overlapping set of ~1,200+ objects rather than
    replacing the sound ones."""
    import bpy as _bpy
    for ob in list(coll.objects):
        md = ob.data
        _bpy.data.objects.remove(ob, do_unlink=True)
        if md is not None and md.users == 0:
            if isinstance(md, _bpy.types.Mesh):
                _bpy.data.meshes.remove(md)


def _place_bolt(bolt_id, pos, normal, tangent, protos, coll, log, mats=None,
                state="SOUND", mark_angle_deg=0.0):
    """One bolt. `state`: SOUND, LOOSE (mark_angle_deg > 0), MISSING (hole
    only), CORRODED (rust-tinted, mark present but degraded). Returns a
    record dict."""
    head_p, washer_p, mark_p, hole_p = protos
    rot = _frame(normal, tangent)
    made = []

    if state == "MISSING":
        h = ML.link_dup(hole_p, f"{bolt_id}_HOLE", pos, rot, (1, 1, 1), coll)
        made.append(h)
    else:
        w = ML.link_dup(washer_p, f"{bolt_id}_WASHER", pos, rot, (1, 1, 1), coll)
        head_pos = tuple(pos[i] + normal[i] * 0.003 for i in range(3))
        h = ML.link_dup(head_p, f"{bolt_id}_NUT", head_pos, rot, (1, 1, 1), coll)
        made += [w, h]
        if state == "CORRODED" and mats and mats.get("steel_bolt_corroded"):
            # Per-OBJECT material-slot override: the nut/washer share a
            # linked-duplicate MESH with every sound bolt (that is what
            # keeps 1,200 bolts cheap), so a DATA-level material swap would
            # recolour all of them. link='OBJECT' recolours only this one.
            corroded = mats["steel_bolt_corroded"]
            for ob in (w, h):
                ob.material_slots[0].link = "OBJECT"
                ob.material_slots[0].material = corroded
        # match mark: plate segment fixed at angle 0, nut segment at
        # mark_angle_deg -- collinear (continuous) unless LOOSE/CORRODED
        plate_rot = rot.copy()
        m1 = ML.link_dup(mark_p, f"{bolt_id}_MARK_PLATE",
                        tuple(pos[i] + normal[i] * 0.006 for i in range(3)),
                        plate_rot, (1, 1, 1), coll)
        nut_rot = _frame(normal, tangent, spin_deg=mark_angle_deg)
        m2 = ML.link_dup(mark_p, f"{bolt_id}_MARK_NUT",
                        tuple(pos[i] + normal[i] * (0.003 + P_H) for i in range(3)),
                        nut_rot, (1, 1, 1), coll)
        made += [m1, m2]

    for ob in made:
        ob["avi_kind"] = "fastener_part"
    return made


P_H = 0.0   # patched to BOLT_HEAD_H by build() -- keeps _place_bolt's
            # signature free of a params argument


def build(params, colls, mats, truss_joints, records_state=None, log=print):
    """Places ~1,200 bolts. `records_state` (optional) maps bolt_id ->
    (state, mark_angle_deg) for the subset damage_steel.py has chosen as
    defects; everything else is SOUND. Returns the full bolt manifest
    (id, assembly, position, normal, tangent, state) for damage_steel.py to
    select from on the FIRST call (records_state=None), and is called AGAIN
    after selection to actually build the defect states -- see
    build_final.py's two-pass wiring."""
    global P_H
    P_H = params.BOLT_HEAD_H
    import bpy
    coll = colls["FASTENERS"]
    protos = _build_protos(coll, mats, params)
    rnd = random.Random(params.SEED + 7777)
    records_state = records_state or {}

    manifest = []
    n_by_assembly = {}

    # ---- GUSSET + FLOOR_STRINGER: real joints from steel_final.py ---------
    # Bolts per joint are the assembly's stated target divided evenly across
    # however many real joints of that kind exist (with the remainder handed
    # to the first few), so the actual total matches SPEC's per-assembly
    # count exactly rather than an area-heuristic approximation of it.
    joints_by_assembly = {}
    for j in truss_joints:
        joints_by_assembly.setdefault(j[5], []).append(j)
    per_joint_count = {}
    for assembly, js in joints_by_assembly.items():
        target = params.FASTENER_ASSEMBLY_TARGETS[assembly]
        base, extra = divmod(target, len(js))
        for idx, j in enumerate(js):
            per_joint_count[j[0]] = base + (1 if idx < extra else 0)

    for jname, centre, normal, tangent, size, assembly in truss_joints:
        n_here = max(1, per_joint_count[jname])
        w, h = size
        cols_ = max(2, int(math.sqrt(n_here * w / max(h, 0.05))))
        rows_ = max(2, (n_here + cols_ - 1) // cols_)
        t = _norm(tangent)
        nrm = _norm(normal)
        b = _cross(nrm, t)
        k = 0
        for r in range(rows_):
            for c in range(cols_):
                if k >= n_here:
                    break
                u = (c - (cols_ - 1) / 2.0) * min(0.14, w / (cols_ + 1))
                v = (r - (rows_ - 1) / 2.0) * min(0.14, h / (rows_ + 1))
                pos = tuple(centre[i] + t[i] * u + b[i] * v for i in range(3))
                bid = f"FAST_{assembly}_{jname}_{k:02d}"
                manifest.append({"id": bid, "assembly": assembly,
                                "position": pos, "normal": nrm,
                                "tangent": t,
                                "host_object": _true_host_name(jname)})
                n_by_assembly[assembly] = n_by_assembly.get(assembly, 0) + 1
                k += 1

    # ---- BEARING: every existing BR_BEARING_*/MB_BEARING_* + truss --------
    bearing_objs = [o for o in bpy.data.objects
                   if o.type == "MESH" and "BEARING" in o.name
                   and o.name.startswith(("BR_", "MB_", "ST_"))]
    n_bear = params.FASTENER_ASSEMBLY_TARGETS["BEARING"]
    # divmod, not floor division, so the total hits n_bear exactly instead of
    # silently undercounting whenever n_bear isn't a multiple of the object
    # count (SPEC's per-assembly targets are asserted to sum to 1200).
    bear_base, bear_extra = divmod(n_bear, max(1, len(bearing_objs)))
    k = 0
    for bi, ob in enumerate(bearing_objs):
        per = bear_base + (1 if bi < bear_extra else 0)
        xs = [v.co.x for v in ob.data.vertices]
        ys = [v.co.y for v in ob.data.vertices]
        zs = [v.co.z for v in ob.data.vertices]
        cx, cy, ztop = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0, max(zs)
        loc = ob.matrix_world.translation
        cx, cy, ztop = cx + loc.x, cy + loc.y, ztop + loc.z
        for i in range(per):
            if k >= n_bear:
                break
            off = (i - (per - 1) / 2.0) * 0.15
            bid = f"FAST_BEARING_{ob.name}_{i:02d}"
            manifest.append({"id": bid, "assembly": "BEARING",
                            "position": (cx + off, cy, ztop),
                            "normal": (0, 0, 1), "tangent": (1, 0, 0),
                            "host_object": ob.name})
            n_by_assembly["BEARING"] = n_by_assembly.get("BEARING", 0) + 1
            k += 1

    # ---- JOINT_ANCHOR: at BR_JOINT_*_GAP locations -------------------------
    joint_objs = [o for o in bpy.data.objects
                 if o.type == "MESH" and o.name.startswith("BR_JOINT_")
                 and o.name.endswith("_GAP")]
    n_anchor = params.FASTENER_ASSEMBLY_TARGETS["JOINT_ANCHOR"]
    n_slots = max(1, len(joint_objs) * 2)   # 2 sides (s in (1,-1)) per joint
    anchor_base, anchor_extra = divmod(n_anchor, n_slots)
    k = 0
    slot = 0
    for ob in joint_objs:
        xs = [v.co.x for v in ob.data.vertices]
        ys = [v.co.y for v in ob.data.vertices]
        zs = [v.co.z for v in ob.data.vertices]
        cx, ztop = (min(xs) + max(xs)) / 2.0, max(zs)
        for s in (1, -1):
            per = anchor_base + (1 if slot < anchor_extra else 0)
            slot += 1
            for i in range(per):
                if k >= n_anchor:
                    break
                y = s * params.DECK_WIDTH * 0.3 + i * 0.3
                bid = f"FAST_JOINT_ANCHOR_{ob.name}_{s}_{i:02d}"
                manifest.append({"id": bid, "assembly": "JOINT_ANCHOR",
                                "position": (cx, y, ztop),
                                "normal": (0, 0, 1), "tangent": (1, 0, 0),
                                "host_object": ob.name})
                n_by_assembly["JOINT_ANCHOR"] = n_by_assembly.get(
                    "JOINT_ANCHOR", 0) + 1
                k += 1

    # ---- WALKWAY_BRACKET / CABLE_CLAMP / HANDRAIL_BASE: new geometry, -----
    # spread along the WHOLE corridor, not just the truss
    for assembly, spacing, mat_key, kind in (
        ("WALKWAY_BRACKET", 12.0, "steel_dark", "walkway_bracket"),
        ("CABLE_CLAMP", 18.0, "steel_dark", "cable_clamp"),
        ("HANDRAIL_BASE", 9.0, "concrete_parapet", "handrail_base"),
    ):
        target = params.FASTENER_ASSEMBLY_TARGETS[assembly]
        xs = []
        x = spacing / 2.0
        while x < params.BRIDGE_LENGTH:
            xs.append(x)
            x += spacing
        bracket_base, bracket_extra = divmod(target, max(1, len(xs)))
        k = 0
        for xi, x in enumerate(xs):
            n_per = bracket_base + (1 if xi < bracket_extra else 0)
            z_deck = params.deck_top_z(x)
            if assembly == "WALKWAY_BRACKET":
                y = -params.DECK_WIDTH / 2.0 - 0.3
                z = z_deck - params.WEARING_COURSE_T - params.DECK_SLAB_T - 0.2
                br = ML.box(f"ST_{kind.upper()}_{xi:03d}", (0.4, 0.5, 0.15),
                           (x, y, z), colls["FASTENERS"], mats[mat_key])
                ML.set_custom(br, {"avi_kind": kind})
                nrm, tan = (0, -1, 0), (1, 0, 0)
                pos_base = (x, y - 0.2, z)
            elif assembly == "CABLE_CLAMP":
                y = params.DECK_WIDTH / 2.0 + 0.25
                z = z_deck - params.WEARING_COURSE_T - params.DECK_SLAB_T - 0.15
                br = ML.box(f"ST_{kind.upper()}_{xi:03d}", (0.25, 0.15, 0.15),
                           (x, y, z), colls["FASTENERS"], mats[mat_key])
                ML.set_custom(br, {"avi_kind": kind})
                nrm, tan = (0, 1, 0), (1, 0, 0)
                pos_base = (x, y + 0.1, z)
            else:
                y = params.DECK_WIDTH / 2.0 - params.PARAPET_THICK / 2.0
                z = z_deck
                br = ML.box(f"ST_{kind.upper()}_{xi:03d}", (0.25, 0.25, 0.08),
                           (x, y, z + 0.04), colls["FASTENERS"], mats[mat_key])
                ML.set_custom(br, {"avi_kind": kind})
                nrm, tan = (0, 0, 1), (1, 0, 0)
                pos_base = (x, y, z + 0.08)
            for i in range(n_per):
                if k >= target:
                    break
                bid = f"FAST_{assembly}_{xi:03d}_{i:02d}"
                off = (i - (n_per - 1) / 2.0) * 0.08
                manifest.append({"id": bid, "assembly": assembly,
                                "position": (pos_base[0] + off, pos_base[1],
                                            pos_base[2]),
                                "normal": nrm, "tangent": tan,
                                "host_object": br.name})
                n_by_assembly[assembly] = n_by_assembly.get(assembly, 0) + 1
                k += 1

    # ---- build the actual bolt geometry for every manifest entry -----------
    n_obj = 0
    for rec in manifest:
        state, angle = records_state.get(rec["id"], ("SOUND", 0.0))
        made = _place_bolt(rec["id"], rec["position"], rec["normal"],
                          rec["tangent"], protos, coll, log, mats=mats,
                          state=state, mark_angle_deg=angle)
        n_obj += len(made)
        rec["state"] = state
        rec["mark_angle_deg"] = angle

    log(f"  bolts   : {len(manifest)} fasteners -> {n_by_assembly}, "
        f"{n_obj} objects")
    return manifest

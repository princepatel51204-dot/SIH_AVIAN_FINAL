"""SIH_AVIAN_FINAL -- detection pass: the steel through-truss main span.

Replaces the concrete main span's girders/diaphragms/drains/service-duct
(x=135..225) with a bolted Warren-with-verticals through-truss. The piers at
x=135/225 and the deck slab/wearing/parapet/median for this span are
bridge.py's own concrete, UNCHANGED -- build_final.py deletes only the
superstructure objects this replaces, before calling this module.

Built entirely from meshlib.box() (built-up box sections, not cylinders, per
the brief) with explicit avi_kind tags added to STRUCTURAL_KINDS
(export_bridge_collision.py) so the truss reaches collision the same way the
drone-base pads do -- classification by avi_kind, not by guessing a prefix
covers it.

Returns, alongside the usual stats, a list of GUSSET and FLOOR/STRINGER
connection points (name, centre, outward normal, tangent, face size) for
fasteners_final.py to bolt -- built here because only this module knows
where a real joint actually is.
"""
from __future__ import annotations
import math

import meshlib as ML


def _box(name, size, centre, coll, mat, kind):
    ob = ML.box(name, size, centre, coll, mat)
    ML.set_custom(ob, {"avi_kind": kind, "avi_structure": "TRUSS"})
    return ob


def _chord(y, z, coll, mats, tag):
    w, d = _P.TRUSS_CHORD_SIZE
    L = _P.TRUSS_X1 - _P.TRUSS_X0
    cx = (_P.TRUSS_X0 + _P.TRUSS_X1) / 2.0
    return _box(f"ST_CHORD_{tag}", (L, w, d), (cx, y, z), coll,
               mats["steel_struct"], "truss_chord")


def _diag_len(dx, dz):
    return math.hypot(dx, dz)


def _member_between(name, x0, z0, x1, z1, y, size, coll, mats, kind):
    """A box member between two (x,z) points at fixed y, oriented along its
    own axis -- built LOCAL-origin-centred then placed via location/rotation
    (meshlib bakes `centre` into vertices, which cannot then be rotated in
    place; see vehicles_final.py's _place() for the same pattern)."""
    dx, dz = x1 - x0, z1 - z0
    length = _diag_len(dx, dz)
    w, d = size
    ob = ML.box(name, (length, w, d), (0, 0, 0), coll, mats["steel_struct"])
    ang = math.atan2(dz, dx)
    ob.location = ((x0 + x1) / 2.0, y, (z0 + z1) / 2.0)
    ob.rotation_euler = (0.0, -ang, 0.0)
    ML.set_custom(ob, {"avi_kind": kind, "avi_structure": "TRUSS"})
    return ob


def _gusset(name, x, y, z, coll, mats, horizontal=True):
    w, h = _P.GUSSET_SIZE
    if horizontal:
        size = (w, _P.GUSSET_THICK, h)
    else:
        size = (w, h, _P.GUSSET_THICK)
    ob = ML.box(name, size, (x, y, z), coll, mats["steel_gusset"])
    ML.set_custom(ob, {"avi_kind": "gusset_plate", "avi_structure": "TRUSS"})
    return ob


_P = None   # set by build() -- avoids importing params_final at module load
            # time so this module stays a plain function library


def build(params, colls, mats, log=print):
    global _P
    _P = params
    coll = colls["STEEL"]
    pts = params.truss_panel_points()
    n = len(pts) - 1
    y_bot, y_top = params.TRUSS_BOTTOM_CHORD_Z, params.TRUSS_TOP_CHORD_Z
    made = []
    joints = []   # (name, centre, normal, tangent, size) for fasteners

    for side, ys in ((0, -params.TRUSS_Y), (1, params.TRUSS_Y)):
        tag = "S" if side == 0 else "N"

        # ---- chords -------------------------------------------------------
        made.append(_chord(ys, y_bot, coll, mats, f"BOT_{tag}"))
        made.append(_chord(ys, y_top, coll, mats, f"TOP_{tag}"))

        # ---- verticals (every panel point, incl. end posts) ---------------
        for i, x in enumerate(pts):
            made.append(_member_between(
                f"ST_VERTICAL_{tag}_{i:02d}", x, y_bot, x, y_top, ys,
                params.TRUSS_VERTICAL_SIZE, coll, mats,
                "truss_vertical"))

        # ---- diagonals (Warren zigzag, one per panel) ----------------------
        for i in range(n):
            x0, x1 = pts[i], pts[i + 1]
            if i % 2 == 0:
                z0, z1 = y_bot, y_top     # rising left-to-right
            else:
                z0, z1 = y_top, y_bot     # falling left-to-right
            made.append(_member_between(
                f"ST_DIAGONAL_{tag}_{i:02d}", x0, z0, x1, z1, ys,
                params.TRUSS_DIAGONAL_SIZE, coll, mats, "truss_diagonal"))

        # ---- gusset plates: every panel point, top and bottom -------------
        # A gusset plate is bolted to the OUTWARD (inward-toward-the-
        # corridor) face of the chord, not floating at the chord's own
        # centreline -- `ys` IS that centreline, and the chord is
        # TRUSS_CHORD_SIZE[0] wide there, so a plate (and every bolt on it)
        # built at exactly `ys` sits buried half INSIDE the solid chord
        # member, invisible to any camera on the correct (inward) side.
        # Caught by CAM_12_LOOSE_BOLT's own render: the camera looked
        # straight into the chord's near face from 0.3 m away, not at the
        # bolt at all. `sign` matches `nrm`'s own sign convention below --
        # y_plate is the plate's own centre (its near face flush against
        # the chord), y_face is where a bolt head standing on its outward
        # face actually sits (what fasteners_final.py needs).
        sign = 1.0 if side == 0 else -1.0
        half_chord_w = params.TRUSS_CHORD_SIZE[0] / 2.0
        half_gusset_t = params.GUSSET_THICK / 2.0
        y_plate = ys + sign * (half_chord_w + half_gusset_t)
        y_face = ys + sign * (half_chord_w + params.GUSSET_THICK)
        for i, x in enumerate(pts):
            g_bot = _gusset(f"ST_GUSSET_{tag}_BOT_{i:02d}", x, y_plate, y_bot,
                            coll, mats, horizontal=True)
            g_top = _gusset(f"ST_GUSSET_{tag}_TOP_{i:02d}", x, y_plate, y_top,
                            coll, mats, horizontal=True)
            made += [g_bot, g_top]
            nrm = (0.0, sign, 0.0)
            joints.append((g_bot.name, (x, y_face, y_bot), nrm, (1, 0, 0),
                          params.GUSSET_SIZE, "GUSSET"))
            joints.append((g_top.name, (x, y_face, y_top), nrm, (1, 0, 0),
                          params.GUSSET_SIZE, "GUSSET"))

    # ---- top lateral bracing: X-pattern between the two top chords --------
    bw, bd = params.TRUSS_BRACING_SIZE
    for i in range(n):
        x0, x1 = pts[i], pts[i + 1]
        for s, nm in ((1, "A"), (-1, "B")):
            y0 = -params.TRUSS_Y if s > 0 else params.TRUSS_Y
            y1 = params.TRUSS_Y if s > 0 else -params.TRUSS_Y
            length = math.hypot(x1 - x0, y1 - y0)
            ob = ML.box(f"ST_BRACING_TOP_{i:02d}_{nm}", (length, bw, bd),
                       (0, 0, 0), coll, mats["steel_struct"])
            ang = math.atan2(y1 - y0, x1 - x0)
            ob.location = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, y_top)
            ob.rotation_euler = (0.0, 0.0, ang)
            ML.set_custom(ob, {"avi_kind": "bracing", "avi_structure": "TRUSS"})
            made.append(ob)

    # ---- portal frames at both ends ----------------------------------------
    for x in (params.TRUSS_X0, params.TRUSS_X1):
        strut = _box(f"ST_PORTAL_STRUT_{x:.0f}",
                    (bw, 2 * params.TRUSS_Y + bw, bd),
                    (x, 0.0, y_top), coll, mats["steel_struct"], "bracing")
        made.append(strut)
        for s in (-1, 1):
            ys = s * params.TRUSS_Y
            knee = ML.box(f"ST_PORTAL_KNEE_{x:.0f}_{'A' if s>0 else 'B'}",
                         (bw, bw, params.TRUSS_DEPTH * 0.28),
                         (0, 0, 0), coll, mats["steel_struct"])
            knee.location = (x, ys * 0.82, y_top - params.TRUSS_DEPTH * 0.12)
            knee.rotation_euler = (math.radians(28.0) * (-s if x < 180 else s),
                                  0.0, 0.0)
            ML.set_custom(knee, {"avi_kind": "bracing", "avi_structure": "TRUSS"})
            made.append(knee)

    # ---- floor beams (every panel point) + stringers -----------------------
    fw, fd = params.FLOOR_BEAM_SIZE
    floor_top_z = y_bot + params.TRUSS_CHORD_SIZE[1] / 2.0
    for i, x in enumerate(pts):
        deck_under = params.deck_top_z(x) - params.WEARING_COURSE_T - params.DECK_SLAB_T
        depth = max(0.5, deck_under - floor_top_z)
        fb = _box(f"ST_FLOORBEAM_{i:02d}", (fw, 2 * params.TRUSS_Y, depth),
                 (x, 0.0, floor_top_z + depth / 2.0), coll, mats["steel_struct"],
                 "floor_beam")
        made.append(fb)
        # unique names per end -- steel_final.py's fb.name would collide
        # between the two ends otherwise, corrupting fasteners_final.py's
        # per-joint even bolt distribution (a dict keyed on this name)
        joints.append((f"{fb.name}_S", (x, -params.TRUSS_Y, floor_top_z + depth / 2.0),
                      (0, -1, 0), (0, 0, 1), (fw, depth), "FLOOR_STRINGER"))
        joints.append((f"{fb.name}_N", (x, params.TRUSS_Y, floor_top_z + depth / 2.0),
                      (0, 1, 0), (0, 0, 1), (fw, depth), "FLOOR_STRINGER"))

    sw, sd = params.STRINGER_SIZE
    stringer_ys = [(-params.TRUSS_Y + (i + 1) * 2 * params.TRUSS_Y
                   / (params.STRINGER_COUNT + 1))
                  for i in range(params.STRINGER_COUNT)]
    stringer_top = floor_top_z + max(
        0.5, params.deck_top_z((params.TRUSS_X0 + params.TRUSS_X1) / 2.0)
        - params.WEARING_COURSE_T - params.DECK_SLAB_T - floor_top_z) - sd / 2.0
    for j, sy in enumerate(stringer_ys):
        L = params.TRUSS_X1 - params.TRUSS_X0
        cx = (params.TRUSS_X0 + params.TRUSS_X1) / 2.0
        st = _box(f"ST_STRINGER_{j:02d}", (L, sw, sd), (cx, sy, stringer_top),
                 coll, mats["steel_struct"], "stringer")
        made.append(st)
        for i, x in enumerate(pts[:-1]):
            xm = (pts[i] + pts[i + 1]) / 2.0
            joints.append((f"{st.name}_J{i:02d}", (xm, sy, stringer_top),
                          (0, 0, 1), (1, 0, 0), (sw, sw), "FLOOR_STRINGER"))

    log(f"  truss   : {len(made)} members, {len(pts)} panel points, "
        f"{n} panels x {params.TRUSS_PANEL_L:.2f} m, "
        f"air draft {params.truss_air_draft():.2f} m")
    log(f"  truss   : {sum(1 for j in joints if j[5]=='GUSSET')} gusset "
        f"joints, {sum(1 for j in joints if j[5]=='FLOOR_STRINGER')} "
        f"floor/stringer joints")
    return {"objects": made, "joints": joints,
           "stats": {"members": len(made), "panels": n,
                     "air_draft_m": round(params.truss_air_draft(), 3)}}

"""Structural defect generation with full ground truth.

HYBRID MODEL
------------
Defects are split by what a sensor could actually detect:

  GEOMETRY   spalling, delamination, exposed reinforcement, repair patches.
             Real displaced mesh, so a depth camera or LiDAR registers them.
             These have measurable depth -- 10 to 60 mm -- which is at or
             above the resolution of a close-range depth sensor.

  SHADER     hairline, longitudinal, transverse and diagonal cracks, crack
             networks, corrosion staining, joint deterioration. Carried on a
             thin decal plane with a procedural crack material.
             A 0.1 mm crack has no geometry any practical mesh could hold and
             no LiDAR resolves it; what a camera sees is a low-contrast dark
             line. Modelling it as geometry would be physically dishonest AND
             would blow the polygon budget.

PLACEMENT
---------
Not scattered. Defects are placed where concrete actually fails:

  * near expansion joints          (water ingress, cyclic movement)
  * at bearing seats               (concentrated load, restraint)
  * at the girder web/flange line  (shear, and the shear crack is diagonal)
  * at drainage outlets            (chronic wetting -> leaching, corrosion)
  * at the waterline on river piers(wet/dry cycling, abrasion)
  * at pier cap ends               (torsion and bursting)
  * at deck slab midspan soffit    (flexural, and those cracks run transverse)

Severity is drawn from a distribution weighted toward minor. Most of the
structure is healthy: that is what a real bridge looks like, and an
environment where every surface is cracked teaches a detector nothing.

NON-REPAIRABLE CASES
--------------------
A deliberate minority of defects sit outside the autonomous repair envelope
(severity 4, area too large, crack too wide, or exposed reinforcement). Those
exist so the later system has something it must correctly REFUSE to repair.
"""
from __future__ import annotations
import json
import math
import os
import random

import bpy
from mathutils import Vector, Matrix

import params as P
import meshlib as ML
import materials as MAT


# ===========================================================================
# HOST SURFACE CATALOGUE
# ===========================================================================
class Site:
    """A candidate defect location on a real surface.

    Carries everything the ground truth needs: where it is, which surface it
    is on, which way that surface faces, and how hard it is to see.
    """

    __slots__ = ("pos", "normal", "tangent", "host", "obj_name", "section",
                 "sector", "occlusion", "reason", "u_max", "v_max")

    def __init__(self, pos, normal, tangent, host, obj_name, reason,
                 occlusion=0.0, u_max=2.0, v_max=2.0):
        self.pos = Vector(pos)
        self.normal = Vector(normal).normalized()
        self.tangent = Vector(tangent).normalized()
        self.host = host
        self.obj_name = obj_name
        self.reason = reason
        self.occlusion = occlusion
        # How much room the defect actually has on this surface, along the
        # tangent (u) and the binormal (v). A decal is a flat rectangle: if it
        # is larger than the face it sits on, it hangs over the edge into open
        # air and reappears clipped through the adjacent member. On a 0.70 m
        # bottom flange or a 1.5 m diaphragm that happens constantly, and it
        # is only visible at close range -- which is exactly the range this
        # model exists to be looked at from.
        self.u_max = u_max
        self.v_max = v_max
        self.section = P.section_at(pos[0])
        self.sector = P.sector_at(pos[0])


def collect_sites(rnd):
    """Enumerate engineering-plausible defect locations in the research zone.

    `occlusion` is a 0-1 estimate of how hard the site is to see: 0 is an
    open surface reachable from any direction, 1 is deep between two girders
    with a diaphragm alongside. It is written into the ground truth so later
    detection results can be scored against viewpoint difficulty rather than
    lumped together.
    """
    sites = []
    ps = [x for x, k in P.pier_stations()
          if P.RESEARCH_X0 - 60 <= x <= P.RESEARCH_X1 + 60]
    y_g0 = -P.GIRDER_SPACING * (P.GIRDER_COUNT - 1) / 2.0

    # ---- 1. deck soffit at midspan (flexural, transverse) ----------------
    for i in range(len(ps) - 1):
        x0, x1 = ps[i], ps[i + 1]
        if not (P.RESEARCH_X0 <= 0.5 * (x0 + x1) <= P.RESEARCH_X1):
            continue
        xm = 0.5 * (x0 + x1)
        depth = P.girder_depth_for_span(x1 - x0)
        z = P.deck_top_z(xm) - P.WEARING_COURSE_T - P.DECK_SLAB_T
        for b in range(P.GIRDER_COUNT - 1):
            y = y_g0 + (b + 0.5) * P.GIRDER_SPACING
            sites.append(Site(
                (xm + rnd.uniform(-6, 6), y + rnd.uniform(-1.2, 1.2), z),
                (0, 0, -1), (1, 0, 0), "DECK_UNDERSIDE",
                f"BR_DECK_SLAB_{i+1:03d}",
                "midspan flexural zone, deck soffit between girders",
                occlusion=0.55,
                u_max=3.5, v_max=P.GIRDER_SPACING - P.GIRDER_TOP_FLANGE_W
                - 0.9))

    # ---- 2. girder webs: shear zone near supports (diagonal) -------------
    for i in range(len(ps) - 1):
        x0, x1 = ps[i], ps[i + 1]
        span = x1 - x0
        depth = P.girder_depth_for_span(span)
        ztop = P.deck_top_z(0.5 * (x0 + x1)) - P.WEARING_COURSE_T \
            - P.DECK_SLAB_T
        for end_x in (x0 + span * 0.14, x1 - span * 0.14):
            if not (P.RESEARCH_X0 <= end_x <= P.RESEARCH_X1):
                continue
            for g in range(P.GIRDER_COUNT):
                y = y_g0 + g * P.GIRDER_SPACING
                inner = 0 < g < P.GIRDER_COUNT - 1
                for s in ((1, -1) if not inner else (1,)):
                    sites.append(Site(
                        (end_x + rnd.uniform(-3, 3),
                         y + s * (P.GIRDER_WEB_T / 2 + 0.01),
                         ztop - depth * rnd.uniform(0.35, 0.65)),
                        (0, s, 0), (1, 0, 0), "GIRDER_WEB",
                        f"BR_GIRDER_{i+1:03d}_G{g+1}",
                        "shear zone near support, girder web",
                        occlusion=0.75 if inner else 0.35,
                        u_max=3.0,
                        v_max=max(0.30, depth - 2 * P.GIRDER_FLANGE_T
                                  - 0.30)))

    # ---- 3. girder bottom flange (corrosion, cover loss) -----------------
    for i in range(len(ps) - 1):
        xm = 0.5 * (ps[i] + ps[i + 1])
        if not (P.RESEARCH_X0 <= xm <= P.RESEARCH_X1):
            continue
        span = ps[i + 1] - ps[i]
        depth = P.girder_depth_for_span(span)
        z = P.deck_top_z(xm) - P.WEARING_COURSE_T - P.DECK_SLAB_T - depth
        for g in range(P.GIRDER_COUNT):
            y = y_g0 + g * P.GIRDER_SPACING
            sites.append(Site(
                (xm + rnd.uniform(-span * 0.3, span * 0.3),
                 y + rnd.uniform(-0.10, 0.10), z),
                (0, 0, -1), (1, 0, 0), "GIRDER_BOTTOM_FLANGE",
                f"BR_GIRDER_{i+1:03d}_G{g+1}",
                "bottom flange soffit, minimum cover face",
                occlusion=0.45,
                u_max=2.5, v_max=P.GIRDER_BOT_FLANGE_W - 0.14))

    # ---- 4. diaphragm faces (confined, hard to reach) --------------------
    for i in range(len(ps) - 1):
        x0, x1 = ps[i], ps[i + 1]
        span = x1 - x0
        n_dia = max(2, int(span / 12.0) + 1)
        depth = P.girder_depth_for_span(span)
        ztop = P.deck_top_z(0.5 * (x0 + x1)) - P.WEARING_COURSE_T \
            - P.DECK_SLAB_T
        for d in range(1, n_dia - 1):
            dx = x0 + span * d / (n_dia - 1)
            if not (P.RESEARCH_X0 <= dx <= P.RESEARCH_X1):
                continue
            dd = min(P.DIAPHRAGM_DEPTH, depth - 0.5)
            for s in (1, -1):
                sites.append(Site(
                    (dx + s * (P.DIAPHRAGM_T / 2 + 0.01),
                     rnd.uniform(-6, 6),
                     ztop - P.GIRDER_FLANGE_T - dd * rnd.uniform(0.3, 0.7)),
                    (s, 0, 0), (0, 1, 0), "DIAPHRAGM",
                    f"BR_DIAPHRAGM_{i+1:03d}_{d+1}",
                    "cross diaphragm face, confined bay",
                    occlusion=0.85,
                    u_max=2.5, v_max=max(0.30, dd - 0.40)))

    # ---- 5. pier columns: waterline and mid-height -----------------------
    for i, (x, kind) in enumerate(P.pier_stations()):
        if not (P.RESEARCH_X0 <= x <= P.RESEARCH_X1):
            continue
        gz = P.ground_z(x, 0.0)
        in_river = kind == "river" and gz < P.RIVER_WATER_Z - 0.5
        r = (P.PIER_COL_D_RIVER if in_river else P.PIER_COL_D) / 2.0
        span = P.span_at(x)
        cap_bot = (P.deck_top_z(x) - P.WEARING_COURSE_T - P.DECK_SLAB_T
                   - P.girder_depth_for_span(span) - P.BEARING_H
                   - P.PIER_CAP_H)
        for c in range(P.PIER_COLS_PER_BENT):
            yc = (c - (P.PIER_COLS_PER_BENT - 1) / 2.0) * P.PIER_COL_SPACING
            # waterline band -- wet/dry cycling, the classic pier defect zone
            if in_river:
                for k in range(2):
                    a = rnd.uniform(0, 2 * math.pi)
                    zc = P.RIVER_WATER_Z + rnd.uniform(0.3, 3.2)
                    sites.append(Site(
                        (x + r * math.cos(a) * 1.002,
                         yc + r * math.sin(a) * 1.002, zc),
                        (math.cos(a), math.sin(a), 0), (0, 0, 1),
                        "PIER_COLUMN", f"BR_PIER_COL_{i+1:03d}_{c+1}",
                        "waterline wet/dry cycling band, river pier",
                        occlusion=0.30,
                        u_max=1.60, v_max=min(0.55, r * 0.80)))
            # general column surface
            for k in range(2):
                a = rnd.uniform(0, 2 * math.pi)
                zc = rnd.uniform(max(gz, P.RIVER_WATER_Z) + 2.0,
                                 cap_bot - 1.5)
                if zc <= max(gz, P.RIVER_WATER_Z) + 1.0:
                    continue
                sites.append(Site(
                    (x + r * math.cos(a) * 1.002,
                     yc + r * math.sin(a) * 1.002, zc),
                    (math.cos(a), math.sin(a), 0), (0, 0, 1),
                    "PIER_COLUMN", f"BR_PIER_COL_{i+1:03d}_{c+1}",
                    "column shaft, general surface",
                    occlusion=0.20,
                    u_max=1.60, v_max=min(0.55, r * 0.80)))

    # ---- 6. pier cap ends and bearing seats ------------------------------
    for i, (x, kind) in enumerate(P.pier_stations()):
        if not (P.RESEARCH_X0 <= x <= P.RESEARCH_X1):
            continue
        span = P.span_at(x)
        cap_top = (P.deck_top_z(x) - P.WEARING_COURSE_T - P.DECK_SLAB_T
                   - P.girder_depth_for_span(span) - P.BEARING_H)
        cap_bot = cap_top - P.PIER_CAP_H
        for s in (1, -1):
            sites.append(Site(
                (x + s * (P.PIER_CAP_W / 2 + 0.01),
                 rnd.uniform(-P.PIER_CAP_L / 2 + 1, P.PIER_CAP_L / 2 - 1),
                 cap_bot + P.PIER_CAP_H * rnd.uniform(0.3, 0.8)),
                (s, 0, 0), (0, 1, 0), "PIER_CAP",
                f"BR_PIER_CAP_{i+1:03d}",
                "pier cap face, bursting and torsion zone",
                occlusion=0.40,
                u_max=2.5, v_max=P.PIER_CAP_H - 0.60))
        for g in (0, P.GIRDER_COUNT - 1):
            y = y_g0 + g * P.GIRDER_SPACING
            if abs(y) > P.PIER_CAP_L / 2 - 0.4:
                continue
            sites.append(Site(
                (x + rnd.uniform(-0.5, 0.5), y, cap_top - 0.02),
                (0, 0, 1), (1, 0, 0), "BEARING_SEAT",
                f"BR_PIER_CAP_{i+1:03d}",
                "bearing seat, concentrated bearing stress",
                occlusion=0.70,
                u_max=0.80, v_max=0.60))

    # ---- 7. parapets (impact, de-icing salt, exposure) -------------------
    x = P.RESEARCH_X0
    while x < P.RESEARCH_X1:
        for s in (1, -1):
            sites.append(Site(
                (x, s * (P.DECK_WIDTH / 2 - P.PARAPET_THICK - 0.01),
                 P.deck_top_z(x) + rnd.uniform(0.2, 0.9)),
                (0, -s, 0), (1, 0, 0), "PARAPET",
                f"BR_PARAPET", "parapet inner face, exposure and impact",
                occlusion=0.10,
                u_max=2.0, v_max=P.PARAPET_HEIGHT - 0.40))
        x += rnd.uniform(45, 95)

    # Several categories are derived from pier stations collected with a 60 m
    # margin either side of the research zone, and a few of them -- diaphragms
    # near a span end, bottom-flange points offset by 0.3 of the span -- land
    # outside it. A defect outside the zone has no sector, which leaves a null
    # in the ground truth and breaks the promise that every defect is inside
    # the high-detail region.
    return [s for s in sites if s.sector is not None]


# ===========================================================================
# DEFECT CONSTRUCTION
# ===========================================================================
def _frame_from(site):
    """Build a rotation matrix whose +Z is the surface normal."""
    n = site.normal
    t = site.tangent - n * site.tangent.dot(n)
    if t.length < 1e-6:
        t = Vector((1, 0, 0)) if abs(n.z) > 0.9 else Vector((0, 0, 1))
        t = (t - n * t.dot(n))
    t.normalize()
    b = n.cross(t)
    return Matrix(((t.x, b.x, n.x, 0.0),
                   (t.y, b.y, n.y, 0.0),
                   (t.z, b.z, n.z, 0.0),
                   (0.0, 0.0, 0.0, 1.0)))


def _severity(rnd):
    r = rnd.random()
    acc = 0.0
    for s, w in sorted(P.SEVERITY_WEIGHTS.items()):
        acc += w
        if r <= acc:
            return s
    return 4


def _repairable(kind, sev, area, width_mm, length_m=0.0):
    """Is this inside the autonomous repair envelope?

    Returns (bool, reason). The reason string is what makes the ground truth
    useful for scoring an escalation decision later -- it is not enough to
    know a defect was rejected, the system has to reject it for the RIGHT
    reason.

    Cracks and area defects are judged on different quantities. A crack is
    repaired by sealing along its length, so what matters is its width and
    how far it runs -- not the footprint of the patch of wall it sits on.
    Testing a crack against the spall area limit rejected most of the
    moderate cracking in the model, which would have left the escalation
    set dominated by ordinary defects.
    """
    E = P.REPAIR_ENVELOPE
    if kind == "REBAR_EXPOSED" and E["rebar_exposed_blocks"]:
        return False, "exposed reinforcement requires structural assessment"
    if kind == "DELAMINATION":
        return False, "delamination extent cannot be bounded from the surface"
    if sev > E["max_severity"]:
        return False, (f"severity {sev} exceeds the maximum auto-repairable "
                       f"severity of {E['max_severity']}")
    if width_mm > 0.0:
        if width_mm > E["max_crack_width_mm"]:
            return False, (f"crack width {width_mm:.2f} mm exceeds the "
                           f"{E['max_crack_width_mm']:.1f} mm surface-seal "
                           "limit; structural investigation required")
        if length_m > E["max_crack_length_m"]:
            return False, (f"crack length {length_m:.2f} m exceeds the "
                           f"{E['max_crack_length_m']:.1f} m spot-repair "
                           "limit")
    elif area > E["max_area_m2"]:
        return False, (f"repair area {area:.2f} m2 exceeds the "
                       f"{E['max_area_m2']:.2f} m2 limit; requires access "
                       "platform")
    return True, "within surface-repair envelope"


def make_crack(idx, kind, site, sev, rnd, coll, mats):
    """Shader-based crack on a decal plane."""
    w0, w1 = P.CRACK_WIDTH_MM[sev]
    width_mm = rnd.uniform(w0, w1)
    size = {1: (0.35, 0.75), 2: (0.55, 1.25),
            3: (0.9, 2.0), 4: (1.4, 3.2)}[sev]
    L = rnd.uniform(*size)
    W = L * rnd.uniform(0.45, 0.95)
    # Never larger than the face it sits on. The decal is a flat rectangle,
    # so an oversized one hangs off the edge of a 0.70 m flange into open air.
    # A transverse crack is rotated 90 deg below, which swaps which of the
    # surface's two extents each side of the decal has to fit inside.
    if kind == "CRACK_TRANSVERSE":
        L = min(L, site.v_max)
        W = min(W, site.u_max)
    else:
        L = min(L, site.u_max)
        W = min(W, site.v_max)

    pattern = {"CRACK_LONGITUDINAL": "LINEAR",
               "CRACK_TRANSVERSE": "LINEAR",
               "CRACK_DIAGONAL": "DIAGONAL",
               "CRACK_NETWORK": "NETWORK",
               "CRACK_HAIRLINE": "LINEAR",
               "CORROSION_STAIN": "NETWORK",
               "JOINT_DETERIORATION": "NETWORK"}[kind]

    rust = 0.0
    if kind == "CORROSION_STAIN":
        rust = rnd.uniform(0.55, 0.95)
    elif sev >= 3 and rnd.random() < 0.45:
        rust = rnd.uniform(0.20, 0.55)

    name = f"DEFECT_{kind}_{idx:03d}"
    mat = MAT.crack_decal(f"MAT_{name}", sev, width_mm, pattern, rust,
                          seed=idx * 1.7)
    ob = ML.plane(name, L, W, (0, 0, 0), "Z", coll, mat)

    m = _frame_from(site)
    # a transverse crack runs across the member, a longitudinal one along it
    if kind == "CRACK_TRANSVERSE":
        m = m @ Matrix.Rotation(math.pi / 2, 4, "Z")
    m = m @ Matrix.Rotation(rnd.uniform(-0.25, 0.25), 4, "Z")
    m.translation = site.pos + site.normal * 0.006
    ob.matrix_world = m

    area = L * W
    ok, why = _repairable(kind, sev, area, width_mm, length_m=L)
    return ob, dict(width_mm=round(width_mm, 3), area_m2=round(area, 4),
                    length_m=round(L, 3), repairable=ok, reason=why,
                    depth_mm=0.0, representation="SHADER_DECAL")


def make_spall(idx, kind, site, sev, rnd, coll, mats):
    """Geometry-based spall, delamination, exposed rebar or repair patch.

    PROFILE
    -------
    Four rings rather than two. The first version put one shallow ring around a
    single deep centre point, which makes a cone, and a 35 mm cone across a
    460 mm opening has a wall slope of 8 degrees -- under diffuse under-deck
    light that shades identically to flat concrete and the defect vanished.

    A real spall has a broad floor and a steep, near-vertical broken rim, so
    that is what this builds: floor at full depth out to 0.60 R, a steep wall
    from 0.60 R to 0.94 R, and the rim closing to the surface at R. The rim is
    where the visual signal actually comes from.

      kind              profile      face material
      SPALL             cavity       fresh fracture, rusty at severity >= 3
      REBAR_EXPOSED     cavity       fresh fracture, always rusty
      DELAMINATION      bulge        intact surface, faint -- see the note in
                                     materials.delamination_face
      REPAIR_PATCH      proud        newer mortar, slightly wrong colour
    """
    dscale = {1: (0.008, 0.016), 2: (0.016, 0.030),
              3: (0.030, 0.048), 4: (0.045, 0.075)}[sev]
    depth = rnd.uniform(*dscale)
    rad = {1: (0.10, 0.20), 2: (0.18, 0.36),
           3: (0.30, 0.62), 4: (0.50, 1.05)}[sev]
    R = rnd.uniform(*rad)
    # The cavity is rotated arbitrarily about the surface normal, so it must
    # fit inside the SMALLER of the two surface extents in any orientation.
    # 1.35 = the largest lobe the boundary function below can produce.
    R = min(R, 0.5 * min(site.u_max, site.v_max) / 1.35)
    R = max(R, 0.06)

    # Delamination is not an open cavity: the cover is still there, it has
    # just lost bond. It reads as a shallow bulge, not a hole.
    if kind == "DELAMINATION":
        depth = -depth * 0.35
    elif kind == "REPAIR_PATCH":
        depth = -min(0.012, depth * 0.5)

    name = f"DEFECT_{kind}_{idx:03d}"
    seg = 24
    # The irregular boundary polygon, shared by the liner and the cutter. A
    # circular cut edge reads as machined; concrete does not break in circles.
    # Independent per-vertex jitter alternates in and out at every segment
    # and draws a star, not a broken edge. Two low-frequency lobes with random
    # phase give a correlated, organic outline; the small uniform term stops it
    # looking like a drawn curve.
    p1 = rnd.uniform(0, 6.283)
    p2 = rnd.uniform(0, 6.283)
    k1 = rnd.choice((2, 3, 3, 4))
    k2 = rnd.choice((5, 6, 7))
    bound = []
    for i in range(seg):
        a_i = 2 * math.pi * i / seg
        f = (1.0 + 0.20 * math.sin(k1 * a_i + p1)
             + 0.10 * math.sin(k2 * a_i + p2)
             + rnd.uniform(-0.035, 0.035))
        bound.append(R * f)

    # (radius factor relative to `bound`, depth fraction)
    rings = [(0.58, 1.00), (0.92, 0.60), (1.10, 0.00)]
    verts = [(0.0, 0.0, -depth)]
    for rf, df in rings:
        for i in range(seg):
            a = 2 * math.pi * i / seg
            rr = bound[i] * rf
            zz = -depth * df * (1.0 + rnd.uniform(-0.14, 0.14))
            verts.append((rr * math.cos(a), rr * math.sin(a), zz))
    faces = []
    for i in range(seg):
        j = (i + 1) % seg
        faces.append((0, 1 + i, 1 + j))
    for k in range(len(rings) - 1):
        b0 = 1 + k * seg
        b1 = b0 + seg
        for i in range(seg):
            j = (i + 1) % seg
            faces.append((b0 + i, b1 + i, b1 + j, b0 + j))

    rusty = kind == "REBAR_EXPOSED" or (kind == "SPALL" and sev >= 3)
    mat = {"REPAIR_PATCH": mats["repair_patch"],
           "DELAMINATION": mats["delamination"]}.get(
        kind, mats["spall_face_rust"] if rusty else mats["spall_face"])
    ob = ML.mesh_obj(name, verts, faces, coll, mat)

    m = _frame_from(site)
    m = m @ Matrix.Rotation(rnd.uniform(0, 6.28), 4, "Z")
    m.translation = site.pos + site.normal * 0.002
    ob.matrix_world = m

    extra = []
    # exposed reinforcement: bars crossing the cavity, corroded.
    # Bar diameter and cover are taken from the real numbers -- 16 mm bars at
    # 40 mm cover -- so the depth at which they appear is not arbitrary.
    if kind == "REBAR_EXPOSED":
        n_bar = rnd.randint(2, 3)
        pitch = min(0.11, R * 0.9)
        for b in range(n_bar):
            off = (b - (n_bar - 1) / 2.0) * pitch
            half = math.sqrt(max(0.0, R * R - off * off))
            bar = ML.cylinder(f"{name}_BAR{b+1}", 0.008,
                              max(0.05, 2.0 * half * 0.96),
                              (0, off, -depth * 0.72), 10,
                              coll, mats["rebar"], axis="X")
            bar.matrix_world = m @ bar.matrix_world
            extra.append(bar)

    # The cutter. Everything above exists inside the host solid, and a solid
    # occludes anything drawn inside it: the first version of this rendered as
    # a faint outline on an untouched soffit, because the host's own face was
    # still there in front of the cavity. A recess in a closed mesh has to be
    # cut, not drawn. carve() below subtracts these from the real host.
    cutter = None
    if depth > 0.0:
        cv, cf = [], []
        top = 0.03                       # well clear of the host surface
        # Deeper than the liner floor. Cutting to exactly -depth left the cut
        # face and the liner floor coplanar, and the two z-fought into a torn
        # mess of white slivers that looked like missing geometry.
        bot = -depth * 1.45
        for i in range(seg):
            a = 2 * math.pi * i / seg
            rr = bound[i]
            cv.append((rr * math.cos(a), rr * math.sin(a), bot))
        for i in range(seg):
            a = 2 * math.pi * i / seg
            rr = bound[i]
            cv.append((rr * math.cos(a), rr * math.sin(a), top))
        for i in range(seg):
            j = (i + 1) % seg
            cf.append((i, j, seg + j, seg + i))
        cf.append(tuple(range(seg - 1, -1, -1)))
        cf.append(tuple(range(seg, 2 * seg)))
        cutter = (cv, cf, m.copy())

    area = math.pi * R * R
    width_mm = 0.0
    ok, why = _repairable(kind, sev, area, width_mm)
    return ob, dict(width_mm=0.0, area_m2=round(area, 4),
                    radius_m=round(R, 3),
                    depth_mm=round(abs(depth) * 1000, 1),
                    repairable=ok, reason=why,
                    representation="GEOMETRY"), extra, cutter


# ===========================================================================
# CARVING THE HOST
# ===========================================================================
# Prefixes the host ray-cast steps past. DEFECT_ because a decal sits 6 mm
# proud of its own host; AVI_ because everything REV-B adds -- cable trays,
# brackets, airspace volumes, markers -- must not be able to intercept a ray
# and silently relocate a frozen defect onto a piece of secondary steelwork.
_RAY_SKIP_PREFIXES = ("DEFECT_", "AVI_", "_")


def _find_host(pos, normal, exclude_prefix=_RAY_SKIP_PREFIXES):
    """Ray-cast back into the surface to find the object a defect sits on.

    Names alone are not trustworthy here: the site catalogue numbers spans
    within the research window while the bridge builder numbers them along the
    whole corridor, so a name assembled from the site index can point at the
    wrong member. A ray finds the object that is actually there.

    Returns (object, world-space hit point, world-space face normal), or
    (None, None, None).
    """
    dg = bpy.context.evaluated_depsgraph_get()
    n = Vector(normal)
    dirv = -n
    org = Vector(pos) + n * 0.25
    remaining = 0.60
    # Step past defect objects rather than giving up on them. The first
    # version returned None as soon as the ray hit anything named DEFECT_,
    # and since a crack decal sits 6 mm proud of the concrete it is the FIRST
    # thing every ray meets -- which left 171 of 192 defects with no verified
    # host. Skipping and continuing is the whole difference.
    for _ in range(8):
        hit, loc, nrm, idx, ob, mw = bpy.context.scene.ray_cast(
            dg, org, dirv, distance=remaining)
        if not hit or ob is None:
            return None, None, None
        if not ob.name.startswith(tuple(exclude_prefix)):
            return ob, Vector(loc), Vector(nrm).normalized()
        step = (Vector(loc) - org).length + 1e-3
        remaining -= step
        if remaining <= 0.0:
            return None, None, None
        org = Vector(loc) + dirv * 1e-3
    return None, None, None


def carve(cutters, log=print):
    """Subtract every spall cavity from the member it sits on.

    One boolean per host, not one per defect: the cutters for a host are
    joined first, so a girder with six spalls is evaluated once. Blender's
    EXACT solver is used -- FAST produces self-intersecting results on the
    thin webs here.

    Failures are counted and reported rather than swallowed. A cavity whose
    cut failed is still present as a liner, so it degrades to the old
    behaviour rather than disappearing.
    """
    by_host = {}
    for host, verts, faces, _mat in cutters:
        by_host.setdefault(host.name, (host, []))[1].append((verts, faces))

    n_ok = n_fail = 0
    tmp = bpy.data.collections.new("_CARVE_TMP")
    bpy.context.scene.collection.children.link(tmp)
    for hname, (host, chunks) in by_host.items():
        v_all, f_all = [], []
        for verts, faces in chunks:
            off = len(v_all)
            v_all += verts
            f_all += [tuple(i + off for i in f) for f in faces]
        cob = ML.mesh_obj(f"_CUT_{hname}", v_all, f_all, tmp)
        try:
            mod = host.modifiers.new("SPALL_CUT", "BOOLEAN")
            mod.operation = "DIFFERENCE"
            mod.object = cob
            mod.solver = "EXACT"
            dg = bpy.context.evaluated_depsgraph_get()
            ev = host.evaluated_get(dg)
            new = bpy.data.meshes.new_from_object(ev)
            if len(new.polygons) == 0:
                raise RuntimeError("boolean produced an empty mesh")
            old = host.data
            host.modifiers.clear()
            host.data = new
            new.name = old.name + "_CUT"
            if old.users == 0:
                bpy.data.meshes.remove(old)
            n_ok += len(chunks)
        except Exception as e:                      # noqa: BLE001
            host.modifiers.clear()
            n_fail += len(chunks)
            log(f"  carve   : FAILED on {hname}: {e}")
        finally:
            m = cob.data
            bpy.data.objects.remove(cob, do_unlink=True)
            if m.users == 0:
                bpy.data.meshes.remove(m)
    bpy.data.collections.remove(tmp)
    log(f"  carve   : {n_ok} cavities cut into {len(by_host)} members"
        + (f", {n_fail} FAILED" if n_fail else ""))
    return {"cut": n_ok, "failed": n_fail, "hosts": len(by_host)}


# ===========================================================================
def build(colls, mats, log=print):
    rnd = random.Random(P.SEED + 97)
    sites = collect_sites(rnd)
    rnd.shuffle(sites)
    log(f"  sites   : {len(sites)} engineering-plausible candidate locations")

    coll_of = {
        "CRACK_HAIRLINE": "CRACKS", "CRACK_LONGITUDINAL": "CRACKS",
        "CRACK_TRANSVERSE": "CRACKS", "CRACK_DIAGONAL": "CRACKS",
        "CRACK_NETWORK": "CRACKS",
        "SPALL": "SPALLING", "DELAMINATION": "SPALLING",
        "REBAR_EXPOSED": "REBAR", "CORROSION_STAIN": "CORROSION",
        "JOINT_DETERIORATION": "JOINT_DAMAGE",
        "REPAIR_PATCH": "SPALLING",
    }
    # host preference per defect type -- this is what makes placement
    # plausible rather than random
    prefer = {
        "CRACK_TRANSVERSE": ("DECK_UNDERSIDE", "GIRDER_BOTTOM_FLANGE"),
        "CRACK_LONGITUDINAL": ("GIRDER_WEB", "PIER_COLUMN", "PARAPET"),
        "CRACK_DIAGONAL": ("GIRDER_WEB", "PIER_CAP", "DIAPHRAGM"),
        "CRACK_NETWORK": ("PIER_COLUMN", "PIER_CAP", "DECK_UNDERSIDE"),
        "CRACK_HAIRLINE": None,
        "SPALL": ("GIRDER_BOTTOM_FLANGE", "PIER_COLUMN", "PIER_CAP",
                  "DECK_UNDERSIDE"),
        "DELAMINATION": ("DECK_UNDERSIDE", "GIRDER_BOTTOM_FLANGE"),
        "REBAR_EXPOSED": ("GIRDER_BOTTOM_FLANGE", "PIER_COLUMN",
                          "DECK_UNDERSIDE"),
        "CORROSION_STAIN": ("GIRDER_WEB", "PIER_COLUMN", "BEARING_SEAT",
                            "PIER_CAP"),
        "JOINT_DETERIORATION": ("BEARING_SEAT", "PIER_CAP"),
        "REPAIR_PATCH": ("PIER_COLUMN", "GIRDER_WEB", "DECK_UNDERSIDE"),
    }

    used = set()
    records = []
    counts = {}
    n_obj = 0
    pending_cuts = []

    for kind, target in P.DEFECT_TARGETS.items():
        target = int(round(target * P.DAMAGE_DENSITY))
        pref = prefer.get(kind)
        pool = [s for s in sites
                if id(s) not in used and (pref is None or s.host in pref)]
        if len(pool) < target:
            pool += [s for s in sites if id(s) not in used and s not in pool]
        made = 0
        for s in pool:
            if made >= target:
                break
            # keep defects apart so they read as distinct features
            too_close = False
            for r in records[-60:]:
                if (Vector(r["position_m"]) - s.pos).length < 1.1:
                    too_close = True
                    break
            if too_close:
                continue
            used.add(id(s))
            sev = _severity(rnd)
            idx = made + 1
            cname = coll_of[kind]
            if kind in P.GEOMETRIC_DEFECTS:
                ob, info, extra, cut = make_spall(idx, kind, s, sev, rnd,
                                                  colls[cname], mats)
                n_obj += 1 + len(extra)
                if cut is not None:
                    pending_cuts.append((ob.name, cut[0], cut[1]))
            else:
                ob, info = make_crack(idx, kind, s, sev, rnd,
                                      colls[cname], mats)
                n_obj += 1

            rec = {
                "defect_id": ob.name,
                "type": kind,
                "severity": sev,
                "position_m": [round(v, 3) for v in s.pos],
                "surface_normal": [round(v, 3) for v in s.normal],
                "host_surface": s.host,
                "host_object": s.obj_name,
                "bridge_section": s.section,
                "inspection_sector": s.sector,
                "occlusion": round(s.occlusion, 2),
                "placement_rationale": s.reason,
            }
            rec.update(info)
            records.append(rec)
            ML.set_custom(ob, {
                "avi_defect_id": ob.name,
                "avi_type": kind,
                "avi_severity": sev,
                "avi_repairable": bool(info["repairable"]),
                "avi_reason": info["reason"],
                "avi_host_surface": s.host,
                "avi_host_object": s.obj_name,
                "avi_section": s.section,
                "avi_sector": s.sector or "OUTSIDE_RESEARCH_ZONE",
                "avi_occlusion": s.occlusion,
                "avi_representation": info["representation"],
                "avi_area_m2": info["area_m2"],
                "avi_width_mm": info["width_mm"],
                "avi_depth_mm": info["depth_mm"],
                "avi_rationale": s.reason,
            })
            made += 1
        counts[kind] = made

    # ---- resolve real host objects and cut the cavities ------------------
    # Done after every defect exists, so the ray-cast cannot hit a defect that
    # had not been created yet, and so each host is cut exactly once.
    bpy.context.view_layer.update()

    # ---- snap every defect onto the surface that is really there ---------
    # The site catalogue computes positions from the design dimensions, but a
    # circular pier is modelled as a 16-sided prism, so a point at the design
    # radius floats up to 30 mm off the flat it is supposed to lie on. A ray
    # finds the true face; the defect is moved onto it.
    n_orphan = 0
    for r in records:
        ob = bpy.data.objects.get(r["defect_id"])
        if ob is None:
            continue
        nrm = Vector(r["surface_normal"])
        host, hit, face_n = _find_host(Vector(r["position_m"]), nrm)
        if host is None:
            n_orphan += 1
            r["host_object"] += " (UNVERIFIED)"
            r["surface_offset_mm"] = -1.0
            continue
        # Align to the face that is REALLY there, not the design normal.
        # A pier column is modelled as a 16-sided prism, so the radial
        # direction the site catalogue computed is up to 11 degrees off the
        # facet the defect lands on. A flat decal tilted 11 degrees on a
        # 0.7 m patch stands 70 mm proud at one edge, and a spall liner does
        # the same -- which renders as a scab stuck ON the column rather than
        # a cavity cut INTO it. The ray already knows the true face normal.
        if face_n.dot(nrm) < 0.0:
            face_n = -face_n
        if face_n.length > 0.5 and face_n.dot(nrm) > 0.30:
            q = nrm.rotation_difference(face_n)
            rot = q.to_matrix().to_4x4() @ ob.matrix_world.to_3x3().to_4x4()
            nrm = face_n
        else:
            rot = ob.matrix_world.to_3x3().to_4x4()

        eps = 0.006 if r["representation"] == "SHADER_DECAL" else 0.002
        newp = hit + nrm * eps
        old_mw = ob.matrix_world.copy()
        ob.matrix_world = Matrix.Translation(newp) @ rot
        rel = ob.matrix_world @ old_mw.inverted()
        for ch in bpy.data.objects:
            if ch.name.startswith(r["defect_id"] + "_BAR"):
                ch.matrix_world = rel @ ch.matrix_world
        r["position_m"] = [round(v, 4) for v in hit]
        r["surface_normal"] = [round(v, 4) for v in nrm]
        r["host_object"] = host.name
        r["surface_offset_mm"] = round(eps * 1000.0, 1)
        ob["avi_host_object"] = host.name

    bpy.context.view_layer.update()

    # ---- cut the cavities out of the real members ------------------------
    cutters = []
    n_nocut = 0
    for dname, cv, cf in pending_cuts:
        ob = bpy.data.objects.get(dname)
        rec = next((r for r in records if r["defect_id"] == dname), None)
        if ob is None or rec is None or rec["surface_offset_mm"] < 0:
            n_nocut += 1
            continue
        host = bpy.data.objects.get(rec["host_object"])
        if host is None:
            n_nocut += 1
            continue
        mtx = ob.matrix_world
        inv = host.matrix_world.inverted()
        local = [tuple(inv @ (mtx @ Vector(v))) for v in cv]
        cutters.append((host, local, cf, None))

    stats = carve(cutters, log) if cutters else {"cut": 0, "failed": 0}
    if n_orphan:
        log(f"  snap    : {n_orphan} defects had no host surface beneath them")
    if n_nocut:
        log(f"  carve   : {n_nocut} cavities skipped, no resolvable host")
    stats["orphans"] = n_orphan

    return records, counts, n_obj


def export_ground_truth(records, path_json, path_csv):
    """Write ground truth in both a structured and a tabular form."""
    import csv
    by_sev = {}
    by_type = {}
    for r in records:
        by_sev[r["severity"]] = by_sev.get(r["severity"], 0) + 1
        by_type[r["type"]] = by_type.get(r["type"], 0) + 1
    n_rep = sum(1 for r in records if r["repairable"])

    doc = {
        "project": "AVIAN Smart Infrastructure Inspection City",
        "phase": "ENVIRONMENT ONLY",
        "coordinate_system": {
            "units": "metres",
            "up_axis": "+Z",
            "origin": "ground level at the start of the bridge deck "
                      "centreline, chainage x = 0",
            "forward": "+X along the corridor",
            "left": "+Y",
        },
        "research_zone": {
            "x_start_m": P.RESEARCH_X0,
            "x_end_m": P.RESEARCH_X1,
            "length_m": P.RESEARCH_LENGTH,
            "sectors": P.SECTOR_NAMES,
        },
        "repair_envelope": P.REPAIR_ENVELOPE,
        "summary": {
            "total_defects": len(records),
            "by_type": by_type,
            "by_severity": by_sev,
            "repairable": n_rep,
            "non_repairable_escalation": len(records) - n_rep,
            "geometry_backed": sum(
                1 for r in records if r["representation"] == "GEOMETRY"),
            "shader_backed": sum(
                1 for r in records if r["representation"] == "SHADER_DECAL"),
        },
        "defects": records,
    }
    with open(path_json, "w") as f:
        json.dump(doc, f, indent=2)

    cols = ["defect_id", "type", "severity", "repairable", "reason",
            "representation", "host_surface", "host_object",
            "bridge_section", "inspection_sector", "occlusion",
            "area_m2", "width_mm", "depth_mm", "placement_rationale"]
    with open(path_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols + ["x_m", "y_m", "z_m"])
        for r in records:
            w.writerow([r.get(c, "") for c in cols] + r["position_m"])
    return doc["summary"]

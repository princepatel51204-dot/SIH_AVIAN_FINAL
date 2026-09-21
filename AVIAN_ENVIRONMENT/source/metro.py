"""REV-C Stage 2 -- the elevated metro viaduct.

A second real structure beside the road bridge, and a full inspection target
in its own right: its own defects, sectors, airspace and ground truth.

WHY A BOX GIRDER AND NOT I-GIRDERS
----------------------------------
This is the visual and the inspection difference from the road bridge. A
single-cell precast box girder is what Indian elevated metro actually uses,
and it gives a genuinely new problem: a hollow interior. Every REV-B
inspection surface is external. The box interior is a confined,
GNSS-dead space reachable only through access hatches, which is a flight
condition that does not exist anywhere else in the model.

GEOMETRY DECISIONS -- all assumptions, labelled per working rule 5
-----------------------------------------------------------------
Centreline y = +45.0 m. Measured, not assumed: BR_ structure reaches
y = +12.00 m, REV-B sector/deck/separation volumes stop at y = +24, and
ground clutter ends at y = +25.7, so y = +45 leaves 33 m to the road bridge
and 21 m to the nearest REV-B airspace.

Deck top z = 32.0 m, flat. The road deck runs a longitudinal profile; the
metro is modelled flat because a constant deck level is both realistic for
a short viaduct section and much easier to assert the ceiling against.

THE CEILING, z < 55 m, is the hard constraint and it is doubly motivated.
Two volumes share that floor:
    AVI_AIRSPACE_SAFE_TRANSIT   z[55, 95]   -- V10 tests this one
    AVI_TRANSIT_AIRSPACE_MAIN   z[55, 100]  -- REV-B transit corridor
Both span y = +45. Nothing on this structure -- deck, parapet, catenary
mast, station roof -- may reach 55 m. The tallest thing built here is the
station roof at z = 42.0, giving 13 m of margin, and V34 asserts it rather
than trusting this comment.

Over the navigation channel x in (2055, 2145) there is NO pier: one 90 m
main span, matching the road bridge's. Its deeper box drops the soffit to
z = 27.0, still 29 m above the water at z = -2.0, well clear of V07's 12 m.
"""
from __future__ import annotations
import math
import random

import bpy

import meshlib as ML
import params as P
import params_c as PC
import terrain as TR


# ---------------------------------------------------------------------------
# ASSUMPTIONS -- metro geometry. None of these come from a real drawing set;
# they are typical Indian elevated-metro practice and are labelled as such.
# ---------------------------------------------------------------------------
Y = 45.0                  # m, viaduct centreline
DECK_TOP_Z = 32.0         # m, top of deck slab (flat)
X0, X1 = 1550.0, 2650.0   # m, extent -- matches the Stage 4 export corridor
SPAN = 28.0               # m, typical precast span
MAIN_X0, MAIN_X1 = 2055.0, 2145.0    # the navigation channel: no pier inside

BOX_W = 8.60              # m, box girder outer width
BOX_D = 2.40              # m, structural depth, approach spans
BOX_D_MAIN = 5.00         # m, deeper over the 90 m main span
TOP_SLAB_T = 0.30         # m
BOT_SLAB_T = 0.28         # m
WEB_T = 0.35              # m
WEB_GAUGE = 5.20          # m, centre-to-centre of the two webs

PIER_D = 2.80             # m, single circular column
PIER_HEAD_L = 6.40        # m, flared pier head, transverse
PIER_HEAD_W = 2.60        # m
PIER_HEAD_H = 1.60        # m
FOOTING = (6.0, 6.0, 1.2)  # m
BEARING = (0.70, 0.70, 0.25)

PARAPET_H = 1.10          # m
PARAPET_T = 0.24          # m
TROUGH = (0.55, 0.45)     # m, cable trough section

SLAB_TRACK_T = 0.35       # m, ballastless track slab
RAIL_GAUGE = 1.435        # m, standard gauge
RAIL = (0.075, 0.16)      # m, rail head section (coarse -- it is a collision
                          # primitive, not a rail profile)
MAST_H = 6.00             # m, catenary mast above deck
MAST_D = 0.28             # m
MAST_EVERY = 2            # masts every N spans

STATION_X0, STATION_X1 = 2240.0, 2320.0   # m, inside the research zone
STATION_W = 22.0          # m, overall station box width
STATION_ROOF_Z = 42.0     # m -- 13 m below the 55 m ceiling
PLATFORM_T = 0.35

CEILING_Z = 55.0          # m, hard ceiling (SAFE_TRANSIT / TRANSIT floor)

# avi_kind values this module uses that are NOT in the collision exporter's
# STRUCTURAL_KINDS vocabulary. They classify via the MB_ prefix instead,
# which is exactly why MB_ must be in STRUCTURAL_PREFIXES -- without it
# these vanish from collision silently.
EXTRA_KINDS = {
    "cable_trough", "track_slab", "rail", "catenary_mast",
    "station_slab", "station_wall", "station_roof", "station_stair",
    "station_platform", "access_hatch",
}


def _box(name, size, centre, coll, mat, kind, extra=None):
    ob = ML.box(name, size, centre, coll, mat)
    props = {"avi_kind": kind, "avi_structure": "METRO"}
    if extra:
        props.update(extra)
    ML.set_custom(ob, props)
    return ob


def span_depth(x0, x1):
    """Structural depth for a span. Deeper over the main span."""
    return BOX_D_MAIN if (x1 - x0) > 40.0 else BOX_D


def pier_stations():
    """Pier x-positions, with a clear 90 m main span over the channel.

    Approach spans march in from both ends at SPAN pitch and stop at the
    channel edges, so no pier lands inside x in (2055, 2145).
    """
    xs = []
    x = X0
    while x < MAIN_X0 - 1e-6:
        xs.append(x)
        x += SPAN
    # snap the last approach pier exactly onto the channel edge
    if xs and abs(xs[-1] - MAIN_X0) > 1e-6:
        xs[-1] = MAIN_X0
    else:
        xs.append(MAIN_X0)
    x = MAIN_X1
    while x <= X1 + 1e-6:
        xs.append(x)
        x += SPAN
    if xs[-1] < X1:
        xs.append(X1)
    return sorted(set(round(v, 3) for v in xs))


def build_span(x0, x1, idx, coll, mats):
    """One precast box-girder span: top slab, bottom slab, two webs.

    Built as four boxes rather than a booleaned shell, which is what makes
    the interior a real void: the collision exporter turns each into its own
    primitive, so a UAV can fly between the webs instead of hitting a solid
    block the shape of the girder.
    """
    tag = f"{idx:03d}"
    L = x1 - x0
    cx = (x0 + x1) / 2.0
    d = span_depth(x0, x1)
    soffit = DECK_TOP_Z - d
    mat = mats["concrete_girder"]
    out = []

    out.append(_box(f"MB_DECK_TOP_{tag}", (L, BOX_W, TOP_SLAB_T),
                    (cx, Y, DECK_TOP_Z - TOP_SLAB_T / 2.0), coll, mat,
                    "deck_box", {"avi_span": idx, "avi_part": "top_slab"}))
    out.append(_box(f"MB_DECK_SOFFIT_{tag}",
                    (L, BOX_W - 2 * 0.9, BOT_SLAB_T),
                    (cx, Y, soffit + BOT_SLAB_T / 2.0), coll, mat,
                    "deck_box", {"avi_span": idx, "avi_part": "bottom_slab"}))
    web_h = d - TOP_SLAB_T - BOT_SLAB_T
    for s, nm in ((+1, "W1"), (-1, "W2")):
        out.append(_box(f"MB_WEB_{tag}_{nm}", (L, WEB_T, web_h),
                        (cx, Y + s * WEB_GAUGE / 2.0,
                         soffit + BOT_SLAB_T + web_h / 2.0), coll, mat,
                        "deck_box", {"avi_span": idx, "avi_part": "web",
                                     "avi_side": nm}))

    # parapets, cable trough, track slab and rails
    for s, nm in ((+1, "N"), (-1, "S")):
        out.append(_box(f"MB_PARAPET_{tag}_{nm}", (L, PARAPET_T, PARAPET_H),
                        (cx, Y + s * (BOX_W / 2.0 - PARAPET_T / 2.0),
                         DECK_TOP_Z + PARAPET_H / 2.0), coll,
                        mats["concrete_parapet"], "parapet",
                        {"avi_span": idx, "avi_side": nm}))
    out.append(_box(f"MB_TROUGH_{tag}", (L, TROUGH[0], TROUGH[1]),
                    (cx, Y + BOX_W / 2.0 - 0.95, DECK_TOP_Z + TROUGH[1] / 2.0),
                    coll, mats["concrete_low"], "cable_trough",
                    {"avi_span": idx}))
    out.append(_box(f"MB_TRACKSLAB_{tag}", (L, 4.2, SLAB_TRACK_T),
                    (cx, Y, DECK_TOP_Z + SLAB_TRACK_T / 2.0), coll,
                    mats["concrete_low"], "track_slab", {"avi_span": idx}))
    for s, nm in ((+1, "R1"), (-1, "R2")):
        out.append(_box(f"MB_RAIL_{tag}_{nm}", (L, RAIL[0], RAIL[1]),
                        (cx, Y + s * RAIL_GAUGE / 2.0,
                         DECK_TOP_Z + SLAB_TRACK_T + RAIL[1] / 2.0), coll,
                        mats["steel"], "rail",
                        {"avi_span": idx, "avi_side": nm}))

    # segmental joint at the start of every span
    out.append(_box(f"MB_JOINT_{tag}", (0.12, BOX_W, 0.45),
                    (x0, Y, DECK_TOP_Z - 0.45 / 2.0), coll,
                    mats["concrete_low"], "joint_gap", {"avi_span": idx}))

    # access hatch in the bottom slab -- the way into the box interior
    out.append(_box(f"MB_HATCH_{tag}", (0.9, 0.7, 0.08),
                    (cx, Y, soffit + 0.04), coll, mats["steel_dark"],
                    "access_hatch", {"avi_span": idx}))
    return out


def build_pier(x, idx, coll, mats):
    """Single circular column with a flared head -- not a twin-column bent.

    A shared bent with the road bridge was rejected in the brief because it
    would perturb BR_ piers, and every road-bridge defect is anchored to its
    host by ray-cast: moving a pier moves its defects and breaks V22.
    """
    tag = f"{idx:03d}"
    gz = TR.height(x, Y)
    out = []
    out.append(_box(f"MB_PIER_FOOTING_{tag}", FOOTING,
                    (x, Y, gz + FOOTING[2] / 2.0), coll,
                    mats["concrete_pier"], "pier_footing", {"avi_pier": idx}))

    d = span_depth(x, x + SPAN)
    head_bot = DECK_TOP_Z - d - BEARING[2] - PIER_HEAD_H
    col_top = head_bot
    col_h = col_top - (gz + FOOTING[2])
    ob = ML.cylinder(f"MB_PIER_COL_{tag}", PIER_D / 2.0, col_h,
                     (x, Y, gz + FOOTING[2] + col_h / 2.0), 16, coll,
                     mats["concrete_pier"])
    ML.set_custom(ob, {"avi_kind": "pier_column", "avi_structure": "METRO",
                       "avi_pier": idx, "avi_height_m": round(col_h, 2)})
    out.append(ob)

    head = ML.taper_box(f"MB_PIER_HEAD_{tag}", (PIER_D, PIER_D),
                        (PIER_HEAD_L, PIER_HEAD_W), PIER_HEAD_H,
                        (x, Y, head_bot + PIER_HEAD_H / 2.0), coll,
                        mats["concrete_pier"])
    ML.set_custom(head, {"avi_kind": "pier_cap", "avi_structure": "METRO",
                         "avi_pier": idx})
    out.append(head)

    for s, nm in ((+1, "B1"), (-1, "B2")):
        out.append(_box(f"MB_BEARING_{tag}_{nm}", BEARING,
                        (x, Y + s * WEB_GAUGE / 2.0,
                         head_bot + PIER_HEAD_H + BEARING[2] / 2.0), coll,
                        mats["bearing"], "bearing",
                        {"avi_pier": idx, "avi_side": nm}))
    return out


def build_station(coll, mats, log=print):
    """One elevated station inside the research zone.

    Also the largest GNSS shadow in the model -- a roofed box 80 m long
    sitting over the track, which is a flight condition REV-B has nowhere.
    Roof at z = 42.0, thirteen metres under the 55 m ceiling.
    """
    out = []
    L = STATION_X1 - STATION_X0
    cx = (STATION_X0 + STATION_X1) / 2.0
    conc = mats["concrete_low"]

    # platforms either side of the track, at deck level
    for s, nm in ((+1, "N"), (-1, "S")):
        out.append(_box(f"MB_STN_PLATFORM_{nm}", (L, 4.0, PLATFORM_T),
                        (cx, Y + s * 4.6, DECK_TOP_Z + SLAB_TRACK_T
                         + PLATFORM_T / 2.0), coll, conc,
                        "station_platform", {"avi_side": nm}))
    # concourse slab above the platforms
    out.append(_box("MB_STN_CONCOURSE", (L, STATION_W, 0.35),
                    (cx, Y, DECK_TOP_Z + 6.5), coll, conc, "station_slab"))
    # side walls
    for s, nm in ((+1, "N"), (-1, "S")):
        out.append(_box(f"MB_STN_WALL_{nm}", (L, 0.30, 9.0),
                        (cx, Y + s * STATION_W / 2.0,
                         DECK_TOP_Z + 4.8), coll, conc, "station_wall",
                        {"avi_side": nm}))
    # roof
    out.append(_box("MB_STN_ROOF", (L + 4.0, STATION_W + 2.0, 0.40),
                    (cx, Y, STATION_ROOF_Z - 0.20), coll,
                    mats["steel_dark"], "station_roof"))
    # end walls
    for s, nm in ((+1, "E"), (-1, "W")):
        out.append(_box(f"MB_STN_END_{nm}", (0.35, STATION_W, 9.0),
                        (cx + s * L / 2.0, Y, DECK_TOP_Z + 4.8), coll,
                        conc, "station_wall", {"avi_end": nm}))
    # stair core down to ground on the far side from the road bridge
    gz = TR.height(cx, Y + STATION_W / 2.0 + 4.0)
    h = DECK_TOP_Z + 6.5 - gz
    out.append(_box("MB_STN_STAIR", (8.0, 6.0, h),
                    (cx, Y + STATION_W / 2.0 + 4.0, gz + h / 2.0), coll,
                    conc, "station_stair"))
    log(f"  station : {len(out)} members, x {STATION_X0:.0f}-{STATION_X1:.0f}, "
        f"roof z={STATION_ROOF_Z:.1f} ({CEILING_Z - STATION_ROOF_Z:.1f} m "
        f"under the ceiling)")
    return out


def build(colls, mats, log=print):
    """The whole viaduct. Returns stats."""
    rnd = random.Random(P.SEED + 91)
    coll = colls.get("AVIAN_METRO") or colls["ROOT"]

    xs = pier_stations()
    objs = []

    # spans between consecutive piers
    for i in range(len(xs) - 1):
        objs += build_span(xs[i], xs[i + 1], i + 1, coll, mats)
    for i, x in enumerate(xs):
        objs += build_pier(x, i + 1, coll, mats)

    # catenary masts, every MAST_EVERY spans, alternating sides
    n_mast = 0
    for i in range(0, len(xs) - 1, MAST_EVERY):
        x = xs[i] + (xs[i + 1] - xs[i]) / 2.0
        if STATION_X0 - 6 < x < STATION_X1 + 6:
            continue                      # no masts inside the station box
        s = 1 if (i // MAST_EVERY) % 2 == 0 else -1
        n_mast += 1
        objs.append(_box(f"MB_MAST_{n_mast:03d}", (MAST_D, MAST_D, MAST_H),
                         (x, Y + s * 3.1,
                          DECK_TOP_Z + SLAB_TRACK_T + MAST_H / 2.0), coll,
                         mats["steel_dark"], "catenary_mast",
                         {"avi_side": "N" if s > 0 else "S"}))

    objs += build_station(coll, mats, log)

    # World-space extents from the vertices, NOT from
    # matrix_world.translation + dimensions/2. meshlib.box() bakes the
    # centre into the vertex coordinates and leaves the object origin at
    # the world origin, so translation is (0,0,0) for every member here --
    # the convenient-looking version reports the envelope of a viaduct
    # sitting at y=0, which is wrong by 45 m and wrong in a way that looks
    # plausible.
    top = max(max((ob.matrix_world @ v.co).z for v in ob.data.vertices)
              for ob in objs if ob.type == "MESH" and ob.data.vertices)
    main = [(xs[i], xs[i + 1]) for i in range(len(xs) - 1)
            if xs[i + 1] - xs[i] > 40.0]
    soffit_main = DECK_TOP_Z - BOX_D_MAIN

    log(f"  metro   : {len(xs)} piers, {len(xs)-1} spans, {n_mast} masts, "
        f"{len(objs)} objects at y={Y:.1f}")
    log(f"  metro   : deck z={DECK_TOP_Z:.1f}, max z={top:.2f} "
        f"(ceiling {CEILING_Z:.1f}), main span "
        f"{main[0][0]:.0f}-{main[0][1]:.0f} soffit z={soffit_main:.1f}, "
        f"air draft {soffit_main - P.RIVER_WATER_Z:.2f} m")
    return {"piers": len(xs), "spans": len(xs) - 1, "masts": n_mast,
            "objects": len(objs), "max_z": round(top, 3),
            "deck_top_z": DECK_TOP_Z, "centreline_y": Y,
            "main_span": list(main[0]) if main else None,
            "air_draft_m": round(soffit_main - P.RIVER_WATER_Z, 2)}

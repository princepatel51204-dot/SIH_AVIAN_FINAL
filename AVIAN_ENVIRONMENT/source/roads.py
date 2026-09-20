"""Road network: bridge approaches, ground-level highway, urban grid, ramps.

The bridge carries its own wearing course (built in bridge.py). What this
module adds is everything that makes the corridor read as a continuous
transport route rather than a viaduct floating in a field: the embankments it
lands on, the highway it becomes, the streets it crosses, and the ramps that
connect them.
"""
from __future__ import annotations
import math
import random
import bpy

import params as P
import meshlib as ML
import terrain as TR

LANE_W = 3.5
MARK_W = 0.14


# ---------------------------------------------------------------------------
def _road_strip(name, x0, x1, y_c, width, z_fn, coll, mat, steps=None):
    steps = steps or max(2, int(abs(x1 - x0) / 25.0))
    L, R = [], []
    for i in range(steps + 1):
        x = x0 + (x1 - x0) * i / steps
        z = z_fn(x)
        L.append((x, y_c + width / 2, z))
        R.append((x, y_c - width / 2, z))
    return ML.ribbon(name, L, R, coll, mat)


def _lane_marks(name, x0, x1, y, z_fn, coll, mat, dashed=True,
                dash=4.0, gap=6.0):
    """Dashed or solid lane line as a thin ribbon, lifted clear of the road."""
    made = []
    x = x0
    k = 0
    step = (dash + gap) if dashed else (x1 - x0)
    while x < x1:
        seg = min(dash if dashed else (x1 - x0), x1 - x)
        if seg <= 0.2:
            break
        k += 1
        L = [(x, y + MARK_W / 2, z_fn(x) + 0.012),
             (x + seg, y + MARK_W / 2, z_fn(x + seg) + 0.012)]
        R = [(x, y - MARK_W / 2, z_fn(x) + 0.012),
             (x + seg, y - MARK_W / 2, z_fn(x + seg) + 0.012)]
        made.append(ML.ribbon(f"{name}_{k:03d}", L, R, coll, mat))
        x += step
    return made


# ---------------------------------------------------------------------------
def build(colls, mats, log=print):
    rnd = random.Random(P.SEED + 11)
    C = colls["ROADS"]
    made = 0

    # =====================================================================
    # 1. BRIDGE DECK MARKINGS
    # =====================================================================
    # Only in and around the research zone -- lane paint 3 km away is
    # thousands of objects a sensor will never resolve.
    mx0 = P.RESEARCH_X0 - P.LOD_MED_MARGIN
    mx1 = P.RESEARCH_X1 + P.LOD_MED_MARGIN

    def deck_z(x):
        return P.deck_top_z(x)

    for s in (1, -1):
        cc = s * (P.MEDIAN_WIDTH / 2 + P.CARRIAGEWAY_WIDTH / 2)
        # edge lines, solid
        for e in (-1, 1):
            y = cc + e * (P.CARRIAGEWAY_WIDTH / 2 - 0.35)
            m = _lane_marks(f"RD_DECK_EDGE_{'L' if s>0 else 'R'}{e}",
                            mx0, mx1, y, deck_z, C, mats["paint_white"],
                            dashed=False)
            made += len(m)
        # two dashed lane lines per carriageway
        for k in (-1, 1):
            y = cc + k * LANE_W / 2 * 1.0
            m = _lane_marks(f"RD_DECK_LANE_{'L' if s>0 else 'R'}{k}",
                            mx0, mx1, y, deck_z, C, mats["paint_white"])
            made += len(m)

    # =====================================================================
    # 2. APPROACH EMBANKMENTS
    # =====================================================================
    # The bridge has to land on something. Each end gets a tapered earth
    # embankment carrying the road down to grade.
    for end, x_b, x_g, sgn in (("SOUTH", 0.0, -420.0, -1),
                               ("NORTH", P.BRIDGE_LENGTH,
                                P.BRIDGE_LENGTH + 420.0, 1)):
        zb = P.deck_top_z(x_b)
        n = 16
        L, R = [], []
        for i in range(n + 1):
            t = i / n
            x = x_b + (x_g - x_b) * t
            z = zb * (1 - t)
            w = P.DECK_WIDTH + t * 26.0        # embankment widens at the toe
            L.append((x, w / 2, z - 0.1))
            R.append((x, -w / 2, z - 0.1))
        emb = ML.ribbon(f"RD_EMBANKMENT_{end}", L, R, C, mats["ground"])
        ML.set_custom(emb, {"avi_kind": "embankment", "avi_end": end})
        made += 1
        # side slopes
        for s in (1, -1):
            SL, SR = [], []
            for i in range(n + 1):
                t = i / n
                x = x_b + (x_g - x_b) * t
                z = zb * (1 - t)
                w = P.DECK_WIDTH + t * 26.0
                SL.append((x, s * w / 2, z - 0.1))
                SR.append((x, s * (w / 2 + z * 2.2 + 2.0), 0.0))
            ML.ribbon(f"RD_EMB_SLOPE_{end}_{'L' if s>0 else 'R'}",
                      SL, SR, C, mats["ground"])
            made += 1
        # carriageway on the embankment
        rz = (lambda xb=x_b, zb=zb, xg=x_g:
              (lambda x: zb * max(0.0, 1.0 - (x - xb) / (xg - xb))))()
        _road_strip(f"RD_APPROACH_{end}", min(x_b, x_g), max(x_b, x_g),
                    0.0, P.DECK_WIDTH - 2.0, rz, C, mats["asphalt"], 18)
        made += 1

    # =====================================================================
    # 3. GROUND-LEVEL HIGHWAY UNDER THE VIADUCT
    # =====================================================================
    # An urban viaduct is normally built over an existing surface road. That
    # road is also what gives the under-deck inspection sectors a floor with
    # traffic on it, rather than empty ground.
    def flat(x):
        return TR.height(x, 19.0) + 0.06

    for s in (1, -1):
        yc = s * 19.0
        _road_strip(f"RD_SURFACE_HWY_{'W' if s>0 else 'E'}",
                    350.0, 4150.0, yc, 11.0, flat, C, mats["asphalt"], 90)
        made += 1
    med = ML.box("RD_SURFACE_MEDIAN", (3800.0, 12.0, 0.14),
                 (2250.0, 0.0, 0.05), C, mats["ground"])
    made += 1
    for s in (1, -1):
        m = _lane_marks(f"RD_HWY_EDGE_{'W' if s>0 else 'E'}",
                        max(350.0, mx0), min(4150.0, mx1),
                        s * 24.0, flat, C, mats["paint_white"], dashed=False)
        made += len(m)

    # =====================================================================
    # 4. URBAN CROSS STREETS
    # =====================================================================
    n_cross = 0
    x = 480.0
    while x < 4100.0:
        if abs(x - P.RIVER_CENTRE_X) < P.RIVER_WIDTH / 2 + 130:
            x += P.CITY_BLOCK * 2
            continue
        n_cross += 1
        Lp, Rp = [], []
        steps = 14
        for i in range(steps + 1):
            yy = -P.CITY_BAND_Y + 2 * P.CITY_BAND_Y * i / steps
            zz = TR.height(x, yy) + 0.05
            Lp.append((x + P.CITY_ROAD_W / 2, yy, zz))
            Rp.append((x - P.CITY_ROAD_W / 2, yy, zz))
        ML.ribbon(f"RD_CROSS_{n_cross:02d}", Lp, Rp, C, mats["asphalt"])
        made += 1
        x += P.CITY_BLOCK * rnd.choice([2, 2, 3])

    # longitudinal city streets
    n_long = 0
    for k in range(1, 9):
        for s in (1, -1):
            y = s * (P.CITY_SETBACK_Y + k * P.CITY_BLOCK)
            if abs(y) > P.CITY_BAND_Y:
                continue
            # Break the road at the river. Only the main corridor crosses
            # it; a surface street running over open water with no bridge is
            # exactly the kind of error the validation pass looks for.
            bank = P.RIVER_WIDTH / 2 + 55.0
            runs = [(-400.0, P.RIVER_CENTRE_X - bank),
                    (P.RIVER_CENTRE_X + bank, P.BRIDGE_LENGTH + 400.0)]
            for ri, (rx0, rx1) in enumerate(runs):
                if rx1 - rx0 < 50.0:
                    continue
                n_long += 1
                pts = []
                steps = max(2, int((rx1 - rx0) / 60.0))
                Lp, Rp = [], []
                for i in range(steps + 1):
                    xx = rx0 + (rx1 - rx0) * i / steps
                    zz = TR.height(xx, y) + 0.05
                    Lp.append((xx, y + P.CITY_ROAD_W / 2, zz))
                    Rp.append((xx, y - P.CITY_ROAD_W / 2, zz))
                ML.ribbon(f"RD_LONG_{n_long:02d}", Lp, Rp, C,
                          mats["asphalt"])
                made += 1

    # =====================================================================
    # 5. RAMPS
    # =====================================================================
    # Two on/off ramps peeling off the viaduct, which is what makes it an
    # interchange rather than a flyover. They also create the only place
    # where the deck edge is not a straight line -- useful later for
    # navigation that assumes a constant-width corridor.
    n_ramp = 0
    for x_r, s in ((900.0, 1), (3500.0, -1)):
        n_ramp += 1
        n = 22
        L, R = [], []
        for i in range(n + 1):
            t = i / n
            x = x_r + t * 300.0
            z = P.deck_top_z(x_r) * (1 - t) + 0.4 * t
            off = s * (P.DECK_WIDTH / 2 + 2.0 + t * t * 46.0)
            L.append((x, off + s * 4.5, z))
            R.append((x, off - s * 4.5, z))
        rp = ML.ribbon(f"RD_RAMP_{n_ramp}", L, R, C, mats["asphalt"])
        ML.set_custom(rp, {"avi_kind": "ramp"})
        made += 1
        # ramp piers
        for i in range(2, n, 4):
            t = i / n
            x = x_r + t * 300.0
            z = P.deck_top_z(x_r) * (1 - t) + 0.4 * t
            off = s * (P.DECK_WIDTH / 2 + 2.0 + t * t * 46.0)
            if z < 1.5:
                continue
            ML.cylinder(f"RD_RAMP_{n_ramp}_PIER_{i}", 0.75, z,
                        (x, off, z / 2), 10, C, mats["concrete_pier"])
            made += 1

    # =====================================================================
    # 6. STREET FURNITURE
    # =====================================================================
    # Lighting columns on the bridge, instanced from one prototype.
    pole = ML.cylinder("_LIGHTPOLE_PROTO", 0.11, 9.0, (0, 0, 4.5), 8,
                       C, mats["steel_dark"])
    arm = ML.box("_LIGHTPOLE_ARM", (1.6, 0.10, 0.10), (0.8, 0, 9.0),
                 C, mats["steel_dark"])
    head = ML.box("_LIGHTPOLE_HEAD", (0.62, 0.26, 0.14), (1.55, 0, 8.95),
                  C, mats["steel_dark"])
    for o in (pole, arm, head):
        o.hide_render = True
        o.hide_viewport = True

    n_lamp = 0
    x = 60.0
    while x < P.BRIDGE_LENGTH:
        s = 1 if n_lamp % 2 == 0 else -1
        y = s * (P.DECK_WIDTH / 2 - P.PARAPET_THICK - 0.1)
        z = P.deck_top_z(x)
        for proto, nm in ((pole, "POLE"), (arm, "ARM"), (head, "HEAD")):
            ML.link_dup(proto, f"RD_LAMP_{n_lamp+1:03d}_{nm}", (x, y, z),
                        (0, 0, 0 if s > 0 else math.pi), (1, 1, 1), C)
        n_lamp += 1
        made += 3
        x += P.CITY_STREETLIGHT_SPACING

    # ---- signage, at the approaches ---------------------------------------
    n_sign = 0
    for x_s, txt in ((-120.0, "S"), (P.BRIDGE_LENGTH + 120.0, "N")):
        for s in (1, -1):
            n_sign += 1
            ML.cylinder(f"RD_SIGN_POST_{n_sign}_A", 0.09, 6.0,
                        (x_s, s * 16.0, 3.0), 6, C, mats["steel_dark"])
            ML.box(f"RD_SIGN_PANEL_{n_sign}", (0.12, 5.2, 2.2),
                   (x_s, s * 16.0, 5.4), C, mats["sign"])
            made += 2

    # ---- crash barriers along the surface highway -------------------------
    for s in (1, -1):
        b = ML.box(f"RD_BARRIER_{'W' if s>0 else 'E'}",
                   (3800.0, 0.35, 0.80), (2250.0, s * 25.5, 0.40),
                   C, mats["barrier"])
        ML.set_custom(b, {"avi_kind": "barrier"})
        made += 1

    log(f"  roads   : {made} objects, {n_cross} cross streets, "
        f"{n_long} longitudinal, {n_ramp} ramps, {n_lamp} lighting columns")
    return {"objects": made, "ramps": n_ramp, "lamps": n_lamp}

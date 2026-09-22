"""SIH_AVIAN_FINAL -- the twelve named pitch cameras (SPEC.md S6, CAM_10-12
added for the detection pass, SPEC S9).

Reuses cameras_b.py's `_cam` helper unchanged (it takes explicit eye/target
coordinates and has no REV-C-specific dependency), with this scene's own
geometry. Metro constants (Y=28, DECK_TOP_Z=19.0, ...) are the SAME values
build_final.py monkeypatches onto metro.py -- duplicated here as plain
numbers because this module runs in measure_final.py's own process, which
does not re-run metro.build() and so has no reason to re-import/monkeypatch
metro.py at all.
"""
from __future__ import annotations

import bpy

import cameras_b as CB
import params_final as PF

METRO_Y = 28.0
METRO_DECK_TOP_Z = 19.0
METRO_SOFFIT_APPROACH_Z = METRO_DECK_TOP_Z - 2.2   # 16.8, matches SPEC.md


def build(coll, hero_pos=None, loose_bolt=None, log=print):
    """Build all 12 cameras. `hero_pos` is the hero defect's actual
    post-ray-snap world position (x,y,z), read from the ground truth --
    more accurate than recomputing the design position by hand.
    `loose_bolt` is (position, normal) for a CERTIFIABLE-band BOLT_LOOSE
    defect, for CAM_12's money shot."""
    cams = []

    # Off-centre in X on purpose: a dead-centre eye reads as a front
    # elevation, not a three-quarter view. Offsetting toward one end shows
    # the corridor receding in perspective as well as across it.
    cams.append(CB._cam(
        "CAM_01_OVERVIEW", (330.0, -190.0, 150.0), (110.0, 10.0, 8.0),
        18.0, "whole corridor, both bridges, three-quarter from above",
        coll))

    cams.append(CB._cam(
        "CAM_02_RIVER", (160.0, 6.0, -1.0), (195.0, 0.0, 13.0),
        24.0, "main span from the water, looking up", coll))

    cams.append(CB._cam(
        "CAM_03_UNDERDECK", (30.0, 0.0, 10.0), (300.0, 0.0, 11.5),
        28.0, "under the soffit, girder bays receding", coll))

    gap_y = (7.0 + (METRO_Y - 8.6 / 2.0)) / 2.0        # mid-gap in Y
    gap_z = (14.3 + METRO_SOFFIT_APPROACH_Z) / 2.0     # between the two decks
    cams.append(CB._cam(
        "CAM_04_INTER_STRUCTURE", (150.0, gap_y, gap_z),
        (280.0, gap_y, gap_z), 24.0,
        "inside the 16.5 m inter-structure corridor, both structures framed",
        coll))

    pier_y = -PF.PIER_COL_SPACING / 2.0
    cams.append(CB._cam(
        # This pier's exposed base sits at grade (the banks reach grade
        # exactly at the piers flanking the main span, by design -- see
        # params_final.py), so the waterline stain band (keyed to world Z
        # near the river's water level) is mostly BELOW visible ground
        # here; aimed low, at the band's top edge, rather than mid-column.
        "CAM_05_PIER", (135.0, pier_y - 4.0, 0.6), (135.0, pier_y, -0.3),
        35.0, "river pier from 4 m, waterline staining visible", coll))

    if hero_pos is None:
        hero_pos = (135.0, pier_y - 0.85, 6.0)
    hx, hy, hz = hero_pos
    # The cavity itself is ~1.3 m across (severity-4 spall, area 1.4 m2).
    # A tight, dead-on close-up frames only the cavity FLOOR -- no rim, no
    # wall, no shadow, and it reads as flat rusty concrete rather than a
    # hole. Pulled back to ~3 m and offset off-axis so the rim and its
    # raked-light shadow are actually in frame, which is the whole point of
    # a "hero" shot.
    cams.append(CB._cam(
        "CAM_06_HERO_DEFECT", (hx - 1.0, hy - 2.8, hz + 1.1),
        (hx, hy, hz), 35.0, "hero defect close-up, raked light", coll))

    cams.append(CB._cam(
        "CAM_07_METRO", (180.0, 62.0, 30.0), (180.0, METRO_Y, 20.0),
        24.0, "the viaduct and the parked train", coll))

    cams.append(CB._cam(
        "CAM_08_DECK", (20.0, 3.5, 16.0), (340.0, -1.75, 14.3),
        24.0, "along the carriageway with traffic", coll))

    import base_final as BF
    y_s, y_r = BF.BASE_Y, BF.BASE_Y + BF.PAD_GAP_Y
    base_mid = (BF.BASE_X, (y_s + y_r) / 2.0)
    cams.append(CB._cam(
        "CAM_09_BASE", (BF.BASE_X - 14.0, base_mid[1] - 10.0, 9.0),
        (base_mid[0], base_mid[1], 0.0), 28.0,
        "the drone base: both landing pads, cabin and mast", coll))

    # ---- detection pass: the steel truss ---------------------------------
    cams.append(CB._cam(
        "CAM_10_TRUSS", (140.0, -55.0, 32.0), (180.0, 0.0, 16.5),
        20.0, "the whole 90 m Warren truss main span, three-quarter from "
        "the riverbank", coll))

    gusset_ob = None
    for name in (f"ST_GUSSET_S_TOP_{i:02d}" for i in (4, 3, 5, 2, 6)):
        gusset_ob = bpy.data.objects.get(name)
        if gusset_ob is not None:
            break
    if gusset_ob is not None:
        verts = [gusset_ob.matrix_world @ v.co
                for v in gusset_ob.data.vertices]
        gx = sum(v.x for v in verts) / len(verts)
        gy = sum(v.y for v in verts) / len(verts)
        gz = sum(v.z for v in verts) / len(verts)
    else:
        gx, gy, gz = 180.0, -7.0, 21.0
    cams.append(CB._cam(
        "CAM_11_GUSSET", (gx - 0.3, gy - 2.6, gz + 0.4), (gx, gy, gz),
        50.0, "a top-chord gusset plate close up, its full bolt pattern "
        "and match marks legible", coll))

    if loose_bolt is not None:
        (lx, ly, lz), (nx, ny, nz) = loose_bolt
    else:
        lx, ly, lz = gx, gy, gz
        nx, ny, nz = 0.0, 1.0, 0.0
    # 0.6 m standoff, near-normal to the bolted face -- the gate's explicit
    # test is whether the broken match-mark line reads clearly in THIS
    # frame; everything else in the environment exists to make this shot
    # possible, so it gets the tightest, most direct framing of any camera
    # here.
    eye = (lx + nx * 0.6, ly + ny * 0.6, lz + nz * 0.6)
    cams.append(CB._cam(
        "CAM_12_LOOSE_BOLT", eye, (lx, ly, lz), 85.0,
        "money shot: a single loose bolt's broken torque match-mark line, "
        "0.6 m standoff, near-normal", coll))

    log(f"  cameras : {len(cams)} named views")
    return cams

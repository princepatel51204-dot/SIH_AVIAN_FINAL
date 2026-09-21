"""SIH_AVIAN_FINAL -- realism pass: a real 3-car EMU instead of a box.

Local frame per car: +X = direction of travel, origin at rail top, car
centred in X. Built as one assembly of small objects placed with
`ob.location` (not baked world coordinates), so the whole train can be
positioned as a unit and the two end cars can each get their raked nose
facing outward.
"""
from __future__ import annotations
import math

import meshlib as ML

CAR_L = 22.0
CAR_W = 2.9
CAR_H = 3.6
GAP = 0.5
NOSE_L = 1.8


def _roof_profile(w, h, taper=0.06):
    """Cross-section for the car body prism: rounded roof, a slight lower
    taper (narrower at the floor than at waist height) -- 'a flat-sided
    box never reads right'. CCW in the (Y,Z) plane for prism(axis='X')."""
    hw, hw2 = w / 2.0, w / 2.0 * (1.0 - taper)
    z0, z1, z2 = 0.0, h * 0.72, h
    return [
        (hw2, z0), (hw, z1 * 0.55), (hw, z1),
        (hw * 0.78, h * 0.94), (hw * 0.35, z2), (-hw * 0.35, z2),
        (-hw * 0.78, h * 0.94), (-hw, z1), (-hw, z1 * 0.55), (-hw2, z0),
    ]


def _x_taper(name, w0, h0, z0_0, w1, h1, z0_1, x0, x1, coll, mat):
    """A wedge: a WxH rectangle at x0 tapering to a (usually smaller) WxH
    rectangle at x1 -- the driver-cab nose. z0_* is the rectangle's own
    floor height, so the nose can rise off the rail top toward the cab."""
    hw0, hw1 = w0 / 2.0, w1 / 2.0
    v = [
        (x0, -hw0, z0_0), (x0, hw0, z0_0), (x0, hw0, z0_0 + h0), (x0, -hw0, z0_0 + h0),
        (x1, -hw1, z0_1), (x1, hw1, z0_1), (x1, hw1, z0_1 + h1), (x1, -hw1, z0_1 + h1),
    ]
    f = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]
    return ML.mesh_obj(name, v, f, coll, mat)


def _door(tag, n, x_centre_list, y_side, z0, h, coll, mats):
    out = []
    for i, xc in enumerate(x_centre_list):
        out.append(ML.mesh_obj(
            f"MB_TRAIN_{tag}_DOOR_{('N' if y_side > 0 else 'S')}{i+1}",
            [(xc - 0.55, y_side, z0), (xc + 0.55, y_side, z0),
             (xc + 0.55, y_side, z0 + h), (xc - 0.55, y_side, z0 + h)],
            [(0, 1, 2, 3)], coll, mats["train_bogie"]))
    return out


def _bogie(tag, end, x, coll, mats):
    """`end` is a label ('F'/'R', front/rear WITHIN this car) -- NOT derived
    from the sign of the absolute x position, which is always positive
    anywhere in this 0..360 corridor and previously made both of a car's
    bogies collide on the same name."""
    out = []
    frame = ML.box(f"MB_TRAIN_{tag}_BOGIE_{end}",
                   (2.4, 2.2, 0.55), (x, 0.0, 0.55), coll, mats["train_bogie"])
    out.append(frame)
    for wi, (wx, s) in enumerate(((0.85, 1), (0.85, -1), (-0.85, 1), (-0.85, -1))):
        out.append(ML.cylinder(
            f"MB_TRAIN_{tag}_WHEEL_{end}_{wi+1}",
            0.42, 0.15, (x + wx, s * 1.15, 0.42), 12, coll,
            mats["train_bogie"], axis="Y"))
    return out


def _car(idx, cx, coll, mats, nose_front, nose_rear):
    """One car centred at world x=cx. nose_front/rear: True where that end
    tapers to a driver's cab (the outer ends of cars 1 and 3)."""
    tag = f"{idx:03d}"
    out = []
    x0, x1 = cx - CAR_L / 2.0, cx + CAR_L / 2.0
    body_x0 = x0 + (NOSE_L if nose_front else 0.0)
    body_x1 = x1 - (NOSE_L if nose_rear else 0.0)
    body_l = body_x1 - body_x0

    profile = _roof_profile(CAR_W, CAR_H)
    body = ML.prism(f"MB_TRAIN_{tag}_BODY", profile, body_l, "X",
                    ((body_x0 + body_x1) / 2.0, 0.0, 0.0), coll,
                    mats["train_body"])
    out.append(body)

    # window band + stripe, the length of the body section
    win_h = 1.1
    win_z0 = CAR_H * 0.42
    for s, nm in ((1, "N"), (-1, "S")):
        out.append(ML.mesh_obj(
            f"MB_TRAIN_{tag}_WINDOW_{nm}",
            [(body_x0 + 0.6, s * (CAR_W / 2.0 + 0.01), win_z0),
             (body_x1 - 0.6, s * (CAR_W / 2.0 + 0.01), win_z0),
             (body_x1 - 0.6, s * (CAR_W / 2.0 + 0.01), win_z0 + win_h),
             (body_x0 + 0.6, s * (CAR_W / 2.0 + 0.01), win_z0 + win_h)],
            [(0, 1, 2, 3)], coll, mats["glass"]))
        out.append(ML.box(
            f"MB_TRAIN_{tag}_STRIPE_{nm}",
            (body_l - 1.2, 0.03, 0.35),
            ((body_x0 + body_x1) / 2.0, s * (CAR_W / 2.0 + 0.02),
             win_z0 - 0.25), coll, mats["train_stripe"]))

    # 4 door pairs per side, recessed panel lines (flush dark strip, not a
    # true boolean cut -- cheap and reads correctly at any camera distance
    # this scene uses)
    door_xs = [body_x0 + body_l * f for f in (0.14, 0.38, 0.62, 0.86)]
    for s in (1, -1):
        out += _door(tag, 4, door_xs, s * (CAR_W / 2.0 + 0.015), 0.15, 2.0,
                    coll, mats)

    # driver's cab nose(s)
    if nose_front:
        out.append(_x_taper(f"MB_TRAIN_{tag}_NOSE_F", CAR_W, CAR_H, 0.0,
                            CAR_W * 0.35, CAR_H * 0.55, 0.05,
                            body_x0, x0, coll, mats["train_body"]))
        out.append(ML.mesh_obj(
            f"MB_TRAIN_{tag}_WINDSCREEN_F",
            [(body_x0, -CAR_W * 0.30, CAR_H * 0.55),
             (body_x0, CAR_W * 0.30, CAR_H * 0.55),
             (x0 + 0.3, CAR_W * 0.16, CAR_H * 0.85),
             (x0 + 0.3, -CAR_W * 0.16, CAR_H * 0.85)],
            [(0, 1, 2, 3)], coll, mats["glass"]))
    if nose_rear:
        out.append(_x_taper(f"MB_TRAIN_{tag}_NOSE_R", CAR_W, CAR_H, 0.0,
                            CAR_W * 0.35, CAR_H * 0.55, 0.05,
                            body_x1, x1, coll, mats["train_body"]))
        out.append(ML.mesh_obj(
            f"MB_TRAIN_{tag}_WINDSCREEN_R",
            [(body_x1, CAR_W * 0.30, CAR_H * 0.55),
             (body_x1, -CAR_W * 0.30, CAR_H * 0.55),
             (x1 - 0.3, -CAR_W * 0.16, CAR_H * 0.85),
             (x1 - 0.3, CAR_W * 0.16, CAR_H * 0.85)],
            [(0, 1, 2, 3)], coll, mats["glass"]))

    # bogies -- 2 per car, near each end
    out += _bogie(tag, "F", cx - CAR_L * 0.28, coll, mats)
    out += _bogie(tag, "R", cx + CAR_L * 0.28, coll, mats)

    # roof equipment: AC units on every car
    for fx in (0.30, 0.70):
        out.append(ML.box(f"MB_TRAIN_{tag}_ACUNIT_{fx:.1f}".replace(".", ""),
                          (2.0, 1.6, 0.42),
                          (body_x0 + body_l * fx, 0.0, CAR_H + 0.21),
                          coll, mats["train_roof"]))
    return out


def _pantograph(cx, coll, mats):
    """A simplified diamond pantograph on the middle car's roof."""
    out = []
    base_z = CAR_H + 0.05
    top_z = base_z + 1.3
    for s in (1, -1):
        out.append(ML.mesh_obj(
            f"MB_TRAIN_PANTO_ARM_{'A' if s > 0 else 'B'}",
            [(cx + s * 1.1, -0.05, base_z), (cx + s * 1.1, 0.05, base_z),
             (cx, 0.05, top_z), (cx, -0.05, top_z)],
            [(0, 1, 2, 3)], coll, mats["train_bogie"]))
    out.append(ML.box("MB_TRAIN_PANTO_SHOE", (1.4, 0.10, 0.06),
                      (cx, 0.0, top_z), coll, mats["train_bogie"]))
    out.append(ML.box("MB_TRAIN_PANTO_BASE", (1.6, 1.0, 0.10),
                      (cx, 0.0, base_z - 0.05), coll, mats["train_roof"]))
    return out


def build(colls, mats, centre_x, centre_y, rail_top_z, log=print):
    """3-car EMU centred on `centre_x`, on the track centreline `centre_y`,
    sitting on `rail_top_z`.

    Every part is built in `_car()`/`_pantograph()`/`_bogie()` with Y taken
    literally as 0 = track centreline (baked into the vertices, per
    meshlib's box/prism/cylinder convention) -- `centre_y` is applied here,
    once, to every object. Forgetting this shift is not hypothetical: the
    first version of this function only shifted Z, and the whole train sat
    at y=0 -- on top of the ROAD bridge, not the metro deck at y=28 -- which
    is exactly the kind of error VF15 (inter-structure corridor clear of
    structure) exists to catch."""
    coll = colls["VEHICLES"]
    total = 3 * CAR_L + 2 * GAP
    x0 = centre_x - total / 2.0
    centres = [x0 + CAR_L / 2.0 + i * (CAR_L + GAP) for i in range(3)]

    out = []
    for i, cx in enumerate(centres):
        out += _car(i + 1, cx, coll, mats,
                   nose_front=(i == 0), nose_rear=(i == 2))
    out += _pantograph(centres[1], coll, mats)

    for ob in out:
        ob.location.y += centre_y
        ob.location.z += rail_top_z
        ob["avi_kind"] = "train_car"
        ob["avi_static"] = True

    log(f"  train   : 3-car EMU, {total:.1f} m overall, centred x={centre_x:.0f}, "
        f"livery cream/{('Mumbai Metro blue')}, {len(out)} objects")
    return {"cars": 3, "length_m": total, "centre_x": centre_x,
            "objects": len(out)}

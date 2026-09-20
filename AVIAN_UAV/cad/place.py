"""Automatic non-interfering placement for the aft equipment bay.

Hand-placing the pump, filter, valves, sensor and HV relay one at a time
converged slowly -- every fix moved something into something else. This does
a small exhaustive search instead: each item is placed on a grid inside the
declared free zone and rejected if it overlaps fixed structure, a keep-out
corridor, or an item already placed.
"""
from __future__ import annotations
import itertools
import numpy as np
import params_b as P
import assy_b
from checks_b import placed

# free zone for the aft equipment bay, from the layout study
# Free zone for the aft equipment bay. The forward face is set by the aft
# spine cross member (x = -187) and the sides by the battery extraction
# corridors (|y| = 62). The bay extends 30 mm aft of the spine into a small
# tail fairing -- the rotor discs reach x = -762, so there is no conflict.
ZONE = dict(x=(-285.0, -190.0), y=(-60.0, 60.0), z=(-147.0, 36.0))

# keep-outs that are not modelled as parts
KEEPOUT = [
    ("battery_corridor_L", (-345.0, 62.0, -145.0), (-60.0, 138.0, -59.0)),
    ("battery_corridor_R", (-345.0, -138.0, -145.0), (-60.0, -62.0, -59.0)),
]

FIXED_SKIP = ("pump", "filter_40um", "check_valve", "relief_valve",
              "pressure_sensor", "power_switch_relay", "hose_", "fluid_charge")


def boxes_fixed():
    reg = assy_b.build("01_FLIGHT")
    out = []
    for p in reg.parts:
        if any(k in p.name for k in FIXED_SKIP):
            continue
        b = placed(p).BoundingBox()
        out.append((p.name, np.array([b.xmin, b.ymin, b.zmin]),
                    np.array([b.xmax, b.ymax, b.zmax])))
    for nm, lo, hi in KEEPOUT:
        out.append((nm, np.array(lo), np.array(hi)))
    return out


def local_extent(name):
    """Local bbox half-extents of a part, from a throwaway placement."""
    reg = assy_b.build("01_FLIGHT")
    for p in reg.parts:
        if p.name == name:
            b = placed(p).BoundingBox()
            c = p.world[:3, 3]
            lo = np.array([b.xmin, b.ymin, b.zmin]) - c
            hi = np.array([b.xmax, b.ymax, b.zmax]) - c
            return lo, hi
    raise KeyError(name)


def overlaps(lo1, hi1, lo2, hi2, gap=3.0):
    return bool(np.all(lo1 - gap < hi2) and np.all(lo2 < hi1 - (-gap)))


def clash(lo, hi, boxes, gap=3.0):
    for nm, l2, h2 in boxes:
        if np.all(lo - gap < h2) and np.all(l2 < hi + gap):
            return nm
    return None


# Preferred station for each item, so the packer spreads the bay sensibly
# instead of first-fitting everything into one corner: pump low and central
# (it is the heaviest and wants to sit near the bay floor), filter directly
# above it in the flow path, valves outboard, electrics on the top shelf.
ANCHOR = {
    "pump":               (-232.0, 0.0, -92.0),
    "filter_40um":        (-250.0, 0.0, 8.0),
    "relief_valve":       (-228.0, 42.0, 8.0),
    "check_valve":        (-228.0, -42.0, 8.0),
    "pressure_sensor":    (-210.0, 46.0, -60.0),
}


def solve(items, step=4.0):
    fixed = boxes_fixed()
    placed_boxes = []
    result = {}
    for name in items:
        lo_e, hi_e = local_extent(name)
        anchor = np.array(ANCHOR.get(name, (-225.0, 0.0, -60.0)))
        xs = np.arange(ZONE["x"][0] - lo_e[0], ZONE["x"][1] - hi_e[0], step)
        ys = np.arange(ZONE["y"][0] - lo_e[1], ZONE["y"][1] - hi_e[1], step)
        zs = np.arange(ZONE["z"][0] - lo_e[2], ZONE["z"][1] - hi_e[2], step)
        best, bestd = None, 1e18
        for x in xs:
            for y in ys:
                for z in zs:
                    c = np.array([x, y, z])
                    d = float(np.linalg.norm(c - anchor))
                    if d >= bestd:
                        continue
                    lo, hi = c + lo_e, c + hi_e
                    if clash(lo, hi, fixed) or clash(lo, hi, placed_boxes):
                        continue
                    best, bestd = c, d
        if best is None:
            result[name] = None
            print(f"  {name:20s} NO FREE POSITION in the aft bay")
        else:
            placed_boxes.append((name, best + lo_e, best + hi_e))
            result[name] = tuple(round(float(v), 1) for v in best)
            print(f"  {name:20s} -> {result[name]}"
                  f"   ({bestd:.0f} mm from its anchor)")
    return result


if __name__ == "__main__":
    print("aft equipment bay placement:")
    solve(["pump", "filter_40um", "relief_valve", "check_valve",
           "pressure_sensor"])

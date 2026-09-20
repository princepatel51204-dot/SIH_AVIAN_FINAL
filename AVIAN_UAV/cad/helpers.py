"""Geometry helpers, transform utilities and the part registry."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

import numpy as np
import cadquery as cq

import params_b as P

# ---------------------------------------------------------------------------
# transforms
# ---------------------------------------------------------------------------
def T(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def R(axis, deg) -> np.ndarray:
    """Rotation matrix about an arbitrary axis through the origin."""
    a = np.array(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(4)
    a = a / n
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    m = np.eye(4)
    m[:3, :3] = np.array([
        [c + x * x * (1 - c),     x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c),     y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])
    return m


def RX(d): return R((1, 0, 0), d)
def RY(d): return R((0, 1, 0), d)
def RZ(d): return R((0, 0, 1), d)


def chain(*mats) -> np.ndarray:
    out = np.eye(4)
    for m in mats:
        out = out @ m
    return out


def xform_pt(M: np.ndarray, p) -> np.ndarray:
    v = np.array([p[0], p[1], p[2], 1.0])
    return (M @ v)[:3]


def xform_vec(M: np.ndarray, v) -> np.ndarray:
    return M[:3, :3] @ np.array(v, dtype=float)


def mat_to_loc(M: np.ndarray) -> cq.Location:
    """numpy 4x4 -> cadquery Location."""
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    t = gp_Trsf()
    t.SetValues(float(M[0, 0]), float(M[0, 1]), float(M[0, 2]), float(M[0, 3]),
                float(M[1, 0]), float(M[1, 1]), float(M[1, 2]), float(M[1, 3]),
                float(M[2, 0]), float(M[2, 1]), float(M[2, 2]), float(M[2, 3]))
    return cq.Location(TopLoc_Location(t))


def rpy_from_mat(M: np.ndarray) -> Tuple[float, float, float]:
    """URDF-style fixed-axis roll-pitch-yaw (radians) from a rotation matrix."""
    r = M[:3, :3]
    sy = math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
    if sy > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# primitive builders
# ---------------------------------------------------------------------------
def octagon_pts(apothem: float) -> List[Tuple[float, float]]:
    """Regular octagon with flats normal to +/-X and +/-Y (across-flats = 2*apothem)."""
    circ = apothem / math.cos(math.radians(22.5))
    return [(circ * math.cos(math.radians(22.5 + 45 * i)),
             circ * math.sin(math.radians(22.5 + 45 * i))) for i in range(8)]


def octagon(apothem: float, thickness: float, z0: float = 0.0) -> cq.Workplane:
    return (cq.Workplane("XY", origin=(0, 0, z0))
            .polyline(octagon_pts(apothem)).close().extrude(thickness))


def octa_ring(apothem_out: float, wall: float, height: float, z0: float = 0.0) -> cq.Workplane:
    outer = octagon(apothem_out, height, z0)
    inner = octagon(apothem_out - wall, height + 2, z0 - 1)
    return outer.cut(inner)


def tube(od: float, idia: float, length: float, z0: float = 0.0) -> cq.Workplane:
    w = cq.Workplane("XY", origin=(0, 0, z0)).circle(od / 2.0).extrude(length)
    if idia > 0:
        w = w.cut(cq.Workplane("XY", origin=(0, 0, z0 - 1)).circle(idia / 2.0).extrude(length + 2))
    return w


def tube_along_x(od: float, idia: float, length: float, x0: float = 0.0) -> cq.Workplane:
    w = tube(od, idia, length, 0.0)
    return w.rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, 0, 0))


def hollow_box(l: float, w: float, h: float, wall: float, open_top=False) -> cq.Workplane:
    b = cq.Workplane("XY").box(l, w, h)
    cut_h = h - 2 * wall + (wall + 2 if open_top else 0)
    cz = 0.0 if not open_top else (wall + 2) / 2.0
    inner = cq.Workplane("XY", origin=(0, 0, cz)).box(l - 2 * wall, w - 2 * wall, cut_h)
    return b.cut(inner)


def rbox(l: float, w: float, h: float, fillet: float = 0.0) -> cq.Workplane:
    b = cq.Workplane("XY").box(l, w, h)
    if fillet > 0:
        try:
            b = b.edges("|Z").fillet(fillet)
        except Exception:
            pass
    return b


def bolt_holes(wp: cq.Workplane, pcd: float, n: int, d: float, depth: float,
               z: float, phase: float = 45.0) -> cq.Workplane:
    for i in range(n):
        a = math.radians(phase + 360.0 * i / n)
        wp = wp.cut(cq.Workplane("XY", origin=(pcd / 2 * math.cos(a),
                                               pcd / 2 * math.sin(a), z))
                    .circle(d / 2).extrude(depth))
    return wp


def lighten(wp: cq.Workplane, holes, z0, depth) -> cq.Workplane:
    for (x, y, d) in holes:
        wp = wp.cut(cq.Workplane("XY", origin=(x, y, z0)).circle(d / 2).extrude(depth))
    return wp


def swept_hose(points, od: float, idia: float = 0.0) -> cq.Workplane:
    """Round hose swept along a smooth spline through the given 3-D points."""
    pts = [cq.Vector(*p) for p in points]
    path = cq.Workplane("XY").spline(pts).wire()
    start = points[0]
    d = np.array(points[1]) - np.array(points[0])
    d = d / (np.linalg.norm(d) + 1e-12)
    prof = (cq.Workplane(cq.Plane(origin=cq.Vector(*start), normal=cq.Vector(*d)))
            .circle(od / 2.0))
    solid = prof.sweep(path, isFrenet=True)
    if idia > 0:
        prof2 = (cq.Workplane(cq.Plane(origin=cq.Vector(*start), normal=cq.Vector(*d)))
                 .circle(idia / 2.0))
        try:
            solid = solid.cut(prof2.sweep(path, isFrenet=True))
        except Exception:
            pass
    return solid


def cone_tip(d0: float, d1: float, length: float, z0: float = 0.0) -> cq.Workplane:
    return (cq.Workplane("XY", origin=(0, 0, z0))
            .circle(d0 / 2).workplane(offset=length).circle(max(d1 / 2, 0.2)).loft())


def helical_spring(d_mean: float, d_wire: float, length: float, turns: int = 6,
                   z0: float = 0.0) -> cq.Workplane:
    """Compression spring represented as a stack of coil rings (robust + cheap)."""
    out = None
    n = max(int(turns), 2)
    for i in range(n):
        z = z0 + (i + 0.5) * length / n
        ring = cq.Workplane("XY").add(
            cq.Solid.makeTorus((d_mean - d_wire) / 2.0, d_wire / 2.0)).translate((0, 0, z))
        out = ring if out is None else out.union(ring)
    return out


def fins(n: int, l: float, w: float, h: float, pitch: float) -> cq.Workplane:
    out = None
    for i in range(n):
        y = (i - (n - 1) / 2.0) * pitch
        f = cq.Workplane("XY", origin=(0, y, 0)).box(l, w, h)
        out = f if out is None else out.union(f)
    return out


# ---------------------------------------------------------------------------
# part registry
# ---------------------------------------------------------------------------
@dataclass
class Part:
    name: str                       # unique component id
    group: str                      # assembly group (e.g. "04_Propulsion_Arm_01")
    solid: cq.Workplane             # geometry in the part LOCAL frame
    world: np.ndarray               # 4x4 local->base_link transform
    material: str = "CFRP"
    color: str = "CFRP"
    mass: Optional[float] = None    # kg; None -> density * volume
    frame: Optional[str] = None     # ROS frame this part defines, if any
    note: str = ""
    explode: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # exploded-view direction (unit-ish)
    moving: bool = False            # part of a kinematic chain

    _vol: float = field(default=0.0, init=False, repr=False)

    def volume_mm3(self) -> float:
        if self._vol == 0.0:
            try:
                self._vol = self.solid.val().Volume()
            except Exception:
                self._vol = sum(s.Volume() for s in self.solid.vals())
        return self._vol

    def mass_kg(self) -> float:
        if self.mass is not None:
            return self.mass
        return self.volume_mm3() * 1e-9 * P.DENSITY[self.material]

    def local_com(self) -> np.ndarray:
        c = self.solid.val().Center()
        return np.array([c.x, c.y, c.z])

    def world_com(self) -> np.ndarray:
        return xform_pt(self.world, self.local_com())

    def rgb(self):
        return P.COLOR.get(self.color, (0.5, 0.5, 0.5))


class Registry:
    def __init__(self):
        self.parts: List[Part] = []

    def add(self, part: Part) -> Part:
        self.parts.append(part)
        return part

    def by_group(self):
        g = {}
        for p in self.parts:
            g.setdefault(p.group, []).append(p)
        return g

    def find(self, name):
        for p in self.parts:
            if p.name == name:
                return p
        return None

    def total_mass(self):
        return sum(p.mass_kg() for p in self.parts)

    def com(self):
        m = 0.0
        acc = np.zeros(3)
        for p in self.parts:
            mi = p.mass_kg()
            m += mi
            acc += mi * p.world_com()
        return acc / max(m, 1e-9), m

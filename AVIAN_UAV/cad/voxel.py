"""Voxel occupancy + Euclidean clearance field for the AVIAN airframe.

Axis-aligned bounding boxes are a poor proxy for parts like the battery tray
(four narrow rails whose bbox is a solid 310 x 284 slab) -- they reported a
48 mm collision where the arm actually passes through open air. This rasterises
the real solids instead and builds a signed distance field from them, so the
manipulator sweep measures clearance against geometry, not against envelopes.
"""
from __future__ import annotations
import numpy as np
from scipy import ndimage
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_OUT

import assy_b
from checks_b import placed

PITCH = 12.0
LO = np.array([-700.0, -700.0, -600.0])
HI = np.array([700.0, 700.0, 300.0])


def occupancy(reg, skip_groups=(), skip_names=()):
    dims = np.ceil((HI - LO) / PITCH).astype(int)
    occ = np.zeros(dims, dtype=bool)
    for p in reg.parts:
        if p.group in skip_groups or any(k in p.name for k in skip_names):
            continue
        s = placed(p)
        bb = s.BoundingBox()
        i0 = np.maximum(((np.array([bb.xmin, bb.ymin, bb.zmin]) - LO)
                         / PITCH).astype(int) - 1, 0)
        i1 = np.minimum(((np.array([bb.xmax, bb.ymax, bb.zmax]) - LO)
                         / PITCH).astype(int) + 2, dims)
        if np.any(i1 <= i0):
            continue
        cls = BRepClass3d_SolidClassifier(s.wrapped)
        for ix in range(i0[0], i1[0]):
            x = LO[0] + (ix + 0.5) * PITCH
            for iy in range(i0[1], i1[1]):
                y = LO[1] + (iy + 0.5) * PITCH
                for iz in range(i0[2], i1[2]):
                    if occ[ix, iy, iz]:
                        continue
                    z = LO[2] + (iz + 0.5) * PITCH
                    cls.Perform(gp_Pnt(x, y, z), 1e-6)
                    if cls.State() != TopAbs_OUT:
                        occ[ix, iy, iz] = True
    return occ


def clearance_field(occ):
    """Distance in mm from every voxel centre to the nearest occupied voxel."""
    return ndimage.distance_transform_edt(~occ, sampling=PITCH)


def sample(field, pts):
    """Nearest-voxel lookup for an (N, 3) array of world points."""
    idx = np.clip(((pts - LO) / PITCH).astype(int), 0,
                  np.array(field.shape) - 1)
    return field[idx[:, 0], idx[:, 1], idx[:, 2]]


def build(cfg="01_FLIGHT"):
    reg = assy_b.build(cfg)
    occ = occupancy(reg,
                    skip_groups=("06_MANIPULATOR", "07_TOOL_INTERFACE",
                                 "08_REPAIR_TOOLS"),
                    skip_names=("manipulator_hub", "propeller", "fluid_charge"))
    return occ, clearance_field(occ)


if __name__ == "__main__":
    import time
    t = time.time()
    occ, f = build()
    print(f"occupied voxels {occ.sum()}  ({PITCH:.0f} mm pitch)  "
          f"{time.time()-t:.1f} s")
    np.save("avian_clearance_field.npy", f.astype(np.float32))

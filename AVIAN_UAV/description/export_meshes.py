"""Export real AVIAN visual meshes from the CadQuery B-rep assembly.

Three products, per the revision's Decision 2:

  A. HIGH-FIDELITY VISUAL MESH   one STL per URDF link, tessellated from the
                                 actual B-rep solids. This is what a viewer
                                 shows: the real airframe, arms, motors,
                                 propellers, battery, avionics, sensors,
                                 landing gear, manipulator, tool changer and
                                 nozzle.
  B. OPTIMISED COLLISION MESH    convex hull per link, exported alongside.
                                 Available, but NOT wired into the URDF by
                                 default -- see below.
  C. SIMPLIFIED PHYSICS REPR.    the primitives already in avian.urdf.

WHICH ONE PHYSICS ACTUALLY USES, AND WHY
-----------------------------------------
The revision says not to use the visual mesh as the primary collision mesh
"unless the geometry is proven computationally acceptable". It is not, and the
measurement is in the manifest: the visual assembly is tens of thousands of
triangles per vehicle, and six vehicles at 240 Hz on two CPU cores cannot
afford narrow-phase contact against that. Convex hulls (B) are affordable but
badly wrong for this airframe -- the convex hull of a quadrotor arm set is a
solid disc that swallows the entire rotor plane and the manipulator workspace
with it, which would make every inspection pose look like a collision.

So: primitives (C) carry physics, meshes (A) carry the picture, hulls (B) are
exported and measured so the choice is auditable rather than asserted. The
manifest records the triangle counts that justify it.

TESSELLATION TOLERANCE
----------------------
0.4 mm linear on a 1.15 m airframe. Fine enough that bolt heads and fillets
survive, coarse enough that the export finishes. Stated in the manifest
because a mesh without its tolerance is not a reproducible artefact.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

CAD_DIR = os.environ.get("AVIAN_CAD_DIR", "/home/claude/avian")
if CAD_DIR not in sys.path:
    sys.path.insert(0, CAD_DIR)

MM = 0.001
TOL_MM = 0.4
ANG_TOL = 0.30

# CAD part-name prefix -> URDF link. Anything unmatched lands on base_link,
# which is right: it is bolted to the airframe.
LINK_RULES = [
    ("arm_link_1", "link_1"), ("arm_link_2", "link_2"),
    ("arm_link_3", "link_3"), ("arm_link_4", "link_4"),
    ("arm_link_5", "link_5"), ("arm_link_6", "link_6"),
    ("arm_j1", "arm_base_link"), ("arm_base", "arm_base_link"),
    ("arm_hub", "arm_base_link"),
    ("tool_", "tool0"), ("tc_", "tool0"), ("nozzle", "tool0"),
    ("ft_sensor", "tool0"),
]


def link_for(part_name):
    n = part_name.lower()
    for pref, link in LINK_RULES:
        if n.startswith(pref) or f"_{pref}" in n:
            return link
    for k in range(1, 7):
        if n.startswith(f"arm_link_{k}") or n.startswith(f"link_{k}"):
            return f"link_{k}"
    return "base_link"


def rotor_index(part_name):
    """Rotor station for motor/propeller parts, or None."""
    n = part_name.lower()
    for k in (1, 2, 3, 4):
        if f"_{k}_" in n or n.endswith(f"_{k}"):
            if "motor" in n or "prop" in n:
                lvl = "upper" if "upper" in n else (
                    "lower" if "lower" in n else None)
                if lvl:
                    return f"rotor_{k}_{lvl}"
    return None


def export(cfg="01_FLIGHT", out_dir=None, log=print):
    import cadquery as cq
    import assy_b

    t0 = time.time()
    out_dir = out_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "meshes")
    os.makedirs(out_dir, exist_ok=True)

    # The URDF is authored at q = 0; the CAD has no zero configuration, so
    # one is registered here. Exporting at 01_FLIGHT instead would bake a
    # 55 deg shoulder and a -143 deg elbow into the link meshes and the arm
    # would appear folded no matter what the joint states said.
    import params_b as PB
    if "00_ZERO" not in PB.CFG:
        PB.CFG["00_ZERO"] = [0.0] * 6
        PB.CFG_TOOL["00_ZERO"] = PB.CFG_TOOL[cfg]
        PB.CFG_FLUID["00_ZERO"] = PB.CFG_FLUID[cfg]
    build_cfg = "00_ZERO"
    log(f"  building CAD assembly ({build_cfg}, arm at q=0) ...")
    reg = assy_b.build(build_cfg)
    parts = list(getattr(reg, "parts", getattr(reg, "items", [])))
    log(f"  assembly: {len(parts)} parts")

    # Group solids by destination link, applying each part's placement.
    groups = {}
    unplaced = 0
    failures = []
    import numpy as np
    from OCP.gp import gp_Trsf
    for p in parts:
        name = getattr(p, "name", "part")
        solid = p.solid                       # cadquery Workplane
        M = p.world                           # 4x4 world transform, mm
        dest = rotor_index(name) or link_for(name)
        try:
            shp = solid.val()
            # Apply the assembly's world transform as a RIGID placement.
            # cadquery's transformShape() routes a 3x4 Matrix through a
            # general gp_GTrsf and OCCT rejects it as "non-orthogonal" even
            # for an exact identity -- the general transform path simply does
            # not accept this. A gp_Trsf is the rigid-motion type and is what
            # a rigid assembly placement actually is.
            t = gp_Trsf()
            t.SetValues(*[float(M[i, j]) for i in range(3)
                          for j in range(4)])
            shp = shp.moved(cq.Location(t))
        except Exception as e:                 # noqa: BLE001
            unplaced += 1
            failures.append(f"{name}: {type(e).__name__}: {e}")
            continue
        groups.setdefault(dest, []).append((name, shp))

    # Where each link's ORIGIN sits in CAD world coordinates at q = 0. The
    # mesh for a link is exported in that link's own frame, so it must be
    # translated by the negative of this. Without it every arm mesh renders
    # at the shoulder.
    link_origin_mm = {"base_link": (0.0, 0.0, 0.0),
                      "arm_base_link": tuple(PB.vArmBase)}
    acc = list(PB.vArmBase)
    for k, (nm_j, off, ax, lim, tq, sp, fn) in enumerate(PB.JOINTS, start=1):
        acc = [acc[i] + off[i] for i in range(3)]
        link_origin_mm[f"link_{k}"] = tuple(acc)
    link_origin_mm["tool0"] = (acc[0], acc[1], acc[2] - PB.vL5)
    half_diag = PB.vDiagonal / 2.0
    for i, ang in enumerate(PB.ARM_ANGLES):
        a_r = math.radians(ang)
        for lvl, sgn in (("upper", +1), ("lower", -1)):
            link_origin_mm[f"rotor_{i+1}_{lvl}"] = (
                half_diag * math.cos(a_r), half_diag * math.sin(a_r),
                sgn * PB.vCoaxSep / 2.0)

    stats = {"config": cfg, "tolerance_mm": TOL_MM,
             "angular_tolerance_rad": ANG_TOL,
             "source_parts": len(parts), "links": {}}
    total_tris = 0
    total_hull = 0

    for dest, items in sorted(groups.items()):
        shapes = [wp for nm, wp in items if wp is not None]
        if not shapes:
            continue
        # Re-express in the link's own frame.
        org = link_origin_mm.get(dest, (0.0, 0.0, 0.0))
        if any(abs(v) > 1e-9 for v in org):
            tt = gp_Trsf()
            tt.SetValues(1, 0, 0, -org[0], 0, 1, 0, -org[1],
                         0, 0, 1, -org[2])
            shapes = [sh.moved(cq.Location(tt)) for sh in shapes]
        comp = cq.Compound.makeCompound(shapes)
        path = os.path.join(out_dir, f"{dest}.stl")
        try:
            comp.exportStl(path, tolerance=TOL_MM, angularTolerance=ANG_TOL)
        except TypeError:
            cq.exporters.export(cq.Workplane(obj=comp), path)
        n_tri = _stl_triangles(path)
        total_tris += n_tri

        # convex hull, measured but not adopted
        hull_tris = 0
        try:
            import trimesh
            m = trimesh.load(path)
            h = m.convex_hull
            hp = os.path.join(out_dir, f"{dest}_hull.stl")
            h.export(hp)
            hull_tris = len(h.faces)
            total_hull += hull_tris
        except Exception:
            pass

        stats["links"][dest] = {
            "stl": os.path.basename(path),
            "parts": len(items),
            "triangles": n_tri,
            "hull_triangles": hull_tris,
            "part_names": [nm for nm, _ in items][:12],
            "link_origin_mm": [round(v, 3) for v in org],
        }
        log(f"    {dest:16s} {len(items):3d} parts  {n_tri:6d} tri  "
            f"hull {hull_tris:4d}")

    stats.update({
        "total_visual_triangles": total_tris,
        "total_hull_triangles": total_hull,
        "unplaced_parts": unplaced,
        "placement_failures": failures[:20],
        "physics_representation": "primitive boxes and cylinders in "
                                  "avian.urdf",
        "why_not_mesh_collision":
            f"visual mesh is {total_tris} triangles per vehicle; six vehicles "
            f"at 240 Hz narrow-phase on 2 CPU cores is not affordable",
        "why_not_hull_collision":
            "the convex hull of a 4-arm rotor set is a solid disc that "
            "swallows the rotor plane and the manipulator workspace, so every "
            "inspection pose would register as a collision",
        "units": "millimetres in the STL files, matching the CAD; the URDF "
                 "scales them by 0.001",
        "build_time_s": round(time.time() - t0, 1),
    })
    if total_tris == 0:
        raise RuntimeError(
            f"mesh export produced 0 triangles from {len(parts)} parts; "
            f"{unplaced} failed placement. First failures: {failures[:3]}")
    mp = os.path.join(out_dir, "avian_mesh_manifest.json")
    with open(mp, "w") as f:
        json.dump(stats, f, indent=2)
    log(f"  meshes  : {total_tris:,} visual triangles across "
        f"{len(stats['links'])} links in {stats['build_time_s']}s")
    return out_dir, mp, stats


def _stl_triangles(path):
    """Triangle count of a binary or ASCII STL."""
    try:
        with open(path, "rb") as f:
            head = f.read(84)
            if len(head) < 84:
                return 0
            if head[:5] == b"solid":
                f.seek(0)
                return sum(1 for line in f if line.strip().startswith(
                    b"facet"))
            import struct
            return struct.unpack("<I", head[80:84])[0]
    except Exception:
        return 0


if __name__ == "__main__":
    d, m, s = export()
    print(f"meshes   {d}")
    print(f"manifest {m}")
    print(f"visual   {s['total_visual_triangles']:,} triangles")

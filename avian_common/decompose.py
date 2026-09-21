"""The one collision decomposition, shared by both packages.

WHY THIS DIRECTORY EXISTS
-------------------------
`AVIAN_ENVIRONMENT` and `AVIAN_UAV` have no shared import path, and the
Gazebo exporter needs exactly the decomposition the PyBullet exporter
already produces. Writing a second one would be two sources of the same
numbers, drifting apart the first time a member changes -- the rule that
makes the URDF generated from CAD rather than hand-written, applied here.

Reached through `AVIAN_COMMON_DIR`, following the `AVIAN_CAD_DIR`
precedent. An env var rather than `pip install -e` because this has to
import from two different interpreters: system python3 for PyBullet, and
Blender's bundled Python for the environment. A pip install would have to
be done twice, into two environments, and would drift.

THE ORIGIN TRAP -- the reason _obb() looks the way it does
----------------------------------------------------------
`meshlib.box()` bakes the centre into the vertex coordinates and leaves the
object origin at (0,0,0). `matrix_world.translation` is therefore ZERO for
almost every object in this scene -- BR_, MB_, terrain, city, all of it.
Any pose taken from the object origin puts every member at the world
origin, stacked. It produces a file that parses, loads, and is wrong.

So the centre always comes from the world-space bounding box:
`matrix_world @ ob.bound_box`. That is what the PyBullet path has always
done, which is why it was never affected.
"""
from __future__ import annotations


def obb(ob, mathutils):
    """Axis-aligned box in world space, plus the object's world Z rotation.

    Every structural member in this model is either axis-aligned or rotated
    about Z only (the corridor runs along X), so a yaw-only oriented box is
    exact rather than a simplification.

    Returns (centre, half_extents, yaw, lo, hi) in world metres.
    """
    m = ob.matrix_world
    pts = [m @ mathutils.Vector(c) for c in ob.bound_box]
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    centre = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
    half = [max(1e-4, (hi[i] - lo[i]) / 2.0) for i in range(3)]
    yaw = m.to_euler("XYZ").z
    return centre, half, yaw, lo, hi


def load_primitives(path):
    """Read a collision asset written by export_bridge_collision.py.

    The Gazebo exporter consumes this rather than re-deriving it, so the
    two worlds are guaranteed to contain the same bodies in the same places.
    """
    import json
    with open(path) as f:
        d = json.load(f)
    prims = d["primitives"] if isinstance(d, dict) else d
    if not prims:
        raise ValueError(f"{path} contains no primitives")
    return prims

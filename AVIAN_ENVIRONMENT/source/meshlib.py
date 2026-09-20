"""Low-level mesh construction helpers.

Everything is built from explicit vertex/face lists rather than bpy.ops, for
three reasons: bpy.ops depends on context and breaks in headless runs, it is
an order of magnitude slower at this object count, and it leaves the scene in
whatever selection state the last operator produced.
"""
from __future__ import annotations
import math
import bpy
import bmesh
from mathutils import Vector, Matrix


# ---------------------------------------------------------------------------
def mesh_obj(name, verts, faces, coll=None, mat=None, smooth=False):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate(verbose=False)
    me.update()
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    ob = bpy.data.objects.new(name, me)
    if mat is not None:
        me.materials.append(mat)
    if coll is not None:
        coll.objects.link(ob)
    return ob


def link_dup(src, name, loc=(0, 0, 0), rot=(0, 0, 0), scale=(1, 1, 1),
             coll=None):
    """Linked duplicate: shares mesh data with the source.

    This is what keeps a 4.5 km corridor tractable -- 130 piers cost one mesh,
    not 130.
    """
    ob = bpy.data.objects.new(name, src.data)
    ob.location = loc
    ob.rotation_euler = rot
    ob.scale = scale
    if coll is not None:
        coll.objects.link(ob)
    return ob


def box(name, size, centre=(0, 0, 0), coll=None, mat=None):
    sx, sy, sz = (s / 2.0 for s in size)
    cx, cy, cz = centre
    v = [(cx - sx, cy - sy, cz - sz), (cx + sx, cy - sy, cz - sz),
         (cx + sx, cy + sy, cz - sz), (cx - sx, cy + sy, cz - sz),
         (cx - sx, cy - sy, cz + sz), (cx + sx, cy - sy, cz + sz),
         (cx + sx, cy + sy, cz + sz), (cx - sx, cy + sy, cz + sz)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return mesh_obj(name, v, f, coll, mat)


def prism(name, profile, length, axis="X", centre=(0, 0, 0), coll=None,
          mat=None, cap=True):
    """Extrude a 2-D profile along an axis.

    profile is a list of (a, b) in the plane normal to `axis`, counter-
    clockwise. Used for every I-girder, parapet and kerb in the model.
    """
    n = len(profile)
    h = length / 2.0
    cx, cy, cz = centre
    v = []
    for s in (-h, h):
        for a, b in profile:
            if axis == "X":
                v.append((cx + s, cy + a, cz + b))
            elif axis == "Y":
                v.append((cx + a, cy + s, cz + b))
            else:
                v.append((cx + a, cy + b, cz + s))
    f = []
    for i in range(n):
        j = (i + 1) % n
        f.append((i, j, n + j, n + i))
    if cap:
        f.append(tuple(range(n - 1, -1, -1)))
        f.append(tuple(range(n, 2 * n)))
    return mesh_obj(name, v, f, coll, mat)


def cylinder(name, r, h, centre=(0, 0, 0), seg=16, coll=None, mat=None,
             axis="Z", smooth=True):
    cx, cy, cz = centre
    v, f = [], []
    for s in (-h / 2.0, h / 2.0):
        for i in range(seg):
            a = 2 * math.pi * i / seg
            ca, sa = r * math.cos(a), r * math.sin(a)
            if axis == "Z":
                v.append((cx + ca, cy + sa, cz + s))
            elif axis == "X":
                v.append((cx + s, cy + ca, cz + sa))
            else:
                v.append((cx + ca, cy + s, cz + sa))
    for i in range(seg):
        j = (i + 1) % seg
        f.append((i, j, seg + j, seg + i))
    f.append(tuple(range(seg - 1, -1, -1)))
    f.append(tuple(range(seg, 2 * seg)))
    return mesh_obj(name, v, f, coll, mat, smooth=smooth)


def taper_box(name, size_bot, size_top, h, centre=(0, 0, 0), coll=None,
              mat=None):
    """Frustum -- pier caps, footings, tapered columns."""
    bx, by = size_bot[0] / 2, size_bot[1] / 2
    tx, ty = size_top[0] / 2, size_top[1] / 2
    cx, cy, cz = centre
    z0, z1 = cz - h / 2, cz + h / 2
    v = [(cx - bx, cy - by, z0), (cx + bx, cy - by, z0),
         (cx + bx, cy + by, z0), (cx - bx, cy + by, z0),
         (cx - tx, cy - ty, z1), (cx + tx, cy - ty, z1),
         (cx + tx, cy + ty, z1), (cx - tx, cy + ty, z1)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return mesh_obj(name, v, f, coll, mat)


def grid(name, x0, x1, y0, y1, nx, ny, zfn, coll=None, mat=None,
         smooth=True):
    """Height-field grid. Terrain, riverbed, water surface."""
    v, f = [], []
    for i in range(nx + 1):
        x = x0 + (x1 - x0) * i / nx
        for j in range(ny + 1):
            y = y0 + (y1 - y0) * j / ny
            v.append((x, y, zfn(x, y)))
    for i in range(nx):
        for j in range(ny):
            a = i * (ny + 1) + j
            f.append((a, a + 1, a + ny + 2, a + ny + 1))
    return mesh_obj(name, v, f, coll, mat, smooth=smooth)


def ribbon(name, pts_l, pts_r, coll=None, mat=None, smooth=False):
    """Quad strip between two polylines. Roads, kerbs, lane markings."""
    n = min(len(pts_l), len(pts_r))
    v = list(pts_l[:n]) + list(pts_r[:n])
    f = [(i, i + 1, n + i + 1, n + i) for i in range(n - 1)]
    return mesh_obj(name, v, f, coll, mat, smooth=smooth)


def plane(name, w, h, centre=(0, 0, 0), normal="Z", coll=None, mat=None,
          uv=True):
    """Single quad with UVs -- the carrier for every crack decal."""
    cx, cy, cz = centre
    a, b = w / 2, h / 2
    if normal == "Z":
        v = [(cx - a, cy - b, cz), (cx + a, cy - b, cz),
             (cx + a, cy + b, cz), (cx - a, cy + b, cz)]
    elif normal == "Y":
        v = [(cx - a, cy, cz - b), (cx + a, cy, cz - b),
             (cx + a, cy, cz + b), (cx - a, cy, cz + b)]
    else:
        v = [(cx, cy - a, cz - b), (cx, cy + a, cz - b),
             (cx, cy + a, cz + b), (cx, cy - a, cz + b)]
    ob = mesh_obj(name, v, [(0, 1, 2, 3)], coll, mat)
    if uv:
        me = ob.data
        lay = me.uv_layers.new(name="UVMap")
        for i, c in enumerate([(0, 0), (1, 0), (1, 1), (0, 1)]):
            lay.data[i].uv = c
    return ob


def join_objects(objs, name, coll):
    """Merge meshes into one object. Used to collapse LOD_LOW runs."""
    if not objs:
        return None
    bm = bmesh.new()
    for o in objs:
        me = o.data.copy()
        me.transform(o.matrix_world)
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out)
    bm.free()
    ob = bpy.data.objects.new(name, out)
    if objs[0].data.materials:
        out.materials.append(objs[0].data.materials[0])
    coll.objects.link(ob)
    for o in objs:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        bpy.data.objects.remove(o, do_unlink=True)
    return ob


def tri_count(ob):
    me = ob.data
    if not hasattr(me, "polygons"):
        return 0
    return sum(max(0, len(p.vertices) - 2) for p in me.polygons)


def scene_tris():
    t = 0
    for o in bpy.data.objects:
        if o.type == "MESH":
            t += tri_count(o) * max(1, _instances(o))
    return t


def _instances(o):
    return 1


def displace_random(ob, amount, seed=0, scale=1.0):
    """Break up a primitive so it does not read as a primitive.

    Applied to rocks and terrain patches. Deliberately not applied to any
    structural concrete: a bridge girder that visibly wobbles is worse than
    one that is clean.
    """
    import random
    rnd = random.Random(seed)
    me = ob.data
    for v in me.vertices:
        v.co.x += rnd.uniform(-amount, amount) * scale
        v.co.y += rnd.uniform(-amount, amount) * scale
        v.co.z += rnd.uniform(-amount, amount) * scale
    me.update()
    return ob


def set_custom(ob, props: dict):
    """Attach ground-truth metadata to an object."""
    for k, v in props.items():
        ob[k] = v
    return ob


def look_at(ob, target, up=(0.0, 0.0, 1.0), roll=0.0):
    """Aim an object's -Z axis at a world point.

    Hand-written Euler angles for a camera are a reliable way to waste an
    afternoon: the sign conventions and the XYZ application order both bite,
    and a wrong guess renders as an empty frame that looks like a missing
    object. Aiming at a target is unambiguous.
    """
    from mathutils import Vector, Matrix
    loc = Vector(ob.location)
    tgt = Vector(target)
    fwd = (loc - tgt).normalized()          # camera looks along -Z
    upv = Vector(up)
    if abs(fwd.dot(upv)) > 0.999:
        upv = Vector((0.0, 1.0, 0.0))
    right = upv.cross(fwd).normalized()
    up2 = fwd.cross(right).normalized()
    m = Matrix((
        (right.x, up2.x, fwd.x, loc.x),
        (right.y, up2.y, fwd.y, loc.y),
        (right.z, up2.z, fwd.z, loc.z),
        (0.0, 0.0, 0.0, 1.0),
    ))
    if roll:
        m = m @ Matrix.Rotation(roll, 4, "Z")
    ob.matrix_world = m
    return ob

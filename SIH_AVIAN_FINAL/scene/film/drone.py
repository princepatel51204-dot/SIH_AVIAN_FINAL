"""Build the x500 airframe in Blender from the real PX4 meshes.

Every transform below is read out of x500_base/model.sdf, not eyeballed:

  base_link (static)
    NXP-HGD-CF.dae   pose (0, 0, 0.025)  yaw pi
    5010Base.dae x4  pose (+-0.174, +-0.174, 0.032)  yaw -0.45

  rotor_N (spins about its own +Z)
    link pose        rotor_0 ( 0.174, -0.174, 0.06)   1345_prop_ccw.stl
                     rotor_1 (-0.174,  0.174, 0.06)   1345_prop_ccw.stl
                     rotor_2 ( 0.174,  0.174, 0.06)   1345_prop_cw.stl
                     rotor_3 (-0.174, -0.174, 0.06)   1345_prop_cw.stl
    prop visual      local (-0.022, -0.146385, -0.016)   <- the STL is modeled
                     off-origin, so the prop is parented under an empty AT the
                     link pose and offset inside it; spinning the mesh directly
                     would make the blades orbit instead of rotate.
    5010Bell.dae     local (0, 0, -0.032)

The 2-D lidar is rebuilt from the box + cylinder primitives declared in
garudanex_sim/models/lidar_2d_v2/model.sdf. That SDF also references
meshes/lidar_2d_v2.dae, which does not exist on this machine, so the primitives
are the faithful fallback rather than an invention.

Everything lands in one AVIAN_DRONE collection so onboard shots hide it with a
single flag, and the whole rig hangs off a DRONE_ROOT empty so the flight data
drives exactly one object.
"""
import bpy, os, math
from mathutils import Vector, Euler

PX4_MESHES = "/home/prince/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes"
# This Blender build ships no Collada importer (bpy.ops.wm.collada_import is
# absent; only obj/ply/stl/gltf/fbx/x3d exist). The three .dae parts are
# therefore converted to .glb by scene/film/convert_meshes.py using trimesh +
# pycollada, which preserves the real geometry and its scene-graph transforms
# -- measured after conversion: frame 0.396 x 0.396 x 0.279 m, motor base
# 28.6 mm, bell 59 mm. Nothing is substituted or re-modelled.
GLB = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/film/meshes"
MESHES = PX4_MESHES
COLL = "AVIAN_DRONE"

ROTORS = [
    (0, ( 0.174, -0.174, 0.06), "1345_prop_ccw.stl",  1.0),
    (1, (-0.174,  0.174, 0.06), "1345_prop_ccw.stl",  1.0),
    (2, ( 0.174,  0.174, 0.06), "1345_prop_cw.stl",  -1.0),
    (3, (-0.174, -0.174, 0.06), "1345_prop_cw.stl",  -1.0),
]
PROP_VIS_OFFSET = (-0.022, -0.14638461538461536, -0.016)
PROP_SCALE = 0.8461538461538461   # <scale> on rotor_N_visual in model.sdf
BELL_OFFSET = (0.0, 0.0, -0.032)
MOTOR_BASES = [( 0.174,  0.174, 0.032), (-0.174,  0.174, 0.032),
               ( 0.174, -0.174, 0.032), (-0.174, -0.174, 0.032)]
MOTOR_BASE_YAW = -0.45
SPIN_RPS = 9.0      # visual only: fast enough to blur, slow enough not to strobe


def _collection(scene):
    if COLL in bpy.data.collections:
        c = bpy.data.collections[COLL]
        for ob in list(c.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        return c
    c = bpy.data.collections.new(COLL)
    scene.collection.children.link(c)
    return c


def _import(path):
    before = set(bpy.data.objects)
    ext = os.path.splitext(path)[1].lower()
    if ext == ".glb":
        bpy.ops.import_scene.gltf(filepath=path)
    else:
        try:
            bpy.ops.wm.stl_import(filepath=path)
        except AttributeError:
            bpy.ops.import_mesh.stl(filepath=path)
    return [o for o in bpy.data.objects if o not in before]


def _adopt(objs, coll):
    for o in objs:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        coll.objects.link(o)


def _place(path, coll, name, loc, yaw=0.0, parent=None, report=None, scale=1.0):
    if not os.path.exists(path):
        if report is not None:
            report["missing"].append(path)
        return None
    objs = _import(path)
    if not objs:
        if report is not None:
            report["missing"].append(path + " (imported 0 objects)")
        return None
    _adopt(objs, coll)
    root = objs[0]
    for o in objs[1:]:
        if o.parent is None:
            o.parent = root
    root.name = name
    root.location = Vector(loc)
    root.rotation_euler = Euler((0, 0, yaw))
    if scale != 1.0:
        root.scale = (scale, scale, scale)
    if parent is not None:
        root.parent = parent
    if report is not None:
        report["imported"].append({"mesh": os.path.basename(path), "as": name,
                                   "parts": len(objs)})
    return root


def import_drone(scene, spin=True, frame_start=1, frame_end=3000):
    coll = _collection(scene)
    report = {"collection": COLL, "imported": [], "missing": [], "rotors": []}

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
    root = bpy.context.active_object
    root.name = "DRONE_ROOT"
    root.empty_display_size = 0.25
    _adopt([root], coll)

    _place(f"{GLB}/NXP-HGD-CF.glb", coll, "DRONE_FRAME",
           (0, 0, 0.025), math.pi, root, report)

    for i, loc in enumerate(MOTOR_BASES):
        _place(f"{GLB}/5010Base.glb", coll, f"MOTOR_BASE_{i}",
               loc, MOTOR_BASE_YAW, root, report)

    fps = scene.render.fps or 24
    turns = SPIN_RPS * (frame_end - frame_start) / fps

    for idx, loc, prop_mesh, direction in ROTORS:
        bpy.ops.object.empty_add(type="PLAIN_AXES", location=loc)
        hub = bpy.context.active_object
        hub.name = f"ROTOR_{idx}"
        hub.empty_display_size = 0.05
        _adopt([hub], coll)
        hub.parent = root

        _place(f"{MESHES}/{prop_mesh}", coll, f"PROP_{idx}",
               PROP_VIS_OFFSET, 0.0, hub, report, scale=PROP_SCALE)
        _place(f"{GLB}/5010Bell.glb", coll, f"BELL_{idx}",
               BELL_OFFSET, 0.0, hub, report)

        if spin:
            hub.rotation_mode = "XYZ"
            for f, t in ((frame_start, 0.0), (frame_end, turns)):
                hub.rotation_euler = (0, 0, direction * t * 2 * math.pi)
                hub.keyframe_insert("rotation_euler", index=2, frame=f)
            for fc in hub.animation_data.action.fcurves:
                fc.extrapolation = "LINEAR"
                for kp in fc.keyframe_points:
                    kp.interpolation = "LINEAR"
        report["rotors"].append({"rotor": idx, "mesh": prop_mesh,
                                 "dir": "CCW" if direction > 0 else "CW"})

    # lidar puck from lidar_2d_v2/model.sdf primitives
    lz = 0.09
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, lz))
    base = bpy.context.active_object
    base.name = "LIDAR_BASE"
    base.scale = (0.03, 0.03, 0.0205)
    bpy.ops.mesh.primitive_cylinder_add(radius=0.025, depth=0.028,
                                        location=(0, 0, lz + 0.0345))
    drum = bpy.context.active_object
    drum.name = "LIDAR_DRUM"
    _adopt([base, drum], coll)
    for ob in (base, drum):
        ob.parent = root
    report["imported"].append({
        "mesh": "lidar_2d_v2 primitives (box + cylinder)",
        "note": "model.sdf references meshes/lidar_2d_v2.dae, which is absent here"})

    report["root"] = root.name
    report["n_objects"] = len(coll.objects)
    report["spin_rps"] = SPIN_RPS if spin else 0
    return report


if __name__ == "__main__":
    import json
    r = import_drone(bpy.context.scene)
    print("===DRONE===")
    print(json.dumps(r, indent=1))
    print("===DONE_DRONE===")

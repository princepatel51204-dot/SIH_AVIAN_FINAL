"""Blender (headless) render pass for the Phase 1 demo video.

Run under Blender's own Python, not the project venv:

    blender --background --python media/render_blender.py -- \\
        --track media/avian_demo_track.json --outdir media/frames_main \\
        [--start 0] [--end -1]

Imports the real exported STL meshes (description/meshes/*.stl) -- never
primitives -- and poses each link every frame from the world poses PyBullet
already recorded in avian_demo_track.json. No new geometry, physics or
kinematics is computed here; this script only places meshes and renders.
"""
from __future__ import annotations

import json
import math
import os
import sys

import bpy
from mathutils import Quaternion, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MESH_DIR = os.path.join(ROOT, "description", "meshes")

MESH_LINKS = [
    "base_link", "arm_base_link",
    "rotor_1_upper", "rotor_1_lower", "rotor_2_upper", "rotor_2_lower",
    "rotor_3_upper", "rotor_3_lower", "rotor_4_upper", "rotor_4_lower",
    "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0",
]

MATERIAL_OF = {
    "base_link": "avian_body",
    "arm_base_link": "avian_arm",
    "tool0": "avian_arm",
    **{f"link_{k}": "avian_arm" for k in range(1, 7)},
    **{f"rotor_{a}_{lv}": "avian_rotor"
       for a in range(1, 5) for lv in ("upper", "lower")},
}

MATERIAL_RGBA = {
    "avian_body": (0.16, 0.17, 0.19, 1.0),
    "avian_arm": (0.62, 0.63, 0.66, 1.0),
    "avian_rotor": (0.08, 0.08, 0.09, 1.0),
}


def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    out = {"track": os.path.join(HERE, "avian_demo_track.json"),
          "outdir": os.path.join(HERE, "frames_main"),
          "start": 0, "end": -1}
    i = 0
    while i < len(argv):
        key = argv[i].lstrip("-")
        if key in ("track", "outdir"):
            out[key] = argv[i + 1]
            i += 2
        elif key in ("start", "end"):
            out[key] = int(argv[i + 1])
            i += 2
        else:
            i += 1
    return out


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def make_material(name, rgba, emission_strength=0.0, emission_color=(0, 0, 0)):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Roughness"].default_value = 0.55
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = 0.15
    if emission_strength > 0.0 and "Emission Color" in bsdf.inputs:
        bsdf.inputs["Emission Color"].default_value = (*emission_color, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    return mat


def import_stl(path, name):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=path, forward_axis='Y', up_axis='Z',
                          global_scale=0.001)
    new = list(set(bpy.data.objects) - before)
    obj = new[0]
    obj.name = name
    obj.rotation_mode = 'QUATERNION'
    return obj


def build_studio(floor_z=0.0):
    world = bpy.data.worlds.new("Studio")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.62, 0.63, 0.66, 1.0)
    bg.inputs["Strength"].default_value = 0.4

    grey = make_material("studio_grey", (0.5, 0.51, 0.53, 1.0))
    bpy.ops.mesh.primitive_plane_add(size=60.0, location=(0, 0, floor_z))
    floor = bpy.context.active_object
    floor.name = "studio_floor"
    floor.data.materials.append(grey)

    bpy.ops.mesh.primitive_plane_add(size=60.0,
                                     location=(20.0, 0, floor_z + 20.0))
    wall = bpy.context.active_object
    wall.name = "studio_wall"
    wall.rotation_euler = (0.0, math.radians(90.0), 0.0)
    wall.data.materials.append(grey)

    key = bpy.data.lights.new("key", type='AREA')
    key.energy = 700.0
    key.size = 6.0
    key_obj = bpy.data.objects.new("key", key)
    key_obj.location = (4.0, -6.0, 9.0)
    key_obj.rotation_euler = (math.radians(55.0), 0.0, math.radians(30.0))
    bpy.context.collection.objects.link(key_obj)

    fill = bpy.data.lights.new("fill", type='AREA')
    fill.energy = 250.0
    fill.size = 8.0
    fill_obj = bpy.data.objects.new("fill", fill)
    fill_obj.location = (-6.0, -3.0, 6.0)
    fill_obj.rotation_euler = (math.radians(65.0), 0.0, math.radians(-40.0))
    bpy.context.collection.objects.link(fill_obj)

    rim = bpy.data.lights.new("rim", type='AREA')
    rim.energy = 400.0
    rim.size = 4.0
    rim_obj = bpy.data.objects.new("rim", rim)
    rim_obj.location = (2.0, 8.0, 7.0)
    rim_obj.rotation_euler = (math.radians(110.0), 0.0, math.radians(160.0))
    bpy.context.collection.objects.link(rim_obj)


def build_panel(panel):
    half = panel["half_extents_m"]
    cx, cy, cz = panel["center_m"]
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(cx, cy, cz))
    obj = bpy.context.active_object
    obj.name = "repair_panel"
    obj.scale = (half[0], half[1], half[2])
    mat = make_material("panel_mat", (0.58, 0.58, 0.60, 1.0))
    obj.data.materials.append(mat)
    return obj, mat


def set_pose(obj, pos, quat_xyzw):
    obj.location = Vector(pos)
    x, y, z, w = quat_xyzw
    obj.rotation_quaternion = Quaternion((w, x, y, z))


def look_at(cam_obj, cam_pos, target):
    cam_obj.location = Vector(cam_pos)
    direction = (Vector(target) - Vector(cam_pos))
    cam_obj.rotation_mode = 'QUATERNION'
    cam_obj.rotation_quaternion = direction.to_track_quat('-Z', 'Y')


def main():
    args = parse_args()
    track = json.load(open(args["track"]))
    frames = track["frames"]
    start = args["start"]
    end = args["end"] if args["end"] >= 0 else len(frames)

    clear_scene()
    build_studio(floor_z=0.0)

    mats = {name: make_material(name, rgba)
           for name, rgba in MATERIAL_RGBA.items()}
    objs = {}
    for link in MESH_LINKS:
        obj = import_stl(os.path.join(MESH_DIR, f"{link}.stl"), link)
        obj.data.materials.append(mats[MATERIAL_OF[link]])
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.shade_smooth()
        try:
            obj.data.use_auto_smooth = True
            obj.data.auto_smooth_angle = math.radians(35.0)
        except AttributeError:
            pass
        objs[link] = obj

    panel_obj, panel_mat = build_panel(track["panel"])
    panel_bsdf = panel_mat.node_tree.nodes["Principled BSDF"]

    cam_data = bpy.data.cameras.new("main_cam")
    cam_data.lens = 35.0
    cam_obj = bpy.data.objects.new("main_cam", cam_data)
    bpy.context.collection.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj

    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE_NEXT' if 'BLENDER_EEVEE_NEXT' in \
        [e.identifier for e in
         bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items] \
        else 'BLENDER_EEVEE'
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.view_settings.view_transform = 'Standard'
    scene.frame_start = 1
    scene.frame_end = end - start

    n = len(frames)
    orbit_deg_total = 130.0
    orbit_radius = 4.6
    orbit_height = 1.2

    os.makedirs(args["outdir"], exist_ok=True)

    for i in range(start, end):
        fr = frames[i]
        for link in MESH_LINKS:
            lp = fr["links"][link]
            set_pose(objs[link], lp["position"], lp["quaternion"])

        # Illustrative repair-applied cue: emission glow only during the hold.
        glow = 2.0 if fr["phase"] == "repair_cue" else 0.0
        if "Emission Color" in panel_bsdf.inputs:
            panel_bsdf.inputs["Emission Color"].default_value = \
                (0.15, 0.9, 0.35, 1.0)
            panel_bsdf.inputs["Emission Strength"].default_value = glow

        target = Vector(fr["links"]["base_link"]["position"])
        frac = i / max(1, n - 1)
        ang = math.radians(-65.0 + orbit_deg_total * frac)
        cam_pos = (target.x + orbit_radius * math.cos(ang),
                  target.y + orbit_radius * math.sin(ang),
                  target.z + orbit_height)
        look_at(cam_obj, cam_pos, target)

        scene.render.filepath = os.path.join(args["outdir"],
                                             f"frame_{i:05d}.png")
        bpy.ops.render.render(write_still=True)

        if i % 25 == 0 or i == end - 1:
            print(f"blender render {i + 1}/{n}", flush=True)

    print(f"wrote frames {start}..{end - 1} to {args['outdir']}")


if __name__ == "__main__":
    main()

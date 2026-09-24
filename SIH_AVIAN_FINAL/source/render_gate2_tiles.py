"""Gate 3 (time-boxed): render one tile per Gazebo Gate 2 achieved pose.

Simplified from render_zoom_tiles_final.py (1 tile/waypoint, not a full
zoom grid) to fit a hard session deadline. Same camera-aim rule: point at
the structure, ray-cast to confirm a real hit, skip ENV_* meshes. Reads
mission/gazebo_gate2_{mission,flight_log}.json (produced by
export_gazebo_gate2_flightlog.py from real Gazebo TF poses). No defect
ground truth is read here.

Usage: blender --background --python run_blender.py -- source/render_gate2_tiles.py
"""
import json
import os
import sys

import bpy
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
MISSION_DIR = os.path.join(ROOT, "mission")
DET_DIR = os.path.join(ROOT, "detection")
OUT_DIR = os.path.join(DET_DIR, "renders_gate2")

RES = 480
HFOV_DEG = 40.0
_ENV_SKIP_PREFIX = "ENV_"


def cam_frame(cam_pos, target):
    fwd = (cam_pos - target)
    if fwd.length < 1e-9:
        fwd = Vector((0, 0, 1))
    fwd = fwd.normalized()
    up_world = Vector((0.0, 0.0, 1.0))
    if abs(fwd.dot(up_world)) > 0.999:
        up_world = Vector((0.0, 1.0, 0.0))
    right = up_world.cross(fwd).normalized()
    up2 = fwd.cross(right).normalized()
    return right, up2, fwd


def set_pose(cam, cam_pos, direction):
    fwd = -direction.normalized()
    up_world = Vector((0.0, 0.0, 1.0))
    if abs(fwd.dot(up_world)) > 0.999:
        up_world = Vector((0.0, 1.0, 0.0))
    right = up_world.cross(fwd).normalized()
    up2 = fwd.cross(right).normalized()
    cam.matrix_world = Matrix((
        (right.x, up2.x, fwd.x, cam_pos.x),
        (right.y, up2.y, fwd.y, cam_pos.y),
        (right.z, up2.z, fwd.z, cam_pos.z),
        (0.0, 0.0, 0.0, 1.0)))


def main():
    print("== Gate 3: render_gate2_tiles.py ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(OUT_DIR, exist_ok=True)

    with open(os.path.join(MISSION_DIR, "gazebo_gate2_flight_log.json")) as f:
        flight_log = json.load(f)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = RES
    scene.render.resolution_y = RES
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.use_nodes = False

    cam_data = bpy.data.cameras.get("AVI_GATE2_CAM") or bpy.data.cameras.new("AVI_GATE2_CAM")
    cam_data.clip_start, cam_data.clip_end = 0.01, 400.0
    cam_data.sensor_fit = "HORIZONTAL"
    cam_data.angle = __import__("math").radians(HFOV_DEG)
    cam = bpy.data.objects.get("AVI_GATE2_CAM")
    if cam is None:
        cam = bpy.data.objects.new("AVI_GATE2_CAM", cam_data)
        bpy.context.scene.collection.objects.link(cam)
    scene.camera = cam

    dg = bpy.context.evaluated_depsgraph_get()
    manifest = []
    for e in flight_log["log"]:
        wid = e["waypoint_id"]
        cam_pos = Vector(e["achieved_position_m"])
        target = Vector(e["target_m"])
        right, up2, fwd = cam_frame(cam_pos, target)
        direction = -fwd
        hit, loc, nrm, idx, ob, mw = scene.ray_cast(
            dg, cam_pos, direction, distance=40.0)
        if not hit or ob is None or ob.name.startswith(_ENV_SKIP_PREFIX):
            print(f"  {wid}: no structure hit, skipped")
            continue
        set_pose(cam, cam_pos, direction)
        out_path = os.path.join(OUT_DIR, f"{wid}.png")
        scene.render.filepath = out_path
        bpy.ops.render.render(write_still=True)
        manifest.append({
            "waypoint_id": wid,
            "tile_id": wid,
            "achieved_position_m": list(cam_pos),
            "tile_target_m": list(target),
            "standoff_m": (cam_pos - target).length,
            "image_path": os.path.relpath(out_path, ROOT),
            "hit_object": ob.name,
        })
        print(f"  {wid}: rendered, hit {ob.name}, standoff {(cam_pos-target).length:.1f} m")

    with open(os.path.join(DET_DIR, "AVIAN_gate2_tiles.json"), "w") as f:
        json.dump({"tiles": manifest}, f, indent=2)
    print(f"== done: {len(manifest)} tiles ==")


if __name__ == "__main__":
    main()

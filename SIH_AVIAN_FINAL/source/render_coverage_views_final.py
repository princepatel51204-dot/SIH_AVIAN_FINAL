"""SIH_AVIAN_FINAL -- Phase 2 step 6: render the achieved-pose views for the
real detector to run on.

Every settled, non-stuck, non-skipped Stage 2 coverage waypoint already
has a REAL achieved position logged in `mission/coverage_mission_flight_log.json`
(Stage 1/2's whole reason for logging achieved pose instead of the
commanded target -- VB01). This renders the RGB view from that achieved
position, not the commanded one, so `detect_stub_final.py`'s real
inference runs on what the aircraft actually saw.

EEVEE, not Cycles: no instance-index mask is needed here (that was only
ever for the detection DATASET's ground-truth masks), so there is no
Cycles-after-`ray_cast()` segfault risk and no reason to pay Cycles' cost
-- ~150 renders at EEVEE speed is the "~6 s each" the master prompt
expects.

Orientation is a stated simplification: the camera looks from the
achieved POSITION toward the waypoint's own recorded `target_m` (the
patch point `coverage_final.py` generated it to see), not the achieved
quaternion converted from PyBullet's body frame into Blender's camera
frame. Position is where "achieved vs commanded" actually matters for
what ends up in frame (SETTLE_TOL_M=0.15 m of position error dwarfs any
heading error at these standoffs); reusing `_look_at()` avoids a body-
frame axis-convention conversion this pass has no time to validate.

Usage:
    blender --background --python run_blender.py -- source/render_coverage_views_final.py
"""
from __future__ import annotations
import json
import math
import os
import sys
import time

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
MISSION_DIR = os.path.join(ROOT, "mission")
DET_DIR = os.path.join(ROOT, "detection")
RENDER_DIR = os.path.join(DET_DIR, "renders")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
import dataset_final as DSF   # reuses HFOV_DEG/LOOP_RES, sensor intrinsics

HFOV_DEG = DSF.HFOV_DEG
LOOP_RES = DSF.LOOP_RES


def _get_or_create_camera(name="AVI_DETECT_CAM"):
    cam_data = bpy.data.cameras.get(name)
    if cam_data is None:
        cam_data = bpy.data.cameras.new(name)
    cam_data.clip_start = 0.01
    cam_data.clip_end = 400.0
    cam_data.sensor_fit = "HORIZONTAL"
    cam_data.angle = math.radians(HFOV_DEG)
    cam = bpy.data.objects.get(name)
    if cam is None:
        cam = bpy.data.objects.new(name, cam_data)
        bpy.context.scene.collection.objects.link(cam)
    return cam


def _look_at(ob, loc, target, up=(0.0, 0.0, 1.0)):
    from mathutils import Matrix
    loc = Vector(loc)
    tgt = Vector(target)
    fwd = (loc - tgt)
    if fwd.length < 1e-9:
        fwd = Vector((0, 0, 1))
    fwd = fwd.normalized()
    upv = Vector(up)
    if abs(fwd.dot(upv)) > 0.999:
        upv = Vector((0.0, 1.0, 0.0))
    right = upv.cross(fwd).normalized()
    up2 = fwd.cross(right).normalized()
    ob.matrix_world = Matrix((
        (right.x, up2.x, fwd.x, loc.x),
        (right.y, up2.y, fwd.y, loc.y),
        (right.z, up2.z, fwd.z, loc.z),
        (0.0, 0.0, 0.0, 1.0)))


def _settled_waypoint_ids(flight_log):
    return {e["waypoint_id"] for e in flight_log["log"]
           if e.get("settled") and not e.get("stuck")
           and not e.get("skipped")}


def _achieved_position(flight_log, waypoint_id):
    for e in flight_log["log"]:
        if e["waypoint_id"] == waypoint_id:
            return e["achieved_position_m"]
    return None


def main():
    t0 = time.time()
    print("== SIH_AVIAN_FINAL :: render_coverage_views_final.py (Phase 2) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(RENDER_DIR, exist_ok=True)

    with open(os.path.join(MISSION_DIR, "coverage_mission.json")) as f:
        mission = json.load(f)
    with open(os.path.join(MISSION_DIR,
                          "coverage_mission_flight_log.json")) as f:
        flight_log = json.load(f)

    mission_ids = {w["waypoint_id"] for w in mission["waypoints"]}
    settled = _settled_waypoint_ids(flight_log) & mission_ids
    print(f"  settled : {len(settled)}/{mission['n_waypoints']} coverage "
         "waypoints eligible for render")

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = LOOP_RES[0]
    scene.render.resolution_y = LOOP_RES[1]
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.use_nodes = False
    cam = _get_or_create_camera()
    scene.camera = cam

    manifest = []
    for i, wp in enumerate(mission["waypoints"]):
        if wp["waypoint_id"] not in settled:
            continue
        pos = _achieved_position(flight_log, wp["waypoint_id"])
        target = wp["target_m"]
        _look_at(cam, pos, target)
        out_path = os.path.join(RENDER_DIR, f"{wp['waypoint_id']}.png")
        scene.render.filepath = out_path
        bpy.ops.render.render(write_still=True)
        manifest.append({
            "waypoint_id": wp["waypoint_id"],
            "image_path": os.path.relpath(out_path, ROOT),
            "achieved_position_m": pos,
            "target_m": target,
            "prim_name": wp.get("prim_name"),
            "prim_kind": wp.get("prim_kind"),
            "covers": wp.get("covers"),
        })
        if (i + 1) % 20 == 0:
            print(f"  ... {len(manifest)} rendered")

    out_path = os.path.join(DET_DIR, "AVIAN_coverage_renders_FINAL.json")
    with open(out_path, "w") as f:
        json.dump({"renders": manifest}, f, indent=2)
    dt = time.time() - t0
    print(f"  render  : {len(manifest)} frames in {dt:.1f}s "
         f"({dt / max(1, len(manifest)):.2f} s/frame)")
    print(f"  saved   : {out_path}")
    print("FINAL_RENDER_COMPLETE")


if __name__ == "__main__":
    main()

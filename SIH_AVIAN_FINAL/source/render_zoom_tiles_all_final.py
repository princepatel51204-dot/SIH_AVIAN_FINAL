"""SIH_AVIAN_FINAL -- Phase 2 Section 3 step 1: zoom tiles for EVERY
settled Stage 2 waypoint, not just Section 2's 16.

Reuses Section 2's own tiling/ray-cast/zoom-FOV code verbatim (imported
from `render_zoom_tiles_final.py`, not copied) so the two passes agree on
what a tile is; only the waypoint SELECTION differs (all 82 settled
waypoints here, instead of a 16-waypoint stratified sample) and the
outputs land in separate files so Section 2's own committed artifacts
are untouched.

Usage:
    blender --background --python run_blender.py -- source/render_zoom_tiles_all_final.py
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
ZOOM_RENDER_DIR = os.path.join(DET_DIR, "renders_zoom_all")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
import render_zoom_tiles_final as RZT   # reused verbatim -- see docstring


def main():
    t0 = time.time()
    print("== SIH_AVIAN_FINAL :: render_zoom_tiles_all_final.py "
         "(Phase 2 sec3 step1) ==")
    bpy.ops.wm.open_mainfile(filepath=RZT.BLEND_PATH)
    os.makedirs(ZOOM_RENDER_DIR, exist_ok=True)

    target_mm_px = RZT._median_train_gsd_mm_px()
    print(f"  zoom target: median train LOOP mm/px = {target_mm_px:.4f}")

    with open(os.path.join(MISSION_DIR, "coverage_mission.json")) as f:
        mission = json.load(f)
    with open(os.path.join(MISSION_DIR,
                          "coverage_mission_flight_log.json")) as f:
        flight_log = json.load(f)

    mission_by_id = {w["waypoint_id"]: w for w in mission["waypoints"]}
    settled = {e["waypoint_id"]: e for e in flight_log["log"]
              if e.get("settled") and not e.get("stuck")
              and not e.get("skipped")}
    settled = {k: v for k, v in settled.items() if k in mission_by_id}
    all_ids = sorted(settled.keys())
    print(f"  waypoints: {len(all_ids)}/82 settled (ALL, not a sample)")

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = RZT.LOOP_W
    scene.render.resolution_y = RZT.LOOP_H
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.use_nodes = False

    cam_data = bpy.data.cameras.get("AVI_ZOOM_CAM")
    if cam_data is None:
        cam_data = bpy.data.cameras.new("AVI_ZOOM_CAM")
    cam_data.clip_start, cam_data.clip_end = 0.01, 400.0
    cam_data.sensor_fit = "HORIZONTAL"
    cam = bpy.data.objects.get("AVI_ZOOM_CAM")
    if cam is None:
        cam = bpy.data.objects.new("AVI_ZOOM_CAM", cam_data)
        bpy.context.scene.collection.objects.link(cam)
    scene.camera = cam

    dg = bpy.context.evaluated_depsgraph_get()
    manifest = []
    n_ray_tested = n_ray_kept = 0
    for wi, wid in enumerate(all_ids):
        wp = mission_by_id[wid]
        e = settled[wid]
        cam_pos = Vector(e["achieved_position_m"])
        target = Vector(wp["target_m"])
        standoff = (cam_pos - target).length
        zoom_hfov, zoom_vfov = RZT._zoom_hfov_vfov_deg(target_mm_px, standoff)
        right, up2, fwd = RZT._camera_frame(cam_pos, target)
        to_target = -fwd
        tile_dirs, n_h, n_v = RZT._tile_directions(
            right, up2, to_target, RZT.HFOV_WIDE_DEG, RZT.VFOV_WIDE_DEG,
            zoom_hfov, zoom_vfov, RZT.TILE_OVERLAP_FRAC)

        cam_data.angle = math.radians(zoom_hfov)
        n_kept_here = 0
        for ih, iv, direction in tile_dirs:
            n_ray_tested += 1
            hit, loc, nrm, idx, ob, mw = scene.ray_cast(
                dg, cam_pos, direction, distance=RZT.MAX_RAY_RANGE_M)
            if (not hit or ob is None or
                    ob.name.startswith(RZT._ENV_SKIP_PREFIX)):
                continue
            n_ray_kept += 1
            n_kept_here += 1
            tile_target = cam_pos + direction * standoff
            RZT._set_camera_pose(cam, cam_pos, direction)
            tile_id = f"{wid}_T{ih:02d}{iv:02d}"
            out_path = os.path.join(ZOOM_RENDER_DIR, f"{tile_id}.png")
            # Resumable: the first attempt was killed by its own timeout
            # at 6,148/~7,200 tiles across 70/82 waypoints. Tile ids are
            # deterministic (same seed, same geometry), so a file that's
            # already on disk from that run is skipped rather than
            # re-rendered -- only the metadata is regenerated (cheap, no
            # bpy.ops.render.render call) so the manifest still ends up
            # covering all 82 waypoints in one pass.
            if not os.path.exists(out_path):
                scene.render.filepath = out_path
                bpy.ops.render.render(write_still=True)
            manifest.append({
                "waypoint_id": wid, "tile_id": tile_id,
                "achieved_position_m": list(cam_pos),
                "tile_direction": list(direction),
                "tile_target_m": list(tile_target),
                "standoff_m": standoff,
                "zoom_hfov_deg": zoom_hfov, "zoom_vfov_deg": zoom_vfov,
                "image_path": os.path.relpath(out_path, ROOT),
                "hit_object": ob.name,
            })
        if (wi + 1) % 10 == 0:
            print(f"  ... {wi + 1}/{len(all_ids)} waypoints done, "
                 f"{n_ray_kept} tiles kept so far "
                 f"({time.time() - t0:.0f}s elapsed)")

    out = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "target_mm_px": target_mm_px,
        "tile_overlap_frac": RZT.TILE_OVERLAP_FRAC,
        "n_waypoints": len(all_ids),
        "waypoints": all_ids,
        "n_tiles_tested": n_ray_tested,
        "n_tiles_kept": n_ray_kept,
        "tiles": manifest,
    }
    out_path = os.path.join(DET_DIR, "AVIAN_zoom_tiles_all_FINAL.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    dt = time.time() - t0
    print(f"  render  : {n_ray_kept}/{n_ray_tested} tiles kept, rendered "
         f"in {dt:.1f}s ({dt / max(1, n_ray_kept):.2f} s/tile)")
    print(f"  saved   : {out_path}")
    print("FINAL_ZOOM_RENDER_ALL_COMPLETE")


if __name__ == "__main__":
    main()

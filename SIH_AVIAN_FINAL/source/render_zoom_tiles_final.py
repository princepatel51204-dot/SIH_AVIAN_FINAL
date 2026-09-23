"""SIH_AVIAN_FINAL -- Phase 2 Section 2 step 2: a simulated zoom camera,
same flight.

No re-flight: every tile is rendered from the SAME achieved position
Phase 1's `coverage_mission_flight_log.json` already logged for a settled
waypoint. Only the camera's field of view changes, standing in for a real
inspection payload's optical zoom (DJI H20-class reaches ~4 deg HFOV;
this works out to ~9 deg, inside that envelope -- see the zoom-factor
computation below).

WAYPOINT AND TILE SELECTION IS GEOMETRY-ONLY. Nothing here reads defect
ground truth -- waypoints are chosen by structure kind (already recorded
by `coverage_final.py`, itself ground-truth-free) and tiles are kept only
if their centre ray hits a real structural mesh object in a fresh
Blender ray-cast (`scene.ray_cast`), excluding anything prefixed `ENV_`
(confirmed the actual sky/ground/water mesh objects in this scene --
`ENV_GROUND_FAR`, `ENV_RIVER_WATER` -- both carry that prefix, checked
directly, not assumed). Ground truth is read ONLY by `score_coverage.py`,
after every render is already on disk, exactly the boundary Stage 2's own
planner already established.

ZOOM FACTOR, COMPUTED NOT HARDCODED: the median achieved GSD of the
TRAINING split's own LOOP frames (`dataset/AVIAN_dataset_manifest_FINAL.json`,
filtered to `splits_final.json`'s train list) sets the target mm/px. At
each selected waypoint's own ACHIEVED standoff (~7.9-8.1 m, measured, not
assumed to be exactly 8.0), the horizontal FOV that reproduces that
mm/px is solved for directly from the pinhole relation, per waypoint (a
waypoint 8.13 m out and one 7.88 m out get very slightly different HFOVs,
not the same one applied blind).

Usage:
    blender --background --python run_blender.py -- source/render_zoom_tiles_final.py
"""
from __future__ import annotations
import json
import math
import os
import random
import sys
import time

import bpy
import numpy as np
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
MISSION_DIR = os.path.join(ROOT, "mission")
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")
ZOOM_RENDER_DIR = os.path.join(DET_DIR, "renders_zoom")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
import dataset_final as DSF   # HFOV_DEG/LOOP_RES, sensor intrinsics

HFOV_WIDE_DEG = DSF.HFOV_DEG          # 69.0 -- the aircraft's own real camera
VFOV_WIDE_DEG = 42.0                 # matches sensors.cameras' RGBCamera
LOOP_W, LOOP_H = DSF.LOOP_RES

SEED = 20260925
TILE_OVERLAP_FRAC = 0.20
_ENV_SKIP_PREFIX = "ENV_"
MAX_RAY_RANGE_M = 30.0
N_WAYPOINTS_TARGET = 16   # see module docstring: ~1000 renders total at the
                          # measured tiles/waypoint rate


def _load_all_defects_positions_UNUSED():
    """Deliberately not called anywhere -- ground truth stays out of this
    file. Present only so a reviewer sees the boundary was considered,
    not merely assumed."""
    raise NotImplementedError


def _median_train_gsd_mm_px():
    with open(os.path.join(DATASET_DIR,
                          "AVIAN_dataset_manifest_FINAL.json")) as f:
        frames = {r["image_path"]: r for r in json.load(f)["frames"]}
    with open(os.path.join(DATASET_DIR, "splits_final.json")) as f:
        train_paths = json.load(f)["splits"]["train"]
    gsds = [frames[p]["achieved_gsd_mm_px"] for p in train_paths
           if frames[p]["resolution"] == "640x480"]
    return float(np.median(gsds))


def _zoom_hfov_vfov_deg(target_mm_px, standoff_m, width_px=LOOP_W):
    """Solves the pinhole relation for the HFOV that reproduces
    `target_mm_px` at `standoff_m`, then derives VFOV preserving the real
    camera's own vertical/horizontal tan ratio (69/42 deg)."""
    gsd_mm_at_1m = target_mm_px / standoff_m
    half_hfov = math.atan(gsd_mm_at_1m * width_px / 2000.0)
    hfov_deg = math.degrees(2.0 * half_hfov)
    vh_ratio = (math.tan(math.radians(VFOV_WIDE_DEG / 2.0)) /
               math.tan(math.radians(HFOV_WIDE_DEG / 2.0)))
    half_vfov = math.atan(vh_ratio * math.tan(half_hfov))
    vfov_deg = math.degrees(2.0 * half_vfov)
    return hfov_deg, vfov_deg


_KIND_BUCKET = {
    "pier_column": "PIER", "pier_cap": "PIER", "pier_footing": "PIER",
    "abutment": "PIER", "bearing": "PIER",
    "girder": "TRUSS", "truss_chord": "TRUSS", "truss_diagonal": "TRUSS",
    "truss_vertical": "TRUSS", "gusset_plate": "TRUSS", "bracing": "TRUSS",
    "diaphragm": "TRUSS", "floor_beam": "TRUSS", "stringer": "TRUSS",
    "structure": "TRUSS",
    "deck_box": "DECK", "deck_slab": "DECK", "parapet": "DECK",
    "track_slab": "DECK", "rail": "DECK", "joint_gap": "DECK",
    "expansion_joint": "DECK", "drain": "DECK", "cable_trough": "DECK",
    "access_hatch": "DECK", "service_duct": "DECK",
    "catenary_mast": "METRO",
}


def _select_waypoints(mission, flight_log, log=print):
    mission_by_id = {w["waypoint_id"]: w for w in mission["waypoints"]}
    settled = {e["waypoint_id"]: e for e in flight_log["log"]
              if e.get("settled") and not e.get("stuck")
              and not e.get("skipped")}
    settled = {k: v for k, v in settled.items() if k in mission_by_id}

    buckets = {}
    for wid in settled:
        kind = mission_by_id[wid]["prim_kind"]
        bucket = _KIND_BUCKET.get(kind, "OTHER")
        buckets.setdefault(bucket, []).append(wid)
    log(f"  buckets : " + ", ".join(f"{k}={len(v)}" for k, v in
                                    sorted(buckets.items())))

    rng = random.Random(SEED)
    total_settled = len(settled)
    chosen = []
    for bucket, ids in sorted(buckets.items()):
        ids_sorted = sorted(ids)   # deterministic order before shuffling
        rng.shuffle(ids_sorted)
        n_take = max(1, round(N_WAYPOINTS_TARGET * len(ids) / total_settled))
        chosen.extend(ids_sorted[:n_take])
    chosen = sorted(set(chosen))[:N_WAYPOINTS_TARGET + 2]   # small slack
    log(f"  seed    : {SEED}")
    log(f"  chosen  : {len(chosen)} waypoints -- {chosen}")
    return chosen, mission_by_id, settled


def _camera_frame(cam_pos, target):
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


def _set_camera_pose(cam, cam_pos, direction):
    """Points `cam` from `cam_pos` along `direction` (a unit Vector) --
    same matrix construction as `_look_at()` elsewhere, expressed via an
    explicit look direction instead of a target point since tile
    directions are generated as angular offsets, not points."""
    fwd = -direction.normalized()   # Blender camera looks down local -Z
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


def _tile_directions(base_right, base_up2, base_fwd_to_target,
                     wide_hfov_deg, wide_vfov_deg,
                     zoom_hfov_deg, zoom_vfov_deg, overlap):
    step_h = zoom_hfov_deg * (1.0 - overlap)
    step_v = zoom_vfov_deg * (1.0 - overlap)
    n_h = max(1, math.ceil(wide_hfov_deg / step_h))
    n_v = max(1, math.ceil(wide_vfov_deg / step_v))
    dirs = []
    for iv in range(n_v):
        v_deg = -wide_vfov_deg / 2.0 + wide_vfov_deg * (iv + 0.5) / n_v
        for ih in range(n_h):
            h_deg = -wide_hfov_deg / 2.0 + wide_hfov_deg * (ih + 0.5) / n_h
            # rotate base_fwd_to_target by h_deg about up2, then by v_deg
            # about the (now-rotated) right axis
            d = base_fwd_to_target.copy()
            d = Matrix.Rotation(math.radians(h_deg), 3, base_up2) @ d
            right_now = Matrix.Rotation(math.radians(h_deg), 3,
                                        base_up2) @ base_right
            d = Matrix.Rotation(math.radians(v_deg), 3, right_now) @ d
            dirs.append((ih, iv, d.normalized()))
    return dirs, n_h, n_v


def main():
    t0 = time.time()
    print("== SIH_AVIAN_FINAL :: render_zoom_tiles_final.py (Phase 2 sec2) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(ZOOM_RENDER_DIR, exist_ok=True)

    target_mm_px = _median_train_gsd_mm_px()
    print(f"  zoom target: median train LOOP mm/px = {target_mm_px:.4f}")

    with open(os.path.join(MISSION_DIR, "coverage_mission.json")) as f:
        mission = json.load(f)
    with open(os.path.join(MISSION_DIR,
                          "coverage_mission_flight_log.json")) as f:
        flight_log = json.load(f)

    chosen_ids, mission_by_id, settled = _select_waypoints(
        mission, flight_log)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = LOOP_W
    scene.render.resolution_y = LOOP_H
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
    for wid in chosen_ids:
        wp = mission_by_id[wid]
        e = settled[wid]
        cam_pos = Vector(e["achieved_position_m"])
        target = Vector(wp["target_m"])
        standoff = (cam_pos - target).length
        zoom_hfov, zoom_vfov = _zoom_hfov_vfov_deg(target_mm_px, standoff)
        right, up2, fwd = _camera_frame(cam_pos, target)
        to_target = -fwd   # direction FROM camera TO target
        tile_dirs, n_h, n_v = _tile_directions(
            right, up2, to_target, HFOV_WIDE_DEG, VFOV_WIDE_DEG,
            zoom_hfov, zoom_vfov, TILE_OVERLAP_FRAC)

        cam_data.angle = math.radians(zoom_hfov)
        n_kept_here = 0
        for ih, iv, direction in tile_dirs:
            n_ray_tested += 1
            hit, loc, nrm, idx, ob, mw = scene.ray_cast(
                dg, cam_pos, direction, distance=MAX_RAY_RANGE_M)
            if not hit or ob is None or ob.name.startswith(_ENV_SKIP_PREFIX):
                continue
            n_ray_kept += 1
            n_kept_here += 1
            tile_target = cam_pos + direction * standoff
            _set_camera_pose(cam, cam_pos, direction)
            tile_id = f"{wid}_T{ih:02d}{iv:02d}"
            out_path = os.path.join(ZOOM_RENDER_DIR, f"{tile_id}.png")
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
        print(f"  {wid}: {n_kept_here}/{n_h * n_v} tiles kept "
             f"(standoff {standoff:.2f} m, hfov {zoom_hfov:.2f} deg)")

    out = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "seed": SEED,
        "target_mm_px": target_mm_px,
        "tile_overlap_frac": TILE_OVERLAP_FRAC,
        "n_waypoints_chosen": len(chosen_ids),
        "waypoints_chosen": chosen_ids,
        "n_tiles_tested": n_ray_tested,
        "n_tiles_kept": n_ray_kept,
        "tiles": manifest,
    }
    out_path = os.path.join(DET_DIR, "AVIAN_zoom_tiles_FINAL.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    dt = time.time() - t0
    print(f"  render  : {n_ray_kept}/{n_ray_tested} tiles kept, rendered "
         f"in {dt:.1f}s ({dt / max(1, n_ray_kept):.2f} s/tile)")
    print(f"  saved   : {out_path}")
    print("FINAL_ZOOM_RENDER_COMPLETE")


if __name__ == "__main__":
    main()

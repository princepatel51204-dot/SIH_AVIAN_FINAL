"""Render the picture-in-picture camera feed from the recorded track.

Uses PyBullet's OWN renderer (RGBCamera.read() / DepthCamera.read(), the
exact sensor classes Phase 1 verified) at each recorded base pose -- this is
NOT a Blender look-alike. A fresh AvianVehicle is posed frame-by-frame from
avian_demo_track.json (resetBasePositionAndOrientation + resetJointState,
never re-simulated) so the sensor sees exactly the geometry the flight
recorder saw.

Output: media/frames_cam/frame_?????.png, one PIP composite (RGB main panel
+ depth thumbnail) per track frame, at the resolution declared in
CAM_INSET_SIZE.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
from PIL import Image
from matplotlib import colormaps

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
os.environ.setdefault("AVIAN_CAD_DIR", os.path.join(ROOT, "cad"))
for p in (ROOT, os.environ["AVIAN_CAD_DIR"]):
    if p not in sys.path:
        sys.path.insert(0, p)

import pybullet as pb                                   # noqa: E402
from simulation.world import AvianWorld                  # noqa: E402

TRACK_PATH = os.path.join(HERE, "avian_demo_track.json")
OUT_DIR = os.path.join(HERE, "frames_cam")

RGB_MAIN_W = 480          # ~25% of a 1920-wide frame
DEPTH_THUMB_W = 168       # ~35% of the inset width
BORDER = 4
CMAP = colormaps["turbo"]


def colorize_depth(depth_m, near, far):
    valid = ~np.isnan(depth_m)
    norm = np.clip((depth_m - near) / max(far - near, 1e-6), 0.0, 1.0)
    norm = np.nan_to_num(norm, nan=0.0)
    rgb = (CMAP(norm)[:, :, :3] * 255).astype(np.uint8)
    rgb[~valid] = (12, 12, 16)
    return rgb


def compose_inset(rgb_img, depth_img):
    h, w = rgb_img.shape[:2]
    scale = RGB_MAIN_W / float(w)
    main = Image.fromarray(rgb_img).resize(
        (RGB_MAIN_W, int(round(h * scale))), Image.BILINEAR)

    dh, dw = depth_img.shape[:2]
    dscale = DEPTH_THUMB_W / float(dw)
    thumb = Image.fromarray(depth_img).resize(
        (DEPTH_THUMB_W, int(round(dh * dscale))), Image.NEAREST)

    canvas = Image.new("RGB", main.size, (0, 0, 0))
    canvas.paste(main, (0, 0))
    tx = canvas.width - thumb.width - BORDER
    ty = canvas.height - thumb.height - BORDER
    frame_border = Image.new(
        "RGB", (thumb.width + 2 * BORDER // 2, thumb.height + 2 * BORDER // 2),
        (230, 230, 230))
    canvas.paste(frame_border, (tx - BORDER // 2, ty - BORDER // 2))
    canvas.paste(thumb, (tx, ty))
    return canvas


def main():
    track = json.load(open(TRACK_PATH))
    frames = track["frames"]
    os.makedirs(OUT_DIR, exist_ok=True)

    world = AvianWorld(gui=False, load_bridge=False)
    v, = world.spawn_fleet(1, positions=[(0.0, 0.0, 3.0)])
    sensors = v.attach_sensors(seed=0)
    rgb_cam = sensors["rgb_down"]
    depth_cam = sensors["depth"]

    n = len(frames)
    for i, fr in enumerate(frames):
        pb.resetBasePositionAndOrientation(
            v.body, fr["base_com_position"], fr["base_com_quaternion"])
        for ji, q in zip(v.arm_joints, fr["arm_joint_rad"]):
            pb.resetJointState(v.body, ji, q)
        for li, theta in zip(v.rotor_link, fr["rotor_angle_rad"]):
            pb.resetJointState(v.body, li, theta)

        rgb_env = rgb_cam.read(fr["t"], capture=False)
        depth_env = depth_cam.read(fr["t"])
        depth_rgb = colorize_depth(depth_env["depth_m"], depth_cam.min_range,
                                   depth_cam.max_range)

        canvas = compose_inset(rgb_env["image"], depth_rgb)
        canvas.save(os.path.join(OUT_DIR, f"frame_{i:05d}.png"))

        if i % 50 == 0 or i == n - 1:
            print(f"camera feed {i + 1}/{n}", flush=True)

    world.close()
    print(f"wrote {n} composite frames to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

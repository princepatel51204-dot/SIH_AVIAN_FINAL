"""Drive the drone and both cameras from the REAL flown trajectory.

Source: gazebo/mission_follower/results/full_pass_05/pose_audit_track.csv --
160,053 rows at 20 Hz of true base_link pose (x, y, z + quaternion) and the
true gimbal joint angle, recorded during the 1161/1162-waypoint pass.

The onboard camera pose reproduces measure_flown.py exactly:

    R   = quat_to_matrix(qw, qx, qy, qz)
    Rc  = R @ ry(gimbal_joint_rad)
    cam = p + R @ BOOM + Rc @ SENSOR      BOOM (0.35, 0, 0.05), SENSOR (0.03, 0, 0)

with Rc's columns being the ROS camera axes (forward, left, up). Blender's
camera looks down -Z with +Y up, so the basis is re-expressed as

    blender_X (right) = -Rc[:,1]      ROS +Y is left
    blender_Y (up)    =  Rc[:,2]
    blender_Z (back)  = -Rc[:,0]      ROS +X is forward

The drone body itself needs no conversion: Gazebo ENU and Blender are both
right-handed Z-up, so the quaternion applies directly.

CHASE is not flight data -- it is a camera flown alongside the real trajectory:
it sits at a body-frame offset from the true pose and is then boxcar-smoothed
over a window, which is what produces cinematic lag. The drone it follows is on
the real path; the chase camera's own path is derived, and the film report says
so.
"""
import bpy
import csv
import math
import os

import numpy as np
from mathutils import Matrix, Quaternion, Vector

TRACK = ("/home/prince/avian_rev_c/SIH_AVIAN_FINAL/gazebo/mission_follower/"
         "results/full_pass_05/pose_audit_track.csv")
BOOM = np.array([0.35, 0.0, 0.05])
SENSOR = np.array([0.03, 0.0, 0.0])
MAX_GAP_S = 0.25

_TRACK_CACHE = None


def load_track(path=TRACK):
    """-> ndarray [t_sim, t_wall, x, y, z, qw, qx, qy, qz, gimbal_rad], t-sorted."""
    global _TRACK_CACHE
    if _TRACK_CACHE is None:
        a = np.genfromtxt(path, delimiter=",", skip_header=1)
        a = a[np.argsort(a[:, 0])]
        _TRACK_CACHE = a[np.isfinite(a).all(axis=1)]
    return _TRACK_CACHE


def qmat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def sample(track, stamps):
    """Interpolate the track at each sim-time stamp. None where the gap is too big."""
    t = track[:, 0]
    out = []
    for s in stamps:
        k = int(np.searchsorted(t, s))
        if k <= 0 or k >= len(t) or (t[k] - t[k - 1]) > MAX_GAP_S:
            out.append(None)
            continue
        a, b = track[k - 1], track[k]
        w = (s - a[0]) / max(b[0] - a[0], 1e-9)
        p = a[2:5] + w * (b[2:5] - a[2:5])
        qa, qb = a[5:9], b[5:9]
        q = qa + w * (qb * np.sign(np.dot(qa, qb)) - qa)
        q /= max(np.linalg.norm(q), 1e-12)
        jnt = a[9] + w * (b[9] - a[9])
        R = qmat(*q)
        Rc = R @ ry(jnt)
        cam = p + R @ BOOM + Rc @ SENSOR
        out.append({"t": float(s), "p": p, "q": q, "R": R, "Rc": Rc,
                    "cam": cam, "gimbal": float(jnt)})
    return out


def _cam_matrix(Rc, pos):
    """ROS camera axes -> a Blender camera world matrix."""
    bx = -Rc[:, 1]
    by = Rc[:, 2]
    bz = -Rc[:, 0]
    m = Matrix(((bx[0], by[0], bz[0], pos[0]),
                (bx[1], by[1], bz[1], pos[1]),
                (bx[2], by[2], bz[2], pos[2]),
                (0.0, 0.0, 0.0, 1.0)))
    return m


def _new_camera(name, lens, scene):
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start = 0.02
    cd.clip_end = 2000.0
    ob = bpy.data.objects.new(name, cd)
    scene.collection.objects.link(ob)
    return ob


def _key_obj(ob, frame, loc=None, quat=None, matrix=None):
    if matrix is not None:
        ob.matrix_world = matrix
        ob.keyframe_insert("location", frame=frame)
        ob.rotation_mode = "QUATERNION"
        ob.keyframe_insert("rotation_quaternion", frame=frame)
        return
    if loc is not None:
        ob.location = Vector(loc)
        ob.keyframe_insert("location", frame=frame)
    if quat is not None:
        ob.rotation_mode = "QUATERNION"
        ob.rotation_quaternion = Quaternion(quat)
        ob.keyframe_insert("rotation_quaternion", frame=frame)


def _boxcar(arr, win):
    if win <= 1:
        return arr
    pad = win // 2
    padded = np.pad(arr, ((pad, pad), (0, 0)), mode="edge")
    k = np.ones(win) / win
    return np.stack([np.convolve(padded[:, i], k, mode="valid")[:len(arr)]
                     for i in range(arr.shape[1])], axis=1)


def build_shot(scene, t0, t1, frame_start, fps=24, drone_root="DRONE_ROOT",
               chase_offset=(-3.2, 0.0, 1.1), chase_smooth=13, onboard_lens=24.0,
               chase_lens=35.0):
    """Key the drone + both cameras over sim-time [t0, t1].

    Returns a dict of what was actually keyed, including how many frames had no
    usable track data.
    """
    track = load_track()
    n = int(round((t1 - t0) * fps))
    stamps = t0 + np.arange(n) / fps
    S = sample(track, stamps)
    good = [s for s in S if s is not None]
    if len(good) < 2:
        return {"error": "no usable track data", "t0": t0, "t1": t1,
                "frames": n, "usable": len(good)}

    onboard = _new_camera("CAM_ONBOARD", onboard_lens, scene)
    chase = _new_camera("CAM_CHASE", chase_lens, scene)
    root = bpy.data.objects.get(drone_root)

    # chase path: body-frame offset from the true pose, then smoothed
    raw = np.array([s["p"] + s["R"] @ np.array(chase_offset) for s in good])
    look = np.array([s["p"] for s in good])
    smooth = _boxcar(raw, chase_smooth)
    look_s = _boxcar(look, max(3, chase_smooth // 2))

    fi = frame_start
    kept = 0
    for idx, s in enumerate(S):
        if s is None:
            fi += 1
            continue
        j = min(kept, len(smooth) - 1)
        if root is not None:
            _key_obj(root, fi, loc=s["p"], quat=(s["q"][0], s["q"][1], s["q"][2], s["q"][3]))
        _key_obj(onboard, fi, matrix=_cam_matrix(s["Rc"], s["cam"]))

        cpos = Vector(smooth[j])
        target = Vector(look_s[j])
        d = (target - cpos)
        if d.length < 1e-6:
            d = Vector((1, 0, 0))
        quat = d.to_track_quat("-Z", "Y")
        _key_obj(chase, fi, loc=cpos, quat=quat)

        kept += 1
        fi += 1

    for ob in (onboard, chase, root):
        if ob is None or not ob.animation_data:
            continue
        for fc in ob.animation_data.action.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = "LINEAR"

    return {
        "t0": round(t0, 3), "t1": round(t1, 3), "fps": fps,
        "frames": n, "keyed": kept, "dropped_no_track": n - kept,
        "frame_range": [frame_start, frame_start + n - 1],
        "onboard_lens_mm": onboard_lens, "chase_lens_mm": chase_lens,
        "chase_offset_body_m": list(chase_offset), "chase_smooth_frames": chase_smooth,
        "start_pos": [round(float(v), 3) for v in good[0]["p"]],
        "end_pos": [round(float(v), 3) for v in good[-1]["p"]],
        "path_len_m": round(float(np.sum(np.linalg.norm(np.diff(look, axis=0), axis=1))), 2),
        "gimbal_deg": [round(math.degrees(good[0]["gimbal"]), 2),
                       round(math.degrees(good[-1]["gimbal"]), 2)],
    }

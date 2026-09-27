"""D01-D07 acceptance checks for the Phase 1 demo video.

Prints every check's result, PASS or FAIL, and never adjusts a threshold to
make one pass. Run after generate_track.py, render_camera_feed.py,
render_captions.py and compose_video.py have all completed.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
os.environ.setdefault("AVIAN_CAD_DIR", os.path.join(ROOT, "cad"))
for p in (ROOT, os.environ["AVIAN_CAD_DIR"]):
    if p not in sys.path:
        sys.path.insert(0, p)

TRACK_PATH = os.path.join(HERE, "avian_demo_track.json")
VIDEO_PATH = os.path.join(HERE, "avian_demo.mp4")
README_PATH = os.path.join(HERE, "README.md")

RESULTS = []


def check(cid, name, ok, detail=""):
    RESULTS.append((cid, name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}  {name:<55} {detail}")
    return ok


def d01_frames_differ(track):
    tmp_dir = "/tmp/avian_demo_d01_frames"
    os.makedirs(tmp_dir, exist_ok=True)
    n = track["n_frames"]
    fps = track["fps"]
    sample_times = [0.5, n / 2 / fps, (n - 2) / fps]
    stds = []
    for t in sample_times:
        out_a = os.path.join(tmp_dir, "a.png")
        out_b = os.path.join(tmp_dir, "b.png")
        subprocess.run(["ffmpeg", "-y", "-ss", str(t), "-i", VIDEO_PATH,
                       "-frames:v", "1", out_a],
                      capture_output=True, check=True)
        subprocess.run(["ffmpeg", "-y", "-ss", str(t + 1.0 / fps),
                       "-i", VIDEO_PATH, "-frames:v", "1", out_b],
                      capture_output=True, check=True)
        a = np.asarray(Image.open(out_a).convert("RGB"), dtype=np.float64)
        b = np.asarray(Image.open(out_b).convert("RGB"), dtype=np.float64)
        stds.append(float(np.std(a - b)))
    ok = all(s > 0.0 for s in stds)
    return check("D01", "every sampled frame differs from its neighbour", ok,
                f"stddevs {[round(s, 3) for s in stds]}")


def d02_within_limits(track):
    bad = [f["frame"] for f in track["frames"]
          if f["arm_check"] is not None
          and not f["arm_check"]["within_limits"]]
    ok = len(bad) == 0
    n_checked = sum(1 for f in track["frames"] if f["arm_check"] is not None)
    return check("D02", "arm trajectory never exceeds a joint limit", ok,
                f"{n_checked} keyframes checked, {len(bad)} bad")


def d03_collision_free(track):
    bad_self = [f["frame"] for f in track["frames"]
               if f["arm_check"] is not None
               and not f["arm_check"]["self_collision_free"]]
    bad_env = [f["frame"] for f in track["frames"]
              if f["arm_check"] is not None
              and not f["arm_check"]["environment_collision_free"]]
    ok = not bad_self and not bad_env
    n_checked = sum(1 for f in track["frames"] if f["arm_check"] is not None)
    return check("D03", "arm trajectory is collision-free at every keyframe",
                ok, f"{n_checked} keyframes, {len(bad_self)} self-collision, "
                   f"{len(bad_env)} environment-collision")


def d04_rotors_monotonic(track):
    angles = np.array([f["rotor_angle_rad"] for f in track["frames"]])
    ok = True
    detail = []
    for k in range(angles.shape[1]):
        d = np.diff(angles[:, k])
        mono = bool(np.all(d > 0) or np.all(d < 0))
        ok = ok and mono
        detail.append(mono)
    return check("D04", "rotors rotate continuously (strictly monotonic)",
                ok, f"per-rotor monotonic: {detail}")


def d05_camera_is_real(track):
    import pybullet as pb
    from simulation.world import AvianWorld
    import render_camera_feed as R

    frames = track["frames"]
    spot_indices = [2, 4, 6]
    all_match = True
    details = []
    for idx in spot_indices:
        world = AvianWorld(gui=False, load_bridge=False)
        v, = world.spawn_fleet(1, positions=[(0.0, 0.0, 3.0)])
        sensors = v.attach_sensors(seed=0)
        rgb_cam, depth_cam = sensors["rgb_down"], sensors["depth"]
        for i in range(idx + 1):
            fr = frames[i]
            pb.resetBasePositionAndOrientation(
                v.body, fr["base_com_position"], fr["base_com_quaternion"])
            for ji, q in zip(v.arm_joints, fr["arm_joint_rad"]):
                pb.resetJointState(v.body, ji, q)
            for li, theta in zip(v.rotor_link, fr["rotor_angle_rad"]):
                pb.resetJointState(v.body, li, theta)
            rgb_env = rgb_cam.read(fr["t"], capture=False)
            depth_env = depth_cam.read(fr["t"])
        depth_rgb = R.colorize_depth(depth_env["depth_m"], depth_cam.min_range,
                                     depth_cam.max_range)
        expected = R.compose_inset(rgb_env["image"], depth_rgb)
        actual = Image.open(os.path.join(HERE, "frames_cam",
                                         f"frame_{idx:05d}.png")).convert("RGB")
        diff = np.asarray(expected) - np.asarray(actual)
        match = bool(np.abs(diff).max() == 0)
        all_match = all_match and match
        details.append((idx, match, int(np.abs(diff).max())))
        world.close()
    return check("D05", "camera inset matches a standalone sensor read()",
                all_match, f"frames {details}")


def d06_video_valid(track):
    r = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries",
         "stream=width,height,codec_name,r_frame_rate,duration",
         "-of", "json", VIDEO_PATH],
        capture_output=True, text=True, check=True)
    info = json.loads(r.stdout)["streams"][0]
    w, h = info["width"], info["height"]
    codec = info["codec_name"]
    num, den = info["r_frame_rate"].split("/")
    fps = float(num) / float(den)
    dur = float(info.get("duration", 0.0))
    ok = (w == 1920 and h == 1080 and codec == "h264"
         and abs(fps - track["fps"]) < 0.5 and 20.0 <= dur <= 60.0)
    return check("D06", "video file is valid H.264 1920x1080 ~30fps", ok,
                f"{w}x{h} {codec} {fps:.2f}fps {dur:.1f}s")


def d07_labelled(track):
    readme = open(README_PATH).read() if os.path.exists(README_PATH) else ""
    captions = " ".join(f.get("caption", "") or "" for f in track["frames"])
    checks = {
        "panel labelled synthetic (not a defect)":
            "synthetic repair target" in track["panel"]["label"],
        "rotor spin rate labelled illustrative in README":
            "illustrative" in readme.lower() and "rotor" in readme.lower(),
        "repair cue labelled ILLUSTRATIVE on screen":
            "ILLUSTRATIVE" in captions,
        "README states which parts are simulated vs illustrative":
            "simulated" in readme.lower() and "illustrative" in readme.lower(),
    }
    ok = all(checks.values())
    return check("D07", "illustrative vs simulated is labelled", ok,
                str(checks))


def main():
    track = json.load(open(TRACK_PATH))
    print("AVIAN PHASE 1 DEMO VIDEO -- ACCEPTANCE CHECKS")
    d02_within_limits(track)
    d03_collision_free(track)
    d04_rotors_monotonic(track)
    d07_labelled(track)
    if os.path.exists(VIDEO_PATH):
        d01_frames_differ(track)
        d06_video_valid(track)
    else:
        check("D01", "every sampled frame differs from its neighbour", False,
             "SKIPPED: avian_demo.mp4 not found")
        check("D06", "video file is valid H.264 1920x1080 ~30fps", False,
             "SKIPPED: avian_demo.mp4 not found")
    if os.path.isdir(os.path.join(HERE, "frames_cam")):
        d05_camera_is_real(track)
    else:
        check("D05", "camera inset matches a standalone sensor read()", False,
             "SKIPPED: frames_cam/ not found")

    n_pass = sum(1 for *_, ok, _ in RESULTS if ok)
    n_total = len(RESULTS)
    print(f"\n{n_pass}/{n_total} checks passed")
    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())

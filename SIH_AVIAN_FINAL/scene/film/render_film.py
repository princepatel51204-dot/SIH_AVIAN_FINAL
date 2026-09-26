"""Render the inspection film's frames. Resumable; writes PNGs, no video.

Frames only, deliberately: the detector has to run on the real rendered pixels
and the UI is composited afterwards, so encoding happens at the very end from
finished frames (compose_film.py), never here.

Two shot types:

  beat   onboard view, drone hidden. For a `real` beat the camera occupies
         positions the drone actually held during full_pass_05, sampled from
         pose_audit_track.csv across a window centred on the closest approach;
         aim and zoom are synthesized. For a `staged` beat the camera flies a
         purpose-built approach along the defect's own recommended view
         direction, and never comes closer than the shot's declared standoff,
         which is itself >= MIN_LEGAL_STANDOFF.

  wide   chase view, drone visible, riding the real trajectory with a smoothed
         body-frame offset.

Zoom follows the same fit rule used to cast the shots: lens = (35/1.5) * range,
which reproduces the angular size the detector was shown when it was asked
whether each defect was visible at all. Lens is keyframed so motion blur sees
the zoom as motion.

Env:  ONLY=beat3,wide_open   restrict to named shots
      TAA=16  RES=100
"""
import bpy, sys, os, json, time, math

import numpy as np
from mathutils import Vector

sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/film")
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/cine")
import cine_setup as cs
import drone as D
import rigs as R
import shotlist as SL

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
OUT = f"{ROOT}/media/_film"
TAA = int(os.environ.get("TAA", "16"))
RES = int(os.environ.get("RES", "100"))
ONLY = [s for s in os.environ.get("ONLY", "").split(",") if s]

FIT_K = 35.0 / 1.5          # lens-per-metre that matches the casting framing
DOF_FSTOP = 3.5
MIN_LENS, MAX_LENS = 18.0, 400.0

os.makedirs(OUT, exist_ok=True)
scene = bpy.context.scene
cs.setup_look(scene, taa=TAA, motion_blur=True)
cs.setup_output(scene, res_pct=RES)
scene.eevee.use_bokeh_jittered = False
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGB"

drep = D.import_drone(scene, spin=True, frame_start=1, frame_end=6000)
DRONE = bpy.data.collections[D.COLL]

gt = {d["defect_id"]: d
      for d in json.load(open(f"{ROOT}/scene/AVIAN_defect_ground_truth_FINAL.json"))["defects"]}
track = R.load_track()


def new_cam(name, lens):
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start = 0.02
    cd.clip_end = 2000.0
    ob = bpy.data.objects.new(name, cd)
    scene.collection.objects.link(ob)
    return ob


def fit_lens(rng):
    return max(MIN_LENS, min(MAX_LENS, FIT_K * rng))


def beat_camera_path(beat):
    """-> list of (position, target, lens) for every frame of the beat."""
    d = gt[beat["defect"]]
    tgt = Vector(d["position_m"])
    n = SL.BEAT_FRAMES
    path = []

    if beat["kind"] == "real":
        half = (n / 2) / SL.FPS
        stamps = beat["t"] - half + np.arange(n) / SL.FPS
        S = R.sample(track, stamps)
        last = None
        for s in S:
            if s is None:
                s = last
            last = s
            if s is None:
                path.append(None)
                continue
            pos = Vector([float(v) for v in s["cam"]])
            path.append((pos, tgt, fit_lens((tgt - pos).length)))
        # backfill any leading gap
        first = next((p for p in path if p), None)
        path = [p if p else first for p in path]
    else:
        vd = Vector(d.get("recommended_view_direction") or d.get("surface_normal") or (0, 1, 0))
        if vd.length < 1e-6:
            vd = Vector((0, 1, 0))
        vd.normalize()
        near = float(beat["standoff"])
        far = near * 1.7
        for i in range(n):
            # ease in over the approach, hold, then ease out on the peel
            if i < SL.APPROACH:
                u = i / max(1, SL.APPROACH - 1)
                k = 1 - (1 - u) ** 3
                r = far + (near - far) * k
            elif i < SL.APPROACH + SL.HOLD + SL.CARD:
                r = near
            else:
                u = (i - SL.APPROACH - SL.HOLD - SL.CARD) / max(1, SL.PEEL - 1)
                r = near + (far - near) * 0.35 * (u ** 2)
            r = max(r, SL.MIN_LEGAL_STANDOFF)
            path.append((tgt + vd * r, tgt, fit_lens(r)))
    return path


def render_beat(beat):
    tag = f"beat{beat['n']}_{beat['defect']}"
    if ONLY and not any(o in tag for o in ONLY):
        return None
    dirp = f"{OUT}/{tag}"
    os.makedirs(dirp, exist_ok=True)
    path = beat_camera_path(beat)
    cam = new_cam("CAM_BEAT", 50.0)
    cam.data.dof.use_dof = True
    cam.data.dof.aperture_fstop = DOF_FSTOP
    cam.data.dof.aperture_blades = 7
    scene.camera = cam
    DRONE.hide_render = True

    # keyframe the whole move so EEVEE's motion blur has real motion vectors
    for i, p in enumerate(path):
        pos, tgt, lens = p
        cam.location = pos
        cam.rotation_mode = "QUATERNION"
        cam.rotation_quaternion = (tgt - pos).to_track_quat("-Z", "Y")
        cam.data.dof.focus_distance = max(0.1, (tgt - pos).length)
        cam.data.lens = lens
        f = i + 1
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_quaternion", frame=f)
        cam.data.keyframe_insert("lens", frame=f)
        cam.data.dof.keyframe_insert("focus_distance", frame=f)

    ranges = [float((t - p).length) for p, t, _ in path]
    meta = {"shot": tag, "kind": beat["kind"], "defect": beat["defect"],
            "frames": len(path), "lens_mm": [round(p[2], 1) for p in path[::24]],
            "range_m_min": round(min(ranges), 3), "range_m_max": round(max(ranges), 3),
            "expected_conf": beat["conf_at_range"], "surface": beat["surface"],
            "section": beat["section"],
            "cam_xyz_first": [round(v, 3) for v in path[0][0]],
            "cam_xyz_last": [round(v, 3) for v in path[-1][0]],
            "defect_xyz": [round(v, 3) for v in gt[beat['defect']]["position_m"]],
            "dof_fstop": DOF_FSTOP}
    if beat["kind"] == "real":
        meta["t_sim_window"] = [round(beat["t"] - (len(path) / 2) / SL.FPS, 2),
                                round(beat["t"] + (len(path) / 2) / SL.FPS, 2)]
    else:
        meta["declared_standoff_m"] = beat["standoff"]
        meta["cam_clearance_m"] = beat.get("cam_clearance")

    t0 = time.time()
    done = 0
    for i in range(len(path)):
        fp = f"{dirp}/f{i:04d}.png"
        if os.path.exists(fp) and os.path.getsize(fp) > 20000:
            continue
        scene.frame_set(i + 1)
        scene.render.filepath = fp
        bpy.ops.render.render(write_still=True)
        done += 1
        if done % 24 == 0:
            print(f"    {tag} {i+1}/{len(path)}  {(time.time()-t0)/done:.2f}s/f", flush=True)
    meta["render_s"] = round(time.time() - t0, 1)
    meta["rendered_now"] = done
    meta["min_range_m"] = round(min(ranges), 3)
    json.dump(meta, open(f"{dirp}/meta.json", "w"), indent=1)
    print(f"<<< {tag}  {done} frames  {meta['render_s']}s  "
          f"range {meta['range_m_min']}-{meta['range_m_max']} m", flush=True)
    return meta


def render_wide(w):
    tag = f"wide_{w['name']}"
    if ONLY and not any(o in tag for o in ONLY):
        return None
    dirp = f"{OUT}/{tag}"
    os.makedirs(dirp, exist_ok=True)
    n = w["frames"]
    t0s = w["t"] - (n / 2) / SL.FPS
    shot = R.build_shot(scene, t0s, t0s + n / SL.FPS, frame_start=1,
                        chase_offset=w["chase_offset"], chase_lens=w["lens"],
                        chase_smooth=15)
    cam = bpy.data.objects["CAM_CHASE"]
    cam.data.dof.use_dof = False
    scene.camera = cam
    DRONE.hide_render = False

    t0 = time.time()
    done = 0
    for i in range(n):
        fp = f"{dirp}/f{i:04d}.png"
        if os.path.exists(fp) and os.path.getsize(fp) > 20000:
            continue
        scene.frame_set(i + 1)
        scene.render.filepath = fp
        bpy.ops.render.render(write_still=True)
        done += 1
        if done % 24 == 0:
            print(f"    {tag} {i+1}/{n}  {(time.time()-t0)/done:.2f}s/f", flush=True)
    meta = {"shot": tag, "kind": "chase", "label": w["label"], "frames": n,
            "lens_mm": w["lens"], "chase_offset_body_m": list(w["chase_offset"]),
            "track": shot, "render_s": round(time.time() - t0, 1),
            "rendered_now": done, "drone_visible": True}
    json.dump(meta, open(f"{dirp}/meta.json", "w"), indent=1)
    print(f"<<< {tag}  {done} frames  {meta['render_s']}s", flush=True)
    return meta


if __name__ == "__main__":
    print(f"CONFIG engine={scene.render.engine} taa={TAA} res={RES}% "
          f"drone_objects={drep['n_objects']} missing={drep['missing']}", flush=True)
    t_all = time.time()
    metas = []
    for w in SL.WIDES:
        if w["name"] == "open":
            m = render_wide(w)
            if m:
                metas.append(m)
    for b in SL.BEATS:
        m = render_beat(b)
        if m:
            metas.append(m)
    for w in SL.WIDES:
        if w["name"] != "open":
            m = render_wide(w)
            if m:
                metas.append(m)
    json.dump(metas, open(f"{OUT}/render_index.json", "w"), indent=1)
    print(f"\n===FILM_FRAMES_DONE=== {time.time()-t_all:.1f}s, {len(metas)} shots")

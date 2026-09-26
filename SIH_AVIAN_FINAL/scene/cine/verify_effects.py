"""Prove DoF and motion blur are in the FINISHED FILM, not merely requested.

Asking Blender what it was configured to do proves nothing about the mp4 on
disk. So for a given (camera, scene frame) this renders three references with
the production settings and one knob flipped:

    prod     - exactly what render_clips.py used
    dof_off  - same frame, depth of field disabled
    mb_off   - same frame, motion blur disabled

then pulls the SAME moment out of the finished film and measures which
reference it matches. If the film frame sits close to `prod` and measurably far
from `dof_off`, the defocus survived into the encode. Same logic for motion
blur. Codec noise is far smaller than either effect, so H.264 does not blur the
distinction.

Frame mapping: shot k begins in the final timeline at k*(SHOT_FRAMES-XFADE) and
scene frame = shot_range(k)[0] + (final_frame - k*(SHOT_FRAMES-XFADE)).
"""
import bpy, sys, os, json, subprocess
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

MEDIA = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media"
OUT = f"{MEDIA}/_stills"
VID = f"{MEDIA}/bridge_defect_walkthrough_cinematic.mp4"
os.makedirs(OUT, exist_ok=True)

STEP = cs.SHOT_FRAMES - cs.XFADE_FRAMES

# (shot index, final-video frame to sample)
TARGETS = [(10, 1000), (11, 1100)]

scene = bpy.context.scene
look, shots = cs.build(scene, taa=16, motion_blur=True, res_pct=100)
scene.eevee.use_bokeh_jittered = False          # match render_clips.py exactly
scene.render.image_settings.file_format = "PNG"


def render_still(tag, cam, frame):
    scene.camera = cam
    scene.frame_set(frame)
    scene.render.filepath = f"{OUT}/{tag}.png"
    bpy.ops.render.render(write_still=True)
    return load(f"{OUT}/{tag}.png")


def load(path):
    im = bpy.data.images.load(path)
    a = np.array(im.pixels[:], dtype=np.float32).reshape(im.size[1], im.size[0], 4)[:, :, :3]
    bpy.data.images.remove(im)
    return a[::-1]          # Blender's buffer is bottom-up; ffmpeg's is top-down


def grab_film(final_frame, tag):
    p = f"{OUT}/{tag}.png"
    subprocess.run(
        f'ffmpeg -y -v error -i "{VID}" -vf "select=eq(n\\,{final_frame})" '
        f'-vsync 0 -frames:v 1 "{p}"', shell=True, check=False)
    return load(p) if os.path.exists(p) else None


def mad(a, b):
    return float(np.abs(a - b).mean())


def sharpness(x):
    g = x @ np.array([.2126, .7152, .0722], dtype=np.float32)
    lap = (-4 * g[1:-1, 1:-1] + g[:-2, 1:-1] + g[2:, 1:-1]
           + g[1:-1, :-2] + g[1:-1, 2:])
    return float(lap.var())


results = []
for k, final_frame in TARGETS:
    name = cs.SHOT_ORDER[k]
    cam = bpy.data.objects[name]
    f0, _ = cs.shot_range(k)
    scene_frame = f0 + (final_frame - k * STEP)

    prod = render_still(f"{name}_prod", cam, scene_frame)

    cam.data.dof.use_dof = False
    dof_off = render_still(f"{name}_dofoff", cam, scene_frame)
    cam.data.dof.use_dof = True

    scene.eevee.use_motion_blur = False
    mb_off = render_still(f"{name}_mboff", cam, scene_frame)
    scene.eevee.use_motion_blur = True

    film = grab_film(final_frame, f"{name}_film")

    r = {
        "camera": name,
        "final_video_frame": final_frame,
        "scene_frame": scene_frame,
        "lens_mm": cam.data.lens,
        "dof_enabled": bool(cam.data.dof.use_dof),
        "fstop": getattr(cam.data.dof, "aperture_fstop", None),
        "focus_distance_m": round(cam.data.dof.focus_distance, 3),
        "camera_world_xyz": [round(v, 3) for v in cam.matrix_world.translation],
        "sharpness_prod": round(sharpness(prod), 6),
        "sharpness_dof_off": round(sharpness(dof_off), 6),
        "dof_effect_mad": round(mad(prod, dof_off), 6),
        "dof_pct_pixels_changed": round(
            float((np.abs(prod - dof_off).max(axis=2) > 1 / 255).mean() * 100), 3),
        "mb_effect_mad": round(mad(prod, mb_off), 6),
        "mb_pct_pixels_changed": round(
            float((np.abs(prod - mb_off).max(axis=2) > 1 / 255).mean() * 100), 3),
    }
    if film is not None:
        r["film_vs_prod_mad"] = round(mad(film, prod), 6)
        r["film_vs_dof_off_mad"] = round(mad(film, dof_off), 6)
        r["film_vs_mb_off_mad"] = round(mad(film, mb_off), 6)
        r["dof_present_in_film"] = bool(r["film_vs_prod_mad"] < r["film_vs_dof_off_mad"])
        r["mb_present_in_film"] = bool(r["film_vs_prod_mad"] < r["film_vs_mb_off_mad"])
    else:
        r["film"] = "COULD NOT EXTRACT"
    results.append(r)
    print(f"  done {name}", flush=True)

print("\n===EFFECTS===")
print(json.dumps(results, indent=2))
print("===DONE_EFFECTS===")

"""Prove DoF and motion blur actually change pixels, instead of assuming they do."""
import bpy, sys, os, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

OUT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_verify"
os.makedirs(OUT, exist_ok=True)
scene = bpy.context.scene
cs.build(scene, taa=16, motion_blur=True, res_pct=50)
scene.eevee.use_bokeh_jittered = False
scene.render.image_settings.file_format = "PNG"


def shoot(tag, frame, cam):
    scene.camera = cam
    scene.frame_set(frame)
    p = f"{OUT}/{tag}.png"
    scene.render.filepath = p
    bpy.ops.render.render(write_still=True)
    img = bpy.data.images.load(p)
    a = np.array(img.pixels[:], dtype=np.float32).reshape(
        img.size[1], img.size[0], 4)[:, :, :3]
    bpy.data.images.remove(img)
    return a


def sharpness(a):
    """Laplacian variance - higher means more high-frequency detail."""
    g = a @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    lap = (-4 * g[1:-1, 1:-1] + g[:-2, 1:-1] + g[2:, 1:-1]
           + g[1:-1, :-2] + g[1:-1, 2:])
    return float(lap.var())


def compare(name, a, b):
    d = np.abs(a - b)
    return {"test": name,
            "mean_abs_diff": round(float(d.mean()), 6),
            "max_abs_diff": round(float(d.max()), 6),
            "pct_pixels_changed_gt_1_255": round(
                float((d.max(axis=2) > 1 / 255).mean() * 100), 3),
            "sharpness_A": round(sharpness(a), 6),
            "sharpness_B": round(sharpness(b), 6)}


results = []

# ---- DoF on vs off, on each DoF camera -------------------------------
for name in ["CAM_06_HERO_DEFECT", "CAM_11_GUSSET", "CAM_12_LOOSE_BOLT"]:
    cam = bpy.data.objects[name]
    idx = cs.SHOT_ORDER.index(name)
    f0, f1 = cs.shot_range(idx)
    mid = (f0 + f1) // 2
    cam.data.dof.use_dof = True
    on = shoot(f"dof_on_{name}", mid, cam)
    cam.data.dof.use_dof = False
    off = shoot(f"dof_off_{name}", mid, cam)
    cam.data.dof.use_dof = True
    r = compare(f"DoF {name}", on, off)
    r["fstop"] = cs.DOF_CAMS[name]
    r["focus_distance_m"] = round(cam.data.dof.focus_distance, 3)
    r["note"] = "sharpness_A=DoF on, sharpness_B=DoF off"
    results.append(r)

# ---- motion blur on vs off, mid-shot where the camera is fastest -----
for name in ["CAM_01_OVERVIEW", "CAM_06_HERO_DEFECT"]:
    cam = bpy.data.objects[name]
    idx = cs.SHOT_ORDER.index(name)
    f0, f1 = cs.shot_range(idx)
    mid = (f0 + f1) // 2
    scene.eevee.use_motion_blur = True
    on = shoot(f"mb_on_{name}", mid, cam)
    scene.eevee.use_motion_blur = False
    off = shoot(f"mb_off_{name}", mid, cam)
    scene.eevee.use_motion_blur = True
    r = compare(f"MotionBlur {name}", on, off)
    r["note"] = "sharpness_A=MB on, sharpness_B=MB off"
    results.append(r)

# ---- confirm the camera actually moves across the shot --------------
moves = []
for name in cs.SHOT_ORDER:
    cam = bpy.data.objects[name]
    idx = cs.SHOT_ORDER.index(name)
    f0, f1 = cs.shot_range(idx)
    scene.frame_set(f0)
    dg = bpy.context.evaluated_depsgraph_get()
    p0 = cam.evaluated_get(dg).matrix_world.translation.copy()
    r0 = cam.evaluated_get(dg).matrix_world.to_euler()
    scene.frame_set(f1)
    dg = bpy.context.evaluated_depsgraph_get()
    p1 = cam.evaluated_get(dg).matrix_world.translation.copy()
    r1 = cam.evaluated_get(dg).matrix_world.to_euler()
    scene.frame_set((f0 + f1) // 2)
    dg = bpy.context.evaluated_depsgraph_get()
    pm = cam.evaluated_get(dg).matrix_world.translation.copy()
    import math
    moves.append({
        "camera": name,
        "travel_m": round((p1 - p0).length, 4),
        "yaw_change_deg": round(math.degrees(abs(r1.z - r0.z)), 4),
        "midpoint_offset_from_linear_m": round(
            (pm - (p0 + p1) / 2).length, 5),
    })

print("\n===VERIFY===")
print(json.dumps(results, indent=2))
print("\n===MOTION===")
print(json.dumps(moves, indent=2))
print("===DONE_VERIFY===")

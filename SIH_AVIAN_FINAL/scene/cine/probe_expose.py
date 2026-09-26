import bpy, sys, os, json, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

OUT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_probe"
os.makedirs(OUT, exist_ok=True)

scene = bpy.context.scene
look, shots = cs.build(scene, taa=16, motion_blur=False, res_pct=15)

print("\n===LOOK_REPORT===")
print(json.dumps(look, indent=2, default=str))
print("\n===SHOT_REPORT===")
print(json.dumps(shots, indent=2, default=str))

# ---- exposure sweep: mid-frame of every shot, cheap resolution ----------
scene.render.image_settings.file_format = "PNG"
stats = []
for i, name in enumerate(cs.SHOT_ORDER):
    cam = bpy.data.objects.get(name)
    if not cam:
        continue
    f0, f1 = cs.shot_range(i)
    mid = (f0 + f1) // 2
    scene.camera = cam
    scene.frame_set(mid)
    path = f"{OUT}/{i:02d}_{name}.png"
    scene.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    dt = time.time() - t

    img = bpy.data.images.load(path)
    px = np.array(img.pixels[:], dtype=np.float32).reshape(-1, 4)
    rgb = px[:, :3]
    lum = rgb @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    stats.append({
        "camera": name,
        "frame": mid,
        "secs": round(dt, 2),
        "mean_lum": round(float(lum.mean()), 4),
        "p01": round(float(np.percentile(lum, 1)), 4),
        "p50": round(float(np.percentile(lum, 50)), 4),
        "p99": round(float(np.percentile(lum, 99)), 4),
        "blown_pct": round(float((lum > 0.98).mean() * 100), 3),
        "crushed_pct": round(float((lum < 0.02).mean() * 100), 3),
    })
    bpy.data.images.remove(img)

print("\n===EXPOSURE_REPORT===")
print(f"view_transform={scene.view_settings.view_transform} "
      f"exposure={scene.view_settings.exposure}")
print(json.dumps(stats, indent=2))
print("\n===DONE_PROBE===")

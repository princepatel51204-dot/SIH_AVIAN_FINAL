import bpy, sys, os, json, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

OUT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_look"
os.makedirs(OUT, exist_ok=True)

scene = bpy.context.scene
cs.build(scene, taa=32, motion_blur=False, res_pct=40)
scene.render.image_settings.file_format = "PNG"

vs = scene.view_settings
print("AVAILABLE_LOOKS:", [i.identifier for i in
      vs.bl_rna.properties['look'].enum_items])

# CAM_01 darkest wide, CAM_08 brightest/most clipped, CAM_06 hero, CAM_04 suspect framing
TEST_CAMS = ["CAM_01_OVERVIEW", "CAM_08_DECK", "CAM_06_HERO_DEFECT",
             "CAM_04_INTER_STRUCTURE"]
VARIANTS = [("cur", -1.1, "None"), ("e05", -0.5, "None"),
            ("e05p", -0.5, "AgX - Punchy"), ("e02", -0.2, "None")]

rows = []
for tag, exposure, look in VARIANTS:
    vs.exposure = exposure
    try:
        vs.look = look
    except Exception as e:
        print(f"look {look!r} unavailable: {e}")
        vs.look = "None"
        look = f"{look}(FAILED)"
    for name in TEST_CAMS:
        cam = bpy.data.objects.get(name)
        idx = cs.SHOT_ORDER.index(name)
        f0, f1 = cs.shot_range(idx)
        scene.camera = cam
        scene.frame_set((f0 + f1) // 2)
        path = f"{OUT}/{tag}_{name}.png"
        scene.render.filepath = path
        t = time.time()
        bpy.ops.render.render(write_still=True)
        dt = time.time() - t
        img = bpy.data.images.load(path)
        px = np.array(img.pixels[:], dtype=np.float32).reshape(-1, 4)
        lum = px[:, :3] @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
        rows.append({"variant": tag, "exposure": exposure, "look": look,
                     "cam": name, "secs": round(dt, 2),
                     "mean": round(float(lum.mean()), 4),
                     "p50": round(float(np.percentile(lum, 50)), 4),
                     "p99": round(float(np.percentile(lum, 99)), 4),
                     "blown_pct": round(float((lum > 0.98).mean() * 100), 3),
                     "crushed_pct": round(float((lum < 0.02).mean() * 100), 3)})
        bpy.data.images.remove(img)

print("\n===LOOK_TEST===")
hdr = f"{'var':<6}{'exp':>6}{'look':<18}{'cam':<24}{'mean':>8}{'p50':>8}{'p99':>8}{'blown%':>9}{'crush%':>9}"
print(hdr)
for r in rows:
    print(f"{r['variant']:<6}{r['exposure']:>6}{r['look']:<18}{r['cam']:<24}"
          f"{r['mean']:>8.3f}{r['p50']:>8.3f}{r['p99']:>8.3f}"
          f"{r['blown_pct']:>9.2f}{r['crushed_pct']:>9.2f}")
print("===DONE_LOOK===")

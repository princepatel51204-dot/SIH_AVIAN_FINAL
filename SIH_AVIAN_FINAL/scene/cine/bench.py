import bpy, sys, os, time, statistics, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

TAA     = int(os.environ.get("TAA", "64"))
MB      = os.environ.get("MB", "1") == "1"
JIT     = os.environ.get("JIT", "1") == "1"
RES     = int(os.environ.get("RES", "100"))
NFRAMES = int(os.environ.get("NFRAMES", "6"))
CAM     = os.environ.get("CAM", "CAM_06_HERO_DEFECT")
SAVE    = os.environ.get("SAVE", "")

scene = bpy.context.scene
cs.build(scene, taa=TAA, motion_blur=MB, res_pct=RES)
if not JIT and hasattr(scene.eevee, "use_bokeh_jittered"):
    scene.eevee.use_bokeh_jittered = False

if os.environ.get("TUNE", "0") == "1":
    ee = scene.eevee
    ee.use_ssr_halfres = True
    ee.ssr_quality = 0.4
    ee.ssr_max_roughness = 0.4
    ee.shadow_cascade_size = "2048"
    ee.use_shadow_high_bitdepth = False
    ee.use_gtao_bounce = False

cam = bpy.data.objects[CAM]
idx = cs.SHOT_ORDER.index(CAM)
f0, f1 = cs.shot_range(idx)
scene.camera = cam

times = []
for k in range(NFRAMES):
    scene.frame_set(f0 + k)
    write = bool(SAVE) and k == NFRAMES - 1
    if write:
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = SAVE
    t = time.time()
    bpy.ops.render.render(write_still=write)
    times.append(time.time() - t)

steady = times[1:] if len(times) > 1 else times
print("\n===BENCH===")
print(json.dumps({
    "cam": CAM, "res_pct": RES,
    "pixels": f"{int(1920*RES/100)}x{int(1080*RES/100)}",
    "taa": TAA, "motion_blur": MB, "jittered_dof": JIT,
    "dof_on": cam.data.dof.use_dof,
    "frames_timed": NFRAMES,
    "first_frame_s": round(times[0], 2),
    "steady_frames_s": [round(t, 2) for t in steady],
    "steady_mean_s": round(statistics.mean(steady), 3),
    "steady_median_s": round(statistics.median(steady), 3),
}, indent=2))
print("===DONE_BENCH===")

import bpy, time, os, sys
eng = os.environ.get("ENG","BLENDER_EEVEE")
sc = bpy.context.scene
sc.render.engine = eng
sc.camera = bpy.data.objects["CAM_06_HERO_DEFECT"]
sc.render.resolution_x, sc.render.resolution_y = 1920,1080
sc.render.resolution_percentage = 25
sc.render.image_settings.file_format='PNG'
sc.render.filepath = f"/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_smoke_{eng}_"
sc.frame_set(1)
if eng=='CYCLES':
    sc.cycles.samples=32; sc.cycles.use_denoising=True
    sc.cycles.denoiser='OPENIMAGEDENOISE'; sc.cycles.device='CPU'
    sc.cycles.time_limit=120
else:
    sc.eevee.taa_render_samples=32
t=time.time()
try:
    bpy.ops.render.render(write_still=True)
    print(f"SMOKE_OK engine={eng} seconds={time.time()-t:.2f}")
except Exception as e:
    print(f"SMOKE_FAIL engine={eng} err={type(e).__name__}: {e}")

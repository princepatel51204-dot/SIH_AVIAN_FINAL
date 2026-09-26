import bpy, time, os
sc = bpy.context.scene
cy = sc.cycles
print("DENOISER_ENUM:", [i.identifier for i in cy.bl_rna.properties['denoiser'].enum_items])
print("HAS use_denoising attr:", hasattr(cy,'use_denoising'))
try:
    cy.use_denoising = False
    print("use_denoising set OK ->", cy.use_denoising)
except Exception as e:
    print("use_denoising FAILED:", e)
# compositor denoise node availability (OIDN via compositor)
try:
    nt = bpy.data.node_groups
    sc.use_nodes = True
    n = sc.node_tree.nodes.new('CompositorNodeDenoise')
    print("CompositorNodeDenoise: AVAILABLE")
    sc.node_tree.nodes.remove(n)
except Exception as e:
    print("CompositorNodeDenoise: UNAVAILABLE ->", e)

sc.render.engine='CYCLES'
sc.cycles.device='CPU'
sc.camera = bpy.data.objects["CAM_06_HERO_DEFECT"]
sc.render.resolution_x, sc.render.resolution_y = 1920,1080
sc.render.resolution_percentage = 25
sc.cycles.samples = 32
sc.cycles.use_adaptive_sampling = True
sc.cycles.use_denoising = False
sc.render.image_settings.file_format='PNG'
sc.render.filepath = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_smoke_CYCLES_"
sc.frame_set(1)
t=time.time()
bpy.ops.render.render(write_still=True)
print(f"SMOKE_OK engine=CYCLES_CPU res=25% samples=32 seconds={time.time()-t:.2f}")

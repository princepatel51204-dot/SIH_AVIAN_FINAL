import bpy, sys, os, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs
OUT="/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_verify"
os.makedirs(OUT, exist_ok=True)
scene=bpy.context.scene
look,shots=cs.build(scene,taa=16,motion_blur=True,res_pct=100)
scene.eevee.use_bokeh_jittered=False
scene.render.image_settings.file_format="PNG"
print("===SHOT12==="); print(json.dumps(shots[11],indent=2,default=str))
def shoot(tag,frame,cam):
    scene.camera=cam; scene.frame_set(frame)
    scene.render.filepath=f"{OUT}/{tag}.png"
    bpy.ops.render.render(write_still=True)
    im=bpy.data.images.load(f"{OUT}/{tag}.png")
    a=np.array(im.pixels[:],dtype=np.float32).reshape(im.size[1],im.size[0],4)[:,:,:3]
    bpy.data.images.remove(im); return a
def sharp(x):
    g=x@np.array([.2126,.7152,.0722],dtype=np.float32)
    l=-4*g[1:-1,1:-1]+g[:-2,1:-1]+g[2:,1:-1]+g[1:-1,:-2]+g[1:-1,2:]
    return float(l.var())
cam=bpy.data.objects["CAM_12_LOOSE_BOLT"]
f0,f1=cs.shot_range(11); mid=(f0+f1)//2
cam.data.dof.use_dof=True;  on=shoot("new12_dof_on",mid,cam)
cam.data.dof.use_dof=False; off=shoot("new12_dof_off",mid,cam)
cam.data.dof.use_dof=True
d=np.abs(on-off)
print("===DOF12_AFTER_REFRAME===")
print(json.dumps({"pct_pixels_changed_gt_1_255":round(float((d.max(axis=2)>1/255).mean()*100),3),
 "mean_abs_diff":round(float(d.mean()),6),"max_abs_diff":round(float(d.max()),5),
 "sharpness_dof_on":round(sharp(on),6),"sharpness_dof_off":round(sharp(off),6),
 "fstop":cam.data.dof.aperture_fstop,"focus_m":round(cam.data.dof.focus_distance,3)},indent=2))

"""Render decal textures for Gazebo from the Blender digital twin (OFFLINE ASSET BUILD).

For each requested defect, place an orthographic camera looking straight at the defect along its
ground-truth surface normal and render a square crop (the defect mesh with its real material and
the host surface around it) to a PNG. The image becomes the albedo map of a thin decal quad in
Gazebo. The Blender file has no image textures (all materials are procedural), so a render of the
defect from its own scene is the way to obtain an equivalent texture.

Visualisation asset only. Nothing here reads or feeds navigation.

Run:  blender -b scene/SIH_AVIAN_FINAL.blend -P gazebo/decals/render_decals.py -- ID [ID ...]
      env: DECAL_PX (default 512), DECAL_M (crop width m, default max(1.2, 2.2*defect size)),
           DECAL_EXPOSURE (extra exposure stops, default 0), DECAL_OUT (output dir)
"""
import bpy, sys, os, json, math
from mathutils import Vector

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
sys.path.insert(0, f"{ROOT}/scene/cine")
import cine_setup as cs

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
PX = int(os.environ.get("DECAL_PX", "512"))
EXPO = float(os.environ.get("DECAL_EXPOSURE", "0"))
OUT = os.environ.get("DECAL_OUT", f"{ROOT}/gazebo/models/avian_final_decals/materials/textures")
os.makedirs(OUT, exist_ok=True)

gt = {}
for f in ("AVIAN_defect_ground_truth_FINAL.json", "AVIAN_metro_ground_truth_FINAL.json", "AVIAN_steel_ground_truth_FINAL.json"):
    for d in json.load(open(f"{ROOT}/scene/{f}"))["defects"]:
        gt[d["defect_id"]] = d

scene = bpy.context.scene
cs.setup_look(scene, taa=8, motion_blur=False)
scene.render.resolution_x = scene.render.resolution_y = PX
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGB"
scene.view_settings.exposure = scene.view_settings.exposure + EXPO
if hasattr(scene.render, "film_transparent"):
    scene.render.film_transparent = False

cd = bpy.data.cameras.new("DECAL_CAM")
cd.type = "ORTHO"
cd.clip_start = 0.05
cd.clip_end = 6.0
cam = bpy.data.objects.new("DECAL_CAM", cd)
scene.collection.objects.link(cam)
scene.camera = cam

meta = {}
for did in argv:
    d = gt[did]
    ob = bpy.data.objects.get(did)
    size = max(ob.dimensions.x, ob.dimensions.y, ob.dimensions.z) if ob else 0.5
    width = float(os.environ.get("DECAL_M", max(1.2, 2.2 * size)))
    p = Vector(d["position_m"])
    n = Vector(d.get("surface_normal") or (0, 0, 1)).normalized()
    cam.location = p + n * 1.5
    # look along -n; keep world +Z as "up" unless the normal is vertical
    up = Vector((0, 0, 1)) if abs(n.z) < 0.95 else Vector((0, 1, 0))
    cam.rotation_euler = (-n).to_track_quat("-Z", "Y").to_euler()
    cd.ortho_scale = width
    fp = f"{OUT}/{did}.png"
    scene.render.filepath = fp
    bpy.ops.render.render(write_still=True)
    meta[did] = {"file": os.path.basename(fp), "width_m": width, "defect_dims_m": [round(x, 3) for x in ob.dimensions] if ob else None,
                 "position_m": d["position_m"], "surface_normal": [round(x, 4) for x in n], "type": d["type"], "px": PX,
                 "exposure_extra": EXPO}
    print("DECAL", did, fp, "width", width, flush=True)
mp = f"{OUT}/decals_meta.json"
old = json.load(open(mp)) if os.path.exists(mp) else {}
old.update(meta)
json.dump(old, open(mp, "w"), indent=1)

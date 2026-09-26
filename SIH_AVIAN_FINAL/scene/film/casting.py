"""Casting call: render one detector-framed close-up of every modeled defect.

The 12 hero cameras were framed for cinematography, and at threshold 0.65 the
real detector fires on exactly one of them (CAM_06, the pier spall). That is
not a bug -- the detector was trained on 0.3-1.5 m close-ups, and the project's
own zoom-tile study already showed that narrowing FOV onto a defect is what
makes it fire.

So instead of guessing which defects to feature, this renders a candidate view
of each of the 96 ground-truth defects, using the view direction and clearance
the scene build already measured for it, and lets the detector vote. Whatever
it fires on becomes the shortlist for the film. Nothing here draws a box or
decides a defect is present -- it only produces frames for detect.py to judge.

Env:
  RES   render percentage (default 100)
  TAA   samples (default 16, the verified floor)
  ONLY  comma-separated defect_id substrings to restrict to
"""
import bpy, sys, os, json, math, time
from mathutils import Vector, Quaternion

sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/cine")
import cine_setup as cs

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
OUT = f"{ROOT}/media/_casting"
os.makedirs(OUT, exist_ok=True)

RES = int(os.environ.get("RES", "100"))
TAA = int(os.environ.get("TAA", "16"))
ONLY = [s for s in os.environ.get("ONLY", "").split(",") if s]

LENS_MM = 35.0
STANDOFF_MAX = 1.5      # detector training distribution tops out near here
STANDOFF_MIN = 0.40

scene = bpy.context.scene
cs.setup_look(scene, taa=TAA, motion_blur=False)
cs.setup_output(scene, res_pct=RES)
scene.eevee.use_bokeh_jittered = False
scene.render.image_settings.file_format = "PNG"

defects = json.load(open(f"{ROOT}/scene/AVIAN_defect_ground_truth_FINAL.json"))["defects"]
if ONLY:
    defects = [d for d in defects if any(o in d["defect_id"] for o in ONLY)]

cam_data = bpy.data.cameras.new("CASTING_CAM")
cam_data.lens = LENS_MM
cam = bpy.data.objects.new("CASTING_CAM", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam

dg = bpy.context.evaluated_depsgraph_get()
rows = []
t_all = time.time()

for i, d in enumerate(defects):
    pos = Vector(d["position_m"])
    vd = Vector(d.get("recommended_view_direction") or d.get("surface_normal") or (0, 1, 0))
    if vd.length < 1e-6:
        vd = Vector((0, 1, 0))
    vd.normalize()

    clear = d.get("best_view_clear_m") or STANDOFF_MAX
    standoff = max(STANDOFF_MIN, min(STANDOFF_MAX, clear * 0.9))

    cam.location = pos + vd * standoff
    # -Z looks forward in Blender; aim it back down the view direction at the defect
    cam.rotation_euler = (-vd).to_track_quat('-Z', 'Y').to_euler()
    bpy.context.view_layer.update()

    # is the line of sight actually clear, or are we inside/behind something?
    origin = cam.matrix_world.translation.copy()
    fwd = (cam.matrix_world.to_3x3() @ Vector((0, 0, -1))).normalized()
    hit, loc, _n, _idx, obj, _m = scene.ray_cast(dg, origin + fwd * 0.01, fwd)
    hit_dist = (Vector(loc) - origin).length if hit else float("inf")
    blocked = bool(hit and hit_dist < standoff - 0.12)

    tag = d["defect_id"]
    path = f"{OUT}/{tag}.png"
    scene.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    dt = time.time() - t

    rows.append({
        "defect_id": tag, "type": d["type"], "severity": d.get("severity"),
        "position_m": [round(v, 3) for v in pos],
        "host_surface": d.get("host_surface"), "host_object": d.get("host_object"),
        "bridge_section": d.get("bridge_section"), "sector": d.get("inspection_sector"),
        "width_mm": d.get("width_mm"), "length_m": d.get("length_m"),
        "depth_mm": d.get("depth_mm"),
        "visible_fraction": d.get("visible_fraction"),
        "standoff_m": round(standoff, 3), "lens_mm": LENS_MM,
        "cam_xyz": [round(v, 3) for v in origin],
        "los_hit": (obj.name if (hit and obj) else None),
        "los_hit_dist_m": None if math.isinf(hit_dist) else round(hit_dist, 3),
        "los_blocked": blocked,
        "render_s": round(dt, 2), "image": path,
    })
    print(f"[{i+1}/{len(defects)}] {tag:<34} {d['type']:<20} "
          f"standoff {standoff:.2f}m {dt:5.2f}s{'  BLOCKED' if blocked else ''}", flush=True)

json.dump(rows, open(f"{OUT}/casting_index.json", "w"), indent=1)
print(f"\n===CASTING_DONE=== {len(rows)} frames in {time.time()-t_all:.1f}s -> {OUT}")

import bpy, sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

scene = bpy.context.scene
cs.build(scene, taa=1, motion_blur=False, res_pct=10)


def sample(cam, f):
    scene.frame_set(f)
    dg = bpy.context.evaluated_depsgraph_get()
    m = cam.evaluated_get(dg).matrix_world
    return m.translation.copy(), m.to_euler()


rows = []
for name in cs.SHOT_ORDER:
    cam = bpy.data.objects[name]
    i = cs.SHOT_ORDER.index(name)
    f0, f1 = cs.shot_range(i)
    mid = (f0 + f1) // 2
    # per-frame speed at the very start, the middle, and the very end
    def speed(fa):
        pa, ra = sample(cam, fa)
        pb, rb = sample(cam, fa + 1)
        lin = (pb - pa).length
        ang = math.degrees(abs(rb.z - ra.z))
        return lin, ang
    s0 = speed(f0)
    sm = speed(mid)
    s1 = speed(f1 - 1)
    total_lin = (sample(cam, f1)[0] - sample(cam, f0)[0]).length
    metric = max(s0[0], s0[1] * 0.1), max(sm[0], sm[1] * 0.1)
    ratio = (metric[1] / metric[0]) if metric[0] > 1e-9 else float("inf")
    rows.append({
        "camera": name,
        "total_travel_m": round(total_lin, 4),
        "start_speed_m_per_frame": round(s0[0], 6),
        "mid_speed_m_per_frame": round(sm[0], 6),
        "end_speed_m_per_frame": round(s1[0], 6),
        "start_yaw_deg_per_frame": round(s0[1], 6),
        "mid_yaw_deg_per_frame": round(sm[1], 6),
        "mid_over_start_ratio": (None if math.isinf(ratio) else round(ratio, 2)),
        "eased": (ratio > 1.3) if not math.isinf(ratio) else True,
    })

print("\n===EASING===")
print(json.dumps(rows, indent=2))

# ---- how much room is BEHIND the macro camera, for a pull-back reveal ----
from mathutils import Vector
scene.frame_set(1)
dg = bpy.context.evaluated_depsgraph_get()
probe = {}
for name in ["CAM_12_LOOSE_BOLT", "CAM_11_GUSSET"]:
    cam = bpy.data.objects[name]
    mw = cam.matrix_world
    back = (mw.to_3x3() @ Vector((0, 0, 1))).normalized()
    dist, obj = cs._cast(scene, dg, mw.translation.copy(), back)
    probe[name] = {
        "clearance_behind_m": (None if math.isinf(dist) else round(dist, 3)),
        "first_hit_behind": obj,
        "current_subject_dist_m": round(
            cs._cast(scene, dg, mw.translation.copy(),
                     (mw.to_3x3() @ Vector((0, 0, -1))).normalized())[0], 3),
        "lens_mm": cam.data.lens,
    }
print("\n===PULLBACK_ROOM===")
print(json.dumps(probe, indent=2))
print("===DONE_EASING===")

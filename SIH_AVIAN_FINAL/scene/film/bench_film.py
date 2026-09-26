"""Gate benchmark: what does one chase frame and one onboard frame actually cost?

Builds the real thing -- drone imported, both rigs keyed off the flown track,
production look (EEVEE, TAA 16, motion blur) -- then times steady-state frames
on each rig, with and without DoF, so the film's total can be projected from
measured numbers instead of guessed ones.

The first frame of each block is discarded: it carries shader compilation and
depsgraph warm-up that the other ~2,900 frames will not pay.

Env: TAA (16), NFRAMES (4), T0 (sim time to sample around)
"""
import bpy, sys, os, json, time, statistics

sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/film")
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/cine")
import cine_setup as cs
import drone as D
import rigs as R

TAA = int(os.environ.get("TAA", "16"))
NF = int(os.environ.get("NFRAMES", "4"))
T0 = float(os.environ.get("T0", "7982.4"))      # V5_1015, the strongest real detection

scene = bpy.context.scene
cs.setup_look(scene, taa=TAA, motion_blur=True)
cs.setup_output(scene, res_pct=100)
scene.eevee.use_bokeh_jittered = False

drep = D.import_drone(scene, spin=True, frame_start=1, frame_end=4000)
print("drone objects:", drep["n_objects"], "missing:", drep["missing"], flush=True)

shot = R.build_shot(scene, T0 - 3.0, T0 + 2.0, frame_start=1)
print("shot:", json.dumps(shot), flush=True)

coll = bpy.data.collections[D.COLL]


def time_block(cam_name, dof, hide_drone, label):
    cam = bpy.data.objects[cam_name]
    scene.camera = cam
    cam.data.dof.use_dof = dof
    if dof:
        cam.data.dof.aperture_fstop = 4.0
        cam.data.dof.focus_distance = 3.0
    coll.hide_render = hide_drone
    ts = []
    f0 = shot["frame_range"][0] + 5
    for k in range(NF):
        scene.frame_set(f0 + k * 3)
        t = time.time()
        bpy.ops.render.render(write_still=False)
        ts.append(time.time() - t)
    steady = ts[1:] if len(ts) > 1 else ts
    r = {"block": label, "camera": cam_name, "dof": dof,
         "drone_hidden": hide_drone, "first_s": round(ts[0], 2),
         "steady_s": [round(x, 2) for x in steady],
         "mean_s": round(statistics.mean(steady), 3)}
    print("  " + json.dumps(r), flush=True)
    return r


out = [
    time_block("CAM_CHASE", False, False, "chase_deep"),
    time_block("CAM_ONBOARD", False, True, "onboard_deep"),
    time_block("CAM_ONBOARD", True, True, "onboard_dof"),
    time_block("CAM_CHASE", True, False, "chase_dof"),
]

print("\n===FILMBENCH===")
print(json.dumps({"taa": TAA, "res": "1920x1080", "t0_sim": T0,
                  "drone": {"objects": drep["n_objects"], "missing": drep["missing"]},
                  "shot": shot, "blocks": out}, indent=1))
print("===DONE_FILMBENCH===")

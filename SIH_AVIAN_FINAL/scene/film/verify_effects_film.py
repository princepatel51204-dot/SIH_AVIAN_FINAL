"""Render production / DoF-off / motion-blur-off references for chosen beat frames.

Replaces scene/cine/verify_effects.py for the inspection film (that script
targets the older walkthrough and its dof_present_in_film / mb_present_in_film
fields carry an image-orientation bug). This one only renders the three
references; scene/film/analyse_effects_film.py then compares each against the
frame decoded from the finished mp4, in PIL, top-down on both sides.

Run:  blender -b scene/SIH_AVIAN_FINAL.blend -P scene/film/verify_effects_film.py
"""
import bpy, sys, os, json
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/film")
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/cine")
import render_film as RF          # module top-level only configures the scene; main is guarded
import shotlist as SL

OUT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_vfilm/fx"
TARGETS = [(4, 30), (4, 100), (8, 30), (8, 100)]      # (beat n, frame index)
scene = RF.scene

for n, _ in sorted(set((t[0], 0) for t in TARGETS)):
    beat = next(b for b in SL.BEATS if b["n"] == n)
    path = RF.beat_camera_path(beat)
    cam = RF.new_cam("CAM_BEAT", 50.0)
    cam.data.dof.use_dof = True
    cam.data.dof.aperture_fstop = RF.DOF_FSTOP
    cam.data.dof.aperture_blades = 7
    scene.camera = cam
    RF.DRONE.hide_render = True
    for i, (pos, tgt, lens) in enumerate(path):          # same keyframing as render_beat
        cam.location = pos
        cam.rotation_mode = "QUATERNION"
        cam.rotation_quaternion = (tgt - pos).to_track_quat("-Z", "Y")
        cam.data.dof.focus_distance = max(0.1, (tgt - pos).length)
        cam.data.lens = lens
        f = i + 1
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_quaternion", frame=f)
        cam.data.keyframe_insert("lens", frame=f)
        cam.data.dof.keyframe_insert("focus_distance", frame=f)
    for bn, i in [t for t in TARGETS if t[0] == n]:
        scene.frame_set(i + 1)
        for tag, dof, mb in (("prod", True, True), ("dofoff", False, True), ("mboff", True, False)):
            cam.data.dof.use_dof = dof
            scene.eevee.use_motion_blur = mb
            scene.render.filepath = f"{OUT}/b{bn}_f{i:04d}_{tag}.png"
            bpy.ops.render.render(write_still=True)
        cam.data.dof.use_dof = True
        scene.eevee.use_motion_blur = True
        print(f"<<< fx beat{bn} f{i}", flush=True)
print("===FX_DONE===", flush=True)

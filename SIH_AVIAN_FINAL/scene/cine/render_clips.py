"""Render the 12 shots as separate clips, resumable.

Separate clips exist so the crossfades can be done properly in ffmpeg. Each
clip is SHOT_FRAMES long; the 10-frame overlap consumed by each crossfade is
paid for by rendering slightly longer shots, so the finished film still runs
its full ~48 s.

Already-finished clips are skipped, so an interrupted run resumes instead of
starting over.
"""
import bpy, sys, os, glob, time, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cine_setup as cs

CLIPS = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_clips"
TAA = int(os.environ.get("TAA", "16"))
RES = int(os.environ.get("RES", "100"))
ONLY = os.environ.get("ONLY", "")

os.makedirs(CLIPS, exist_ok=True)
scene = bpy.context.scene
look, shots = cs.build(scene, taa=TAA, motion_blur=True, res_pct=RES)

# Post-process DoF, not jittered: jittered costs ~35% more per frame and the
# scene has no raytraced noise for the extra samples to resolve.
scene.eevee.use_bokeh_jittered = False

scene.render.image_settings.file_format = "FFMPEG"
scene.render.ffmpeg.format = "MPEG4"
scene.render.ffmpeg.codec = "H264"
scene.render.ffmpeg.constant_rate_factor = "PERC_LOSSLESS"
scene.render.ffmpeg.ffmpeg_preset = "GOOD"
scene.render.ffmpeg.gopsize = 12
scene.render.ffmpeg.audio_codec = "NONE"

print(f"CONFIG taa={TAA} res={RES}% engine={scene.render.engine} "
      f"exposure={scene.view_settings.exposure} "
      f"view_transform={scene.view_settings.view_transform}")

t_all = time.time()
done = []
for i, name in enumerate(cs.SHOT_ORDER):
    if ONLY and name not in ONLY.split(","):
        continue
    final = f"{CLIPS}/shot_{i:02d}_{name}.mp4"
    if os.path.exists(final) and os.path.getsize(final) > 10000:
        print(f"SKIP shot {i:02d} {name} (already rendered)")
        done.append(final)
        continue

    cam = bpy.data.objects[name]
    f0, f1 = cs.shot_range(i)
    scene.camera = cam
    scene.frame_start, scene.frame_end = f0, f1

    workdir = f"{CLIPS}/_w{i:02d}"
    os.makedirs(workdir, exist_ok=True)
    for stale in glob.glob(f"{workdir}/*"):
        os.remove(stale)
    scene.render.filepath = f"{workdir}/f"

    t = time.time()
    print(f"\n>>> SHOT {i:02d} {name} frames {f0}-{f1} "
          f"dof={cam.data.dof.use_dof} lens={cam.data.lens}mm", flush=True)
    bpy.ops.render.render(animation=True)
    dt = time.time() - t

    produced = sorted(glob.glob(f"{workdir}/*.mp4"))
    if not produced:
        print(f"!!! shot {i:02d} produced no file in {workdir}")
        continue
    os.replace(produced[0], final)
    for leftover in glob.glob(f"{workdir}/*"):
        os.remove(leftover)
    os.rmdir(workdir)
    size = os.path.getsize(final)
    nf = f1 - f0 + 1
    print(f"<<< SHOT {i:02d} {name} {dt:.1f}s  {dt/nf:.2f}s/frame  "
          f"{size/1e6:.2f} MB -> {final}", flush=True)
    done.append(final)

print(f"\n===CLIPS_DONE=== total {time.time()-t_all:.1f}s, {len(done)} clips")
for d in done:
    print(f"  {os.path.basename(d)}  {os.path.getsize(d)/1e6:.2f} MB")

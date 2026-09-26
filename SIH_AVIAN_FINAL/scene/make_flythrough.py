import bpy

scene = bpy.context.scene

# Inspection-story order: context first, then structure, then defects, ending on the hero shots
order = [
    "CAM_01_OVERVIEW", "CAM_02_RIVER", "CAM_09_BASE", "CAM_05_PIER",
    "CAM_10_TRUSS", "CAM_11_GUSSET", "CAM_04_INTER_STRUCTURE",
    "CAM_03_UNDERDECK", "CAM_08_DECK", "CAM_07_METRO",
    "CAM_06_HERO_DEFECT", "CAM_12_LOOSE_BOLT",
]

fps = 24
seconds_per_shot = 4
frames_per_shot = fps * seconds_per_shot

scene.render.fps = fps
scene.frame_start = 1
scene.frame_end = frames_per_shot * len(order)

# Clear old markers, bind each camera to its time slot
for m in list(scene.timeline_markers):
    scene.timeline_markers.remove(m)

for i, cam_name in enumerate(order):
    cam = bpy.data.objects.get(cam_name)
    if cam is None:
        print(f"WARNING: camera {cam_name} not found, skipping")
        continue
    frame = i * frames_per_shot + 1
    marker = scene.timeline_markers.new(name=cam_name, frame=frame)
    marker.camera = cam

scene.camera = bpy.data.objects.get(order[0])

# Output: video only, no image sequence files left behind
scene.render.image_settings.file_format = 'FFMPEG'
scene.render.ffmpeg.format = 'MPEG4'
scene.render.ffmpeg.codec = 'H264'
scene.render.ffmpeg.constant_rate_factor = 'HIGH'
scene.render.filepath = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/bridge_defect_walkthrough.mp4"
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.resolution_percentage = 100

print(f"Total frames: {scene.frame_end}, duration: {scene.frame_end/fps:.1f}s")
bpy.ops.render.render(animation=True)
print("DONE:", scene.render.filepath)

import bpy, sys, json, math

print("\n" + "="*70)
print("BLENDER:", bpy.app.version_string)
print("="*70)

# ---- 1. CYCLES DEVICES -------------------------------------------------
print("\n--- CYCLES COMPUTE DEVICES ---")
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    try:
        prefs.refresh_devices()
    except Exception as e:
        print("refresh_devices failed:", e)
    print("compute_device_type options:", [i.identifier for i in
          prefs.bl_rna.properties['compute_device_type'].enum_items])
    print("current compute_device_type:", prefs.compute_device_type)
    for dtype in ('CUDA','OPTIX','HIP','ONEAPI','METAL'):
        try:
            devs = prefs.get_devices_for_type(dtype)
        except Exception as e:
            print(f"  {dtype}: query failed ({e})")
            continue
        if devs:
            for d in devs:
                print(f"  {dtype}: {d.name} | type={d.type} | use={d.use}")
        else:
            print(f"  {dtype}: (none)")
    print("all prefs.devices:")
    for d in prefs.devices:
        print(f"   - {d.name} [{d.type}] use={d.use}")
except Exception as e:
    print("cycles prefs unavailable:", e)

# ---- 2. ENGINES --------------------------------------------------------
sc = bpy.context.scene
print("\n--- RENDER ENGINE ---")
print("engines available:", [i.identifier for i in
      sc.render.bl_rna.properties['engine'].enum_items])
print("current engine:", sc.render.engine)
print("resolution:", sc.render.resolution_x, "x", sc.render.resolution_y,
      "@", sc.render.resolution_percentage, "%  fps:", sc.render.fps)
print("frame range:", sc.frame_start, "-", sc.frame_end)
print("cycles samples:", getattr(sc.cycles, 'samples', 'n/a'),
      "adaptive:", getattr(sc.cycles, 'use_adaptive_sampling', 'n/a'),
      "threshold:", getattr(sc.cycles, 'adaptive_threshold', 'n/a'))
print("cycles denoise:", getattr(sc.cycles, 'use_denoising', 'n/a'),
      "denoiser:", getattr(sc.cycles, 'denoiser', 'n/a'))
print("cycles max bounces:", getattr(sc.cycles, 'max_bounces', 'n/a'))
print("motion blur (cycles):", getattr(sc.render, 'use_motion_blur', 'n/a'))
try:
    print("eevee taa_render_samples:", sc.eevee.taa_render_samples)
    print("eevee gtao:", sc.eevee.use_gtao, " ssr:", sc.eevee.use_ssr,
          " bloom:", getattr(sc.eevee,'use_bloom','n/a'),
          " soft_shadows:", getattr(sc.eevee,'use_soft_shadows','n/a'),
          " motion_blur:", getattr(sc.eevee,'use_motion_blur','n/a'))
except Exception as e:
    print("eevee settings:", e)

# ---- 3. COLOR MANAGEMENT ----------------------------------------------
vs = sc.view_settings
print("\n--- COLOR MANAGEMENT ---")
print("view_transform options:", [i.identifier for i in
      vs.bl_rna.properties['view_transform'].enum_items])
print("current view_transform:", vs.view_transform, "| look:", vs.look,
      "| exposure:", vs.exposure, "| gamma:", vs.gamma)

# ---- 4. WORLD ----------------------------------------------------------
print("\n--- WORLD ---")
w = sc.world
if w:
    print("world:", w.name, "use_nodes:", w.use_nodes)
    if w.use_nodes:
        for n in w.node_tree.nodes:
            print(f"   node {n.name} [{n.bl_idname}]")
            if n.bl_idname == 'ShaderNodeTexSky':
                print("      sky_type:", n.sky_type,
                      "sun_elevation:", round(math.degrees(getattr(n,'sun_elevation',0)),2), "deg",
                      "sun_rotation:", round(math.degrees(getattr(n,'sun_rotation',0)),2), "deg",
                      "sun_intensity:", getattr(n,'sun_intensity','n/a'),
                      "altitude:", getattr(n,'altitude','n/a'),
                      "air:", getattr(n,'air_density','n/a'),
                      "dust:", getattr(n,'dust_density','n/a'))
            if n.bl_idname == 'ShaderNodeBackground':
                print("      strength:", n.inputs['Strength'].default_value,
                      "color:", list(n.inputs['Color'].default_value))
else:
    print("NO WORLD")

# ---- 5. SCENE SCALE / STATS -------------------------------------------
objs = bpy.data.objects
meshes = [o for o in objs if o.type == 'MESH']
tris = 0
for o in meshes:
    try:
        tris += sum(len(p.vertices)-2 for p in o.data.polygons)
    except Exception:
        pass
print("\n--- SCENE STATS ---")
print("objects:", len(objs), "| meshes:", len(meshes),
      "| lights:", len([o for o in objs if o.type=='LIGHT']),
      "| cameras:", len([o for o in objs if o.type=='CAMERA']))
print("approx tris:", tris)
print("materials:", len(bpy.data.materials), "| images:", len(bpy.data.images))
big = sorted(bpy.data.images, key=lambda i: (i.size[0]*i.size[1]), reverse=True)[:8]
for i in big:
    print(f"   img {i.name}: {i.size[0]}x{i.size[1]}")
print("lights detail:")
for o in objs:
    if o.type=='LIGHT':
        print(f"   {o.name}: {o.data.type} energy={o.data.energy} size={getattr(o.data,'size','n/a')}")

# ---- 6. CAMERAS --------------------------------------------------------
print("\n--- CAMERAS ---")
cams = sorted([o for o in objs if o.type=='CAMERA'], key=lambda o: o.name)
for c in cams:
    d = c.data
    loc = tuple(round(v,3) for v in c.location)
    rot = tuple(round(math.degrees(v),2) for v in c.rotation_euler)
    anim = c.animation_data.action.name if (c.animation_data and c.animation_data.action) else None
    cons = [(x.type, getattr(x,'target',None).name if getattr(x,'target',None) else None) for x in c.constraints]
    print(f"{c.name}")
    print(f"   loc={loc} rot_euler_deg={rot} rot_mode={c.rotation_mode}")
    print(f"   lens={d.lens}mm sensor={d.sensor_width} clip={d.clip_start}/{d.clip_end}")
    print(f"   dof.use={d.dof.use_dof} focus_obj={d.dof.focus_object.name if d.dof.focus_object else None} focus_dist={round(d.dof.focus_distance,3)} fstop={d.dof.aperture_fstop}")
    print(f"   parent={c.parent.name if c.parent else None} constraints={cons} action={anim}")

# ---- 7. MARKERS --------------------------------------------------------
print("\n--- TIMELINE MARKERS ---")
for m in sorted(sc.timeline_markers, key=lambda m: m.frame):
    print(f"   f{m.frame}: {m.name} -> {m.camera.name if m.camera else None}")

# ---- 8. DEFECT OBJECTS -------------------------------------------------
print("\n--- COLLECTIONS ---")
for col in bpy.data.collections:
    print(f"   {col.name}: {len(col.objects)} objs")
print("\nDONE_INSPECT")

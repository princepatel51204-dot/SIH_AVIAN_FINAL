"""REV-C diagnostic renders: views that exist to verify REV-C's own work.

WHY THESE EXIST
---------------
The nine validation views in renders.py are inspection views -- they frame
the bridge, the river, the research zone and individual defects. Not one of
them gets within 50 m of a building, because nothing in the mission ever
needs to. That means the entire Stage 1 facade shader -- window grid, floor
banding, per-building hue, monsoon mould streaking -- was unverifiable from
the standard render set. A shader nobody can see is a shader nobody can
check.

Same reasoning that rewrote S14 in the UAV package: if the measurement
cannot see the thing it claims to measure, the measurement is the problem.
So REV-C carries its own diagnostic cameras, and they are part of the gate
rather than a scratch file.

FACADE_01  a street-level group, framed to show several buildings at once.
           This is the view that proves the per-object hue and weathering
           variation is real -- 680 buildings sharing four materials must
           not read as four colours.
FACADE_02  one facade at inspection standoff, where the window grid and the
           vertical mould streaking below sills have to hold up.
"""
from __future__ import annotations

import os
import time

import bpy
from mathutils import Vector

import meshlib as ML
import params as P

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "renders")


def _research_buildings(limit=400):
    """City buildings inside the research zone, tallest first."""
    out = []
    for o in bpy.data.objects:
        if not o.name.startswith("CITY_BLD_"):
            continue
        p = o.matrix_world.translation
        if not (P.RESEARCH_X0 - 200 <= p.x <= P.RESEARCH_X1 + 200):
            continue
        out.append(o)
        if len(out) >= limit:
            break
    out.sort(key=lambda o: -o.dimensions.z)
    return out


def _cam(name, lens=35.0):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start = 0.05
    cd.clip_end = 60000.0
    ob = bpy.data.objects.new(name, cd)
    coll = bpy.data.collections.get("CAMERAS") or bpy.context.scene.collection
    coll.objects.link(ob)
    return ob


def run(scene, samples=None, w=1280, h=720, log=print):
    os.makedirs(OUT_DIR, exist_ok=True)
    import lighting as LT
    LT.configure_render(samples=samples or 32, w=w, h=h)

    blds = _research_buildings()
    if not blds:
        log("  [SKIP] facade views: no CITY_BLD_ objects in the research zone")
        return []

    prev_cam = scene.camera
    manifest = []
    log("")
    log("  REV-C FACADE DIAGNOSTIC VIEWS")

    # ---- FACADE_01: a group, to show colour and weathering variety ------
    target = blds[0]
    tp = target.matrix_world.translation
    hz = max(target.dimensions.z, 12.0)
    cam1 = _cam("REVC_FACADE_GROUP", lens=35.0)
    cam1.location = Vector((tp.x - 55.0, tp.y - 62.0, hz * 0.55 + 6.0))
    ML.look_at(cam1, (tp.x, tp.y, hz * 0.42))
    scene.camera = cam1
    path = os.path.join(OUT_DIR, "FACADE_01_STREET_GROUP.png")
    scene.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    log(f"  facade 01 {os.path.basename(path):<36} {time.time()-t:5.0f} s  "
        f"several buildings: per-object hue and weathering variation")
    manifest.append({"view": "FACADE_01", "file": os.path.basename(path),
                     "purpose": "per-building hue/weathering variation",
                     "anchor_object": target.name})

    # ---- FACADE_02: one facade at inspection standoff -------------------
    # Stand off the FACE, not the centre. A fixed offset from the origin put
    # the camera inside a 40 m tower and rendered the inside of a wall --
    # a flat grey frame that looked exactly like a broken shader.
    half_y = max(target.dimensions.y, 4.0) / 2.0
    cam2 = _cam("REVC_FACADE_CLOSEUP", lens=50.0)
    cam2.location = Vector((tp.x, tp.y - half_y - 16.0, hz * 0.40))
    ML.look_at(cam2, (tp.x, tp.y - half_y, hz * 0.36))
    scene.camera = cam2
    path = os.path.join(OUT_DIR, "FACADE_02_CLOSEUP.png")
    scene.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    log(f"  facade 02 {os.path.basename(path):<36} {time.time()-t:5.0f} s  "
        f"window grid and mould streaking at standoff")
    manifest.append({"view": "FACADE_02", "file": os.path.basename(path),
                     "purpose": "window grid, floor banding, mould streaking",
                     "anchor_object": target.name})

    scene.camera = prev_cam
    log(f"  {len(manifest)} facade diagnostic views into "
        f"{os.path.basename(OUT_DIR)}/")
    return manifest

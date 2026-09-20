"""The nine validation views.

Views 1-7 are the seven named scene cameras. Views 8 and 9 are inspection-range
views that exist specifically to prove the defect system reads correctly at the
distance a UAV would actually work from -- a 300 m overview cannot show whether
a 0.2 mm crack looks like a crack.

INSPECTION LAMP
---------------
Views 8 and 9 add a small area light riding with the camera. This is not
cheating the lighting setup: the scene's daylight is deliberately configured so
the bridge underside is genuinely dark, because that is the real condition the
inspection research has to cope with. A real inspection UAV carries its own
light, and the lamp here reproduces that. Views 1-7 use daylight only.
"""
from __future__ import annotations
import os
import time

import bpy
from mathutils import Vector

import params as P
import meshlib as ML
import lighting as LT

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "renders")

VIEW_NOTES = {
    "CAMERA_01_GLOBAL_CITY":
        "whole city and corridor in context",
    "CAMERA_02_BRIDGE_FULL_LENGTH":
        "the full 4.5 km corridor, all seven sections",
    "CAMERA_03_RIVER_CROSSING":
        "90 m main span, river piers and air draft",
    "CAMERA_04_RESEARCH_ZONE":
        "the 900 m high-detail zone and its six sectors",
    "CAMERA_05_UNDERBRIDGE":
        "under-deck inspection volume, naturally dark",
    "CAMERA_06_DEFECT_CLOSEUP":
        "inspection standoff on a real defect",
    "CAMERA_07_FULL_INFRASTRUCTURE":
        "corridor, river, city and horizon together",
}


def _lamp(coll):
    ld = bpy.data.lights.new("VALIDATION_INSPECT_LAMP", "AREA")
    ld.energy = 90.0
    ld.size = 0.25
    ld.color = (1.0, 0.97, 0.93)
    ob = bpy.data.objects.new("VALIDATION_INSPECT_LAMP", ld)
    coll.objects.link(ob)
    return ob


def _aim_inspection(cam, lamp, pos, normal, dist):
    """Oblique standoff view of a surface point, lit slightly off-axis.

    Dead-on axial light flattens a shallow defect to nothing -- the relief a
    spall or a crack lip produces is entirely a grazing-illumination effect.
    """
    p = Vector(pos)
    n = Vector(normal).normalized()
    side = n.cross(Vector((0, 0, 1)))
    if side.length < 1e-3:
        side = Vector((1, 0, 0))
    side.normalize()
    eye = p + n * dist * 0.88 + side * dist * 0.42 - Vector((0, 0, dist * 0.2))
    cam.location = eye
    ML.look_at(cam, p)
    lamp.location = eye + side * 0.35 + Vector((0, 0, 0.30))
    ML.look_at(lamp, p)


def _pick(records, **kw):
    for r in records or []:
        if all(r.get(k) == v for k, v in kw.items()):
            return r
    return None


def run(scene, records, samples=None, w=1280, h=720, log=print):
    os.makedirs(OUT_DIR, exist_ok=True)
    LT.configure_render(samples=samples or 32, w=w, h=h)
    lamp = _lamp(bpy.data.collections["LIGHTING_ENVIRONMENT"])
    lamp.hide_render = True

    manifest = []
    log("")
    log("  VALIDATION RENDERS")

    # ---- views 1-7: the named cameras, daylight only ---------------------
    for i, name in enumerate(VIEW_NOTES, start=1):
        cam = bpy.data.objects.get(name)
        if cam is None:
            log(f"  [SKIP] view {i:02d}: camera {name} not in scene")
            continue
        scene.camera = cam
        path = os.path.join(OUT_DIR, f"VIEW_{i:02d}_{name[10:]}.png")
        scene.render.filepath = path
        t = time.time()
        bpy.ops.render.render(write_still=True)
        dt = time.time() - t
        log(f"  view {i:02d}  {os.path.basename(path):<42} {dt:5.0f} s  "
            f"{VIEW_NOTES[name]}")
        manifest.append({"view": i, "file": os.path.basename(path),
                         "camera": name, "lighting": "daylight only",
                         "purpose": VIEW_NOTES[name]})

    # ---- views 8-9: inspection range, with the UAV-style lamp ------------
    lamp.hide_render = False
    cd = bpy.data.cameras.new("VALIDATION_INSPECT_CAM")
    cd.lens = 50.0
    cd.clip_start = 0.02
    cd.clip_end = 60000.0
    cam = bpy.data.objects.new("VALIDATION_INSPECT_CAM", cd)
    bpy.data.collections["CAMERAS"].objects.link(cam)
    scene.camera = cam

    extra = [
        (8, "DEFECT_GEOMETRY_CLOSEUP", 1.5,
         (_pick(records, type="REBAR_EXPOSED", severity=4)
          or _pick(records, type="REBAR_EXPOSED")
          or _pick(records, type="SPALL", severity=3)),
         "geometry-backed defect at inspection range: real depth, "
         "sensor-detectable"),
        (9, "DEFECT_CRACK_CLOSEUP", 1.1,
         (_pick(records, type="CRACK_DIAGONAL", severity=3)
          or _pick(records, type="CRACK_NETWORK", severity=3)
          or _pick(records, type="CRACK_LONGITUDINAL")),
         "shader-backed cracking at inspection range: sub-sensor width, "
         "camera-only"),
    ]
    for i, tag, dist, rec, note in extra:
        if rec is None:
            log(f"  [SKIP] view {i:02d}: no matching defect in the ground truth")
            continue
        _aim_inspection(cam, lamp, rec["position_m"], rec["surface_normal"],
                        dist)
        path = os.path.join(OUT_DIR, f"VIEW_{i:02d}_{tag}.png")
        scene.render.filepath = path
        t = time.time()
        bpy.ops.render.render(write_still=True)
        dt = time.time() - t
        log(f"  view {i:02d}  {os.path.basename(path):<42} {dt:5.0f} s  {note}")
        manifest.append({
            "view": i, "file": os.path.basename(path),
            "camera": "VALIDATION_INSPECT_CAM",
            "lighting": "daylight + UAV-style inspection lamp",
            "purpose": note,
            "defect_id": rec["defect_id"], "defect_type": rec["type"],
            "severity": rec["severity"], "host_surface": rec["host_surface"],
            "standoff_m": dist})

    lamp.hide_render = True
    scene.camera = bpy.data.objects.get("CAMERA_04_RESEARCH_ZONE")

    import json
    with open(os.path.join(OUT_DIR, "RENDER_MANIFEST.json"), "w") as f:
        json.dump({"resolution": [w, h], "samples": samples or 32,
                   "engine": "CYCLES/CPU", "views": manifest}, f, indent=2)
    log(f"  {len(manifest)} of 9 validation views rendered into renders/")
    return manifest

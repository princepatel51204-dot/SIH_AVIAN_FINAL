"""Measured defect-vs-background contrast, and the visibility it corrects.

WHY THIS EXISTS
---------------
visibility.py answers "can a sensor geometrically see this defect": it casts
a 61-direction hemisphere, measures how much of it is open, and checks the
feature size against the camera's ground sample distance. That is a question
about occlusion and resolving power, and it is the right question -- but it
is not the whole one.

Weathering attacks contrast, not geometry. A hairline crack on mottled,
streaked concrete can stay 100 % geometrically visible and a hundred percent
resolvable, and still be undetectable, because it no longer differs from
what is around it. expected_rgb_visibility would not notice, and it becomes
optimistic the moment the concrete gets dirty. This module measures the
thing that field actually depends on.

HOW IT MEASURES
---------------
Not modelled from the palette constants -- rendered. For each defect, a
camera is placed along the view direction visibility.py already chose as
best, at the distance that frames the defect's own extent to fill
CONTRAST_DEFECT_FILL of the frame, and one small image is rendered with
Cycles' DIFFUSE COLOUR pass routed to the output. That pass is albedo: it
carries the surface's own reflectance with no lighting in it, which is what
a contrast comparison wants.

The centre disc of that image is the defect, the annulus around it is the
host surface behind it. Michelson contrast between the two mean luminances
is the measurement. Shader decals fall out of this correctly by
construction: a crack decal is mostly transparent, so its rendered albedo is
genuinely the blend of dark crack line and the concrete showing through,
which is exactly what a camera would see.

EXR, not PNG: the pass is read back as linear float. An 8-bit sRGB PNG would
put a nonlinear transform between the render and the ratio.
"""
from __future__ import annotations

import math
import os
import time

import bpy
from mathutils import Vector

import params_c as PC

_SENSOR_MM = 36.0
_LENS_MM = 50.0


def _luma(r, g, b):
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _setup_albedo_render(scene, res, samples, out_path):
    """Route the diffuse-colour pass to the composite output.

    EEVEE, not Cycles, and one sample. Two separate reasons:

    ENGINE. Cycles builds its acceleration structure through Embree, and on
    this machine any Cycles render issued after visibility.py's
    scene.ray_cast() pass segfaults in rtcSetSharedGeometryBuffer (see
    contrast_worker_c for the bisection). EEVEE is a rasteriser and never
    calls Embree, so it cannot reach that code path at all. Measured at
    ~0.85 s/defect against Cycles' 1.08-1.76 s, after a one-time shader
    compilation on the first render.

    SAMPLES. The measurement is of DIFFUSE COLOUR -- surface albedo, with
    no lighting in it. That is deliberate and it is the right quantity:
    detectability should not change when scenarios.py swaps the scene to
    overcast, night or monsoon, and lit contrast would. Albedo is a
    property of the surface, so one sample is exact rather than merely
    cheap, and the comparison of two small regions carries no sampling
    noise to begin with. Path-tracing a fully lit image to extract a
    quantity that does not depend on light was simply the wrong shape.
    """
    vl = scene.view_layers[0]
    vl.use_pass_diffuse_color = True

    scene.use_nodes = True
    nt = scene.node_tree
    nt.nodes.clear()
    rl = nt.nodes.new("CompositorNodeRLayers")
    rl.location = (0, 0)
    comp = nt.nodes.new("CompositorNodeComposite")
    comp.location = (400, 0)
    if "DiffCol" not in rl.outputs:
        raise RuntimeError("contrast_c: DiffCol pass not available")
    nt.links.new(rl.outputs["DiffCol"], comp.inputs["Image"])

    scene.render.engine = "BLENDER_EEVEE"
    scene.eevee.taa_render_samples = max(1, int(samples))
    scene.render.resolution_x = res
    scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    # NOT use_persistent_data. It is the obvious optimisation here -- 192
    # renders of a static scene with only the camera moving, and it did cut
    # the cost from 1.72 to 1.08 s/defect when measured against a saved
    # scene. But in the live build path it segfaults inside libembree4
    # (rtcSetSharedGeometryBuffer), and forcing BVH2 does not prevent it.
    # Established by bisection, not guessed: the same code with persistent
    # data off completes all 192 defects in the same process. 0.6 s/defect
    # is not worth a crash in the middle of a gate run.
    scene.render.use_persistent_data = False
    scene.render.image_settings.file_format = "OPEN_EXR"
    scene.render.image_settings.color_depth = "32"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.filepath = out_path


def _sample(path, res):
    """Mean luminance of the centre disc and of the surrounding annulus."""
    img = bpy.data.images.load(path, check_existing=False)
    try:
        px = list(img.pixels)
    finally:
        bpy.data.images.remove(img)

    n = 4                      # bpy image pixels are always RGBA
    cx = cy = (res - 1) / 2.0
    half = (res - 1) / 2.0
    disc_sum = disc_n = 0.0
    ann_sum = ann_n = 0.0
    for j in range(res):
        for i in range(res):
            k = (j * res + i) * n
            r, g, b = px[k], px[k + 1], px[k + 2]
            rad = math.hypot(i - cx, j - cy) / max(half, 1e-6)
            if rad <= PC.CONTRAST_DISC_R:
                disc_sum += _luma(r, g, b)
                disc_n += 1
            elif PC.CONTRAST_ANNULUS_R0 <= rad <= PC.CONTRAST_ANNULUS_R1:
                ann_sum += _luma(r, g, b)
                ann_n += 1
    if disc_n == 0 or ann_n == 0:
        return None, None
    return disc_sum / disc_n, ann_sum / ann_n


def _frame_distance(extent_m):
    """Distance at which extent_m fills CONTRAST_DEFECT_FILL of the frame."""
    fill = max(0.05, PC.CONTRAST_DEFECT_FILL)
    return max(0.25, extent_m * _LENS_MM / (_SENSOR_MM * fill))


def measure(records, strength_label=None, limit=None, log=print):
    """Measure contrast for every defect and re-derive RGB visibility.

    Adds to each record:
      defect_background_contrast  Michelson contrast, albedo, 0..1
      contrast_limited            True when the defect is resolvable but
                                  does not stand out enough to be detected

    and rewrites expected_rgb_visibility to require both. The original
    resolution-only verdict is preserved as
    expected_rgb_visibility_geometric so nothing is lost.
    """
    t0 = time.time()
    scene = bpy.context.scene
    prev_cam = scene.camera
    prev_use_nodes = scene.use_nodes
    prev_res = (scene.render.resolution_x, scene.render.resolution_y)
    prev_fmt = scene.render.image_settings.file_format
    prev_samples = scene.cycles.samples
    prev_path = scene.render.filepath

    tmp = bpy.app.tempdir or "/tmp"
    out_path = os.path.join(tmp, "avian_contrast")
    res = PC.CONTRAST_RES_PX
    _setup_albedo_render(scene, res, PC.CONTRAST_SAMPLES, out_path)

    cd = bpy.data.cameras.new("CONTRAST_PROBE_CAM")
    cd.lens = _LENS_MM
    cd.sensor_width = _SENSOR_MM
    cd.clip_start = 0.01
    cd.clip_end = 10000.0
    cam = bpy.data.objects.new("CONTRAST_PROBE_CAM", cd)
    scene.collection.objects.link(cam)
    scene.camera = cam

    todo = records if limit is None else records[:limit]
    n_measured = n_skipped = 0
    n_flip = 0
    changed = []

    for rec in todo:
        ob = bpy.data.objects.get(rec["defect_id"])
        if ob is None:
            rec["defect_background_contrast"] = None
            rec["contrast_limited"] = None
            n_skipped += 1
            continue

        extent = max(ob.dimensions) if max(ob.dimensions) > 1e-4 else 0.2
        d = _frame_distance(float(extent))
        n = Vector(rec.get("recommended_view_direction")
                   or rec["surface_normal"]).normalized()
        if n.length < 1e-6:
            n = Vector((0.0, 1.0, 0.0))
        p = Vector(rec["position_m"])
        cam.location = p + n * d
        # -Z looks at the defect
        cam.rotation_euler = (p - cam.location).to_track_quat("-Z",
                                                              "Y").to_euler()

        bpy.ops.render.render(write_still=True)
        exr = out_path + ".exr"
        ld, lb = _sample(exr, res)
        if ld is None:
            rec["defect_background_contrast"] = None
            rec["contrast_limited"] = None
            n_skipped += 1
            continue

        denom = ld + lb
        contrast = abs(ld - lb) / denom if denom > 1e-9 else 0.0
        rec["defect_background_contrast"] = round(float(contrast), 5)

        geo_ok = bool(rec.get("expected_rgb_visibility_geometric",
                              rec.get("expected_rgb_visibility", False)))
        rec["expected_rgb_visibility_geometric"] = geo_ok
        limited = bool(geo_ok and contrast < PC.CONTRAST_MIN_MICHELSON)
        rec["contrast_limited"] = limited
        rec["expected_rgb_visibility"] = bool(geo_ok and not limited)
        if limited:
            n_flip += 1
            changed.append(rec["defect_id"])
        n_measured += 1

    # ---- restore the scene we borrowed --------------------------------
    scene.camera = prev_cam
    scene.use_nodes = prev_use_nodes
    scene.render.resolution_x, scene.render.resolution_y = prev_res
    scene.render.image_settings.file_format = prev_fmt
    scene.cycles.samples = prev_samples
    scene.render.filepath = prev_path
    bpy.data.objects.remove(cam, do_unlink=True)
    bpy.data.cameras.remove(cd)

    vals = [r["defect_background_contrast"] for r in todo
            if r.get("defect_background_contrast") is not None]
    mean_c = sum(vals) / len(vals) if vals else 0.0
    dt = time.time() - t0
    label = strength_label if strength_label is not None else "n/a"
    log(f"  contrast: {n_measured} measured, {n_skipped} skipped, "
        f"mean {mean_c:.4f}, {n_flip} contrast-limited "
        f"(weather {label}, threshold {PC.CONTRAST_MIN_MICHELSON}, "
        f"{dt:.1f} s)")
    return {"measured": n_measured, "skipped": n_skipped,
            "mean_contrast": round(mean_c, 5),
            "contrast_limited": n_flip,
            "contrast_limited_ids": changed,
            "threshold_michelson": PC.CONTRAST_MIN_MICHELSON,
            "weather_strength": strength_label,
            "wall_s": round(dt, 1)}


FIELDS = ("defect_background_contrast", "contrast_limited",
          "expected_rgb_visibility_geometric")

"""
Cinematic setup for the AVIAN bridge walkthrough.

Applied at render time, in memory. Never saves SIH_AVIAN_FINAL.blend, so the
master scene file is left byte-identical.

Timeline layout: 12 shots of SHOT_FRAMES each, laid end to end on the global
timeline. Each shot is rendered as its own clip, then the clips are joined with
XFADE_FRAMES of crossfade, so the overlap is paid for by rendering slightly
longer shots rather than by losing screen time.
"""

import bpy
import math
import json
from mathutils import Vector

FPS = 24
SHOT_FRAMES = 105        # 4.375 s rendered per shot
XFADE_FRAMES = 10        # ~0.417 s crossfade between shots
# final length = 12*105 - 11*10 = 1150 frames = 47.92 s

SHOT_ORDER = [
    "CAM_01_OVERVIEW", "CAM_02_RIVER", "CAM_09_BASE", "CAM_05_PIER",
    "CAM_10_TRUSS", "CAM_11_GUSSET", "CAM_04_INTER_STRUCTURE",
    "CAM_03_UNDERDECK", "CAM_08_DECK", "CAM_07_METRO",
    "CAM_06_HERO_DEFECT", "CAM_12_LOOSE_BOLT",
]

# Shallow-ish DoF only on the three defect close-ups. Everything else stays
# deep-focus so the context shots read clearly.
# f-stops are chosen from the measured subject distance, not by eye: at 85 mm
# and 0.59 m, f/3.2 would leave only ~8 mm in focus and the nut itself would go
# soft, so the macro gets f/8 (~20 mm sharp) and still throws the truss behind
# it well out of focus.
DOF_CAMS = {
    "CAM_06_HERO_DEFECT": 4.0,   # 35 mm @ 3.23 m -> 2.23 m sharp, river blurs
    "CAM_11_GUSSET": 4.5,        # 50 mm @ 2.64 m -> 0.75 m sharp
    "CAM_12_LOOSE_BOLT": 5.6,    # 85 mm @ 2.59 m -> ~0.30 m sharp after reframe
}

# CAM_12 sat 0.588 m from a hex nut behind an 85 mm lens: a 25 cm-wide frame
# that read as abstract grey geometry, and DoF measurably did nothing there
# (0.009% of pixels changed) because the backing plate sat at the same depth as
# the nut. Backing the camera off 2 m opens the frame to ~1.1 m so the bolt is
# legible in context, and puts real distance between it and the structure
# behind so the defocus has something to act on. 12.7 m of clearance was
# measured behind this camera, so the move is safe.
BASE_OFFSETS = {
    "CAM_12_LOOSE_BOLT": 2.0,    # metres backwards along the camera's local +Z
}

# Deliberately varied per shot. push_pct is a fraction of the measured distance
# to whatever the camera is actually pointing at; every move is clamped by a
# real clearance probe before it is committed.
MOVES = {
    "CAM_01_OVERVIEW":       ("pan",    dict(yaw_deg=-3.2, rise=-4.0)),
    "CAM_02_RIVER":          ("rise",   dict(rise=2.2, push_pct=0.05)),
    "CAM_09_BASE":           ("drift",  dict(lateral=3.0)),
    "CAM_05_PIER":           ("push",   dict(push_pct=0.12)),
    "CAM_10_TRUSS":          ("orbit",  dict(orbit_deg=2.6)),
    "CAM_11_GUSSET":         ("push",   dict(push_pct=0.09)),
    "CAM_04_INTER_STRUCTURE":("drift",  dict(lateral=-2.4)),
    "CAM_03_UNDERDECK":      ("push",   dict(push_pct=0.15)),
    "CAM_08_DECK":           ("pan",    dict(yaw_deg=2.4, rise=0.0)),
    "CAM_07_METRO":          ("drift",  dict(lateral=3.5)),
    "CAM_06_HERO_DEFECT":    ("push",   dict(push_pct=0.14)),
    "CAM_12_LOOSE_BOLT":     ("push",   dict(push_pct=0.10)),
}

MIN_CLEARANCE = 0.45     # metres of geometry standoff we refuse to eat into
CLEARANCE_FRACTION = 0.35  # never spend more than this much of the free run


def shot_range(index):
    """Global timeline frames (start, end) for shot `index`."""
    start = index * SHOT_FRAMES + 1
    return start, start + SHOT_FRAMES - 1


# ---------------------------------------------------------------------------
# geometry probing
# ---------------------------------------------------------------------------

def _cast(scene, dg, origin, direction):
    d = direction.normalized()
    hit, loc, _n, _i, obj, _m = scene.ray_cast(dg, origin + d * 0.02, d)
    if not hit:
        return float("inf"), None
    return (Vector(loc) - origin).length, (obj.name if obj else None)


def clearance_at(scene, dg, point):
    """Shortest distance from `point` to any geometry, sampled on 6 axes."""
    best = float("inf")
    for d in (Vector((1, 0, 0)), Vector((-1, 0, 0)), Vector((0, 1, 0)),
              Vector((0, -1, 0)), Vector((0, 0, 1)), Vector((0, 0, -1))):
        dist, _ = _cast(scene, dg, point, d)
        best = min(best, dist)
    return best


def probe_camera(scene, dg, cam):
    """What is this camera looking at, and how much room does it have?"""
    mw = cam.matrix_world
    origin = mw.translation.copy()
    rot = mw.to_3x3()
    fwd = (rot @ Vector((0, 0, -1))).normalized()

    center_dist, center_obj = _cast(scene, dg, origin, fwd)

    # frustum corners, pulled in to 70% so we sample what is actually on screen
    corner_dists = []
    try:
        for v in cam.data.view_frame(scene=scene):
            cd = (rot @ (Vector(v) * 0.7)).normalized()
            dist, _ = _cast(scene, dg, origin, cd)
            corner_dists.append(dist)
    except Exception:
        pass

    finite = [d for d in ([center_dist] + corner_dists) if math.isfinite(d)]
    min_dist = min(finite) if finite else float("inf")

    subject_dist = center_dist if math.isfinite(center_dist) else (
        min(finite) if finite else 40.0)

    return {
        "camera": cam.name,
        "origin": [round(v, 3) for v in origin],
        "forward": [round(v, 4) for v in fwd],
        "center_dist": None if math.isinf(center_dist) else round(center_dist, 3),
        "center_hit": center_obj,
        "min_frustum_dist": None if math.isinf(min_dist) else round(min_dist, 3),
        "subject_dist": round(subject_dist, 3),
        "_origin": origin,
        "_fwd": fwd,
        "_rot": rot,
        "_min": min_dist,
        "_subject": subject_dist,
    }


def safe_push(probe, requested):
    """Clamp a forward dolly to the measured free run in front of the lens."""
    free = probe["_min"]
    if math.isinf(free):
        return requested, "no geometry ahead; unclamped"
    budget = max(0.0, min(free * CLEARANCE_FRACTION, free - MIN_CLEARANCE))
    if requested <= budget:
        return requested, f"fits (budget {budget:.2f} m of {free:.2f} m free)"
    return budget, f"CLAMPED {requested:.2f}->{budget:.2f} m (only {free:.2f} m ahead)"


# ---------------------------------------------------------------------------
# camera animation
# ---------------------------------------------------------------------------

def _key(cam, frame, loc=None, rot=None, focus=None):
    if loc is not None:
        cam.location = loc
        cam.keyframe_insert("location", frame=frame)
    if rot is not None:
        cam.rotation_euler = rot
        cam.keyframe_insert("rotation_euler", frame=frame)
    if focus is not None:
        cam.data.dof.focus_distance = focus
        cam.data.dof.keyframe_insert("focus_distance", frame=frame)


def _smooth(cam):
    """Ease in / ease out via explicitly flattened bezier handles.

    AUTO_CLAMPED is NOT enough here: on a two-key curve Blender aims the auto
    handles straight at each other, which reproduces linear motion exactly
    (verified - the easing silently did nothing). Forcing the handles flat at
    both ends is what actually produces the slow-in / slow-out S-curve.
    """
    for holder in (cam, cam.data):
        ad = holder.animation_data
        if not ad or not ad.action:
            continue
        for fc in ad.action.fcurves:
            kps = fc.keyframe_points
            if len(kps) < 2:
                continue
            for kp in kps:
                kp.interpolation = "BEZIER"
            first, last = kps[0], kps[-1]
            span = max(1.0, last.co.x - first.co.x)
            ease = span * 0.4        # how much of the shot is spent accelerating
            for kp, inward in ((first, +1), (last, -1)):
                kp.handle_left_type = "FREE"
                kp.handle_right_type = "FREE"
                y = kp.co.y
                if inward > 0:
                    kp.handle_right = (kp.co.x + ease, y)
                    kp.handle_left = (kp.co.x - span * 0.1, y)
                else:
                    kp.handle_left = (kp.co.x - ease, y)
                    kp.handle_right = (kp.co.x + span * 0.1, y)
            fc.update()


def apply_base_offset(scene, cam):
    """Reposition a camera before probing, for shots framed too tight to read."""
    back = BASE_OFFSETS.get(cam.name)
    if not back:
        return None
    before = cam.matrix_world.translation.copy()
    away = (cam.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
    cam.location = before + away * back
    # matrix_world is stale until the view layer re-evaluates, and every probe
    # below reads it, so refresh before measuring anything.
    bpy.context.view_layer.update()
    return {"moved_back_m": back,
            "from": [round(v, 3) for v in before],
            "to": [round(v, 3) for v in cam.matrix_world.translation]}


def animate_camera(scene, dg, cam, index, report):
    kind, params = MOVES.get(cam.name, ("push", dict(push_pct=0.08)))
    f0, f1 = shot_range(index)
    offset_info = apply_base_offset(scene, cam)
    if offset_info:
        dg = bpy.context.evaluated_depsgraph_get()
    p = probe_camera(scene, dg, cam)

    start_loc = p["_origin"].copy()
    start_rot = cam.rotation_euler.copy()
    end_loc = start_loc.copy()
    end_rot = start_rot.copy()
    notes = []

    if kind in ("push", "rise") and params.get("push_pct"):
        want = p["_subject"] * params["push_pct"]
        got, why = safe_push(p, want)
        end_loc = start_loc + p["_fwd"] * got
        notes.append(f"dolly {got:.2f} m fwd ({params['push_pct']*100:.0f}% of "
                     f"{p['_subject']:.1f} m) - {why}")

    if kind == "rise" or params.get("rise"):
        rise = params.get("rise", 0.0)
        cand = end_loc + Vector((0, 0, rise))
        room = clearance_at(scene, dg, cand)
        if room < MIN_CLEARANCE:
            rise *= 0.3
            cand = end_loc + Vector((0, 0, rise))
            notes.append(f"vertical move reduced, only {room:.2f} m clearance")
        end_loc = cand
        if abs(rise) > 1e-6:
            notes.append(f"{'rise' if rise > 0 else 'descend'} {abs(rise):.2f} m")

    if kind == "drift":
        lateral = params["lateral"]
        right = (p["_rot"] @ Vector((1, 0, 0))).normalized()
        cand = start_loc + right * lateral
        room = clearance_at(scene, dg, cand)
        if room < MIN_CLEARANCE:
            lateral *= max(0.2, room / max(MIN_CLEARANCE, 1e-6) * 0.5)
            cand = start_loc + right * lateral
            notes.append(f"lateral reduced, {room:.2f} m clearance at target")
        end_loc = cand
        notes.append(f"parallax drift {lateral:+.2f} m lateral")

    if kind == "pan":
        yaw = math.radians(params["yaw_deg"])
        end_rot = start_rot.copy()
        end_rot.z += yaw
        notes.append(f"yaw pan {params['yaw_deg']:+.2f} deg")

    if kind == "orbit":
        theta = math.radians(params["orbit_deg"])
        subject = start_loc + p["_fwd"] * p["_subject"]
        rel = start_loc - subject
        c, s = math.cos(theta), math.sin(theta)
        rel_rot = Vector((rel.x * c - rel.y * s, rel.x * s + rel.y * c, rel.z))
        cand = subject + rel_rot
        room = clearance_at(scene, dg, cand)
        if room < MIN_CLEARANCE:
            theta *= 0.3
            c, s = math.cos(theta), math.sin(theta)
            rel_rot = Vector((rel.x * c - rel.y * s, rel.x * s + rel.y * c, rel.z))
            cand = subject + rel_rot
            notes.append(f"orbit reduced, {room:.2f} m clearance at target")
        end_loc = cand
        end_rot = start_rot.copy()
        end_rot.z += theta
        notes.append(f"orbit {math.degrees(theta):+.2f} deg around subject at "
                     f"{p['_subject']:.1f} m")

    # --- depth of field ---------------------------------------------------
    dof_on = cam.name in DOF_CAMS
    cam.data.dof.use_dof = dof_on
    focus_start = focus_end = None
    if dof_on:
        cam.data.dof.aperture_fstop = DOF_CAMS[cam.name]
        cam.data.dof.aperture_blades = 7
        focus_start = p["_subject"]
        # focus tracks the dolly so the defect stays sharp while the frame moves
        focus_end = max(0.1, p["_subject"] - (end_loc - start_loc).dot(p["_fwd"]))
        notes.append(f"DoF f/{DOF_CAMS[cam.name]} focus {focus_start:.2f}->"
                     f"{focus_end:.2f} m")

    _key(cam, f0, loc=start_loc, rot=start_rot, focus=focus_start)
    _key(cam, f1, loc=end_loc, rot=end_rot, focus=focus_end)
    _smooth(cam)

    travel = (end_loc - start_loc).length
    rot_delta = math.degrees(abs(end_rot.z - start_rot.z))
    report.append({
        "shot": index + 1,
        "camera": cam.name,
        "frames": [f0, f1],
        "move": kind,
        "travel_m": round(travel, 3),
        "yaw_deg": round(rot_delta, 3),
        "lens_mm": cam.data.lens,
        "subject_dist_m": p["center_dist"],
        "subject_hit": p["center_hit"],
        "min_frustum_clearance_m": p["min_frustum_dist"],
        "dof": (f"f/{DOF_CAMS[cam.name]}" if dof_on else "deep focus"),
        "base_offset": offset_info,
        "notes": notes,
    })


def clear_camera_anim(cam):
    cam.animation_data_clear()
    if cam.data.animation_data:
        cam.data.animation_data_clear()


# ---------------------------------------------------------------------------
# look
# ---------------------------------------------------------------------------

def setup_look(scene, taa=64, motion_blur=True, volumetrics=False):
    """EEVEE realism pass. Returns a dict of what actually took effect."""
    applied = {}
    scene.render.engine = "BLENDER_EEVEE"
    ee = scene.eevee

    def setattr_report(obj, attr, value, key=None):
        key = key or attr
        if not hasattr(obj, attr):
            applied[key] = "UNAVAILABLE in this build"
            return
        try:
            setattr(obj, attr, value)
            applied[key] = getattr(obj, attr)
        except Exception as e:
            applied[key] = f"FAILED: {e}"

    # sampling
    setattr_report(ee, "taa_render_samples", taa)
    setattr_report(ee, "use_taa_reprojection", True)

    # ambient occlusion - the biggest single realism win on untextured geometry
    setattr_report(ee, "use_gtao", True)
    setattr_report(ee, "gtao_distance", 2.5)
    setattr_report(ee, "gtao_factor", 1.0)
    setattr_report(ee, "use_gtao_bent_normals", True)
    setattr_report(ee, "use_gtao_bounce", True)

    # screen space reflections - water + steel
    setattr_report(ee, "use_ssr", True)
    setattr_report(ee, "use_ssr_refraction", True)
    setattr_report(ee, "use_ssr_halfres", False)
    setattr_report(ee, "ssr_quality", 0.75)
    setattr_report(ee, "ssr_max_roughness", 0.6)
    setattr_report(ee, "ssr_thickness", 0.2)

    # shadows
    setattr_report(ee, "shadow_cube_size", "2048")
    setattr_report(ee, "shadow_cascade_size", "4096")
    setattr_report(ee, "use_soft_shadows", True)
    setattr_report(ee, "use_shadow_high_bitdepth", True)
    setattr_report(ee, "light_threshold", 0.005)

    # very subtle bloom; heavy bloom is the fastest way to look like a game demo
    setattr_report(ee, "use_bloom", True)
    setattr_report(ee, "bloom_intensity", 0.025)
    setattr_report(ee, "bloom_threshold", 1.0)
    setattr_report(ee, "bloom_radius", 4.0)

    # depth of field quality
    setattr_report(ee, "bokeh_max_size", 24.0)
    setattr_report(ee, "use_bokeh_high_quality_slight_defocus", True)
    setattr_report(ee, "use_bokeh_jittered", True)
    setattr_report(ee, "bokeh_overblur", 5.0)

    # motion blur
    setattr_report(ee, "use_motion_blur", bool(motion_blur))
    setattr_report(ee, "motion_blur_shutter", 0.5)
    setattr_report(ee, "motion_blur_steps", 3)
    setattr_report(ee, "motion_blur_position", "CENTER")

    if volumetrics:
        setattr_report(ee, "use_volumetric_lights", True)
        setattr_report(ee, "volumetric_samples", 48)
        setattr_report(ee, "volumetric_start", 1.0)
        setattr_report(ee, "volumetric_end", 400.0)

    # sun shadow cascades need to cover a ~350 m span
    sun = bpy.data.objects.get("SUN_KEY")
    if sun and sun.type == "LIGHT":
        sun.data.use_shadow = True
        for attr, val in (("shadow_cascade_max_distance", 400.0),
                          ("shadow_cascade_count", 4),
                          ("shadow_cascade_exponent", 0.8),
                          ("shadow_buffer_bias", 0.02),
                          ("angle", math.radians(1.5))):
            if hasattr(sun.data, attr):
                try:
                    setattr(sun.data, attr, val)
                    applied[f"sun.{attr}"] = getattr(sun.data, attr)
                except Exception as e:
                    applied[f"sun.{attr}"] = f"FAILED: {e}"
        applied["sun.energy"] = sun.data.energy

    # colour management
    vs = scene.view_settings
    try:
        vs.view_transform = "AgX"
        applied["view_transform"] = vs.view_transform
    except Exception as e:
        try:
            vs.view_transform = "Filmic"
            applied["view_transform"] = vs.view_transform + " (AgX unavailable)"
        except Exception:
            applied["view_transform"] = f"FAILED: {e}"
    # Measured, not guessed: at the scene's stored -1.1 the darkest shots sat at
    # a median luminance of ~0.10 against an ~0.18 mid-grey target. -0.5 brings
    # CAM_01 to 0.164 and the only new clipping is sky in the backlit shots,
    # which is correct for a low sun. The lighting itself is untouched.
    vs.exposure = -0.5
    applied["exposure"] = vs.exposure
    applied["look"] = vs.look

    return applied


def setup_output(scene, res_pct=100):
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = res_pct
    scene.render.fps = FPS
    scene.render.film_transparent = False


def build(scene, taa=64, motion_blur=True, volumetrics=False, res_pct=100):
    """Full cinematic setup. Returns (look_report, shot_report)."""
    look = setup_look(scene, taa=taa, motion_blur=motion_blur,
                      volumetrics=volumetrics)
    setup_output(scene, res_pct=res_pct)

    dg = bpy.context.evaluated_depsgraph_get()
    shots = []
    for i, name in enumerate(SHOT_ORDER):
        cam = bpy.data.objects.get(name)
        if cam is None:
            shots.append({"shot": i + 1, "camera": name, "error": "NOT FOUND"})
            continue
        clear_camera_anim(cam)
        animate_camera(scene, dg, cam, i, shots)

    scene.frame_start = 1
    scene.frame_end = len(SHOT_ORDER) * SHOT_FRAMES
    scene.camera = bpy.data.objects.get(SHOT_ORDER[0])
    return look, shots

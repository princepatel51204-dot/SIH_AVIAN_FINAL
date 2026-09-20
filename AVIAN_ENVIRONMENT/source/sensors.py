"""Sensor anchor frames for the future AVIAN UAV.

THERE IS NO DRONE HERE.
What this builds is a set of named reference frames -- Empties with axes --
arranged in the body frame a later AVIAN airframe will use, each carrying the
optical and geometric parameters the simulation layer needs to instantiate a
real sensor. Building a placeholder quadrotor would be worse than useless: it
would fix an airframe that has not been designed yet, and every downstream
system would silently inherit its dimensions.

BODY FRAME
----------
Standard aerospace body convention mapped into the scene's Z-up world:

    +X body   forward (direction of flight)
    +Y body   left
    +Z body   up

The rig root `AVI_SENSOR_RIG` is parked at a documented pose in the research
zone so the frames can be inspected in context. Moving the root moves every
sensor with it; the offsets between them are what matters, not where the rig
happens to sit.

Each frame's -Z axis is its optical axis, matching Blender's camera
convention, so a camera parented to a frame points where the frame says it
points with no extra rotation.

WHY THE NUMBERS ARE WHAT THEY ARE
---------------------------------
Every FOV, range and resolution below is a *specification for a sensor that
could actually be bought and flown on a sub-30 kg multirotor*, not a wish.
The depth camera's 0.3-8 m range and the LiDAR's 0.1 m range accuracy are the
reason cracks are shader decals and spalls are geometry: a 0.2 mm crack is
three orders of magnitude below what either instrument resolves.
"""
from __future__ import annotations
import math

import bpy
from mathutils import Vector, Euler

import params as P
import meshlib as ML


# name, offset (x,y,z) in body frame, (pitch_deg, yaw_deg), spec dict
#   pitch: positive tilts the optical axis DOWN from horizontal
#   yaw:   positive rotates to the left (+Y)
SENSOR_SPECS = [
    ("AVI_SENSOR_RGB_FRONT", (0.22, 0.0, -0.02), (0.0, 0.0), {
        "avi_sensor_class": "RGB_CAMERA",
        "avi_purpose": "primary inspection imagery, forward standoff work",
        "avi_hfov_deg": 69.0,
        "avi_vfov_deg": 42.0,
        "avi_resolution_px": [1920, 1080],
        "avi_focal_length_mm": 24.0,
        "avi_sensor_width_mm": 23.5,
        "avi_min_range_m": 0.35,
        "avi_max_useful_range_m": 12.0,
        "avi_gsd_mm_at_1m": 0.62,
        "avi_rolling_shutter": True,
        "avi_notes": "rolling shutter is a real constraint: image smear "
                     "scales with angular rate, so this sensor is only "
                     "trustworthy in a stabilised hover",
    }),
    ("AVI_SENSOR_RGB_DOWN", (0.02, 0.0, -0.11), (90.0, 0.0), {
        "avi_sensor_class": "RGB_CAMERA",
        "avi_purpose": "downward optical flow and deck-surface imagery",
        "avi_hfov_deg": 82.0,
        "avi_vfov_deg": 52.0,
        "avi_resolution_px": [1280, 800],
        "avi_focal_length_mm": 16.0,
        "avi_sensor_width_mm": 23.5,
        "avi_min_range_m": 0.25,
        "avi_max_useful_range_m": 25.0,
        "avi_gsd_mm_at_1m": 1.40,
        "avi_rolling_shutter": False,
        "avi_notes": "global shutter for flow. Fails over open water and "
                     "over the untextured deck soffit -- both present here",
    }),
    ("AVI_SENSOR_RGB_SIDE", (0.05, -0.19, -0.02), (12.0, -90.0), {
        "avi_sensor_class": "RGB_CAMERA",
        "avi_purpose": "lateral surface inspection while translating along "
                       "a girder web or pier face",
        "avi_hfov_deg": 69.0,
        "avi_vfov_deg": 42.0,
        "avi_resolution_px": [1920, 1080],
        "avi_focal_length_mm": 24.0,
        "avi_sensor_width_mm": 23.5,
        "avi_min_range_m": 0.35,
        "avi_max_useful_range_m": 10.0,
        "avi_gsd_mm_at_1m": 0.62,
        "avi_optional": True,
    }),
    ("AVI_SENSOR_DEPTH", (0.20, 0.0, 0.04), (0.0, 0.0), {
        "avi_sensor_class": "DEPTH_CAMERA",
        "avi_purpose": "close-range obstacle avoidance and spall geometry",
        "avi_hfov_deg": 87.0,
        "avi_vfov_deg": 58.0,
        "avi_resolution_px": [848, 480],
        "avi_min_range_m": 0.30,
        "avi_max_useful_range_m": 8.0,
        "avi_depth_accuracy_pct": 2.0,
        "avi_baseline_mm": 50.0,
        "avi_notes": "stereo baseline sets the minimum range. Inside a "
                     "3.8 m girder bay the far wall is at the edge of the "
                     "useful range and the near wall is inside the minimum",
    }),
    ("AVI_SENSOR_LIDAR", (0.0, 0.0, 0.13), (0.0, 0.0), {
        "avi_sensor_class": "LIDAR",
        "avi_purpose": "structural mapping, SLAM, clearance measurement",
        "avi_hfov_deg": 360.0,
        "avi_vfov_deg": 59.0,
        "avi_channels": 32,
        "avi_min_range_m": 0.10,
        "avi_max_useful_range_m": 70.0,
        "avi_range_accuracy_m": 0.03,
        "avi_points_per_second": 640000,
        "avi_notes": "will not see any crack in this model and is not "
                     "supposed to. It sees spalls, the structure, and the "
                     "absence of returns off water",
    }),
    ("AVI_SENSOR_THERMAL", (0.20, 0.10, -0.02), (0.0, 0.0), {
        "avi_sensor_class": "THERMAL_CAMERA",
        "avi_purpose": "delamination detection by thermal contrast -- the "
                       "one modality that can find a defect this model "
                       "renders as visually almost nothing",
        "avi_hfov_deg": 57.0,
        "avi_vfov_deg": 44.0,
        "avi_resolution_px": [640, 512],
        "avi_min_range_m": 0.50,
        "avi_max_useful_range_m": 15.0,
        "avi_netd_mk": 50.0,
        "avi_optional": True,
        "avi_notes": "the scene carries no thermal emission model, so this "
                     "frame is a mounting definition only",
    }),
    ("AVI_SENSOR_IMU", (0.0, 0.0, 0.0), (0.0, 0.0), {
        "avi_sensor_class": "IMU",
        "avi_purpose": "body reference frame and inertial origin. Every "
                       "other sensor offset is measured from here",
        "avi_rate_hz": 400.0,
        "avi_is_body_origin": True,
    }),
]


def build(coll, log=print):
    """Create the sensor rig at a documented pose in the research zone."""
    # Parked in SECTOR_C, off the west side of the deck at working standoff,
    # at a height where all seven frames sit clear of the structure.
    xm = P.RESEARCH_X0 + 2.5 * P.SECTOR_LENGTH
    root_pos = (xm,
                P.DECK_WIDTH / 2.0 + P.AIRSPACE_INSPECTION_OFFSET,
                P.soffit_z(xm) - 2.0)

    root = bpy.data.objects.new("AVI_SENSOR_RIG", None)
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 1.2
    root.location = root_pos
    coll.objects.link(root)
    ML.set_custom(root, {
        "avi_kind": "sensor_rig_root",
        "avi_object_type": "SENSOR_RIG",
        "avi_object_id": "AVI_SENSOR_RIG",
        "avi_body_frame": "+X forward, +Y left, +Z up",
        "avi_optical_convention": "each frame's -Z is its optical axis",
        "avi_sector": P.sector_at(xm) or "OUTSIDE_RESEARCH_ZONE",
        "avi_note": "reference frames only -- no airframe is modelled",
        "avi_sensor_count": len(SENSOR_SPECS),
    })

    made = []
    for name, off, (pitch, yaw), spec in SENSOR_SPECS:
        ob = bpy.data.objects.new(name, None)
        ob.empty_display_type = "SINGLE_ARROW" if spec[
            "avi_sensor_class"] != "IMU" else "PLAIN_AXES"
        ob.empty_display_size = 0.45
        ob.parent = root
        ob.location = off
        # -Z is the optical axis. A frame with zero rotation looks straight
        # DOWN in Blender, so a forward-looking sensor is pitched up 90 deg
        # from that, and `pitch` then tips it back down toward the surface.
        ob.rotation_euler = Euler(
            (math.radians(90.0 - pitch), 0.0, math.radians(yaw)), "XYZ")
        coll.objects.link(ob)
        d = dict(spec)
        d.update({
            "avi_kind": "sensor_frame",
            "avi_object_type": "SENSOR_FRAME",
            "avi_object_id": name,
            "avi_body_offset_m": [round(v, 4) for v in off],
            "avi_mount_pitch_deg": pitch,
            "avi_mount_yaw_deg": yaw,
        })
        ML.set_custom(ob, d)
        made.append(ob)

    log(f"  sensors : {len(made)} sensor frames on 1 rig root at "
        f"x={root_pos[0]:.0f}")
    return {"rig": root, "frames": made}


def export_manifest(path):
    """Write the sensor rig to JSON, offsets resolved in the body frame."""
    import json
    out = {"body_frame": "+X forward, +Y left, +Z up (Z-up world)",
           "optical_convention": "-Z of each frame is the optical axis",
           "note": "reference frames only; no airframe is modelled",
           "rig_world_position_m": None,
           "sensors": []}
    root = bpy.data.objects.get("AVI_SENSOR_RIG")
    if root is not None:
        out["rig_world_position_m"] = [round(v, 3) for v in root.location]
    for ob in bpy.data.objects:
        if ob.get("avi_kind") != "sensor_frame":
            continue
        rec = {k: (list(ob[k]) if hasattr(ob[k], "__len__")
                   and not isinstance(ob[k], str) else ob[k])
               for k in ob.keys() if k.startswith("avi_")}
        rec["name"] = ob.name
        rec["world_position_m"] = [round(v, 3) for v in ob.matrix_world.
                                   translation]
        rec["optical_axis_world"] = [
            round(v, 4) for v in (ob.matrix_world.to_3x3()
                                  @ Vector((0.0, 0.0, -1.0)))]
        out["sensors"].append(rec)
    out["sensors"].sort(key=lambda r: r["name"])
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return len(out["sensors"])

"""Physically plausible daylight baseline.

One sun lamp for the direct component, a Nishita sky for ambient, and no
double-counting between them. Deliberately NOT cinematic: no fill lights, no
rim lights, no artificial bounce. Anything that lifts the bridge underside
would destroy the exact contrast condition the inspection research depends on.
"""
from __future__ import annotations
import math
import bpy

import params as P


def build(coll, log=print):
    sc = bpy.context.scene
    el = math.radians(P.SUN_ELEVATION_DEG)
    az = math.radians(P.SUN_ROTATION_DEG)

    # ---- direct sun ------------------------------------------------------
    ld = bpy.data.lights.new("SUN_KEY", "SUN")
    ld.energy = P.SUN_STRENGTH
    ld.angle = math.radians(P.SUN_ANGLE_DEG)
    ld.color = (1.0, 0.965, 0.912)
    sun = bpy.data.objects.new("SUN_KEY", ld)
    sun.rotation_euler = (math.radians(90.0) - el, 0.0, az)
    coll.objects.link(sun)
    sun["avi_kind"] = "sun"
    sun["avi_elevation_deg"] = P.SUN_ELEVATION_DEG
    sun["avi_azimuth_deg"] = P.SUN_ROTATION_DEG

    # ---- sky (ambient only) ---------------------------------------------
    w = bpy.data.worlds.get("AVIAN_SKY") or bpy.data.worlds.new("AVIAN_SKY")
    sc.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    sky = nt.nodes.new("ShaderNodeTexSky")
    sky.location = (-600, 0)
    sky.sky_type = "NISHITA"
    sky.sun_elevation = el
    sky.sun_rotation = az
    sky.sun_disc = False            # the lamp above IS the sun
    sky.dust_density = P.SKY_DUST
    if hasattr(sky, "ozone_density"):
        sky.ozone_density = P.SKY_OZONE
    # The Nishita model is a SKY, not an environment: everything below the
    # horizon is pure black. An aerial camera therefore sees a void wherever
    # the terrain runs out. Blend the sky into a ground-haze colour below the
    # horizon so the world always closes.
    tc = nt.nodes.new("ShaderNodeTexCoord")
    tc.location = (-1100, -320)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-920, -320)
    # In a world shader the Object output carries the view direction, so its
    # Z component is sin(elevation) -- exactly the horizon test we need.
    nt.links.new(tc.outputs["Object"], sep.inputs["Vector"])
    blend = math.sin(math.radians(P.HAZE_BLEND_DEG))
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.location = (-720, -320)
    el0 = ramp.color_ramp.elements
    el0[0].position = -blend
    el0[0].color = (1, 1, 1, 1)
    el0[1].position = blend
    el0[1].color = (0, 0, 0, 1)
    nt.links.new(sep.outputs["Z"], ramp.inputs["Fac"])

    haze = nt.nodes.new("ShaderNodeRGB")
    haze.location = (-720, -520)
    haze.outputs[0].default_value = (*P.HAZE_COLOR, 1.0)

    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.location = (-450, -120)
    nt.links.new(ramp.outputs["Color"], mix.inputs["Factor"])
    nt.links.new(sky.outputs["Color"], mix.inputs[6])
    nt.links.new(haze.outputs[0], mix.inputs[7])

    bg = nt.nodes.new("ShaderNodeBackground")
    bg.location = (-200, 0)
    bg.inputs["Strength"].default_value = P.SKY_BACKGROUND_STRENGTH
    out = nt.nodes.new("ShaderNodeOutputWorld")
    out.location = (60, 0)
    nt.links.new(mix.outputs[2], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])

    log(f"  sun     : {P.SUN_ELEVATION_DEG:.0f} deg elevation, "
        f"{P.SUN_ROTATION_DEG:.0f} deg azimuth, {P.SUN_STRENGTH} W/m2")
    log(f"  sky     : Nishita, disc off, ambient {P.SKY_BACKGROUND_STRENGTH}")
    return sun


def configure_render(samples=None, w=None, h=None):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples or P.RENDER_SAMPLES
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 6
    sc.cycles.diffuse_bounces = 3
    sc.cycles.glossy_bounces = 3
    sc.cycles.transmission_bounces = 4
    sc.cycles.transparent_max_bounces = 8
    sc.render.resolution_x = w or P.RENDER_W
    sc.render.resolution_y = h or P.RENDER_H
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.exposure = P.RENDER_EXPOSURE
    return sc

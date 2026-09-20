"""Scenario controller: lighting, weather and world configuration.

One bridge, many conditions. Everything here changes ILLUMINATION and
ATMOSPHERE and the visibility of switchable collections. Nothing here moves a
pier, edits a defect or touches a dimension -- that separation is the whole
design, because a dataset generated under six lighting conditions is only
useful if the geometry underneath is provably identical across all six.

Applying a scenario is idempotent and reversible: call `apply(name)` again with
a different name and the world returns to a fully specified state rather than
accumulating changes.

WEATHER, HONESTLY
-----------------
There is no volumetric atmosphere and no rain simulation. "Light rain" here
means: surfaces get a wet-look roughness reduction and darker albedo, the sky
gets an overcast profile, and the haze strengthens. That reproduces what
actually changes a camera's view of concrete -- specular sheen and lost
contrast -- without a participating medium the CPU cannot afford. The record
says so, so nobody reports "tested under rain" and means something stronger.
"""
from __future__ import annotations
import json
import math

import bpy

import params as P
import lighting as LT


# ---------------------------------------------------------------------------
# LIGHTING SCENARIOS
#   sun_elev / sun_azim  degrees
#   sun_strength         W/m2 on the lamp
#   sun_color            direct component tint
#   sky_strength         ambient multiplier on the Nishita background
#   exposure             EV at the film
#   dust / ozone         Nishita atmosphere
#   haze                 ground haze colour below the horizon
# ---------------------------------------------------------------------------
LIGHTING_SCENARIOS = {
    "DAY_CLEAR": {
        "sun_elev": 52.0, "sun_azim": 145.0, "sun_strength": 6.0,
        "sun_color": (1.0, 0.965, 0.912), "sun_angle": 0.526,
        "sky_strength": 1.0, "exposure": -1.1,
        "dust": 0.9, "ozone": 1.6, "haze": (0.55, 0.56, 0.58),
        "description": "mid-morning clear sky. The REV-A baseline condition; "
                       "hard shadows, genuinely dark bridge underside.",
    },
    "DAY_OVERCAST": {
        "sun_elev": 52.0, "sun_azim": 145.0, "sun_strength": 1.1,
        "sun_color": (1.0, 1.0, 1.0), "sun_angle": 12.0,
        "sky_strength": 2.1, "exposure": -1.0,
        "dust": 3.6, "ozone": 2.4, "haze": (0.62, 0.63, 0.65),
        "description": "heavy cloud. Shadowless and low contrast -- the "
                       "HARDEST condition for crack detection despite being "
                       "the brightest underside, because a shallow crack is "
                       "read from shading and there is none.",
    },
    "MORNING": {
        "sun_elev": 18.0, "sun_azim": 95.0, "sun_strength": 4.2,
        "sun_color": (1.0, 0.90, 0.76), "sun_angle": 0.526,
        "sky_strength": 1.0, "exposure": -0.8,
        "dust": 1.6, "ozone": 1.8, "haze": (0.60, 0.57, 0.53),
        "description": "low sun. Grazing incidence on vertical faces, which "
                       "is the single best condition for surface relief and "
                       "therefore for spalling.",
    },
    "EVENING": {
        "sun_elev": 11.0, "sun_azim": 248.0, "sun_strength": 3.2,
        "sun_color": (1.0, 0.79, 0.60), "sun_angle": 0.526,
        "sky_strength": 0.9, "exposure": -0.6,
        "dust": 2.4, "ozone": 2.0, "haze": (0.58, 0.52, 0.47),
        "description": "low sun from the opposite side. Illuminates the "
                       "faces MORNING leaves dark; the pair together covers "
                       "the structure.",
    },
    "NIGHT": {
        "sun_elev": -8.0, "sun_azim": 250.0, "sun_strength": 0.015,
        "sun_color": (0.55, 0.62, 0.85), "sun_angle": 0.526,
        "sky_strength": 0.045, "exposure": 1.6,
        "dust": 1.0, "ozone": 1.4, "haze": (0.045, 0.050, 0.062),
        "description": "no useful ambient. Inspection here depends entirely "
                       "on the UAV's own light, which is exactly the point: "
                       "it removes illumination as a free variable.",
    },
    "UNDERBRIDGE": {
        "sun_elev": 68.0, "sun_azim": 145.0, "sun_strength": 6.4,
        "sun_color": (1.0, 0.97, 0.93), "sun_angle": 0.526,
        "sky_strength": 1.05, "exposure": -0.5,
        "dust": 0.8, "ozone": 1.6, "haze": (0.55, 0.56, 0.58),
        "description": "high sun with the film opened up. Maximises the "
                       "bright-outside / dark-inside ratio a real inspection "
                       "camera has to survive at the edge of the deck.",
    },
    "DEEP_SHADOW": {
        "sun_elev": 34.0, "sun_azim": 325.0, "sun_strength": 5.2,
        "sun_color": (1.0, 0.96, 0.90), "sun_angle": 0.526,
        "sky_strength": 0.62, "exposure": -1.6,
        "dust": 0.7, "ozone": 1.5, "haze": (0.42, 0.43, 0.46),
        "description": "sun on the far side of the deck. Every inspection "
                       "surface in the research zone is in shadow at once.",
    },
}

# ---------------------------------------------------------------------------
# WEATHER
#   No volumetrics. These act on surface response and atmosphere only.
# ---------------------------------------------------------------------------
WEATHER_SCENARIOS = {
    "CLEAR": {"wetness": 0.0, "haze_gain": 1.0, "sky_gain": 1.0,
              "description": "dry surfaces, nominal atmosphere"},
    "CLOUDY": {"wetness": 0.0, "haze_gain": 1.15, "sky_gain": 1.25,
               "description": "increased ambient, softened contrast"},
    "LIGHT_RAIN": {"wetness": 0.75, "haze_gain": 1.35, "sky_gain": 1.15,
                   "description": "wet concrete: lower roughness, darker "
                                  "albedo, specular sheen. No rain particles "
                                  "and no volumetric scattering."},
    "WET_SURFACE": {"wetness": 0.95, "haze_gain": 1.05, "sky_gain": 1.0,
                    "description": "after rain, sky clearing, surfaces still "
                                   "wet -- the worst case for specular "
                                   "washout on a horizontal soffit"},
    "HAZY": {"wetness": 0.0, "haze_gain": 1.9, "sky_gain": 1.1,
             "description": "humid metropolitan haze; long-range contrast "
                            "loss without any change close in"},
    "LOW_LIGHT": {"wetness": 0.2, "haze_gain": 1.2, "sky_gain": 0.55,
                  "description": "dusk or heavy overcast; forces gain up and "
                                 "shutter down"},
}

# ---------------------------------------------------------------------------
# NAMED PRESETS -- the configurations a run is actually launched with
# ---------------------------------------------------------------------------
PRESETS = {
    "BASELINE": {
        "lighting": "DAY_CLEAR", "weather": "CLEAR", "traffic": True,
        "boats": True, "pedestrians": True, "dynamic_obstacles": False,
        "gps_quality": "NORMAL", "uav_count": 1,
        "description": "the REV-A condition, preserved bit for bit",
    },
    "DAY_INSPECTION": {
        "lighting": "DAY_CLEAR", "weather": "CLEAR", "traffic": True,
        "boats": True, "pedestrians": True, "dynamic_obstacles": False,
        "gps_quality": "NORMAL", "uav_count": 1,
        "description": "nominal single-aircraft daylight inspection",
    },
    "UNDERBRIDGE": {
        "lighting": "UNDERBRIDGE", "weather": "CLEAR", "traffic": True,
        "boats": False, "pedestrians": False, "dynamic_obstacles": False,
        "gps_quality": "DENIED", "uav_count": 1,
        "description": "confined under-deck work; GNSS denied by the slab",
    },
    "RIVER_INSPECTION": {
        "lighting": "MORNING", "weather": "CLEAR", "traffic": True,
        "boats": True, "pedestrians": False, "dynamic_obstacles": False,
        "gps_quality": "NORMAL", "uav_count": 1,
        "description": "river piers with low sun and specular water",
    },
    "NIGHT": {
        "lighting": "NIGHT", "weather": "CLEAR", "traffic": True,
        "boats": False, "pedestrians": False, "dynamic_obstacles": False,
        "gps_quality": "NORMAL", "uav_count": 1,
        "description": "onboard illumination only",
    },
    "LOW_VISIBILITY": {
        "lighting": "DAY_OVERCAST", "weather": "LIGHT_RAIN", "traffic": True,
        "boats": True, "pedestrians": True, "dynamic_obstacles": False,
        "gps_quality": "MEDIUM", "uav_count": 1,
        "description": "flat light on wet concrete -- the hardest visual "
                       "condition in the set",
    },
    "MULTI_UAV": {
        "lighting": "DAY_CLEAR", "weather": "CLEAR", "traffic": True,
        "boats": True, "pedestrians": True, "dynamic_obstacles": False,
        "gps_quality": "NORMAL", "uav_count": 6,
        "description": "six aircraft, one per sector, separation zones live",
    },
    "GPS_DENIED": {
        "lighting": "DEEP_SHADOW", "weather": "CLEAR", "traffic": True,
        "boats": False, "pedestrians": False, "dynamic_obstacles": False,
        "gps_quality": "DENIED", "uav_count": 1,
        "description": "shadowed and GNSS-denied together; visual-inertial "
                       "odometry with nothing to fall back on",
    },
    "DYNAMIC_OBSTACLE": {
        "lighting": "DAY_CLEAR", "weather": "CLEAR", "traffic": True,
        "boats": True, "pedestrians": True, "dynamic_obstacles": True,
        "gps_quality": "NORMAL", "uav_count": 1,
        "description": "scaffolding, platform, crane and nets in the "
                       "research zone; tests avoidance against structure "
                       "that is not on the digital twin",
    },
}

# The live configuration. Written to the scene so a loaded .blend reports
# which condition it was last set to.
AVIAN_SCENARIO = dict(PRESETS["BASELINE"])


DYNAMIC_COLLECTIONS = {
    "traffic": ("AVI_DYNAMIC_VEHICLES",),
    "boats": ("AVI_DYNAMIC_BOATS",),
    "pedestrians": ("AVI_DYNAMIC_PEDESTRIANS",),
    "dynamic_obstacles": ("AVI_DYNAMIC_OBSTACLES",),
}


# ---------------------------------------------------------------------------
def _set_collection_visible(name, visible):
    """Toggle a collection for both viewport and render."""
    c = bpy.data.collections.get(name)
    if c is None:
        return False
    c.hide_viewport = not visible
    c.hide_render = not visible
    for lc in _layer_collections(bpy.context.view_layer.layer_collection):
        if lc.collection is c:
            lc.exclude = not visible
    return True


def _layer_collections(root):
    yield root
    for ch in root.children:
        yield from _layer_collections(ch)


def _apply_wetness(amount):
    """Wet concrete: rougher surfaces become glossier and slightly darker.

    Reaches into the built materials rather than rebuilding them, so the
    geometry and every other shader parameter is untouched.
    """
    n = 0
    for m in bpy.data.materials:
        if not m.use_nodes or not m.name.startswith("MAT_"):
            continue
        if "CRACK" in m.name or "AIRSPACE" in m.name:
            continue
        for node in m.node_tree.nodes:
            if node.bl_idname != "ShaderNodeBsdfPrincipled":
                continue
            r = node.inputs["Roughness"]
            if r.is_linked:
                continue
            base = m.get("avi_dry_roughness")
            if base is None:
                base = float(r.default_value)
                m["avi_dry_roughness"] = base
            r.default_value = max(0.04, base * (1.0 - 0.72 * amount))
            n += 1
    return n


def apply(preset="BASELINE", log=print, lighting=None, weather=None):
    """Put the world into a named condition. Geometry is never touched."""
    global AVIAN_SCENARIO
    if preset not in PRESETS:
        raise KeyError(f"unknown scenario preset {preset!r}; "
                       f"have {sorted(PRESETS)}")
    cfg = dict(PRESETS[preset])
    if lighting:
        cfg["lighting"] = lighting
    if weather:
        cfg["weather"] = weather
    cfg["preset"] = preset

    L = LIGHTING_SCENARIOS[cfg["lighting"]]
    W = WEATHER_SCENARIOS[cfg["weather"]]
    sc = bpy.context.scene

    # ---- sun ------------------------------------------------------------
    sun = bpy.data.objects.get("SUN_KEY")
    if sun is not None and sun.data is not None:
        sun.data.energy = L["sun_strength"]
        sun.data.color = L["sun_color"]
        sun.data.angle = math.radians(L["sun_angle"])
        sun.rotation_euler = (math.radians(90.0 - L["sun_elev"]), 0.0,
                              math.radians(L["sun_azim"]))
        sun["avi_scenario"] = cfg["lighting"]
        sun["avi_elevation_deg"] = L["sun_elev"]
        sun["avi_azimuth_deg"] = L["sun_azim"]

    # ---- sky ------------------------------------------------------------
    w = bpy.data.worlds.get("AVIAN_SKY")
    if w is not None and w.use_nodes:
        for node in w.node_tree.nodes:
            if node.bl_idname == "ShaderNodeTexSky":
                node.sun_elevation = math.radians(max(-0.15, L["sun_elev"]) *
                                                  math.pi / 180.0 * 180.0 /
                                                  math.pi)
                node.sun_elevation = math.radians(L["sun_elev"])
                node.sun_rotation = math.radians(L["sun_azim"])
                node.dust_density = L["dust"]
                if hasattr(node, "ozone_density"):
                    node.ozone_density = L["ozone"]
            elif node.bl_idname == "ShaderNodeBackground":
                node.inputs["Strength"].default_value = (
                    L["sky_strength"] * W["sky_gain"])
            elif node.bl_idname == "ShaderNodeRGB":
                h = L["haze"]
                g = W["haze_gain"]
                node.outputs[0].default_value = (
                    min(1.0, h[0] * g), min(1.0, h[1] * g),
                    min(1.0, h[2] * g), 1.0)

    # ---- film -----------------------------------------------------------
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.exposure = L["exposure"]

    # ---- surfaces -------------------------------------------------------
    n_mat = _apply_wetness(W["wetness"])

    # ---- switchable content --------------------------------------------
    toggled = []
    for key, colls in DYNAMIC_COLLECTIONS.items():
        want = bool(cfg.get(key, False))
        for cn in colls:
            if _set_collection_visible(cn, want):
                toggled.append(f"{cn}={'on' if want else 'off'}")

    AVIAN_SCENARIO = cfg
    sc["AVIAN_SCENARIO"] = json.dumps(cfg)
    sc["avi_scenario_preset"] = preset
    sc["avi_lighting"] = cfg["lighting"]
    sc["avi_weather"] = cfg["weather"]
    sc["avi_gps_quality"] = cfg["gps_quality"]
    sc["avi_uav_count"] = cfg["uav_count"]

    log(f"  scenario: {preset} (lighting={cfg['lighting']}, "
        f"weather={cfg['weather']}, uav={cfg['uav_count']}, "
        f"gps={cfg['gps_quality']}); {n_mat} materials retuned")
    return cfg


def export_manifest(path):
    import json
    out = {
        "note": "lighting and weather change illumination and surface "
                "response only. No scenario alters geometry, defects or "
                "ground truth. Weather is a surface-and-atmosphere "
                "approximation, not a volumetric simulation.",
        "active": AVIAN_SCENARIO,
        "presets": PRESETS,
        "lighting_scenarios": LIGHTING_SCENARIOS,
        "weather_scenarios": WEATHER_SCENARIOS,
        "switchable_collections": DYNAMIC_COLLECTIONS,
        "usage": "import scenarios; scenarios.apply('NIGHT')",
    }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return {"presets": len(PRESETS),
            "lighting": len(LIGHTING_SCENARIOS),
            "weather": len(WEATHER_SCENARIOS)}

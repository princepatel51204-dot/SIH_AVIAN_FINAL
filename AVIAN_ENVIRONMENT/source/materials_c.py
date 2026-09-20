"""REV-C Stage 1 -- South Mumbai facade materials.

Replaces the four flat MAT_BUILDING_A..D Principled BSDFs -- base colour
plus roughness, nothing else; MAT_BUILDING_B was 7% saturated -- with a
facade shader that reads as lived-in: per-object hue drawn from a named
palette, floor banding, plaster patchiness, a window grid, and monsoon mould
streaking below sills. This is the single highest-value fix in Stage 1,
because every one of the 680 buildings shares just these four materials.

Daylight only this stage (master brief v2 S5.1): no emissive window grid.

Materials are rebuilt IN PLACE by name rather than through materials.py's
_new(), which is a fetch-or-create cache -- calling it again for an existing
name returns the flat original unchanged, since that is exactly the
idempotency build_scene_b.py's re-run relies on for everything else.
city.py already holds references to these four Material datablocks (via
mats["bldg_a"] .. mats["bldg_d"]), so rebuilding their node graphs here is
picked up with zero changes to city.py: same Python object, new graph.

No UVs exist on box() geometry (only plane(), used for crack decals, gets
one) -- consistent with the rest of materials.py, the window grid below is
built from Object-space coordinates the same way concrete()'s formwork
lines are, not from a UV-mapped texture. It is a shader-only approximation:
real per-face window recesses are geometry, and that is Stage 3's
facade_c.py, gated to the HIGH LOD band, not this stage.
"""
from __future__ import annotations
import bpy

import materials as M


# ===========================================================================
# SOUTH MUMBAI PALETTE -- named constants, each with a real-world reference.
# Linear-light RGB, matching the convention materials.py uses throughout:
# these are unlit surface reflectance values (roughly 0.03-0.6), not final
# rendered brightness.
# ===========================================================================
ART_DECO_CREAM      = (0.62, 0.56, 0.46)   # buttermilk/cream render -- Marine Drive, Oval Maidan frontage
ART_DECO_OCHRE      = (0.58, 0.44, 0.28)   # pale ochre render, banding and vertical fins
RCC_WEATHERED_GREY  = (0.30, 0.30, 0.29)   # exposed mid-century concrete, gone grey-black
RCC_WEATHERED_GREY2 = (0.235, 0.235, 0.225)  # a second, darker RCC tone -- pour-to-pour variation
RCC_MOULD_BLACK     = (0.045, 0.048, 0.042)  # monsoon mould streaking below sills/slab edges -- the most recognisable Bombay surface
CHAWL_OXIDE_RED     = (0.42, 0.16, 0.10)   # oxide-red woodwork and trim
CHAWL_OCHRE         = (0.52, 0.34, 0.16)   # chawl / walk-up wall ochre
CHAWL_GREEN         = (0.10, 0.19, 0.12)   # painted woodwork, deep green
STONE_MALAD_YELLOW  = (0.52, 0.44, 0.28)   # Malad yellow basalt
STONE_KOTA_GREY     = (0.36, 0.36, 0.34)   # Kota grey, older stock
WINDOW_GLASS_DAY    = (0.075, 0.090, 0.105)  # unlit glass, daylight only -- no emission this stage

# Each MAT_BUILDING_[A-D] biases toward one archetype but the ramp still
# reaches the whole palette at the far end of its range, so no material is
# a single flat colour and no material is locked out of the others' looks.
# Position, not just presence, sets the bias: more of the 0..1 Random range
# maps to the archetype's own tones than to the rest.
_FACADE_RAMPS = {
    "MAT_BUILDING_A": [    # Art Deco frontage stock
        (0.00, ART_DECO_CREAM), (0.55, ART_DECO_OCHRE),
        (0.72, RCC_WEATHERED_GREY), (0.86, CHAWL_OCHRE),
        (1.00, STONE_KOTA_GREY)],
    "MAT_BUILDING_B": [    # mid-century RCC, monsoon-stained
        (0.00, RCC_WEATHERED_GREY), (0.55, RCC_WEATHERED_GREY2),
        (0.75, ART_DECO_CREAM), (0.90, CHAWL_OCHRE),
        (1.00, STONE_KOTA_GREY)],
    "MAT_BUILDING_C": [    # chawls and walk-ups
        (0.00, CHAWL_OCHRE), (0.35, CHAWL_OXIDE_RED),
        (0.58, CHAWL_GREEN), (0.78, ART_DECO_CREAM),
        (1.00, RCC_WEATHERED_GREY)],
    "MAT_BUILDING_D": [    # stone-clad older stock
        (0.00, STONE_KOTA_GREY), (0.45, STONE_MALAD_YELLOW),
        (0.65, RCC_WEATHERED_GREY), (0.85, ART_DECO_OCHRE),
        (1.00, CHAWL_OCHRE)],
}

# floor-to-floor height and window pitch vary a little per archetype so the
# four prototypes' grids do not all line up identically.
_GEOMETRY = {
    "MAT_BUILDING_A": (3.4, 2.8),
    "MAT_BUILDING_B": (3.0, 2.4),
    "MAT_BUILDING_C": (2.9, 2.2),
    "MAT_BUILDING_D": (3.6, 3.0),
}


def _rgba(c):
    return (*c, 1.0)


def _rebuild(name):
    """Fetch an existing Material and clear its graph for a rebuild.

    Deliberately NOT materials._new(), which returns an existing material's
    graph unchanged -- this module's whole job is to replace that graph in
    place while keeping the same Material datablock city.py already
    references.
    """
    m = bpy.data.materials.get(name)
    if m is None:
        raise RuntimeError(f"materials_c: {name!r} must already exist -- "
                           "call materials.build_library() first")
    nt = m.node_tree
    nt.nodes.clear()
    return m, nt


def _facade(name, ramp_stops, floor_h, window_pitch):
    m, nt = _rebuild(name)
    tc = M._tc(nt, -1500, 0)
    obj_info = nt.nodes.new("ShaderNodeObjectInfo")
    obj_info.location = (-1500, 560)
    rnd = obj_info.outputs["Random"]

    # ---- per-object base tone: this building's own point in the palette --
    base_ramp = M._ramp(nt, [(p, _rgba(c)) for p, c in ramp_stops],
                        -1250, 560)
    nt.links.new(rnd, base_ramp.inputs["Fac"])

    # ---- floor banding: a faint tonal step every storey -------------------
    floors = nt.nodes.new("ShaderNodeTexWave")
    floors.location = (-1250, 300)
    floors.wave_type = "BANDS"
    floors.bands_direction = "Z"
    floors.inputs["Scale"].default_value = 1.0 / floor_h
    floors.inputs["Distortion"].default_value = 0.0
    floors.inputs["Detail"].default_value = 1.0
    nt.links.new(tc.outputs["Object"], floors.inputs["Vector"])
    r_floor = M._ramp(nt, [(0.0, (0.90, 0.90, 0.90, 1)),
                           (1.0, (1.05, 1.05, 1.05, 1))], -1000, 300)
    nt.links.new(floors.outputs["Fac"], r_floor.inputs["Fac"])
    banded = M._mix(nt, "MULTIPLY", -900, 480, 1.0)
    nt.links.new(base_ramp.outputs["Color"], banded.inputs[6])
    nt.links.new(r_floor.outputs["Color"], banded.inputs[7])

    # ---- plaster patchiness -------------------------------------------
    patch = M._noise(nt, 3.2, 8.0, 0.6, 0.3, -1250, 40)
    nt.links.new(tc.outputs["Object"], patch.inputs["Vector"])
    r_patch = M._ramp(nt, [(0.35, (0.88, 0.88, 0.86, 1)),
                           (0.65, (1.05, 1.04, 1.02, 1))], -1000, 40)
    nt.links.new(patch.outputs["Fac"], r_patch.inputs["Fac"])
    mottled = M._mix(nt, "MULTIPLY", -700, 420, 0.5)
    nt.links.new(banded.outputs[2], mottled.inputs[6])
    nt.links.new(r_patch.outputs["Color"], mottled.inputs[7])

    # ---- window grid: Object-space bands on two axes, multiplied ---------
    # An approximation, not UV-mapped geometry: box() carries no UVs, so
    # this reads correctly on faces aligned with the axis it was tuned
    # against and stretches on the perpendicular ones, same trade-off as
    # concrete()'s own formwork-line trick. Real per-face windows are
    # Stage 3 geometry (facade_c.py), gated to the HIGH LOD band only.
    wx = nt.nodes.new("ShaderNodeTexWave")
    wx.location = (-1250, -260)
    wx.wave_type = "BANDS"
    wx.bands_direction = "X"
    wx.inputs["Scale"].default_value = 1.0 / window_pitch
    wx.inputs["Distortion"].default_value = 0.0
    wx.inputs["Detail"].default_value = 1.0
    nt.links.new(tc.outputs["Object"], wx.inputs["Vector"])
    wz = nt.nodes.new("ShaderNodeTexWave")
    wz.location = (-1250, -440)
    wz.wave_type = "BANDS"
    wz.bands_direction = "Z"
    wz.inputs["Scale"].default_value = 1.0 / (floor_h * 0.55)
    wz.inputs["Distortion"].default_value = 0.0
    wz.inputs["Detail"].default_value = 1.0
    nt.links.new(tc.outputs["Object"], wz.inputs["Vector"])
    r_wx = M._ramp(nt, [(0.30, (0, 0, 0, 1)), (0.42, (1, 1, 1, 1))],
                   -1000, -260)
    nt.links.new(wx.outputs["Fac"], r_wx.inputs["Fac"])
    r_wz = M._ramp(nt, [(0.28, (0, 0, 0, 1)), (0.40, (1, 1, 1, 1))],
                   -1000, -440)
    nt.links.new(wz.outputs["Fac"], r_wz.inputs["Fac"])
    win_mask = nt.nodes.new("ShaderNodeMath")
    win_mask.location = (-800, -350)
    win_mask.operation = "MULTIPLY"
    nt.links.new(r_wx.outputs["Color"], win_mask.inputs[0])
    nt.links.new(r_wz.outputs["Color"], win_mask.inputs[1])

    windowed = M._mix(nt, "MIX", -480, 220)
    windowed.inputs[7].default_value = _rgba(WINDOW_GLASS_DAY)
    nt.links.new(win_mask.outputs[0], windowed.inputs["Factor"])
    nt.links.new(mottled.outputs[2], windowed.inputs[6])

    # ---- monsoon mould streaking below sills/slab edges (universal) ------
    map_st = M._map(nt, (1.0, 1.0, 0.10), -1500, -680)
    nt.links.new(tc.outputs["Object"], map_st.inputs["Vector"])
    n_st = M._noise(nt, 4.5, 9.0, 0.72, 1.6, -1250, -680)
    nt.links.new(map_st.outputs["Vector"], n_st.inputs["Vector"])
    r_st = M._ramp(nt, [(0.42, (0, 0, 0, 1)), (0.76, (1, 1, 1, 1))],
                   -1000, -680)
    nt.links.new(n_st.outputs["Fac"], r_st.inputs["Fac"])
    # per-object weathering amount -- some buildings freshly painted, some
    # heavily monsoon-stained. A second, independent read of the same
    # per-object Random via a ramp, not a new hash: cheap and sufficient --
    # it only needs to decorrelate from the base-tone ramp's own shape, not
    # be statistically independent of Random itself.
    r_weather = M._ramp(nt, [(0.0, (0.20, 0.20, 0.20, 1)),
                             (1.0, (0.90, 0.90, 0.90, 1))], -1250, -840)
    nt.links.new(rnd, r_weather.inputs["Fac"])
    stain_amt = nt.nodes.new("ShaderNodeMath")
    stain_amt.location = (-780, -720)
    stain_amt.operation = "MULTIPLY"
    nt.links.new(r_st.outputs["Color"], stain_amt.inputs[0])
    nt.links.new(r_weather.outputs["Color"], stain_amt.inputs[1])

    stained = M._mix(nt, "MULTIPLY", -220, 220)
    stained.inputs[7].default_value = _rgba(RCC_MOULD_BLACK)
    nt.links.new(stain_amt.outputs[0], stained.inputs["Factor"])
    nt.links.new(windowed.outputs[2], stained.inputs[6])

    # ---- shading ------------------------------------------------------
    bsdf = M._bsdf(nt, 400, 0)
    nt.links.new(M._aerial(nt, stained, 120, 340).outputs[2],
                 bsdf.inputs["Base Color"])

    r_rough = M._ramp(nt, [(0.20, (0.55, 0.55, 0.55, 1)),
                           (0.85, (0.92, 0.92, 0.92, 1))], -220, -80)
    nt.links.new(stain_amt.outputs[0], r_rough.inputs["Fac"])
    # windows read smoother/glassier than the wall even unlit -- mix toward
    # a fixed low roughness in window cells, driven by the same win_mask.
    rough_mix = nt.nodes.new("ShaderNodeMix")
    rough_mix.data_type = "FLOAT"
    rough_mix.location = (40, -80)
    rough_mix.inputs[3].default_value = 0.12
    nt.links.new(win_mask.outputs[0], rough_mix.inputs[0])
    nt.links.new(r_rough.outputs["Color"], rough_mix.inputs[2])
    nt.links.new(rough_mix.outputs[0], bsdf.inputs["Roughness"])

    bump = M._bump(nt, 0.10, 0.006, 120, -440)
    nt.links.new(patch.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])

    out = M._out(nt)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return m


def build_city_materials(log=print):
    """Rebuild MAT_BUILDING_A..D in place as South Mumbai facade shaders.

    Must run after materials.build_library(), which is what creates these
    four Material datablocks in the first place (as flat placeholders).
    Looked up by name via bpy.data.materials, not passed in, so this can run
    any time after that -- including, as build_scene_c.py does, after
    city.py has already assigned them to every building mesh. city.py needs
    no changes: it holds the same four Material datablocks by reference, and
    replacing their node graphs in place is picked up automatically.
    """
    for name, stops in _FACADE_RAMPS.items():
        floor_h, pitch = _GEOMETRY[name]
        _facade(name, stops, floor_h, pitch)
    log(f"  materials_c: {len(_FACADE_RAMPS)} facade shaders rebuilt "
        f"(South Mumbai palette, daylight only, per-object variation)")
    return {"facade_materials": len(_FACADE_RAMPS)}

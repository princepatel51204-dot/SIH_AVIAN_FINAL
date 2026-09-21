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
import params_c as PC


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

    # ---- window grid: per-face horizontal axis, then bands --------------
    # box() carries no UVs, so the horizontal "along the wall" coordinate
    # has to be derived. Driving the vertical mullions straight off Object
    # X was WRONG and visibly so: on a face whose normal IS X, that
    # coordinate is constant across the whole face, so the mask stuck at 1
    # -- no grid at all on two of the four faces, and the entire face
    # washed with the glass tint. The REV-C facade diagnostic views caught
    # it; none of the nine validation cameras gets close enough to a
    # building to have shown it.
    #
    # So: pick the horizontal axis per face from the surface normal. A face
    # normal to X runs along Y, and vice versa. Real recessed window
    # geometry is still Stage 3 (facade_c.py, HIGH LOD band only); this
    # only has to be right, not three-dimensional.
    geo_n = nt.nodes.new("ShaderNodeNewGeometry")
    geo_n.location = (-1860, -200)
    # Geometry -> Normal is in WORLD space, and city.py rotates buildings by
    # a random choice of 0 or 90 degrees. Comparing a world normal against
    # OBJECT coordinates therefore picks the wrong axis on every rotated
    # building -- which is not a subtle error: it removes the mullions
    # entirely from the faces that had them. Transform to object space so
    # the normal and the coordinate it selects are in the same frame.
    n_obj = nt.nodes.new("ShaderNodeVectorTransform")
    n_obj.location = (-1700, -200)
    n_obj.vector_type = "NORMAL"
    n_obj.convert_from = "WORLD"
    n_obj.convert_to = "OBJECT"
    nt.links.new(geo_n.outputs["Normal"], n_obj.inputs["Vector"])
    sep_n = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep_n.location = (-1520, -200)
    nt.links.new(n_obj.outputs["Vector"], sep_n.inputs["Vector"])
    abs_nx = nt.nodes.new("ShaderNodeMath")
    abs_nx.location = (-1360, -140)
    abs_nx.operation = "ABSOLUTE"
    nt.links.new(sep_n.outputs["X"], abs_nx.inputs[0])
    abs_ny = nt.nodes.new("ShaderNodeMath")
    abs_ny.location = (-1360, -260)
    abs_ny.operation = "ABSOLUTE"
    nt.links.new(sep_n.outputs["Y"], abs_ny.inputs[0])
    face_sel = nt.nodes.new("ShaderNodeMath")
    face_sel.location = (-1200, -200)
    face_sel.operation = "GREATER_THAN"     # 1.0 on an X-normal face
    nt.links.new(abs_nx.outputs[0], face_sel.inputs[0])
    nt.links.new(abs_ny.outputs[0], face_sel.inputs[1])

    sep_p = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep_p.location = (-1520, -380)
    nt.links.new(tc.outputs["Object"], sep_p.inputs["Vector"])
    u_mix = nt.nodes.new("ShaderNodeMix")
    u_mix.data_type = "FLOAT"
    u_mix.location = (-1200, -380)
    nt.links.new(face_sel.outputs[0], u_mix.inputs[0])
    nt.links.new(sep_p.outputs["X"], u_mix.inputs[2])   # Y-normal -> use X
    nt.links.new(sep_p.outputs["Y"], u_mix.inputs[3])   # X-normal -> use Y
    u_vec = nt.nodes.new("ShaderNodeCombineXYZ")
    u_vec.location = (-1040, -380)
    nt.links.new(u_mix.outputs[0], u_vec.inputs["X"])

    wx = nt.nodes.new("ShaderNodeTexWave")
    wx.location = (-880, -260)
    wx.wave_type = "BANDS"
    wx.bands_direction = "X"
    wx.inputs["Scale"].default_value = 1.0 / window_pitch
    wx.inputs["Distortion"].default_value = 0.0
    wx.inputs["Detail"].default_value = 1.0
    nt.links.new(u_vec.outputs["Vector"], wx.inputs["Vector"])
    wz = nt.nodes.new("ShaderNodeTexWave")
    wz.location = (-1250, -440)
    wz.wave_type = "BANDS"
    wz.bands_direction = "Z"
    wz.inputs["Scale"].default_value = 1.0 / (floor_h * 0.55)
    wz.inputs["Distortion"].default_value = 0.0
    wz.inputs["Detail"].default_value = 1.0
    nt.links.new(tc.outputs["Object"], wz.inputs["Vector"])
    r_wx = M._ramp(nt, [(0.30, (0, 0, 0, 1)), (0.42, (1, 1, 1, 1))],
                   -700, -260)
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


# ===========================================================================
# WEATHERING -- injected into the existing REV-A graphs, not rebuilt
# ===========================================================================
# concrete(), asphalt() and water() in materials.py are good shaders. These
# functions splice extra layers into their existing node trees rather than
# replacing them: find what currently feeds the Principled BSDF's Base
# Colour, insert a layer, relink. That keeps every REV-A decision intact and
# makes the REV-C addition a readable delta.
#
# HOST CONCRETE ONLY. MAT_DELAMINATION and MAT_REPAIR_PATCH are built by
# concrete() too (materials.py delamination_face/repair_patch) and are
# ground-truth-bearing, so they are excluded by name along with the crack,
# spall and rebar materials.
_CONCRETE_HOSTS = ("MAT_CONCRETE_DECK", "MAT_CONCRETE_GIRDER",
                   "MAT_CONCRETE_PIER", "MAT_CONCRETE_PARAPET",
                   "MAT_CONCRETE_LOD_LOW")
_NEVER_TOUCH = ("MAT_DELAMINATION", "MAT_REPAIR_PATCH", "MAT_SPALL_FACE",
                "MAT_SPALL_FACE_RUST", "MAT_REBAR_CORRODED")

# The two weathering layers' strength multipliers are named nodes so a
# sweep can retune them in place. Re-running weather_concrete() would splice
# a second copy of the whole layer into the graph; set_concrete_weather()
# just moves these two values, which is what CONCRETE_WEATHER_SWEEP needs.
_GRIME_NODE = "REVC_GRIME_STRENGTH"
_EFFL_NODE = "REVC_EFFL_STRENGTH"
GRIME_BASE = 0.85                    # grime factor at strength 1.0
EFFL_BASE = 0.45                     # efflorescence factor at strength 1.0

GRIME_DARK = (0.055, 0.052, 0.046)   # crevice grime and soot
EFFLORESCENCE = (0.72, 0.71, 0.68)   # pale leached-salt bloom at joints
ASPHALT_PATCH = (0.048, 0.048, 0.050)  # fresher bitumen of a patch repair
WATER_TURBID = (0.085, 0.072, 0.048)   # silt-loaded water near the banks


def _bsdf_of(nt):
    for n in nt.nodes:
        if n.bl_idname == "ShaderNodeBsdfPrincipled":
            return n
    return None


def _splice_base_colour(nt, make_layer):
    """Insert a layer between whatever feeds Base Colour and the BSDF.

    make_layer(nt, src_socket) must build nodes and return the socket that
    should now feed Base Colour. Returns False if the material has no
    Principled BSDF at all, in which case the caller leaves it alone.

    Where Base Colour is a flat default rather than a linked socket -- which
    is how MAT_RIVER carries its depth tint -- the existing default_value is
    materialised into an RGB node first and that becomes the source. The
    colour is preserved exactly; nothing is guessed.
    """
    bsdf = _bsdf_of(nt)
    if bsdf is None:
        return False
    inp = bsdf.inputs["Base Color"]
    if inp.is_linked:
        src = inp.links[0].from_socket
    else:
        rgb = nt.nodes.new("ShaderNodeRGB")
        rgb.location = (-1700, -500)
        rgb.outputs[0].default_value = tuple(inp.default_value)
        src = rgb.outputs[0]
    out = make_layer(nt, src)
    nt.links.new(out, inp)
    return True


def weather_concrete(strength=None, log=print):
    """Geometry-driven weathering on the host concrete.

    Two layers, both scaled by CONCRETE_WEATHER_STRENGTH:

      1. crevice grime  -- Geometry -> Pointiness, so dirt collects where the
         surface is concave: the re-entrant corners between web and flange,
         behind bearings, under the deck edge. This is the layer the brief
         asks for by name, and it is the one that costs defect contrast,
         because it is mottled at the scale defects live at.
      2. efflorescence  -- a pale leached bloom keyed to the same Z bands
         concrete() already uses for formwork lines, so it appears along
         construction joints rather than at random.

    strength is a swept corpus parameter, not an aesthetic setting -- see
    params_c.CONCRETE_WEATHER_STRENGTH.
    """
    s = PC.CONCRETE_WEATHER_STRENGTH if strength is None else float(strength)
    s = max(0.0, min(1.0, s))
    done = []
    for name in _CONCRETE_HOSTS:
        m = bpy.data.materials.get(name)
        if m is None or name in _NEVER_TOUCH:
            continue
        nt = m.node_tree

        def layer(nt, src, _s=s):
            geo = nt.nodes.new("ShaderNodeNewGeometry")
            geo.location = (-560, -900)
            # Pointiness is 0.5 on a flat face, below 0.5 in a concavity.
            # The ramp keeps only the concave side and fades it in.
            r_point = M._ramp(nt, [(0.30, (1, 1, 1, 1)),
                                   (0.50, (0, 0, 0, 1))], -380, -900)
            nt.links.new(geo.outputs["Pointiness"], r_point.inputs["Fac"])
            # break the crevice line up so it is not a clean wireframe
            n_dirt = M._noise(nt, 18.0, 8.0, 0.62, 0.4, -560, -1080)
            r_dirt = M._ramp(nt, [(0.35, (0.35, 0.35, 0.35, 1)),
                                  (0.70, (1, 1, 1, 1))], -380, -1080)
            nt.links.new(n_dirt.outputs["Fac"], r_dirt.inputs["Fac"])
            g1 = nt.nodes.new("ShaderNodeMath")
            g1.location = (-200, -940)
            g1.operation = "MULTIPLY"
            nt.links.new(r_point.outputs["Color"], g1.inputs[0])
            nt.links.new(r_dirt.outputs["Color"], g1.inputs[1])
            g2 = nt.nodes.new("ShaderNodeMath")
            g2.location = (-40, -940)
            g2.operation = "MULTIPLY"
            g2.name = _GRIME_NODE
            g2.inputs[1].default_value = GRIME_BASE * _s
            nt.links.new(g1.outputs[0], g2.inputs[0])
            mix_g = M._mix(nt, "MIX", 120, -700)
            mix_g.inputs[7].default_value = (*GRIME_DARK, 1.0)
            nt.links.new(g2.outputs[0], mix_g.inputs["Factor"])
            nt.links.new(src, mix_g.inputs[6])

            # efflorescence along construction joints
            wave = nt.nodes.new("ShaderNodeTexWave")
            wave.location = (-560, -1280)
            wave.wave_type = "BANDS"
            wave.bands_direction = "Z"
            wave.inputs["Scale"].default_value = 0.22
            wave.inputs["Distortion"].default_value = 1.6
            wave.inputs["Detail"].default_value = 2.0
            r_eff = M._ramp(nt, [(0.00, (1, 1, 1, 1)),
                                 (0.08, (0, 0, 0, 1))], -380, -1280)
            nt.links.new(wave.outputs["Fac"], r_eff.inputs["Fac"])
            n_eff = M._noise(nt, 6.0, 6.0, 0.6, 0.8, -560, -1460)
            r_eff2 = M._ramp(nt, [(0.48, (0, 0, 0, 1)),
                                  (0.80, (1, 1, 1, 1))], -380, -1460)
            nt.links.new(n_eff.outputs["Fac"], r_eff2.inputs["Fac"])
            e1 = nt.nodes.new("ShaderNodeMath")
            e1.location = (-200, -1340)
            e1.operation = "MULTIPLY"
            nt.links.new(r_eff.outputs["Color"], e1.inputs[0])
            nt.links.new(r_eff2.outputs["Color"], e1.inputs[1])
            e2 = nt.nodes.new("ShaderNodeMath")
            e2.location = (-40, -1340)
            e2.operation = "MULTIPLY"
            e2.name = _EFFL_NODE
            e2.inputs[1].default_value = EFFL_BASE * _s
            nt.links.new(e1.outputs[0], e2.inputs[0])
            mix_e = M._mix(nt, "MIX", 260, -700)
            mix_e.inputs[7].default_value = (*EFFLORESCENCE, 1.0)
            nt.links.new(e2.outputs[0], mix_e.inputs["Factor"])
            nt.links.new(mix_g.outputs[2], mix_e.inputs[6])
            return mix_e.outputs[2]

        if _splice_base_colour(nt, layer):
            done.append(name)
    log(f"  weather : concrete strength {s:.2f} on {len(done)} host "
        f"materials (defect materials untouched)")
    return {"concrete_weather_strength": s, "materials": done}


def set_concrete_weather(strength, log=None):
    """Retune the weathering strength in place, for a sweep.

    weather_concrete() must have run first. This only moves the two named
    multiplier nodes, so it can be called repeatedly without stacking more
    copies of the layer into the graph.
    """
    s = max(0.0, min(1.0, float(strength)))
    n = 0
    for name in _CONCRETE_HOSTS:
        m = bpy.data.materials.get(name)
        if m is None:
            continue
        for node_name, base in ((_GRIME_NODE, GRIME_BASE),
                                (_EFFL_NODE, EFFL_BASE)):
            node = m.node_tree.nodes.get(node_name)
            if node is not None:
                node.inputs[1].default_value = base * s
                n += 1
    if log:
        log(f"  weather : concrete strength set to {s:.2f} "
            f"({n} multiplier nodes)")
    return s


def weather_asphalt(log=print):
    """Patch repairs and a joint-adjacent albedo shift on the wearing course.

    Wheel-path polishing is NOT added here -- materials.py:505 already has
    it (the master brief listed it as outstanding; struck in v2.1).
    """
    m = bpy.data.materials.get("MAT_ASPHALT")
    if m is None:
        return {}
    nt = m.node_tree

    def layer(nt, src):
        # patch repairs: large irregular blotches of fresher, darker binder
        n_patch = M._noise(nt, 0.9, 3.0, 0.5, 0.9, -700, -700)
        r_patch = M._ramp(nt, [(0.52, (0, 0, 0, 1)), (0.58, (1, 1, 1, 1))],
                          -500, -700)
        nt.links.new(n_patch.outputs["Fac"], r_patch.inputs["Fac"])
        mix_p = M._mix(nt, "MIX", -260, -600)
        mix_p.inputs[7].default_value = (*ASPHALT_PATCH, 1.0)
        nt.links.new(r_patch.outputs["Color"], mix_p.inputs["Factor"])
        nt.links.new(src, mix_p.inputs[6])

        # joints: a narrow lighter band where the wearing course meets a
        # deck joint, where binder has been lost and aggregate shows
        wave = nt.nodes.new("ShaderNodeTexWave")
        wave.location = (-700, -900)
        wave.wave_type = "BANDS"
        wave.bands_direction = "X"
        wave.inputs["Scale"].default_value = 0.04
        wave.inputs["Distortion"].default_value = 0.4
        wave.inputs["Detail"].default_value = 1.0
        r_j = M._ramp(nt, [(0.00, (1, 1, 1, 1)), (0.05, (0, 0, 0, 1))],
                      -500, -900)
        nt.links.new(wave.outputs["Fac"], r_j.inputs["Fac"])
        j = nt.nodes.new("ShaderNodeMath")
        j.location = (-320, -900)
        j.operation = "MULTIPLY"
        j.inputs[1].default_value = 0.35
        nt.links.new(r_j.outputs["Color"], j.inputs[0])
        mix_j = M._mix(nt, "MIX", -100, -600)
        mix_j.inputs[7].default_value = (0.115, 0.113, 0.110, 1.0)
        nt.links.new(j.outputs[0], mix_j.inputs["Factor"])
        nt.links.new(mix_p.outputs[2], mix_j.inputs[6])
        return mix_j.outputs[2]

    ok = _splice_base_colour(nt, layer)
    log(f"  weather : asphalt patch repairs and joint albedo "
        f"{'applied' if ok else 'SKIPPED (no linked base colour)'}")
    return {"asphalt": bool(ok)}


def weather_water(log=print):
    """Bank turbidity and a flow direction on the river.

    MAT_RIVER's structure is kept: the depth tint, the ripple layers and the
    wind-lane roughness banding all stay. This adds silt near the banks,
    where a real monsoon-fed river carries its load, and stretches the fine
    ripple along the flow axis so the surface has a direction instead of
    being isotropic.

    The water grid is built in world coordinates with its object at the
    origin, so Object-space coordinates here are world metres and the bank
    distance can be taken directly against RIVER_CENTRE_X.
    """
    m = bpy.data.materials.get("MAT_RIVER")
    if m is None:
        return {}
    nt = m.node_tree
    half = PC.P.RIVER_WIDTH / 2.0

    def layer(nt, src):
        tc = None
        for n in nt.nodes:
            if n.bl_idname == "ShaderNodeTexCoord":
                tc = n
                break
        if tc is None:
            tc = M._tc(nt, -1500, -700)
        sep = nt.nodes.new("ShaderNodeSeparateXYZ")
        sep.location = (-1300, -700)
        nt.links.new(tc.outputs["Object"], sep.inputs["Vector"])
        dx = nt.nodes.new("ShaderNodeMath")
        dx.location = (-1120, -700)
        dx.operation = "SUBTRACT"
        dx.inputs[1].default_value = PC.P.RIVER_CENTRE_X
        nt.links.new(sep.outputs["X"], dx.inputs[0])
        ax = nt.nodes.new("ShaderNodeMath")
        ax.location = (-960, -700)
        ax.operation = "ABSOLUTE"
        nt.links.new(dx.outputs[0], ax.inputs[0])
        nx = nt.nodes.new("ShaderNodeMath")
        nx.location = (-800, -700)
        nx.operation = "DIVIDE"
        nx.inputs[1].default_value = max(half, 1.0)
        nt.links.new(ax.outputs[0], nx.inputs[0])
        # silt rises over the outer third of the channel
        r_bank = M._ramp(nt, [(0.62, (0, 0, 0, 1)), (1.00, (1, 1, 1, 1))],
                         -620, -700)
        nt.links.new(nx.outputs[0], r_bank.inputs["Fac"])
        # broken up so the bank line is not a clean gradient
        n_silt = M._noise(nt, 0.08, 5.0, 0.6, 0.7, -620, -900)
        r_silt = M._ramp(nt, [(0.35, (0.45, 0.45, 0.45, 1)),
                              (0.70, (1, 1, 1, 1))], -440, -900)
        nt.links.new(n_silt.outputs["Fac"], r_silt.inputs["Fac"])
        b1 = nt.nodes.new("ShaderNodeMath")
        b1.location = (-260, -760)
        b1.operation = "MULTIPLY"
        nt.links.new(r_bank.outputs["Color"], b1.inputs[0])
        nt.links.new(r_silt.outputs["Color"], b1.inputs[1])
        mix_b = M._mix(nt, "MIX", -80, -620)
        mix_b.inputs[7].default_value = (*WATER_TURBID, 1.0)
        nt.links.new(b1.outputs[0], mix_b.inputs["Factor"])
        nt.links.new(src, mix_b.inputs[6])
        return mix_b.outputs[2]

    ok = _splice_base_colour(nt, layer)

    # flow direction: stretch the finer ripple noise along Y, the axis the
    # channel runs, so the surface reads as moving rather than stippled.
    flowed = False
    tc = next((n for n in nt.nodes
               if n.bl_idname == "ShaderNodeTexCoord"), None)
    if tc is not None:
        for n in nt.nodes:
            if n.bl_idname != "ShaderNodeTexNoise":
                continue
            sc = n.inputs["Scale"].default_value
            if not (2.0 <= sc <= 6.0):      # the fine chop layer
                continue
            mp = M._map(nt, (1.0, 0.18, 1.0), -1340, -300)
            nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
            nt.links.new(mp.outputs["Vector"], n.inputs["Vector"])
            flowed = True
            break

    log(f"  weather : water bank turbidity "
        f"{'applied' if ok else 'SKIPPED'}, flow direction "
        f"{'applied' if flowed else 'SKIPPED'}")
    return {"water_turbidity": bool(ok), "water_flow": bool(flowed)}


def build_all(strength=None, log=print):
    """Every REV-C Stage 1 material change, in order."""
    out = {}
    out.update(build_city_materials(log))
    out.update(weather_concrete(strength, log))
    out.update(weather_asphalt(log))
    out.update(weather_water(log))
    return out

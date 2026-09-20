"""Fully procedural material library.

No external image files. Every surface is built from Blender's noise, Voronoi
and gradient nodes, so the whole environment regenerates from script and every
material parameter stays live and editable.

That choice is not only about reproducibility. For computer-vision research a
procedural surface has no tiling period and no baked-in lighting, so a network
cannot learn a texture artefact instead of the defect.
"""
from __future__ import annotations
import bpy
from mathutils import Vector

import params as P


# ---------------------------------------------------------------------------
def _new(name, fake=True):
    m = bpy.data.materials.get(name)
    if m:
        return m, False
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.use_fake_user = fake
    nt = m.node_tree
    nt.nodes.clear()
    return m, True


def _out(nt, x=900):
    n = nt.nodes.new("ShaderNodeOutputMaterial")
    n.location = (x, 0)
    return n


def _tc(nt, x=-1500, y=0):
    n = nt.nodes.new("ShaderNodeTexCoord")
    n.location = (x, y)
    return n


def _noise(nt, scale, detail=8.0, rough=0.55, dist=0.0, x=-1200, y=0,
           dim="3D"):
    n = nt.nodes.new("ShaderNodeTexNoise")
    n.noise_dimensions = dim
    n.location = (x, y)
    n.inputs["Scale"].default_value = scale
    n.inputs["Detail"].default_value = detail
    n.inputs["Roughness"].default_value = rough
    if "Distortion" in n.inputs:
        n.inputs["Distortion"].default_value = dist
    return n


def _ramp(nt, stops, x=-900, y=0, interp="LINEAR"):
    n = nt.nodes.new("ShaderNodeValToRGB")
    n.location = (x, y)
    n.color_ramp.interpolation = interp
    el = n.color_ramp.elements
    while len(el) > 1:
        el.remove(el[-1])
    el[0].position, el[0].color = stops[0][0], stops[0][1]
    for pos, col in stops[1:]:
        e = el.new(pos)
        e.color = col
    return n


def _mix(nt, blend="MIX", x=-600, y=0, fac=0.5):
    n = nt.nodes.new("ShaderNodeMix")
    n.data_type = "RGBA"
    n.blend_type = blend
    n.location = (x, y)
    n.inputs["Factor"].default_value = fac
    return n


def _bump(nt, strength=0.25, dist=0.02, x=-300, y=-400):
    n = nt.nodes.new("ShaderNodeBump")
    n.location = (x, y)
    n.inputs["Strength"].default_value = strength
    n.inputs["Distance"].default_value = dist
    return n


def _bsdf(nt, x=400, y=0):
    n = nt.nodes.new("ShaderNodeBsdfPrincipled")
    n.location = (x, y)
    return n


def _aerial(nt, src, x=200, y=300, start=1200.0, end=42000.0, strength=0.86):
    """Aerial perspective: mix a surface toward the haze colour with distance.

    Real atmosphere is what makes a far horizon pale, and without it a 185 km
    ground plane is rendered at full contrast right up to the horizon line,
    where it meets a hazed sky at a hard edge. Volumetrics would do this
    properly, but a participating medium over a 90 km scene on CPU Cycles is
    not affordable, and the shader form is indistinguishable for opaque
    surfaces seen from outside the medium.

    Applying a fixed haze tint instead -- which is what was here first --
    lightens near ground as much as far ground, so the modelled terrain sat as
    a dark slab on a pale plain. The mix has to be driven by camera distance,
    which is what this does.

    `src` is the node whose Color output carries the surface colour; the
    returned node's Color output replaces it.
    """
    cam = nt.nodes.new("ShaderNodeCameraData")
    cam.location = (x - 620, y - 160)
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.location = (x - 420, y - 160)
    rng.inputs["From Min"].default_value = start
    rng.inputs["From Max"].default_value = end
    rng.inputs["To Min"].default_value = 0.0
    rng.inputs["To Max"].default_value = strength
    rng.clamp = True
    nt.links.new(cam.outputs["View Distance"], rng.inputs["Value"])
    mix = _mix(nt, "MIX", x, y)
    mix.inputs[7].default_value = (*P.HAZE_COLOR, 1.0)
    nt.links.new(rng.outputs["Result"], mix.inputs["Factor"])
    out = src.outputs["Color"] if "Color" in src.outputs else src.outputs[
        2 if src.bl_idname == "ShaderNodeMix" else 0]
    nt.links.new(out, mix.inputs[6])
    return mix


def _map(nt, scale=(1, 1, 1), x=-1350, y=0):
    n = nt.nodes.new("ShaderNodeMapping")
    n.location = (x, y)
    n.inputs["Scale"].default_value = scale
    return n


# ===========================================================================
# CONCRETE
# ===========================================================================
def concrete(name="MAT_CONCRETE", base=(0.315, 0.310, 0.296),
             weather=0.55, stain=0.5, moss=0.12, form_lines=True,
             line_pitch=1.2):
    """Aged in-situ concrete.

    Layers, in the order they are combined:
      1. broad tonal drift          -- pour-to-pour colour variation
      2. fine aggregate mottle      -- surface texture
      3. formwork panel lines       -- the joints between shutter boards
      4. vertical water staining    -- runoff, driven downward in Z
      5. algae / moss               -- only in the damp lower band
      6. roughness variation        -- wet-looking where stained

    The result is meant to read as maintained-but-aged, not derelict.
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-1350, -500)
    nt.links.new(tc.outputs["Object"], sep.inputs["Vector"])

    # 1. broad tonal drift
    n_broad = _noise(nt, 0.035, 4.0, 0.5, 0.4, -1200, 380)
    r_broad = _ramp(nt, [(0.30, (base[0] * 0.86, base[1] * 0.86,
                                base[2] * 0.85, 1)),
                         (0.70, (min(base[0] * 1.14, 1), min(base[1] * 1.14, 1),
                                 min(base[2] * 1.13, 1), 1))], -950, 380)
    nt.links.new(tc.outputs["Object"], n_broad.inputs["Vector"])
    nt.links.new(n_broad.outputs["Fac"], r_broad.inputs["Fac"])

    # 2. fine aggregate mottle
    n_fine = _noise(nt, 22.0, 12.0, 0.62, 0.0, -1200, 140)
    r_fine = _ramp(nt, [(0.38, (0.22, 0.22, 0.21, 1)),
                        (0.62, (0.40, 0.40, 0.385, 1))], -950, 140)
    nt.links.new(tc.outputs["Object"], n_fine.inputs["Vector"])
    nt.links.new(n_fine.outputs["Fac"], r_fine.inputs["Fac"])

    mix1 = _mix(nt, "MIX", -700, 260, 0.30)
    nt.links.new(r_broad.outputs["Color"], mix1.inputs[6])
    nt.links.new(r_fine.outputs["Color"], mix1.inputs[7])

    # 2b. micro-texture, roughly 7 mm features.
    # The aggregate layer above is scaled for viewing at tens of metres; at the
    # 1 m standoff an inspection UAV actually works from, it leaves the surface
    # looking like smooth paper and every defect trivially separable from it.
    # This layer is deliberately low amplitude -- it must add surface noise at
    # the scale of the defect signal without reading as texture at range.
    n_micro = _noise(nt, 140.0, 8.0, 0.55, 0.0, -1200, -20)
    nt.links.new(tc.outputs["Object"], n_micro.inputs["Vector"])
    r_micro = _ramp(nt, [(0.34, (0.80, 0.80, 0.80, 1)),
                         (0.66, (1.06, 1.06, 1.06, 1))], -950, -20)
    nt.links.new(n_micro.outputs["Fac"], r_micro.inputs["Fac"])
    mix_micro = _mix(nt, "MULTIPLY", -540, 300, 0.55)
    nt.links.new(mix1.outputs[2], mix_micro.inputs[6])
    nt.links.new(r_micro.outputs["Color"], mix_micro.inputs[7])
    mix1 = mix_micro

    # 3. formwork panel lines
    if form_lines:
        wave = nt.nodes.new("ShaderNodeTexWave")
        wave.location = (-1200, -80)
        wave.wave_type = "BANDS"
        wave.bands_direction = "Z"
        wave.inputs["Scale"].default_value = 1.0 / max(line_pitch, 0.05)
        wave.inputs["Distortion"].default_value = 1.2
        wave.inputs["Detail"].default_value = 2.0
        nt.links.new(tc.outputs["Object"], wave.inputs["Vector"])
        r_wave = _ramp(nt, [(0.0, (0, 0, 0, 1)), (0.055, (1, 1, 1, 1))],
                       -950, -80)
        nt.links.new(wave.outputs["Fac"], r_wave.inputs["Fac"])
        mixw = _mix(nt, "MULTIPLY", -560, 120, 0.22)
        nt.links.new(mix1.outputs[2], mixw.inputs[6])
        nt.links.new(r_wave.outputs["Color"], mixw.inputs[7])
        col_src = mixw
    else:
        col_src = mix1

    # 4. vertical water staining -- streaks that run DOWN, stretched in Z
    map_st = _map(nt, (1.0, 1.0, 0.09), -1350, -260)
    nt.links.new(tc.outputs["Object"], map_st.inputs["Vector"])
    n_st = _noise(nt, 5.0, 9.0, 0.72, 1.6, -1180, -260)
    nt.links.new(map_st.outputs["Vector"], n_st.inputs["Vector"])
    r_st = _ramp(nt, [(0.44, (0, 0, 0, 1)), (0.78, (1, 1, 1, 1))],
                 -950, -260)
    nt.links.new(n_st.outputs["Fac"], r_st.inputs["Fac"])
    st_amt = nt.nodes.new("ShaderNodeMath")
    st_amt.location = (-780, -260)
    st_amt.operation = "MULTIPLY"
    st_amt.inputs[1].default_value = stain * weather
    nt.links.new(r_st.outputs["Color"], st_amt.inputs[0])

    mix_st = _mix(nt, "MULTIPLY", -420, 160)
    mix_st.inputs[7].default_value = (0.44, 0.42, 0.39, 1.0)
    nt.links.new(st_amt.outputs[0], mix_st.inputs["Factor"])
    nt.links.new(col_src.outputs[2], mix_st.inputs[6])

    # 5. algae in the damp lower band only
    z_band = _ramp(nt, [(0.0, (1, 1, 1, 1)), (0.25, (0, 0, 0, 1))],
                   -950, -640)
    nt.links.new(sep.outputs["Z"], z_band.inputs["Fac"])
    n_moss = _noise(nt, 9.0, 6.0, 0.6, 0.8, -1180, -640)
    nt.links.new(tc.outputs["Object"], n_moss.inputs["Vector"])
    r_moss = _ramp(nt, [(0.52, (0, 0, 0, 1)), (0.80, (1, 1, 1, 1))],
                   -780, -640)
    nt.links.new(n_moss.outputs["Fac"], r_moss.inputs["Fac"])
    moss_f = nt.nodes.new("ShaderNodeMath")
    moss_f.location = (-600, -640)
    moss_f.operation = "MULTIPLY"
    nt.links.new(r_moss.outputs["Color"], moss_f.inputs[0])
    nt.links.new(z_band.outputs["Color"], moss_f.inputs[1])
    moss_a = nt.nodes.new("ShaderNodeMath")
    moss_a.location = (-440, -640)
    moss_a.operation = "MULTIPLY"
    moss_a.inputs[1].default_value = moss
    nt.links.new(moss_f.outputs[0], moss_a.inputs[0])

    mix_moss = _mix(nt, "MIX", -220, 160)
    mix_moss.inputs[7].default_value = (0.115, 0.145, 0.088, 1.0)
    nt.links.new(moss_a.outputs[0], mix_moss.inputs["Factor"])
    nt.links.new(mix_st.outputs[2], mix_moss.inputs[6])

    # 6. shading
    bsdf = _bsdf(nt)
    nt.links.new(_aerial(nt, mix_moss, 120, 320).outputs[2],
                 bsdf.inputs["Base Color"])
    r_rough = _ramp(nt, [(0.20, (0.80, 0.80, 0.80, 1)),
                         (0.85, (0.98, 0.98, 0.98, 1))], -220, -60)
    nt.links.new(n_fine.outputs["Fac"], r_rough.inputs["Fac"])
    nt.links.new(r_rough.outputs["Color"], bsdf.inputs["Roughness"])
    bsdf.inputs["Specular IOR Level"].default_value = 0.32

    # Two bump stages: the coarse aggregate relief, then the micro relief on
    # top of it. Chaining them (micro's Normal feeding coarse) is what gives a
    # grazing inspection light something to catch at both scales.
    bump_m = _bump(nt, 0.10, 0.0025, 0, -560)
    nt.links.new(n_micro.outputs["Fac"], bump_m.inputs["Height"])
    bump = _bump(nt, 0.22, 0.012, 120, -420)
    nt.links.new(n_fine.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump_m.outputs["Normal"], bump.inputs["Normal"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])

    out = _out(nt)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return m


# ===========================================================================
# CRACK DECAL  -- the shader half of the hybrid defect model
# ===========================================================================
def crack_decal(name, severity=1, width_mm=0.2, pattern="LINEAR",
                rust=0.0, seed=0.0):
    """A transparent decal carrying a procedural crack network.

    Geometry-free by design: a 0.2 mm crack would need sub-millimetre mesh to
    exist as real geometry, and no LiDAR or depth sensor resolves it anyway.
    What a camera sees is a dark line with a soft rim, and that is what this
    reproduces.

    The crack itself is a Voronoi distance-to-edge field, warped by noise. That
    gives irregular branching with natural junctions -- unlike a drawn line, it
    tapers, changes width and disappears on its own. A second large-scale noise
    mask breaks the network up so it does not run edge to edge.

    pattern:
      LINEAR      one dominant direction (longitudinal / transverse)
      DIAGONAL    rotated 40 deg, shear-crack orientation
      NETWORK     isotropic map cracking
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    # Blender 4.2+ replaced blend_method/shadow_method with these.
    #
    # use_transparent_shadow MUST be True. With it off, the decal is
    # see-through to the camera but casts the shadow of its full rectangle,
    # and since the decal sits 6 mm off the concrete that shadow lands almost
    # exactly under it -- a solid black rectangle pasted on the girder with
    # the crack barely visible inside it. From 50 m it is invisible; at
    # inspection range it is the most obvious error in the model.
    for attr, val in (("surface_render_method", "BLENDED"),
                      ("use_transparent_shadow", True),
                      ("blend_method", "BLEND")):
        if hasattr(m, attr):
            try:
                setattr(m, attr, val)
            except (TypeError, AttributeError):
                pass
    nt = m.node_tree
    tc = _tc(nt)

    # Anisotropy: compressing an axis of the COORDINATE stretches the pattern
    # along that axis. The decal's U axis is the host surface's tangent -- the
    # direction the member runs -- so compressing U is what makes a crack run
    # along the member. Compressing V instead (which is what this did first)
    # produced longitudinal cracks that ran vertically down a girder web:
    # a perfectly plausible-looking crack pointing the wrong way, which is a
    # worse error than an obviously broken one because it survives inspection.
    if pattern == "LINEAR":
        sc = (0.14, 1.0, 1.0)
        rot = 0.0
    elif pattern == "DIAGONAL":
        sc = (0.22, 1.0, 1.0)
        rot = 0.70                  # ~40 deg: shear-crack inclination
    else:
        sc = (1.0, 1.0, 1.0)
        rot = 0.0

    mp = _map(nt, sc, -1350, 0)
    mp.inputs["Rotation"].default_value = (0.0, 0.0, rot)
    mp.inputs["Location"].default_value = (seed * 7.3, seed * 3.1, 0.0)
    nt.links.new(tc.outputs["UV"], mp.inputs["Vector"])

    # warp so the cell walls are not straight
    warp = _noise(nt, 6.0, 8.0, 0.6, 0.0, -1180, -260)
    nt.links.new(mp.outputs["Vector"], warp.inputs["Vector"])
    vm = nt.nodes.new("ShaderNodeVectorMath")
    vm.location = (-1000, -140)
    vm.operation = "ADD"
    nt.links.new(mp.outputs["Vector"], vm.inputs[0])
    scl = nt.nodes.new("ShaderNodeVectorMath")
    scl.location = (-1000, -320)
    scl.operation = "SCALE"
    scl.inputs["Scale"].default_value = 0.28
    nt.links.new(warp.outputs["Color"], scl.inputs[0])
    nt.links.new(scl.outputs["Vector"], vm.inputs[1])

    vor = nt.nodes.new("ShaderNodeTexVoronoi")
    vor.location = (-820, 0)
    vor.feature = "DISTANCE_TO_EDGE"
    vor.inputs["Scale"].default_value = 5.0 if pattern == "NETWORK" else 3.2
    nt.links.new(vm.outputs["Vector"], vor.inputs["Vector"])

    # width: severity drives how far the dark band extends from the cell wall.
    #
    # DISTANCE_TO_EDGE is 0 ON the cell wall and rises going into the cell, so
    # the crack is where the value is SMALL. The ramp therefore has to run
    # opaque -> transparent as distance increases. It was written the other way
    # round first, which produced a decal that was solid everywhere EXCEPT the
    # crack -- a dark rectangle pasted on the concrete with faint pale lines
    # through it. It renders as an obvious error the moment you look at one
    # close up, and as nothing at all from 50 m, which is why it survived
    # several passes of numeric checking.
    w = min(0.16, 0.012 + width_mm * 0.016)
    r_crack = _ramp(nt, [(0.0, (0, 0, 0, 1)),
                         (w, (0.65, 0.65, 0.65, 1)),
                         (w * 2.6, (1, 1, 1, 1))], -600, 0)
    nt.links.new(vor.outputs["Distance"], r_crack.inputs["Fac"])
    inv = nt.nodes.new("ShaderNodeMath")
    inv.location = (-420, 0)
    inv.operation = "SUBTRACT"
    inv.inputs[0].default_value = 1.0
    nt.links.new(r_crack.outputs["Color"], inv.inputs[1])

    # break-up mask: cracks fade out rather than running to the decal edge
    mask = _noise(nt, 1.7, 4.0, 0.5, 0.6, -820, -520)
    nt.links.new(mp.outputs["Vector"], mask.inputs["Vector"])
    r_mask = _ramp(nt, [(0.30, (0, 0, 0, 1)), (0.62, (1, 1, 1, 1))],
                   -600, -520)
    nt.links.new(mask.outputs["Fac"], r_mask.inputs["Fac"])

    # radial falloff so the decal has no visible rectangular border
    uvsep = nt.nodes.new("ShaderNodeSeparateXYZ")
    uvsep.location = (-1180, 420)
    nt.links.new(tc.outputs["UV"], uvsep.inputs["Vector"])
    dx = nt.nodes.new("ShaderNodeMath"); dx.location = (-1000, 480)
    dx.operation = "SUBTRACT"; dx.inputs[1].default_value = 0.5
    nt.links.new(uvsep.outputs["X"], dx.inputs[0])
    dy = nt.nodes.new("ShaderNodeMath"); dy.location = (-1000, 340)
    dy.operation = "SUBTRACT"; dy.inputs[1].default_value = 0.5
    nt.links.new(uvsep.outputs["Y"], dy.inputs[0])
    ln = nt.nodes.new("ShaderNodeVectorMath"); ln.location = (-820, 420)
    ln.operation = "LENGTH"
    cmb = nt.nodes.new("ShaderNodeCombineXYZ"); cmb.location = (-900, 420)
    nt.links.new(dx.outputs[0], cmb.inputs["X"])
    nt.links.new(dy.outputs[0], cmb.inputs["Y"])
    nt.links.new(cmb.outputs["Vector"], ln.inputs[0])
    r_fall = _ramp(nt, [(0.26, (1, 1, 1, 1)), (0.50, (0, 0, 0, 1))],
                   -640, 420)
    nt.links.new(ln.outputs["Value"], r_fall.inputs["Fac"])

    a1 = nt.nodes.new("ShaderNodeMath"); a1.location = (-240, -200)
    a1.operation = "MULTIPLY"
    nt.links.new(inv.outputs[0], a1.inputs[0])
    nt.links.new(r_mask.outputs["Color"], a1.inputs[1])
    a2 = nt.nodes.new("ShaderNodeMath"); a2.location = (-80, -200)
    a2.operation = "MULTIPLY"
    nt.links.new(a1.outputs[0], a2.inputs[0])
    nt.links.new(r_fall.outputs["Color"], a2.inputs[1])

    # contrast: a hairline crack is LOW contrast. Severity raises it.
    # This is what stops the defects being trivially detectable.
    gain = nt.nodes.new("ShaderNodeMath"); gain.location = (80, -200)
    gain.operation = "MULTIPLY"
    gain.inputs[1].default_value = {1: 0.34, 2: 0.58, 3: 0.80,
                                    4: 0.94}.get(severity, 0.5)
    nt.links.new(a2.outputs[0], gain.inputs[0])

    bsdf = _bsdf(nt, 400, 0)
    dark = (0.055, 0.050, 0.047, 1.0) if severity >= 3 \
        else (0.105, 0.098, 0.092, 1.0)
    bsdf.inputs["Base Color"].default_value = dark
    bsdf.inputs["Roughness"].default_value = 0.95

    # rust bleed for reinforcement-associated cracking
    if rust > 0.0:
        rn = _noise(nt, 3.0, 6.0, 0.6, 0.9, -820, 680)
        nt.links.new(mp.outputs["Vector"], rn.inputs["Vector"])
        r_rust = _ramp(nt, [(0.40, (0.0, 0.0, 0.0, 1)),
                            (0.75, (1, 1, 1, 1))], -600, 680)
        nt.links.new(rn.outputs["Fac"], r_rust.inputs["Fac"])
        rf = nt.nodes.new("ShaderNodeMath"); rf.location = (-400, 680)
        rf.operation = "MULTIPLY"; rf.inputs[1].default_value = rust
        nt.links.new(r_rust.outputs["Color"], rf.inputs[0])
        mixr = _mix(nt, "MIX", 180, 120)
        mixr.inputs[6].default_value = dark
        mixr.inputs[7].default_value = (0.235, 0.088, 0.030, 1.0)
        nt.links.new(rf.outputs[0], mixr.inputs["Factor"])
        nt.links.new(mixr.outputs[2], bsdf.inputs["Base Color"])
        # rust stains a wider halo than the crack itself
        halo = nt.nodes.new("ShaderNodeMath"); halo.location = (240, -340)
        halo.operation = "MAXIMUM"
        nt.links.new(gain.outputs[0], halo.inputs[0])
        rh = nt.nodes.new("ShaderNodeMath"); rh.location = (80, -420)
        rh.operation = "MULTIPLY"; rh.inputs[1].default_value = rust * 0.55
        nt.links.new(rf.outputs[0], rh.inputs[0])
        rh2 = nt.nodes.new("ShaderNodeMath"); rh2.location = (160, -460)
        rh2.operation = "MULTIPLY"
        nt.links.new(rh.outputs[0], rh2.inputs[0])
        nt.links.new(r_fall.outputs["Color"], rh2.inputs[1])
        nt.links.new(rh2.outputs[0], halo.inputs[1])
        alpha_src = halo
    else:
        alpha_src = gain

    trans = nt.nodes.new("ShaderNodeBsdfTransparent")
    trans.location = (400, -260)
    mixsh = nt.nodes.new("ShaderNodeMixShader")
    mixsh.location = (650, 0)
    nt.links.new(alpha_src.outputs[0], mixsh.inputs["Fac"])
    nt.links.new(trans.outputs["BSDF"], mixsh.inputs[1])
    nt.links.new(bsdf.outputs["BSDF"], mixsh.inputs[2])

    out = _out(nt)
    nt.links.new(mixsh.outputs["Shader"], out.inputs["Surface"])
    return m


# ===========================================================================
# OTHER SURFACES
# ===========================================================================
def asphalt(name="MAT_ASPHALT"):
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    n = _noise(nt, 60.0, 12.0, 0.7, 0.0, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    r = _ramp(nt, [(0.35, (0.030, 0.030, 0.032, 1)),
                   (0.70, (0.082, 0.082, 0.086, 1))], -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    # wheel-path polishing: two darker, smoother bands per carriageway
    wear = _noise(nt, 0.6, 3.0, 0.5, 0.3, -1100, -320)
    nt.links.new(tc.outputs["Object"], wear.inputs["Vector"])
    r_w = _ramp(nt, [(0.40, (0.62, 0.62, 0.62, 1)),
                     (0.75, (0.92, 0.92, 0.92, 1))], -600, -320)
    nt.links.new(wear.outputs["Fac"], r_w.inputs["Fac"])
    b = _bsdf(nt)
    nt.links.new(r.outputs["Color"], b.inputs["Base Color"])
    nt.links.new(r_w.outputs["Color"], b.inputs["Roughness"])
    bm = _bump(nt, 0.30, 0.006, 120, -400)
    nt.links.new(n.outputs["Fac"], bm.inputs["Height"])
    nt.links.new(bm.outputs["Normal"], b.inputs["Normal"])
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def road_paint(name="MAT_ROAD_PAINT", col=(0.72, 0.70, 0.64)):
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    n = _noise(nt, 30.0, 8.0, 0.6, 0.0, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    # worn paint: the noise erodes the marking rather than tinting it
    r = _ramp(nt, [(0.36, (col[0] * 0.45, col[1] * 0.45, col[2] * 0.45, 1)),
                   (0.64, (col[0], col[1], col[2], 1))], -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    b = _bsdf(nt)
    nt.links.new(r.outputs["Color"], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.68
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def water(name="MAT_RIVER"):
    """Silty river water. Not a swimming pool.

    Depth-tinted, low transmission, with a broad wind ripple and a finer
    chop. A clear-blue water shader would be wrong for a monsoon-fed river
    and would also give a UAV altimeter an unrealistically clean reflection.
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    big = _noise(nt, 0.30, 5.0, 0.55, 0.5, -1150, 260)
    nt.links.new(tc.outputs["Object"], big.inputs["Vector"])
    fine = _noise(nt, 3.6, 9.0, 0.62, 1.1, -1150, -40)
    nt.links.new(tc.outputs["Object"], fine.inputs["Vector"])
    add = nt.nodes.new("ShaderNodeMath"); add.location = (-880, 120)
    add.operation = "MULTIPLY_ADD"
    add.inputs[1].default_value = 0.35
    nt.links.new(fine.outputs["Fac"], add.inputs[0])
    nt.links.new(big.outputs["Fac"], add.inputs[2])

    # Large-scale roughness banding: wind lanes and current streaks, 25-80 m
    # across. This is the layer that makes water read as water AT RANGE. The
    # ripple layers above are 30 cm and go sub-pixel beyond about 200 m, so a
    # 620 m river lit by a hazy sky rendered as one flat pale sheet
    # indistinguishable from the sky -- physically defensible and useless to
    # look at. Roughness variation survives the distance because it modulates
    # how much sky each patch reflects.
    lanes = _noise(nt, 0.022, 6.0, 0.6, 0.9, -1150, -320)
    nt.links.new(tc.outputs["Object"], lanes.inputs["Vector"])
    r_rough = _ramp(nt, [(0.32, (0.020, 0.020, 0.020, 1)),
                         (0.70, (0.135, 0.135, 0.135, 1))], -880, -320)
    nt.links.new(lanes.outputs["Fac"], r_rough.inputs["Fac"])

    b = _bsdf(nt)
    b.inputs["Base Color"].default_value = (0.030, 0.041, 0.034, 1.0)
    b.inputs["Transmission Weight"].default_value = 0.18
    b.inputs["IOR"].default_value = 1.333
    nt.links.new(r_rough.outputs["Color"], b.inputs["Roughness"])
    bm = _bump(nt, 0.45, 0.10, 120, -380)
    nt.links.new(add.outputs[0], bm.inputs["Height"])
    nt.links.new(bm.outputs["Normal"], b.inputs["Normal"])
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def steel(name="MAT_STEEL", col=(0.44, 0.44, 0.46), rough=0.42):
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    b = _bsdf(nt)
    b.inputs["Base Color"].default_value = (*col, 1.0)
    b.inputs["Metallic"].default_value = 0.92
    b.inputs["Roughness"].default_value = rough
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def rebar(name="MAT_REBAR_CORRODED"):
    """Exposed, corroded reinforcement -- section loss and scale."""
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    n = _noise(nt, 26.0, 10.0, 0.68, 0.6, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    # Heavily corroded bar: dark iron oxide, not bright orange. A clean
    # orange bar reads as a plastic dowel; what gives real section-loss
    # corrosion away in photographs is how dark and matte it is.
    r = _ramp(nt, [(0.25, (0.030, 0.013, 0.007, 1)),
                   (0.55, (0.068, 0.029, 0.013, 1)),
                   (0.88, (0.115, 0.052, 0.022, 1))], -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    b = _bsdf(nt)
    nt.links.new(r.outputs["Color"], b.inputs["Base Color"])
    b.inputs["Metallic"].default_value = 0.0
    b.inputs["Roughness"].default_value = 0.96
    b.inputs["Specular IOR Level"].default_value = 0.15
    bm = _bump(nt, 0.95, 0.004, 120, -400)
    nt.links.new(n.outputs["Fac"], bm.inputs["Height"])
    nt.links.new(bm.outputs["Normal"], b.inputs["Normal"])
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def spall_face(name="MAT_SPALL_FACE", rust=0.0):
    """The broken face inside a spall.

    A spall is not a dent in the same surface -- it is a hole through the
    cover into material that has never been exposed. Fresh fracture is darker
    than a weathered soffit, much rougher, and coarse: the aggregate is
    exposed rather than buried under a float finish. Rendering the cavity in
    the host's own material left a 35 mm bowl that read as flat, because the
    only cue was shading and the bowl is far too shallow to shade itself.

    `rust` bleeds iron staining across the face, which is what a spall driven
    by reinforcement corrosion actually looks like.
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)

    # coarse exposed aggregate
    agg = nt.nodes.new("ShaderNodeTexVoronoi")
    agg.location = (-1150, 120)
    agg.feature = "F1"
    agg.inputs["Scale"].default_value = 105.0
    if "Randomness" in agg.inputs:
        agg.inputs["Randomness"].default_value = 1.0
    nt.links.new(tc.outputs["Object"], agg.inputs["Vector"])
    # Dark cement paste at the cell walls rising to a lighter stone face in
    # the middle of each cell. A three-stop ramp that ran dark-light-dark
    # rendered every aggregate particle as a ring, which reads as bubbles in
    # foam rather than as broken concrete.
    r_agg = _ramp(nt, [(0.00, (0.072, 0.068, 0.062, 1)),
                       (0.16, (0.185, 0.176, 0.162, 1))], -900, 120)
    nt.links.new(agg.outputs["Distance"], r_agg.inputs["Fac"])

    grit = _noise(nt, 180.0, 10.0, 0.7, 0.0, -1150, -180)
    nt.links.new(tc.outputs["Object"], grit.inputs["Vector"])
    r_grit = _ramp(nt, [(0.35, (0.72, 0.72, 0.72, 1)),
                        (0.68, (1.12, 1.12, 1.12, 1))], -900, -180)
    nt.links.new(grit.outputs["Fac"], r_grit.inputs["Fac"])

    mix = _mix(nt, "MULTIPLY", -620, 60, 0.7)
    nt.links.new(r_agg.outputs["Color"], mix.inputs[6])
    nt.links.new(r_grit.outputs["Color"], mix.inputs[7])
    col = mix

    if rust > 0.0:
        rn = _noise(nt, 8.0, 8.0, 0.62, 1.1, -1150, -420)
        nt.links.new(tc.outputs["Object"], rn.inputs["Vector"])
        r_r = _ramp(nt, [(0.38, (0, 0, 0, 1)), (0.72, (1, 1, 1, 1))],
                    -900, -420)
        nt.links.new(rn.outputs["Fac"], r_r.inputs["Fac"])
        amt = nt.nodes.new("ShaderNodeMath")
        amt.location = (-700, -420)
        amt.operation = "MULTIPLY"
        amt.inputs[1].default_value = rust
        nt.links.new(r_r.outputs["Color"], amt.inputs[0])
        mr = _mix(nt, "MIX", -420, 60)
        mr.inputs[7].default_value = (0.192, 0.072, 0.026, 1.0)
        nt.links.new(amt.outputs[0], mr.inputs["Factor"])
        nt.links.new(mix.outputs[2], mr.inputs[6])
        col = mr

    b = _bsdf(nt)
    nt.links.new(col.outputs[2], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.97
    b.inputs["Specular IOR Level"].default_value = 0.18
    bm = _bump(nt, 0.42, 0.006, 120, -400)
    nt.links.new(agg.outputs["Distance"], bm.inputs["Height"])
    bm2 = _bump(nt, 0.35, 0.0015, 0, -560)
    nt.links.new(grit.outputs["Fac"], bm2.inputs["Height"])
    nt.links.new(bm2.outputs["Normal"], bm.inputs["Normal"])
    nt.links.new(bm.outputs["Normal"], b.inputs["Normal"])
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def delamination_face(name="MAT_DELAMINATION"):
    """A delaminated zone: intact surface, hollow behind it.

    Visually almost nothing -- faint perimeter cracking and slight
    discolouration. That is the honest appearance, and it is precisely why
    delamination is found by sounding rather than by looking, and why every
    delamination in this model escalates rather than being auto-repaired.
    """
    return concrete(name, base=(0.298, 0.290, 0.272), weather=0.75,
                    stain=0.70, moss=0.10, form_lines=False)


def repair_patch(name="MAT_REPAIR_PATCH"):
    """A previous repair: newer mortar, slightly wrong colour, visible edge.

    Included because a real inspection has to distinguish a repair from a
    defect, and an autonomous one that cannot will keep re-repairing the same
    patch.
    """
    m = concrete(name, base=(0.372, 0.366, 0.352), weather=0.22,
                 stain=0.18, moss=0.03, form_lines=False)
    return m


def ground_mat(name="MAT_GROUND"):
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    # Object space at a real-world scale: one noise cycle per ~40 m. Generated
    # coordinates normalise across the object, so a 5 km terrain would get a
    # single stretched cycle.
    n = _noise(nt, 0.025, 9.0, 0.55, 0.5, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    r = _ramp(nt, [(0.30, (0.086, 0.072, 0.046, 1)),
                   (0.55, (0.128, 0.112, 0.068, 1)),
                   (0.80, (0.092, 0.098, 0.055, 1))], -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    haze = _aerial(nt, r, -560, 0)
    b = _bsdf(nt)
    nt.links.new(haze.outputs[2], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.94
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def distant_ground(name="MAT_GROUND_FAR"):
    """Land beyond the modelled terrain.

    Separate from MAT_GROUND for two reasons. Its noise is scaled in OBJECT
    space at a few-hundred-metre period -- MAT_GROUND uses Generated
    coordinates, which normalise across the object's bounding box and would
    stretch one noise cycle across 185 km of far ground, rendering it as a
    flat grey slab. And it is tinted toward the haze colour so distant land
    recedes instead of ending at a hard line.
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    n = _noise(nt, 0.0024, 6.0, 0.55, 0.35, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    # Deliberately the SAME palette as MAT_GROUND. The two surfaces meet along
    # a 5.7 km seam; any colour difference between them draws that seam.
    r = _ramp(nt, [(0.30, (0.086, 0.072, 0.046, 1)),
                   (0.55, (0.128, 0.112, 0.068, 1)),
                   (0.80, (0.092, 0.098, 0.055, 1))], -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    # a second, coarser band so the far field is not one flat tone
    n2 = _noise(nt, 0.00035, 4.0, 0.5, 0.2, -1100, -260)
    nt.links.new(tc.outputs["Object"], n2.inputs["Vector"])
    r2 = _ramp(nt, [(0.35, (0.72, 0.72, 0.70, 1)),
                    (0.68, (1.18, 1.16, 1.10, 1))], -850, -260)
    nt.links.new(n2.outputs["Fac"], r2.inputs["Fac"])
    band = _mix(nt, "MULTIPLY", -600, 0, 0.7)
    nt.links.new(r.outputs["Color"], band.inputs[6])
    nt.links.new(r2.outputs["Color"], band.inputs[7])
    haze = _aerial(nt, band, -320, 0)
    b = _bsdf(nt)
    nt.links.new(haze.outputs[2], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.95
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def simple(name, col, rough=0.6, metallic=0.0, emit=0.0, aerial=True):
    """Flat colour with optional aerial perspective.

    `aerial` is on by default because most users of this material are city
    geometry seen from hundreds or thousands of metres away, and a building at
    4 km rendered at full contrast against hazed ground looks pasted on.
    """
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    b = _bsdf(nt)
    b.inputs["Base Color"].default_value = (*col, 1.0)
    if aerial:
        rgb = nt.nodes.new("ShaderNodeRGB")
        rgb.location = (-400, 300)
        rgb.outputs[0].default_value = (*col, 1.0)
        nt.links.new(_aerial(nt, rgb, 120, 300).outputs[2],
                     b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    if emit:
        b.inputs["Emission Color"].default_value = (*col, 1.0)
        b.inputs["Emission Strength"].default_value = emit
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def vegetation(name="MAT_VEGETATION", col=(0.055, 0.105, 0.038)):
    m, fresh = _new(name)
    if not fresh:
        return m
    nt = m.node_tree
    tc = _tc(nt)
    n = _noise(nt, 14.0, 8.0, 0.6, 0.4, -1100, 0)
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    r = _ramp(nt, [(0.32, (col[0] * 0.6, col[1] * 0.6, col[2] * 0.6, 1)),
                   (0.72, (col[0] * 1.5, col[1] * 1.35, col[2] * 1.4, 1))],
              -850, 0)
    nt.links.new(n.outputs["Fac"], r.inputs["Fac"])
    b = _bsdf(nt)
    nt.links.new(r.outputs["Color"], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.82
    o = _out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


# ===========================================================================
def build_library():
    """Every material the environment uses, created once."""
    lib = {
        "concrete_deck": concrete("MAT_CONCRETE_DECK", weather=0.50,
                                  stain=0.55, moss=0.10, line_pitch=1.6),
        "concrete_girder": concrete("MAT_CONCRETE_GIRDER", weather=0.62,
                                    stain=0.62, moss=0.16, line_pitch=1.1),
        "concrete_pier": concrete("MAT_CONCRETE_PIER", weather=0.70,
                                  stain=0.70, moss=0.30, line_pitch=1.2),
        "concrete_parapet": concrete("MAT_CONCRETE_PARAPET", weather=0.45,
                                     stain=0.40, moss=0.05, line_pitch=2.0),
        "concrete_low": concrete("MAT_CONCRETE_LOD_LOW", weather=0.35,
                                 stain=0.30, moss=0.06, form_lines=False),
        "asphalt": asphalt(),
        "paint_white": road_paint("MAT_PAINT_WHITE", (0.74, 0.72, 0.67)),
        "paint_yellow": road_paint("MAT_PAINT_YELLOW", (0.62, 0.47, 0.10)),
        "water": water(),
        "steel": steel(),
        "steel_dark": steel("MAT_STEEL_DARK", (0.15, 0.15, 0.16), 0.55),
        "rebar": rebar(),
        "spall_face": spall_face("MAT_SPALL_FACE", rust=0.0),
        "spall_face_rust": spall_face("MAT_SPALL_FACE_RUST", rust=0.55),
        "delamination": delamination_face(),
        "repair_patch": repair_patch(),
        "ground": ground_mat(),
        "ground_far": distant_ground(),
        "bearing": simple("MAT_BEARING", (0.045, 0.045, 0.048), 0.75),
        "rock": simple("MAT_ROCK", (0.145, 0.135, 0.122), 0.88),
        "veg": vegetation(),
        "veg_dry": vegetation("MAT_VEGETATION_DRY", (0.115, 0.098, 0.042)),
        "glass": simple("MAT_GLASS", (0.115, 0.145, 0.165), 0.14, 0.35),
        "bldg_a": simple("MAT_BUILDING_A", (0.315, 0.288, 0.252), 0.78),
        "bldg_b": simple("MAT_BUILDING_B", (0.245, 0.238, 0.228), 0.80),
        "bldg_c": simple("MAT_BUILDING_C", (0.352, 0.315, 0.272), 0.75),
        "bldg_d": simple("MAT_BUILDING_D", (0.198, 0.205, 0.212), 0.72),
        "vehicle_a": simple("MAT_VEHICLE_A", (0.42, 0.44, 0.46), 0.35, 0.15),
        "vehicle_b": simple("MAT_VEHICLE_B", (0.30, 0.09, 0.07), 0.35, 0.15),
        "vehicle_c": simple("MAT_VEHICLE_C", (0.08, 0.10, 0.16), 0.35, 0.15),
        "vehicle_d": simple("MAT_VEHICLE_D", (0.55, 0.52, 0.45), 0.40, 0.10),
        "sign": simple("MAT_SIGN", (0.055, 0.145, 0.075), 0.45),
        "barrier": simple("MAT_BARRIER", (0.38, 0.36, 0.33), 0.72),
    }
    return lib

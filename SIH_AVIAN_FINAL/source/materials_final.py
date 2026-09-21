"""SIH_AVIAN_FINAL -- realism-pass materials: vehicle body colour variation,
waterline staining on the river-adjacent piers, and a routed-through-aerial
treatment for the distant city silhouettes.

Built on materials.py's own node-graph helpers (`_new`, `_ramp`, `_bsdf`,
`_out`, `_aerial`) for consistency with the rest of the library. `_new(name)`
returns `(material, is_freshly_created)` -- NOT a node tree -- and `_bsdf`/
`_out` return bare nodes that still need manually linking and an explicit
`_out()` + link, exactly as materials.py's own `simple()`/`concrete()` do.
"""
from __future__ import annotations
import bpy

import materials as MAT


def vehicle_body(name="MAT_VEHICLE_BODY"):
    """One material, many colours: Object Info -> Random into a CONSTANT
    ColorRamp, so 6-8 body colours come from a single material rather than
    a separate datablock per colour -- every vehicle using it gets an
    automatically-varied colour with zero per-object material bookkeeping.
    """
    m, fresh = MAT._new(name)
    if not fresh:
        return m
    nt = m.node_tree
    info = nt.nodes.new("ShaderNodeObjectInfo")
    info.location = (-900, 200)

    # 8 body colours: silver, white, red, dark blue, black, taxi yellow,
    # grey-green, maroon.
    stops = [
        (0.00, (0.72, 0.73, 0.75, 1)),
        (0.13, (0.90, 0.90, 0.88, 1)),
        (0.26, (0.46, 0.07, 0.06, 1)),
        (0.39, (0.08, 0.14, 0.34, 1)),
        (0.52, (0.03, 0.03, 0.035, 1)),
        (0.65, (0.80, 0.60, 0.04, 1)),
        (0.78, (0.20, 0.28, 0.20, 1)),
        (0.91, (0.30, 0.06, 0.10, 1)),
    ]
    ramp = MAT._ramp(nt, stops, -600, 200, interp="CONSTANT")
    nt.links.new(info.outputs["Random"], ramp.inputs["Fac"])

    b = MAT._bsdf(nt, -200, 0)
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.32
    b.inputs["Metallic"].default_value = 0.55
    o = MAT._out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def _flat(name, col, rough, metallic=0.0):
    m, fresh = MAT._new(name)
    if not fresh:
        return m
    nt = m.node_tree
    b = MAT._bsdf(nt, -200, 0)
    b.inputs["Base Color"].default_value = (*col, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    o = MAT._out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m


def vehicle_tyre(name="MAT_VEHICLE_TYRE"):
    return _flat(name, (0.018, 0.018, 0.02), 0.85)


def vehicle_trim(name="MAT_VEHICLE_TRIM"):
    """Bumpers, mirrors, wheel hubs -- dark grey semi-gloss plastic/metal."""
    return _flat(name, (0.10, 0.10, 0.11), 0.42, metallic=0.30)


def rickshaw_body(name="MAT_RICKSHAW_BODY"):
    """Auto-rickshaws in this setting run the classic black-over-yellow."""
    return _flat(name, (0.82, 0.62, 0.05), 0.45, metallic=0.15)


def rickshaw_hood(name="MAT_RICKSHAW_HOOD"):
    return _flat(name, (0.03, 0.03, 0.035), 0.55)


# ===========================================================================
# TRAIN LIVERY
# ===========================================================================
# Body: off-white/cream. Stripe: Mumbai Metro blue. A real, specific choice
# rather than "some colour" -- picked to match the South Mumbai / coastal
# metro city character materials_c.py already establishes, and it mirrors
# Mumbai Metro Line 1's actual white-and-blue livery.
TRAIN_BODY = (0.86, 0.85, 0.80)
TRAIN_STRIPE = (0.06, 0.28, 0.58)


def train_body(name="MAT_TRAIN_BODY"):
    return _flat(name, TRAIN_BODY, 0.35, metallic=0.25)


def train_stripe(name="MAT_TRAIN_STRIPE"):
    return _flat(name, TRAIN_STRIPE, 0.30, metallic=0.30)


def train_roof(name="MAT_TRAIN_ROOF"):
    return _flat(name, (0.42, 0.42, 0.44), 0.55, metallic=0.4)


def train_bogie(name="MAT_TRAIN_BOGIE"):
    return _flat(name, (0.10, 0.10, 0.10), 0.6, metallic=0.5)


# ===========================================================================
# WATERLINE STAINING -- dark algal band + efflorescence, river-adjacent
# piers only (x=135, x=225 -- both structures). SPEC.md: "Waterline staining
# on every pier that meets water".
# ===========================================================================
def waterline_stain(name="MAT_CONCRETE_PIER_WATERLINE", water_z=-2.0):
    """A dedicated variant of MAT_CONCRETE_PIER for the river-adjacent piers.

    Object-space Z on these columns IS world Z (meshlib bakes the centre
    into the vertices and leaves the object's own matrix at identity), so a
    plain Object-coordinate Z band works without any per-object tuning.
    Dark algae right at the waterline, a pale efflorescence bloom just
    above it (the classic tide-line bleach ring), fading back to plain
    concrete by ~2 m above the water.
    """
    existing = bpy.data.materials.get(name)
    if existing is not None:
        return existing
    base = bpy.data.materials.get("MAT_CONCRETE_PIER")
    if base is None:
        return None
    mat = base.copy()
    mat.name = name
    nt = mat.node_tree

    bsdf = next(n for n in nt.nodes if n.bl_idname == "ShaderNodeBsdfPrincipled")
    base_col_input = bsdf.inputs["Base Color"]
    src = base_col_input.links[0].from_socket if base_col_input.links else None

    tc = nt.nodes.new("ShaderNodeTexCoord")
    tc.location = (-1400, -700)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-1200, -700)
    nt.links.new(tc.outputs["Object"], sep.inputs["Vector"])

    # algae band: dark, wet-looking, strongest at/just below the waterline
    algae_ramp = nt.nodes.new("ShaderNodeValToRGB")
    algae_ramp.location = (-950, -600)
    els = algae_ramp.color_ramp.elements
    els[0].position = 0.0
    els[0].color = (1, 1, 1, 1)
    els[1].position = 1.0
    els[1].color = (0, 0, 0, 1)
    mp = nt.nodes.new("ShaderNodeMapRange")
    mp.location = (-1100, -600)
    mp.inputs["From Min"].default_value = water_z - 0.4
    mp.inputs["From Max"].default_value = water_z + 1.0
    nt.links.new(sep.outputs["Z"], mp.inputs["Value"])
    nt.links.new(mp.outputs["Result"], algae_ramp.inputs["Fac"])

    algae_col = nt.nodes.new("ShaderNodeRGB")
    algae_col.location = (-950, -820)
    algae_col.outputs[0].default_value = (0.028, 0.055, 0.032, 1)

    mix_algae = nt.nodes.new("ShaderNodeMix")
    mix_algae.data_type = "RGBA"
    mix_algae.location = (-650, -500)
    if src is not None:
        nt.links.new(src, mix_algae.inputs[6])
    else:
        mix_algae.inputs[6].default_value = bsdf.inputs["Base Color"].default_value
    nt.links.new(algae_col.outputs[0], mix_algae.inputs[7])
    nt.links.new(algae_ramp.outputs["Color"], mix_algae.inputs["Factor"])

    # efflorescence bloom just above the algae band -- pale leached salt
    effl_ramp = nt.nodes.new("ShaderNodeValToRGB")
    effl_ramp.location = (-950, -280)
    e = effl_ramp.color_ramp.elements
    e[0].position = 0.35
    e[0].color = (0, 0, 0, 1)
    e[1].position = 0.55
    e[1].color = (1, 1, 1, 1)
    e0 = effl_ramp.color_ramp.elements.new(0.75)
    e0.color = (0, 0, 0, 1)
    mp2 = nt.nodes.new("ShaderNodeMapRange")
    mp2.location = (-1100, -280)
    mp2.inputs["From Min"].default_value = water_z + 0.3
    mp2.inputs["From Max"].default_value = water_z + 2.2
    nt.links.new(sep.outputs["Z"], mp2.inputs["Value"])
    nt.links.new(mp2.outputs["Result"], effl_ramp.inputs["Fac"])

    effl_col = nt.nodes.new("ShaderNodeRGB")
    effl_col.location = (-950, -60)
    effl_col.outputs[0].default_value = (0.70, 0.68, 0.62, 1)

    mix_effl = nt.nodes.new("ShaderNodeMix")
    mix_effl.data_type = "RGBA"
    mix_effl.location = (-350, -350)
    nt.links.new(mix_algae.outputs[2], mix_effl.inputs[6])
    nt.links.new(effl_col.outputs[0], mix_effl.inputs[7])
    nt.links.new(effl_ramp.outputs["Color"], mix_effl.inputs["Factor"])

    nt.links.new(mix_effl.outputs[2], bsdf.inputs["Base Color"])
    return mat


def apply_waterline_staining(pier_name_prefixes, log=print):
    """Swap MAT_CONCRETE_PIER_WATERLINE onto every mesh whose name starts
    with one of the given prefixes (the river-adjacent pier columns)."""
    mat = waterline_stain()
    if mat is None:
        log("  stain   : SKIPPED, MAT_CONCRETE_PIER_WATERLINE could not be built")
        return 0
    n = 0
    for ob in bpy.data.objects:
        if ob.type != "MESH" or not ob.name.startswith(pier_name_prefixes):
            continue
        if not ob.data.materials:
            continue
        for i in range(len(ob.data.materials)):
            if ob.data.materials[i] and ob.data.materials[i].name == "MAT_CONCRETE_PIER":
                ob.data.materials[i] = mat
        n += 1
    log(f"  stain   : waterline staining applied to {n} pier columns")
    return n


# ===========================================================================
# DISTANT SILHOUETTES -- routed through _aerial(), lower albedo, flat roof
# ===========================================================================
def silhouette(name="MAT_BLDG_FAR_SILHOUETTE"):
    """Scale reference, not architecture: dim, hazed-back, never the
    brightest thing in frame. Mirrors materials.py's own `simple()` pattern
    exactly (an RGB node feeding `_aerial`, not a BSDF output socket --
    `_aerial` inspects `src.outputs`/`src.bl_idname`, so it needs a NODE,
    not a socket)."""
    m, fresh = MAT._new(name)
    if not fresh:
        return m
    nt = m.node_tree
    col = (0.16, 0.17, 0.19)
    b = MAT._bsdf(nt, 400, 0)
    b.inputs["Base Color"].default_value = (*col, 1.0)
    b.inputs["Roughness"].default_value = 0.85
    rgb = nt.nodes.new("ShaderNodeRGB")
    rgb.location = (-400, 300)
    rgb.outputs[0].default_value = (*col, 1.0)
    nt.links.new(MAT._aerial(nt, rgb, 120, 300).outputs[2],
                b.inputs["Base Color"])
    o = MAT._out(nt)
    nt.links.new(b.outputs["BSDF"], o.inputs["Surface"])
    return m

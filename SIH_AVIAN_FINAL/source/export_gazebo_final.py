"""SIH_AVIAN_FINAL -- export the corridor as a Gazebo (SDF) world.

    blender -b --python run_blender.py -- source/export_gazebo_final.py

Adapted from AVIAN_ENVIRONMENT/source/export_gazebo.py's structure (SDF
writing helpers, defect markers), NOT imported unchanged: that module
hardcodes REV-C's corridor window and a -2100 m origin shift for a research
zone nowhere near this scene's. This corridor is already only 360 m and
close to the origin, so no filtering or origin shift is needed -- ORIGIN=
(0,0,0), every primitive included.

MATERIALS -- READ FROM THE LIVE NODE GRAPH, NOT A SECOND TABLE
----------------------------------------------------------------
Every shader in this scene is an object-space procedural node graph (noise,
ramps, mixes), and only the crack decals have UVs -- nothing here can be
baked to a texture, and Gazebo gets no image to sample even if it could.
The correct translation is a flat colour per visual, resolved from each
object's ACTUAL Blender material at export time:

  1. Base Color unlinked -> use its constant directly (water() sets this,
     for instance).
  2. Base Color linked -> walk backward through the node graph looking for
     the first node that actually carries an authored constant: a
     ShaderNodeRGB, a ShaderNodeValToRGB (colour ramp, stops averaged), or
     an UNLINKED colour-type input socket on any node along the way (e.g.
     spall_face()'s rust tint lives in a Mix node's unwired "B" input, not
     a separate node -- checked on each node before recursing deeper, so
     the immediate constant wins over a more distant one).
  3. Nothing resolvable -> mid-grey, and the material is named in a logged
     warning, once per material, so an unmapped case is visible rather than
     silently invisible.

This reads the graph MATERIALS.PY ACTUALLY BUILT, every time, so it can
never drift from a hand-typed second copy of the same numbers -- if
materials.py changes a colour, this reflects it on the next export with no
edit here.

One exception, both explicitly requested rather than derived: water's
photoreal colour is realistically murky (silt, not a swimming pool) and its
Gazebo transparency has no analogue in the Blender shader at all, so both
are set directly for the flat-colour schematic view.
"""
from __future__ import annotations
import hashlib
import importlib.util
import json
import os
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom

import bpy
import mathutils

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
SCENE_DIR = os.path.join(ROOT, "scene")
GZ_DIR = os.path.join(ROOT, "gazebo")
WORLDS = os.path.join(GZ_DIR, "worlds")
MODELS = os.path.join(GZ_DIR, "models")

BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
COLLISION_ASSET = os.path.join(SCENE_DIR, "collision",
                               "avian_bridge_collision.json")
ORIGIN = (0.0, 0.0, 0.0)          # no shift needed -- see module docstring

# Fallback only -- used when a primitive's own object can't be found in the
# scene (should not happen) or carries no material at all.
KIND_COLOUR = {
    "deck_slab": (0.62, 0.61, 0.58), "deck_box": (0.58, 0.57, 0.55),
    "girder": (0.55, 0.54, 0.52), "diaphragm": (0.52, 0.51, 0.49),
    "parapet": (0.68, 0.67, 0.64), "pier_cap": (0.54, 0.53, 0.51),
    "pier_column": (0.50, 0.49, 0.47), "pier_footing": (0.44, 0.43, 0.41),
    "pier_collar": (0.48, 0.47, 0.45), "bearing": (0.16, 0.16, 0.17),
    "joint_gap": (0.24, 0.24, 0.25), "joint_nose": (0.30, 0.30, 0.31),
    "median": (0.60, 0.58, 0.54), "wearing": (0.20, 0.20, 0.21),
    "expansion_joint": (0.26, 0.26, 0.27), "abutment": (0.50, 0.49, 0.47),
    "rail": (0.42, 0.42, 0.45), "track_slab": (0.56, 0.55, 0.53),
    "catenary_mast": (0.30, 0.30, 0.32), "cable_trough": (0.50, 0.49, 0.47),
    "access_hatch": (0.28, 0.28, 0.30), "landing_pad": (0.60, 0.58, 0.42),
}
DEFAULT_COLOUR = (0.55, 0.55, 0.55)
DEFECT_COLOUR = (0.85, 0.16, 0.10)
HERO_COLOUR = (1.00, 0.75, 0.00)

# The collision exporter also synthesizes a flat "ENV_GROUND_PLANE" box
# (kind="ground") with no real corresponding Blender object -- resolved by
# MATERIAL name directly (materials.py's ground_mat()) rather than via an
# object, same reasoning as the water case above.
KIND_MATERIAL_NAME = {"ground": "MAT_GROUND", "water": "MAT_RIVER"}

# Explicit overrides -- not derived, because there is nothing to derive
# from (transparency has no Blender-shader analogue) or the photoreal
# colour is deliberately unlike what reads well as a flat schematic tint
# (water() is realistically murky silt, per its own docstring).
WATER_COLOUR = (0.14, 0.34, 0.36)
WATER_TRANSPARENCY = 0.35
# The collision exporter SYNTHESIZES a simplified "ENV_WATER_SURFACE" box
# primitive for the river (kind="water") rather than exporting the real
# terrain_final.py object ("ENV_RIVER_WATER") by name -- matched by kind,
# not name, or this silently never fires and water falls through to
# DEFAULT_COLOUR grey (which is exactly what happened on the first pass).
WATER_NAME_HINTS = ("ENV_RIVER_WATER", "ENV_WATER_SURFACE")
WATER_KINDS = ("water",)

# The one material built with per-instance (Object Info -> Random) colour
# variation -- see materials_final.vehicle_body(). Structural knowledge of
# THIS repo's own authored materials, not a duplicated colour table.
_PER_INSTANCE_MATERIALS = {"MAT_VEHICLE_BODY"}


def _common():
    root = os.environ.get("AVIAN_COMMON_DIR") or os.path.join(
        REPO, "avian_common")
    spec = importlib.util.spec_from_file_location(
        "_avian_common", os.path.join(root, "decompose.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _shift(c):
    return (c[0] - ORIGIN[0], c[1] - ORIGIN[1], c[2] - ORIGIN[2])


def _sub(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, **attrs)
    if text is not None:
        e.text = text
    return e


def _model_package(dirname, sdf_name, description):
    d = os.path.join(MODELS, dirname)
    os.makedirs(d, exist_ok=True)
    cfg = ET.Element("model")
    _sub(cfg, "name", dirname)
    _sub(cfg, "version", "1.0")
    sdf = _sub(cfg, "sdf", sdf_name)
    sdf.set("version", "1.9")
    author = _sub(cfg, "author")
    _sub(author, "name", "SIH_AVIAN_FINAL")
    _sub(cfg, "description", description)
    _write(cfg, os.path.join(d, "model.config"))
    return d


def _write(elem, path):
    raw = ET.tostring(elem, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    with open(path, "w") as f:
        f.write(pretty)
    return os.path.getsize(path)


def _write_material(el, colour, transparency=None):
    """`el` is the <visual> element. <transparency> is a child of <visual>
    itself in the SDF schema, NOT of <material> -- nesting it under
    <material> parses (sdformat tolerates and auto-corrects unknown
    elements) but logs a schema warning on every `gz sdf -k`, caught by
    actually reading gz's own output rather than trusting "it loaded"."""
    r, g, b = (max(0.0, min(1.0, v)) for v in colour)
    mat = _sub(el, "material")
    _sub(mat, "ambient", f"{r*0.4:.3f} {g*0.4:.3f} {b*0.4:.3f} 1")
    _sub(mat, "diffuse", f"{r:.3f} {g:.3f} {b:.3f} 1")
    _sub(mat, "specular", "0.1 0.1 0.1 1")
    if transparency is not None:
        _sub(el, "transparency", f"{transparency:.2f}")


# ===========================================================================
# COLOUR RESOLUTION -- reads the live node graph, see module docstring
# ===========================================================================
def _find_bsdf(nt):
    return next((n for n in nt.nodes
                if n.bl_idname == "ShaderNodeBsdfPrincipled"), None)


def _walk_colour_source(start_node):
    seen = set()
    stack = [start_node]
    while stack:
        node = stack.pop()
        if node is None or id(node) in seen:
            continue
        seen.add(id(node))
        if node.bl_idname == "ShaderNodeRGB":
            c = node.outputs[0].default_value
            return (c[0], c[1], c[2])
        if node.bl_idname == "ShaderNodeValToRGB":
            els = node.color_ramp.elements
            r = sum(e.color[0] for e in els) / len(els)
            g = sum(e.color[1] for e in els) / len(els)
            b = sum(e.color[2] for e in els) / len(els)
            return (r, g, b)
        # An unlinked RGBA input holds an authored constant directly on
        # THIS node (e.g. a Mix node's "B" colour when only "A" leads
        # further into the shader network) -- checked before recursing
        # deeper, so the immediate constant wins over a more distant one.
        for sock in node.inputs:
            if (not sock.is_linked and sock.type == "RGBA"):
                c = tuple(round(v, 4) for v in sock.default_value[:3])
                if c not in ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)):
                    return c
        for sock in node.inputs:
            if sock.is_linked:
                stack.append(sock.links[0].from_node)
    return None


def _object_info_ramp(start_node):
    """If Base Color is ultimately driven by Object Info -> Random into a
    ColorRamp, return that ramp node -- the per-instance case needs the
    object's own random value, not one averaged colour for everyone."""
    seen = set()
    stack = [start_node]
    while stack:
        node = stack.pop()
        if node is None or id(node) in seen:
            continue
        seen.add(id(node))
        if node.bl_idname == "ShaderNodeValToRGB":
            for sock in node.inputs:
                if (sock.is_linked and
                        sock.links[0].from_node.bl_idname == "ShaderNodeObjectInfo"):
                    return node
        for sock in node.inputs:
            if sock.is_linked:
                stack.append(sock.links[0].from_node)
    return None


def _object_random(ob):
    """Approximates Blender's internal Object Info 'Random' output (there
    is no documented, bit-exact Python accessor for it) with a stable hash
    of the object's name. Used only to pick a band from an already
    CONSTANT-interpolated ColorRamp read live from the material -- it will
    not always land on the identical stop Cycles picked for that object in
    the photoreal render, but it draws from the same real colour palette,
    is deterministic, and gives each instance a plausible, varied colour,
    which is the actual intent."""
    h = hashlib.md5(ob.name.encode()).hexdigest()
    return (int(h, 16) % 100000) / 100000.0


def _resolve_material_colour(mat, ob=None):
    if mat is None or mat.node_tree is None:
        return None
    nt = mat.node_tree
    bsdf = _find_bsdf(nt)
    if bsdf is None:
        return None
    inp = bsdf.inputs.get("Base Color")
    if inp is None:
        return None
    if not inp.is_linked:
        c = inp.default_value
        return (c[0], c[1], c[2])
    src = inp.links[0].from_node
    if ob is not None and mat.name in _PER_INSTANCE_MATERIALS:
        ramp = _object_info_ramp(src)
        if ramp is not None:
            els = sorted(ramp.color_ramp.elements, key=lambda e: e.position)
            t = _object_random(ob)
            band = els[0]
            for e in els:
                if e.position <= t:
                    band = e
            c = band.color
            return (c[0], c[1], c[2])
    return _walk_colour_source(src)


_material_colour_cache = {}


def _resolve_object_colour(ob, warnings, log):
    if ob is None or ob.type != "MESH" or not ob.data.materials:
        return None
    mat = ob.data.materials[0]
    if mat is None:
        return None
    if mat.name in _PER_INSTANCE_MATERIALS:
        return _resolve_material_colour(mat, ob)
    if mat.name in _material_colour_cache:
        return _material_colour_cache[mat.name]
    c = _resolve_material_colour(mat, None)
    if c is None and mat.name not in warnings:
        warnings.add(mat.name)
        log(f"  WARNING : material {mat.name!r} (object {ob.name}) has no "
            f"resolvable colour -- falling back to mid-grey")
    _material_colour_cache[mat.name] = c
    return c


def _colour_for(ob, kind, warnings, log):
    is_water = kind in WATER_KINDS or (
        ob is not None and any(ob.name.startswith(h) for h in WATER_NAME_HINTS))
    if is_water:
        return WATER_COLOUR, WATER_TRANSPARENCY
    c = _resolve_object_colour(ob, warnings, log) if ob is not None else None
    if c is not None:
        return c, None
    mat_name = KIND_MATERIAL_NAME.get(kind)
    if mat_name is not None:
        c = _resolve_material_colour(bpy.data.materials.get(mat_name))
        if c is not None:
            return c, None
    return KIND_COLOUR.get(kind, DEFAULT_COLOUR), None


# ===========================================================================
# EXPORT
# ===========================================================================
def export(log=print):
    t0 = time.time()
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    common = _common()
    prims = common.load_primitives(COLLISION_ASSET)
    log(f"  source  : {len(prims)} primitives from "
        f"{os.path.basename(COLLISION_ASSET)}")

    os.makedirs(WORLDS, exist_ok=True)
    os.makedirs(MODELS, exist_ok=True)
    warnings = set()

    groups = {"avian_final_road": ("BR_", "road bridge structure"),
              "avian_final_metro": ("MB_", "metro viaduct structure"),
              "avian_final_terrain": ("ENV", "terrain and river"),
              "avian_final_base": ("AVI_BASE_", "drone base landing pads")}
    written = {}
    poses = {}
    distinct_colours = set()

    for dirname, (prefix, desc) in groups.items():
        sel = [p for p in prims if str(p.get("name", "")).startswith(prefix)]
        if not sel:
            continue
        d = _model_package(dirname, "model.sdf", desc)
        sdf = ET.Element("sdf", version="1.9")
        model = _sub(sdf, "model", name=dirname)
        _sub(model, "static", "true")
        _sub(model, "pose", "0 0 0 0 0 0")
        link = _sub(model, "link", name="link")
        for p in sel:
            nm = p["name"]
            ob = bpy.data.objects.get(nm)
            colour, transparency = _colour_for(ob, p.get("kind"), warnings, log)
            distinct_colours.add(tuple(round(v, 3) for v in colour))
            c = _shift(p["centre"])
            poses[nm] = p["centre"]
            is_cyl = p.get("type") == "CYLINDER"
            for tag in ("collision", "visual"):
                el = _sub(link, tag, name=f"{nm}_{tag}")
                _sub(el, "pose",
                     f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 "
                     f"{p.get('yaw', 0.0):.6f}")
                geo = _sub(el, "geometry")
                if is_cyl:
                    cyl = _sub(geo, "cylinder")
                    _sub(cyl, "radius", f"{p['radius']:.6f}")
                    _sub(cyl, "length", f"{p['half_height']*2:.6f}")
                else:
                    half = p["half_extents"]
                    box = _sub(geo, "box")
                    _sub(box, "size", f"{half[0]*2:.6f} {half[1]*2:.6f} "
                                      f"{half[2]*2:.6f}")
                if tag == "visual":
                    _write_material(el, colour, transparency)
        size = _write(sdf, os.path.join(d, "model.sdf"))
        written[dirname] = {"members": len(sel), "bytes": size}
        log(f"  model   : {dirname:<20} {len(sel):>5} members, "
            f"{size/1e6:.2f} MB")

    # ---- visual-only groups: vehicles, vegetation --------------------------
    # Not in the collision JSON at all (decorative, not obstacles -- see
    # README) -- built here straight from the live scene via the SAME
    # shared decomposition (avian_common.decompose.obb) the collision
    # exporter uses, so there is still only one box-fitting implementation.
    for dirname, prefix, desc in (
        ("avian_final_vehicles", "VEH_", "road traffic (visual only)"),
        ("avian_final_vegetation", "ENV_BANKVEG_", "bank trees (visual only)"),
    ):
        n = _visual_only_group(dirname, prefix, desc, common, warnings,
                               distinct_colours, log)
        if n:
            written[dirname] = n

    skipped = len(prims) - sum(v["members"] for v in written.values()
                              if isinstance(v, dict) and "members" in v)
    n_def = _export_defects(log, poses, warnings, distinct_colours)
    world_path, world_bytes = _write_world(written, log)

    manifest = {
        "revision": "SIH_AVIAN_FINAL",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "world": os.path.relpath(world_path, ROOT),
        "world_origin_offset_m": list(ORIGIN),
        "origin_note": "no shift -- corridor is 0..360, already near origin",
        "source_primitives": len(prims),
        "exported_primitives": sum(v["members"] for v in written.values()
                                  if isinstance(v, dict) and "members" in v),
        "skipped": skipped,
        "models": written,
        "defect_models": n_def,
        "world_bytes": world_bytes,
        "distinct_diffuse_colours": len(distinct_colours),
        "materials_unresolved": sorted(warnings),
    }
    mpath = os.path.join(GZ_DIR, "avian_final_gazebo_manifest.json")
    with open(mpath, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  manifest: {os.path.basename(mpath)}")
    log(f"  colours : {len(distinct_colours)} distinct diffuse values, "
        f"{len(warnings)} materials unresolved -> mid-grey")
    log(f"  export  : {time.time()-t0:.1f} s")
    return manifest


def _visual_only_group(dirname, prefix, desc, common, warnings,
                       distinct_colours, log):
    objs = [o for o in bpy.data.objects
           if o.type == "MESH" and o.name.startswith(prefix)]
    if not objs:
        return None
    d = _model_package(dirname, "model.sdf", desc)
    sdf = ET.Element("sdf", version="1.9")
    model = _sub(sdf, "model", name=dirname)
    _sub(model, "static", "true")
    _sub(model, "pose", "0 0 0 0 0 0")
    link = _sub(model, "link", name="link")
    n = 0
    for ob in objs:
        centre, half, yaw, lo, hi = common.obb(ob, mathutils)
        colour, transparency = _colour_for(ob, ob.get("avi_kind"),
                                           warnings, log)
        distinct_colours.add(tuple(round(v, 3) for v in colour))
        c = _shift(centre)
        el = _sub(link, "visual", name=f"{ob.name}_visual")
        _sub(el, "pose", f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 {yaw:.6f}")
        geo = _sub(el, "geometry")
        box = _sub(geo, "box")
        _sub(box, "size", f"{half[0]*2:.6f} {half[1]*2:.6f} {half[2]*2:.6f}")
        _write_material(el, colour, transparency)
        n += 1
    size = _write(sdf, os.path.join(d, "model.sdf"))
    log(f"  model   : {dirname:<20} {n:>5} visuals (no collision), "
        f"{size/1e6:.2f} MB")
    return {"members": n, "bytes": size, "visual_only": True}


def _export_defects(log, poses, warnings, distinct_colours):
    d = _model_package("avian_final_defects", "model.sdf",
                       "defect markers at ground-truth poses")
    sdf = ET.Element("sdf", version="1.9")
    model = _sub(sdf, "model", name="avian_final_defects")
    _sub(model, "static", "true")
    _sub(model, "pose", "0 0 0 0 0 0")
    link = _sub(model, "link", name="link")

    n = 0
    for fn in ("AVIAN_defect_ground_truth_FINAL.json",
               "AVIAN_metro_ground_truth_FINAL.json"):
        path = os.path.join(SCENE_DIR, fn)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            blob = json.load(f)
        recs = blob["defects"] if isinstance(blob, dict) and "defects" in blob \
            else blob
        for r in recs:
            p = r["position_m"]
            c = _shift(p)
            el = _sub(link, "visual", name=r["defect_id"])
            _sub(el, "pose", f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 0")
            geo = _sub(el, "geometry")
            sph = _sub(geo, "sphere")
            # avi_hero lives on the Blender OBJECT's custom properties (set
            # by build_final.py's ML.set_custom), not in the ground-truth
            # JSON record -- r.get("avi_hero") is always None/False here,
            # a real bug caught while wiring this up (the record dict never
            # had the field added to it, only the object did).
            ob = bpy.data.objects.get(r["defect_id"])
            hero = bool(ob and ob.get("avi_hero"))
            _sub(sph, "radius", "0.45" if hero else "0.25")
            # Resolve the defect's OWN material (MAT_SPALL_FACE_RUST,
            # MAT_REBAR_CORRODED, a per-instance crack decal, ...) rather
            # than a flat marker red -- damage should read as damage. The
            # hero gets the bigger radius for visibility "from across the
            # river", not a different colour -- it should read as REAL
            # damage, not a distinct marker colour.
            colour = _resolve_object_colour(ob, warnings, log) if ob else None
            if colour is None:
                colour = HERO_COLOUR if hero else DEFECT_COLOUR
            distinct_colours.add(tuple(round(v, 3) for v in colour))
            _write_material(el, colour)
            poses[r["defect_id"]] = p
            n += 1
    size = _write(sdf, os.path.join(d, "model.sdf"))
    log(f"  model   : avian_final_defects {n:>5} markers, {size/1e6:.2f} MB")
    return n


CAMERAS = [
    ("whole corridor from above", "180 -260 170 0 0.6 1.5708"),
    ("river crossing from the water", "160 6 -1 0 -1.0 1.0"),
    ("inter-structure corridor", "150 15 15 0 0 0"),
]


def _write_world(written, log):
    sdf = ET.Element("sdf", version="1.9")
    sdf.append(ET.Comment(
        " SIH_AVIAN_FINAL corridor, x in [0, 360] of the Blender model. "
        "No world origin shift (see export_gazebo_final.py docstring). "))
    world = _sub(sdf, "world", name="sih_avian_final")

    phys = _sub(world, "physics", name="default", type="dart")
    _sub(phys, "max_step_size", "0.002")
    _sub(phys, "real_time_factor", "1.0")
    for p in ("gz-sim-physics-system", "gz-sim-user-commands-system",
              "gz-sim-scene-broadcaster-system", "gz-sim-sensors-system",
              "gz-sim-contact-system"):
        pl = _sub(world, "plugin",
                 name=p.replace("gz-sim-", "gz::sim::systems::")
                 .replace("-system", "").replace("-", "_"), filename=p)
        if p.endswith("sensors-system"):
            _sub(pl, "render_engine", "ogre2")

    light = _sub(world, "light", name="sun", type="directional")
    _sub(light, "cast_shadows", "true")
    _sub(light, "pose", "180 0 200 0 0 0")
    _sub(light, "diffuse", "0.9 0.88 0.85 1")
    _sub(light, "specular", "0.2 0.2 0.2 1")
    _sub(light, "direction", "-0.3 0.9 -0.35")   # matches the low, raking
                                                  # BASELINE sun (~11 deg
                                                  # elevation, ~100 deg azimuth)

    gp = _sub(world, "model", name="ground_plane")
    _sub(gp, "static", "true")
    gl = _sub(gp, "link", name="link")
    for tag in ("collision", "visual"):
        el = _sub(gl, tag, name=tag)
        geo = _sub(el, "geometry")
        pl = _sub(geo, "plane")
        _sub(pl, "normal", "0 0 1")
        _sub(pl, "size", "600 300")
        if tag == "visual":
            mat = _sub(el, "material")
            _sub(mat, "ambient", "0.25 0.24 0.20 1")
            _sub(mat, "diffuse", "0.45 0.43 0.36 1")
    _sub(gp, "pose", "180 15 -1.3 0 0 0")

    for dirname in list(written) + ["avian_final_defects"]:
        inc = _sub(world, "include")
        _sub(inc, "uri", f"model://{dirname}")
        _sub(inc, "pose", "0 0 0 0 0 0")

    gui = _sub(world, "gui", fullscreen="0")
    cam = _sub(gui, "camera", name="user_camera")
    _sub(cam, "pose", CAMERAS[0][1])
    for label, pose in CAMERAS:
        gui.append(ET.Comment(f" view: {label} -> <pose>{pose}</pose> "))

    os.makedirs(WORLDS, exist_ok=True)
    path = os.path.join(WORLDS, "sih_avian_final.sdf")
    size = _write(sdf, path)
    log(f"  world   : {os.path.basename(path)}, {size/1e3:.1f} kB")
    return path, size


if __name__ == "__main__":
    export()
    print("FINAL_GAZEBO_COMPLETE")

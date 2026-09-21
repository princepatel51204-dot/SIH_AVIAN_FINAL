"""REV-C Stage 4 -- export the corridor as a Gazebo (SDF) world.

    blender -b --python run_blender.py -- source/export_gazebo.py

WHAT THIS IS AND IS NOT
-----------------------
Gazebo gets physics, sensors and ROS. Blender keeps photoreal rendering and
the dataset. This exporter therefore emits the COLLISION world -- the same
primitive set PyBullet already flies against -- plus flat per-material
colours so the structure is legible rather than uniform grey. It is not an
attempt to carry the procedural shaders across; they are object-space noise
with no UVs and would not survive the trip.

ONE DECOMPOSITION
-----------------
The primitives come from the asset `export_bridge_collision.py` writes, via
`avian_common`. Nothing here re-derives a box from a mesh. Two decomposers
would be two sources of the same numbers.

THE ORIGIN TRAP
---------------
Every pose is the world-space bounding-box centre carried in that asset,
which `_obb()` computed as `matrix_world @ ob.bound_box`. NOT
`matrix_world.translation` -- meshlib bakes the centre into the vertices and
leaves the origin at (0,0,0), so an origin-derived pose stacks all 1354
models on top of each other at the world origin, in a file that parses
cleanly and loads without error.

WORLD ORIGIN SHIFT
------------------
The research zone sits at x = 2100 m. SDF poses are float32 in the GUI, and
a camera opening at the origin would look at empty sky a kilometre from
anything. So the whole world is translated by -ORIGIN, recorded in the
manifest and as a comment in the world file. Every pose downstream depends
on it.
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_DIR = os.path.dirname(SOURCE_DIR)
REPO = os.path.dirname(ENV_DIR)
SCENE_DIR = os.path.join(ENV_DIR, "scene")
GZ_DIR = os.path.join(ENV_DIR, "gazebo")
WORLDS = os.path.join(GZ_DIR, "worlds")
MODELS = os.path.join(GZ_DIR, "models")

COLLISION_ASSET = os.path.join(SCENE_DIR, "collision",
                               "avian_bridge_collision.json")

# Export corridor. Narrower than the 4.5 km model on purpose: Gazebo with
# the full corridor will not fit alongside aircraft, and Stage 1b already
# met the OOM reaper with two Blender processes.
CORRIDOR_X0 = 1550.0
CORRIDOR_X1 = 2650.0

# World origin shift -- the research-zone centre goes to Gazebo's origin.
ORIGIN = (2100.0, 0.0, 0.0)

# Flat SDF colours, keyed by the avi_kind the primitive carries. Sourced
# from the same palette family as the Blender materials so the structure
# reads correctly; deliberately flat, because Gazebo is not the render path.
KIND_COLOUR = {
    "deck_slab": (0.62, 0.61, 0.58), "deck_box": (0.58, 0.57, 0.55),
    "girder": (0.55, 0.54, 0.52), "diaphragm": (0.52, 0.51, 0.49),
    "parapet": (0.68, 0.67, 0.64), "pier_cap": (0.54, 0.53, 0.51),
    "pier_column": (0.50, 0.49, 0.47), "pier_footing": (0.44, 0.43, 0.41),
    "pier_collar": (0.48, 0.47, 0.45), "bearing": (0.16, 0.16, 0.17),
    "joint_gap": (0.24, 0.24, 0.25), "joint_nose": (0.30, 0.30, 0.31),
    "median": (0.60, 0.58, 0.54), "wearing": (0.20, 0.20, 0.21),
    "expansion_joint": (0.26, 0.26, 0.27), "abutment": (0.50, 0.49, 0.47),
    # metro
    "rail": (0.42, 0.42, 0.45), "track_slab": (0.56, 0.55, 0.53),
    "catenary_mast": (0.30, 0.30, 0.32),
    "cable_trough": (0.50, 0.49, 0.47),
    "access_hatch": (0.28, 0.28, 0.30),
    "station_slab": (0.64, 0.63, 0.60),
    "station_wall": (0.66, 0.64, 0.60),
    "station_roof": (0.34, 0.35, 0.38),
    "station_stair": (0.58, 0.57, 0.54),
    "station_platform": (0.60, 0.59, 0.56),
}
DEFAULT_COLOUR = (0.55, 0.55, 0.55)
DEFECT_COLOUR = (0.85, 0.16, 0.10)


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


def _pose(centre, yaw=0.0):
    c = _shift(centre)
    return f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 {yaw:.6f}"


def _sub(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, **attrs)
    if text is not None:
        e.text = text
    return e


def _static_model(parent, name, pose, colour, size=None, radius=None,
                  length=None):
    """One static SDF model: collision + visual, box or cylinder."""
    m = _sub(parent, "model", name=name)
    _sub(m, "static", "true")
    _sub(m, "pose", pose)
    link = _sub(m, "link", name="link")
    for tag in ("collision", "visual"):
        el = _sub(link, tag, name=tag)
        geo = _sub(el, "geometry")
        if radius is not None:
            cyl = _sub(geo, "cylinder")
            _sub(cyl, "radius", f"{radius:.6f}")
            _sub(cyl, "length", f"{length:.6f}")
        else:
            box = _sub(geo, "box")
            _sub(box, "size",
                 f"{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}")
        if tag == "visual":
            mat = _sub(el, "material")
            r, g, b = colour
            _sub(mat, "ambient", f"{r*0.4:.3f} {g*0.4:.3f} {b*0.4:.3f} 1")
            _sub(mat, "diffuse", f"{r:.3f} {g:.3f} {b:.3f} 1")
            _sub(mat, "specular", "0.1 0.1 0.1 1")
    return m


def _model_package(dirname, sdf_name, description):
    """model.config beside a model.sdf, as Gazebo expects."""
    d = os.path.join(MODELS, dirname)
    os.makedirs(d, exist_ok=True)
    cfg = ET.Element("model")
    _sub(cfg, "name", dirname)
    _sub(cfg, "version", "1.0")
    sdf = _sub(cfg, "sdf", sdf_name)
    sdf.set("version", "1.9")
    author = _sub(cfg, "author")
    _sub(author, "name", "AVIAN REV-C")
    _sub(cfg, "description", description)
    _write(cfg, os.path.join(d, "model.config"))
    return d


def _write(elem, path):
    raw = ET.tostring(elem, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    with open(path, "w") as f:
        f.write(pretty)
    return os.path.getsize(path)


def _in_corridor(centre):
    return CORRIDOR_X0 <= centre[0] <= CORRIDOR_X1


def export(log=print):
    t0 = time.time()
    common = _common()
    prims = common.load_primitives(COLLISION_ASSET)
    log(f"  source  : {len(prims)} primitives from "
        f"{os.path.basename(COLLISION_ASSET)}")

    os.makedirs(WORLDS, exist_ok=True)
    os.makedirs(MODELS, exist_ok=True)

    groups = {"avian_bridge": ("BR_", "road bridge structure"),
              "avian_metro": ("MB_", "metro viaduct structure"),
              "avian_terrain": ("ENV", "terrain and river")}
    written = {}
    poses = {}
    skipped = 0

    for dirname, (prefix, desc) in groups.items():
        sel = [p for p in prims
               if str(p.get("name", "")).startswith(prefix)
               and _in_corridor(p["centre"])]
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
            colour = KIND_COLOUR.get(p.get("kind"), DEFAULT_COLOUR)
            c = _shift(p["centre"])
            poses[nm] = p["centre"]
            # Two schemas in the asset: BOX carries half_extents, CYLINDER
            # carries radius and half_height. The type string is
            # "CYLINDER", not "CYL" -- getting that wrong silently emits
            # every pier as a box, which would load and look almost right.
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
                    mat = _sub(el, "material")
                    r, g, b = colour
                    _sub(mat, "ambient",
                         f"{r*0.4:.3f} {g*0.4:.3f} {b*0.4:.3f} 1")
                    _sub(mat, "diffuse", f"{r:.3f} {g:.3f} {b:.3f} 1")
                    _sub(mat, "specular", "0.1 0.1 0.1 1")
        size = _write(sdf, os.path.join(d, "model.sdf"))
        written[dirname] = {"members": len(sel), "bytes": size}
        log(f"  model   : {dirname:<16} {len(sel):>5} members, "
            f"{size/1e6:.2f} MB")

    skipped = len(prims) - sum(v["members"] for v in written.values())

    # ---- defects, one named model each ---------------------------------
    n_def = _export_defects(log, poses)

    # ---- the world ------------------------------------------------------
    world_path, world_bytes = _write_world(written, log)

    manifest = {
        "revision": "REV_C_STAGE_4",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "world": os.path.relpath(world_path, ENV_DIR),
        "world_origin_offset_m": list(ORIGIN),
        "origin_note": ("Blender x=2100 (research-zone centre) maps to "
                        "Gazebo x=0. Add this offset to a Gazebo pose to "
                        "recover the Blender world coordinate."),
        "corridor_x": [CORRIDOR_X0, CORRIDOR_X1],
        "source_primitives": len(prims),
        "exported_primitives": sum(v["members"] for v in written.values()),
        "skipped_outside_corridor": skipped,
        "models": written,
        "defect_models": n_def,
        "world_bytes": world_bytes,
        "poses_world_m": poses,
    }
    mpath = os.path.join(GZ_DIR, "avian_gazebo_manifest.json")
    with open(mpath, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  manifest: {os.path.basename(mpath)}, origin offset "
        f"{ORIGIN[0]:.0f} m in x")
    log(f"  export  : {time.time()-t0:.1f} s")
    return manifest


def _export_defects(log, poses):
    """One named SDF model per defect, at its ground-truth pose.

    Named identically to the Blender object so a Gazebo-side detector can be
    scored against the same answer key. Tiny visual markers, no collision --
    a defect is a surface condition, not a body to bump into.
    """
    d = _model_package("avian_defects", "model.sdf",
                       "defect markers at ground-truth poses")
    sdf = ET.Element("sdf", version="1.9")
    model = _sub(sdf, "model", name="avian_defects")
    _sub(model, "static", "true")
    _sub(model, "pose", "0 0 0 0 0 0")
    link = _sub(model, "link", name="link")

    n = 0
    for fn in ("AVIAN_defect_ground_truth_REV_C.json",
               "AVIAN_metro_ground_truth_REV_C.json"):
        path = os.path.join(SCENE_DIR, fn)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            blob = json.load(f)
        recs = blob["defects"] if isinstance(blob, dict) \
            and "defects" in blob else blob
        for r in recs:
            p = r["position_m"]
            if not _in_corridor(p):
                continue
            c = _shift(p)
            el = _sub(link, "visual", name=r["defect_id"])
            _sub(el, "pose", f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 0")
            geo = _sub(el, "geometry")
            sph = _sub(geo, "sphere")
            _sub(sph, "radius", "0.25")
            mat = _sub(el, "material")
            r_, g_, b_ = DEFECT_COLOUR
            _sub(mat, "ambient", f"{r_*0.4:.3f} {g_*0.4:.3f} {b_*0.4:.3f} 1")
            _sub(mat, "diffuse", f"{r_:.3f} {g_:.3f} {b_:.3f} 1")
            poses[r["defect_id"]] = p
            n += 1
    size = _write(sdf, os.path.join(d, "model.sdf"))
    log(f"  model   : avian_defects    {n:>5} markers, {size/1e6:.2f} MB")
    return n


CAMERAS = [
    ("whole corridor from above", "0 -420 520 0 0.95 1.5708"),
    ("river crossing from the water", "0 -260 18 0 0.12 1.5708"),
    ("under the deck", "-40 0 12 0 0.18 0.0"),
    ("inter-structure corridor", "-60 26 26 0 0.05 0.35"),
]


def _write_world(written, log):
    sdf = ET.Element("sdf", version="1.9")
    sdf.append(ET.Comment(
        f" AVIAN REV-C corridor, x in [{CORRIDOR_X0:.0f}, {CORRIDOR_X1:.0f}] "
        f"of the Blender model.\n"
        f"     WORLD ORIGIN SHIFT: Blender x={ORIGIN[0]:.0f} maps to Gazebo "
        f"x=0. Add {ORIGIN[0]:.0f} to a Gazebo x to recover Blender x.\n"
        f"     Recorded in gazebo/avian_gazebo_manifest.json as "
        f"world_origin_offset_m. "))
    world = _sub(sdf, "world", name="avian_sic")

    phys = _sub(world, "physics", name="default", type="dart")
    _sub(phys, "max_step_size", "0.002")
    _sub(phys, "real_time_factor", "1.0")
    for p in ("gz-sim-physics-system", "gz-sim-user-commands-system",
              "gz-sim-scene-broadcaster-system", "gz-sim-sensors-system",
              "gz-sim-contact-system"):
        pl = _sub(world, "plugin", name=p.replace("gz-sim-", "gz::sim::systems::")
                  .replace("-system", "").replace("-", "_"), filename=p)
        if p.endswith("sensors-system"):
            _sub(pl, "render_engine", "ogre2")

    light = _sub(world, "light", name="sun", type="directional")
    _sub(light, "cast_shadows", "true")
    _sub(light, "pose", "0 0 300 0 0 0")
    _sub(light, "diffuse", "0.9 0.88 0.85 1")
    _sub(light, "specular", "0.2 0.2 0.2 1")
    _sub(light, "direction", "-0.5 0.3 -0.8")

    # ground plane at the model's own datum
    gp = _sub(world, "model", name="ground_plane")
    _sub(gp, "static", "true")
    gl = _sub(gp, "link", name="link")
    for tag in ("collision", "visual"):
        el = _sub(gl, tag, name=tag)
        geo = _sub(el, "geometry")
        pl = _sub(geo, "plane")
        _sub(pl, "normal", "0 0 1")
        _sub(pl, "size", "4000 4000")
        if tag == "visual":
            mat = _sub(el, "material")
            _sub(mat, "ambient", "0.25 0.24 0.20 1")
            _sub(mat, "diffuse", "0.45 0.43 0.36 1")
    _sub(gp, "pose", "0 0 0 0 0 0")

    for dirname in list(written) + ["avian_defects"]:
        inc = _sub(world, "include")
        _sub(inc, "uri", f"model://{dirname}")
        _sub(inc, "pose", "0 0 0 0 0 0")

    gui = _sub(world, "gui", fullscreen="0")
    cam = _sub(gui, "camera", name="user_camera")
    _sub(cam, "pose", CAMERAS[0][1])
    for label, pose in CAMERAS:
        gui.append(ET.Comment(f" view: {label} -> <pose>{pose}</pose> "))

    os.makedirs(WORLDS, exist_ok=True)
    path = os.path.join(WORLDS, "avian_sic.sdf")
    size = _write(sdf, path)
    log(f"  world   : {os.path.basename(path)}, {size/1e3:.1f} kB")
    return path, size


if __name__ == "__main__":
    export()
    print("REVC_PHASE_COMPLETE")

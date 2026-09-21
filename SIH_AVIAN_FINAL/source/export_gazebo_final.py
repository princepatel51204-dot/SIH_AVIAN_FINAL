"""SIH_AVIAN_FINAL -- export the corridor as a Gazebo (SDF) world.

    blender -b --python run_blender.py -- source/export_gazebo_final.py

Adapted from AVIAN_ENVIRONMENT/source/export_gazebo.py's structure (SDF
writing helpers, flat per-avi_kind colours, defect markers), NOT imported
unchanged: that module hardcodes REV-C's corridor window (x=1550..2650) and
a -2100 m origin shift for a research zone nowhere near this scene's. This
corridor is already only 360 m and close to the origin (well inside float32
precision, same reasoning as params.py's own note for the 4.5 km case), so
no filtering or origin shift is needed here at all -- ORIGIN=(0,0,0),
every primitive included.
"""
from __future__ import annotations
import importlib.util
import json
import os
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
SCENE_DIR = os.path.join(ROOT, "scene")
GZ_DIR = os.path.join(ROOT, "gazebo")
WORLDS = os.path.join(GZ_DIR, "worlds")
MODELS = os.path.join(GZ_DIR, "models")

COLLISION_ASSET = os.path.join(SCENE_DIR, "collision",
                               "avian_bridge_collision.json")
ORIGIN = (0.0, 0.0, 0.0)          # no shift needed -- see module docstring

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
    "access_hatch": (0.28, 0.28, 0.30),
    "landing_pad": (0.60, 0.58, 0.42),
}
DEFAULT_COLOUR = (0.55, 0.55, 0.55)
DEFECT_COLOUR = (0.85, 0.16, 0.10)
HERO_COLOUR = (1.00, 0.75, 0.00)


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


def export(log=print):
    t0 = time.time()
    common = _common()
    prims = common.load_primitives(COLLISION_ASSET)
    log(f"  source  : {len(prims)} primitives from "
        f"{os.path.basename(COLLISION_ASSET)}")

    os.makedirs(WORLDS, exist_ok=True)
    os.makedirs(MODELS, exist_ok=True)

    groups = {"avian_final_road": ("BR_", "road bridge structure"),
              "avian_final_metro": ("MB_", "metro viaduct structure"),
              "avian_final_terrain": ("ENV", "terrain and river"),
              # The landing pads: present in the collision JSON (kind=
              # landing_pad) but invisible in the actual Gazebo WORLD
              # without their own group here -- none of the three groups
              # above match "AVI_BASE_", so the pads would silently vanish
              # from the SDF a UAV actually flies in even though VF22
              # (which reads the collision JSON, not the SDF) still passes.
              "avian_final_base": ("AVI_BASE_", "drone base landing pads")}
    written = {}
    poses = {}

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
            colour = KIND_COLOUR.get(p.get("kind"), DEFAULT_COLOUR)
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
                    mat = _sub(el, "material")
                    r, g, b = colour
                    _sub(mat, "ambient",
                         f"{r*0.4:.3f} {g*0.4:.3f} {b*0.4:.3f} 1")
                    _sub(mat, "diffuse", f"{r:.3f} {g:.3f} {b:.3f} 1")
                    _sub(mat, "specular", "0.1 0.1 0.1 1")
        size = _write(sdf, os.path.join(d, "model.sdf"))
        written[dirname] = {"members": len(sel), "bytes": size}
        log(f"  model   : {dirname:<20} {len(sel):>5} members, "
            f"{size/1e6:.2f} MB")

    skipped = len(prims) - sum(v["members"] for v in written.values())
    n_def = _export_defects(log, poses)
    world_path, world_bytes = _write_world(written, log)

    manifest = {
        "revision": "SIH_AVIAN_FINAL",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "world": os.path.relpath(world_path, ROOT),
        "world_origin_offset_m": list(ORIGIN),
        "origin_note": "no shift -- corridor is 0..360, already near origin",
        "source_primitives": len(prims),
        "exported_primitives": sum(v["members"] for v in written.values()),
        "skipped": skipped,
        "models": written,
        "defect_models": n_def,
        "world_bytes": world_bytes,
    }
    mpath = os.path.join(GZ_DIR, "avian_final_gazebo_manifest.json")
    with open(mpath, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  manifest: {os.path.basename(mpath)}")
    log(f"  export  : {time.time()-t0:.1f} s")
    return manifest


def _export_defects(log, poses):
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
            hero = bool(r.get("avi_hero"))
            _sub(sph, "radius", "0.45" if hero else "0.25")
            mat = _sub(el, "material")
            r_, g_, b_ = HERO_COLOUR if hero else DEFECT_COLOUR
            _sub(mat, "ambient", f"{r_*0.4:.3f} {g_*0.4:.3f} {b_*0.4:.3f} 1")
            _sub(mat, "diffuse", f"{r_:.3f} {g_:.3f} {b_:.3f} 1")
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

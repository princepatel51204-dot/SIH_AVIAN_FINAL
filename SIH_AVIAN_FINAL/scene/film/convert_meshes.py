"""Convert the x500's Collada parts to glTF so Blender can read them.

This Blender build has no Collada importer -- bpy.ops.wm.collada_import does
not exist, and the only mesh importers present are obj / ply / stl / gltf /
fbx / x3d. Rather than re-model the airframe (which would mean shipping
something that is not the simulated drone), the three .dae parts are converted
outside Blender with trimesh + pycollada, which preserves both the geometry and
the Collada scene-graph transforms.

Verified after conversion, against the real part dimensions:
    NXP-HGD-CF   0.396 x 0.396 x 0.279 m   (x500 frame, 396 mm across)
    5010Base     28.6 x 28.6 x 18 mm       (5010 motor stator)
    5010Bell     59 x 59 x 21 mm           (5010 motor bell)

The propellers are already .stl and are imported directly; model.sdf applies a
0.8461538 scale to them, which drone.py reproduces.

    python3 scene/film/convert_meshes.py
"""
import json
import os

import numpy as np
import trimesh

SRC = "/home/prince/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes"
OUT = os.path.dirname(os.path.abspath(__file__)) + "/meshes"
PARTS = ("NXP-HGD-CF.dae", "5010Base.dae", "5010Bell.dae")


def main():
    os.makedirs(OUT, exist_ok=True)
    report = []
    for fn in PARTS:
        src = os.path.join(SRC, fn)
        scene = trimesh.load(src, force="scene")
        geoms = {k: g for k, g in scene.geometry.items() if hasattr(g, "faces")}
        dst = os.path.join(OUT, fn.replace(".dae", ".glb"))
        # dump(concatenate=True) BAKES the Collada scene-graph transforms into
        # the vertices. Exporting trimesh.Scene(geoms) instead would rebuild a
        # scene with identity transforms and silently drop them -- which made
        # the motor parts arrive in Blender 100x oversized.
        baked = scene.dump(concatenate=True)
        # glTF is a Y-up format and Blender's importer rotates +90 deg about X
        # on the way in. These Collada parts are already Z-up, so that import
        # rotation is spurious -- it lands the frame on its side (measured:
        # span 0.396 x 0.279 x 0.396 instead of 0.396 x 0.396 x 0.279).
        # Pre-rotating -90 deg about X here cancels it exactly: verified
        # round-trip z range -0.2526..0.0264, identical to the .dae source.
        baked.apply_transform(
            trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))
        baked.export(dst)
        lo, hi = scene.bounds
        span = [round(float(v), 4) for v in (hi - lo)]
        report.append({
            "src": fn, "out": os.path.basename(dst),
            "geometries": len(geoms),
            "vertices": int(sum(len(g.vertices) for g in geoms.values())),
            "faces": int(sum(len(g.faces) for g in geoms.values())),
            "span_m": span, "bytes": os.path.getsize(dst),
        })
        print(f"{fn:<20} {len(geoms)} geom  span {span} m  -> {os.path.basename(dst)}")
    json.dump(report, open(os.path.join(OUT, "conversion.json"), "w"), indent=1)
    print(f"\nwrote {len(report)} meshes + conversion.json to {OUT}")


if __name__ == "__main__":
    main()

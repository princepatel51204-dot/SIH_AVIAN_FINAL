"""Measure camera clearance for every frame of every beat.

For each beat's camera path (the same one render_film.py renders), find the
distance from the camera to the nearest surface of any scene mesh (the drone,
which is hidden in beats, and camera objects are excluded), and the camera-to-
defect range. Prints per-beat minima; nothing here is asserted, only measured.

Run:  blender -b scene/SIH_AVIAN_FINAL.blend -P scene/film/verify_clearance_film.py
"""
import bpy, sys, json
from mathutils import Vector
from mathutils.bvhtree import BVHTree
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/film")
sys.path.insert(0, "/home/prince/avian_rev_c/SIH_AVIAN_FINAL/scene/cine")
import render_film as RF
import shotlist as SL
import drone as D

dg = bpy.context.evaluated_depsgraph_get()
drone_objs = set(bpy.data.collections[D.COLL].all_objects)
trees = []
for ob in bpy.context.scene.objects:
    if ob.type != "MESH" or ob in drone_objs or not ob.visible_get():
        continue
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    if len(me.polygons) == 0:
        ev.to_mesh_clear(); continue
    mw = ob.matrix_world
    verts = [mw @ v.co for v in me.vertices]
    polys = [tuple(p.vertices) for p in me.polygons]
    trees.append((ob.name, BVHTree.FromPolygons(verts, polys)))
    ev.to_mesh_clear()
print(f"CLEAR scene meshes in BVH: {len(trees)}", flush=True)

res = []
for beat in SL.BEATS:
    path = RF.beat_camera_path(beat)
    tgt = RF.gt[beat["defect"]]["position_m"]
    best = (1e9, None)
    rng = min((Vector(tgt) - p[0]).length for p in path)
    for i, (pos, _, _) in enumerate(path):
        for name, t in trees:
            hit = t.find_nearest(pos)
            if hit[0] is not None and hit[3] < best[0]:
                best = (hit[3], (name, i))
    r = {"beat": beat["n"], "defect": beat["defect"], "kind": beat["kind"],
         "min_camera_clearance_m": round(best[0], 3), "at_frame": best[1][1], "nearest_object": best[1][0],
         "min_range_to_defect_m": round(rng, 3)}
    res.append(r); print("CLEAR", json.dumps(r), flush=True)
json.dump(res, open("/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media/_vfilm/clearance_result.json", "w"), indent=1)
print("CLEAR_DONE", flush=True)

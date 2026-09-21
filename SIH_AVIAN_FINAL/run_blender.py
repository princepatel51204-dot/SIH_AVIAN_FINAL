"""Launcher for running source/*.py scripts under system Blender's bundled
Python, working around `blender --python script.py` not adding the script's
own directory to sys.path the way plain `python script.py` does.

No pip `bpy` wheel exists for this machine's Python (3.12 -- bpy wheels are
cp311-only as of bpy 4.5.x), so build_scene_b.py etc. must run inside
Blender's own interpreter instead of a venv. This launcher makes that the
same invocation shape as the plain-python form the docs describe.

Usage (from AVIAN_ENVIRONMENT/):
    blender --background --python run_blender.py -- source/build_scene_b.py
    blender --background --python run_blender.py -- source/build_scene_b.py --render
"""
import os
import runpy
import sys

if "--" not in sys.argv:
    raise SystemExit("usage: blender --background --python run_blender.py -- <script.py> [args...]")

argv = sys.argv[sys.argv.index("--") + 1:]
if not argv:
    raise SystemExit("usage: blender --background --python run_blender.py -- <script.py> [args...]")

target = argv[0]
target_dir = os.path.dirname(os.path.abspath(target))
if target_dir not in sys.path:
    sys.path.insert(0, target_dir)

sys.argv = argv
runpy.run_path(target, run_name="__main__")

"""SIH_AVIAN_FINAL -- generate the PyBullet/Gazebo collision asset.

Calls AVIAN_UAV/simulation/export_bridge_collision.py's own `export()`
function unchanged (dynamically loaded, same pattern build_scene_c.py's
phase_collision uses) against THIS scene's .blend, rather than writing a
second decomposer -- "one decomposition" is the whole point of that module.

export_bridge_collision.py derives its corridor margin from objects tagged
avi_kind="inspection_sector", falling back to `import params as P` (REV-C's
own params.py, x=1650..2550) when none exist. This scene has no sector
objects (SIH_AVIAN_FINAL has no zones_final.py yet) and REV-C's fallback
range is nowhere near our 0..360 corridor, so `params` is hijacked in
sys.modules to point at THIS scene's params_final before the call --
`import params as P` then resolves to params_final's RESEARCH_X0/X1 (90/270)
instead, giving a sane margin (-10..370, comfortably covering the whole
corridor including embankments).

Usage:
    blender --background --python run_blender.py -- source/collision_final.py
"""
from __future__ import annotations
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
ENV_SRC = os.environ.get("AVIAN_ENV_SRC",
                         os.path.join(REPO, "AVIAN_ENVIRONMENT", "source"))
UAV_SIM = os.environ.get("AVIAN_UAV_SIM_DIR",
                         os.path.join(REPO, "AVIAN_UAV", "simulation"))
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import params_final as PF
sys.modules["params"] = PF          # see module docstring

BLEND_PATH = os.path.join(ROOT, "scene", "SIH_AVIAN_FINAL.blend")
COLLISION_DIR = os.path.join(ROOT, "scene", "collision")


def _load_export_bridge_collision():
    path = os.path.join(UAV_SIM, "export_bridge_collision.py")
    spec = importlib.util.spec_from_file_location(
        "_export_bridge_collision", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    print("== SIH_AVIAN_FINAL :: collision_final.py ==")
    mod = _load_export_bridge_collision()
    asset, manifest_path, manifest = mod.export(
        blend=BLEND_PATH, out_dir=COLLISION_DIR, margin_m=100.0, log=print)
    print(f"asset    {asset}")
    print(f"manifest {manifest_path}")
    print("FINAL_COLLISION_COMPLETE")


if __name__ == "__main__":
    main()

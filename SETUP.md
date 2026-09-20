# AVIAN REV-C — repo setup notes

Records how this repo was assembled and what was changed to get both
package gates passing, per the "normalise the layout once and record which
you chose" instruction.

## Source of each package

- `AVIAN_ENVIRONMENT/source/` — extracted from
  `~/Downloads/AVIAN_zips/AVIAN_REV_B.zip` (`AVIAN_REV_B/` folder).
- `AVIAN_UAV/` — extracted from `~/Downloads/AVIAN_2_UAV_PHASE1.zip`, chosen
  over `AVIAN_phase1.zip` / `AVIAN_phase34.zip` because it is the only variant
  carrying the `cad/` package (`params_b.py`, `kin_b.py`, `parts_b.py`,
  `assy_b.py`, `render.py`, `views.py`) that `tests/test_description.py` and
  `description/build_urdf.py` import. Confirmed as the signed-off build via
  `logs/phase1_report.json`: `verdict: PASS`, `total: 46`,
  `generated_utc: 2026-08-28T18:39:09Z`. Its top-level folder is already
  `AVIAN_UAV/` (flat — `simulation/`, `description/`, `cad/`, `sensors/`,
  `manipulator/` directly underneath), which matches the master brief's own
  §6.3/§11 spelling (`AVIAN_UAV/simulation/export_bridge_collision.py`), so no
  brief edit was needed. The other two zips use a nested
  `avian/sim/...` layout instead — **do not merge them in.**

## Fixes applied to the UAV package (packaging drift, not logic changes)

The `cad/` package was originally built and zipped from a different machine
(`/home/claude/avian`) and the flat `AVIAN_UAV/` layout is a repack of an
originally-nested `avian/` namespace package. Both left drift:

1. **`AVIAN_CAD_DIR` defaulted to `/home/claude/avian`**, which doesn't exist
   here. Fixed in `description/build_urdf.py` and `tests/test_description.py`
   to default to `<repo>/AVIAN_UAV/cad` instead (still overridable via the
   env var). Same class of fix needed in `description/export_meshes.py`,
   `simulation/vehicle.py`, `manipulator/arm.py`, and every `tests/test_*.py`
   — those all already respect `AVIAN_CAD_DIR`/`AVIAN_ENV_DIR` env vars
   correctly, so they're handled by exporting the vars below rather than by
   editing each file.
2. **`tests/test_description.py` and `scripts/make_phase1_report.py`
   hardcoded `avian/description/...`** — a leftover from the nested-package
   layout. Fixed to `description/...` (this repo's actual flat path).
3. **Nine files imported `from avian.sim...` / `from avian.sensors...` /
   `from avian.manipulator...`**, expecting the nested package that doesn't
   exist in this variant: `sensors/inertial.py`, `sensors/cameras.py`,
   `simulation/world.py`, `simulation/vehicle.py`, and five `tests/test_*.py`
   files. Rewritten to the flat equivalents (`simulation.*`, `sensors.*`,
   `manipulator.*`), since the flat layout is what the brief itself specifies.
4. **`tests/test_sensors.py` and `tests/test_manipulator.py` put
   `AVIAN_ENV_DIR` ahead of the UAV package root on `sys.path`.** The
   environment package has its own top-level `sensors.py` (Blender camera
   rigs) which shadowed `AVIAN_UAV/sensors/` (the PyBullet sensor package) and
   crashed on `import bpy`. Reordered so the UAV package root wins.

After these four fixes: `python3 tests/run_all.py` → **46 pass, 0 fail**,
genuinely (not from a stale `phase1_report.json`).

## Required environment variables

```bash
export AVIAN_CAD_DIR=/home/prince/avian_rev_c/AVIAN_UAV/cad
export AVIAN_ENV_DIR=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source
export AVIAN_ENV_BLEND=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source/AVIAN_Smart_Infrastructure_City_REV_B.blend
```

## Python environments

- **`AVIAN_UAV/` tests run under `/home/prince/avian_rev_c/.venv`**
  (`pybullet==3.2.7`, `numpy`, `cadquery` — installed fresh, this machine has
  no system pybullet). Python 3.12.

  ```bash
  /home/prince/avian_rev_c/.venv/bin/python3 tests/run_all.py
  ```

- **`AVIAN_ENVIRONMENT/` scripts run under system Blender (`/usr/bin/blender`,
  4.0.2), not a pip `bpy`.** `bpy` on PyPI only ships `cp311` wheels (as of
  4.5.x); this machine has Python 3.12 and no 3.11 interpreter, and there's no
  root access here to install one via apt. System Blender's bundled Python
  produces byte-identical gate results (V16 triangle count, V22 drift, etc.
  all match the reference numbers exactly), so this is a build-plumbing
  substitution, not a functional gap — but it does mean the actual Blender
  version in use is 4.0.2, not the 4.5 LTS the brief's dependency list names.
  Flagging in case that version gap ever matters (a REV-C node type or Python
  API call that only exists in 4.5).

  `blender --python script.py` does **not** add the script's own directory to
  `sys.path` the way `python script.py` does, so a small launcher,
  `AVIAN_ENVIRONMENT/run_blender.py`, does that and then runs the target:

  ```bash
  cd /home/prince/avian_rev_c/AVIAN_ENVIRONMENT
  blender --background --python run_blender.py -- source/build_scene_b.py
  blender --background --python run_blender.py -- source/build_scene_b.py --render
  ```

  Confirmed: **32/32** (21 REV-A + 11 REV-B), including
  **V22: 192 baseline, 192 now, max drift 0.00 mm**.

## Not yet done

- No git repo has been initialized here — worth doing before REV-C edits
  start, so stage-by-stage changes are reviewable and reversible.

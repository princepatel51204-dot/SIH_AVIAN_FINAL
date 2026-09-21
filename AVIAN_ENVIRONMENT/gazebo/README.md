# AVIAN REV-C in Gazebo

The road bridge and metro viaduct corridor, as an SDF world for Gazebo
Harmonic (gz-sim 8).

## See it

```bash
/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/view.sh
```

That is the whole thing. It sources ROS, sets both resource paths, and
launches the GUI. **The world loads paused — press Play, bottom left.**
Scroll to zoom, middle-drag to orbit, shift-middle-drag to pan.

Other modes:

```bash
./view.sh --check      # gz sdf -k, validate and exit
./view.sh --headless   # gz sim -s -r --iterations 100, no GUI
./shoot.sh             # render the four views below to screenshots/
```

## Two environment traps

**`gz` here is the ROS-vendored build.** Without sourcing ROS it reports
*"I cannot find any available 'gz' command — did you install any Gazebo
library?"*, which reads like Gazebo is missing when Harmonic 8.15.0 is
installed and working:

```bash
source /opt/ros/jazzy/setup.bash
```

**`gz sdf -k` needs `SDF_PATH`, not just `GZ_SIM_RESOURCE_PATH`.** `gz sim`
resolves `model://` through the latter; the standalone validator uses
sdformat's own `findFile`, which reads the former. With only the first set,
a perfectly good world reports `Unable to find uri[model://avian_bridge]`
and exits 255. `view.sh` sets both.

```bash
export GZ_SIM_RESOURCE_PATH=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/models
export SDF_PATH=$GZ_SIM_RESOURCE_PATH
```

## The world origin is shifted — read this before using any pose

The Blender model puts the research zone at **x = 2100 m**. SDF poses are
float32 and the GUI camera opens at the origin, so a world exported at true
chainage would lose precision and open looking at empty sky a kilometre
from anything.

So the export subtracts the offset:

```
Gazebo x = Blender x - 2100
Blender x = Gazebo x + 2100
```

Recorded as `world_origin_offset_m` in `avian_gazebo_manifest.json` and as
a comment at the top of `worlds/avian_sic.sdf`. Every pose in the world,
including the defect markers, is in shifted coordinates.

## Four views

Poses are `x y z roll pitch yaw` in **Gazebo** coordinates. `shoot.sh`
renders all four; the first is also the GUI's opening camera.

| View | Pose | Shows |
|---|---|---|
| Whole corridor from above | `0 -430 430 0 0.80 1.5708` | both structures over the full 1100 m, the station, the pier-free main span |
| River crossing | `0 -300 26 0 0.06 1.5708` | the 90 m main span and its air draft |
| Under the deck | `-30 0 14 0 0.10 0.0` | road soffit, girders, the under-deck inspection volume |
| Inter-structure corridor | `-70 30 30 0 0.10 0.55` | the gap between road bridge and metro, looking along the viaduct |

## What is in the world

| Model | Contents |
|---|---|
| `avian_bridge` | 664 `BR_` members in the corridor |
| `avian_metro` | 677 `MB_` members — box girder, single circular piers, track, masts, station |
| `avian_terrain` | 2 `ENV` bodies |
| `avian_defects` | 252 markers at ground-truth poses, named exactly as the Blender objects |

1343 collision primitives, the same set PyBullet flies against — both come
from one decomposition in `avian_common/decompose.py`. 11 of the 1354 total
fall outside the exported corridor.

**Materials are flat colours, deliberately.** Every Blender shader is
object-space procedural noise and only the crack decals carry UVs, so none
of it survives an export. Gazebo gets physics, sensors and ROS; Blender
keeps the photoreal and dataset path.

## Checks

`python3 ../source/validate_gazebo_c.py` runs V44–V50 against the exported
artifacts. `python3 ../source/sabotage_gazebo_c.py` breaks each one and
confirms it goes red.

The pair worth understanding is **V45 and V46**. V45 proves the right
number of bodies came out; V46 proves they are not all stacked at the
origin. Collapse every pose and the world still parses and still steps —
V44 and V48 stay green — so V46 is the only thing between a stacked export
and a green gate. It asserts a non-degenerate pose spread and prints the
measured envelope for that reason.

## Rebuild

```bash
cd /home/prince/avian_rev_c/AVIAN_ENVIRONMENT
blender -b --python run_blender.py -- source/export_gazebo.py
```

Reads `scene/collision/avian_bridge_collision.json`, so the collision asset
must exist first — `build_scene_c.py` writes it in its `--phase collision`.

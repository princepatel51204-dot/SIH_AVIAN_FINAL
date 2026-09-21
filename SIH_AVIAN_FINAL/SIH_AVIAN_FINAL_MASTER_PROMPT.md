# SIH_AVIAN_FINAL — Master Prompt

**One session. One road bridge, one metro viaduct, one river. No city.**
**Built in Blender, exported to Gazebo, with damage you can actually see.**

This file is the message pasted into Claude Code to start this build. It is
kept here, at the project root, mirroring `AVIAN_REV_C_MASTER_PROMPT.md`'s own
place at the `avian_rev_c` root. `concept/SPEC.md` and `concept/0*.png` are the
six drawings + dimension table that came with it — the actual specification.

## Locked pre-flight decisions (2026-09-21)

1. **Road bridge deck: slight vertical curve**, not flat. A gentle symmetric
   longitudinal crown, high point over the river main span (which only helps
   air draft), low point at the two abutments. `params_final.py` documents the
   exact rise and the resulting per-x soffit/pier numbers as measured values,
   not the SPEC.md constants taken literally at every x.
2. **No metro station.** Matches `SPEC.md`, which omits it to hold the
   ~900-object / <5-minute-build target.
3. **2–3 distant building silhouettes on the horizon for scale.** Simple,
   non-structural, `CITY_`-prefixed so they stay out of collision.

## What follows is the original prompt, verbatim

> I'm building a new environment, `SIH_AVIAN_FINAL`, replacing the 4.5 km REV-C
> scene with a compact one. Read `SIH_AVIAN_FINAL_MASTER_PROMPT.md` in full and
> unzip `SIH_AVIAN_FINAL_CONCEPT.zip` — the six drawings are the spec, `SPEC.md`
> has every dimension.
>
> **This is one session, not a staged build.** Work through the checkpoints in
> §7 in order; each one leaves a scene that loads. Do not stop between them
> unless a checkpoint fails.
>
> §3 of `AVIAN_REV_C_MASTER_PROMPT.md` (the working rules) still binds, including
> §3.10 — every new check must be proven capable of failing.
>
> Before writing code: give me **A–F** and only the blocking questions.

See `../AVIAN_REV_C_MASTER_PROMPT.md` §3 for the working rules (adopted
verbatim) and `concept/SPEC.md` for every dimension. Reused modules are reached
through `AVIAN_ENV_SRC` (default `../AVIAN_ENVIRONMENT/source`), following the
`AVIAN_CAD_DIR`/`AVIAN_COMMON_DIR` precedent in `avian_common/decompose.py`.

## Checkpoints (§7)

| # | Checkpoint | Done when |
|---|---|---|
| A | Geometry | Both bridges, piers, river, banks. Loads in Blender. Triangle count reported. |
| B | Materials | Shaders applied, waterline staining, low-sun `BASELINE`. Renders `CAM_01`, `CAM_05`. |
| C | Micro-detail | Formwork, joints, runoff, honeycombing, crack geometry over 3 mm. Renders `CAM_03`, `CAM_06`. |
| D | Defects | 96 placed, all fields, measured visibility, measured contrast, hero defect tagged. Baseline frozen. |
| E | Validation | New check set passes, each sabotage-tested. |
| F | Gazebo | SDF exported, loads in `gz sim`, screenshot taken. |

## Commands

```bash
cd /home/prince/avian_rev_c/SIH_AVIAN_FINAL
blender --background --python run_blender.py -- source/build_final.py

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/prince/avian_rev_c/SIH_AVIAN_FINAL/gazebo/models
gz sim -v 4 gazebo/worlds/sih_avian_final.sdf
```

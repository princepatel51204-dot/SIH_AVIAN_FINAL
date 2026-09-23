# SIH_AVIAN_FINAL — Phase 2 Step 1: the detection dataset

**Render the images the detector will actually see, from Blender, through the
UAV's own camera, at the standoff each defect requires.**

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`
for the original build and `SIH_AVIAN_DETECTION_MASTER_PROMPT.md` for the
steel/resolvability pass this one builds on).

## Locked pre-flight decisions (2026-09-22)

Per the prompt's own §8 blocking questions, each answered with the stated
default rather than re-asked, since the prompt gave a reasoned default for
every one:

1. **Scenario: BASELINE only.** The `scenario` field is carried in the
   manifest so a lighting sweep can be added later without a re-render of
   what already exists.
2. **No baked-in augmentation.** Renders stay clean (no synthetic motion
   blur or sensor noise baked into the pixels); that is left to the
   training pipeline. The manifest still carries the numbers
   (`avian_description_manifest.json`'s IMU gyro RMS, the aircraft's
   station-keeping tolerance) that a later augmentation step would need.
3. **No split column yet.** One manifest, ungenerated split. Recorded
   explicitly: any future split MUST be by `sector` (or by `defect_id`
   grouping standoff variants together), never at random or by image,
   since the same defect at two standoffs sharing one background must not
   land on both sides.

## Sensor intrinsics — read from source, not retyped

The resolvability argument (`SIH_AVIAN_DETECTION_MASTER_PROMPT.md`) rests on
**2.1478 mm/px @ 1 m**. That number is `AVIAN_UAV/sensors/cameras.py`'s
`RGBCamera` (`hfov_deg=69.0`) evaluated at its own `LOOP_RES=(640, 480)` —
confirmed against `AVIAN_UAV/logs/phase1_sensors.json`'s own measured S13
result ("GSD 2.1478 mm/px @1m"), not re-derived by hand. `CAPTURE_RES=
(1920, 1080)` is the same camera at 3x the linear resolution.

The REV-B sensor manifest (`AVIAN_ENVIRONMENT/source/
AVIAN_sensor_manifest_REV_B.json`)'s own `avi_gsd_mm_at_1m: 0.62` for
`AVI_SENSOR_RGB_FRONT` is a DIFFERENT number from a differently-specified
sensor entry (`avi_focal_length_mm`/`avi_sensor_width_mm` that do not
themselves reduce to 0.62 under the same pinhole formula either — an
internal inconsistency in that pre-existing file, not something this pass
edits or relies on). `visibility.py`'s own `RGB_GSD_MM_AT_1M = 0.62`
constant is this same figure, used only for that module's own internal
range calculation — this is the exact "unlinked constant" mistake flagged
once already; the dataset render must NOT reach for that number.

`dataset_final.py` therefore computes GSD itself, from the SAME formula
`cameras.py` uses (`2 * d * tan(hfov/2) / w`), at the ACTUAL Blender camera
angle and resolution used for that frame — asserted, not assumed, per VD02.

## Render engine: EEVEE, not Cycles

The 12 named pitch cameras use Cycles (`lighting.py`'s `configure_render`).
This pass uses EEVEE instead, for the same two reasons `contrast_c.py`
already established for its own per-defect renders: it is a rasteriser, so
it cannot hit the Cycles-after-`scene.ray_cast()` Embree segfault this
project has hit before, and it is fast enough to make an 850+ frame batch
a real option instead of an hours-long one.

## What follows is the original prompt, verbatim

> **Phase 2, step 1 — build the detection dataset. Read
> `SIH_AVIAN_DATASET_MASTER_PROMPT.md` in full. §3 of
> `AVIAN_REV_C_MASTER_PROMPT.md` still binds, including §3.10.**
>
> [... see the pasted prompt in the session transcript for the full text:
> §1 sensor fidelity, §2 positives/survey-standoff/negatives, §3 render
> passes, §4 manifest schema, §5 checks VD01-VD08, §6 gate, §7 commands,
> §8 blocking questions ...]

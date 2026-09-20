# AVIAN UAV — the aircraft that flies in the environment

Phase 1 complete. **46/46 acceptance checks pass.** Open
`reports/phase1_report.html` in a browser for the full gate.

## The aircraft

| | |
|---|---|
| Configuration | Coaxial **X8** — 4 arms × 2 rotors, alternating handedness |
| Modelled mass | **23.693 kg** (from a closed 389-row mass budget) |
| Max thrust | **536.3 N** → thrust-to-weight **2.31** |
| Manipulator | **6-DOF**, 950 mm reach, nozzle tool |
| Sensors | RGB ×3, depth, LiDAR, thermal, IMU, GNSS |
| Arm span | 575 mm per arm |

Yaw is decoupled from roll and pitch by the coaxial symmetry — the counter-
rotating pairs cancel reaction torque unless yaw is explicitly commanded.

## Run it

```bash
python3 description/build_urdf.py    # regenerate URDF from the CAD
python3 tests/run_all.py             # full acceptance gate + report, ~4 min
python3 scripts/make_phase1_report.py
```

Requires `pybullet`, `numpy`, `cadquery` (CAD export only).

## What is inside

| Folder | Contents |
|---|---|
| `cad/` | CadQuery/OCCT B-rep source, 389-row mass budget, kinematics (`kin_b.py`), parameters (`params_b.py`) |
| `description/` | URDF generator + generated `avian.urdf` + 17 exported link meshes (280,155 triangles) |
| `simulation/` | PyBullet vehicle, cascaded controller, world, safety supervisor, bridge-collision exporter |
| `sensors/` | Cameras (RGB/depth), LiDAR, IMU, GNSS — all rate-gated on simulation time |
| `manipulator/` | Verified IK, trajectory planning, collision checking |
| `tests/` | Six suites, 46 checks |
| `reports/`, `logs/` | Phase 1 gate report, HTML and JSON |

## Design rules this package enforces

- **The CAD is the single source of truth.** The URDF is *generated*; a
  hand-written one would be a second source of the same numbers and the two
  would drift. URDF↔CAD forward kinematics agree to **0.000 mm**.
- **PyBullet's IK is a proposer, not a solver.** Every proposal is checked
  against reach, joint limits, self-collision, environment collision, and the
  CAD's own FK before it is returned. It returns `None` with a reason rather
  than a pose that does not work.
- **Determinism is bit-exact** (0.00e+00 m between identical seeded runs).
  Every noise term is drawn from a per-sensor generator seeded from the world
  seed and the sensor name.

## Known limitations — stated, not buried

1. **Self-collision is detected, not prevented.** `URDF_USE_SELF_COLLISION`
   is off (enabling it changes already-validated flight dynamics). Planning
   must not command a pose `check_self_collision()` rejects.
2. **Link inertia is a point-mass aggregation.** Mass and CG are exact from
   the budget; the inertia tensor is approximate.
3. **Arm acceleration limits are estimated** (`a = τ/I`, I from downstream
   link masses — an optimistic lower bound). Trajectory timing only, never a
   safety decision.
4. **Rendering is CPU-bound:** 640×480 ≈ 300 ms/frame, 1920×1080 ≈ 1.8 s.
   Cameras are **waypoint-triggered, not free-running** — this is a hardware
   consequence, recorded rather than hidden.
5. **The GNSS datum is arbitrary.** No real site is claimed. Signal
   degradation is computed by casting rays at the sky, not looked up.
6. **The arm was validated with the airframe held.** Whether the aircraft
   holds station against the reaction of a moving 6-DOF arm is a Phase 2
   question.

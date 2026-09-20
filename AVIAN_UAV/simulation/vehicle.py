"""AVIAN vehicle: coaxial-X8 propulsion, control allocation, energy.

THE AIRCRAFT IS AN X8, NOT A HEXACOPTER.
Four arms, eight rotors, upper and lower counter-rotating on every arm. This
is the approved CAD configuration and the mixer below is derived from it
rather than adapted from a generic quadrotor.

WHY X8 GIVES CLEAN YAW AUTHORITY
--------------------------------
Handedness alternates around the airframe:

    arm 1   upper CCW   lower CW
    arm 2   upper CW    lower CCW
    arm 3   upper CCW   lower CW
    arm 4   upper CW    lower CCW

The four CCW rotors therefore sit at all four arm azimuths, and so do the four
CW rotors. Raising every CCW rotor and lowering every CW rotor produces a pure
yaw moment whose roll and pitch contributions cancel by symmetry. Yaw is
decoupled from roll and pitch by construction -- that is the reason to build an
X8 rather than a flat octocopter, and it is why the allocation matrix below has
a clean null structure instead of needing a weighted least-squares fudge.

CONTROL ALLOCATION
------------------
Eight rotor thrusts f (N) map to four body wrenches:

    [ T  ]   [  1      1     ...   1   ]  [ f1 ]
    [ tx ] = [  y1     y2    ...   y8  ]  [ .. ]
    [ ty ]   [ -x1    -x2    ...  -x8  ]  [ .. ]
    [ tz ]   [ s1*km  s2*km  ...  s8*km]  [ f8 ]

with (x_i, y_i) the rotor station in the body frame, s_i = +1 for CCW and -1
for CW, and km the torque-to-thrust ratio. The mixer is the Moore-Penrose
pseudo-inverse of that matrix, which for this geometry is the minimum-norm
allocation -- it spreads a demand across all eight rotors rather than
saturating two of them. The matrix is printed by `allocation_report()` and
asserted in the tests; nothing here is hand-tuned.

MOTOR MODEL
-----------
First order: thrust follows its command with time constant TAU_MOTOR. 0.12 s
is used, which is slow, and deliberately so -- a 28-inch carbon propeller on a
U8 II has real rotational inertia and a 20 ms time constant borrowed from a
5-inch racing quad would make the controller look far better than the aircraft
can actually be. Commands are saturated at the vendor thrust figure from the
CAD, not at an idealised maximum.

ENERGY
------
Momentum theory with the CAD's own coefficients (figure of merit, coaxial
interference factor, drive efficiency). Reported as an estimate, because that
is what it is -- there is no blade-element model here and no measured motor
map.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

CAD_DIR = os.environ.get("AVIAN_CAD_DIR", "/home/claude/avian")
if CAD_DIR not in sys.path:
    sys.path.insert(0, CAD_DIR)

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.join(os.path.dirname(HERE), "description")

G = 9.80665

# Torque-to-thrust ratio, metres. For a large low-pitch carbon propeller this
# is 0.015-0.025 m; 0.020 is used. It sets yaw authority and nothing else, and
# it is an ASSUMPTION -- no motor map was measured.
KM_TORQUE_THRUST = 0.020
TAU_MOTOR = 0.12                 # s, first-order thrust lag
ROTOR_INERTIA_NOTE = "28 in CF propeller; slow on purpose"


class AvianVehicle:
    """One AVIAN aircraft in a PyBullet world.

    Namespaced by `vid` so six of these can coexist without touching the
    core: every log line, every frame name and every topic-equivalent is
    prefixed with the vehicle id from the start.
    """

    def __init__(self, pb, vid="AVIAN_01", urdf=None, position=(0, 0, 2.0),
                 yaw=0.0, config="NOMINAL", fixed=False):
        self.pb = pb
        self.vid = vid
        self.urdf = urdf or os.path.join(DESC, "avian.urdf")
        man_path = os.path.join(DESC, "avian_description_manifest.json")
        self.man = json.load(open(man_path))

        q = pb.getQuaternionFromEuler([0, 0, yaw])
        self.body = pb.loadURDF(self.urdf, position, q, useFixedBase=fixed,
                                flags=pb.URDF_USE_INERTIA_FROM_FILE)

        self.n_j = pb.getNumJoints(self.body)
        self.joint_of = {}
        self.link_of = {}
        for i in range(self.n_j):
            info = pb.getJointInfo(self.body, i)
            self.joint_of[info[1].decode()] = i
            self.link_of[info[12].decode()] = i

        # ---- rotors -----------------------------------------------------
        self.rotors = self.man["rotors"]
        self.n_rotors = len(self.rotors)
        self.rotor_pos = np.array([r["position_m"] for r in self.rotors])
        self.rotor_sign = np.array([1.0 if r["spin"] == "CCW" else -1.0
                                    for r in self.rotors])
        self.f_max = np.array([r["max_thrust_N"] for r in self.rotors])
        self.rotor_link = [self.link_of[r["name"]] for r in self.rotors]

        # Free-spinning joints: the rotor links must not fight the solver.
        for li in self.rotor_link:
            pb.setJointMotorControl2(self.body, li, pb.VELOCITY_CONTROL,
                                     force=0.0)

        self.thrust = np.zeros(self.n_rotors)     # actual, after lag
        self.cmd = np.zeros(self.n_rotors)        # commanded

        # ---- allocation --------------------------------------------------
        self.A = self._allocation_matrix()
        self.A_pinv = np.linalg.pinv(self.A)
        self.authority = self._measure_authority()

        # ---- manipulator -------------------------------------------------
        self.arm_joints = [self.joint_of[j["name"]]
                           for j in self.man["joints"]]
        self.arm_names = [j["name"] for j in self.man["joints"]]
        for ji, spec in zip(self.arm_joints, self.man["joints"]):
            pb.setJointMotorControl2(self.body, ji, pb.POSITION_CONTROL,
                                     targetPosition=0.0,
                                     force=spec["effort_Nm"])

        # ---- mass ---------------------------------------------------------
        self.mass = pb.getDynamicsInfo(self.body, -1)[0]
        for i in range(self.n_j):
            self.mass += pb.getDynamicsInfo(self.body, i)[0]
        self.weight = self.mass * G
        self.hover_thrust_total = self.weight
        self.hover_thrust_per_rotor = self.weight / self.n_rotors

        # ---- energy -------------------------------------------------------
        self.energy_J = 0.0
        self.battery_Wh = 1300.0        # from the CAD battery group
        self.battery_used_Wh = 0.0

        # ---- health -------------------------------------------------------
        self.motor_failed = [False] * self.n_rotors
        self.armed = False
        self.sensors = {}
        self.arm = None
        self.saturation_events = 0
        self.max_saturation = 0.0       # cumulative, for reporting
        self.saturation_now = 0.0       # instantaneous, for decisions

    # -----------------------------------------------------------------
    def attach_sensors(self, seed=0, rates=None):
        """Instantiate the sensor suite from the CAD-derived frames.

        Called explicitly rather than in __init__ so a pure-dynamics test
        pays nothing for sensors it does not use -- rendering is by far the
        most expensive thing in this simulator.
        """
        from sensors.cameras import RGBCamera, DepthCamera
        from sensors.inertial import Lidar, IMU, GNSS
        r = {"rgb": 2.0, "depth": 2.0, "lidar": 5.0, "imu": 200.0,
             "gnss": 5.0}
        r.update(rates or {})
        self.sensors = {
            "rgb_front": RGBCamera(self, "rgb_front", "sensor_rgb_front",
                                   r["rgb"], seed=seed),
            "rgb_down": RGBCamera(self, "rgb_down", "sensor_rgb_down",
                                  r["rgb"], hfov_deg=82.0, vfov_deg=52.0,
                                  seed=seed),
            "depth": DepthCamera(self, "depth", "sensor_depth", r["depth"],
                                 seed=seed),
            "lidar": Lidar(self, "lidar", "sensor_lidar", r["lidar"],
                           seed=seed),
            "imu": IMU(self, "imu", "sensor_imu", r["imu"], seed=seed),
            "gnss": GNSS(self, "gnss", "sensor_imu", r["gnss"], seed=seed),
        }
        return self.sensors

    def attach_manipulator(self):
        from manipulator.arm import Manipulator
        self.arm = Manipulator(self)
        return self.arm

    # -----------------------------------------------------------------
    def _allocation_matrix(self):
        """[T, tx, ty, tz] = A @ f. Derived from rotor geometry, not typed."""
        A = np.zeros((4, self.n_rotors))
        for i, r in enumerate(self.rotors):
            x, y, _ = r["position_m"]
            A[0, i] = 1.0
            A[1, i] = y
            A[2, i] = -x
            # NEGATIVE sign. A rotor spinning CCW has its motor driving the
            # propeller CCW, so the REACTION on the airframe is CW, i.e.
            # negative about +Z. The first version of this matrix used
            # +sign here while step_actuators() correctly applied -sign,
            # so the allocator and the physics disagreed and the yaw loop
            # became positive feedback: yaw ran away to 162 deg in 1.6 s and
            # took roll and pitch with it through allocator clipping.
            A[3, i] = -self.rotor_sign[i] * KM_TORQUE_THRUST
        return A

    def _measure_authority(self):
        """Largest moment the aircraft can hold about each axis AT HOVER.

        Measured by bisection against the real per-rotor limits, not assumed.
        The controller clamps its torque demand to these, because a demand
        the aircraft cannot meet is turned into a clipped allocation whose
        roll and pitch are wrong -- which is how a yaw problem becomes a
        tumble.
        """
        out = {}
        w = float(self.f_max.sum()) * 0.5      # nominal hover collective
        for k, nm in ((1, "roll"), (2, "pitch"), (3, "yaw")):
            lo, hi = 0.0, 500.0
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                d = np.zeros(4)
                d[0] = w
                d[k] = mid
                f = self.A_pinv @ d
                if f.max() <= self.f_max.min() and f.min() >= 0.0:
                    lo = mid
                else:
                    hi = mid
            out[nm] = lo
        return out

    def allocation_report(self):
        lines = ["control allocation A ([T, tx, ty, tz] = A @ f):"]
        for k, nm in enumerate(("T ", "tx", "ty", "tz")):
            lines.append("  " + nm + " " + " ".join(
                f"{v:+7.3f}" for v in self.A[k]))
        lines.append("  rotors: " + ", ".join(
            f"{r['name'][6:]}{'+' if s > 0 else '-'}"
            for r, s in zip(self.rotors, self.rotor_sign)))
        cond = np.linalg.cond(self.A)
        lines.append(f"  condition number {cond:.2f}")
        return "\n".join(lines)

    # -----------------------------------------------------------------
    def set_wrench(self, T, tx, ty, tz):
        """Demand a body wrench; returns the per-rotor commands after limits.

        Saturation is recorded rather than silently clipped: a controller that
        is quietly asking for more than the aircraft has is a controller whose
        test results mean nothing.
        """
        f = self.A_pinv @ np.array([T, tx, ty, tz])
        for i in range(self.n_rotors):
            if self.motor_failed[i]:
                f[i] = 0.0
        over = float(np.max(f / self.f_max)) if self.f_max.max() > 0 else 0.0
        # INSTANTANEOUS demand, for live decisions. `max_saturation` below is
        # a cumulative high-water mark for reporting only -- using it as a
        # trigger meant that once the aircraft had ever saturated, the safety
        # supervisor latched into HOLD forever and every later emergency test
        # passed for the wrong reason.
        self.saturation_now = over
        if over > 1.0 or f.min() < 0.0:
            self.saturation_events += 1
            self.max_saturation = max(self.max_saturation, over)
        self.cmd = np.clip(f, 0.0, self.f_max)
        return self.cmd

    def step_actuators(self, dt):
        """First-order motor lag, then apply forces and reaction torques."""
        alpha = dt / max(dt, TAU_MOTOR)
        self.thrust += alpha * (self.cmd - self.thrust)

        pb = self.pb
        pos, orn = pb.getBasePositionAndOrientation(self.body)
        R = np.array(pb.getMatrixFromQuaternion(orn)).reshape(3, 3)

        tz_total = 0.0
        for i in range(self.n_rotors):
            f = float(self.thrust[i])
            if f <= 0.0:
                continue
            # Thrust acts along body +Z at the rotor station.
            pb.applyExternalForce(self.body, -1, [0, 0, f],
                                  list(self.rotor_pos[i]), pb.LINK_FRAME)
            tz_total += -self.rotor_sign[i] * KM_TORQUE_THRUST * f

        # Reaction torque about body Z. A CCW rotor drags the airframe CW,
        # hence the sign inversion above.
        if abs(tz_total) > 0.0:
            pb.applyExternalTorque(self.body, -1,
                                   list(R @ np.array([0, 0, tz_total])),
                                   pb.WORLD_FRAME)

        # ---- energy, momentum theory with the CAD's coefficients --------
        rho, FM, k_coax, eta = 1.225, 0.72, 0.82, 0.82
        prop_r = 0.5 * 28.0 * 0.0254
        A_disc = math.pi * prop_r * prop_r
        P = 0.0
        for f in self.thrust:
            if f > 0:
                P += (f ** 1.5) / math.sqrt(2.0 * rho * A_disc) \
                    / (FM * k_coax) / eta
        self.energy_J += P * dt
        self.battery_used_Wh = self.energy_J / 3600.0
        return P

    # -----------------------------------------------------------------
    def state(self):
        pb = self.pb
        pos, orn = pb.getBasePositionAndOrientation(self.body)
        lin, ang = pb.getBaseVelocity(self.body)
        rpy = pb.getEulerFromQuaternion(orn)
        return {
            "vid": self.vid,
            "position": np.array(pos), "quaternion": np.array(orn),
            "rpy": np.array(rpy), "velocity": np.array(lin),
            "angular_velocity": np.array(ang),
            "thrust": self.thrust.copy(),
            "battery_used_Wh": self.battery_used_Wh,
            "battery_pct": max(0.0, 100.0 * (1.0 - self.battery_used_Wh
                                             / self.battery_Wh)),
            "armed": self.armed,
        }

    def set_arm_joints(self, q_rad):
        for ji, spec, q in zip(self.arm_joints, self.man["joints"], q_rad):
            self.pb.setJointMotorControl2(
                self.body, ji, self.pb.POSITION_CONTROL, targetPosition=q,
                force=spec["effort_Nm"],
                maxVelocity=math.radians(spec["velocity_deg_s"]))

    def fail_motor(self, index):
        """Inject a rotor failure. The rotor produces no thrust from now on."""
        self.motor_failed[index] = True

    def set_battery_pct(self, pct):
        """Inject a battery state.

        battery_used_Wh is DERIVED from integrated energy and is recomputed
        every step, so assigning to it does nothing -- a failure-injection
        test that poked it saw the value silently overwritten on the next
        tick and the low-battery emergency never fired. Energy is the state;
        this sets it.
        """
        pct = float(max(0.0, min(100.0, pct)))
        self.energy_J = (1.0 - pct / 100.0) * self.battery_Wh * 3600.0
        self.battery_used_Wh = self.energy_J / 3600.0
        return pct

    def battery_pct(self):
        return max(0.0, 100.0 * (1.0 - self.battery_used_Wh
                                 / self.battery_Wh))

    def tool_pose(self):
        st = self.pb.getLinkState(self.body, self.link_of["nozzle_tip"],
                                  computeForwardKinematics=True)
        return np.array(st[4]), np.array(st[5])

"""Cascaded flight controller for the AVIAN X8.

Built in the order the revision asks for, because that is also the order in
which it can be tested: motors, then rates, then attitude, then altitude, then
position. Each loop closes around the one inside it and none of them is
allowed to be clever.

    position error -> velocity setpoint      (P)
    velocity error -> acceleration setpoint  (PID)
    acceleration   -> thrust vector          (algebra, not a gain)
    thrust vector  -> tilt + collective      (algebra)
    attitude error -> rate setpoint          (P)
    rate error     -> body torque            (PID)
    wrench         -> 8 rotor thrusts        (allocation pseudo-inverse)

WHY A THRUST VECTOR AND NOT ROLL/PITCH GAINS
--------------------------------------------
The desired acceleration plus gravity IS the required thrust direction. Tilt
angles fall out of that vector by trigonometry. Tuning separate roll and pitch
gains against a position error works, but it hides the fact that a multirotor
cannot accelerate without tilting, and it breaks the moment the aircraft is
asked to hold position in wind. This form stays correct at large tilt.

GAIN CHOICE
-----------
Tuned for THIS aircraft: 23.7 kg, 536 N of thrust, T/W 2.3, and a 0.12 s motor
lag. The motor lag is the binding constraint -- rate gains that would suit a
fast actuator oscillate here, so the rate loop is deliberately slower than a
small multirotor's. The gains are in one dict, `GAINS`, so a tuning study
changes one place.

Every limit below is a real aircraft limit, not a number chosen to make a test
pass: tilt is capped at 25 deg because beyond that the horizontal component of
a 2.3 T/W aircraft's thrust exceeds what the position loop can recover from
inside the corridor widths in this environment.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.80665

GAINS = {
    # position -> velocity
    "kp_xy": 0.9, "kp_z": 1.6,
    # velocity -> acceleration
    "kv_xy": 2.6, "kvi_xy": 0.28, "kvd_xy": 0.10,
    "kv_z": 5.0, "kvi_z": 1.6, "kvd_z": 0.15,
    # attitude -> rate
    "kr_rp": 6.5, "kr_yaw": 3.0,
    # rate -> torque
    "kw_rp": 14.0, "kwi_rp": 1.2, "kwd_rp": 0.6,
    # Yaw is the weak axis on an X8: authority comes only from the
    # torque-to-thrust ratio, ~4.7 Nm here against ~94 Nm in roll and pitch.
    # Yaw gains are scaled to that reality rather than to the other axes.
    "kw_yaw": 2.2, "kwi_yaw": 0.15, "kwd_yaw": 0.05,
}

LIMITS = {
    "v_xy_max": 4.0,          # m/s
    "v_z_max": 2.0,
    "a_xy_max": 3.5,          # m/s^2
    "tilt_max_rad": math.radians(25.0),
    "rate_max_rad": math.radians(120.0),
    "i_max": 4.0,
    # Fraction of the aircraft's MEASURED authority the controller may ask
    # for. Demanding more does not produce more moment -- it produces a
    # clipped allocation whose other axes are wrong.
    "torque_margin": 0.85,
}


class Cascade:
    """One controller per vehicle. Stateless between vehicles by design."""

    def __init__(self, vehicle, gains=None, limits=None):
        self.v = vehicle
        self.g = dict(GAINS)
        if gains:
            self.g.update(gains)
        self.lim = dict(LIMITS)
        if limits:
            self.lim.update(limits)
        self.i_vel = np.zeros(3)
        self.i_rate = np.zeros(3)
        self.prev_ev = np.zeros(3)
        self.prev_ew = np.zeros(3)
        self.target = np.array([0.0, 0.0, 2.0])
        self.target_yaw = 0.0
        self.mode = "IDLE"

    # -----------------------------------------------------------------
    def set_target(self, xyz, yaw=None):
        self.target = np.asarray(xyz, dtype=float)
        if yaw is not None:
            self.target_yaw = float(yaw)

    def reset_integrators(self):
        self.i_vel[:] = 0.0
        self.i_rate[:] = 0.0

    # -----------------------------------------------------------------
    def update(self, dt):
        v = self.v
        st = v.state()
        p, vel = st["position"], st["velocity"]
        rpy, w = st["rpy"], st["angular_velocity"]
        g = self.g
        lim = self.lim

        # ---- 1. position -> velocity setpoint --------------------------
        ep = self.target - p
        v_sp = np.array([g["kp_xy"] * ep[0], g["kp_xy"] * ep[1],
                         g["kp_z"] * ep[2]])
        v_sp[:2] = _clip_norm(v_sp[:2], lim["v_xy_max"])
        v_sp[2] = float(np.clip(v_sp[2], -lim["v_z_max"], lim["v_z_max"]))

        # ---- 2. velocity -> acceleration setpoint -----------------------
        ev = v_sp - vel
        self.i_vel += ev * dt
        self.i_vel = np.clip(self.i_vel, -lim["i_max"], lim["i_max"])
        dev = (ev - self.prev_ev) / max(dt, 1e-6)
        self.prev_ev = ev.copy()
        a_sp = np.array([
            g["kv_xy"] * ev[0] + g["kvi_xy"] * self.i_vel[0]
            + g["kvd_xy"] * dev[0],
            g["kv_xy"] * ev[1] + g["kvi_xy"] * self.i_vel[1]
            + g["kvd_xy"] * dev[1],
            g["kv_z"] * ev[2] + g["kvi_z"] * self.i_vel[2]
            + g["kvd_z"] * dev[2]])
        a_sp[:2] = _clip_norm(a_sp[:2], lim["a_xy_max"])

        # ---- 3. acceleration -> thrust vector ---------------------------
        # The vector the rotors must produce, in world coordinates.
        f_world = v.mass * (a_sp + np.array([0.0, 0.0, G]))
        f_norm = float(np.linalg.norm(f_world))
        if f_norm < 1e-6:
            f_world = np.array([0.0, 0.0, v.weight])
            f_norm = v.weight
        b3_des = f_world / f_norm

        # Cap tilt: clamp the desired body-z away from horizontal.
        cos_tilt = float(np.clip(b3_des[2], -1.0, 1.0))
        tilt = math.acos(cos_tilt)
        if tilt > lim["tilt_max_rad"]:
            horiz = b3_des[:2]
            hn = np.linalg.norm(horiz)
            if hn > 1e-9:
                horiz = horiz / hn * math.sin(lim["tilt_max_rad"])
            b3_des = np.array([horiz[0], horiz[1],
                               math.cos(lim["tilt_max_rad"])])
            b3_des /= np.linalg.norm(b3_des)

        # Collective is the projection of the required force on the CURRENT
        # body z -- not the desired one. Using the desired axis over-commands
        # thrust during a large attitude error and makes the aircraft balloon.
        R = _rot(rpy)
        b3 = R[:, 2]
        T = float(np.dot(f_world, b3))
        T = float(np.clip(T, 0.0, float(v.f_max.sum())))

        # ---- 4. desired attitude ----------------------------------------
        yaw_des = self.target_yaw
        roll_des = math.asin(float(np.clip(
            b3_des[0] * math.sin(yaw_des) - b3_des[1] * math.cos(yaw_des),
            -1.0, 1.0)))
        pitch_des = math.atan2(
            b3_des[0] * math.cos(yaw_des) + b3_des[1] * math.sin(yaw_des),
            max(1e-6, b3_des[2]))

        # ---- 5. attitude -> rate setpoint --------------------------------
        e_att = np.array([_wrap(roll_des - rpy[0]),
                          _wrap(pitch_des - rpy[1]),
                          _wrap(yaw_des - rpy[2])])
        w_sp = np.array([g["kr_rp"] * e_att[0], g["kr_rp"] * e_att[1],
                         g["kr_yaw"] * e_att[2]])
        w_sp = np.clip(w_sp, -lim["rate_max_rad"], lim["rate_max_rad"])

        # ---- 6. rate -> torque -------------------------------------------
        # Body rates. PyBullet reports angular velocity in WORLD frame.
        w_body = R.T @ np.asarray(w)
        ew = w_sp - w_body
        self.i_rate += ew * dt
        self.i_rate = np.clip(self.i_rate, -lim["i_max"], lim["i_max"])
        dew = (ew - self.prev_ew) / max(dt, 1e-6)
        self.prev_ew = ew.copy()
        tau = np.array([
            g["kw_rp"] * ew[0] + g["kwi_rp"] * self.i_rate[0]
            + g["kwd_rp"] * dew[0],
            g["kw_rp"] * ew[1] + g["kwi_rp"] * self.i_rate[1]
            + g["kwd_rp"] * dew[1],
            g["kw_yaw"] * ew[2] + g["kwi_yaw"] * self.i_rate[2]
            + g["kwd_yaw"] * dew[2]])

        # ---- 7. clamp to real authority, then allocate --------------------
        auth = v.authority
        m = lim["torque_margin"]
        tau[0] = float(np.clip(tau[0], -m * auth["roll"], m * auth["roll"]))
        tau[1] = float(np.clip(tau[1], -m * auth["pitch"], m * auth["pitch"]))
        tau[2] = float(np.clip(tau[2], -m * auth["yaw"], m * auth["yaw"]))
        v.set_wrench(T, tau[0], tau[1], tau[2])
        return {"T": T, "tau": tau, "e_pos": ep, "e_att": e_att,
                "tilt_deg": math.degrees(tilt), "v_sp": v_sp}


# ---------------------------------------------------------------------------
def _clip_norm(v, m):
    n = float(np.linalg.norm(v))
    return v if n <= m or n < 1e-12 else v * (m / n)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _rot(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])

#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 1: single-pass mission follower.

Flies mission/gazebo_mission_plan.json's inspection waypoints IN ORDER,
each exactly once, with direct PX4 OFFBOARD velocity setpoints (no Nav2).

Why direct velocity control instead of Nav2 goals:
  * The waypoints are 3D (z 3.5-29 m). Nav2 plans and controls in 2D; the
    GarudaNEX cmd_vel bridge holds one fixed cruise altitude. Nav2 simply
    cannot command most of this mission.
  * The route is already known and planned offline, so a
    global planner adds nothing; the previous MPPI stack also ran its
    control loop at 6.9-12.5 Hz on this machine and reached 0/4 goals.
  * Dropping SLAM + Nav2 frees that CPU for Gazebo rendering + the camera.

Inputs (everything this node reads):
  * the mission plan JSON (waypoints, pass-through route points, headings)
  * PX4's own EKF estimate: /fmu/out/vehicle_local_position(_v1),
    /fmu/out/vehicle_attitude, /fmu/out/vehicle_status(_v1),
    /fmu/out/vehicle_land_detected
  * onboard range sensors: the 360 x 15-channel LiDAR point cloud and the
    up/down range cones (/mission/lidar_points, /mission/up_points,
    /mission/down_points)
It never reads scene geometry or any simulator pose. The safety layer below
acts only on sensed returns.

Sensed safety layer (runs every control tick, underneath the mission):
  * every sensed return within SENSED_CLEARANCE_M (3.0 m) of the airframe
    removes the commanded velocity component toward it and adds a gentle
    push away;
  * along the commanded direction, the free travel before any sensed
    return would come within 3.0 m caps speed so that twice the stopping
    distance (v^2/2a + v*t_lat) always fits inside that free travel;
  * the LiDAR sees +/-14 deg of elevation and the cones +/-45 deg around
    vertical, so motion is only commanded near-horizontal (<=12 deg) or
    near-vertical (>=50 deg): steeper-than-12-deg route pieces are flown
    as vertical-then-horizontal steps, never through the unsensed band.
"""
import json
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from px4_msgs.msg import (VehicleCommand, VehicleStatus, VehicleLocalPosition,
                          VehicleAttitude, VehicleLandDetected,
                          OffboardControlMode, TrajectorySetpoint)

PX4_MODE_OFFBOARD = 6.0
NAN = float('nan')

# ---- speed / safety numbers (see mission_speed_calc in the results JSON) --
SENSED_CLEARANCE_M = 3.0
# Stopping model, from FOUR brake tests of this vehicle under offboard
# velocity control (distance timed from the zero-velocity command, so PX4's
# response lag is inside it):
#   2.451 m/s -> 1.371 m | 2.256 -> 1.307 | 2.050 -> 1.186 | 2.051 -> 1.214
# distance/speed is ~constant (0.559-0.592 s): the stop is dominated by
# the velocity loop's response lag, not by a constant deceleration, so a
# pure v^2/2a model under-predicts. The -3.0 m/s2 figure used before was
# Nav2's accel LIMIT, not the vehicle's achieved stop. Use the worse of:
DECEL_MPS2 = 1.95         # worst effective constant decel measured
STOP_K_S = 0.592          # worst measured stop distance / speed
T_LAT_S = 0.15            # sensing latency on top: LiDAR 10 Hz frame age
                          # (<=0.10 s) + 20 Hz control tick (<=0.05 s)
SAFETY_FACTOR = 2.0
SPEED_OVERSHOOT = 1.025   # PX4 overshot 2.2 -> 2.256 m/s in a brake test
CRUISE_MPS = 1.9          # 1.95 with overshoot -> stop 1.447 m, margin
                          # 2.07x; v_max at 2.0x is 2.02 m/s
VZ_UP_MAX = 1.5
VZ_DN_MAX = 1.0
A_PLAN = 1.5              # planned (comfortable) decel into each waypoint
LOOKAHEAD_M = 3.0
HORIZ_ELEV_MAX = math.radians(12.0)
SETTLE_TOL_M = 0.30
SETTLE_YAW_TOL = math.radians(10.0)
SETTLE_SPEED = 0.30
SETTLE_HOLD_S = 1.0
BLOCKED_GIVEUP_S = 20.0
# airframe bounding box (body FLU, m): returns inside it are the aircraft
# itself (rotor tips reach ~0.38 m, legs to -0.24 m), not obstacles
SELF_BOX = np.array([[-0.55, -0.55, -0.45], [0.55, 0.55, 0.60]])
YAW_RATE_MAX = math.radians(60.0)
TICK_HZ = 20.0
GIMBAL_SETTLE_S = 1.0     # time allowed for the pitch gimbal after a new command
GIMBAL_LIMIT_RAD = 1.5708  # gimbal_pitch_joint travel, +/-90 deg (airframe model's joint limit)
GIMBAL_PIVOT_BODY = np.array([0.35, 0.0, 0.05])   # gimbal boom pivot, body FLU (m)
AIM_REFINE_DIST_M = 3.0    # re-aim once from the EKF pose, close to the waypoint AND nearly stationary
AIM_REFINE_MIN_RAD = math.radians(0.5)   # ...but only if that changes the command


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_rotate(q, v):
    """Rotate (N,3) vectors by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return v @ R.T


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in 'xyz') or msg.width * msg.height == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in 'xyz']
    p = np.stack(cols, 1).astype(float)
    return p[np.isfinite(p).all(1)]


def stop_dist(v):
    return max(v * v / (2 * DECEL_MPS2), STOP_K_S * v) + v * T_LAT_S


def v_allowed(free_m):
    """Largest v with SAFETY_FACTOR * stop_dist(v) <= free_m."""
    if free_m <= 0:
        return 0.0
    if free_m == math.inf:
        return math.inf
    lo, hi = 0.0, 20.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if SAFETY_FACTOR * stop_dist(mid) <= free_m:
            lo = mid
        else:
            hi = mid
    return lo


class MissionFollower(Node):
    def __init__(self):
        super().__init__('sih_mission_follower')
        self.declare_parameter('plan', '')
        self.declare_parameter('results_dir', '/tmp/sih_mission')
        self.declare_parameter('home_world_z', 0.24)
        self.declare_parameter('brake_test', True)
        self.declare_parameter('max_waypoints', 0)   # 0 = all
        self.declare_parameter('origin_lat_deg', 47.397971057728974)
        self.declare_parameter('geodesy_fix', True)
        # Plans that carry no gimbal_pitch_rad but do carry a per-waypoint
        # target_m (the 150-waypoint plan) get their pitch computed from that
        # target; false reproduces the old fixed-camera behaviour.
        self.declare_parameter('aim_from_target', True)
        # Continuous column aim: on orbit legs (both ends on the same column in the plan) yaw tracks the
        # column axis (the plan's target_m) on every tick instead of turning to the NEXT viewpoint's heading.
        # Aiming only: velocity setpoints are unchanged, and the LiDAR ring and range cones sense 360 deg /
        # vertically whatever the heading, so this never changes what the safety layer sees or does.
        self.declare_parameter('continuous_aim', True)
        # Demo looping (default OFF = the unchanged single pass). The plan's
        # waypoints are flown forward then backward (ping-pong) using the plan's
        # own route pieces reversed, so nothing new is ever flown. 0 = off,
        # N > 0 = N laps (a lap is forward + back), -1 = until interrupted.
        # demo_wall_minutes > 0 also ends the looping after that much wall clock.
        self.declare_parameter('demo_laps', 0)
        self.declare_parameter('demo_wall_minutes', 0.0)
        self.plan_path = self.get_parameter('plan').value
        self.results_dir = self.get_parameter('results_dir').value
        os.makedirs(self.results_dir, exist_ok=True)
        self.plan = json.load(open(self.plan_path))
        hg = self.plan['home_ground_world_m']
        # EKF local origin = where the vehicle booted, i.e. resting on the
        # home pad. home_world_z = height of the body origin above the
        # terrain datum when resting (airframe geometry: base_link sits
        # 0.24 m above the model origin at the feet).
        self.home = np.array([hg[0], hg[1], self.get_parameter('home_world_z').value])
        self.do_brake = self.get_parameter('brake_test').value
        self.aim_from_target = self.get_parameter('aim_from_target').value
        self.continuous_aim = self.get_parameter('continuous_aim').value
        self.last_reached_wp = None
        # Geodesy correction (found in full_pass_03's audit): Gazebo's GNSS
        # turns world metres into lat/lon on the WGS84 ellipsoid, PX4's local
        # projection turns lat/lon back into metres on a sphere of radius
        # 6,371,000 m (src/lib/geo/geo.h). At the world origin latitude the
        # ellipsoid's radii are N (east) = 6,389,735 m and M (north) =
        # 6,370,065 m, so PX4-local east = world east * R/N (0.99707) and
        # north = world north * R/M (1.00015). Without this, a waypoint
        # 325 m east of home lands 0.96 m long. Only constants and the
        # world's declared origin latitude are used here.
        lat = math.radians(self.get_parameter('origin_lat_deg').value)
        a_, f_ = 6378137.0, 1 / 298.257223563
        e2 = f_ * (2 - f_)
        s_ = 1 - e2 * math.sin(lat) ** 2
        N_, M_ = a_ / math.sqrt(s_), a_ * (1 - e2) / s_ ** 1.5
        R_ = 6371000.0
        on = self.get_parameter('geodesy_fix').value
        self.k_east = R_ / N_ if on else 1.0
        self.k_north = R_ / M_ if on else 1.0
        mx = self.get_parameter('max_waypoints').value
        self.wps = self.plan['waypoints'][:mx] if mx else self.plan['waypoints']
        self.demo_laps = int(self.get_parameter('demo_laps').value)
        self.demo_wall_s = 60.0 * float(self.get_parameter('demo_wall_minutes').value)
        self.demo_on = self.demo_laps != 0 or self.demo_wall_s > 0
        self.dir, self.lap = 1, 0
        self.interrupts = self.interrupts_handled = 0
        if self.demo_on and (len(self.wps) < 2 or any(not w.get('reachable_in_plan', True) for w in self.wps)):
            raise SystemExit('demo looping needs >= 2 waypoints, all reachable in the plan')

        be = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST)
        self.pos = self.status = self.att = self.land = None
        for suf in ('', '_v1'):
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position' + suf, self.on_pos, be)
            self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status' + suf, self.on_status, be)
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude' + suf, self.on_att, be)
            self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected' + suf, self.on_land, be)
        self.clouds = {}
        for name in ('lidar', 'up', 'down'):
            self.create_subscription(PointCloud2, f'/mission/{name}_points',
                                     lambda m, n=name: self.on_cloud(n, m), qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', be)
        self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', be)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', be)
        self.evt_pub = self.create_publisher(String, '/mission/event', 10)
        # pitch gimbal: joint angle = nose-down pitch = -(plan's gimbal_pitch_rad)
        self.gimbal_pub = self.create_publisher(Float64, '/mission/gimbal_cmd', 10)
        self.gimbal_cmd = None
        self.gimbal_t = -1e9
        self.gimbal_last_pub = -1e9

        self.phase = 'WAIT'
        self.phase_t0 = self.t()
        self.cmd_v = np.zeros(3)
        self.cmd_yaw = None
        self.log = {'waypoints': [], 'events': [], 'track': []}
        self.i = 0
        self.leg = None
        self.t_start = None
        self.sense_stats = {'ticks': 0, 'ticks_limited': 0, 'ticks_pushed': 0,
                            'min_range_m': math.inf, 'no_fresh_lidar_ticks': 0}
        self.last_track = 0.0
        self.dbg = None
        self.stale = False
        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.note(f'plan {self.plan_path}: {len(self.wps)} waypoints, home {self.home.tolist()}, '
                  f'geodesy k_east={self.k_east:.6f} k_north={self.k_north:.6f}')

    def t(self):
        """Sim time (s) -- node runs with use_sim_time, so every timing,
        timeout and freshness check here is in simulated seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- inputs ----------------
    def on_pos(self, m): self.pos = m
    def on_status(self, m): self.status = m
    def on_att(self, m): self.att = m
    def on_land(self, m): self.land = m

    def on_cloud(self, name, msg):
        p = cloud_xyz(msg)
        # sensor frame -> body FLU (mount poses from the airframe SDF)
        if name == 'lidar':
            # lidar link 0.26 above the model origin = ~0.02 above base_link
            # (base_link sits 0.24 up), sensor +0.055 on top: ~+0.08 m
            p = p + np.array([0.12, 0.0, 0.08])
        elif name == 'up':      # pitched -90 deg: sensor +x -> body +z
            p = np.stack([-p[:, 2], p[:, 1], p[:, 0]], 1) + np.array([0, 0, 0.42])
        else:                   # pitched +90 deg: sensor +x -> body -z
            p = np.stack([p[:, 2], p[:, 1], -p[:, 0]], 1) + np.array([0, 0, -0.12])
        inside = ((p > SELF_BOX[0]) & (p < SELF_BOX[1])).all(1)
        p = p[~inside]
        if len(p):
            k = 'min_' + name
            self.sense_stats[k] = min(self.sense_stats.get(k, math.inf), float(np.linalg.norm(p, axis=1).min()))
        self.clouds[name] = (self.t(), p)

    # ---------------- frames ----------------
    def ned_from_world(self, w):
        w = np.asarray(w, float)
        return np.array([(w[1] - self.home[1]) * self.k_north,
                         (w[0] - self.home[0]) * self.k_east, -(w[2] - self.home[2])])

    def world_from_ned(self, n):
        return np.array([n[1] / self.k_east + self.home[0], n[0] / self.k_north + self.home[1],
                         -n[2] + self.home[2]])

    def p_ned(self):
        return np.array([self.pos.x, self.pos.y, self.pos.z])

    def v_ned(self):
        return np.array([self.pos.vx, self.pos.vy, self.pos.vz])

    def sensed_points_ned(self, now):
        """All fresh sensed returns, relative to the vehicle, level NED frame."""
        if self.att is None:
            return np.zeros((0, 3)), False
        pts = []
        lidar_fresh = False
        for name, (t, p) in self.clouds.items():
            if now - t > 0.5:
                continue
            # a fresh scan with zero returns is a valid all-clear reading
            # (e.g. at 29 m altitude nothing is within the 30 m range);
            # freshness must not depend on point count
            lidar_fresh |= name == 'lidar'
            if len(p):
                pts.append(p)
        if not pts:
            return np.zeros((0, 3)), lidar_fresh
        flu = np.vstack(pts)
        frd = flu * np.array([1.0, -1.0, -1.0])
        return quat_rotate(self.att.q, frd), lidar_fresh

    # ---------------- safety layer ----------------
    def safety(self, v_des, allow_down_contact=False):
        now = self.t()
        P, fresh = self.sensed_points_ned(now)
        st = self.sense_stats
        st['ticks'] += 1
        if not fresh:
            st['no_fresh_lidar_ticks'] += 1
            self.stale = True
            return np.zeros(3), math.inf, 0.0   # no fresh LiDAR -> hold, never fly blind
        if allow_down_contact:
            P = P[P[:, 2] < 0.5]           # landing: ignore returns below
        if len(P) == 0:
            return v_des, math.inf, math.inf
        d = np.linalg.norm(P, axis=1)
        dmin = float(d.min())
        st['min_range_m'] = min(st['min_range_m'], dmin)
        v = v_des.copy()
        close = d < SENSED_CLEARANCE_M
        if close.any():
            st['ticks_pushed'] += 1
            for p, di in zip(P[close], d[close]):
                u = p / di
                if v @ u > 0:
                    v = v - (v @ u) * u
            u = P[close][np.argmin(d[close])] / d[close].min()
            v = v - 0.4 * (SENSED_CLEARANCE_M - d[close].min()) / SENSED_CLEARANCE_M * u
        speed = float(np.linalg.norm(v))
        free = math.inf
        if speed > 1e-3:
            u = v / speed
            along = P @ u
            lat2 = np.maximum(d * d - along * along, 0.0)
            # returns already inside the ring are handled above (no motion
            # toward them); free travel is judged against the rest
            m = (along > 0) & (lat2 < SENSED_CLEARANCE_M ** 2) & (d >= SENSED_CLEARANCE_M)
            if m.any():
                free = float((along[m] - np.sqrt(SENSED_CLEARANCE_M ** 2 - lat2[m])).min())
            va = v_allowed(free)
            if va < speed:
                st['ticks_limited'] += 1
                v = u * va
        return v, dmin, free

    # ---------------- outputs ----------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.param1, m.param2 = p1, p2
        m.command = command
        m.target_system = m.target_component = 1
        m.source_system, m.source_component = 255, 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def publish_setpoint(self, v_ned, yaw_ned):
        ocm = OffboardControlMode()
        ocm.timestamp = self.now_us()
        ocm.velocity = True
        self.ocm_pub.publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [NAN, NAN, NAN]
        sp.velocity = [float(v) for v in v_ned]
        sp.yaw = float(yaw_ned) if yaw_ned is not None else NAN
        self.sp_pub.publish(sp)

    def note(self, msg):
        self.get_logger().info(msg)
        self.log['events'].append({'t_sim': round(self.t(), 3), 't_wall': round(time.time(), 3), 'msg': msg})

    def event(self, d):
        self.evt_pub.publish(String(data=json.dumps(d)))

    def set_phase(self, p):
        self.note(f'phase {self.phase} -> {p}')
        self.phase, self.phase_t0 = p, self.t()

    # ---------------- guidance helpers ----------------
    def yaw_toward(self, target_yaw_ned):
        cur = self.pos.heading if self.cmd_yaw is None else self.cmd_yaw
        step = YAW_RATE_MAX / TICK_HZ
        self.cmd_yaw = wrap(cur + max(-step, min(step, wrap(target_yaw_ned - cur))))
        return self.cmd_yaw

    def set_gimbal(self, pitch_up):
        if pitch_up is None:
            return
        cmd = -float(pitch_up)
        if self.gimbal_cmd is None or abs(cmd - self.gimbal_cmd) > 1e-4:
            self.gimbal_cmd = cmd
            self.gimbal_t = self.t()
            self.gimbal_last_pub = -1e9

    def body_pitch_up(self):
        """Airframe pitch (rad, nose-up positive) from PX4's attitude estimate."""
        if self.att is None:
            return 0.0
        w, x, y, z = self.att.q
        return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))

    def target_pitch_up(self, body_world, yaw_enu, target_world, body_pitch_up=0.0):
        """Gimbal pitch (rad, up positive) that puts target_world on the camera
        axis, seen from the gimbal pivot on a body at body_world with heading
        yaw_enu. Clamped to the gimbal's real joint travel. Yaw is not
        commanded here: the airframe already turns to the waypoint heading."""
        c, s_ = math.cos(yaw_enu), math.sin(yaw_enu)
        pivot = np.asarray(body_world, float) + np.array(
            [GIMBAL_PIVOT_BODY[0] * c, GIMBAL_PIVOT_BODY[0] * s_, GIMBAL_PIVOT_BODY[2]])
        v = np.asarray(target_world, float) - pivot
        # signed distance along the heading: a target straight above or below the airframe is
        # slightly BEHIND the pivot (0.35 m forward of the body), i.e. needs a pitch past
        # vertical, which the joint limit then clamps to +/-90 deg
        along = v[0] * c + v[1] * s_
        elev = math.atan2(v[2], along)
        return max(-GIMBAL_LIMIT_RAD, min(GIMBAL_LIMIT_RAD, elev - body_pitch_up))

    def start_leg(self, name, route_world, final_world, yaw_enu, settle=True, gimbal_pitch=None):
        self.set_gimbal(gimbal_pitch)
        pts = [self.p_ned()] + [self.ned_from_world(p) for p in route_world] + [self.ned_from_world(final_world)]
        self.leg = {'name': name, 'pts': np.array(pts), 'seg': 0, 'settle': settle,
                    'final': self.ned_from_world(final_world), 'final_world': list(map(float, final_world)),
                    'yaw_ned': None if yaw_enu is None else wrap(math.pi / 2 - yaw_enu),
                    't0': self.t(), 'held_since': None, 'blocked_since': None,
                    'dist': 0.0, 'last_p': self.p_ned(), 'min_range': math.inf,
                    # inf, not 0: the post-corner ramp-up only applies after an
                    # actual sharp corner (reset to 0.0 in carrot() below) --
                    # a leg with no corner (TAKEOFF, BRAKE_*, a single-via
                    # waypoint) must fly at full planned speed from a standing
                    # start like it always did, not get capped at 0.3 m/s
                    # forever (found 25 Sep: that starved takeoff -- 0.3 m/s
                    # commanded near the pad never produced enough measured
                    # motion to grow the ramp, a self-reinforcing standstill)
                    'since_corner': math.inf, 'limited_ticks': 0, 'ticks': 0,
                    'timeout': self.route_len(pts) / (0.5 * CRUISE_MPS) + 45.0}

    @staticmethod
    def route_len(pts):
        pts = np.asarray(pts)
        return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())

    @staticmethod
    def _turn(pts, k):
        """Direction change (rad) at vertex k of the polyline."""
        if k <= 0 or k >= len(pts) - 1:
            return 0.0
        u, w = pts[k] - pts[k - 1], pts[k + 1] - pts[k]
        nu, nw = np.linalg.norm(u), np.linalg.norm(w)
        if nu < 1e-6 or nw < 1e-6:
            return 0.0
        return math.acos(max(-1.0, min(1.0, float(u @ w) / (nu * nw))))

    def carrot(self):
        """Returns (carrot point, remaining path length, distance to the next
        sharp corner). Planned segments are the clearance-checked ones, so the
        carrot never cuts across a vertex that turns more than 20 deg."""
        L = self.leg
        p = self.p_ned()
        pts = L['pts']
        while L['seg'] < len(pts) - 2:
            a, b = pts[L['seg']], pts[L['seg'] + 1]
            ab = b - a
            t = (p - a) @ ab / max(ab @ ab, 1e-9)
            sharp = self._turn(pts, L['seg'] + 1) > math.radians(20)
            if np.linalg.norm(p - b) < (0.5 if sharp else 1.0) or (t >= 1.0 and not sharp):
                L['seg'] += 1
                if sharp:
                    # leaving a sharp corner (e.g. the 90 deg vertex between a
                    # vertical-first descent and the flat leg after it): reset
                    # the ramp-up distance so guidance() re-applies the same
                    # slow-out it used slowing IN, instead of jumping straight
                    # to cruise speed while residual vertical momentum from
                    # the corner is still bleeding off (found 25 Sep: this gap
                    # let a leg sag ~3.6 m toward the ground before the sensed
                    # safety layer caught it -- no contact, but avoidable)
                    L['since_corner'] = 0.0
            else:
                break
        a, b = pts[L['seg']], pts[L['seg'] + 1]
        ab = b - a
        t = min(1.0, max(0.0, (p - a) @ ab / max(ab @ ab, 1e-9)))
        proj = a + t * ab
        rem = np.linalg.norm(b - proj) + self.route_len(pts[L['seg'] + 1:])
        need = LOOKAHEAD_M
        cur, k = proj, L['seg'] + 1
        to_corner = np.linalg.norm(b - proj)
        corner_found = False
        while k < len(pts):
            seg = np.linalg.norm(pts[k] - cur)
            if seg >= need:
                cur = cur + (pts[k] - cur) * need / max(seg, 1e-9)
                break
            need -= seg
            cur = pts[k]
            if self._turn(pts, k) > math.radians(20):
                corner_found = True
                break              # stop the carrot AT the corner
            k += 1
            if k < len(pts):
                to_corner += np.linalg.norm(pts[k] - pts[k - 1])
        if not corner_found:
            # distance to the next sharp vertex beyond the lookahead
            to_corner = np.linalg.norm(b - proj)
            for j in range(L['seg'] + 1, len(pts) - 1):
                if self._turn(pts, j) > math.radians(20):
                    break
                to_corner += np.linalg.norm(pts[j + 1] - pts[j])
        return cur, rem, to_corner

    def guidance(self):
        """Desired NED velocity toward the leg's carrot (before safety)."""
        L = self.leg
        p = self.p_ned()
        c, rem, to_corner = self.carrot()
        err_final = L['final'] - p
        if rem < 2.0:
            v = 1.0 * err_final
            spd = min(1.0, float(np.linalg.norm(v)))
            v = v / max(np.linalg.norm(v), 1e-9) * spd
        else:
            d = c - p
            spd = min(CRUISE_MPS, math.sqrt(2 * A_PLAN * rem), max(0.4, rem),
                      0.3 + 1.0 * to_corner,      # arrive at a sharp corner slowly:
                      0.3 + 1.0 * L['since_corner'])  # ...and leave one slowly too, so
                      # residual momentum from the corner (e.g. a vertical descent
                      # just before a 90 deg turn into a flat leg) has time to bleed
                      # off before cruise speed is commanded, instead of fighting a
                      # sudden near-max horizontal command against still-nonzero
                      # vertical velocity. PX4's measured stop lag is ~0.59 s.
            v = d / max(np.linalg.norm(d), 1e-9) * spd
        # sensed-coverage rule: near-horizontal or near-vertical only
        dxy = math.hypot(v[0], v[1])
        elev = math.atan2(abs(v[2]), dxy)
        if elev > HORIZ_ELEV_MAX and dxy > 0.05:
            if abs(err_final[2]) > 0.15 or rem >= 2.0:
                v[0] = v[1] = 0.0           # vertical step first
                L['elev_split_ticks'] = L.get('elev_split_ticks', 0) + 1
        v[2] = max(-VZ_UP_MAX, min(VZ_DN_MAX, v[2]))
        return v, rem, float(np.linalg.norm(err_final))

    def run_leg(self):
        """Returns None while flying, else a result string."""
        L = self.leg
        p = self.p_ned()
        L['dist'] += float(np.linalg.norm(p - L['last_p']))
        L['since_corner'] += float(np.linalg.norm(p - L['last_p']))
        L['last_p'] = p
        v_des, rem, err = self.guidance()
        if (L.get('aim_target') is not None and not L.get('aim_refined') and rem < AIM_REFINE_DIST_M
                and float(np.linalg.norm(self.v_ned())) < SETTLE_SPEED):
            # only once nearly stationary: while braking the airframe is pitched by ~10-17 deg,
            # and aiming with that transient pitch left a persistent error (measured, aim_after)
            # one re-aim from the EKF pose, now that the vehicle is nearly on station
            L['aim_refined'] = True
            # no body-pitch term: hover pitch is within +/-0.3 deg (measured), and a leftover
            # braking transient of a few degrees would be frozen into the command
            new = self.target_pitch_up(self.world_from_ned(p), wrap(math.pi / 2 - self.pos.heading),
                                       L['aim_target'])
            if self.gimbal_cmd is None or abs(new - (-self.gimbal_cmd)) > AIM_REFINE_MIN_RAD:
                self.set_gimbal(new)
        self.stale = False
        v, dmin, free = self.safety(v_des)
        L['ticks'] += 1
        L['stale_ticks'] = L.get('stale_ticks', 0) + self.stale
        L['min_range'] = min(L['min_range'], dmin)
        if np.linalg.norm(v) < 0.9 * np.linalg.norm(v_des) - 0.05:
            L['limited_ticks'] += 1
        # yaw: on orbit legs track the column continuously; otherwise face travel while far, the
        # waypoint heading when close
        vxy = math.hypot(v_des[0], v_des[1])
        aim_pts = L.get('aim_xy') or []
        if aim_pts:
            here = self.world_from_ned(p)
            tx, ty = min(aim_pts, key=lambda q: math.hypot(q[0] - here[0], q[1] - here[1]))
            if math.hypot(tx - here[0], ty - here[1]) > 0.5:
                yaw = self.yaw_toward(wrap(math.pi / 2 - math.atan2(ty - here[1], tx - here[0])))
            else:
                yaw = self.yaw_toward(L['yaw_ned'] if L['yaw_ned'] is not None else self.pos.heading)
        elif L['yaw_ned'] is not None and rem < 8.0:
            yaw = self.yaw_toward(L['yaw_ned'])
        elif vxy > 0.5:
            yaw = self.yaw_toward(math.atan2(v_des[1], v_des[0]))
        else:
            yaw = self.yaw_toward(self.pos.heading if self.cmd_yaw is None else self.cmd_yaw)
        self.cmd_v = v
        self.dbg = [round(float(dmin), 2) if dmin != math.inf else None,
                    round(float(free), 2) if free != math.inf else None,
                    [round(float(x), 2) for x in v_des], [round(float(x), 2) for x in v]]
        self.publish_setpoint(v, yaw)
        now = self.t()
        # blocked: wants to move but the sensed layer won't allow it
        if np.linalg.norm(v_des) > 0.3 and np.linalg.norm(v) < 0.1:
            L['blocked_since'] = L['blocked_since'] or now
            if now - L['blocked_since'] > BLOCKED_GIVEUP_S:
                return 'blocked'
        else:
            L['blocked_since'] = None
        if not L['settle']:
            return 'passed' if rem < 1.0 else ('timeout' if now - L['t0'] > L['timeout'] else None)
        yaw_ok = L['yaw_ned'] is None or abs(wrap(self.pos.heading - L['yaw_ned'])) < SETTLE_YAW_TOL
        yaw_ok = yaw_ok and now - self.gimbal_t >= GIMBAL_SETTLE_S
        if err <= SETTLE_TOL_M and yaw_ok and np.linalg.norm(self.v_ned()) < SETTLE_SPEED:
            L['held_since'] = L['held_since'] or now
            if now - L['held_since'] >= SETTLE_HOLD_S:
                return 'reached'
        else:
            L['held_since'] = None
        if now - L['t0'] > L['timeout']:
            return 'timeout'
        return None

    # ---------------- logging ----------------
    def record_track(self):
        now = self.t()
        if now - self.last_track < 0.2:
            return
        self.last_track = now
        w = self.world_from_ned(self.p_ned())
        self.log['track'].append([round(now, 2), *[round(float(x), 3) for x in w],
                                  round(float(np.linalg.norm(self.v_ned())), 3), self.phase, self.dbg])

    def write(self, final=False, track=False):
        out = {
            'plan': self.plan_path,
            'home_world_m': self.home.tolist(),
            'speed_calc': self.speed_calc(),
            'sense_stats': {k: (None if v == math.inf else v) for k, v in self.sense_stats.items()},
            'final': final,
            **{k: v for k, v in self.log.items() if final or track or k != 'track'},
        }
        tmp = os.path.join(self.results_dir, 'mission_log.json.tmp')
        json.dump(out, open(tmp, 'w'))
        os.replace(tmp, os.path.join(self.results_dir, 'mission_log.json'))

    @staticmethod
    def speed_calc():
        v = CRUISE_MPS * SPEED_OVERSHOOT
        stop = stop_dist(v)
        return {'decel_mps2': DECEL_MPS2, 'stop_k_s': STOP_K_S, 't_lat_s': T_LAT_S, 'clearance_m': SENSED_CLEARANCE_M,
                'required_margin': SAFETY_FACTOR, 'v_max_at_margin': round(v_allowed(SENSED_CLEARANCE_M), 3),
                'cruise_mps': CRUISE_MPS, 'cruise_with_overshoot_mps': round(v, 3), 'stop_dist_m': round(stop, 3),
                'achieved_margin': round(SENSED_CLEARANCE_M / stop, 3)}

    # ---------------- state machine ----------------
    def tick(self):
        if self.interrupts != self.interrupts_handled:
            self.handle_interrupt()
        if self.pos is not None:
            self.record_track()
        if self.gimbal_cmd is not None and self.t() - self.gimbal_last_pub >= 0.5:
            self.gimbal_pub.publish(Float64(data=self.gimbal_cmd))
            self.gimbal_last_pub = self.t()
        ph = self.phase
        if ph == 'WAIT':
            self.publish_setpoint([0.0, 0.0, 0.0], None)
            if self.pos is not None and self.status is not None and self.att is not None \
                    and 'lidar' in self.clouds and self.t() - self.phase_t0 > 3.0:
                self.set_phase('ARM')
            return
        if ph == 'ARM':
            self.publish_setpoint([0.0, 0.0, 0.0], self.pos.heading)
            el = self.t() - self.phase_t0
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if int(el * TICK_HZ) % 20 == 0:
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_MODE_OFFBOARD)
            elif self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                if int(el * TICK_HZ) % 20 == 0:
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                self.t_start = self.t()
                self.t_start_wall = time.time()
                self.event({'type': 'mission_start', 't_wall': self.t_start})
                self.start_leg('TAKEOFF', [], self.plan['home_air_world_m'], None)
                self.set_phase('TAKEOFF')
            if el > 40:
                self.note('FAIL: could not reach OFFBOARD+ARMED in 40 s')
                self.set_phase('ABORT')
            return
        if ph in ('TAKEOFF', 'BRAKE_OUT', 'BRAKE_BACK', 'MISSION', 'RTH'):
            res = self.run_leg()
            if res is None:
                return
            self.finish_leg(res)
            return
        if ph == 'BRAKE_RUN':
            self.brake_tick()
            return
        if ph == 'LAND':
            v, _, _ = self.safety(np.array([0.0, 0.0, 0.6]), allow_down_contact=True)
            v[2] = 0.6 if self.p_ned()[2] < -0.5 else 0.3
            self.publish_setpoint(v, self.cmd_yaw)
            landed = self.land is not None and self.land.landed
            if landed or self.t() - self.phase_t0 > 60:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.note(f'landed={landed}, disarmed')
                    self.finish_mission()
            return
        if ph in ('ABORT', 'DONE'):
            self.publish_setpoint([0.0, 0.0, 0.0], self.cmd_yaw)
            return

    def finish_leg(self, res):
        L = self.leg
        ph = self.phase
        p_world = self.world_from_ned(self.p_ned())
        if ph == 'TAKEOFF':
            self.note(f'takeoff {res} at {np.round(p_world, 2).tolist()}')
            if self.do_brake:
                ha = np.array(self.plan['home_air_world_m'])
                self.start_leg('BRAKE_OUT', [], (ha - [8, 0, 0]).tolist(), 0.0 + math.pi, settle=True)
                self.set_phase('BRAKE_OUT')
            else:
                self.next_waypoint()
            return
        if ph == 'BRAKE_OUT':
            self.brake = {'phase': 'accel', 't0': self.t()}
            self.set_phase('BRAKE_RUN')
            return
        if ph == 'BRAKE_BACK':
            self.next_waypoint()
            return
        if ph == 'RTH':
            self.note(f'RTH {res} at {np.round(p_world, 2).tolist()}')
            self.set_phase('LAND')
            return
        # MISSION waypoint
        wp = self.wps[self.i]
        if res == 'reached':
            self.last_reached_wp = wp
        err = float(np.linalg.norm(p_world - np.array(wp['position_m'])))
        rec = {
            'waypoint_id': wp['waypoint_id'], 'result': res, 'reached': res == 'reached',
            'target_world_m': wp['position_m'],
            'ekf_world_m': [round(float(x), 4) for x in p_world],
            'ekf_err_m': round(err, 4),
            'heading_target_enu_rad': wp['heading_rad'],
            'gimbal_pitch_rad': wp.get('gimbal_pitch_rad'),
            'gimbal_source': L.get('gimbal_src'),
            'continuous_aim': bool(L.get('aim_xy')),
            'gimbal_cmd_pitch_up_rad': None if self.gimbal_cmd is None else round(-self.gimbal_cmd, 4),
            'heading_ekf_enu_rad': round(wrap(math.pi / 2 - self.pos.heading), 4),
            'leg_time_s': round(self.t() - L['t0'], 2),
            'leg_dist_m': round(L['dist'], 2),
            'planned_leg_m': (self.wps[self.i + 1]['route_length_m'] if self.demo_on and self.dir < 0
                              else wp['route_length_m']),
            'pass_direction': self.dir if self.demo_on else 1,
            'lap': self.lap if self.demo_on else 0,
            'min_sensed_range_m': None if L['min_range'] == math.inf else round(L['min_range'], 3),
            'speed_limited_frac': round(L['limited_ticks'] / max(L['ticks'], 1), 3),
            'stale_lidar_ticks': L.get('stale_ticks', 0),
            'elev_split_ticks': L.get('elev_split_ticks', 0),
            't_mission_s': round(self.t() - self.t_start, 2),
        }
        self.log['waypoints'].append(rec)
        self.event({'type': 'waypoint', **rec})
        self.note(f"{wp['waypoint_id']} {res}: ekf err {err:.3f} m, leg {L['dist']:.1f} m "
                  f"in {rec['leg_time_s']:.1f} s, min sensed {rec['min_sensed_range_m']}")
        self.write()
        self.advance()

    def advance(self):
        if not self.demo_on:
            self.i += 1
            self.next_waypoint()
            return
        n = len(self.wps)
        if self.demo_wall_s > 0 and time.time() - self.t_start_wall > self.demo_wall_s:
            self.note(f'demo wall-clock limit reached after {self.lap} full laps')
            self.begin_retreat('demo_wall_limit', mid_leg=False)
            return
        if self.dir > 0 and self.i == n - 1:
            self.dir = -1
        elif self.dir < 0 and self.i == 0:
            self.lap += 1
            self.note(f'lap {self.lap} complete')
            if self.demo_laps > 0 and self.lap >= self.demo_laps:
                self.begin_retreat('demo_laps_done', mid_leg=False)
                return
            self.dir = 1
        self.i += self.dir
        self.next_waypoint()

    # ---------------- interrupt: retrace home along flown pieces, then land ----------------
    def request_interrupt(self):
        """Called from the signal handler. First call: return home along the route
        already flown and land. Second call: land where the vehicle is."""
        self.interrupts += 1

    def handle_interrupt(self):
        n = self.interrupts
        self.interrupts_handled = n
        ph = self.phase
        if ph in ('WAIT', 'ARM'):
            self.note('interrupt before takeoff: disarming and exiting')
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.set_phase('ABORT')
        elif ph in ('LAND', 'DONE', 'ABORT'):
            self.note(f'interrupt ({n}) ignored: already in {ph}')
        elif n >= 2 or ph in ('TAKEOFF', 'BRAKE_OUT', 'BRAKE_RUN', 'BRAKE_BACK'):
            self.note(f'interrupt ({n}) in {ph}: landing here')
            self.set_phase('LAND')
        elif ph == 'RTH':
            self.note('interrupt: already returning home (press again to land here)')
        else:
            self.begin_retreat('interrupt', mid_leg=True)

    def begin_retreat(self, reason, mid_leg):
        """Fly home over pieces the plan already cleared, in reverse, then land.
        mid_leg: a leg is in progress (interrupt); otherwise the vehicle sits on
        waypoint self.i (loop finished / wall-clock limit)."""
        W = self.wps
        via, k = [], None
        if mid_leg:
            L = self.leg
            world = lambda q: self.world_from_ned(q).tolist()   # noqa: E731
            if self.demo_on and self.dir < 0:
                # already heading to a lower index: finish this leg, then carry on down
                via += [world(q) for q in L['pts'][L['seg'] + 1:-1]]
                via.append(list(W[self.i]['position_m']))
                k = self.i
            else:
                # forward pass or single pass: back along what this leg has flown
                via += [world(q) for q in L['pts'][L['seg']:0:-1]]
                k = self.i - 1
                if k >= 0:
                    via.append(list(W[k]['position_m']))
        else:
            k = self.i
        while k is not None and k >= 0:
            via += [list(v) for v in reversed(W[k]['route_in'])]
            k -= 1
            if k >= 0:
                via.append(list(W[k]['position_m']))
        self.note(f'retreat ({reason}): {len(via)} route points home over the planned pieces')
        self.start_leg('RTH', via, self.plan['home_air_world_m'], 0.0)
        self.leg['aim_target'] = None
        self.leg['gimbal_src'] = None
        self.set_phase('RTH')

    def next_waypoint(self):
        while self.i < len(self.wps) and not self.wps[self.i].get('reachable_in_plan', True):
            wp = self.wps[self.i]
            rec = {'waypoint_id': wp['waypoint_id'], 'result': 'not_attempted_unreachable_in_plan',
                   'reached': False}
            self.log['waypoints'].append(rec)
            self.note(f"{wp['waypoint_id']} not attempted: no position with >=3.2 m clearance "
                      f"and line of sight to its target (see plan)")
            self.i += 1
        if self.i >= len(self.wps):
            rth = self.plan['rth']['route'] if len(self.wps) == len(self.plan['waypoints']) else []
            self.start_leg('RTH', rth, self.plan['home_air_world_m'], 0.0)
            self.set_phase('RTH')
            return
        wp = self.wps[self.i]
        route = wp['route_in']
        if self.demo_on and self.dir < 0:
            route = list(reversed(self.wps[self.i + 1]['route_in']))   # back over the piece just flown
        gp, src, tgt = wp.get('gimbal_pitch_rad'), None, None
        if gp is not None:
            src = 'plan'
        elif self.aim_from_target and wp.get('target_m') is not None:
            tgt = wp['target_m']
            gp = self.target_pitch_up(wp['position_m'], wp['heading_rad'], tgt)
            src = 'target_m'
        self.start_leg(wp['waypoint_id'], route, wp['position_m'], wp['heading_rad'], gimbal_pitch=gp)
        self.leg['aim_target'] = tgt
        self.leg['gimbal_src'] = src
        prev = self.last_reached_wp
        if (self.continuous_aim and wp.get('column_id') and wp.get('target_m') is not None and prev is not None
                and prev.get('column_id') == wp['column_id'] and prev.get('target_m') is not None):
            # both ends of this leg inspect the same column: aim at whichever end's column axis is nearer
            self.leg['aim_xy'] = [tuple(prev['target_m'][:2]), tuple(wp['target_m'][:2])]
        if self.phase != 'MISSION':
            self.set_phase('MISSION')

    def brake_tick(self):
        """Measured stopping distance at cruise: accelerate along world -x
        (open ground west of the pad), command zero, measure."""
        B = self.brake
        v_ned_cmd = np.array([0.0, -CRUISE_MPS, 0.0])   # world -x == NED -E
        spd = float(np.linalg.norm(self.v_ned()[:2]))
        now = self.t()
        if B['phase'] == 'accel':
            v, _, _ = self.safety(v_ned_cmd)
            self.publish_setpoint(v, self.cmd_yaw)
            if spd >= 0.97 * CRUISE_MPS:
                B.setdefault('at_speed', now)
            if 'at_speed' in B and now - B['at_speed'] > 1.0:
                B.update(phase='stop', t_cmd=now, p_cmd=self.p_ned().copy(), v_cmd=spd)
            elif now - B['t0'] > 25:
                B.update(phase='stop', t_cmd=now, p_cmd=self.p_ned().copy(), v_cmd=spd)
            return
        if B['phase'] == 'stop':
            self.publish_setpoint([0.0, 0.0, 0.0], self.cmd_yaw)
            if spd < 0.05 or now - B['t_cmd'] > 10:
                d = float(np.linalg.norm((self.p_ned() - B['p_cmd'])[:2]))
                res = {'speed_at_cmd_mps': round(B['v_cmd'], 3), 'stop_dist_m': round(d, 3),
                       'stop_time_s': round(now - B['t_cmd'], 2),
                       'implied_decel_mps2': round(B['v_cmd'] ** 2 / (2 * d), 3) if d > 0 else None,
                       'budget_stop_dist_m': self.speed_calc()['stop_dist_m'],
                       'note': 'measured from the instant zero velocity was commanded; excludes sensing latency (budgeted separately in t_lat)'}
                self.log['brake_test'] = res
                self.note(f'BRAKE TEST: {res}')
                self.start_leg('BRAKE_BACK', [], self.plan['home_air_world_m'], None)
                self.set_phase('BRAKE_BACK')

    def finish_mission(self):
        wl = [w for w in self.log['waypoints']]
        reached = [w for w in wl if w.get('reached')]
        errs = np.array([w['ekf_err_m'] for w in reached]) if reached else np.zeros(0)
        tr = np.array([t[1:4] for t in self.log['track']], float) if self.log['track'] else np.zeros((0, 3))
        self.log['summary'] = {
            'waypoints_in_plan': len(self.wps),
            'attempted': sum(1 for w in wl if w['result'] != 'not_attempted_unreachable_in_plan'),
            'reached': len(reached),
            'results': {r: sum(1 for w in wl if w['result'] == r) for r in sorted({w['result'] for w in wl})},
            'ekf_err_m': None if not len(errs) else {'mean': round(float(errs.mean()), 4), 'max': round(float(errs.max()), 4),
                                                     'p95': round(float(np.percentile(errs, 95)), 4)},
            'mission_time_sim_s': round(self.t() - self.t_start, 1),
            'mission_time_wall_s': round(time.time() - self.t_start_wall, 1),
            'distance_ekf_m': round(float(np.linalg.norm(np.diff(tr, axis=0), axis=1).sum()), 1) if len(tr) > 1 else 0.0,
        }
        self.note(f"SUMMARY {self.log['summary']}")
        self.event({'type': 'mission_end', 'summary': self.log['summary']})
        self.write(final=True)
        self.set_phase('DONE')


def main():
    # own SIGINT/SIGTERM handling (rclpy's default would shut the context down and
    # leave the vehicle hovering): the first signal returns home and lands, the second lands here
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = MissionFollower()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: n.request_interrupt())
    try:
        while rclpy.ok() and n.phase not in ('DONE', 'ABORT'):
            rclpy.spin_once(n, timeout_sec=0.05)
        t_end = time.time()
        while rclpy.ok() and time.time() - t_end < 2.0:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        n.note('interrupted')
    finally:
        n.write(final=n.phase == 'DONE', track=True)
        n.destroy_node()
        rclpy.try_shutdown()
    return 0 if n.phase == 'DONE' else 1


if __name__ == '__main__':
    sys.exit(main())

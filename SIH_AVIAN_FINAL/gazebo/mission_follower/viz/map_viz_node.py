#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- live 3D map and RViz support for the Gazebo mission.

VISUALISATION AND DEMO ONLY. This node never publishes anything the mission
follower reads, and it reads nothing the follower does not already read plus the
detector's optional output feed. It does not use the simulator's pose, the
collision data, the world model or any ground truth: the map is built from the
drone's own sensors and the PX4 EKF pose.

Reads
  /mission/lidar_points, /mission/up_points, /mission/down_points   range sensors
  /fmu/out/vehicle_local_position, /fmu/out/vehicle_attitude        PX4 EKF
  /mission/gimbal_cmd                                               (fallback gimbal angle)
  /viz/gz_joint_states                                              (true gimbal angle, optional)
  /detection/detections                                             (live detector output, optional)
Publishes
  TF          map -> base_link                 (sensor frames come from the URDF)
  /joint_states                                gimbal angle for robot_state_publisher
  /viz/map_cloud                               occupied voxels, coloured by height
  /viz/lidar_live, /viz/up_live, /viz/down_live  the current scans, in their own frames
  /viz/trajectory                              EKF track
  /viz/camera_axis                             where the gimbal camera is pointing
  /viz/detections                              localized defects (3D register) pinned on the map

Map method: a voxel-grid accumulator. Every scan is moved into the world frame
with the EKF pose (interpolated to the scan time), quantised to VOXEL_M voxels
and counted; voxels seen by fewer than min_hits rays are not shown. No free-space
ray-casting: nothing here navigates on this map.

What it can honestly show: the LiDAR is a 360 x 15-ray ring (+/-14 deg), so it
paints horizontal stripes on walls at flight altitude and fills piers and
columns when the drone climbs; soffits and the ground come only from the 9x9
up/down cones. Expect a scan-painted structure densest along the flown path,
not a solid surface model.
"""
import bisect
import json
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Path  # noqa: F401  (kept for users who prefer a Path display)
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float64, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import viz_frames as V  # noqa: E402
import defect_register as DR  # noqa: E402

FRAME = {'lidar': 'lidar_link', 'up': 'up_range_link', 'down': 'down_range_link'}
FAMILY_COLOR = {"CRACK": (255, 60, 60), "SPALL_DELAM": (255, 170, 30), "CORROSION_COATING": (60, 170, 255),
                "FASTENER": (170, 60, 255), "OTHER": (200, 200, 200)}
GIMBAL_JOINT_URDF = 'base_to_gimbal_cam_link'
GIMBAL_JOINT_GZ = 'gimbal_pitch_joint'


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in 'xyz') or msg.width * msg.height == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in 'xyz']
    p = np.stack(cols, 1).astype(float)
    return p[np.isfinite(p).all(1)]


def make_cloud(xyz, rgb, frame, stamp):
    n = len(xyz)
    arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr['rgb'] = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = frame, stamp
    m.height, m.width = 1, n
    m.fields = [PointField(name='x', offset=0, datatype=7, count=1),
                PointField(name='y', offset=4, datatype=7, count=1),
                PointField(name='z', offset=8, datatype=7, count=1),
                PointField(name='rgb', offset=12, datatype=7, count=1)]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * n, True
    m.data = arr.tobytes()
    return m


class MapViz(Node):
    def __init__(self):
        super().__init__('avian_map_viz')
        for k, v in (('plan', ''), ('out_dir', '/tmp/avian_viz'), ('voxel_m', 0.25), ('min_hits', 2),
                     ('max_voxels', 300000), ('map_hz', 1.0), ('merge_hz', 2.0), ('near_full_res_m', 25.0),
                     ('z_color_min', 0.0), ('z_color_max', 30.0), ('max_range_m', 29.5),
                     ('home_world_z', 0.24), ('origin_lat_deg', 47.397971057728974), ('save_map', True),
                     ('gate_m', 2.0), ('retry_s', 40.0)):
            self.declare_parameter(k, v)
        P = lambda k: self.get_parameter(k).value  # noqa: E731
        self.vox = float(P('voxel_m'))
        self.min_hits = int(P('min_hits'))
        self.cap = int(P('max_voxels'))
        self.near_m = float(P('near_full_res_m'))
        self.zc = (float(P('z_color_min')), float(P('z_color_max')))
        self.max_range = float(P('max_range_m'))
        self.out_dir = P('out_dir')
        self.save_map = bool(P('save_map'))
        os.makedirs(self.out_dir, exist_ok=True)

        home = [0.0, 0.0, float(P('home_world_z'))]
        plan = P('plan')
        if plan and os.path.exists(plan):
            hg = json.load(open(plan))['home_ground_world_m']
            home = [hg[0], hg[1], float(P('home_world_z'))]
        self.home = np.array(home)
        self.k_east, self.k_north = V.geodesy_factors(float(P('origin_lat_deg')))

        # pose history: (sim time received, position ENU, R base->map)
        self.att = None
        self.pt, self.pp, self.pR = [], [], []
        self.last_pose = None
        # gimbal angle history (rad, nose-down positive)
        self.gj_t, self.gj_v = [], []
        self.gimbal_cmd = 0.4               # airframe model's initial joint position
        self.gimbal_truth = (-1e9, 0.4)

        # map: sorted voxel keys + ray counts, plus a buffer of unmerged scans
        self.table = np.zeros(0, np.int64)
        self.cnt = np.zeros(0, np.int64)
        self.pending = []
        self.stats = {'scans': {'lidar': 0, 'up': 0, 'down': 0}, 'points_integrated': 0, 'scans_no_pose': 0,
                      'cb_ms': [], 'merge_ms': [], 'publish_ms': [], 'det_msgs': 0, 'loc_ms': []}
        self.track = []                     # decimated EKF track for the trajectory line
        self.last_track_p = None
        self.reg = DR.Register(gate_m=float(P('gate_m')), retry_s=float(P('retry_s')))
        self.eph = self.epv = None          # the EKF's own position accuracy estimate
        self._occ_cache = (None, -1)        # (sorted occupied keys, table version)
        self._table_ver = 0
        self._t_wall0, self._cpu0 = time.time(), time.process_time()

        for suf in ('', '_v1'):
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position' + suf,
                                     self.on_pos, qos_profile_sensor_data)
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude' + suf,
                                     self.on_att, qos_profile_sensor_data)
        for name in ('lidar', 'up', 'down'):
            self.create_subscription(PointCloud2, f'/mission/{name}_points',
                                     lambda m, n=name: self.on_cloud(n, m), qos_profile_sensor_data)
        self.create_subscription(Float64, '/mission/gimbal_cmd', self.on_gimbal_cmd, 10)
        self.create_subscription(JointState, '/viz/gz_joint_states', self.on_gz_joints, qos_profile_sensor_data)
        self.create_subscription(String, '/detection/detections', self.on_detections, 20)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)
        self.map_pub = self.create_publisher(PointCloud2, '/viz/map_cloud', latched)
        # RELIABLE: RViz subscribes reliable by default, and a best-effort publisher makes that
        # subscription incompatible (the display then sits in an error state).
        live_qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST)
        self.live_pub = {n: self.create_publisher(PointCloud2, f'/viz/{n}_live', live_qos) for n in FRAME}
        self.track_pub = self.create_publisher(Marker, '/viz/trajectory', latched)
        self.axis_pub = self.create_publisher(Marker, '/viz/camera_axis', latched)
        self.det_pub = self.create_publisher(MarkerArray, '/viz/detections', latched)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf = TransformBroadcaster(self)

        self.create_timer(1 / 30.0, self.tick_tf)
        self.create_timer(1 / 20.0, self.tick_joint)
        self.create_timer(1.0 / float(P('merge_hz')), self.tick_merge)
        self.create_timer(1.0 / float(P('map_hz')), self.tick_publish_map)
        self.create_timer(2.0, self.tick_track_axis)
        self.create_timer(1.0, self.tick_detections)
        self.create_timer(10.0, self.write_stats)
        self.create_timer(5.0, self.write_register)
        self.get_logger().info(f'map viz up: voxel {self.vox} m, min_hits {self.min_hits}, cap {self.cap}, '
                               f'home {self.home.tolist()}, out {self.out_dir}')

    # ---------------------------------------------------------------- time --
    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- pose --
    def on_att(self, m):
        self.att = m

    def on_pos(self, m):
        if self.att is None:
            return
        n = np.array([m.x, m.y, m.z])
        self.eph, self.epv = float(m.eph), float(m.epv)
        p = V.world_from_ned(n, self.home, self.k_east, self.k_north)
        R = V.R_map_from_base(self.att.q)
        t = self.t()
        self.pt.append(t)
        self.pp.append(p)
        self.pR.append(R)
        if len(self.pt) > 600:
            del self.pt[:100], self.pp[:100], self.pR[:100]
        self.last_pose = (t, p, R)
        if self.last_track_p is None or np.linalg.norm(p - self.last_track_p) > 1.0:
            self.track.append(p.copy())
            self.last_track_p = p
            if len(self.track) > 20000:
                self.track = self.track[::2]

    def pose_at(self, ts):
        """(position, R) at sim time ts: linear in position, nearest in attitude."""
        if not self.pt:
            return None
        i = bisect.bisect_left(self.pt, ts)
        if i <= 0:
            return self.pp[0], self.pR[0]
        if i >= len(self.pt):
            return self.pp[-1], self.pR[-1]
        t0, t1 = self.pt[i - 1], self.pt[i]
        a = 0.0 if t1 <= t0 else (ts - t0) / (t1 - t0)
        return self.pp[i - 1] * (1 - a) + self.pp[i] * a, (self.pR[i - 1] if a < 0.5 else self.pR[i])

    # -------------------------------------------------------------- gimbal --
    def on_gimbal_cmd(self, m):
        self.gimbal_cmd = float(m.data)

    def on_gz_joints(self, m):
        if GIMBAL_JOINT_GZ in m.name:
            v = float(m.position[m.name.index(GIMBAL_JOINT_GZ)])
            self.gimbal_truth = (self.t(), v)
            self.gj_t.append(self.t())
            self.gj_v.append(v)
            if len(self.gj_t) > 400:
                del self.gj_t[:100], self.gj_v[:100]

    def gimbal_angle(self, ts=None):
        """Nose-down joint angle: the bridged joint state while fresh, else the
        last command the follower published (a target, not a measurement)."""
        if self.gj_t and self.t() - self.gimbal_truth[0] < 1.0:
            if ts is not None:
                i = bisect.bisect_right(self.gj_t, ts) - 1
                if i >= 0:
                    return self.gj_v[i]
            return self.gimbal_truth[1]
        return self.gimbal_cmd

    def tick_joint(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [GIMBAL_JOINT_URDF]
        js.position = [float(max(-V.GIMBAL_LIMIT, min(V.GIMBAL_LIMIT, self.gimbal_angle())))]
        self.js_pub.publish(js)

    # ------------------------------------------------------------------ tf --
    def tick_tf(self):
        if self.last_pose is None:
            return
        _, p, R = self.last_pose
        q = Rotation.from_matrix(R).as_quat()
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'map', 'base_link'
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, p)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
        self.tf.sendTransform(m)

    # ---------------------------------------------------------------- scans --
    def on_cloud(self, name, msg):
        t0 = time.perf_counter()
        # live scan, unchanged, in its own frame (RViz places it through the URDF mounts)
        msg.header.frame_id = FRAME[name]
        self.live_pub[name].publish(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = cloud_xyz(msg)
        if len(p):
            r = np.linalg.norm(p, axis=1)
            p = p[(r > 0.15) & (r < self.max_range)]
        if len(p) == 0:
            return
        pose = self.pose_at(stamp)
        if pose is None:
            self.stats['scans_no_pose'] += 1
            return
        pos, R = pose
        world = V.sensor_to_body(name, p) @ R.T + pos
        k, c = np.unique(V.voxel_keys(world, self.vox), return_counts=True)
        self.pending.append((k, c.astype(np.int64)))
        self.stats['scans'][name] += 1
        self.stats['points_integrated'] += len(p)
        self.stats['cb_ms'].append((time.perf_counter() - t0) * 1e3)

    def tick_merge(self):
        if not self.pending:
            return
        t0 = time.perf_counter()
        pend, self.pending = self.pending, []
        keys = np.concatenate([self.table] + [k for k, _ in pend])
        cnts = np.concatenate([self.cnt] + [c for _, c in pend])
        u, inv = np.unique(keys, return_inverse=True)
        self.table = u
        self.cnt = np.bincount(inv, weights=cnts, minlength=len(u)).astype(np.int64)
        self._table_ver += 1
        self.stats['merge_ms'].append((time.perf_counter() - t0) * 1e3)

    def tick_publish_map(self):
        if len(self.table) == 0:
            return
        t0 = time.perf_counter()
        sel = self.cnt >= self.min_hits
        keys = self.table[sel]
        if len(keys) == 0:
            return
        c = V.keys_to_centres(keys, self.vox)
        n_all = len(keys)
        if n_all > self.cap:
            # stable level-of-detail: everything near the drone, a fixed hash
            # subsample of the rest, so the picture does not shimmer
            ref = self.last_pose[1] if self.last_pose else c.mean(0)
            near = np.linalg.norm(c - ref, axis=1) < self.near_m
            n_far = int((~near).sum())
            room = max(self.cap - int(near.sum()), 0)
            keep_p = 1.0 if n_far == 0 else min(1.0, room / n_far)
            h = ((keys.astype(np.uint64) * np.uint64(2654435761)) % np.uint64(1000003)).astype(np.float64) / 1000003.0
            keep = near | (h < keep_p)
            c = c[keep]
        rgb = V.turbo((c[:, 2] - self.zc[0]) / max(self.zc[1] - self.zc[0], 1e-6))
        self.map_pub.publish(make_cloud(c.astype(np.float32), rgb, 'map', self.get_clock().now().to_msg()))
        self.stats['publish_ms'].append((time.perf_counter() - t0) * 1e3)
        self.stats['voxels_total'] = int(len(self.table))
        self.stats['voxels_shown_min_hits'] = int(n_all)
        self.stats['voxels_published'] = int(len(c))

    # --------------------------------------------- trajectory + camera axis --
    def _marker(self, ns, mid, mtype, frame):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def tick_track_axis(self):
        if len(self.track) >= 2:
            m = self._marker('track', 0, Marker.LINE_STRIP, 'map')
            m.scale.x = 0.12
            m.color = ColorRGBA(r=1.0, g=0.92, b=0.1, a=1.0)
            m.points = [Point(x=float(a[0]), y=float(a[1]), z=float(a[2])) for a in self.track]
            self.track_pub.publish(m)
        a = self._marker('camera_axis', 0, Marker.LINE_LIST, 'camera_link')
        a.header.stamp = rclpy.time.Time().to_msg()      # "latest TF": the axis is drawn in a moving frame
        a.scale.x = 0.06
        a.color = ColorRGBA(r=1.0, g=0.3, b=0.8, a=0.95)
        a.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=15.0, y=0.0, z=0.0)]
        self.axis_pub.publish(a)

    # ---------------------------------------------------------- detections --
    def occ_keys(self):
        """Sorted keys of voxels seen by >= min_hits rays (cached until the map changes)."""
        keys, ver = self._occ_cache
        if ver != self._table_ver:
            keys = self.table[self.cnt >= self.min_hits]
            self._occ_cache = (keys, self._table_ver)
        return keys

    def gimbal_angle_src(self, ts):
        """(nose-down joint angle, True if it is the bridged joint state, False if only the last command)."""
        if self.gj_t and self.t() - self.gimbal_truth[0] < 1.0:
            return self.gimbal_angle(ts), True
        return self.gimbal_cmd, False

    def on_detections(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.stats['det_msgs'] += 1
        self.tick_merge()
        stamp = float(d['stamp_sim_s'])
        pose = self.pose_at(stamp)
        if pose is None:
            return
        t0 = time.perf_counter()
        pos, R = pose
        ang, measured = self.gimbal_angle_src(stamp)
        cp, cR = V.camera_pose_in_base(ang)
        o, Rc = pos + R @ cp, R @ cR
        keys = self.occ_keys()
        for det in d.get('detections', []):
            self.reg.observe({'family': det['family'], 'score': float(det['score']),
                              'bbox': [float(v) for v in det['bbox_xyxy_px']], 'stamp': stamp,
                              'wp': d.get('waypoint_id'), 'o': o.copy(), 'R_cam': Rc.copy(),
                              'sigma_gimbal': DR.SIGMA_GIMBAL_MEASURED if measured else DR.SIGMA_GIMBAL_COMMAND,
                              'eph': self.eph, 'epv': self.epv}, keys, self.vox, self.t())
        self.stats['loc_ms'].append((time.perf_counter() - t0) * 1e3)

    def tick_detections(self):
        if self.reg.pending:
            self.reg.retry(self.occ_keys(), self.vox, self.t())
        if not self.reg.clusters:
            return
        arr = MarkerArray()
        clear = self._marker('detection_clear', 0, Marker.SPHERE, 'map')
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for c in self.reg.clusters:
            col = FAMILY_COLOR.get(c['fam'], (255, 255, 255))
            rgba = ColorRGBA(r=col[0] / 255, g=col[1] / 255, b=col[2] / 255, a=0.55)
            n_id = int(c['id'][1:])
            head = f"{c['id']} {c['fam']} {c['best_conf']:.2f}  n={c['n_obs']} ({c['n_viewpoints']} vp)"
            if c['localized']:
                x, y, z = map(float, c['pos'])
                sz = max(0.5, math.sqrt(c['size_w_m'] * c['size_h_m']))
                s = self._marker('detection', n_id, Marker.SPHERE, 'map')
                s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, z
                s.scale.x = s.scale.y = s.scale.z = sz
                s.color = rgba
                arr.markers.append(s)
                t = self._marker('detection_label', n_id, Marker.TEXT_VIEW_FACING, 'map')
                t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, z + 0.5 * sz + 0.6
                t.scale.z = 0.6
                t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                t.text = (f"{head}\n{c['size_w_m']:.1f} x {c['size_h_m']:.1f} m  +/-{c['pos_sigma_m']:.1f} m  "
                          f"({(c['size_method'] == 'plane_fit') and 'plane' or 'frontal'} size)")
                arr.markers.append(t)
            else:
                o, r = c['last_ray']
                a = self._marker('detection_ray_no_map_return', n_id, Marker.ARROW, 'map')
                a.scale.x, a.scale.y, a.scale.z = 0.08, 0.25, 0.25
                a.color = ColorRGBA(r=rgba.r, g=rgba.g, b=rgba.b, a=0.45)
                a.points = [Point(x=float(o[0]), y=float(o[1]), z=float(o[2])),
                            Point(x=float(o[0] + 8 * r[0]), y=float(o[1] + 8 * r[1]), z=float(o[2] + 8 * r[2]))]
                arr.markers.append(a)
                t = self._marker('detection_label', n_id, Marker.TEXT_VIEW_FACING, 'map')
                t.pose.position.x, t.pose.position.y, t.pose.position.z = float(o[0] + 8 * r[0]), float(o[1] + 8 * r[1]), float(o[2] + 8 * r[2] + 0.6)
                t.scale.z = 0.5
                t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.7)
                t.text = f"{head}\nUNLOCALIZED (no map return on ray)"
                arr.markers.append(t)
        self.det_pub.publish(arr)

    def write_register(self, final=False):
        try:
            if final:
                self.tick_merge()
                self.reg.flush(self.occ_keys(), self.vox, self.t())
            if self.reg.stats['raw_detections'] or final:
                self.reg.write(self.out_dir)
        except Exception as e:
            self.get_logger().warn(f'register write: {e}')

    # --------------------------------------------------------------- output --
    def write_stats(self, final=False):
        def ms(v):
            return None if not v else {'n': len(v), 'mean_ms': round(float(np.mean(v)), 3),
                                       'p95_ms': round(float(np.percentile(v, 95)), 3), 'max_ms': round(float(np.max(v)), 3)}
        wall, cpu = time.time() - self._t_wall0, time.process_time() - self._cpu0
        out = {'voxel_m': self.vox, 'min_hits': self.min_hits, 'max_voxels': self.cap,
               'scans': self.stats['scans'], 'scans_without_pose': self.stats['scans_no_pose'],
               'points_integrated': self.stats['points_integrated'],
               'voxels_total': self.stats.get('voxels_total'),
               'voxels_with_min_hits': self.stats.get('voxels_shown_min_hits'),
               'voxels_published': self.stats.get('voxels_published'),
               'scan_callback': ms(self.stats['cb_ms']), 'merge': ms(self.stats['merge_ms']),
               'map_publish': ms(self.stats['publish_ms']),
               'this_process_cpu_s': round(cpu, 2), 'wall_s': round(wall, 1),
               'this_process_cpu_fraction_of_one_core': round(cpu / max(wall, 1e-6), 4),
               'detections': dict(self.reg.snapshot()['summary'], messages=self.stats['det_msgs'],
                                  localize_ms=ms(self.stats['loc_ms'])),
               'final': final}
        json.dump(out, open(os.path.join(self.out_dir, 'map_viz_stats.json'), 'w'), indent=1)

    def close(self):
        try:
            self.write_register(final=True)
            self.write_stats(final=True)
            if self.save_map and len(self.table):
                np.savez_compressed(os.path.join(self.out_dir, 'map_voxels.npz'), keys=self.table, counts=self.cnt,
                                    voxel_m=self.vox, home=self.home)
        except Exception as e:
            self.get_logger().warn(f'close: {e}')


def main():
    rclpy.init()
    n = MapViz()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""End-to-end check of map_viz_node with synthetic sensors (no simulator needed).

Puts a known wall, ceiling and floor in the world, feeds the node PX4-style pose
messages and scans expressed in each sensor's own frame, and checks that the
published voxel map puts them where they are, that TF reports the pose, and that
a synthetic detection ray lands on the wall.

Run:  source /opt/ros/jazzy/setup.bash; source ~/GarudaNEX/ros2_ws/install/setup.bash
      python3 gazebo/mission_follower/viz/test_map_node_synthetic.py
"""
import json
import math
import os
import sys
import tempfile
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import viz_frames as V  # noqa: E402

out = tempfile.mkdtemp(prefix='avian_viz_test_')
rclpy.init(args=['--ros-args', '-p', f'out_dir:={out}', '-p', 'min_hits:=1'])
import map_viz_node as M  # noqa: E402

node = M.MapViz()
tst = Node('viz_test')
ex = SingleThreadedExecutor()
ex.add_node(node)
ex.add_node(tst)
buf = Buffer()
TransformListener(buf, tst)

pos_pub = tst.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile_sensor_data)
att_pub = tst.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile_sensor_data)
cl_pub = {n: tst.create_publisher(PointCloud2, f'/mission/{n}_points', qos_profile_sensor_data) for n in ('lidar', 'up', 'down')}
det_pub = tst.create_publisher(String, '/detection/detections', 10)
got = {}
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy  # noqa: E402
lat = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
tst.create_subscription(PointCloud2, '/viz/map_cloud', lambda m: got.__setitem__('map', m), lat)
from visualization_msgs.msg import MarkerArray  # noqa: E402
tst.create_subscription(MarkerArray, '/viz/detections', lambda m: got.__setitem__('det', m), lat)


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        ex.spin_once(timeout_sec=0.01)


def cloud_msg(p, stamp):
    m = PointCloud2()
    m.header.stamp = stamp
    m.height, m.width = 1, len(p)
    m.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate('xyz')]
    m.point_step, m.row_step = 12, 12 * len(p)
    m.data = np.asarray(p, np.float32).tobytes()
    m.is_dense = True
    return m


# ---- vehicle: NED (5, 3, -10), heading east (90 deg), home at world (20, -30) ----
ke, kn = V.geodesy_factors()
home = np.array([20.0, -30.0, 0.24])
node.home = home.copy()                       # same as the plan's declared home
ned = np.array([5.0, 3.0, -10.0])
psi = math.pi / 2
q = np.array([math.cos(psi / 2), 0, 0, math.sin(psi / 2)])
pos_w = V.world_from_ned(ned, home, ke, kn)
R = V.R_map_from_base(q)
print('vehicle world position', np.round(pos_w, 3), ' forward axis in map', np.round(R @ [1, 0, 0], 3))

pm = VehicleLocalPosition(); pm.x, pm.y, pm.z, pm.heading = map(float, (*ned, psi))
am = VehicleAttitude(); am.q = [float(x) for x in q]
for _ in range(60):
    pos_pub.publish(pm); att_pub.publish(am); spin(0.01)

# ---- world: wall at east = pos_e + 15, ceiling z = pos_z + 12, floor z = pos_z - 9 ----
wall_x, ceil_z, floor_z = pos_w[0] + 15.0, pos_w[2] + 12.0, pos_w[2] - 9.0
rng = np.random.default_rng(0)
wy = pos_w[1] + rng.uniform(-6, 6, 2500)
wz = pos_w[2] + rng.uniform(-3, 3, 2500)
wall_w = np.stack([np.full_like(wy, wall_x), wy, wz], 1)
b = (wall_w - pos_w) @ R                                # body FLU (R^T applied to rows)
lidar_s = b - V.LIDAR_MOUNT
cx = pos_w[0] + rng.uniform(-4, 4, 600); cy = pos_w[1] + rng.uniform(-4, 4, 600)
ceil_b = (np.stack([cx, cy, np.full_like(cx, ceil_z)], 1) - pos_w) @ R
up_s = np.stack([ceil_b[:, 2] - V.UP_MOUNT[2], ceil_b[:, 1], -ceil_b[:, 0]], 1)     # inverse of the follower's up mapping
floor_b = (np.stack([cx, cy, np.full_like(cx, floor_z)], 1) - pos_w) @ R
dn_s = np.stack([-(floor_b[:, 2] - V.DOWN_MOUNT[2]), floor_b[:, 1], floor_b[:, 0]], 1)

for _ in range(6):                                       # a few scans of each
    now = node.get_clock().now().to_msg()
    pos_pub.publish(pm); att_pub.publish(am)
    cl_pub['lidar'].publish(cloud_msg(lidar_s, now))
    cl_pub['up'].publish(cloud_msg(up_s, now))
    cl_pub['down'].publish(cloud_msg(dn_s, now))
    spin(0.15)
spin(2.5)

fails = 0
def check(name, ok, detail=''):
    global fails
    fails += (not ok)
    print(('PASS ' if ok else 'FAIL ') + name + (f'  {detail}' if detail else ''))

m = got.get('map')
check('map cloud published', m is not None)
arr = np.frombuffer(bytes(m.data), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
xyz = np.stack([arr['x'], arr['y'], arr['z']], 1)
check('map cloud fields parse (x,y,z,rgb; 16-byte points)', m.point_step == 16 and len(xyz) == m.width, f'{m.width} voxels')
wall = xyz[np.abs(xyz[:, 0] - wall_x) < 0.4]
check('wall voxels sit on x = wall (within one voxel)', len(wall) > 100 and float(np.abs(wall[:, 0] - wall_x).max()) <= 0.25 + 1e-6,
      f'{len(wall)} voxels, max |dx| {float(np.abs(wall[:, 0] - wall_x).max()):.3f}')
ce = xyz[np.abs(xyz[:, 2] - ceil_z) < 0.4]
check('ceiling voxels (up cone) sit on z = ceiling', len(ce) > 50 and float(np.abs(ce[:, 2] - ceil_z).max()) <= 0.25 + 1e-6,
      f'{len(ce)} voxels')
fl = xyz[np.abs(xyz[:, 2] - floor_z) < 0.4]
check('floor voxels (down cone) sit on z = floor', len(fl) > 50 and float(np.abs(fl[:, 2] - floor_z).max()) <= 0.25 + 1e-6,
      f'{len(fl)} voxels')
check('no stray voxels: every voxel is on wall, ceiling or floor', len(wall) + len(ce) + len(fl) == len(xyz), f'{len(xyz)} total')
check('colour follows height (floor and ceiling colours differ)', int(arr['rgb'][np.argmin(xyz[:, 2])]) != int(arr['rgb'][np.argmax(xyz[:, 2])]))

tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
t = tf.transform.translation
check('TF map->base_link position equals EKF-derived pose', np.allclose([t.x, t.y, t.z], pos_w, atol=1e-6))
qq = tf.transform.rotation
from scipy.spatial.transform import Rotation  # noqa: E402
check('TF map->base_link rotation equals EKF-derived attitude', np.allclose(Rotation.from_quat([qq.x, qq.y, qq.z, qq.w]).as_matrix(), R, atol=1e-6))
check('drone faces east (+x map) after heading 90 deg', np.allclose(R @ [1, 0, 0], [1, 0, 0], atol=1e-6))

# detection ray: a detection at the image centre with the gimbal level looks straight along +x
node.gimbal_cmd = 0.0
det_pub.publish(String(data=json.dumps({'stamp_sim_s': node.get_clock().now().nanoseconds * 1e-9, 'frame_id': 1,
                                        'waypoint_id': 'T1', 'detections': [{'family': 'SPALL_DELAM', 'score': 0.9, 'bbox_xyxy_px': [300, 220, 340, 260]}]})))
spin(2.5)
d = got.get('det')
check('detection marker published', d is not None and len(d.markers) >= 1)
sph = [mk for mk in d.markers if mk.ns == 'detection'] if d else []
check('detection ray lands on the wall (real ray-cast into the built map)', len(sph) == 1 and abs(sph[0].pose.position.x - wall_x) < 0.6,
      f'marker x {sph[0].pose.position.x:.2f} vs wall {wall_x:.2f}' if sph else 'no sphere')

node.close()
reg = json.load(open(os.path.join(out, 'defect_register.json')))
check('defect register JSON written on close, 1 raw detection -> 1 distinct localized defect',
      reg['summary']['raw_detections'] == 1 and reg['summary']['distinct_localized_defects'] == 1, json.dumps(reg['summary']))
r0 = reg['defects'][0]
check('register position on the wall, with size and uncertainty',
      abs(r0['x_m'] - wall_x) < 0.4 and r0['size_w_m'] > 0 and r0['pos_sigma_m'] > 0, json.dumps(r0))
check('defect_register.csv written', os.path.exists(os.path.join(out, 'defect_register.csv')))
s = json.load(open(os.path.join(out, 'map_viz_stats.json')))
print('stats:', json.dumps({k: s[k] for k in ('scans', 'voxels_total', 'scan_callback', 'merge', 'map_publish', 'detections')}))
print('\nRESULT:', 'ALL PASS' if not fails else f'{fails} FAILED')
node.destroy_node(); tst.destroy_node(); rclpy.shutdown()
sys.exit(1 if fails else 0)

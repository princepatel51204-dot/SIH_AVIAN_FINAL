"""Pin viz_frames.py to the mission follower's own frame maths (offline, no ROS graph).

Run:  source /opt/ros/jazzy/setup.bash; source ~/GarudaNEX/ros2_ws/install/setup.bash
      python3 gazebo/mission_follower/viz/test_viz_frames.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import viz_frames as V                                   # noqa: E402
import mission_follower_node as MF                       # noqa: E402

rng = np.random.default_rng(1)
fails = 0


def check(name, ok, detail=''):
    global fails
    fails += (not ok)
    print(('PASS ' if ok else 'FAIL ') + name + (f'  {detail}' if detail else ''))


# 1. attitude: the follower turns a body-FLU vector into a level-NED vector by
#    flipping to FRD and applying quat_rotate(att.q); the viz maps it to ENU.
worst = 0.0
for _ in range(200):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    v_flu = rng.normal(size=(5, 3)) * 10
    ned = MF.quat_rotate(q, v_flu * np.array([1.0, -1.0, -1.0]))          # follower
    enu_ref = np.stack([ned[:, 1], ned[:, 0], -ned[:, 2]], 1)             # NED -> ENU
    enu = v_flu @ V.R_map_from_base(q).T                                   # viz
    worst = max(worst, float(np.abs(enu - enu_ref).max()))
check('attitude: viz base->map rotation == follower quat_rotate path (200 random attitudes)', worst < 1e-9, f'max err {worst:.2e}')

# 2. position: identical geodesy and home handling
ke, kn = V.geodesy_factors()
home = np.array([20.0, -30.0, 0.24])
n = np.array([12.3, -4.5, -8.0])
w_ref = np.array([(n[1] / ke) + home[0], (n[0] / kn) + home[1], -n[2] + home[2]])
check('position: NED -> world matches follower formula', np.allclose(V.world_from_ned(n, home, ke, kn), w_ref))

# 3. sensor mounts: known points
lid = V.sensor_to_body('lidar', np.array([[5.0, 0, 0]]))
up = V.sensor_to_body('up', np.array([[5.0, 0, 0]]))
dn = V.sensor_to_body('down', np.array([[5.0, 0, 0]]))
check('lidar mount: sensor +x 5 m -> body (5.12, 0, 0.08)', np.allclose(lid, [[5.12, 0, 0.08]]))
check('up cone: sensor +x 5 m -> body (0, 0, 5.42)', np.allclose(up, [[0, 0, 5.42]]))
check('down cone: sensor +x 5 m -> body (0, 0, -5.12)', np.allclose(dn, [[0, 0, -5.12]]))
check('airframe returns are cropped', len(V.sensor_to_body('lidar', np.array([[0.1, 0, 0.0]]))) == 0)

# 4. the same mounts appear in the follower module source (guards against drift)
src = open(os.path.join(HERE, '..', 'mission_follower_node.py')).read()
check("follower still uses lidar mount [0.12, 0.0, 0.08]", 'np.array([0.12, 0.0, 0.08])' in src)
check("follower still uses up mount z 0.42 and down mount z -0.12", '[0, 0, 0.42]' in src and '[0, 0, -0.12]' in src)

# 5. URDF mounts equal the constants (the RViz TF tree is built from the URDF)
urdf = open(os.path.join(HERE, 'x500_viz.urdf.in')).read()
for tag, xyz in (('lidar_link', V.LIDAR_MOUNT), ('up_range_link', V.UP_MOUNT), ('down_range_link', V.DOWN_MOUNT),
                 ('gimbal_cam_link', V.GIMBAL_PIVOT)):
    key = f'name="base_to_{tag}"'
    i = urdf.index(key)
    seg = urdf[i:i + 400]
    o = seg.index('<origin xyz="') + len('<origin xyz="')
    got = np.array([float(x) for x in seg[o:seg.index('"', o)].split()])
    check(f'URDF {tag} origin {got.tolist()} == constant {xyz.tolist()}', np.allclose(got, xyz))

# 6. camera model
check('camera: forward point projects to image centre', np.allclose(V.project_to_pixel([8, 0, 0]), [320, 240]))
u, v = V.project_to_pixel([8, 0, 8 * math.tan(math.radians(20))])
check('camera: point 20 deg above axis is above centre by fx*tan20 px', abs((240 - v) - V.CAM_FX * math.tan(math.radians(20))) < 1e-6)
check('camera: pixel_ray inverts project_to_pixel', np.allclose(V.pixel_ray(*V.project_to_pixel([8, 2, -1])) * 8.0 / V.pixel_ray(*V.project_to_pixel([8, 2, -1]))[0], [8, 2, -1]))
p, R = V.camera_pose_in_base(0.0)
check('camera at joint 0 sits at pivot + 0.03 m ahead', np.allclose(p, [0.38, 0, 0.05]))
p, R = V.camera_pose_in_base(math.pi / 2)
check('gimbal +90 deg (nose down): camera x axis points body -z', np.allclose(R @ [1, 0, 0], [0, 0, -1]))

# 7. voxel keys round trip
pts = rng.uniform(-300, 300, size=(1000, 3))
k = V.voxel_keys(pts, 0.25)
c = V.keys_to_centres(k, 0.25)
check('voxel keys: centre within half a voxel of the point', float(np.abs(c - pts).max()) <= 0.125 + 1e-9)
print('\nRESULT:', 'ALL PASS' if not fails else f'{fails} FAILED')
sys.exit(1 if fails else 0)

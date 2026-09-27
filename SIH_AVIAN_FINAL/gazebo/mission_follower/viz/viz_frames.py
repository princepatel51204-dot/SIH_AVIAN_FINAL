"""Frame and camera maths shared by the RViz visualisation nodes and their tests.

Pure numpy, no ROS. Everything here is derived from constants the mission
follower already uses (sensor mount offsets, the EKF home and geodesy factors)
or from the airframe model (gimbal pivot, camera pose, 80 deg HFOV, 640x480).

VISUALISATION ONLY. Nothing here, or in the nodes that import it, feeds
navigation or avoidance.

Frames
  map        world-aligned ENU (x east, y north, z up); origin = the world
             origin, so a point in `map` has the same numbers as the same point
             in the Gazebo world. Derived only from the EKF pose plus the
             plan's declared home -- no simulator pose.
  base_link  the airframe, FLU (x forward, y left, z up)
Sensors are fixed to base_link at the mount offsets below.
"""
import math

import numpy as np

# ---- sensor mounts in base_link FLU (same numbers as mission_follower_node) --
LIDAR_MOUNT = np.array([0.12, 0.0, 0.08])
UP_MOUNT = np.array([0.0, 0.0, 0.42])       # pitched -90 deg: sensor +x -> body +z
DOWN_MOUNT = np.array([0.0, 0.0, -0.12])    # pitched +90 deg: sensor +x -> body -z
# airframe bounding box (body FLU, m): returns inside it are the aircraft itself
SELF_BOX = np.array([[-0.55, -0.55, -0.45], [0.55, 0.55, 0.60]])

# ---- gimbal camera (airframe model) ----------------------------------------
GIMBAL_PIVOT = np.array([0.35, 0.0, 0.05])  # gimbal_cam_link in base_link
CAM_IN_LINK = np.array([0.03, 0.0, 0.0])    # front_camera in gimbal_cam_link
GIMBAL_LIMIT = 1.5708                       # joint travel +/- (rad)
CAM_W, CAM_H = 640, 480
CAM_HFOV = 1.3962634
CAM_FX = (CAM_W / 2) / math.tan(CAM_HFOV / 2)   # square pixels
CAM_FY = CAM_FX
CAM_CX, CAM_CY = CAM_W / 2, CAM_H / 2
INSPECT_HFOV = 0.2792527                    # inspect_camera (16 deg), same link and pose as front_camera


def use_camera_hfov(hfov_rad):
    """Switch the intrinsics used by this module (and its importers) to a 640x480 camera of this HFOV.
    Both gimbal cameras share the pose above, so only the focal length changes."""
    global CAM_HFOV, CAM_FX, CAM_FY
    CAM_HFOV = float(hfov_rad)
    CAM_FX = CAM_FY = (CAM_W / 2) / math.tan(CAM_HFOV / 2)

# ---- PX4 local NED -> world ENU (identical to the follower) ------------------
_S = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # NED -> ENU
_F = np.diag([1.0, -1.0, -1.0])                                        # FRD <-> FLU


def geodesy_factors(origin_lat_deg=47.397971057728974):
    """(k_east, k_north): PX4-local metres per world metre, from WGS84 vs the
    sphere PX4's local projection uses (see mission_follower_node)."""
    lat = math.radians(origin_lat_deg)
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    s = 1 - e2 * math.sin(lat) ** 2
    n_, m_ = a / math.sqrt(s), a * (1 - e2) / s ** 1.5
    r = 6371000.0
    return r / n_, r / m_


def world_from_ned(n, home, k_east, k_north):
    n = np.asarray(n, float)
    return np.array([n[1] / k_east + home[0], n[0] / k_north + home[1], -n[2] + home[2]])


def rot_from_quat_wxyz(q):
    """Rotation matrix of quaternion (w, x, y, z): body -> parent."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_map_from_base(att_q):
    """base_link (FLU) -> map (ENU) rotation from PX4's attitude quaternion
    (which is body FRD -> NED)."""
    return _S @ rot_from_quat_wxyz(att_q) @ _F


def sensor_to_body(name, p):
    """(N,3) sensor-frame returns -> base_link FLU, cropped of the airframe."""
    p = np.asarray(p, float)
    if name == 'lidar':
        b = p + LIDAR_MOUNT
    elif name == 'up':
        b = np.stack([-p[:, 2], p[:, 1], p[:, 0]], 1) + UP_MOUNT
    elif name == 'down':
        b = np.stack([p[:, 2], p[:, 1], -p[:, 0]], 1) + DOWN_MOUNT
    else:
        raise ValueError(name)
    inside = ((b > SELF_BOX[0]) & (b < SELF_BOX[1])).all(1)
    return b[~inside]


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def camera_pose_in_base(theta_j):
    """front_camera pose in base_link: (position, rotation). theta_j is the
    gimbal joint angle (rad), positive = nose DOWN (rotation about +y)."""
    R = Ry(theta_j)
    return GIMBAL_PIVOT + R @ CAM_IN_LINK, R


def project_to_pixel(p_cam):
    """Point in the camera frame (x forward, y left, z up) -> (u, v, in_front)."""
    x, y, z = p_cam
    if x <= 1e-6:
        return None
    return CAM_CX - CAM_FX * y / x, CAM_CY - CAM_FY * z / x


def pixel_ray(u, v):
    """Unit ray in the camera frame through pixel (u, v)."""
    d = np.array([1.0, -(u - CAM_CX) / CAM_FX, -(v - CAM_CY) / CAM_FY])
    return d / np.linalg.norm(d)


# ---- voxel keys ---------------------------------------------------------------
_OFF = 1 << 20


def voxel_keys(p, vox):
    i = np.floor(np.asarray(p, float) / vox).astype(np.int64) + _OFF
    return (i[:, 0] << 42) | (i[:, 1] << 21) | i[:, 2]


def keys_to_centres(k, vox):
    ix = (k >> 42) & 0x1FFFFF
    iy = (k >> 21) & 0x1FFFFF
    iz = k & 0x1FFFFF
    return (np.stack([ix, iy, iz], 1).astype(np.float64) - _OFF + 0.5) * vox


def turbo(t):
    """Google's polynomial approximation of the Turbo colormap; t in [0,1] -> (N,3) uint8."""
    t = np.clip(np.asarray(t, float), 0.0, 1.0)
    r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))))
    g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))))
    b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))))
    return (np.clip(np.stack([r, g, b], 1), 0, 1) * 255).astype(np.uint8)

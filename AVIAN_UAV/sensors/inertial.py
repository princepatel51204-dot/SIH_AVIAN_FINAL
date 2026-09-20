"""LiDAR, IMU and GNSS.

LIDAR
-----
Ray-batch against the real collision world. Measured on this host:

     7,200 rays   101 ms
    57,600 rays   682 ms

A full 32 x 1800 scan at 10 Hz would cost 6.8 s of CPU per simulated second,
per aircraft. The default is therefore DECIMATED -- 16 channels x 450 azimuth
at 5 Hz -- and the full configuration is available and measured rather than
quietly unavailable. The decimation is a hardware consequence, not a modelling
opinion, and `decimation_factor` records it in every scan.

IMU
---
Specific force and angular rate in the body frame, with bias random-walk and
white noise. The bias walk matters: an IMU with only white noise integrates to
a bounded error and makes dead reckoning look far better than it is.

GNSS
----
No geodetic datum exists in this project -- the environment is a local metric
frame with its origin at the bridge deck start. A datum is therefore CHOSEN,
documented, and clearly labelled arbitrary. Nothing in the system depends on
it being a real place; it exists so that a lat/lon appears where a lat/lon is
expected, and so the local/global conversion is exercised.

Quality degradation is NOT looked up from a table. It is computed by casting
rays at the sky and measuring how much of the upper hemisphere is occluded --
under the deck that is most of it, which is the physically real reason GNSS
fails there. HDOP and position error follow from the visible-satellite count.
"""
from __future__ import annotations

import math

import numpy as np

from sensors.base import Sensor

# Arbitrary but documented datum. NOT a real survey point, and nothing is
# claimed about the site. Local +X (corridor) maps to East, +Y to North.
DATUM_LAT_DEG = 22.300000
DATUM_LON_DEG = 73.200000
DATUM_ALT_M = 32.0
EARTH_R = 6378137.0


class Lidar(Sensor):
    kind = "LIDAR"

    def __init__(self, vehicle, name="lidar", frame_id="sensor_lidar",
                 rate_hz=5.0, channels=16, azimuth=450, seed=0,
                 min_range_m=0.10, max_range_m=70.0, vfov_deg=59.0,
                 range_sigma_m=0.03, full_channels=32, full_azimuth=1800):
        super().__init__(vehicle, name, frame_id, rate_hz, seed)
        self.channels = channels
        self.azimuth = azimuth
        self.min_range = min_range_m
        self.max_range = max_range_m
        self.vfov = vfov_deg
        self.range_sigma = range_sigma_m
        self.full_channels = full_channels
        self.full_azimuth = full_azimuth
        self.decimation = (full_channels * full_azimuth) / float(
            channels * azimuth)
        self._dirs = self._ray_directions()
        self.last_scan = None

    def _ray_directions(self):
        """Unit directions in the sensor frame, computed once."""
        el = np.radians(np.linspace(-self.vfov / 2.0, self.vfov / 2.0,
                                    self.channels))
        az = np.linspace(0.0, 2 * math.pi, self.azimuth, endpoint=False)
        E, A = np.meshgrid(el, az, indexing="ij")
        d = np.stack([np.cos(E) * np.cos(A),
                      np.cos(E) * np.sin(A),
                      np.sin(E)], axis=-1)
        return d.reshape(-1, 3)

    def read(self, t):
        pos, q = self.pose()
        R = np.array(self.pb.getMatrixFromQuaternion(q)).reshape(3, 3)
        dirs_w = self._dirs @ R.T
        n = len(dirs_w)
        starts = np.tile(pos, (n, 1)) + dirs_w * self.min_range
        ends = np.tile(pos, (n, 1)) + dirs_w * self.max_range

        hits = np.full(n, np.nan)
        objs = np.full(n, -1, dtype=int)
        self_hits = 0
        # rayTestBatch has a hard cap on batch size; chunking is required.
        CH = 10000
        for i in range(0, n, CH):
            res = self.pb.rayTestBatch(starts[i:i + CH].tolist(),
                                       ends[i:i + CH].tolist())
            for k, r in enumerate(res):
                # Discard returns off our OWN airframe. The sensor sits
                # 130 mm above the body on a vehicle with 575 mm arms, so a
                # large share of the lower hemisphere strikes the aircraft
                # itself; counting those as environment returns put a
                # phantom obstacle 2.7 cm away and would have made every
                # proximity check meaningless.
                if r[0] >= 0 and r[0] != self.v.body:
                    frac = r[2]
                    hits[i + k] = self.min_range + frac * (
                        self.max_range - self.min_range)
                    objs[i + k] = r[0]
                elif r[0] == self.v.body:
                    self_hits += 1

        valid = ~np.isnan(hits)
        hits[valid] += self.rng.normal(0.0, self.range_sigma, valid.sum())
        pts = starts[valid] + dirs_w[valid] * (
            hits[valid] - self.min_range)[:, None]
        self.last_scan = pts

        return self.envelope(t, {
            "n_rays": n, "n_returns": int(valid.sum()),
            "return_fraction": round(float(valid.sum()) / n, 4),
            "points_world": pts,
            "ranges_m": hits,
            "nearest_m": (round(float(np.nanmin(hits)), 4)
                          if valid.any() else None),
            "self_occluded_rays": self_hits,
            "channels": self.channels, "azimuth_bins": self.azimuth,
            "decimation_factor": round(self.decimation, 2),
            "full_config": f"{self.full_channels}x{self.full_azimuth}",
            "position": [round(float(v), 4) for v in pos],
        }, valid=bool(valid.any()))


class IMU(Sensor):
    kind = "IMU"

    def __init__(self, vehicle, name="imu", frame_id="sensor_imu",
                 rate_hz=200.0, seed=0,
                 accel_sigma=0.02, gyro_sigma=0.0015,
                 accel_bias_walk=0.0008, gyro_bias_walk=0.00008):
        super().__init__(vehicle, name, frame_id, rate_hz, seed)
        self.accel_sigma = accel_sigma
        self.gyro_sigma = gyro_sigma
        self.accel_bias_walk = accel_bias_walk
        self.gyro_bias_walk = gyro_bias_walk
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self._prev_v = None
        self._prev_t = None

    def read(self, t):
        st = self.v.state()
        R = np.array(self.pb.getMatrixFromQuaternion(
            st["quaternion"])).reshape(3, 3)
        vel = st["velocity"]
        if self._prev_v is None or self._prev_t is None or t <= self._prev_t:
            a_world = np.zeros(3)
        else:
            a_world = (vel - self._prev_v) / (t - self._prev_t)
        self._prev_v = vel.copy()
        self._prev_t = t

        # SPECIFIC FORCE, not acceleration. An accelerometer in free fall
        # reads zero and one at rest reads +g; subtracting gravity here is
        # what makes the reading physical rather than a kinematic derivative.
        f_world = a_world + np.array([0.0, 0.0, 9.80665])
        f_body = R.T @ f_world
        w_body = R.T @ st["angular_velocity"]

        dt = self.period if self.period > 0 else 1.0 / 200.0
        self.accel_bias += self.rng.normal(
            0.0, self.accel_bias_walk * math.sqrt(dt), 3)
        self.gyro_bias += self.rng.normal(
            0.0, self.gyro_bias_walk * math.sqrt(dt), 3)

        accel = f_body + self.accel_bias + self.rng.normal(
            0.0, self.accel_sigma, 3)
        gyro = w_body + self.gyro_bias + self.rng.normal(
            0.0, self.gyro_sigma, 3)

        return self.envelope(t, {
            "linear_acceleration": accel,
            "angular_velocity": gyro,
            "orientation_quat": st["quaternion"],
            "accel_bias": self.accel_bias.copy(),
            "gyro_bias": self.gyro_bias.copy(),
            "note": "specific force in the body frame; gravity included",
        })


class GNSS(Sensor):
    kind = "GNSS"

    def __init__(self, vehicle, name="gnss", frame_id="sensor_imu",
                 rate_hz=5.0, seed=0, base_sigma_m=1.2,
                 n_constellation=14):
        super().__init__(vehicle, name, frame_id, rate_hz, seed)
        self.base_sigma = base_sigma_m
        self.n_constellation = n_constellation
        # Directions to a nominal constellation: an elevation/azimuth spread
        # over the upper hemisphere. Fixed, so sky occlusion is repeatable.
        rng = np.random.default_rng(20260828)
        el = np.radians(rng.uniform(12.0, 85.0, n_constellation))
        az = rng.uniform(0.0, 2 * math.pi, n_constellation)
        self.sat_dirs = np.stack([np.cos(el) * np.cos(az),
                                  np.cos(el) * np.sin(az),
                                  np.sin(el)], axis=-1)

    def _visible_satellites(self, pos):
        """Ray-cast to each satellite direction. Occluded = not received.

        This is why GNSS fails under a bridge, and computing it rather than
        tabulating it means the answer stays correct if the structure moves.
        """
        starts = np.tile(pos, (self.n_constellation, 1)) + \
            self.sat_dirs * 0.5
        ends = np.tile(pos, (self.n_constellation, 1)) + \
            self.sat_dirs * 400.0
        res = self.pb.rayTestBatch(starts.tolist(), ends.tolist())
        return sum(1 for r in res if r[0] < 0)

    def read(self, t):
        st = self.v.state()
        pos = st["position"]
        n_vis = self._visible_satellites(pos)

        # HDOP rises sharply as the constellation thins. Below 4 satellites
        # there is no 3-D fix at all -- reported as invalid, not as a
        # degraded number, because a receiver with 3 satellites does not
        # output a slightly worse position.
        if n_vis < 4:
            quality, hdop, fix = "DENIED", float("inf"), 0
            return self.envelope(t, {
                "fix_type": fix, "quality": quality,
                "satellites_visible": n_vis, "hdop": None,
                "latitude_deg": None, "longitude_deg": None,
                "altitude_m": None, "position_enu": None,
                "sigma_m": None,
                "reason": "fewer than 4 satellites: sky occluded by "
                          "structure",
            }, valid=False)

        hdop = max(0.8, 12.0 / n_vis)
        sigma = self.base_sigma * hdop
        quality = ("HIGH" if n_vis >= 10 else
                   "MEDIUM" if n_vis >= 7 else "LOW")
        fix = 3

        noise = self.rng.normal(0.0, sigma, 3)
        noise[2] *= 1.6                    # vertical is always worse
        enu = pos + noise

        # Local ENU -> geodetic about the documented arbitrary datum.
        lat = DATUM_LAT_DEG + math.degrees(enu[1] / EARTH_R)
        lon = DATUM_LON_DEG + math.degrees(
            enu[0] / (EARTH_R * math.cos(math.radians(DATUM_LAT_DEG))))
        alt = DATUM_ALT_M + enu[2]

        return self.envelope(t, {
            "fix_type": fix, "quality": quality,
            "satellites_visible": n_vis, "hdop": round(hdop, 3),
            "latitude_deg": round(lat, 8), "longitude_deg": round(lon, 8),
            "altitude_m": round(float(alt), 3),
            "position_enu": enu,
            "sigma_m": round(float(sigma), 3),
            "datum": {"lat": DATUM_LAT_DEG, "lon": DATUM_LON_DEG,
                      "alt_m": DATUM_ALT_M,
                      "note": "ARBITRARY reference; no real site is claimed"},
        })

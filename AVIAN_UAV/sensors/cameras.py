"""RGB and depth cameras, rendered by PyBullet's CPU rasteriser.

RESOLUTION POLICY, AND WHY IT IS A HARD CONSTRAINT
--------------------------------------------------
Measured on this host:

    320x240    121 ms/frame     8.2 fps
    640x480    348 ms/frame     2.9 fps
    1920x1080  2259 ms/frame    0.44 fps

Those are the numbers the whole mission design has to live inside. A 30-minute
budget for a six-aircraft mission leaves roughly 15 minutes of rendering, which
is about 2,500 frames at 640x480 across the whole fleet -- roughly 400 each.
Continuous perception at any useful rate is therefore impossible, and the
architecture reflects that: cameras are WAYPOINT-TRIGGERED, not free-running.
`LOOP` resolution is used while inspecting, `CAPTURE` only at a defect
waypoint, exactly as the brief specifies.

If this ran on a GPU the policy would change; the policy is a consequence of
the hardware, and it is recorded rather than hidden.

DEPTH
-----
PyBullet returns a non-linear depth buffer. It is converted to metres here,
because a buffer value is not a measurement and every consumer would otherwise
have to repeat the conversion and eventually one of them would get it wrong.
Beyond `max_range_m` the reading is marked invalid rather than clamped: a
depth camera does not return 8 m for something 40 m away, it returns nothing.
"""
from __future__ import annotations

import math

import numpy as np

from sensors.base import Sensor

LOOP_RES = (640, 480)
CAPTURE_RES = (1920, 1080)


class RGBCamera(Sensor):
    kind = "RGB_CAMERA"

    def __init__(self, vehicle, name="rgb_front", frame_id="sensor_rgb_front",
                 rate_hz=2.0, hfov_deg=69.0, vfov_deg=42.0, seed=0,
                 near=0.05, far=150.0, noise_sigma=1.6, exposure_gain=1.0):
        super().__init__(vehicle, name, frame_id, rate_hz, seed)
        self.hfov = hfov_deg
        self.vfov = vfov_deg
        self.near = near
        self.far = far
        self.noise_sigma = noise_sigma        # per-channel, 0-255 counts
        self.exposure_gain = exposure_gain
        self.last_image = None

    def _matrices(self, w, h):
        pos, q = self.pose()
        R = np.array(self.pb.getMatrixFromQuaternion(q)).reshape(3, 3)
        fwd = R @ np.array([0.0, 0.0, -1.0])      # optical axis
        up = R @ np.array([0.0, 1.0, 0.0])
        view = self.pb.computeViewMatrix(list(pos), list(pos + fwd * 10.0),
                                         list(up))
        proj = self.pb.computeProjectionMatrixFOV(
            self.vfov, w / float(h), self.near, self.far)
        return view, proj, pos, R

    def read(self, t, capture=False, resolution=None):
        w, h = resolution or (CAPTURE_RES if capture else LOOP_RES)
        view, proj, pos, R = self._matrices(w, h)
        img = self.pb.getCameraImage(
            w, h, view, proj, renderer=self.pb.ER_TINY_RENDERER,
            flags=self.pb.ER_NO_SEGMENTATION_MASK)
        rgb = np.reshape(img[2], (h, w, 4))[:, :, :3].astype(np.float32)

        # Sensor noise and exposure. Applied AFTER rendering, so the detector
        # never sees a noise-free synthetic image -- which would flatter it.
        if self.exposure_gain != 1.0:
            rgb *= self.exposure_gain
        if self.noise_sigma > 0:
            rgb += self.rng.normal(0.0, self.noise_sigma, rgb.shape)
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        self.last_image = rgb

        return self.envelope(t, {
            "width": w, "height": h, "channels": 3,
            "image": rgb,
            "hfov_deg": self.hfov, "vfov_deg": self.vfov,
            "position": [round(float(v), 4) for v in pos],
            "optical_axis": [round(float(v), 4) for v in
                             (R @ np.array([0, 0, -1.0]))],
            "capture_mode": "CAPTURE" if capture else "LOOP",
            "gsd_mm_at_1m": round(
                1000.0 * 2.0 * math.tan(math.radians(self.hfov / 2.0)) / w, 4),
        })


class DepthCamera(Sensor):
    kind = "DEPTH_CAMERA"

    def __init__(self, vehicle, name="depth", frame_id="sensor_depth",
                 rate_hz=2.0, hfov_deg=87.0, vfov_deg=58.0, seed=0,
                 min_range_m=0.30, max_range_m=8.0, accuracy_pct=2.0):
        super().__init__(vehicle, name, frame_id, rate_hz, seed)
        self.hfov = hfov_deg
        self.vfov = vfov_deg
        self.min_range = min_range_m
        self.max_range = max_range_m
        self.accuracy_pct = accuracy_pct
        self.last_depth = None

    def read(self, t, resolution=(320, 240)):
        w, h = resolution
        pos, q = self.pose()
        R = np.array(self.pb.getMatrixFromQuaternion(q)).reshape(3, 3)
        fwd = R @ np.array([0.0, 0.0, -1.0])
        up = R @ np.array([0.0, 1.0, 0.0])
        near, far = 0.05, max(20.0, self.max_range * 3)
        view = self.pb.computeViewMatrix(list(pos), list(pos + fwd * 10.0),
                                         list(up))
        proj = self.pb.computeProjectionMatrixFOV(self.vfov, w / float(h),
                                                  near, far)
        img = self.pb.getCameraImage(w, h, view, proj,
                                     renderer=self.pb.ER_TINY_RENDERER)
        buf = np.reshape(img[3], (h, w)).astype(np.float64)
        # Segmentation, used ONLY to mask the aircraft's own structure. The
        # depth sensor sits 200 mm forward on a vehicle whose arms reach
        # 575 mm, so a propulsion arm is squarely in frame and was being
        # reported as an obstacle 0.3 m away -- which would have made every
        # obstacle check downstream meaningless. A real airborne depth camera
        # is calibrated with exactly this mask.
        seg = np.reshape(img[4], (h, w))
        # Background is -1, and -1 & 0xFFFFFF is 16777215, which would alias
        # onto a body id if one ever got that high. Test for a real hit first.
        self_mask = (seg >= 0) & ((seg & ((1 << 24) - 1)) == self.v.body)

        # Non-linear buffer -> metres. This is the standard OpenGL depth
        # linearisation; getting it wrong yields plausible-looking but
        # completely incorrect ranges, which is a hard bug to spot downstream.
        z = far * near / (far - (far - near) * buf)

        # Range-proportional noise, which is how a stereo depth camera
        # actually behaves: error grows with the square of range, but the
        # datasheet figure is a percentage, so that is what is modelled.
        sigma = z * (self.accuracy_pct / 100.0)
        z_noisy = z + self.rng.normal(0.0, np.maximum(sigma, 1e-6))

        valid_mask = ((z_noisy >= self.min_range)
                      & (z_noisy <= self.max_range) & (~self_mask))
        z_out = np.where(valid_mask, z_noisy, np.nan)
        self.last_depth = z_out

        n_valid = int(valid_mask.sum())
        # Range at the principal point, median-filtered over a small window.
        # This is the quantity that can be compared against a single ray cast
        # along the boresight, which is how the sensor is actually verified;
        # `nearest_m` is a whole-frame minimum and answers a different
        # question.
        k = 6
        cen = z_out[max(0, h // 2 - k):h // 2 + k + 1,
                    max(0, w // 2 - k):w // 2 + k + 1]
        centre = (float(np.nanmedian(cen))
                  if np.any(~np.isnan(cen)) else None)
        return self.envelope(t, {
            "width": w, "height": h,
            "depth_m": z_out,
            "centre_range_m": (round(centre, 4) if centre is not None
                               else None),
            "centre_window_px": 2 * k + 1,
            "valid_pixels": n_valid,
            "self_masked_pixels": int(self_mask.sum()),
            "valid_fraction": round(n_valid / float(w * h), 4),
            "min_range_m": self.min_range, "max_range_m": self.max_range,
            "nearest_m": (round(float(np.nanmin(z_out)), 4)
                          if n_valid else None),
            "position": [round(float(v), 4) for v in pos],
        }, valid=n_valid > 0)

    def deproject(self, u, v, z, resolution=(320, 240)):
        """Pixel + range -> a point in WORLD coordinates.

        This is the function 3-D defect localisation depends on, so it is a
        method rather than something each caller re-derives.
        """
        w, h = resolution
        pos, q = self.pose()
        R = np.array(self.pb.getMatrixFromQuaternion(q)).reshape(3, 3)
        fy = (h / 2.0) / math.tan(math.radians(self.vfov / 2.0))
        fx = fy                                   # square pixels
        cx, cy = w / 2.0, h / 2.0
        x_c = (u - cx) / fx * z
        y_c = -(v - cy) / fy * z
        p_cam = np.array([x_c, y_c, -z])          # -Z forward
        return pos + R @ p_cam

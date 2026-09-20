"""Sensor base class: frames, rates, timestamps and noise.

Every AVIAN sensor is a rate-limited producer with an explicit mounting
transform and an explicit noise model. Four properties are mandatory on every
reading, because a reading missing any of them cannot be fused or replayed:

    frame_id     which frame the data is expressed in
    stamp        simulation time the measurement is valid for
    seq          monotonic sequence number, so dropped samples are visible
    valid        whether the sensor produced usable data this tick

MOUNTING
--------
Offsets come from `avian_description_manifest.json`, which is generated from
the CAD and matches `AVIAN_sensor_manifest_REV_B.json` exactly. A sensor's
pose is therefore never typed in twice: the URDF, the environment package and
this module all read the same numbers.

RATE LIMITING
-------------
`due(t)` gates production. Physics runs at 240 Hz; a 5 Hz LiDAR must not be
sampled 240 times a second, and a controller that receives fresh data every
tick is being tested against a sensor that does not exist. The gate is on
SIMULATION time, not wall time, so determinism is preserved.

NOISE
-----
Every noise term is drawn from a per-sensor `numpy.random.Generator` seeded
from the world seed and the sensor name. Two runs of the same mission with the
same seed therefore produce identical noise, which is what makes the
determinism guarantee survive turning sensors on.
"""
from __future__ import annotations

import hashlib
import math

import numpy as np


def _seed_for(world_seed, name):
    h = hashlib.sha256(f"{world_seed}:{name}".encode()).digest()
    return int.from_bytes(h[:8], "little")


class Sensor:
    """Base for all AVIAN sensors."""

    kind = "SENSOR"

    def __init__(self, vehicle, name, frame_id, rate_hz, seed=0,
                 enabled=True):
        self.v = vehicle
        self.pb = vehicle.pb
        self.name = name
        self.frame_id = frame_id
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.0
        self.rng = np.random.default_rng(_seed_for(seed, f"{vehicle.vid}"
                                                         f"/{name}"))
        self.enabled = enabled
        self.failed = False
        self.seq = 0
        self.last_stamp = -1e9
        self._next_due = None
        self.n_produced = 0
        self.link_index = vehicle.link_of.get(frame_id)
        if self.link_index is None:
            raise KeyError(
                f"{vehicle.vid}: URDF has no link '{frame_id}'. Sensor frames "
                f"must exist in the description, not be assumed.")

    # -----------------------------------------------------------------
    def due(self, t):
        """Phase-accumulating gate, so the LONG-RUN rate is exact.

        A naive `t - last >= period` gate loses samples whenever the sensor
        rate does not divide the physics rate: a 200 Hz IMU stepped at 240 Hz
        fired only 120 times a second, because after firing at 0.005 the next
        opportunity was 0.00833. Accumulating the phase instead of resetting
        it to `t` keeps the average rate correct.
        """
        if not (self.enabled and not self.failed):
            return False
        if self.period <= 0.0:
            return True
        if self._next_due is None:
            self._next_due = t
        return t >= self._next_due - 1e-9

    def reset_timing(self, t=None):
        """Re-anchor the rate gate to `t`, or to the next call if None.

        The gate keys off `_next_due`, not `last_stamp`, so clearing
        `last_stamp` alone leaves a sensor that has already been read
        silently muted until simulation time catches up with its accumulated
        phase. Anything that restarts a clock -- a new mission leg, a replay,
        a test that rewinds time -- must call this.
        """
        self._next_due = t
        self.last_stamp = -1e9

    def pose(self):
        """World pose of the sensor frame: (position, quaternion)."""
        st = self.pb.getLinkState(self.v.body, self.link_index,
                                  computeForwardKinematics=True)
        return np.array(st[4]), np.array(st[5])

    def basis(self):
        """Rotation matrix of the sensor frame in world coordinates."""
        _, q = self.pose()
        return np.array(self.pb.getMatrixFromQuaternion(q)).reshape(3, 3)

    def optical_axis(self):
        """-Z of the frame, matching the camera convention in the CAD."""
        return self.basis() @ np.array([0.0, 0.0, -1.0])

    def envelope(self, t, payload, valid=True):
        self.seq += 1
        self.last_stamp = t
        if self.period > 0.0:
            if self._next_due is None:
                self._next_due = t
            self._next_due += self.period
            # If the caller has fallen far behind, resynchronise rather than
            # firing a burst to catch up.
            if self._next_due < t - self.period:
                self._next_due = t + self.period
        if valid:
            self.n_produced += 1
        out = {
            "sensor": self.name, "kind": self.kind,
            "frame_id": self.frame_id, "vehicle_id": self.v.vid,
            "stamp": round(float(t), 6), "seq": self.seq,
            "valid": bool(valid), "rate_hz": self.rate_hz,
        }
        out.update(payload)
        return out

    def fail(self, reason="injected"):
        self.failed = True
        self.fail_reason = reason

    def restore(self):
        self.failed = False

    def read(self, t):
        raise NotImplementedError

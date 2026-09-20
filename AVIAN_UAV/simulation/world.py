"""Simulation world: physics setup and the bridge collision asset.

Config-driven vehicle spawning from the start. Six aircraft are not spawned in
Phase 4 -- the revision is explicit about validating one first -- but
`spawn_fleet()` exists and takes a count, so Phase 9 adds vehicles rather than
rewriting the world.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")

DEFAULT_DT = 1.0 / 240.0


class AvianWorld:
    def __init__(self, pb=None, gui=False, dt=DEFAULT_DT, seed=0,
                 load_bridge=True):
        import pybullet as _pb
        self.pb = pb or _pb
        self.dt = dt
        self.seed = seed
        self.gui = gui
        self.cid = self.pb.connect(self.pb.GUI if gui else self.pb.DIRECT)
        self.pb.setGravity(0, 0, -9.80665)
        self.pb.setTimeStep(dt)
        # Determinism: a fixed substep count and a fixed solver iteration
        # count. Leaving these to PyBullet's adaptive defaults makes two runs
        # of the same mission differ, which would make the determinism test
        # meaningless rather than passing it honestly.
        self.pb.setPhysicsEngineParameter(
            numSolverIterations=50, numSubSteps=1,
            fixedTimeStep=dt, deterministicOverlappingPairs=1,
            enableConeFriction=0)
        np.random.seed(seed)

        self.vehicles = {}
        self.bridge_ids = []
        self.bridge_manifest = None
        self.t = 0.0
        self.steps = 0
        if load_bridge:
            self.load_bridge()

    # -----------------------------------------------------------------
    def load_bridge(self, asset=None):
        """Load the Phase 3 collision asset as static primitives."""
        asset = asset or os.path.join(ASSETS, "avian_bridge_collision.json")
        man = os.path.join(ASSETS, "avian_bridge_collision_manifest.json")
        if not os.path.exists(asset):
            raise FileNotFoundError(
                f"{asset} not found. Run avian/sim/export_bridge_collision.py")
        prims = json.load(open(asset))["primitives"]
        self.bridge_manifest = json.load(open(man))
        pb = self.pb

        shapes, poses = [], []
        for p in prims:
            if p["type"] == "BOX":
                cs = pb.createCollisionShape(
                    pb.GEOM_BOX, halfExtents=p["half_extents"])
            else:
                cs = pb.createCollisionShape(
                    pb.GEOM_CYLINDER, radius=p["radius"],
                    height=p["half_height"] * 2.0)
            shapes.append(cs)
            poses.append((p["centre"],
                          pb.getQuaternionFromEuler([0, 0, p.get("yaw", 0)])))

        # One multibody per primitive would cost 677 bodies and a broadphase
        # entry each. A single static multibody with 677 collision links is
        # one broadphase island and steps far faster.
        for i in range(0, len(shapes), 120):
            chunk_s = shapes[i:i + 120]
            chunk_p = poses[i:i + 120]
            bid = pb.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                basePosition=[0, 0, 0],
                linkMasses=[0.0] * len(chunk_s),
                linkCollisionShapeIndices=chunk_s,
                linkVisualShapeIndices=[-1] * len(chunk_s),
                linkPositions=[p[0] for p in chunk_p],
                linkOrientations=[p[1] for p in chunk_p],
                linkInertialFramePositions=[[0, 0, 0]] * len(chunk_s),
                linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(chunk_s),
                linkParentIndices=[0] * len(chunk_s),
                linkJointTypes=[pb.JOINT_FIXED] * len(chunk_s),
                linkJointAxis=[[0, 0, 1]] * len(chunk_s))
            self.bridge_ids.append(bid)
        return self.bridge_ids

    # -----------------------------------------------------------------
    def spawn_fleet(self, count=1, positions=None, config=None,
                    start_index=None):
        """Config-driven spawn. Six aircraft is `count=6`, nothing else.

        Numbering CONTINUES from the vehicles already present. The first
        version restarted at AVIAN_01 on every call, so a second spawn
        silently replaced the first in the registry -- which would have been
        a very confusing failure with six aircraft.
        """
        from simulation.vehicle import AvianVehicle
        base = (start_index if start_index is not None
                else len(self.vehicles))
        out = []
        for k in range(count):
            vid = f"AVIAN_{base + k + 1:02d}"
            if vid in self.vehicles:
                raise ValueError(f"{vid} already exists in this world")
            pos = (positions[k] if positions
                   else (1600.0 + 12.0 * k, -60.0, 30.0))
            v = AvianVehicle(self.pb, vid=vid, position=pos)
            self.vehicles[vid] = v
            out.append(v)
        return out

    def step(self, controllers=None):
        # Order matters: controllers decide, actuators apply, THEN the
        # solver steps. PyBullet clears external forces after every step, so
        # applying them before the controller has run wastes them.
        if controllers:
            for c in controllers:
                c.update(self.dt)
        for v in self.vehicles.values():
            v.step_actuators(self.dt)
        self.pb.stepSimulation()
        self.t += self.dt
        self.steps += 1

    def min_clearance(self, vid):
        """Closest approach between a vehicle and the bridge, metres."""
        v = self.vehicles[vid]
        best = 1e9
        for bid in self.bridge_ids:
            pts = self.pb.getClosestPoints(v.body, bid, 50.0)
            for c in pts:
                best = min(best, c[8])
        return best

    def contacts(self, vid):
        v = self.vehicles[vid]
        n = 0
        for bid in self.bridge_ids:
            n += len(self.pb.getContactPoints(v.body, bid))
        return n

    def close(self):
        self.pb.disconnect(self.cid)

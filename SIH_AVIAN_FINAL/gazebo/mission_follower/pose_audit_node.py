#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 1: mapping-accuracy AUDIT (logging only).

Reads the simulator's TRUE vehicle pose straight from Gazebo transport
(/world/<world>/dynamic_pose/info) and, each time the mission follower
reports a waypoint on /mission/event, logs the true pose next to the
plan's intended pose and the follower's own EKF pose.

This node is deliberately separate from the follower and publishes
nothing on ROS: the true pose never reaches navigation or avoidance. It
exists only to measure (a) whether the plan's frame really lines up with
the Gazebo world ("3D model exactly mapped") and (b) how far the vehicle's
own estimate is from the truth.

Usage: pose_audit_node.py <out.json> [world] [model]
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


def quat_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class PoseAudit(Node):
    def __init__(self, out, world, model):
        super().__init__('sih_pose_audit')
        self.out, self.model = out, model
        self.lock = threading.Lock()
        self.gt = None          # (t_wall, xyz, yaw, entry_name)
        self.names_seen = None
        self.rest_pose = None
        self.entity = None
        self.track = []
        self.records = []
        self.gz = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_gz):
            self.get_logger().error(f'could not subscribe {topic}')
        self.create_subscription(String, '/mission/event', self.on_event, 50)
        self.create_timer(5.0, self.write)
        self.get_logger().info(f'pose audit on {topic}, model {model}')

    def on_gz(self, msg):
        names = [p.name for p in msg.pose]
        if self.names_seen is None:
            self.names_seen = names[:40]
        ent = {p.name: p for p in msg.pose}
        if self.model not in ent:
            return
        m = ent[self.model]
        mp = np.array([m.position.x, m.position.y, m.position.z])
        mq = m.orientation
        R = quat_mat(mq.w, mq.x, mq.y, mq.z)
        # Audit the body origin (base_link) -- the point PX4's EKF estimates
        # and the follower commands -- not the model origin at the feet.
        # dynamic_pose link entries are relative to their model; compose.
        name = self.model
        xyz, Rw = mp, R
        if 'base_link' in ent:
            b = ent['base_link']
            bp = np.array([b.position.x, b.position.y, b.position.z])
            bq = b.orientation
            if np.linalg.norm(bp) < 2.0:   # a small offset => model-relative
                xyz = mp + R @ bp
                Rw = R @ quat_mat(bq.w, bq.x, bq.y, bq.z)
                name = self.model + '::base_link (composed)'
            else:
                xyz, Rw, name = bp, quat_mat(bq.w, bq.x, bq.y, bq.z), 'base_link'
        yaw = math.atan2(Rw[1, 0], Rw[0, 0])
        now = time.time()
        with self.lock:
            self.gt = (now, xyz, yaw, name)
            self.entity = name
            if self.rest_pose is None:
                self.rest_pose = xyz.round(4).tolist()
            if not self.track or now - self.track[-1][0] >= 0.2:
                self.track.append([round(now, 2), *xyz.round(3).tolist()])

    def on_event(self, msg):
        d = json.loads(msg.data)
        with self.lock:
            gt = self.gt
        if d.get('type') != 'waypoint' or gt is None or 'ekf_world_m' not in d:
            if d.get('type') == 'mission_end':
                self.write()
            return
        tgt = np.array(d['target_world_m'])
        ekf = np.array(d['ekf_world_m'])
        xyz = gt[1]
        self.records.append({
            'waypoint_id': d['waypoint_id'], 'result': d['result'],
            'target_world_m': tgt.tolist(),
            'true_world_m': xyz.round(4).tolist(),
            'ekf_world_m': ekf.tolist(),
            'true_minus_target_m': (xyz - tgt).round(4).tolist(),
            'true_err_m': round(float(np.linalg.norm(xyz - tgt)), 4),
            'true_minus_ekf_m': (xyz - ekf).round(4).tolist(),
            'ekf_vs_true_err_m': round(float(np.linalg.norm(xyz - ekf)), 4),
            'true_yaw_enu_rad': round(gt[2], 4),
            'target_yaw_enu_rad': d.get('heading_target_enu_rad'),
            'gt_age_s': round(time.time() - gt[0], 3),
        })

    def summary(self):
        r = [x for x in self.records if x['result'] == 'reached']
        if not r:
            return None
        e = np.array([x['true_err_m'] for x in r])
        off = np.array([x['true_minus_target_m'] for x in r])
        ev = np.array([x['ekf_vs_true_err_m'] for x in r])
        yaw = np.array([abs((x['true_yaw_enu_rad'] - x['target_yaw_enu_rad'] + math.pi) % (2 * math.pi) - math.pi)
                        for x in r if x['target_yaw_enu_rad'] is not None])
        tr = np.array([t[1:] for t in self.track])
        return {
            'n_reached': len(r),
            'true_err_m': {'mean': round(float(e.mean()), 4), 'median': round(float(np.median(e)), 4),
                           'p95': round(float(np.percentile(e, 95)), 4), 'max': round(float(e.max()), 4)},
            'mean_offset_true_minus_target_xyz_m': off.mean(0).round(4).tolist(),
            'std_offset_xyz_m': off.std(0).round(4).tolist(),
            'ekf_vs_true_err_m': {'mean': round(float(ev.mean()), 4), 'max': round(float(ev.max()), 4)},
            'yaw_err_deg': None if not len(yaw) else {'mean': round(float(np.degrees(yaw.mean())), 2),
                                                      'max': round(float(np.degrees(yaw.max())), 2)},
            'distance_true_m': round(float(np.linalg.norm(np.diff(tr, axis=0), axis=1).sum()), 1) if len(tr) > 1 else 0,
        }

    def write(self):
        with self.lock:
            out = {'source': 'Gazebo dynamic_pose/info (simulator truth, audit only)',
                   'model': self.model, 'audited_entity': self.entity, 'names_seen_first_msg': self.names_seen,
                   'rest_pose_first_sample_m': self.rest_pose,
                   'summary': self.summary(), 'records': self.records, 'track': self.track}
        tmp = self.out + '.tmp'
        json.dump(out, open(tmp, 'w'))
        os.replace(tmp, self.out)


def main():
    out = sys.argv[1]
    world = sys.argv[2] if len(sys.argv) > 2 else 'sih_avian_final'
    model = sys.argv[3] if len(sys.argv) > 3 else 'x500_lidar_2d_0'
    rclpy.init()
    n = PoseAudit(out, world, model)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.write()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

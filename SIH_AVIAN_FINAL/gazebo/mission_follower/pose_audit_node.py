#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 1: mapping-accuracy AUDIT (logging only).

Reads the simulator's TRUE vehicle pose straight from Gazebo transport
(/world/<world>/dynamic_pose/info) and the true gimbal angle from the
model's JointStatePublisher, and
  * appends the full true pose (sim time, base_link position, attitude
    quaternion, gimbal joint angle) at 20 Hz sim to <out>_track.csv -- this
    is what flown coverage is measured from, frame by frame;
  * each time the mission follower reports a waypoint on /mission/event,
    logs the true pose next to the plan's intended pose and the follower's
    own EKF pose (<out>.json).

This node is deliberately separate from the follower and publishes
nothing on ROS: the true pose never reaches navigation or avoidance. It
exists only to measure (a) whether the plan's frame really lines up with
the Gazebo world, (b) how far the vehicle's own estimate is from the truth
and (c) what the camera actually saw.

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
from gz.msgs10.model_pb2 import Model

TRACK_DT_SIM = 0.05
GIMBAL_JOINT = 'gimbal_pitch_joint'


def quat_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


class PoseAudit(Node):
    def __init__(self, out, world, model):
        super().__init__('sih_pose_audit')
        self.out, self.model = out, model
        self.lock = threading.Lock()
        self.gt = None          # (t_wall, xyz, yaw, entity, t_sim, quat, joint)
        self.names_seen = None
        self.rest_pose = None
        self.entity = None
        self.joint = None       # (t_sim, angle)
        self.last_track_sim = -1e9
        self.last_xyz = None
        self.dist = 0.0
        self.n_track = 0
        self.records = []
        base = os.path.splitext(out)[0]
        self.track_path = base + '_track.csv'
        self.track_f = open(self.track_path, 'w')
        self.track_f.write('t_sim,t_wall,x,y,z,qw,qx,qy,qz,gimbal_joint_rad\n')
        self.gz = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_gz):
            self.get_logger().error(f'could not subscribe {topic}')
        jt = f'/world/{world}/model/{model}/joint_state'
        if not self.gz.subscribe(Model, jt, self.on_joint):
            self.get_logger().error(f'could not subscribe {jt}')
        self.create_subscription(String, '/mission/event', self.on_event, 50)
        self.create_timer(5.0, self.write)
        self.get_logger().info(f'pose audit on {topic} + {jt}, model {model}')

    def on_joint(self, msg):
        for j in msg.joint:
            if j.name == GIMBAL_JOINT or j.name.endswith('::' + GIMBAL_JOINT):
                t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                with self.lock:
                    self.joint = (t, j.axis1.position)
                return

    def on_gz(self, msg):
        if self.names_seen is None:
            self.names_seen = [p.name for p in msg.pose][:40]
        ent = {p.name: p for p in msg.pose}
        if self.model not in ent:
            return
        t_sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        m = ent[self.model]
        mp = np.array([m.position.x, m.position.y, m.position.z])
        mq = (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
        R = quat_mat(*mq)
        # Audit the body origin (base_link) -- the point PX4's EKF estimates
        # and the follower commands -- not the model origin at the feet.
        # dynamic_pose link entries are relative to their model; compose.
        name, xyz, q = self.model, mp, mq
        if 'base_link' in ent:
            b = ent['base_link']
            bp = np.array([b.position.x, b.position.y, b.position.z])
            bq = (b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z)
            if np.linalg.norm(bp) < 2.0:   # a small offset => model-relative
                xyz, q, name = mp + R @ bp, quat_mul(mq, bq), self.model + '::base_link (composed)'
            else:
                xyz, q, name = bp, bq, 'base_link'
        Rw = quat_mat(*q)
        yaw = math.atan2(Rw[1, 0], Rw[0, 0])
        now = time.time()
        with self.lock:
            joint = self.joint[1] if self.joint else float('nan')
            self.gt = (now, xyz, yaw, name, t_sim, q, joint)
            self.entity = name
            if self.rest_pose is None:
                self.rest_pose = xyz.round(4).tolist()
            if t_sim - self.last_track_sim >= TRACK_DT_SIM - 1e-9:
                self.last_track_sim = t_sim
                self.track_f.write(f'{t_sim:.3f},{now:.3f},{xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f},'
                                   f'{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f},{joint:.5f}\n')
                self.n_track += 1
                if self.last_xyz is not None:
                    self.dist += float(np.linalg.norm(xyz - self.last_xyz))
                self.last_xyz = xyz

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
            'true_quat_wxyz': [round(v, 6) for v in gt[5]],
            'true_gimbal_joint_rad': None if gt[6] != gt[6] else round(gt[6], 4),
            'target_gimbal_pitch_up_rad': d.get('gimbal_pitch_rad'),
            't_sim': round(gt[4], 3),
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
        gim = np.array([abs(-x['true_gimbal_joint_rad'] - x['target_gimbal_pitch_up_rad']) for x in r
                        if x.get('true_gimbal_joint_rad') is not None and x.get('target_gimbal_pitch_up_rad') is not None])
        return {
            'n_reached': len(r),
            'true_err_m': {'mean': round(float(e.mean()), 4), 'median': round(float(np.median(e)), 4),
                           'p95': round(float(np.percentile(e, 95)), 4), 'max': round(float(e.max()), 4)},
            'mean_offset_true_minus_target_xyz_m': off.mean(0).round(4).tolist(),
            'std_offset_xyz_m': off.std(0).round(4).tolist(),
            'ekf_vs_true_err_m': {'mean': round(float(ev.mean()), 4), 'max': round(float(ev.max()), 4)},
            'yaw_err_deg': None if not len(yaw) else {'mean': round(float(np.degrees(yaw.mean())), 2),
                                                      'max': round(float(np.degrees(yaw.max())), 2)},
            'gimbal_err_deg': None if not len(gim) else {'mean': round(float(np.degrees(gim.mean())), 2),
                                                         'max': round(float(np.degrees(gim.max())), 2)},
            'distance_true_m': round(self.dist, 1),
        }

    def write(self):
        with self.lock:
            self.track_f.flush()
            out = {'source': 'Gazebo dynamic_pose/info + joint_state (simulator truth, audit only)',
                   'model': self.model, 'audited_entity': self.entity, 'names_seen_first_msg': self.names_seen,
                   'rest_pose_first_sample_m': self.rest_pose, 'track_csv': os.path.basename(self.track_path),
                   'track_samples': self.n_track, 'summary': self.summary(), 'records': self.records}
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
        n.track_f.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

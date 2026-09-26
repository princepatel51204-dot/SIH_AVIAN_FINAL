#!/usr/bin/env python3
"""Save one frame from each /decal_cam_* topic (bridged to ROS) as PNG. Usage: capture_frames.py <outdir> <topic> [...]"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PI
out = sys.argv[1]; topics = sys.argv[2:]
rclpy.init(); n = Node('decal_capture'); got = {}
def cb(t):
    def f(m):
        if t in got or time.time() - t0 < 4.0: return   # skip warm-up frames
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        PI.fromarray(a).save(f'{out}/{t.strip("/")}.png'); got[t] = 1
    return f
t0 = time.time()
for t in topics: n.create_subscription(Image, t, cb(t), qos_profile_sensor_data)
while rclpy.ok() and len(got) < len(topics) and time.time() - t0 < 60: rclpy.spin_once(n, timeout_sec=0.2)
print('captured', sorted(got))

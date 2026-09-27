#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 1: record the front_camera for the whole
flight (live-view camera only; flat-shaded Gazebo geometry has no defect
textures, so this is NOT the detection pipeline).

  * every frame -> piped into ffmpeg -> <out_dir>/front_camera.mp4
  * frames.csv: frame index, ROS stamp, wall time (so any frame can be
    matched to the flight log later)
  * on each /mission/event waypoint: a JPEG of the current frame,
    <out_dir>/waypoints/<CWP_xxx>.jpg
  * camera_stats.json: frame count, measured rate, gaps

Usage: camera_recorder_node.py <out_dir> [image_topic (default /camera/image_raw)] [node_name]
"""
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PILImage


class CameraRecorder(Node):
    def __init__(self, out_dir, topic='/camera/image_raw', name='sih_camera_recorder'):
        super().__init__(name)
        self.out = out_dir
        os.makedirs(os.path.join(out_dir, 'waypoints'), exist_ok=True)
        self.ff = None
        self.n = 0
        self.t_first = self.t_last = None
        self.max_gap = 0.0
        self.last = None
        self.wp_frames = []
        self.csv = open(os.path.join(out_dir, 'frames.csv'), 'w')
        self.csv.write('frame,stamp_s,wall_s\n')
        self.create_subscription(Image, topic, self.on_img, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/event', self.on_event, 50)
        self.create_timer(5.0, self.write_stats)

    def on_img(self, m):
        if m.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'unexpected encoding {m.encoding}', throttle_duration_sec=10)
            return
        if self.ff is None:
            self.size = (m.width, m.height)
            self.enc = m.encoding
            self.ff = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                 '-pix_fmt', 'rgb24' if m.encoding == 'rgb8' else 'bgr24',
                 '-s', f'{m.width}x{m.height}', '-r', '13', '-i', '-',
                 '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
                 os.path.join(self.out, 'front_camera.mp4')], stdin=subprocess.PIPE)
        now = time.time()
        if self.t_last is not None:
            self.max_gap = max(self.max_gap, now - self.t_last)
        self.t_first = self.t_first or now
        self.t_last = now
        data = bytes(m.data)
        if m.step != m.width * 3:
            data = b''.join(data[r * m.step:r * m.step + m.width * 3] for r in range(m.height))
        try:
            self.ff.stdin.write(data)
        except BrokenPipeError:
            pass
        self.last = data
        self.csv.write(f'{self.n},{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.3f},{now:.3f}\n')
        self.n += 1

    def on_event(self, msg):
        d = json.loads(msg.data)
        if d.get('type') != 'waypoint' or self.last is None:
            if d.get('type') == 'mission_end':
                self.write_stats()
            return
        img = PILImage.frombytes('RGB', self.size, self.last)
        if self.enc == 'bgr8':
            img = img.convert('RGB')
        img.save(os.path.join(self.out, 'waypoints', f"{d['waypoint_id']}.jpg"), quality=90)
        self.wp_frames.append({'waypoint_id': d['waypoint_id'], 'frame': self.n - 1})

    def write_stats(self):
        dur = (self.t_last - self.t_first) if self.n > 1 else 0
        st = {'frames': self.n, 'duration_wall_s': round(dur, 1),
              'rate_hz': round((self.n - 1) / dur, 2) if dur > 0 else None,
              'max_gap_s': round(self.max_gap, 3), 'waypoint_snapshots': len(self.wp_frames),
              'waypoint_frames': self.wp_frames}
        json.dump(st, open(os.path.join(self.out, 'camera_stats.json'), 'w'), indent=1)
        self.csv.flush()

    def close(self):
        self.write_stats()
        self.csv.close()
        if self.ff:
            self.ff.stdin.close()
            self.ff.wait(timeout=30)


def main():
    rclpy.init()
    n = CameraRecorder(*sys.argv[1:4])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

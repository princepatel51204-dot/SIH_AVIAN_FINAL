#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 2: REAL live crack detection on the mission.

Loads the existing trained detector (headline model: torchvision Faster
R-CNN MobileNetV3-Large FPN, weights v2, threshold 0.65 -- picked by F1 on
MISSION-VAL, see detection/AVIAN_detector_report_FINAL.md) into a ROS2 node
that subscribes to /camera/image_raw, runs inference on every frame it can
keep up with (never queues stale frames -- always the newest), and:
  * publishes /detection/image_annotated (bboxes + family + confidence drawn)
  * records that annotated feed to its own MP4 (detections/annotated.mp4)
  * logs every individual detection to detections.json: waypoint id, sim
    time, frame id, bbox, confidence, drone pose (PX4 EKF, world frame --
    the same estimate the follower flies on, not simulator ground truth)
  * reports the ACTUAL achieved inference rate measured on this machine
    during this run (detection_stats.json) -- never an assumed number

Honesty boundary (see the report this node's output feeds): Gazebo's
corridor world is flat-shaded, untextured collision geometry -- it has no
crack imagery. Detections in-sim exercise the real trained weights and the
real inference/publish/log pipeline end to end, which is what this node
proves. They are NOT evidence of real-world detection accuracy; that
evidence is the separate offline Blender zoom-tile benchmark in
detection/AVIAN_detector_report_FINAL.md. This node's own output must never
be quoted as an accuracy number, only as a pipeline/rate number.

Usage: live_detector_node.py --ros-args -p out_dir:=... -p plan:=... \
         [-p weights:=v2] [-p score_thresh:=0.65] [-p torch_threads:=4]
Must run under the venv that has torch+torchvision AND can see the sourced
ROS2 environment: /home/prince/avian_rev_c/.venv/bin/python3
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DATASET_DIR = os.path.join(ROOT, "dataset")
DET_DIR = os.path.join(ROOT, "detection")

WEIGHTS_BY_NAME = {
    "v1": "AVIAN_detector_weights_FINAL.pt",
    "v2": "AVIAN_detector_weights_v2_FINAL.pt",
    "v3": "AVIAN_detector_weights_v3_FINAL.pt",
}
FAMILY_COLOR = {
    "CRACK": (255, 60, 60), "SPALL_DELAM": (255, 170, 30), "CORROSION_COATING": (60, 170, 255),
    "FASTENER": (170, 60, 255), "OTHER": (200, 200, 200),
}


class LiveDetector(Node):
    def __init__(self):
        super().__init__('sih_live_detector')
        self.declare_parameter('out_dir', '/tmp/sih_detection')
        self.declare_parameter('plan', '')
        self.declare_parameter('model', 'v2')
        self.declare_parameter('score_thresh', 0.65)
        self.declare_parameter('torch_threads', 4)
        # demo throttle: run inference at most this often (wall clock); 0 = as fast as it can. Frames
        # that arrive sooner are dropped, never queued, exactly like frames that arrive while busy.
        self.declare_parameter('max_hz', 0.0)
        self.declare_parameter('home_world_z', 0.24)
        self.declare_parameter('origin_lat_deg', 47.397971057728974)
        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.model_name = self.get_parameter('model').value
        self.score_thresh = float(self.get_parameter('score_thresh').value)
        mh = float(self.get_parameter('max_hz').value)
        self.min_period = 1.0 / mh if mh > 0 else 0.0
        self.last_infer_wall = 0.0
        self.frames_throttled = 0

        # ---- geodesy / frame: identical formula to mission_follower_node.py,
        # duplicated (not imported) so this node has no runtime dependency on
        # the follower process; both derive from the same physical constants
        # and the plan's declared home.
        plan_path = self.get_parameter('plan').value
        home = [0.0, 0.0, 0.0]
        if plan_path and os.path.exists(plan_path):
            hg = json.load(open(plan_path))['home_ground_world_m']
            home = [hg[0], hg[1], self.get_parameter('home_world_z').value]
        self.home = np.array(home)
        lat = math.radians(self.get_parameter('origin_lat_deg').value)
        a_, f_ = 6378137.0, 1 / 298.257223563
        e2 = f_ * (2 - f_)
        s_ = 1 - e2 * math.sin(lat) ** 2
        N_, M_ = a_ / math.sqrt(s_), a_ * (1 - e2) / s_ ** 1.5
        R_ = 6371000.0
        self.k_east, self.k_north = R_ / N_, R_ / M_

        self.pos = self.att = None
        be_topics = ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position_v1')
        att_topics = ('/fmu/out/vehicle_attitude', '/fmu/out/vehicle_attitude_v1')
        from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
        for t in be_topics:
            self.create_subscription(VehicleLocalPosition, t, self.on_pos, qos_profile_sensor_data)
        for t in att_topics:
            self.create_subscription(VehicleAttitude, t, self.on_att, qos_profile_sensor_data)

        self.current_wp = 'PRE_MISSION'
        self.create_subscription(String, '/mission/event', self.on_event, 50)

        self.latest = None       # (stamp_sim_s, np.ndarray HxWx3 uint8)
        self.lock = threading.Lock()
        self.create_subscription(Image, '/camera/image_raw', self.on_img, qos_profile_sensor_data)

        self.ann_pub = self.create_publisher(Image, '/detection/image_annotated', 10)
        # Additive, visualisation-only: one JSON message per processed frame that
        # produced at least one detection (the detector's own output, unchanged).
        # Nothing in the mission subscribes to it; the RViz marker node does.
        self.det_pub = self.create_publisher(String, '/detection/detections', 10)
        self.ff = None
        self.detections = []
        self.frames_processed = 0
        self.infer_times = []
        self.t_node_start = time.time()
        self.last_processed_stamp = None

        self._load_model()
        self.create_timer(0.05, self.tick)   # poll for a new frame at 20 Hz; only infers when one exists
        self.create_timer(5.0, self.write)
        self.get_logger().info(f'live detector up: model {self.model_name} @ {self.score_thresh}, '
                               f'torch_threads {self.get_parameter("torch_threads").value}')

    # ---------------- model ----------------
    def _load_model(self):
        import torch
        torch.set_num_threads(int(self.get_parameter('torch_threads').value))
        from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        self.torch = torch
        labels = json.load(open(os.path.join(DATASET_DIR, 'labels_final.json')))
        self.families = labels['family_order']
        self.id_to_family = {i + 1: fam for i, fam in enumerate(self.families)}
        num_classes = len(self.families) + 1
        m = fasterrcnn_mobilenet_v3_large_fpn(weights=None, min_size=480, max_size=640)
        in_f = m.roi_heads.box_predictor.cls_score.in_features
        m.roi_heads.box_predictor = FastRCNNPredictor(in_f, num_classes)
        weights_path = os.path.join(DET_DIR, WEIGHTS_BY_NAME[self.model_name])
        state = torch.load(weights_path, map_location='cpu')
        m.load_state_dict(state)
        m.eval()
        self.model = m
        self.weights_path = weights_path

    # ---------------- inputs ----------------
    def on_pos(self, m):
        self.pos = m

    def on_att(self, m):
        self.att = m

    def on_event(self, msg):
        d = json.loads(msg.data)
        if d.get('type') == 'waypoint':
            self.current_wp = d['waypoint_id']
        elif d.get('type') == 'mission_start':
            self.current_wp = 'TAKEOFF'

    def on_img(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        w, h = msg.width, msg.height
        data = bytes(msg.data)
        if msg.step != w * 3:
            data = b''.join(data[r * msg.step:r * msg.step + w * 3] for r in range(h))
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == 'bgr8':
            arr = arr[:, :, ::-1]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.latest = (stamp, arr.copy())

    # ---------------- world pose (PX4 EKF -> world; same formula as the
    # follower, but this is a separate reader: detection logging never
    # writes back into navigation) ----------------
    def world_pose(self):
        if self.pos is None or self.att is None:
            return None, None
        n = np.array([self.pos.x, self.pos.y, self.pos.z])
        w = np.array([n[1] / self.k_east + self.home[0], n[0] / self.k_north + self.home[1], -n[2] + self.home[2]])
        yaw_ned = self.pos.heading
        yaw_enu = (math.pi / 2 - yaw_ned + math.pi) % (2 * math.pi) - math.pi
        return w, yaw_enu

    # ---------------- inference ----------------
    def tick(self):
        with self.lock:
            item = self.latest
            self.latest = None
        if item is None:
            return
        stamp, arr = item
        if self.last_processed_stamp is not None and stamp == self.last_processed_stamp:
            return
        if self.min_period and time.time() - self.last_infer_wall < self.min_period:
            self.frames_throttled += 1
            return
        self.last_infer_wall = time.time()
        self.last_processed_stamp = stamp
        t0 = time.time()
        tensor = self.torch.from_numpy(arr).permute(2, 0, 1).float() / 255.0
        with self.torch.no_grad():
            pred = self.model([tensor])[0]
        dt = time.time() - t0
        self.infer_times.append(dt)
        self.frames_processed += 1
        pose, yaw = self.world_pose()
        boxes = []
        for box, label, score in zip(pred['boxes'], pred['labels'], pred['scores']):
            if float(score) < self.score_thresh:
                continue
            fam = self.id_to_family.get(int(label))
            if fam is None:
                continue
            bbox = [round(float(v), 2) for v in box.tolist()]
            boxes.append((fam, float(score), bbox))
            self.detections.append({
                'waypoint_id': self.current_wp, 'sim_time_s': round(stamp, 3),
                'frame_id': self.frames_processed, 'class': fam, 'confidence': round(float(score), 4),
                'bbox_xyxy_px': bbox,
                'drone_pose_ekf_world_m': None if pose is None else [round(float(v), 4) for v in pose],
                'drone_yaw_ekf_enu_rad': None if yaw is None else round(float(yaw), 4),
            })
        self.publish_and_record(arr, boxes, stamp)
        if boxes:
            try:
                self.det_pub.publish(String(data=json.dumps({
                    'stamp_sim_s': stamp, 'frame_id': self.frames_processed, 'waypoint_id': self.current_wp,
                    'detections': [{'family': fam, 'score': round(sc, 4), 'bbox_xyxy_px': bb}
                                   for fam, sc, bb in boxes],
                    'drone_pose_ekf_world_m': None if pose is None else [round(float(v), 4) for v in pose],
                    'drone_yaw_ekf_enu_rad': None if yaw is None else round(float(yaw), 4)})))
            except Exception as e:      # never let the viz feed disturb detection
                self.get_logger().warn(f'detection feed publish failed: {e}')
        if self.frames_processed % 20 == 0:
            hz = self.frames_processed / max(sum(self.infer_times), 1e-6)
            self.get_logger().info(f'{self.frames_processed} frames, {len(self.detections)} detections, '
                                   f'mean inference {sum(self.infer_times) / len(self.infer_times):.3f} s '
                                   f'({hz:.2f} Hz achieved)')

    def publish_and_record(self, arr, boxes, stamp):
        from PIL import Image as PILImage, ImageDraw
        img = PILImage.fromarray(arr)
        if boxes:
            d = ImageDraw.Draw(img)
            for fam, score, (x0, y0, x1, y1) in boxes:
                col = FAMILY_COLOR.get(fam, (255, 255, 255))
                d.rectangle([x0, y0, x1, y1], outline=col, width=2)
                d.text((x0 + 2, max(0, y0 - 12)), f'{fam} {score:.2f}', fill=col)
        out = np.array(img)
        if self.ff is None:
            self.h, self.w = out.shape[0], out.shape[1]
            self.ff = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                 '-s', f'{self.w}x{self.h}', '-r', '5', '-i', '-',
                 '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
                 os.path.join(self.out_dir, 'annotated.mp4')], stdin=subprocess.PIPE)
        try:
            self.ff.stdin.write(out.tobytes())
        except BrokenPipeError:
            pass
        m = Image()
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        m.height, m.width = out.shape[0], out.shape[1]
        m.encoding = 'rgb8'
        m.step = out.shape[1] * 3
        m.data = out.tobytes()
        self.ann_pub.publish(m)

    # ---------------- output ----------------
    def write(self):
        json.dump({'model': self.model_name, 'weights': os.path.relpath(self.weights_path, ROOT),
                   'score_thresh': self.score_thresh, 'n_detections': len(self.detections),
                   'detections': self.detections}, open(os.path.join(self.out_dir, 'detections.json'), 'w'), indent=1)
        n = len(self.infer_times)
        stats = {
            'frames_processed': self.frames_processed,
            'max_hz_throttle': None if not self.min_period else round(1.0 / self.min_period, 3),
            'frames_dropped_by_throttle': self.frames_throttled,
            'n_detections': len(self.detections),
            'mean_inference_s': round(sum(self.infer_times) / n, 4) if n else None,
            'p95_inference_s': round(float(np.percentile(self.infer_times, 95)), 4) if n else None,
            'achieved_hz': round(n / sum(self.infer_times), 3) if n and sum(self.infer_times) > 0 else None,
            'wall_elapsed_s': round(time.time() - self.t_node_start, 1),
            'note': 'achieved_hz is 1/mean(per-frame inference wall time), measured on this run on this '
                    'CPU-only machine while Gazebo/PX4 were also running; this node always processes the '
                    'newest available camera frame and drops any that arrived while busy, so it never queues '
                    'stale frames -- the reported Hz is the true sustained detection rate, not the raw camera rate.',
        }
        json.dump(stats, open(os.path.join(self.out_dir, 'detection_stats.json'), 'w'), indent=1)

    def close(self):
        self.write()
        if self.ff:
            try:
                self.ff.stdin.close()
                self.ff.wait(timeout=30)
            except Exception:
                pass


def main():
    rclpy.init()
    n = LiveDetector()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())

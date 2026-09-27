#!/usr/bin/env python3
"""Demo display only: draw the live detector's real boxes on the natural WIDE camera view.

The detector runs on the 16 deg inspect_camera (launch_mission.sh default, AVIAN_DETECT_CAM=narrow)
because that is what makes it fire at all (14/15 frames vs 0/6 on the wide camera -- see
detection_eval/PHASE1_VERDICT.md). But a 16 deg view makes a poor demo window: it shows an
unrecognisable patch of concrete with no context. This node does NOT change the detector, the
camera that feeds it, the flight, or the aiming -- it only redraws, for DISPLAY, the same real
boxes on the wide 80 deg feed that a person watching the demo actually wants to see.

Geometry (proven exact, not approximate; see detection_eval/verify_wide_reprojection.py):
front_camera (80 deg) and inspect_camera (16 deg) sit at the IDENTICAL pose in gimbal_cam_link
(x500_base_inspect_camera.patch: both "0.03 0 0 0 0 0"). Zero baseline means a narrow-camera
pixel's back-projected ray is valid for the wide camera too -- only the focal length differs, so
narrow pixel (x, y) -> wide pixel is an exact scale-about-centre transform:
    x' = CX + (FX_wide / FX_narrow) * (x - CX)   [same for y]
verify_wide_reprojection.py checks this closed form against the full ray-based method used
elsewhere in this codebase (eval_detection_gazebo.py, measure_orbit_aim.py): 0.000000 px
difference. It also projects a real ground-truth defect directly into the wide camera at the
true recorded pose and shows the reprojected detector box lands on it (frames_aim_after/
wide_reprojection_check.png).

If AVIAN_DETECT_CAM=wide (detector already on the 80 deg camera -- the pre-27-Sep setup), the
boxes are already in wide-pixel space and no reprojection is applied.

Subscribes: /camera/image_raw (wide, always bridged), /detection/detections (the detector's own
per-frame output, unchanged, in the detector's native pixel space).
Publishes: /detection/image_annotated_wide -- a natural wide view, continuous at the wide
camera's frame rate, with the real narrow-camera detections reprojected onto it while they are
current (held for --hold-s so a brief narrow-camera hit is still visible), and a caption stating
the boxes are reprojected.
"""
import json
import math

import numpy as np
import rclpy
from PIL import Image as PILImage, ImageDraw
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

FAMILY_COLOR = {
    "CRACK": (255, 60, 60), "SPALL_DELAM": (255, 170, 30), "CORROSION_COATING": (60, 170, 255),
    "FASTENER": (170, 60, 255), "OTHER": (200, 200, 200),
}
CAM_W, CAM_H = 640, 480
CX, CY = CAM_W / 2, CAM_H / 2


def fx(hfov_rad):
    return (CAM_W / 2) / math.tan(hfov_rad / 2)


class WideBoxReproject(Node):
    def __init__(self):
        super().__init__('sih_wide_box_reproject')
        self.declare_parameter('image_topic_wide', '/camera/image_raw')
        self.declare_parameter('detect_cam', 'narrow')          # mirrors launch_mission.sh's AVIAN_DETECT_CAM
        self.declare_parameter('narrow_hfov_rad', 0.2792527)    # inspect_camera, 16 deg
        self.declare_parameter('wide_hfov_rad', 1.3962634)      # front_camera, 80 deg
        self.declare_parameter('hold_s', 0.35)                  # keep a detection visible this long after it arrives
        detect_cam = self.get_parameter('detect_cam').value
        fx_wide = fx(float(self.get_parameter('wide_hfov_rad').value))
        fx_narrow = fx(float(self.get_parameter('narrow_hfov_rad').value))
        # scale=1 (no-op) when the detector is already on the wide camera: its boxes are already wide-scale.
        self.scale = 1.0 if detect_cam == 'wide' else fx_wide / fx_narrow
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.caption = ('boxes: 16 deg inspect camera, reprojected to this view -- not run on this frame'
                        if detect_cam != 'wide' else 'boxes: detector ran directly on this camera')

        self.last_boxes = None   # [(family, score, (x0,y0,x1,y1) in WIDE pixels)]
        self.last_stamp = None

        self.create_subscription(Image, self.get_parameter('image_topic_wide').value, self.on_wide, qos_profile_sensor_data)
        self.create_subscription(String, '/detection/detections', self.on_det, 10)
        self.pub = self.create_publisher(Image, '/detection/image_annotated_wide', 10)
        self.get_logger().info(f'wide box reprojection up: detect_cam={detect_cam}, scale={self.scale:.4f} '
                               '(narrow-box-pixels -> wide-display-pixels)')

    def reproject(self, x0, y0, x1, y1):
        s = self.scale
        return (CX + s * (x0 - CX), CY + s * (y0 - CY), CX + s * (x1 - CX), CY + s * (y1 - CY))

    def on_det(self, msg):
        d = json.loads(msg.data)
        self.last_stamp = float(d['stamp_sim_s'])
        self.last_boxes = [(det['family'], det['score'], self.reproject(*det['bbox_xyxy_px']))
                            for det in d['detections']]

    def on_wide(self, msg):
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

        img = PILImage.fromarray(arr.copy())
        draw = ImageDraw.Draw(img)
        show_boxes = (self.last_boxes and self.last_stamp is not None
                      and abs(stamp - self.last_stamp) <= self.hold_s)
        if show_boxes:
            for fam, score, (x0, y0, x1, y1) in self.last_boxes:
                col = FAMILY_COLOR.get(fam, (255, 255, 255))
                draw.rectangle([x0, y0, x1, y1], outline=col, width=2)
                draw.text((x0 + 2, max(0, y0 - 12)), f'{fam} {score:.2f}', fill=col)
        draw.rectangle([0, h - 14, w, h], fill=(0, 0, 0))
        draw.text((4, h - 13), self.caption, fill=(255, 255, 255))
        out = np.array(img)

        m = Image()
        m.header = msg.header
        m.height, m.width = out.shape[0], out.shape[1]
        m.encoding = 'rgb8'
        m.step = out.shape[1] * 3
        m.data = out.tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    n = WideBoxReproject()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

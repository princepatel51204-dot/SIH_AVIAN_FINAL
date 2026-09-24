#!/usr/bin/env python3
"""Gate 2: count REAL Gazebo contact-sensor events on the drone's main body.

Subscribes to /body_contact (ros_gz_interfaces/msg/Contacts), bridged from
the gz contact sensor added to x500_base's main-body collision (see
PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf,
"body_contact_sensor"). This is real Gazebo physics-engine contact
detection, not a LiDAR-range proxy.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from ros_gz_interfaces.msg import Contacts


class ContactCounter(Node):
    def __init__(self, out_path):
        super().__init__('gate2_contact_counter')
        self.out_path = out_path
        self.samples_with_contact = 0
        self.episodes = 0
        self.was_in_contact = False
        self.events = []
        self.t0 = time.time()
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Contacts, '/body_contact', self.on_contacts, qos)
        self.create_timer(5.0, self.checkpoint)
        self.get_logger().info('gate2 contact counter up, watching /body_contact')

    def on_contacts(self, msg):
        in_contact = len(msg.contacts) > 0
        if in_contact:
            self.samples_with_contact += 1
            if not self.was_in_contact:
                self.episodes += 1
                names = sorted({c.collision2.name for c in msg.contacts})
                self.events.append({'t': round(time.time() - self.t0, 2), 'with': names})
                self.get_logger().warn('CONTACT #%d with %s' % (self.episodes, names))
        self.was_in_contact = in_contact

    def checkpoint(self):
        self.write()

    def write(self):
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)
        with open(self.out_path, 'w') as f:
            json.dump({
                'contact_episodes': self.episodes,
                'contact_samples': self.samples_with_contact,
                'events': self.events,
                'elapsed_s': round(time.time() - self.t0, 1),
            }, f, indent=2)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/GarudaNEX/results/gate2_sih/contact_summary.json')
    rclpy.init()
    node = ContactCounter(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

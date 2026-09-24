#!/usr/bin/env python3
"""Gate 2: arm + switch to OFFBOARD. garudanex_cmd_vel_bridge must already be
running and streaming OffboardControlMode+TrajectorySetpoint (it starts
doing so as soon as it comes up, regardless of arm state) -- this script
only flips the two mission-level switches, per cmd_vel_bridge.cpp's own
docstring ("does NOT arm ... those are mission decisions").
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def main():
    rclpy.init()
    node = Node('gate2_arm')
    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST)
    status = {}
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                             lambda m: status.update(arming=m.arming_state, nav=m.nav_state), qos)
    pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

    def send(cmd, **kw):
        m = VehicleCommand()
        m.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        m.command = cmd
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 255, 1
        m.from_external = True
        for k, v in kw.items():
            setattr(m, k, v)
        pub.publish(m)

    def spin(s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(node, timeout_sec=0.1)

    print('waiting for vehicle_status ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 30 and not status:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not status:
        print('FAIL: no vehicle_status', flush=True)
        return 1
    print('status:', status, flush=True)

    print('requesting OFFBOARD', flush=True)
    send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
    spin(1.0)
    print('nav_state:', status.get('nav'), flush=True)

    print('arming (retry up to 20s)', flush=True)
    armed = False
    t0 = time.time()
    while time.time() - t0 < 20:
        send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        spin(1.0)
        if status.get('arming') == VehicleStatus.ARMING_STATE_ARMED:
            armed = True
            break
    print('armed:', armed, 'status:', status, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if armed else 1


if __name__ == '__main__':
    sys.exit(main())

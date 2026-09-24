#!/usr/bin/env python3
"""Gate 2: land. garudanex_cmd_vel_bridge reads cruise_altitude only once at
construction (no live-parameter-update callback), so it can't be told to
descend at runtime. Kill it and take over with the same direct
OffboardControlMode + TrajectorySetpoint publisher proven in Gate 1, then
let PX4's land-detector auto-disarm once the setpoint stream stops (an
explicit disarm command while still commanding is refused).
"""
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint

MAX_Z_VEL = 1.0
ALT_KP = 1.0


def main():
    print('killing garudanex_cmd_vel_bridge (it cannot be told to descend at runtime)', flush=True)
    subprocess.run(['pkill', '-f', 'cmd_vel_bridge_node'], check=False)
    time.sleep(1.0)

    rclpy.init()
    node = Node('gate2_land')
    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST)
    st = {}
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                             lambda m: st.update(arming=m.arming_state), qos)
    node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                             lambda m: st.update(z=m.z, vz=m.vz), qos)
    ocm_pub = node.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
    sp_pub = node.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

    timer_active = {'on': True}

    def tick():
        if not timer_active['on']:
            return
        ocm = OffboardControlMode()
        ocm.position = False
        ocm.velocity = True
        ocm.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        ocm_pub.publish(ocm)
        z = st.get('z', 0.0)
        current_alt = -z
        vz_ned = -max(-MAX_Z_VEL, min(MAX_Z_VEL, ALT_KP * (0.0 - current_alt)))
        sp = TrajectorySetpoint()
        sp.position = [float('nan')] * 3
        sp.velocity = [0.0, 0.0, vz_ned]
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yaw = float('nan')
        sp.yawspeed = 0.0
        sp.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        sp_pub.publish(sp)

    node.create_timer(1.0 / 20.0, tick)

    def spin(s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(node, timeout_sec=0.05)

    print('descending (offboard velocity -> alt 0)', flush=True)
    t0 = time.time()
    grounded = False
    while time.time() - t0 < 60:
        spin(1.0)
        alt = -st.get('z', 999.0)
        vz = st.get('vz', 1.0)
        print('  alt=%.2f vz=%.2f' % (alt, vz), flush=True)
        if alt <= 0.08 and abs(vz) < 0.08:
            grounded = True
            break

    print('grounded -- stopping setpoint stream, waiting for auto-disarm', flush=True)
    timer_active['on'] = False

    landed = False
    t0 = time.time()
    while time.time() - t0 < 15:
        spin(1.0)
        if st.get('arming') == VehicleStatus.ARMING_STATE_DISARMED:
            landed = True
            break
    print('grounded=%s landed(disarmed)=%s' % (grounded, landed), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if landed else 1


if __name__ == '__main__':
    sys.exit(main())

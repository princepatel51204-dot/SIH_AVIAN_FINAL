#!/usr/bin/env python3
"""Gate 1 via PX4 OFFBOARD velocity control (the path GarudaNEX's own
cmd_vel_bridge.cpp uses and that is proven to actually produce thrust in
this stack) -- arm, climb to CRUISE_ALT, hover 10s, descend, disarm.
Logs achieved pose throughout to gate1_pose_log.json.
"""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import (VehicleCommand, VehicleStatus, VehicleLocalPosition,
                          OffboardControlMode, TrajectorySetpoint)

OUT = "/tmp/claude-1000/-home-prince-avian-rev-c-SIH-AVIAN-FINAL/5c8a8386-c605-4a1a-8e10-6688e2e63e62/scratchpad/gazebo_gate1/gate1_pose_log.json"
CRUISE_ALT = 3.0      # m AGL
HOVER_S = 10.0
ALT_KP = 1.0
MAX_Z_VEL = 1.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class Gate1(Node):
    def __init__(self):
        super().__init__('gate1_offboard')
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.status = None
        self.pos = None
        self.samples = []
        self.target_alt = 0.0  # commanded via velocity P-control toward this AGL alt
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_pos, qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.setpoint_timer = self.create_timer(1.0 / 20.0, self.tick)

    def on_status(self, msg):
        self.status = msg

    def on_pos(self, msg):
        self.pos = msg
        self.samples.append({
            't_wall': time.time(), 't_us': msg.timestamp,
            'x': msg.x, 'y': msg.y, 'z': msg.z,
            'vx': msg.vx, 'vy': msg.vy, 'vz': msg.vz,
            'heading': msg.heading,
        })

    def send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.param1, m.param2, m.param3, m.param4 = p1, p2, p3, p4
        m.param5, m.param6, m.param7 = p5, p6, p7
        m.command = command
        m.target_system = 1
        m.target_component = 1
        m.source_system = 255
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def tick(self):
        ocm = OffboardControlMode()
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.ocm_pub.publish(ocm)

        if self.pos is None:
            return
        current_alt = -self.pos.z
        alt_err = self.target_alt - current_alt
        vz_ned = -max(-MAX_Z_VEL, min(MAX_Z_VEL, ALT_KP * alt_err))

        sp = TrajectorySetpoint()
        sp.position = [float('nan')] * 3
        sp.velocity = [0.0, 0.0, vz_ned]
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yaw = float('nan')
        sp.yawspeed = 0.0
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(sp)


def spin_for(node, seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = Gate1()
    events = []

    def note(msg):
        t = time.time()
        print(f"[{t:.2f}] {msg}", flush=True)
        events.append({"t": t, "msg": msg})

    note("waiting for vehicle_status + local_position")
    t0 = time.time()
    while time.time() - t0 < 30 and (node.status is None or node.pos is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.status is None or node.pos is None:
        note("FAIL: no status/position")
        json.dump({"events": events, "pose_samples": node.samples}, open(OUT, "w"), indent=2)
        return 1
    note(f"got status (arming={node.status.arming_state}, nav={node.status.nav_state}), pos z={node.pos.z:.3f}")

    # Stream setpoints for >1s before requesting OFFBOARD -- PX4 requires a
    # live setpoint stream before it will accept the mode switch.
    note("streaming offboard setpoints for 2s warm-up (target_alt=0, i.e. hold)")
    spin_for(node, 2.0)

    note("requesting OFFBOARD mode")
    node.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
    spin_for(node, 1.0)
    note(f"nav_state after mode request: {node.status.nav_state if node.status else None}")

    note("arming (retry up to 20s)")
    armed = False
    t0 = time.time()
    while time.time() - t0 < 20:
        node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        spin_for(node, 1.0)
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            armed = True
            break
    note(f"armed={armed} nav_state={node.status.nav_state if node.status else None}")

    if not armed:
        json.dump({"events": events, "pose_samples": node.samples}, open(OUT, "w"), indent=2)
        print("GATE1_RESULT", json.dumps({"armed": False}))
        return 1

    note(f"climbing to {CRUISE_ALT} m AGL via offboard velocity")
    node.target_alt = CRUISE_ALT
    t0 = time.time()
    reached = False
    while time.time() - t0 < 30:
        spin_for(node, 1.0)
        alt = -node.pos.z if node.pos else 0.0
        note(f"  alt={alt:.2f}")
        if alt >= CRUISE_ALT * 0.85:
            reached = True
            break
    note(f"reached_altitude={reached} alt={-node.pos.z if node.pos else None:.2f}")

    note(f"hovering {HOVER_S}s at target_alt={CRUISE_ALT}")
    spin_for(node, HOVER_S)
    note(f"hover end alt={-node.pos.z if node.pos else None:.2f}")

    note("descending to land (target_alt=0 via offboard velocity)")
    node.target_alt = 0.0
    t0 = time.time()
    grounded = False
    while time.time() - t0 < 30:
        spin_for(node, 1.0)
        alt = -node.pos.z if node.pos else 999.0
        note(f"  alt={alt:.2f} vz={node.pos.vz if node.pos else None:.2f}")
        if alt <= 0.08 and abs(node.pos.vz) < 0.08:
            grounded = True
            break

    note("stopping offboard setpoint stream so the land-detector can confirm "
         "landed (a continued velocity stream, even near-zero, appears to "
         "suppress it and PX4 then refuses a commanded disarm)")
    node.setpoint_timer.cancel()

    note("waiting up to 15s for PX4's own COM_DISARM_LAND auto-disarm")
    landed = False
    t0 = time.time()
    while time.time() - t0 < 15:
        spin_for(node, 1.0)
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            landed = True
            break
    if not landed:
        note("auto-disarm did not fire; trying an explicit disarm command")
        t0 = time.time()
        while time.time() - t0 < 10:
            node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)
            spin_for(node, 1.0)
            if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                landed = True
                break
    note(f"grounded={grounded} landed(disarmed)={landed}")

    json.dump({"events": events, "pose_samples": node.samples}, open(OUT, "w"), indent=2)
    note(f"wrote {OUT} ({len(node.samples)} pose samples)")
    print("GATE1_RESULT", json.dumps({
        "armed": armed, "reached_altitude": reached, "grounded": grounded,
        "landed": landed, "n_pose_samples": len(node.samples),
    }))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

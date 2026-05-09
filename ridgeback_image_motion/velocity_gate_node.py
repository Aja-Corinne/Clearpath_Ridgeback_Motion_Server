#!/usr/bin/env python3
"""Jetson velocity gate — clamps and forwards cmd_vel_raw to the robot.

Every motion command from the PC flows through this node before reaching
the Ridgeback motors. Running on the Jetson (Ethernet-connected) means
hardware-level clamping is always active regardless of PC software state.

Topic flow:
  PC / autonomy → /cmd_vel_raw → [this node, clamps] → /r100_0140/cmd_vel → motors
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MAX_LINEAR = 0.5   # m/s
MAX_ANGULAR = 0.5  # rad/s


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class VelocityGate(Node):
    def __init__(self) -> None:
        super().__init__("velocity_gate")

        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/r100_0140/cmd_vel")
        self.declare_parameter("max_linear", MAX_LINEAR)
        self.declare_parameter("max_angular", MAX_ANGULAR)

        inp = self.get_parameter("input_topic").value
        out = self.get_parameter("output_topic").value
        self._max_lin = float(self.get_parameter("max_linear").value)
        self._max_ang = float(self.get_parameter("max_angular").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(Twist, out, qos)
        self.create_subscription(Twist, inp, self._cb, qos)

        self.get_logger().info(
            f"Velocity gate ready: {inp} → {out} "
            f"(max_linear={self._max_lin} m/s, max_angular={self._max_ang} rad/s)"
        )

    def _cb(self, msg: Twist) -> None:
        clamped = Twist()
        clamped.linear.x = _clamp(msg.linear.x, self._max_lin)
        clamped.linear.y = _clamp(msg.linear.y, self._max_lin)
        clamped.linear.z = 0.0
        clamped.angular.z = _clamp(msg.angular.z, self._max_ang)
        self._pub.publish(clamped)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Velocity gate shutting down. Sending final stop.")
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

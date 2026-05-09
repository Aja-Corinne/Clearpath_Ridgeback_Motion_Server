#!/usr/bin/env python3
"""Re-publishes LaserScan with a corrected frame_id.

In Ignition Fortress the bridge sets frame_id to the Gazebo-scoped sensor
name (e.g. ridgeback/base_link/lidar_sensor) which has no entry in the TF
tree.  This node replaces that with base_link so SLAM and Nav2 can process
the scan without any extra static TF plumbing.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.declare_parameter('input_topic', '/r100_0140/sensors/lidar2d_0/scan_bridge')
        self.declare_parameter('output_topic', '/r100_0140/sensors/lidar2d_0/scan')
        self.declare_parameter('frame_id', 'base_link')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._frame = self.get_parameter('frame_id').value
        self._pub = self.create_publisher(LaserScan, out, 10)
        self.create_subscription(LaserScan, inp, self._cb, 10)
        self.get_logger().info(f'scan_relay: {inp} → {out} (frame_id={self._frame})')

    def _cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

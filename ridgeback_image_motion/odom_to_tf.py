#!/usr/bin/env python3
"""Republishes nav_msgs/Odometry as a tf2 odom → base_link TransformStamped.

Used in simulation where the Ridgeback's hardware EKF is absent.
The Gazebo DiffDrive plugin publishes /odom (bridged to the autonomy stack);
this node mirrors it as a TF transform so SLAM Toolbox and Nav2 costmaps
can resolve base_link in the odom frame.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        topic = self.get_parameter('odom_topic').value
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self._cb, 10)
        self.get_logger().info(f'odom_to_tf ready: {topic} → /tf (odom → base_link)')

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTf())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

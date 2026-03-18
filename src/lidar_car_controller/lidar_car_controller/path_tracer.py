#!/usr/bin/env python3
"""
path_tracer.py
Subscribes to /odom and publishes a growing nav_msgs/Path
so RViz2 can draw the robot's travelled route.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathTracer(Node):
    MAX_POSES = 2000   # cap path length to avoid memory growth

    def __init__(self):
        super().__init__('path_tracer')
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.odom_sub  = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_pub  = self.create_publisher(Path, '/odom_path', 10)
        self.get_logger().info('PathTracer node started ✔')

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose

        self.path.poses.append(pose)
        if len(self.path.poses) > self.MAX_POSES:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
obstacle_detector.py
Subscribes to /scan (LaserScan) and publishes obstacle zones
to /obstacle_info as a String for the controller to act on.
Zones: FRONT, LEFT, RIGHT, CLEAR
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class ObstacleDetector(Node):

    # Distance threshold in metres — obstacle detected if closer than this
    OBSTACLE_THRESHOLD = 0.6

    # Angular windows (degrees) for each zone
    # LiDAR angle 0 = forward; positive = left (ROS convention)
    FRONT_ANGLE  = 30   # ±30° in front
    LEFT_ANGLE   = 90   # 30°–120° to the left
    RIGHT_ANGLE  = 90   # 30°–120° to the right

    def __init__(self):
        super().__init__('obstacle_detector')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.obstacle_pub = self.create_publisher(
            String, '/obstacle_info', 10)

        self.get_logger().info('ObstacleDetector node started ✔')

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        ranges      = msg.ranges
        angle_min   = msg.angle_min          # radians
        angle_inc   = msg.angle_increment    # radians per index

        front_hit  = False
        left_hit   = False
        right_hit  = False

        for i, r in enumerate(ranges):
            # Skip invalid readings
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            angle_rad = angle_min + i * angle_inc
            angle_deg = math.degrees(angle_rad)

            # Normalise to [-180, 180]
            angle_deg = (angle_deg + 180) % 360 - 180

            if r < self.OBSTACLE_THRESHOLD:
                if -self.FRONT_ANGLE <= angle_deg <= self.FRONT_ANGLE:
                    front_hit = True
                elif self.FRONT_ANGLE < angle_deg <= (self.FRONT_ANGLE + self.LEFT_ANGLE):
                    left_hit = True
                elif -(self.FRONT_ANGLE + self.RIGHT_ANGLE) <= angle_deg < -self.FRONT_ANGLE:
                    right_hit = True

        # Build status string
        parts = []
        if front_hit: parts.append('FRONT')
        if left_hit:  parts.append('LEFT')
        if right_hit: parts.append('RIGHT')
        status = ','.join(parts) if parts else 'CLEAR'

        out = String()
        out.data = status
        self.obstacle_pub.publish(out)

        self.get_logger().info(f'Obstacle zones: {status}', throttle_duration_sec=1.0)


# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
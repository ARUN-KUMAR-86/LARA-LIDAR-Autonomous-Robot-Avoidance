#!/usr/bin/env python3
"""
car_controller.py
Reads /obstacle_info and drives the robot:
  CLEAR        → move forward
  FRONT        → stop, turn right
  FRONT+LEFT   → turn right
  FRONT+RIGHT  → turn left
  FRONT+LEFT+RIGHT → reverse + turn
  LEFT only    → slight right correction
  RIGHT only   → slight left correction
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CarController(Node):

    # Velocities
    LINEAR_SPEED   = 0.25   # m/s  forward
    ANGULAR_SPEED  = 0.8    # rad/s turning
    REVERSE_SPEED  = -0.15  # m/s  reversing

    def __init__(self):
        super().__init__('car_controller')

        self.obstacle_sub = self.create_subscription(
            String, '/obstacle_info', self.obstacle_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety timer: if no obstacle info arrives, stop the robot
        self.last_msg_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.5, self.safety_check)

        # Store latest obstacle status
        self.obstacle_status = 'CLEAR'

        # Drive timer at 10 Hz
        self.drive_timer = self.create_timer(0.1, self.drive)

        self.get_logger().info('CarController node started ✔')

    # ------------------------------------------------------------------
    def obstacle_callback(self, msg: String):
        self.obstacle_status  = msg.data
        self.last_msg_time    = self.get_clock().now()

    # ------------------------------------------------------------------
    def safety_check(self):
        """Stop if no scan data for 1 second."""
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.publish_twist(0.0, 0.0)
            self.get_logger().warn('No sensor data — robot stopped!', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def drive(self):
        zones = set(self.obstacle_status.split(','))

        front = 'FRONT'  in zones
        left  = 'LEFT'   in zones
        right = 'RIGHT'  in zones
        clear = 'CLEAR'  in zones or (not front and not left and not right)

        if clear:
            # Full speed ahead
            self.publish_twist(self.LINEAR_SPEED, 0.0)

        elif front and left and right:
            # Boxed in — reverse and spin
            self.publish_twist(self.REVERSE_SPEED, self.ANGULAR_SPEED)
            self.get_logger().warn('Boxed in — reversing!')

        elif front and left:
            # Blocked front + left → turn hard right
            self.publish_twist(0.0, -self.ANGULAR_SPEED)

        elif front and right:
            # Blocked front + right → turn hard left
            self.publish_twist(0.0, self.ANGULAR_SPEED)

        elif front:
            # Just front blocked → turn right (default)
            self.publish_twist(0.0, -self.ANGULAR_SPEED)

        elif left:
            # Only left blocked → gentle right steer while moving
            self.publish_twist(self.LINEAR_SPEED * 0.6, -self.ANGULAR_SPEED * 0.4)

        elif right:
            # Only right blocked → gentle left steer while moving
            self.publish_twist(self.LINEAR_SPEED * 0.6, self.ANGULAR_SPEED * 0.4)

    # ------------------------------------------------------------------
    def publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleTrajectory(Node):
    def __init__(self):
        super().__init__('circle_trajectory_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Parameters
        self.radius = 1.0  # meters
        self.omega = 0.5   # rad/s (angular velocity)
        self.linear_velocity = self.radius * self.omega

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.omega
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

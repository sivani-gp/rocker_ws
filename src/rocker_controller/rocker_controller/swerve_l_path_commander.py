#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SwerveLPathCommander(Node):
    def __init__(self):
        super().__init__('swerve_l_path_commander')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.state = 0
        self.counter = 0

    def timer_callback(self):
        msg = Twist()

        if self.state == 0:  # Forward along X
            msg.linear.x = 0.3
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.counter += 1
            if self.counter > 30:  # 3 seconds
                self.counter = 0
                self.state = 1

        elif self.state == 1:  # Move right along Y (L shape)
            msg.linear.x = 0.0
            msg.linear.y = -0.3  # Right strafe
            msg.angular.z = 0.0
            self.counter += 1
            if self.counter > 30:
                self.counter = 0
                self.state = 2

        elif self.state == 2:  # Stop
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('L-shaped path completed!')

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveLPathCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

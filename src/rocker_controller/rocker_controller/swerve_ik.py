#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveIKController(Node):
    def __init__(self):
        super().__init__('swerve_ik_controller')

        self.L = 0.5  # length (front to back)
        self.W = 0.4  # width (left to right)
        self.R = math.hypot(self.L, self.W)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/simple_velocity_controller/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/swerve_steering_controller/commands', 10)

    def compute_ik(self, linear_x, linear_y, angular_z):
        A = linear_x - angular_z * (self.W / self.R)
        B = linear_x + angular_z * (self.W / self.R)
        C = linear_y - angular_z * (self.L / self.R)
        D = linear_y + angular_z * (self.L / self.R)

        vel_fl = math.hypot(B, D)
        vel_fr = math.hypot(B, C)
        vel_rl = math.hypot(A, D)
        vel_rr = math.hypot(A, C)

        angle_fl = math.atan2(B, D)
        angle_fr = math.atan2(B, C)
        angle_rl = math.atan2(A, D)
        angle_rr = math.atan2(A, C)

        return [vel_fl, vel_fr, vel_rl, vel_rr], [angle_fl, angle_fr, angle_rl, angle_rr]

    def publish_commands(self, velocities, angles):
        velocity_msg = Float64MultiArray()
        steering_msg = Float64MultiArray()

        velocity_msg.data = velocities  # Order: fl, fr, rl, rr
        steering_msg.data = angles

        self.velocity_pub.publish(velocity_msg)
        self.steering_pub.publish(steering_msg)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        velocities, angles = self.compute_ik(linear_x, linear_y, angular_z)
        self.publish_commands(velocities, angles)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveIKController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import time

class SwerveDriveLShape(Node):
    def __init__(self):
        super().__init__('swerve_drive_l_shape')

        # Publishers for steering and velocity commands
        self.steering_pub = self.create_publisher(
            Float64MultiArray, 
            '/swerve_steering_controller/commands', 
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 
            '/simple_velocity_controller/commands', 
            10
        )

        # Robot-specific parameters (adjust these to match your robot)
        self.wheel_base = 0.4  # Distance between front and rear axles (meters)
        self.track_width = 0.22  # Distance between left and right wheels (meters)
        
        self.get_logger().info("Swerve Drive L-Shape Movement initialized")
        
        # Start the movement sequence
        self.timer = self.create_timer(0.1, self.execute_l_shape)

        # Movement sequence state
        self.sequence_step = 0
        self.last_step_time = self.get_clock().now().seconds_nanoseconds()[0]

    def normalize_angle(self, theta):
        """Normalize the angle to the range -π to π (-180 to 180 degrees)."""
        return math.atan2(math.sin(theta), math.cos(theta))

    def set_wheel_commands(self, angles, speeds):
        """Helper function to publish wheel commands"""
        # Publish steering commands (FL, FR, RL, RR order)
        steering_msg = Float64MultiArray()
        steering_msg.data = angles
        self.steering_pub.publish(steering_msg)
        
        # Publish velocity commands (FR, FL, RL, RR order - per your YAML)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [
            speeds[1],  # FR
            speeds[0],  # FL
            speeds[2],  # RL
            speeds[3]   # RR
        ]
        self.velocity_pub.publish(velocity_msg)

    def execute_l_shape(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        if self.sequence_step == 0:
            # Step 1: Move forward (all wheels at 0 angle)
            angles = [0.0, 0.0, 0.0, 0.0]
            speeds = [-1.0, -1.0, 1.0, 1.0]  # Forward speed
            self.set_wheel_commands(angles, speeds)
            
            # After 2 seconds, move to next step
            if current_time - self.last_step_time > 20.0:
                self.sequence_step = 1
                self.last_step_time = current_time
                self.get_logger().info("Moving to right turn")
                
        elif self.sequence_step == 1:
            # Step 2: Move right (all wheels at 90 degrees/1.57 radians)
            angles = [-1.57, -1.57, -1.57, -1.57]
            speeds = [1.0,1.0,1.0,1.0]  # Right speed
            self.set_wheel_commands(angles, speeds)
            
            # After 2 seconds, stop
            if current_time - self.last_step_time > 20.0:
                self.sequence_step = 2
                self.last_step_time = current_time
                self.get_logger().info("Stopping")
                
        elif self.sequence_step == 2:
            # Step 3: Stop all wheels
            angles = [0.0, 0.0, 0.0, 0.0]
            speeds = [0.0, 0.0, 0.0, 0.0]
            self.set_wheel_commands(angles, speeds)
            
            # After 1 second, shutdown
            if current_time - self.last_step_time > 1.0:
                self.get_logger().info("Sequence complete")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveLShape()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
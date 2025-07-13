#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveDriveController(Node):
    def __init__(self):
        super().__init__('swerve_drive_controller')

        # Subscribing to /cmd_vel for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

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
        
        self.get_logger().info("Swerve Drive Controller initialized")

    def normalize_angle(self, theta):
        """
        Normalize the angle to the range -π to π (-180 to 180 degrees).
        Returns normalized angle and whether speed should be reversed.
        """
        # Normalize to -π to π first
        theta = math.atan2(math.sin(theta), math.cos(theta))
        
        # Then check if we should reverse direction
        if abs(theta) > math.pi/2:
            return theta - math.copysign(math.pi, theta), True
        return theta, False

    def compute_wheel_velocities_and_angles(self, V_x, V_y, omega):
        """
        Compute wheel velocities and angles for swerve drive
        Returns dict with 'angles' and 'speeds' lists in FL, FR, RL, RR order
        """
        # Calculate wheel velocity components
        wheel_velocities = {
            'fl': (V_x - omega * (self.track_width/2), V_y + omega * (self.wheel_base/2)),
            'fr': (V_x + omega * (self.track_width/2), V_y + omega * (self.wheel_base/2)),
            'rl': (V_x - omega * (self.track_width/2), V_y - omega * (self.wheel_base/2)),
            'rr': (V_x + omega * (self.track_width/2), V_y - omega * (self.wheel_base/2)),
        }

        angles = []
        speeds = []

        for wheel in ['fl', 'fr', 'rl', 'rr']:
            v_x, v_y = wheel_velocities[wheel]
            
            # Calculate speed and angle
            speed = math.sqrt(v_x**2 + v_y**2)
            angle = math.atan2(v_y, v_x)
            
            # Normalize angle and potentially reverse speed
            angle, reverse = self.normalize_angle(angle)
            if reverse:
                speed *= -1
            
            angles.append(angle)
            speeds.append(speed)

        return {
            'angles': angles,  # Order: FL, FR, RL, RR
            'speeds': speeds   # Order: FL, FR, RL, RR
        }

    def cmd_vel_callback(self, msg):
        try:
            # Compute wheel commands
            wheel_commands = self.compute_wheel_velocities_and_angles(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z
            )
            
            # Publish steering commands (in steer_fl, steer_fr, steer_rl, steer_rr order)
            steering_msg = Float64MultiArray()
            steering_msg.data = wheel_commands['angles']
            self.steering_pub.publish(steering_msg)
            
            # Publish velocity commands (in fr, fl, rl, rr order - per your YAML)
            velocity_msg = Float64MultiArray()
            # Reorder speeds to match your YAML joint order: fr, fl, rl, rr
            velocity_msg.data = [
                wheel_commands['speeds'][1],  # FR
                wheel_commands['speeds'][0],  # FL
                wheel_commands['speeds'][2],  # RL
                wheel_commands['speeds'][3]   # RR
            ]
            self.velocity_pub.publish(velocity_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ContinuousSwerveCircle(Node):
    def __init__(self):
        super().__init__('continuous_swerve_circle')
        
        # Publishers
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/swerve_steering_controller/commands', 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/simple_velocity_controller/commands', 10)

        # Fixed parameters for continuous circle
        self.steer_angle = 0.785  # 45° in radians (π/4)
        self.wheel_speed = 1.0    # Speed for all wheels

        # Publish commands immediately and continuously
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.get_logger().info("Starting continuous circle movement")

    def publish_commands(self):
        # Publish fixed steering angles (all wheels at 45°)
        steer_msg = Float64MultiArray()
        steer_msg.data = [self.steer_angle] * 4  # All 4 wheels same angle
        self.steering_pub.publish(steer_msg)
        
        # Publish fixed speeds (all wheels same speed)
        speed_msg = Float64MultiArray()
        speed_msg.data = [self.wheel_speed] * 4  # All 4 wheels same speed
        self.velocity_pub.publish(speed_msg)

    def stop_robot(self):
        # Stop all wheels
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0] * 4
        self.steering_pub.publish(stop_msg)
        self.velocity_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped")

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousSwerveCircle()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
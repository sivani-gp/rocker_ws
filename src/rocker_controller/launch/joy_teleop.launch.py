from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # Update if your joystick is different
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }],
            output='screen'
        ),

        # Teleop twist joy node (converts joystick to Twist)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,      # Left stick vertical
                'axis_angular.yaw': 2,   # Right stick horizontal
                'scale_linear.x': 0.5,   # Max speed (m/s)
                'scale_angular.yaw': 0.8, # Max rotation (rad/s)
                'enable_button': 0        # Usually the A button
            }],
            output='screen'
        ),

        # Your swerve controller node
        Node(
            package='rocker_controller',
            executable='controller.py',
            name='swerve_controller',
            output='screen',
            parameters=[{
                'wheel_base': 0.4,      # Match your robot's dimensions
                'track_width': 0.35
            }]
        )
    ])
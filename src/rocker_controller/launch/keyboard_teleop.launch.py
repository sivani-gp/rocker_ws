from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your swerve controller node
        Node(
            package='rocker_controller',
            executable='controller.py',
            name='swerve_controller',
            output='screen',
            parameters=[{
                'wheel_base': 0.4,
                'track_width': 0.35
            }]
        ),
        
        # Keyboard teleop node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # Opens in new terminal window
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
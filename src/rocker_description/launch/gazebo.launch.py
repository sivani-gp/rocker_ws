import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description = get_package_share_directory("rocker_description")
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "true" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bot_description, "urdf", "rocker.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="empty.sdf",
        description="Path to the world file (.sdf)"
    )

    # Set Ignition resource path so that Gazebo can find models and plugins
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(bot_description).parent.resolve())
    )

    robot_description = ParameterValue(Command([
        "xacro ",
        LaunchConfiguration("model"),
        " is_sim:=true ",
        " is_ignition:=", is_ignition
    ]), value_type=str)

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Launch Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world")],
        }.items()
    )

    # Spawn the robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bot",
            "-x", "0", "-y", "0", "-z", "0.7"
        ],
    )

    # Bridge for /clock (for simulation time sync)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])

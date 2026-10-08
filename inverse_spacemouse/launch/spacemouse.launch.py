import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("inverse_spacemouse"), "config", "spacemouse.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument(
            "command_topic",
            default_value="spacemouse/command",
            description="e.g. /cartesian_velocity_controller/commands",
        ),
        Node(
            package="inverse_spacemouse",
            executable="spacemouse_node",
            name="spacemouse",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[("spacemouse/command", LaunchConfiguration("command_topic"))],
            output="screen",
        ),
    ])

"""Load one Cartesian controller into an existing controller manager."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        DeclareLaunchArgument('controller', default_value='cartesian_velocity_controller'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('velocity_controller'), 'config', 'controllers.yaml'
            ),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                LaunchConfiguration('controller'),
                '--controller-manager', LaunchConfiguration('controller_manager'),
                '--controller-type', 'velocity_controller/CartesianPositionController',
                '--param-file', LaunchConfiguration('params_file'),
            ],
            output='screen',
        ),
    ])

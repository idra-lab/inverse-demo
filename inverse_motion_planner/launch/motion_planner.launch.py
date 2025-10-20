import os.path
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def launch_setup(context, *args, **kwargs):

    nodes_to_start = []

    dilator_node = Node(
        package="magician_motion_planner",
        executable="mesh_dilator",
        #prefix=['xterm -e gdb -ex run --args']
    )

    planner_node = Node(
        package="magician_motion_planner",
        executable="ros2_motion_planner",
        name="ros2_motion_planner",
        output="screen", 
        namespace="motion_planner",
        parameters=[
            os.path.join(
                get_package_share_path("magician_resources"),
                "config",
                "parameters.yaml"
            ),
        ],
        #prefix=['xterm -e gdb -ex run --args']
    )

    nodes_to_start = [
        dilator_node,
        planner_node,
    ]

    return [
        TimerAction(
            period=2.0,
            actions=nodes_to_start,
        ),
    ]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

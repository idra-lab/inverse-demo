from launch.event_handlers import OnProcessExit, OnExecutionComplete
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)


def launch_setup(context, *args, **kwargs):

    nodes_to_start = []
    this_package = "magician_motion_planner"
    this_package_share = get_package_share_path(this_package)

    executor_node = Node(
        package=this_package,
        executable="ros2_plan_executor",
        output="screen",
        parameters=[{
            "debug_prints": False,
        }],
    )

    caller_node = Node(
        package=this_package,
        executable="test_caller.py",
        output="screen",
    )

    rviz_config = os.path.join(this_package_share, "rviz", "test.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    return [
        executor_node,
        caller_node,
        rviz,
    ]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments +
                             [OpaqueFunction(function=launch_setup)])

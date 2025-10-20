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

    bringup_package = get_package_share_path("doosan_bringup")
    doosan_launch_file = os.path.join(
        bringup_package, "launch", "h2515_bringup.launch.py"
    )
    doosan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(doosan_launch_file),
        launch_arguments={
            "xbot": "false",
            # "xbot": "true",
            "setup": "iit",
            "simulator": "false",
            # "simulator": "true",
        }.items(),
    )

    localisation_package = get_package_share_path("magician_localisation")
    workpiece_launch_file = os.path.join(
        localisation_package, "launch", "mesh_spawner.launch.py"
    )
    workpiece_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(workpiece_launch_file)
    )

    executor_package = get_package_share_path("magician_motion_planner")
    executor_launch_file = os.path.join(
        executor_package, "launch", "motion_planner.launch.py"
    )
    executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(executor_launch_file)
    )

    caller_node = Node(
        package=this_package,
        executable="test_caller.py",
        output="screen",
    )

    return [
        doosan_launch,
        workpiece_launch,
        executor_launch,
    ]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

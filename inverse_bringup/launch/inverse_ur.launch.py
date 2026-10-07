# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import launch_ros, launch


def publish_poses(context, *args, **kwargs):

    nodes_to_start = list()

    Z_OFFSET_SCREW = -0.015 #-0.008
    Z_OFFSET = 0.0

    frames = {
        "homing": (
            [-0.053, 0.404, 0.485],
            [-0.572, 0.820, 0.018, -0.003],
        ),
        # "kit1_connector_grasp": (
        #     [-0.084515, 0.9086, -0.036295 + Z_OFFSET],
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
        # "kit1_connector_deposit": (
        #     [0.13952, 0.98642, 0.038777 + Z_OFFSET],
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
        # "kit1_screw1": (
        #     [-0.085748, 0.75795, -0.037221 + Z_OFFSET_SCREW],  # done
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
        # #
        # "kit1_screw2": (
        #     [-0.086444, 0.77307, -0.037636 + Z_OFFSET_SCREW],  # done
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
        # "kit1_screw1_deposit": (  # done
        #     [0.14127, 0.96831, 0.047694 + Z_OFFSET],
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
        # "kit1_screw2_deposit": (
        #     [0.14104, 1.0031, 0.051714 + Z_OFFSET],
        #     [0.0, 1.0, 0.0, 0.0],
        # ),
    }
    for frame_name, (translation, rotation) in frames.items():
        node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_broadcaster_{frame_name}",
            arguments=[
                str(translation[0]),
                str(translation[1]),
                str(translation[2]),
                str(rotation[0]),
                str(rotation[1]),
                str(rotation[2]),
                str(rotation[3]),
                "base_link",
                frame_name,
            ],
        )
        nodes_to_start.append(node)
        print("Appening transform ")
    return nodes_to_start


def launch_realsense(context, *args, **kwargs):
    """Launch RealSense D435 with RGB compressed + aligned depth, plus image_transport republishers."""

    nodes_to_start = []

    # ── RealSense D435 driver ──────────────────────────────────────────────────
    # Publishes:
    #   /camera/color/image_raw            (RGB)
    #   /camera/aligned_depth_to_color/image_raw  (depth aligned to RGB frame)
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        namespace="camera",
        output="screen",
        parameters=[
            {
                # RGB stream
                "rgb_camera.color_profile": "640x480x30",
                "enable_color": True,
                "enable_pointcloud": False,
                # Depth stream
                "depth_module.depth_profile": "640x480x30",
                "enable_depth": True,
                # Align depth to the colour frame
                "align_depth.enable": True,
                # Disable streams we don't need to keep bandwidth low
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                # Publish tf
                "publish_tf": False,
            }
        ],
    )

    nodes_to_start += [realsense_node]
    return nodes_to_start


def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_path("easy_ur_control"),
                    "launch",
                    "easy_ur_launcher.launch.py",
                )
            ],
        ),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": "192.168.3.2",  # TO DO TEST CONTRELLER
            "ctrl": "cartesian_motion_controller",
            # Propagate simulation mode to UR + gripper stack.
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )

    params_file = os.path.join(
        get_package_share_path("inverse_bringup"), "config", "node_parameters.yaml",
    )
    motion_planner_node = Node(
        package="inverse_motion_planner",
        executable="motion_planner",
        name="motion_planner",
        output="screen",
        parameters=[params_file,],
    )
    skill_learner_node = Node(
        package="inverse_motion_planner",
        executable="skill_learner",
        name="skill_learner",
        output="screen",
        parameters=[params_file],
    )

    enable_planner = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/set_broadcast_state",
            "std_srvs/srv/SetBool",
            "{data: true}",
        ],
        output="screen",
    )
    homing_motion = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/reach_position",
            "inverse_msgs/srv/ReachPosition",
            (
                "{desired_pos: {"
                "header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'homing'}, "
                "pose: {"
                "position: {x: 0.0, y: 0.0, z: 0.0}, "
                "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
                "}}, "
                "max_vel: 0.05, immediate_execution: false}"
            ),
        ],
        output="screen",
    )
    nodes_to_start += [
        ur_launch,
        TimerAction(
            period=8.0,
            actions=[motion_planner_node, skill_learner_node],
        ),
        TimerAction(
            period=10.0,
            actions=[enable_planner, homing_motion],
        ),
    ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            # true: avoid opening real hardware drivers (UR + Robotiq).
            description="Use fake hardware for robot and gripper",
        )
    ]

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=launch_setup)]
        + [OpaqueFunction(function=publish_poses)]
        + [OpaqueFunction(function=launch_realsense)]
    )

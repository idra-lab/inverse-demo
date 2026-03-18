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

# def gripper_setup(context, *args, **kwargs):

#     description_pkg_share = launch_ros.substitutions.FindPackageShare(
#         package="robotiq_description"
#     ).find("robotiq_description")
#     default_model_path = os.path.join(
#         description_pkg_share, "urdf", "robotiq_2f_140_gripper.urdf.xacro"
#     )
#     default_rviz_config_path = os.path.join(
#         description_pkg_share, "rviz", "view_urdf.rviz"
#     )

#     args = []
#     args.append(
#         launch.actions.DeclareLaunchArgument(
#             name="model",
#             default_value=default_model_path,
#             description="Absolute path to gripper URDF file",
#         )
#     )
#     args.append(
#         launch.actions.DeclareLaunchArgument(
#             name="rvizconfig",
#             default_value=default_rviz_config_path,
#             description="Absolute path to rviz config file",
#         )
#     )
#     args.append(
#         launch.actions.DeclareLaunchArgument(
#             name="com_port",
#             default_value="/dev/ttyUSB0",
#             description="Port for communicating with Robotiq hardware",
#         )
#     )
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             LaunchConfiguration("model"),
#             " ",
#             "use_fake_hardware:=",
#             LaunchConfiguration("use_fake_hardware"),
#             " ",
#             "com_port:=",
#             LaunchConfiguration("com_port"),
#         ]
#     )

#     gripper_description_param = {
#         "robot_description": launch_ros.parameter_descriptions.ParameterValue(
#             robot_description_content, value_type=str
#         )
#     }

#     update_rate_config_file = PathJoinSubstitution(
#         [
#             description_pkg_share,
#             "config",
#             "robotiq_update_rate.yaml",
#         ]
#     )

#     controllers_file = "robotiq_controllers.yaml"
#     initial_joint_controllers = PathJoinSubstitution(
#         [description_pkg_share, "config", controllers_file]
#     )

#     control_node = launch_ros.actions.Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[
#             gripper_description_param,
#             update_rate_config_file,
#             initial_joint_controllers,
#         ],
#     )

#     gripper_state_publisher_node = launch_ros.actions.Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[gripper_description_param],
#     )

#     gripper_joint_state_broadcaster_spawner = launch_ros.actions.Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )

#     robotiq_gripper_controller_spawner = launch_ros.actions.Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
#     )

#     robotiq_activation_controller_spawner = launch_ros.actions.Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["robotiq_activation_controller", "-c", "/controller_manager"],
#     )

#     nodes = [
#         control_node,
#         robot_state_publisher_node,
#         joint_state_broadcaster_spawner,
#         robotiq_gripper_controller_spawner,
#         robotiq_activation_controller_spawner,
#         rviz_node,
#     ]

#     return launch.LaunchDescription(args + nodes)


def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()
    
    bota_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_path("rokubimini_serial"),
                "launch",
                "rokubimini_serial.launch.py"
            )
        ], ),
    )

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_path("easy_ur_control"),
                "launch",
                "easy_ur_launcher.launch.py"
            )
        ], ),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": "192.168.3.2", # TO DO TEST CONTRELLER
            "ctrl": "cartesian_motion_controller",
            # Propagate simulation mode to UR + gripper stack.
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )

    start_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/planner/set_broadcast_state',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ],
        output='screen'
    )

    # Gripper utilities
    
    # gripper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_path("robotiq_description"),
    #             "launch",
    #             "robotiq_control.launch.py"
    #         )
    #     ], ),
    # )

    nodes_to_start += [
        # bota_launch,
        ur_launch,
        # gripper_launch,
        # Node(
        #     package="inverse_motion_planner",
        #     executable="motion_planner",
        #     name="motion_planner",
        #     output="screen",
        #     namespace="planner",
        #     parameters=[
        #         os.path.join(
        #             get_package_share_path("inverse_bringup"),
        #             "config",
        #             "node_parameters.yaml"
        #         ),
        #     ],
        # ),

        TimerAction(
            period=5.0,
            actions=[start_controller],
        )
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
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

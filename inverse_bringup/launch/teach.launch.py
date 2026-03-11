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


def launch_setup(context, *args, **kwargs):

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_path("easy_ur_control"),
                "launch",
                "easy_ur_launcher.launch.py"
            )
        ], ),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": "192.168.100.10", # to check
            "ctrl": "cartesian_motion_controller",
        }.items(),
    )

    skill_learner_node = Node(
                    package="inverse_motion_planner",
                    executable="skill_learner",
                    name="skill_learner",
                    output="screen",
                    parameters=[
                        os.path.join(
                            get_package_share_path("inverse_bringup"),
                            "config",
                            "parameters.yaml"
                        ),
                    ],
                )

    nodes_to_start = [
        ur_launch,
        skill_learner_node,
    ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
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


def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()

    # frames = {
    #     """"kit1_connector_grasp": ([
    #         0.41213,
    #         -0.058,
    #         0.37193,
    #     ], [
    #         0.99956,
    #         -0.028345,
    #         0.006056,
    #         0.0057203,
    #     ]),"""    }

    # for frame_name, (translation, rotation) in frames.items():
    #     node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name=f'static_broadcaster_{frame_name}',
    #         arguments=[
    #             str(translation[0]),
    #             str(translation[1]),
    #             str(translation[2]),
    #             str(rotation[0]),
    #             str(rotation[1]),
    #             str(rotation[2]),
    #             str(rotation[3]),
    #             'franka1_fr3_link0',
    #             frame_name
    #         ]
    #     )
    #     nodes_to_start.append(node)

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


    nodes_to_start += [
        ur_launch,
        # right_controller_spawner,
        Node(
            package="inverse_motion_planner",
            executable="motion_planner",
            name="motion_planner",
            output="screen",
            namespace="planner",
            parameters=[
                os.path.join(
                    get_package_share_path("inverse_bringup"),
                    "config",
                    "node_parameters.yaml"
                ),
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[start_controller],
        )
    ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

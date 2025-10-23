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

    frames = {
        "kit1_connector_grasp": ([
            0.41213,
            -0.058,
            0.37193,
        ], [
            0.99956,
            -0.028345,
            0.006056,
            0.0057203,
        ]),
        "kit1_screw1": ([
            0.36733,
            -0.036261,
            0.35526,
        ], [1.0, 0.0, 0.0, 0.0]),
        "kit1_screw2": ([
            0.36556,
            -0.0083178,
            0.35534,
        ], [1.0, 0.0, 0.0, 0.0]),
        "kit1_connector_deposit": ([
            0.40234,
            0.3247,
            0.003,
        ], [1.0, 0.0, 0.0, 0.0]),
        # "kit1_screw1_deposit": ([0.41692, 0.32603, 0.01], [0.97579, -0.21575, 0.03461, -0.0088881]),
        "kit1_screw1_deposit": ([0.41462, 0.32599, 0.011266], [0.8697, -0.4918, 0.038902, -0.015386]),
        "kit1_screw2_deposit": ([0.38403, 0.32274, 0.011815], [0.97579, -0.21575, 0.03461, -0.0088881]),
    }

    for frame_name, (translation, rotation) in frames.items():
        node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_broadcaster_{frame_name}',
            arguments=[
                str(translation[0]),
                str(translation[1]),
                str(translation[2]),
                str(rotation[0]),
                str(rotation[1]),
                str(rotation[2]),
                str(rotation[3]),
                'franka1_fr3_link0',
                frame_name
            ]
        )
        nodes_to_start.append(node)

    multimanual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_path("inverse_bringup"),
                "launch",
                "bimanual.launch.py"
            )
        ], ),
        launch_arguments={
            "left_ip": "192.168.9.11",
            "right_ip": "192.168.9.12",
        }.items(),
    )
    # Set Force/Torque Collision Behavior for franka1
    franka1_collision_behavior = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/franka1_service_server/set_force_torque_collision_behavior',
            'franka_msgs/srv/SetForceTorqueCollisionBehavior',
            "{lower_torque_thresholds_nominal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
            "upper_torque_thresholds_nominal: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0], "
            "lower_force_thresholds_nominal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
            "upper_force_thresholds_nominal: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0]}"
        ],
        output='screen'
    )

    # Set Force/Torque Collision Behavior for franka2
    franka2_collision_behavior = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/franka2_service_server/set_force_torque_collision_behavior',
            'franka_msgs/srv/SetForceTorqueCollisionBehavior',
            "{lower_torque_thresholds_nominal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
            "upper_torque_thresholds_nominal: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0], "
            "lower_force_thresholds_nominal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
            "upper_force_thresholds_nominal: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0]}"
        ],
        output='screen'
    )

    left_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_cartesian_impedance_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    right_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_cartesian_impedance_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    left_planner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_controller_spawner,
            on_exit=[
                Node(
                    package="inverse_motion_planner",
                    executable="motion_planner",
                    name="left_motion_planner",
                    output="screen",
                    namespace="left_planner",
                    parameters=[
                        os.path.join(
                            get_package_share_path("inverse_bringup"),
                            "config",
                            "parameters.yaml"
                        ),
                    ],
                )
            ]
        )
    )

    start_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/right_planner/set_broadcast_state',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ],
        output='screen'
    )

    right_planner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_controller_spawner,
            on_exit=[
                Node(
                    package="inverse_motion_planner",
                    executable="motion_planner",
                    name="right_motion_planner",
                    output="screen",
                    namespace="right_planner",
                    parameters=[
                        os.path.join(
                            get_package_share_path("inverse_bringup"),
                            "config",
                            "parameters.yaml"
                        ),
                    ],
                ),
                TimerAction(
                    period=2.0,
                    actions=[start_controller],
                )
            ]
        )
    )

    nodes_to_start += [
        multimanual_launch,
        franka1_collision_behavior,
        franka2_collision_behavior,
        left_controller_spawner,
        right_controller_spawner,
        left_planner_node,
        right_planner_node,
    ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

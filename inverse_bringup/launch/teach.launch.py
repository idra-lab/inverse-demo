# Copyright 2025 IDRA, University of Trento
# Author: Davide Nardi (davide.nardi-1@unitn.com)
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

# Publisher node
import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import Bool
from threading import Thread
import time

# --- funzione per avviare un publisher in background ---
def start_freedrive_publisher():
    rclpy.init()
    node = RclpyNode("freedrive_publisher_inline")
    pub = node.create_publisher(Bool, "/freedrive_mode_controller/enable_freedrive_mode", 10)

    def spin_pub():
        rate = 2.0  # Hz
        msg = Bool()
        msg.data = True
        while rclpy.ok():
            pub.publish(msg)
            time.sleep(1.0 / rate)

    thread = Thread(target=spin_pub)
    thread.daemon = True  # si chiude con il launch
    thread.start()


# --- funzione chiamata dal launch file ---
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
            "ur_type": "ur10",
            "robot_ip": "192.168.3.2", # to check
            "ctrl": "freedrive_mode_controller",
            "use_fake_hardware": "false",
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
                            "node_parameters.yaml"
                        ),
                    ],
                )

    # --- publisher inline per abilitare il freedrive mode ---
    start_freedrive_publisher()

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

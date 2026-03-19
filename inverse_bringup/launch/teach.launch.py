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
from builtin_interfaces.msg import Duration 
import time
from controller_manager_msgs.srv import SwitchController

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


# --- funzione per cambiare controller in modo programmatico ---
def switch_controllers(deactivate: list, activate: list):
    # Inizializza un nodo temporaneo per il servizio
    if not rclpy.ok():
        rclpy.init()
    node = RclpyNode("controller_switcher_inline")

    cli = node.create_client(SwitchController, '/controller_manager/switch_controller')
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for switch_controller service...")

    req = SwitchController.Request()
    req.start_controllers = activate
    req.stop_controllers = deactivate
    req.strictness = SwitchController.Request.STRICT  # STRICT=2, BEST_EFFORT=1
    req.start_asap = True
    req.timeout = Duration(sec=5, nanosec=0)  # <-- correggi così

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Switch result: {future.result().ok}")
    else:
        node.get_logger().error("Failed to call switch_controller service")

    node.destroy_node()


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
            "ctrl": "cartesian_motion_controller",
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

    # --- switch controller solo dopo che tutto è pronto ---
    def switch_after_startup():
        # aspetta qualche secondo per essere sicuro che il controller_manager sia online
        time.sleep(5.0)
        # esempio: disattiva cartesian_motion_controller, attiva freedrive_mode_controller
        switch_controllers(
            deactivate=['cartesian_motion_controller'],
            activate=['freedrive_mode_controller']
        )

    Thread(target=switch_after_startup, daemon=True).start()

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
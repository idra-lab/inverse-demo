#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from inverse_orchestrator.skill_executor import SkillExecutor
from inverse_orchestrator.gripper_controller import GripperController
import time


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # Instantiate action modules with this node

        # franka right
        self.skill_exec_right = SkillExecutor(self, "right_planner/execute_skill")
        self.gripper_right = GripperController(
            self, "franka1/franka_gripper/grasp", "franka1/franka_gripper/move"
        )
        self.base_link1 = "franka1_fr3_link0"
        # franka left
        self.skill_exec_left = SkillExecutor(self, "left_planner/execute_skill")
        self.gripper_left = GripperController(
            self, "franka2/franka_gripper/grasp", "franka2/franka_gripper/move"
        )
        self.base_link2 = "franka2_fr3_link0"

        self.kit1_frame_name = "kit1_connector_grasp"
        self.kit1_connector_deposit_frame_name = "kit1_connector_deposit"

        self.get_logger().info("Orchestrator initialized and ready.")

    def orchestrate(self):
        """Example orchestration routine combining primitives."""

        # open_future = self.gripper_right.open_gripper()
        input("Press Enter to move gripper")
        move_future = self.gripper_right.move_finger(width=0.03, speed=0.1)

        input("Press Enter to move robot.")

        # # # Prepare sample poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.base_link1
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_frame_name
        end_pose.pose.position.x = 0.0
        end_pose.pose.position.y = 0.0
        end_pose.pose.position.z = 0.0
        end_pose.pose.orientation.x = 0.0
        end_pose.pose.orientation.y = 0.0
        end_pose.pose.orientation.z = 0.0
        end_pose.pose.orientation.w = 1.0

        skill_future = self.skill_exec_right.execute_skill(
            "home_to_kit1", start_pose, end_pose
        )

        input("Press Enter to close gripper")

        close_future = self.gripper_right.close_gripper()

        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        end_pose.pose.position.x = 0.0
        end_pose.pose.position.y = 0.0
        end_pose.pose.position.z = 0.0
        end_pose.pose.orientation.x = 0.0
        end_pose.pose.orientation.y = 0.0
        end_pose.pose.orientation.z = 0.0
        end_pose.pose.orientation.w = 1.0

        input("Press Enter to move robot to holder.")
        skill_future = self.skill_exec_right.execute_skill(
            "kit1_to_deposit", start_pose, end_pose, use_learned_final=False
        )

        # # Sequence actions
        # # Open gripper
        # # self.get_logger().info("Opening gripper...")
        # close_future = self.gripper_right.close_gripper()


def main():
    rclpy.init()
    orchestrator = Orchestrator()

    executor = MultiThreadedExecutor()
    executor.add_node(orchestrator)

    # Start orchestration in a separate thread to avoid blocking executor
    import threading

    threading.Thread(target=orchestrator.orchestrate, daemon=True).start()

    try:
        executor.spin()
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

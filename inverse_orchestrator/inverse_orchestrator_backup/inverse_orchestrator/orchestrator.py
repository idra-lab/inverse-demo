#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from my_robot_orchestrator.skill_executor import SkillExecutor
from my_robot_orchestrator.gripper_controller import GripperController


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # Instantiate subnodes
        self.skill_exec = SkillExecutor()
        self.gripper = GripperController("franka_gripper")

        self.get_logger().info("Orchestrator initialized and ready.")

    async def orchestrate(self):
        """Example orchestration routine combining primitives."""

        # Prepare sample poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = "base_link"
        start_pose.pose.position.x = 0.4
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.2
        start_pose.pose.orientation.w = 1.0

        end_pose = PoseStamped()
        end_pose.header.frame_id = "base_link"
        end_pose.pose.position.x = 0.6
        end_pose.pose.position.y = 0.1
        end_pose.pose.position.z = 0.3
        end_pose.pose.orientation.w = 1.0

        # Sequence actions
        await self.gripper.open_gripper()
        success = await self.skill_exec.execute_skill(
            "pick_and_place", start_pose, end_pose
        )
        if success:
            await self.gripper.close_gripper()
        else:
            self.get_logger().warn("Skill failed, leaving gripper open.")


async def main_async():
    rclpy.init()
    orchestrator = Orchestrator()

    await orchestrator.orchestrate()

    # Clean shutdown
    orchestrator.get_logger().info("Orchestration finished.")
    orchestrator.destroy_node()
    orchestrator.skill_exec.destroy_node()
    orchestrator.gripper.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()

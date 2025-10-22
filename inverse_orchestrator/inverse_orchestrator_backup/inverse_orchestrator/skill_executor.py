#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from inverse_msgs.srv import ExecuteSkill


class SkillExecutor(Node):
    """Handles calling the ExecuteSkill service."""

    def __init__(self):
        super().__init__("skill_executor")
        self.client = self.create_client(ExecuteSkill, "execute_skill")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for ExecuteSkill service...")

    async def execute_skill(
        self,
        skill_name,
        initial_pose,
        final_pose,
        use_learned_initial=True,
        use_learned_final=True,
        max_vel=0.05,
    ):
        """Send a skill execution request and await response."""
        req = ExecuteSkill.Request()
        req.skill_name = skill_name
        req.use_learned_initial_pose = use_learned_initial
        req.use_learned_final_pose = use_learned_final
        req.initial_pose = initial_pose
        req.final_pose = final_pose
        req.max_vel = max_vel

        self.get_logger().info(f"Executing skill: {skill_name}")
        future = self.client.call_async(req)
        result = await future
        return result.success

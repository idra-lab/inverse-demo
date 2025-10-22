#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp


class GripperController(Node):
    """Controls the Franka gripper using the Grasp action."""

    def __init__(self, gripper_name="franka_gripper"):
        super().__init__(f"{gripper_name}_controller")
        self.client = ActionClient(self, Grasp, f"/{gripper_name}/grasp")
        self.get_logger().info(f"Waiting for {gripper_name} action server...")
        self.client.wait_for_server()
        self.get_logger().info("Gripper action server ready!")

    async def open_gripper(self, width=0.08, speed=0.1, force=0.0):
        """Open gripper."""
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005

        self.get_logger().info(f"Opening gripper to {width:.3f} m")
        goal_future = self.client.send_goal_async(goal)
        goal_handle = await goal_future
        if not goal_handle.accepted:
            self.get_logger().warn("Open gripper goal rejected!")
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future
        success = result.result.success
        self.get_logger().info(f"Open gripper success: {success}")
        return success

    async def close_gripper(self, width=0.0, speed=0.05, force=30.0):
        """Close gripper."""
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005

        self.get_logger().info(f"Closing gripper with force {force:.1f} N")
        goal_future = self.client.send_goal_async(goal)
        goal_handle = await goal_future
        if not goal_handle.accepted:
            self.get_logger().warn("Close gripper goal rejected!")
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future
        success = result.result.success
        self.get_logger().info(f"Close gripper success: {success}")
        return success

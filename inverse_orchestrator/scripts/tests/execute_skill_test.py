#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform
from inverse_msgs.srv import ExecuteSkill

class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        self.base_link1 = "world"
        self.get_logger().info("Orchestrator initialized and ready.")

    
    def test_execute_skill(self):
        self.get_logger().info("Testing ExecuteSkill service call...")
        client = self.create_client(ExecuteSkill, "/execute_skill")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for ExecuteSkill service...")
        self.get_logger().info("ExecuteSkill service is available, sending request...")
        request = ExecuteSkill.Request()
        request.skill_name = "test_skill"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"ExecuteSkill response: {future.result()}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")



def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_execute_skill()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform
from inverse_msgs.srv import LearnSkill

HUMAN_DISTANCE_TRIGGER = 0.40 # meters


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        self.base_link1 = "world"

        self.get_logger().info("Orchestrator initialized and ready.")


    
    def test_learn_skill(self):
        self.get_logger().info("Testing LearnSkill service call...")
        client = self.create_client(LearnSkill, "/learn_skill")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for LearnSkill service...")
        self.get_logger().info("LearnSkill service is available, sending request...")
        request = LearnSkill.Request()
        request.skill_name = "test_skill"
        request.registration_duration_secs = 10.0
        request.num_basis = 30
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"LearnSkill response: {future.result()}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")



def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_learn_skill()

if __name__ == "__main__":
    main()

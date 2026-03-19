#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform
from inverse_msgs.srv import MoveRelative

HUMAN_DISTANCE_TRIGGER = 0.40 # meters


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # self.reach_pose = ReachPosition_Class(
        #     self, "planner/reach_position"
        # )

        # self.base_link1 = "world"

        self.get_logger().info("Orchestrator initialized and ready.")

        self.get_logger().info("\n\n\n\n\n----------------\nStarting orchestrator...")

    
    def test_move_relative(self):
        self.get_logger().info("Testing MoveRelative service call...")
        client = self.create_client(MoveRelative, "/move_relative")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveRelative service...")
        
        request = MoveRelative.Request()
        request.relative_motion = Transform()
        request.relative_motion.translation.x = 0.1

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"MoveRelative response: {future.result()}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

    
def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_move_relative()

if __name__ == "__main__":
    main()

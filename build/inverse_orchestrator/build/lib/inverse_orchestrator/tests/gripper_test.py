#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from inverse_orchestrator.ur_gripper_controller import URGripperController


class Orchestrator(Node):

    def __init__(self, mode=URGripperController.MODE_GRIPPER_COMMAND):
        super().__init__("ur_open_robotiq_gripper_test")

        self.gripper = URGripperController(
            self,
            mode=mode,
        )

    def test_opening_gripper(self):
        """Test opening the Robotiq gripper through URGripperController."""
        self.get_logger().info("Sending open command to Robotiq gripper")
        future = self.gripper.open_gripper(max_effort=1.0)
        rclpy.spin_until_future_complete(self, future)
        success = future.result()
        if success:
            self.get_logger().info("Gripper open command completed")
        else:
            self.get_logger().error("Gripper open command failed")


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_opening_gripper()
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

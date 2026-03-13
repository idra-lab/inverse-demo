#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from inverse_orchestrator.ur_gripper_controller import URGripperController


class Orchestrator(Node):

    def __init__(self):
        super().__init__("ur_open_robotiq_gripper_test")

        self.gripper = URGripperController(
            self,
            mode=URGripperController.MODE_GRIPPER_COMMAND,
            command_duration_sec=1,
        )

    def test_opening_gripper(self):

        rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Sending open command to Robotiq gripper")
        open_future = self.gripper.open_gripper(max_effort=1.0)
        rclpy.spin_until_future_complete(self, open_future)

        if not open_future.result():
            self.get_logger().error("Gripper open command failed")
            return

        time.sleep(5.0)

        self.get_logger().info("Sending close command to Robotiq gripper")
        close_future = self.gripper.close_gripper(max_effort=1.0)
        rclpy.spin_until_future_complete(self, close_future)

        if close_future.result():
            self.get_logger().info("Gripper open-close cycle completed")
        else:
            self.get_logger().error("Gripper close command failed")


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_opening_gripper()
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

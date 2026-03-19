import rclpy
from rclpy.node import Node

from inverse_orchestrator.ur_gripper_controller import URGripper


class GripperTestNode(Node):

    def __init__(self):
        super().__init__("gripper_test")
        self._gripper = URGripper(node=self)
        self.get_logger().info("GripperTestNode initialized, ready to send commands to the gripper.")

    def test_open(self) -> None:
        self.get_logger().info("Opening gripper...")
        future = self._gripper.open()
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Open result: {future.result()}")

    def test_close(self) -> None:
        self.get_logger().info("Closing gripper...")
        future = self._gripper.close()
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Close result: {future.result()}")

    def test_command(self, position: float = 0.5, max_effort: float = 50.0) -> None:
        self.get_logger().info(f"Sending gripper command: position={position}, max_effort={max_effort}...")
        future = self._gripper.command(position=position, max_effort=max_effort)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Command result: {future.result()}")


def main():
    rclpy.init()
    node = GripperTestNode()
    node.test_open()
    node.test_command(position=0.5, max_effort=50.0)
    node.test_close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

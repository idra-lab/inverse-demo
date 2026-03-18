from rclpy.node import Node
from rclpy.action import ActionClient
from concurrent.futures import Future

from control_msgs.action import GripperCommand


class URGripper:
    """Robotiq gripper controller targeting the UR-mounted gripper.

    Equivalent CLI call:
        ros2 action send_goal /robotiq_gripper/gripper/gripper_cmd \
            control_msgs/action/GripperCommand \
            "{command: {position: 0.5, max_effort: 50.0}}"
    """

    ACTION_NAME = "/robotiq_gripper_controller/gripper_cmd"

    def __init__(
        self,
        node: Node,
        open_position: float = 0.0,
        closed_position: float = 0.7,
        action_wait_timeout_sec: float = 2.0,
    ):
        self.node = node
        self.open_position = open_position
        self.closed_position = closed_position

        self._min_position = min(open_position, closed_position)
        self._max_position = max(open_position, closed_position)

        self._action_client = ActionClient(self.node, GripperCommand, self.ACTION_NAME)

        self._action_available = self._action_client.wait_for_server(
            timeout_sec=action_wait_timeout_sec
        )

        if self._action_available:
            self.node.get_logger().info("Robotiq GripperCommand action server ready")
        else:
            self.node.get_logger().error("GripperCommand action server NOT available")

    # ========================= PUBLIC API =========================

    def open(self, max_effort: float = 50.0) -> Future:
        """Open the gripper fully."""
        return self.command(self.open_position, max_effort)

    def close(self, max_effort: float = 50.0) -> Future:
        """Close the gripper fully."""
        return self.command(self.closed_position, max_effort)

    def command(self, position: float, max_effort: float = 50.0) -> Future:
        """Send a position command to the gripper."""
        clamped = max(self._min_position, min(self._max_position, position))

        if clamped != position:
            self.node.get_logger().warn(
                f"Position {position:.3f} out of range, clamped to {clamped:.3f}"
            )

        if not self._action_available:
            return self._failed_future("GripperCommand unavailable")

        goal = GripperCommand.Goal()
        goal.command.position = clamped
        goal.command.max_effort = max_effort

        goal_future = self._action_client.send_goal_async(goal)
        final_future = Future()

        def _goal_response_cb(fut: Future):
            goal_handle = fut.result()

            if goal_handle is None or not goal_handle.accepted:
                self.node.get_logger().error("Gripper goal rejected")
                final_future.set_result(False)
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_cb)

        def _result_cb(fut: Future):
            result = fut.result()
            success = result is not None

            if success:
                self.node.get_logger().info("Gripper command completed")
            else:
                self.node.get_logger().error("Gripper command failed")

            final_future.set_result(success)

        goal_future.add_done_callback(_goal_response_cb)
        return final_future

    # ========================= INTERNAL =========================

    def _failed_future(self, reason: str) -> Future:
        self.node.get_logger().error(reason)
        f = Future()
        f.set_result(False)
        return f

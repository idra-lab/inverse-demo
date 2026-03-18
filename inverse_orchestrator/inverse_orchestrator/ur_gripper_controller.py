from rclpy.node import Node

from inverse_orchestrator.gripper_controller import URGripperController


class URGripper:
    """Wrapper around URGripperController targeting the UR-mounted Robotiq gripper.

    Equivalent CLI call:
        ros2 action send_goal /robotiq_gripper/gripper/gripper_cmd \
            control_msgs/action/GripperCommand \
            "{command: {position: 0.5, max_effort: 50.0}}"
    """

    ACTION_NAME = "/robotiq_gripper/gripper/gripper_cmd"

    def __init__(
        self,
        node: Node,
        open_position: float = 0.0,
        closed_position: float = 0.7,
        action_wait_timeout_sec: float = 2.0,
    ):
        self._controller = URGripperController(
            node=node,
            action_name=self.ACTION_NAME,
            open_position=open_position,
            closed_position=closed_position,
            action_wait_timeout_sec=action_wait_timeout_sec,
        )

    def open(self, max_effort: float = 50.0):
        """Open the gripper fully."""
        return self._controller.open_gripper(max_effort=max_effort)

    def close(self, max_effort: float = 50.0):
        """Close the gripper fully."""
        return self._controller.close_gripper(max_effort=max_effort)

    def command(self, position: float, max_effort: float = 50.0):
        """Send an arbitrary position command (clamped to [open, closed] range)."""
        return self._controller.command(position=position, max_effort=max_effort)

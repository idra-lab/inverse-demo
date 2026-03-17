import rclpy
import time
from rclpy.node import Node
from concurrent.futures import Future

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class URGripperController:

    MODE_GRIPPER_COMMAND = "gripper_command"
    MODE_JOINT_TRAJECTORY = "joint_trajectory"

    def __init__(
        self,
        node,
        mode=MODE_GRIPPER_COMMAND,
        gripper_command_action_name="/robotiq_gripper_controller/gripper_cmd",
        joint_trajectory_topic="/robotiq_gripper_trajectory_controller/joint_trajectory",
        trajectory_joint_name="finger_joint",
        open_position=0.0,
        closed_position=0.7,
        command_duration_sec=5,
        command_duration_nanosec=0,
        action_wait_timeout_sec=2.0,
        trajectory_discovery_timeout_sec=0.5,
    ):
        self.node = node
        self.mode = mode

        self.open_position = open_position
        self.closed_position = closed_position
        self.command_duration = Duration(
            sec=command_duration_sec,
            nanosec=command_duration_nanosec,
        )

        self._min_position = min(self.closed_position, self.open_position)
        self._max_position = max(self.closed_position, self.open_position)

        self._action_client = None
        self._trajectory_pub = None
        self._action_name = gripper_command_action_name
        self._trajectory_topic = joint_trajectory_topic
        self._trajectory_joint_name = trajectory_joint_name
        self._trajectory_discovery_timeout_sec = trajectory_discovery_timeout_sec
        self._action_available = False

        self._action_client = ActionClient(
            self.node,
            GripperCommand,
            self._action_name,
        )
        self._trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            self._trajectory_topic,
            10,
        )

        self._action_available = self._action_client.wait_for_server(
            timeout_sec=action_wait_timeout_sec
        )

        if self.mode == self.MODE_GRIPPER_COMMAND:
            if self._action_available:
                self.node.get_logger().info("Robotiq GripperCommand action server ready")
            else:
                self.node.get_logger().warn(
                    "GripperCommand action server unavailable"
                )

        elif self.mode == self.MODE_JOINT_TRAJECTORY:
            self.node.get_logger().info(
                f"Robotiq JointTrajectory publisher ready on: {self._trajectory_topic}"
            )
        
        else:
            raise ValueError(
                f"Unsupported mode '{self.mode}'. Expected "
                f"'{self.MODE_GRIPPER_COMMAND}' or '{self.MODE_JOINT_TRAJECTORY}'."
            )


    def command(self, position: float, max_effort: float = 1.0) -> Future:

        clamped_position = max(self._min_position, min(self._max_position, position))
        
        if clamped_position != position:
            self.node.get_logger().warn(
                f"Requested gripper position {position:.4f} out of range, clamped to "
                f"{clamped_position:.4f}"
            )

        if self.mode == self.MODE_GRIPPER_COMMAND:
            if self._action_available:
                return self._command_via_action(clamped_position, max_effort)
            return self._failed_future("GripperCommand unavailable")

        if self.mode == self.MODE_JOINT_TRAJECTORY:
            if self._trajectory_has_subscriber(wait=True):
                return self._command_via_trajectory(clamped_position)
            return self._failed_future(
                "JointTrajectory unavailable"
            )

        return self._failed_future(f"Unsupported active mode '{self.mode}'")


    def open_gripper(self, max_effort: float = 1.0) -> Future:
        return self.command(self.open_position, max_effort=max_effort)


    def close_gripper(self, max_effort: float = 1.0) -> Future:
        return self.command(self.closed_position, max_effort=max_effort)


    def _command_via_action(self, position: float, max_effort: float) -> Future:

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        goal_future = self._action_client.send_goal_async(goal)
        final_future = Future()

        def _on_goal_response(fut: Future) -> None:
            goal_handle = fut.result()
            if goal_handle is None or not goal_handle.accepted:
                self.node.get_logger().warn("Action goal rejected")
                final_future.set_result(False)
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_on_result)

        def _on_result(fut: Future) -> None:
            result = fut.result()
            final_future.set_result(result is not None)

        goal_future.add_done_callback(_on_goal_response)
        return final_future


    def _command_via_trajectory(self, position: float) -> Future:

        msg = JointTrajectory()
        msg.joint_names = [self._trajectory_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = self.command_duration
        msg.points = [point]

        self._trajectory_pub.publish(msg)

        final_future = Future()
        final_future.set_result(True)
        return final_future


    def _trajectory_has_subscriber(self, wait: bool = False) -> bool:

        if self._trajectory_pub.get_subscription_count() > 0:
            return True

        if not wait:
            return False

        deadline = time.monotonic() + self._trajectory_discovery_timeout_sec

        while time.monotonic() < deadline:
            if self._trajectory_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return False


    def _failed_future(self, reason: str) -> Future:
        self.node.get_logger().error(reason)
        failed = Future()
        failed.set_result(False)
        return failed


GripperController = URGripperController
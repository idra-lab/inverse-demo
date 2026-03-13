from concurrent.futures import Future

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class URGripperController:
    """Controls a Robotiq gripper via either GripperCommand or JointTrajectory."""

    # Define mode for the type of command interface to use
    MODE_GRIPPER_COMMAND = "gripper_command"
    MODE_JOINT_TRAJECTORY = "joint_trajectory"

    def __init__(
        self,
        node,
        mode=MODE_GRIPPER_COMMAND,
        gripper_command_action_name="/robotiq_gripper_controller/gripper_cmd",
        joint_trajectory_topic="/robotiq_gripper_controller/joint_trajectory",
        trajectory_joint_name="robotiq_85_left_knuckle_joint",
        open_position=0.025,
        closed_position=0.0,
        command_duration_sec=1,
        command_duration_nanosec=0,
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
        self._trajectory_joint_name = trajectory_joint_name

        if self.mode == self.MODE_GRIPPER_COMMAND:
            self._action_client = ActionClient(
                self.node,
                GripperCommand,
                gripper_command_action_name,
            )
            self.node.get_logger().info(
                f"Waiting for action server: {gripper_command_action_name}"
            )
            self._action_client.wait_for_server()
            self.node.get_logger().info("Robotiq GripperCommand action server ready")
        elif self.mode == self.MODE_JOINT_TRAJECTORY:
            self._trajectory_pub = self.node.create_publisher(
                JointTrajectory,
                joint_trajectory_topic,
                10,
            )
            self.node.get_logger().info(
                f"Robotiq JointTrajectory publisher ready on: {joint_trajectory_topic}"
            )
        else:
            raise ValueError(
                f"Unsupported mode '{self.mode}'. Expected '{self.MODE_GRIPPER_COMMAND}' "
                f"or '{self.MODE_JOINT_TRAJECTORY}'."
            )

    def command(self, position, max_effort=1.0):
        """Command a target gripper position, returning Future[bool]."""
        clamped_position = max(self._min_position, min(self._max_position, position))
        if clamped_position != position:
            self.node.get_logger().warn(
                f"Requested gripper position {position:.4f} out of range, clamped to "
                f"{clamped_position:.4f}"
            )

        if self.mode == self.MODE_GRIPPER_COMMAND:
            return self._command_via_action(clamped_position, max_effort)
        return self._command_via_trajectory(clamped_position)

    def open_gripper(self, max_effort=1.0):
        """Open the gripper and return Future[bool]."""
        return self.command(self.open_position, max_effort=max_effort)

    def close_gripper(self, max_effort=1.0):
        """Close the gripper and return Future[bool]."""
        return self.command(self.closed_position, max_effort=max_effort)

    def _command_via_action(self, position, max_effort):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        goal_future = self._action_client.send_goal_async(goal)
        final_future = Future()

        def _on_goal_response(fut):
            goal_handle = fut.result()
            if goal_handle is None or not goal_handle.accepted:
                self.node.get_logger().warn("Robotiq action goal rejected")
                final_future.set_result(False)
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_on_result)

        def _on_result(fut):
            result = fut.result()
            final_future.set_result(result is not None)

        goal_future.add_done_callback(_on_goal_response)
        return final_future

    def _command_via_trajectory(self, position):
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

GripperController = URGripperController

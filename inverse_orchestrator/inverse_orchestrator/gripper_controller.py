from franka_msgs.action import Grasp, Move
from rclpy.action import ActionClient
from concurrent.futures import Future


class GripperController:
    """Controls the Franka gripper using Grasp and Move actions with callbacks/futures."""

    def __init__(
        self,
        node,
        gripper_action_grasp="franka_gripper/grasp",
        gripper_action_move="franka_gripper/move",
    ):
        self.node = node

        # Grasp action client
        self.grasp_client = ActionClient(self.node, Grasp, f"{gripper_action_grasp}")
        self.node.get_logger().info(
            f"Waiting for {gripper_action_grasp} action server..."
        )
        self.grasp_client.wait_for_server()
        self.node.get_logger().info("Grasp action server ready!")

        # Move action client
        self.move_client = ActionClient(self.node, Move, f"{gripper_action_move}")
        self.node.get_logger().info(
            f"Waiting for {gripper_action_move} action server..."
        )
        self.move_client.wait_for_server()
        self.node.get_logger().info("Move action server ready!")

    # ------------------- Grasp -------------------
    def open_gripper(self, width=0.08, speed=0.1, force=2.0):
        """Open gripper using Grasp action. Returns Future."""
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.02

        self.node.get_logger().info(f"Opening gripper to {width:.3f} m")
        return self._send_grasp_goal(goal, "Open")

    def close_gripper(self, width=0.0, speed=0.05, force=30.0):
        """Close gripper using Grasp action. Returns Future."""
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.05
        goal.epsilon.outer = 0.05

        self.node.get_logger().info(f"Closing gripper with force {force:.1f} N")
        return self._send_grasp_goal(goal, "Close")

    def _send_grasp_goal(self, goal, action_name="Grasp"):
        """Internal helper for sending Grasp goals."""
        goal_future = self.grasp_client.send_goal_async(goal)
        final_future = Future()

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn(f"{action_name} goal rejected!")
                final_future.set_result(False)
                return
            self.node.get_logger().info(f"{action_name} goal accepted.")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)

        def result_callback(fut):
            result = fut.result().result
            success = result.success
            self.node.get_logger().info(f"{action_name} success: {success}")
            final_future.set_result(success)

        goal_future.add_done_callback(goal_response_callback)
        return final_future

    # ------------------- Move -------------------
    def move_finger(self, width, speed):
        """Move the gripper to a precise width using the Move action. Returns Future."""
        goal = Move.Goal()
        goal.width = width
        goal.speed = speed

        self.node.get_logger().info(
            f"Moving gripper finger to width {width:.3f} m with speed {speed:.3f}"
        )
        goal_future = self.move_client.send_goal_async(goal)
        final_future = Future()

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn("Move goal rejected!")
                final_future.set_result(False)
                return
            self.node.get_logger().info("Move goal accepted.")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)

        def result_callback(fut):
            result = fut.result().result
            success = result.success
            self.node.get_logger().info(f"Move success: {success}")
            final_future.set_result(success)

        goal_future.add_done_callback(goal_response_callback)
        return final_future

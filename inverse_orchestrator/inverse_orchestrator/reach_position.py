from inverse_msgs.srv import ReachPosition
from concurrent.futures import Future


class ReachPosition_Class:
    """Handles calling the ReachPosition service using an existing node (non-async)."""

    def __init__(self, node, srv_name="reach_position"):
        self.node = node
        self.client = self.node.create_client(ReachPosition, srv_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for ReachPosition service...")

    def execute_skill(
        self,
        final_pose,
        max_vel=0.15,
    ):
        """Call the ReachPosition service and return a concurrent.futures.Future."""
        req = ReachPosition.Request()
        req.desired_pos = final_pose
        req.max_vel = max_vel
        req.immediate_execution = True

        self.node.get_logger().warn(
            f"Reaching pose: {final_pose}, frame_id: {final_pose.header.frame_id} immediately: {req.immediate_execution}"
        )
        service_future = self.client.call_async(req)
        final_future = Future()

        def callback(fut):
            try:
                result = fut.result()
                final_future.set_result(result.success)
                # self.node.get_logger().info(
                #     f"Reach pose execution success: {result.success}"
                # )
            except Exception as e:
                final_future.set_result(False)
                # self.node.get_logger().error(f"Reach pose execution failed: {e}")

        service_future.add_done_callback(callback)
        return final_future

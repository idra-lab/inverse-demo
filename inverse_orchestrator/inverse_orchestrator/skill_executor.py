from inverse_msgs.srv import ExecuteSkill
from concurrent.futures import Future


class SkillExecutor:
    """Handles calling the ExecuteSkill service using an existing node (non-async)."""

    def __init__(self, node, srv_name="execute_skill"):
        self.node = node
        self.client = self.node.create_client(ExecuteSkill, srv_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for ExecuteSkill service...")

    def execute_skill(
        self,
        skill_name,
        initial_pose,
        final_pose,
        use_learned_initial=True,
        use_learned_final=False,
        max_vel=0.15,
    ):
        """Call the ExecuteSkill service and return a concurrent.futures.Future."""
        req = ExecuteSkill.Request()
        req.skill_name = skill_name
        req.use_learned_initial_pose = use_learned_initial
        req.use_learned_final_pose = use_learned_final
        req.initial_pose = initial_pose
        req.final_pose = final_pose
        req.max_vel = max_vel

        # self.node.get_logger().info(f"Executing skill: {skill_name}")

        service_future = self.client.call_async(req)
        final_future = Future()

        def callback(fut):
            try:
                result = fut.result()
                final_future.set_result(result.success)
                self.node.get_logger().warn(
                    f"Skill execution success: {result.success}"
                )
            except Exception as e:
                final_future.set_result(False)
                # self.node.get_logger().error(f"Skill execution failed: {e}")

        service_future.add_done_callback(callback)
        return final_future

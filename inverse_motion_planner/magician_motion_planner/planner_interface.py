import rclpy

from magician_motion_planner.planner_pose import PlannerPose
import magician_motion_planner.motion_wrappers as wrappers

from rclpy.node import Node
from magician_msgs.srv import (
    PointToPointMotion, ReachPosition, MeshReachPosition, MeshPointToPointMotion, HoldPosition
)
from std_srvs.srv import Trigger, SetBool
from typing import cast


class PlannerInterface(Node):

    def __init__(self):
        super().__init__("plan_service_caller")

        self.status_client = self._create_client_custom(SetBool, "set_broadcast_state")
        self.stop_client = self._create_client_custom(Trigger, "safe_stop")
        self.reach_client = self._create_client_custom(ReachPosition, "reach_position")
        self.mesh_reach_client = self._create_client_custom(MeshReachPosition, "mesh_reach_position")
        self.ptp_client = self._create_client_custom(
            PointToPointMotion, "execute_ptp_motion"
        )
        self.mesh_ptp_client = self._create_client_custom(
            MeshPointToPointMotion, "execute_mesh_ptp_motion"
        )
        self.hold_pos_client = self._create_client_custom(HoldPosition, "hold_position")
        self.get_logger().info("Service connection estabilished!")

    def _create_client_custom(self, srv_type, srv_name):
        cli = self.create_client(srv_type, "/motion_planner/" + srv_name)

        while not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f"Waiting for service {cli.srv_name} availability")

        return cli

    def enable_planner(self):
        req = SetBool.Request()
        req.data = True
        res_future = self.status_client.call_async(req)
        rclpy.spin_until_future_complete(self, res_future)
        res = cast(SetBool.Response, res_future.result())
        assert (res.success == True)

    def disable_mp(self):
        req = SetBool.Request()
        req.data = False
        res_future = self.status_client.call_async(req)
        rclpy.spin_until_future_complete(self, res_future)
        res = cast(SetBool.Response, res_future.result())
        assert (res.success == True)

    def stop(self):
        req = Trigger.Request()
        self.get_logger().info("Requesting motion stop")
        res_future = self.stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, res_future)
        res = cast(SetBool.Response, res_future.result())
        assert (res.success == True)

    def reach_pos(
        self,
        target: PlannerPose,
    ) -> wrappers.ReachPosition:
        motion = wrappers.ReachPosition(self, self.reach_client)
        motion.set_goal(target)
        return motion
    
    def mesh_reach_pos(
        self,
        mesh: str,  
        target: PlannerPose,
    ) -> wrappers.MeshReachPosition:
        motion = wrappers.MeshReachPosition(self, self.mesh_reach_client)
        motion.set_mesh(mesh)
        motion.set_goal(target)
        return motion

    def hold_position_for(self, hold_duration_secs: float) -> wrappers.HoldPosition:
        motion = wrappers.HoldPosition(self, self.hold_pos_client)
        motion.set_hold_duration(hold_duration_secs)
        return motion

    def linear_ptp(
        self,
        y0: PlannerPose,
        g: PlannerPose,
    ) -> wrappers.PointToPointMotion:
        motion = wrappers.PointToPointMotion(self, self.ptp_client)
        motion.set_initial_pose(y0)
        motion.set_final_pose(g)
        return motion

    def mesh_ptp(
        self,
        mesh: str,
        y0: PlannerPose,
        g: PlannerPose,
    ) -> wrappers.MeshPointToPointMotion:
        motion = wrappers.MeshPointToPointMotion(self, self.mesh_ptp_client)
        motion.set_mesh(mesh)
        motion.set_initial_pose(y0)
        motion.set_final_pose(g)
        return motion

import rclpy
import magician_msgs.srv as srvs
import magician_msgs.msg as msgs

from rclpy.node import Node
from typing import TypeVar, Generic, cast
from magician_motion_planner.planner_pose import PlannerPose
from magician_motion_planner.motion_mixins import ControlMixin, SanderMixin


RequestT = TypeVar("RequestT")
ResponseT = TypeVar("ResponseT")


class Motion(Generic[RequestT, ResponseT]):

    request_type: type[RequestT]
    response_type: type[ResponseT]

    def __init__(self, node: Node, service_client):
        self.node = node
        self.client = service_client
        self.msg: RequestT = self.request_type()

    def enqueue(self):
        res_future = self.client.call_async(self.msg)
        rclpy.spin_until_future_complete(self.node, res_future)
        return cast(ResponseT, res_future.result())


class ReachPosition(
    Motion[srvs.ReachPosition.Request, srvs.ReachPosition.Response],
    ControlMixin,
    SanderMixin
):

    request_type = srvs.ReachPosition.Request
    response_type = srvs.ReachPosition.Response

    def set_goal(self, g: PlannerPose) -> "ReachPosition":
        self.msg.desired_pos = g.to_pose_stamped_msg()
        return self

    def set_maximum_velocity(self, max_vel: float) -> "ReachPosition":
        self.msg.max_vel = max_vel
        return self

    def abort_current_motions(self):
        self.msg.immediate_execution = True
        return self
    
    def sander_control(self, activate_sander: msgs.SanderControl) -> "ReachPosition":
        self.msg.sander.commanded_state = activate_sander
        return self
    
class MeshReachPosition(
    Motion[srvs.MeshReachPosition.Request, srvs.MeshReachPosition.Response],
    ControlMixin,
    SanderMixin
):

    request_type = srvs.MeshReachPosition.Request
    response_type = srvs.MeshReachPosition.Response

    def set_mesh(self, mesh: str) -> "MeshReachPosition":
        self.msg.mesh = mesh
        return self

    def set_goal(self, g: PlannerPose) -> "MeshReachPosition":
        self.msg.desired_pos = g.to_pose_stamped_msg()
        return self

    def set_maximum_velocity(self, max_vel: float) -> "MeshReachPosition":
        self.msg.max_vel = max_vel
        return self
    
    def mesh_offset(self, distance_m: float) -> "MeshReachPosition":
        self.msg.offset = distance_m
        return self
    
    def control_mode(self, control_mode: msgs.ControlMode) -> "MeshReachPosition":
        self.msg.control_mode = control_mode
        return self
    
    def sander_control(self, activate_sander: msgs.SanderControl) -> "MeshReachPosition":
        self.msg.sander.commanded_state = activate_sander
        return self


class PointToPointMotion(
    Motion[srvs.PointToPointMotion.Request, srvs.PointToPointMotion.Response],
    ControlMixin,
    SanderMixin
):

    request_type = srvs.PointToPointMotion.Request
    response_type = srvs.PointToPointMotion.Response

    def set_initial_pose(self, y0: PlannerPose) -> "PointToPointMotion":
        self.msg.y0 = y0.to_pose_stamped_msg()
        return self

    def set_final_pose(self, g: PlannerPose) -> "PointToPointMotion":
        self.msg.g = g.to_pose_stamped_msg()
        return self

    def set_maximum_velocity(self, max_vel: float) -> "PointToPointMotion":
        self.msg.max_vel = max_vel
        return self

    def preplan_joining_motion(self) -> "PointToPointMotion":
        self.msg.plan_y0_motion = True
        return self


class MeshPointToPointMotion(
    Motion[srvs.MeshPointToPointMotion.Request, srvs.MeshPointToPointMotion.Response],
    ControlMixin,
    SanderMixin
):

    request_type = srvs.MeshPointToPointMotion.Request
    response_type = srvs.MeshPointToPointMotion.Response

    def set_mesh(self, mesh: str) -> "MeshPointToPointMotion":
        self.msg.mesh = mesh
        return self

    def set_initial_pose(self, y0: PlannerPose) -> "MeshPointToPointMotion":
        self.msg.y0 = y0.to_pose_stamped_msg()
        return self

    def set_final_pose(self, g: PlannerPose) -> "MeshPointToPointMotion":
        self.msg.g = g.to_pose_stamped_msg()
        return self

    def set_maximum_velocity(self, max_vel: float) -> "MeshPointToPointMotion":
        self.msg.max_vel = max_vel
        return self

    def set_approach_velocity(self, vel: float) -> "MeshPointToPointMotion":
        self.msg.approach_vel = vel
        return self

    def preplan_joining_motion(self) -> "MeshPointToPointMotion":
        self.msg.plan_y0_motion = True
        return self
    
    def preplan_joining_motion_no(self) -> "MeshPointToPointMotion":
        self.msg.plan_y0_motion = False
        return self

    def mesh_offset(self, distance_m: float) -> "MeshPointToPointMotion":
        self.msg.offset = distance_m
        return self
    
    def control_mode(self, control_mode: msgs.ControlMode) -> "MeshPointToPointMotion":
        self.msg.control_mode = control_mode
        return self
    
    def sander_control(self, activate_sander: msgs.SanderControl) -> "MeshPointToPointMotion":
        self.msg.sander.commanded_state = activate_sander
        return self


class HoldPosition(
    Motion[srvs.HoldPosition.Request, srvs.MeshPointToPointMotion.Response],
    ControlMixin,
    SanderMixin
):

    request_type = srvs.HoldPosition.Request
    response_type = srvs.HoldPosition.Response

    def set_hold_duration(self, duration_sec: float) -> "HoldPosition":
        self.msg.wait_sec = duration_sec
        return self
    
    def control_mode(self, control_mode: msgs.ControlMode) -> "HoldPosition":
        self.msg.control_mode = control_mode
        return self
    
    def sander_control(self, activate_sander: msgs.SanderControl) -> "HoldPosition":
        self.msg.sander.commanded_state = activate_sander
        return self
    

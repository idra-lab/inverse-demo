#!/usr/bin/python3

import numpy as np
import rclpy

from magician_motion_planner import PlannerInterface, PlannerPose
from magician_msgs.msg import ControlMode


def main(args=None):
    rclpy.init(args=args)
    node = PlannerInterface()

    node.enable_planner()

    q_pose = np.array([0.0, 0.0, -0.7071068, 0.7071068])
    pose1 = PlannerPose(
        np.array([0.1, 0.2, 0.55]), q_pose, "right_back_door_simple_offset"
    )
    pose2 = PlannerPose(
        np.array([0.3, 0.2, 0.55]), q_pose, "right_back_door_simple_offset"
    )

    node.mesh_reach_pos("right_back_door_simple_offset", pose1).mesh_offset(0.05).set_maximum_velocity(0.07).enqueue()

    #tactile loop sensing
    control_mode = ControlMode()
    control_mode.control_mode = ControlMode.CONTROL_MODE_ADMITTANCE
    control_mode.admittance_reference_mode = ControlMode.ADMITTANCE_CONSTANT_REFERENCE
    control_mode.reference_force = 5.0
    node.mesh_reach_pos("right_back_door_simple_offset", pose1).mesh_offset(0.0)\
        .set_maximum_velocity(0.01)\
        .control_mode(control_mode)\
        .enqueue()

    node.mesh_ptp("right_back_door_simple_offset", pose1, pose2)\
                    .mesh_offset(0.0)\
                    .preplan_joining_motion_no()\
                    .set_maximum_velocity(0.02)\
                    .set_approach_velocity(0.01)\
                    .control_mode(control_mode)\
                    .enqueue()
    
    node.hold_position_for(2.0).enqueue()

    node.mesh_ptp("right_back_door_simple_offset", pose2,pose1)\
                .mesh_offset(0.0)\
                .preplan_joining_motion_no()\
                .set_maximum_velocity(0.05)\
                .set_approach_velocity(0.01)\
                .control_mode(control_mode)\
                .enqueue()

    #END : go up
    # control_mode.control_mode = ControlMode.CONTROL_MODE_IMPEDANCE
    # node.mesh_reach_pos("right_back_door_simple_offset", pose1)\
    #     .control_mode(control_mode)\
    #     .mesh_offset(0.10)\
    #     .set_maximum_velocity(0.07)\
    #     .enqueue()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

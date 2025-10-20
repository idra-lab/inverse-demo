#!/usr/bin/python3

import numpy as np
import rclpy

from magician_motion_planner import PlannerInterface, PlannerPose
from magician_msgs.msg import SanderControl, ControlMode

def main(args=None):
    rclpy.init(args=args)
    node = PlannerInterface()

    node.enable_planner()

    q_pose = np.array([0.0, 0.0, -0.7071068, 0.7071068])
    pose1 = PlannerPose(
        np.array([0.1, 0.2, 0.7]), q_pose, "right_back_door_simple_offset"
    )

    pose2 = PlannerPose(
        np.array([0.4, 0.2, 0.5]), q_pose, "right_back_door_simple_offset"
    )
    

    control_mode = ControlMode()
    control_mode.control_mode = ControlMode.CONTROL_MODE_ADMITTANCE
    control_mode.admittance_reference_mode = ControlMode.ADMITTANCE_CONSTANT_REFERENCE
    control_mode.reference_force = 5.0

    node.mesh_reach_pos("right_back_door_simple_offset", pose1).mesh_offset(0.10).set_maximum_velocity(0.07).enqueue()
    node.mesh_reach_pos("right_back_door_simple_offset", pose1)\
        .mesh_offset(0.0)\
        .set_maximum_velocity(0.02)\
        .control_mode(control_mode)\
        .enqueue()
    node.hold_position_for(5.0).enqueue() #wait for reaching the desired force
    node.hold_position_for(5.0).sander_control(SanderControl.SANDER_CMD_ENABLE).enqueue()

    #we go back to impedance mode to go up and then reach the other point. 
    #this is important because if we change the reference force while keeping always the admittance mode a jump happens
    control_mode.control_mode = ControlMode.CONTROL_MODE_IMPEDANCE
    node.mesh_reach_pos("right_back_door_simple_offset", pose1)\
        .mesh_offset(0.15).set_maximum_velocity(0.07).control_mode(control_mode)\
            .sander_control(SanderControl.SANDER_CMD_DISABLE).enqueue()
    

    node.mesh_reach_pos("right_back_door_simple_offset", pose2).mesh_offset(0.10).set_maximum_velocity(0.07).enqueue()
    control_mode.control_mode = ControlMode.CONTROL_MODE_ADMITTANCE
    control_mode.reference_force = 10.0
    node.mesh_reach_pos("right_back_door_simple_offset", pose2)\
        .mesh_offset(0.0)\
        .set_maximum_velocity(0.02)\
        .control_mode(control_mode)\
        .enqueue()
    node.hold_position_for(5.0).enqueue() #wait for reaching the desired force
    node.hold_position_for(3.0).sander_control(SanderControl.SANDER_CMD_ENABLE).enqueue()
    node.mesh_reach_pos("right_back_door_simple_offset", pose2)\
         .mesh_offset(0.15).set_maximum_velocity(0.07).sander_control(SanderControl.SANDER_CMD_DISABLE).enqueue()



    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

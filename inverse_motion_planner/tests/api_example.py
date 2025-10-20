#!/usr/bin/python3

import numpy as np
import rclpy

from magician_motion_planner import PlannerInterface, PlannerPose


def main(args=None):
    rclpy.init(args=args)
    node = PlannerInterface()

    node.enable_planner()

    q_pose = np.array([0.0, 0.0, -0.7071068, 0.7071068])
    pose1 = PlannerPose(
        np.array([0.1, 0.2, 0.70]), q_pose, "right_back_door_simple_offset"
    )
    pose2 = PlannerPose(
        np.array([0.3, 0.2, 0.33]), q_pose, "right_back_door_simple_offset"
    )

    #Reach an initial pose
    node.reach_pos(pose1).set_maximum_velocity(0.10).enqueue()

    #Wait 
    node.hold_position_for(5.0).enqueue()

    # point-to-point mesh following, 
    # reach mesh in the nearest point from pose1 (with offset)
    # then go to pose2 (with offset)
    node.mesh_ptp("right_back_door_simple_offset", pose1,
                  pose2).mesh_offset(0.05).preplan_joining_motion(
                  ).set_maximum_velocity(0.08).set_approach_velocity(0.10).enqueue()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

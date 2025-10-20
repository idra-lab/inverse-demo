#!/usr/bin/python3

import numpy as np
import rclpy

from magician_motion_planner import PlannerInterface, PlannerPose


def main(args=None):
    rclpy.init(args=args)
    node = PlannerInterface()

    node.enable_planner()

    q_pose = np.array([0.0, 1.0, 0.0, 0.0])
    pose1 = PlannerPose(
        np.array([0.2, 0.3, 0.3]), q_pose, "base_link"
    )
    node.reach_pos(pose1).set_maximum_velocity(0.10).enqueue()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
from inverse_msgs.srv import MoveRelative
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from inverse_orchestrator.skill_executor import SkillExecutor
from inverse_orchestrator.ur_gripper_controller import URGripper
import time

from inverse_orchestrator.smpl_model import SMPLModel

HUMAN_DISTANCE_TRIGGER = 0.40 # meters


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # Instantiate action modules with this node

        # # franka right
        self.gripper = URGripper(node=self)
        self.get_logger().info("GripperTestNode initialized.")


        self.execute_learned_skill = SkillExecutor(self, "/execute_skill")
        self.execute_reach_pose = ReachPosition_Class(self, "/reach_position")
        self.base_link = "base_link"
        self.target_link = "target_link"
        

        self.kit1_connector_grasp_frame_name = "kit1_connector_grasp"
        self.kit1_connector_deposit_frame_name = "kit1_connector_deposit"
        self.kit1_screw1_frame_name = "kit1_screw1"
        self.kit1_screw2_frame_name = "kit1_screw2"
        self.kit1_screw1_deposit_frame_name = "kit1_screw1_deposit"
        self.kit1_screw2_deposit_frame_name = "kit1_screw2_deposit"

        self.get_logger().info("Orchestrator initialized and ready.")

        # self.smpl = SMPLModel(self, "/smpl_markers")

        # WAIT SMPL
        # while self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit") is not None:
        #     self.get_logger().warn("Waiting for SMPL model data...")
        #     time.sleep(0.5)

        self.get_logger().info("\n\n\n\n\n----------------\nStarting orchestrator...")

    def orchestrate(self):
        # self.task1()  
        self.task1()

    def task1(self):
        """Example orchestration routine combining primitives."""

        # while rclpy.ok():
        #     data = self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit")
        #     if data is None:
        #         self.get_logger().info(f"None data!")
        #         continue
        #     if data < HUMAN_DISTANCE_TRIGGER:
        #         self.get_logger().info("Unsafe")
        #     else:
        #         self.get_logger().info("Safe")

        #     time.sleep(0.1)


        # Screw pose
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        end_pose.pose.position.z = -0.10

        # # OPEN GRIPPER
        input("Press Enter to move gripper")
        self.gripper.open()

        # Move down using the move relative
        # # REACH POSE
        self.execute_reach_pose.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        input()

        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        end_pose.pose.position.z = 0.0

        self.execute_reach_pose.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # time.sleep(2)
        # time.sleep(2)
        # time.sleep(2)
        # time.sleep(2)
        # time.sleep(2)


        # end_pose.pose.position.z = 0.0
        # # input("Press Enter to move robot above kit1 connector.")
        # reach_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # # GRASP CONNECTOR
        # input("Press Enter to close gripper")
        # close_future = self.gripper_right.close_gripper()
        # time.sleep(2)
        # end_pose = PoseStamped()
        # end_pose.header.frame_id = self.kit1_frame_name

        # # MOVE TO DEPOSIT
        # end_pose.pose.position.z = -0.10
        # end_pose.header.frame_id = self.kit1_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = -0.20
        # end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = 0.0
        # end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # # OPEN GRIPPER
        # input("Press Enter to move gripper")
        # self.gripper_right.move_finger(width=0.03, speed=0.1)
        # time.sleep(2)
        # end_pose.pose.position.z = -0.30
        # end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # # MOVE TO SCREW 1
        # end_pose.pose.position.z = -0.10
        # end_pose.header.frame_id = self.kit1_screw1_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = -0.0
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # # GRASP SCREW 1
        # input("Press Enter to close gripper on screw 1.")
        # move_future = self.gripper_right.close_gripper(width=0.006, speed=0.1)
        # time.sleep(2)

        # end_pose.pose.position.z = -0.10
        # end_pose.header.frame_id = self.kit1_screw1_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # # DEPOSIT SCREW 1
        # end_pose.pose.position.z = -0.15
        # end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = 0.0
        # end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # # OPEN GRIPPER
        # input("Press Enter to move gripper")
        # self.gripper_right.move_finger(width=0.019, speed=0.1)
        # time.sleep(2)

        # end_pose.pose.position.z = -0.40
        # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # # #############

        # # MOVE TO SCREW 2
        # end_pose.pose.position.z = -0.10
        # end_pose.header.frame_id = self.kit1_screw2_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = -0.0
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # # GRASP SCREW 2
        # input("Press Enter to close gripper on screw 2.")
        # move_future = self.gripper_right.close_gripper(width=0.0, speed=0.1)
        # time.sleep(2)

        # end_pose.header.frame_id = self.kit1_screw2_frame_name
        # end_pose.pose.position.z = -0.10
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # # WAIT HUMAN BEFORE DEPOSITING
        # while rclpy.ok():
        #     dist =  self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit")

        #     while (dist is None or dist < HUMAN_DISTANCE_TRIGGER ):
        #         self.get_logger().info(
        #             "Waiting for human to move away before depositing screw 2..."
        #         )
        #         dist =  self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit")
        #         time.sleep(0.1)

        #     break

        # # DEPOSIT SCREW 2
        # end_pose.pose.position.z = -0.15
        # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # end_pose.pose.position.z = 0.0
        # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )
        # # OPEN GRIPPER
        # input("Press Enter to move gripper")
        # self.gripper_right.move_finger(width=0.01, speed=0.1)
        # time.sleep(1)

        # self.get_logger().info(
        #     "\n\n\nHuman moved away, continuing orchestration..."
        # )

        # # DEPOSIT SCREW 2
        # end_pose.pose.position.z = -0.35
        # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )

        # end_pose.pose.position.z = -0.1
        # end_pose.header.frame_id = self.kit1_screw1_frame_name
        # skill_future = self.reach_pose_right.execute_skill(
        #     final_pose=end_pose, max_vel=0.1
        # )


def main():
    rclpy.init()
    orchestrator = Orchestrator()

    executor = MultiThreadedExecutor()
    executor.add_node(orchestrator)

    # Start orchestration in a separate thread to avoid blocking executor
    import threading

    threading.Thread(target=orchestrator.orchestrate, daemon=True).start()

    try:
        executor.spin()
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

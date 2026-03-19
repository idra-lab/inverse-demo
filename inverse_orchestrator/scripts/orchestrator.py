#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
from inverse_msgs.srv import MoveRelative
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform
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

        self.gripper = URGripper(node=self)
        self.get_logger().info("Orchestrator initialized.")

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


        
        move_relative_client = self.create_client(MoveRelative, "/move_relative")
        while not move_relative_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveRelative service...")
        move_relative_request = MoveRelative.Request()
        move_relative_request.relative_motion = Transform()

        end_pose = PoseStamped()


        # Set UP the initial gripper: open

        # input("Press Enter to open gripper")
        # self.gripper.open()
        # time.sleep(2)

        # # ── 1. CONNECTOR ──────────────────────────────────────────────────────

        # OPEN GRIPPER
        input("Press Enter to open gripper")
        self.gripper.open()
        time.sleep(2)

        # # MOVE TO CONNECTOR GRASP POSE
        end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
        end_pose.pose.position.z = +0.05
        __future = self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        time.sleep(10.0)

        # if __future.done():
        #     print(__future.result())
        #     self.get_logger().info(f"ReachPosition response")
        # else:
        #     self.get_logger().error(f"Service call faileed")   

        # MOVE DOWN TO GRASP CONNECTOR
        move_relative_request.relative_motion.translation.y = -0.2
        move_future = move_relative_client.call_async(move_relative_request)
        rclpy.spin_until_future_complete(self, move_future)
        if move_future.result() is not None:
            self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        else:
            self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # CLOSE GRIPPER TO GRASP CONNECTOR
        # input("Press Enter to close gripper on connector.")
        # self.gripper.close()
        # time.sleep(2)

        # # MOVE UP
        # move_relative_request.relative_motion.translation.z = -0.15
        # move_future = move_relative_client.call_async(move_relative_request)
        # rclpy.spin_until_future_complete(self, move_future)
        # if move_future.result() is not None:
        #     self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # MOVE TO CONNECTOR DEPOSIT
        # end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        # end_pose.pose.position.z = -0.10
        # self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        # # OPEN GRIPPER TO DEPOSIT CONNECTOR
        # input("Press Enter to open gripper")
        # self.gripper.open()
        # time.sleep(2)

        # # ── 2. SCREW 1 ────────────────────────────────────────────────────────

        # # MOVE TO KIT1 SCREW 1
        # end_pose.header.frame_id = self.kit1_screw1_frame_name
        # end_pose.pose.position.z = -0.10
        # self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        # # MOVE DOWN TO GRASP KIT1 SCREW 1
        # move_relative_request.relative_motion.translation.z = 0.07
        # move_future = move_relative_client.call_async(move_relative_request)
        # rclpy.spin_until_future_complete(self, move_future)
        # if move_future.result() is not None:
        #     self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # CLOSE GRIPPER TO GRASP KIT1 SCREW 1
        # input("Press Enter to close gripper on screw 1.")
        # self.gripper.close()
        # time.sleep(2)

        # # MOVE UP
        # move_relative_request.relative_motion.translation.z = -0.15
        # move_future = move_relative_client.call_async(move_relative_request)
        # rclpy.spin_until_future_complete(self, move_future)
        # if move_future.result() is not None:
        #     self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # MOVE TO SCREW 1 DEPOSIT
        # end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        # end_pose.pose.position.z = -0.10
        # self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        # # OPEN GRIPPER TO DEPOSIT SCREW 1
        # input("Press Enter to open gripper")
        # self.gripper.open()
        # time.sleep(2)

        # # ── 3. SCREW 2 ────────────────────────────────────────────────────────

        # # MOVE TO KIT1 SCREW 2
        # end_pose.header.frame_id = self.kit1_screw2_frame_name
        # end_pose.pose.position.z = -0.10
        # self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        # # MOVE DOWN TO GRASP KIT1 SCREW 2
        # move_relative_request.relative_motion.translation.z = 0.07
        # move_future = move_relative_client.call_async(move_relative_request)
        # rclpy.spin_until_future_complete(self, move_future)
        # if move_future.result() is not None:
        #     self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # CLOSE GRIPPER TO GRASP KIT1 SCREW 2
        # input("Press Enter to close gripper on screw 2.")
        # self.gripper.close()
        # time.sleep(2)

        # # MOVE UP
        # move_relative_request.relative_motion.translation.z = -0.15
        # move_future = move_relative_client.call_async(move_relative_request)
        # rclpy.spin_until_future_complete(self, move_future)
        # if move_future.result() is not None:
        #     self.get_logger().info(f"MoveRelative response: {move_future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed: {move_future.exception()}")

        # # MOVE TO SCREW 2 DEPOSIT
        # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        # end_pose.pose.position.z = -0.10
        # self.execute_reach_pose.execute_skill(final_pose=end_pose, max_vel=0.1)

        # # OPEN GRIPPER TO DEPOSIT SCREW 2
        # input("Press Enter to open gripper")
        # self.gripper.open()
        # time.sleep(2)


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

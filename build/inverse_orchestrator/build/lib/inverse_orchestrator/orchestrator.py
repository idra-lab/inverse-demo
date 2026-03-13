#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from inverse_orchestrator.skill_executor import SkillExecutor
from inverse_orchestrator.gripper_controller import GripperController
import time

from inverse_orchestrator.smpl_model import SMPLModel

HUMAN_DISTANCE_TRIGGER = 0.40 # meters


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # Instantiate action modules with this node

        # # franka right
        self.skill_exec_right = SkillExecutor(self, "right_planner/execute_skill")
        self.gripper_right = GripperController(
            self, "franka1/franka_gripper/grasp", "franka1/franka_gripper/move"
        )
        self.reach_pose_right = ReachPosition_Class(
            self, "right_planner/reach_position"
        )
        self.base_link1 = "franka1_fr3_link0"

        # franka left
        self.skill_exec_left = SkillExecutor(self, "left_planner/execute_skill")
        self.gripper_left = GripperController(
            self, "franka2/franka_gripper/grasp", "franka2/franka_gripper/move"
        )
        self.reach_pose_left = ReachPosition_Class(self, "left_planner/reach_position")
        self.base_link2 = "franka2_fr3_link0"

        self.kit1_frame_name = "kit1_connector_grasp"
        self.kit1_connector_deposit_frame_name = "kit1_connector_deposit"
        self.kit1_screw1_frame_name = "kit1_screw1"
        self.kit1_screw2_frame_name = "kit1_screw2"
        self.kit1_screw1_deposit_frame_name = "kit1_screw1_deposit"
        self.kit1_screw2_deposit_frame_name = "kit1_screw2_deposit"

        self.get_logger().info("Orchestrator initialized and ready.")

        self.smpl = SMPLModel(self, "/smpl_markers")

        # WAIT SMPL
        # while self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit") is not None:
        #     self.get_logger().warn("Waiting for SMPL model data...")
        #     time.sleep(0.5)

        self.get_logger().info("\n\n\n\n\n----------------\nStarting orchestrator...")

    def orchestrate(self):
        # self.task1()  
        self.task2()

    def task2(self):

        #     time.sleep(0.1)
        pose = PoseStamped()


        MAX_VEL = 0.07
        input("Press to start")
        self.gripper_left.move_finger(width=0.037, speed=0.08)
        pose.header.frame_id = "kit2_connector_grasp"
        self.skill_exec_left.execute_skill("home_to_kit2_connector", max_vel=MAX_VEL, final_pose=pose)

        input("Close the gripper")
        os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_z 3000.0")
        close_future = self.gripper_left.close_gripper(force=70.0)
        time.sleep(3)
        self.skill_exec_left.execute_skill("kit2_connector_grasp_prepare_peg", max_vel=MAX_VEL)
        # self.skill_exec_left.execute_skill("kit2_connector_to_hole", max_vel=0.09, final_pose=peg_pose)
        # pose.header.frame_id = "kit2_connector_deposit"
        # self.skill_exec_left.execute_skill("kit2_connector_peg_in_hole", final_pose=pose,max_vel=0.03)
        pose.header.frame_id = "kit2_connector_deposit"
        pose.pose.position.z = -0.05
        self.reach_pose_left.execute_skill(pose, max_vel=0.015)
        pose.pose.position.z = 0.0
        self.reach_pose_left.execute_skill(pose, max_vel=0.015)
        
        input("Press to lower stiffness")
        os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_x 4000.0")
        os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_y 4000.0")
        os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_z 800.0")
        pose = PoseStamped()
        pose.header.frame_id = "kit2_connector_deposit"
        self.skill_exec_left.execute_skill("peg_in_hole", final_pose=pose)

        while rclpy.ok():
            data = self.smpl.get_keypoints_shortest_distance("kit2_connector_deposit")
            if data is None:
                self.get_logger().info(f"None data!")
                continue
            if data < HUMAN_DISTANCE_TRIGGER:
                self.get_logger().info("Unsafe: Human close -> Stopping peg in hole")
                time.sleep(1.0)
                break
            else:
                self.get_logger().info("Safe: Human far")
        
        os.system("ros2 service call /left_planner/safe_stop std_srvs/srv/Trigger \{\}")
        self.get_logger().warn("\nSafe stop triggered! Human is screwing")

        while rclpy.ok():
            data = self.smpl.get_keypoints_shortest_distance("kit2_connector_deposit")
            if data is None:
                self.get_logger().info(f"None data!")
                continue
            if data > HUMAN_DISTANCE_TRIGGER:
                self.get_logger().info(f"Safe: Continuin execution as distance is: {data}")
                time.sleep(1.0)
                break
            else:
                self.get_logger().info(f"Unsafe: Human is screwing as distance is: {data}")
                time.sleep(1.0)
        
        self.gripper_left.move_finger(width=0.05, speed=0.15)
        input("Press to move away")

        pose = PoseStamped()
        pose.header.frame_id = "kit2_connector_deposit"
        pose.pose.position.z = -0.08
        self.reach_pose_left.execute_skill(pose, max_vel=0.015)



        


        # os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_r 300.0")
        # os.system("ros2 param set /left_cartesian_impedance_controller stiffness.trans_z 300.0")


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

        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_frame_name
        end_pose.pose.position.z = -0.10

        # OPEN GRIPPER
        input("Press Enter to move gripper")
        self.gripper_right.move_finger(width=0.03, speed=0.15)

        # REACH POSE
        reach_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = 0.0
        # input("Press Enter to move robot above kit1 connector.")
        reach_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # GRASP CONNECTOR
        input("Press Enter to close gripper")
        close_future = self.gripper_right.close_gripper()
        time.sleep(2)
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_frame_name

        # MOVE TO DEPOSIT
        end_pose.pose.position.z = -0.10
        end_pose.header.frame_id = self.kit1_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = -0.20
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = 0.0
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # OPEN GRIPPER
        input("Press Enter to move gripper")
        self.gripper_right.move_finger(width=0.03, speed=0.1)
        time.sleep(2)
        end_pose.pose.position.z = -0.30
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        # MOVE TO SCREW 1
        end_pose.pose.position.z = -0.10
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = -0.0
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        # GRASP SCREW 1
        input("Press Enter to close gripper on screw 1.")
        move_future = self.gripper_right.close_gripper(width=0.006, speed=0.1)
        time.sleep(2)

        end_pose.pose.position.z = -0.10
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # DEPOSIT SCREW 1
        end_pose.pose.position.z = -0.15
        end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = 0.0
        end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        # OPEN GRIPPER
        input("Press Enter to move gripper")
        self.gripper_right.move_finger(width=0.019, speed=0.1)
        time.sleep(2)

        end_pose.pose.position.z = -0.40
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # #############

        # MOVE TO SCREW 2
        end_pose.pose.position.z = -0.10
        end_pose.header.frame_id = self.kit1_screw2_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = -0.0
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        # GRASP SCREW 2
        input("Press Enter to close gripper on screw 2.")
        move_future = self.gripper_right.close_gripper(width=0.0, speed=0.1)
        time.sleep(2)

        end_pose.header.frame_id = self.kit1_screw2_frame_name
        end_pose.pose.position.z = -0.10
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        # WAIT HUMAN BEFORE DEPOSITING
        while rclpy.ok():
            dist =  self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit")

            while (dist is None or dist < HUMAN_DISTANCE_TRIGGER ):
                self.get_logger().info(
                    "Waiting for human to move away before depositing screw 2..."
                )
                dist =  self.smpl.get_keypoints_shortest_distance("kit1_screw2_deposit")
                time.sleep(0.1)

            break

        # DEPOSIT SCREW 2
        end_pose.pose.position.z = -0.15
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        end_pose.pose.position.z = 0.0
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )
        # OPEN GRIPPER
        input("Press Enter to move gripper")
        self.gripper_right.move_finger(width=0.01, speed=0.1)
        time.sleep(1)

        self.get_logger().info(
            "\n\n\nHuman moved away, continuing orchestration..."
        )

        # DEPOSIT SCREW 2
        end_pose.pose.position.z = -0.35
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        end_pose.pose.position.z = -0.1
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

    # def orchestrate(self):
    #     """Example orchestration routine combining primitives."""
    #     # # # Prepare sample poses
    #     start_pose = PoseStamped()
    #     start_pose.header.frame_id = self.base_link1
    #     end_pose = PoseStamped()
    #     end_pose.header.frame_id = self.kit1_frame_name
    #     end_pose.pose.position.x = 0.0
    #     end_pose.pose.position.y = 0.0
    #     end_pose.pose.position.z = 0.0
    #     end_pose.pose.orientation.x = 0.0
    #     end_pose.pose.orientation.y = 0.0
    #     end_pose.pose.orientation.z = 0.0
    #     end_pose.pose.orientation.w = 1.0

    #     # OPEN GRIPPER
    #     input("Press Enter to move gripper")
    #     move_future = self.gripper_right.move_finger(width=0.03, speed=0.1)
    #     # MOVE TO CONNECTOR
    #     # input("Press Enter to move robot.")
    #     skill_future = self.skill_exec_right.execute_skill(
    #         # "home_to_kit1", start_pose, end_pose, max_vel=0.15
    #         "random",
    #         start_pose,
    #         end_pose,
    #         max_vel=0.15,
    #     )
    # GRASP CONNECTOR
    # input("Press Enter to close gripper")
    # close_future = self.gripper_right.close_gripper()
    # end_pose = PoseStamped()
    # end_pose.header.frame_id = self.kit1_connector_deposit_frame_name

    # # MOVE TO DEPOSIT
    # input("Press Enter to move robot to holder.")
    # skill_future = self.skill_exec_right.execute_skill(
    #     "kit1_to_deposit", start_pose, end_pose, max_vel=0.15
    # )
    # # DROP CONNECTOR
    # input("Press Enter to open gripper")
    # open_future = self.gripper_right.open_gripper()

    # # GOTO TO SCREW 1
    # input("Press Enter to move robot to screw 1.")
    # end_pose = PoseStamped()
    # end_pose.header.frame_id = self.kit1_screw1_frame_name
    # skill_future = self.skill_exec_right.execute_skill(
    #     "deposit_to_kit1_screw1",
    #     start_pose,
    #     end_pose,
    #     max_vel=0.15,
    #     # use_learned_final=True,
    #     # use_learned_initial=True,
    # )
    # # # GRASP SCREW 1

    # input("Press Enter to close gripper on screw 1.")
    # move_future = self.gripper_right.move_finger(width=0.015, speed=0.1)
    # # input("Press Enter to close gripper on screw 1.")
    # time.sleep(3.0)
    # close_future = self.gripper_right.close_gripper()
    # input("Press Enter to move robot to deposit screw 1.")
    # skill_future = self.skill_exec_right.execute_skill(
    #     "kit1_screw1_to_deposit",
    #     start_pose,
    #     end_pose,
    #     max_vel=0.15,
    # )

    # # # Sequence actions
    # # # Open gripper
    # # # self.get_logger().info("Opening gripper...")
    # # close_future = self.gripper_right.close_gripper()


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

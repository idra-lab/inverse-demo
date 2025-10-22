#!/usr/bin/env python3
from inverse_orchestrator.reach_position import ReachPosition, ReachPosition_Class
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from inverse_orchestrator.skill_executor import SkillExecutor
from inverse_orchestrator.gripper_controller import GripperController
import time


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")

        # Instantiate action modules with this node

        # franka right
        self.skill_exec_right = SkillExecutor(self, "right_planner/execute_skill")
        # self.gripper_right = GripperController(
        #     self, "franka1/franka_gripper/grasp", "franka1/franka_gripper/move"
        # )
        self.reach_pose_right = ReachPosition_Class(
            self, "right_planner/reach_position"
        )
        self.base_link1 = "franka1_fr3_link0"
        # franka left
        self.skill_exec_left = SkillExecutor(self, "left_planner/execute_skill")
        # self.gripper_left = GripperController(
        #     self, "franka2/franka_gripper/grasp", "franka2/franka_gripper/move"
        # )
        # self.reach_pose_left = ReachPosition(self, "left_planner/reach_position")
        self.base_link2 = "franka2_fr3_link0"

        self.kit1_frame_name = "kit1_connector_grasp"
        self.kit1_connector_deposit_frame_name = "kit1_connector_deposit"
        self.kit1_screw1_frame_name = "kit1_screw1"
        self.kit1_screw2_frame_name = "kit1_screw2"
        self.kit1_screw1_deposit_frame_name = "kit1_screw1_deposit"
        self.kit1_screw2_deposit_frame_name = "kit1_screw2_deposit"

        self.get_logger().info("Orchestrator initialized and ready.")

    def orchestrate(self):
        """Example orchestration routine combining primitives."""

        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_frame_name
        end_pose.pose.position.z = -0.10

        # OPEN GRIPPER
        input("Press Enter to move gripper")
        # self.gripper_right.move_finger(width=0.03, speed=0.15)

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
        # close_future = self.gripper_right.close_gripper()
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_frame_name

        # MOVE TO DEPOSIT
        input("Press Enter to move robot to holder.")
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
        # self.gripper_right.move_finger(width=0.03, speed=0.1)
        input("Press Enter to move robot to holder.")
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
        # move_future = self.gripper_right.close_gripper(width=0.015, speed=0.1)
        input("Press Enter to move robot to deposit screw 1.")

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
        # self.gripper_right.move_finger(width=0.03, speed=0.1)
        input("Press Enter to move gripper")

        end_pose.pose.position.z = -0.30
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

        #############

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
        # move_future = self.gripper_right.close_gripper(width=0.015, speed=0.1)
        input("Press Enter to move robot to deposit screw 2.")

        end_pose.header.frame_id = self.kit1_screw2_frame_name
        end_pose.pose.position.z = -0.10
        skill_future = self.reach_pose_right.execute_skill(
            final_pose=end_pose, max_vel=0.1
        )

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
        # self.gripper_right.move_finger(width=0.03, speed=0.1)
        input("Press Enter to move gripper")

        end_pose.pose.position.z = -0.30
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

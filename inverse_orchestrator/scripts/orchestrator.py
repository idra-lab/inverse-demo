#!/usr/bin/env python3
import random
from scripts.reach_position import ReachPosition, ReachPosition_Class
from inverse_msgs.srv import MoveRelative
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform
from rclpy.executors import MultiThreadedExecutor
from scripts.skill_executor import SkillExecutor
from scripts.ur_gripper_controller import URGripper
import time
import threading
import tf2_ros
from tf2_ros import TransformException
import math
from scripts.smpl_model import SMPLModel

HUMAN_DISTANCE_TRIGGER = 0.40  # meters



velocity=random.uniform(0.1, 0.3)
print(f"Using velocity {velocity:.2f} m/s for this run")


class Orchestrator(Node):
    """Main orchestrator node that sequences gripper + skill actions."""

    def __init__(self):
        super().__init__("orchestrator")
        self.mode = self.declare_parameter("mode", "forward").value

        self.get_logger().info(f"Execution mode: {self.mode}")


        self.gripper = URGripper(node=self)
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Orchestrator initialized and ready.")
        self.get_logger().info("\n\n\n\n\n----------------\nStarting orchestrator...")

    # ──────────────────────────────────────────────────────────────────────────
    # UTILITY
    # ──────────────────────────────────────────────────────────────────────────


    def pose_distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def wait_until_reached(
        self,
        target_pose: PoseStamped,
        pos_tolerance: float = 0.005,
        timeout: float = 10.0,
    ) -> bool:
        """
        Block until target_link is close to target_pose.
        Returns True if reached, False if timeout.
        """
        start_time = time.time()

        while rclpy.ok():
            # timeout check
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting to reach target pose")
                return False

            try:
                # get current transform of target_link in target frame
                transform = self.tf_buffer.lookup_transform(
                    target_pose.header.frame_id,  # reference frame
                    self.target_link,             # moving link
                    rclpy.time.Time()
                )

                current_pos = transform.transform.translation
                target_pos = target_pose.pose.position

                dist = self.pose_distance(current_pos, target_pos)

                if dist < pos_tolerance:
                    self.get_logger().info(
                        f"Target reached (distance={dist:.4f} m)"
                    )
                    return True

            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed: {ex}")

        time.sleep(0.005)
    
    def execute_and_wait(self, target_pose: PoseStamped, max_vel=0.1, tolerance=0.005):
        self.execute_reach_pose.execute_skill(
            final_pose=target_pose,
            max_vel=max_vel
        )

        success = self.wait_until_reached(target_pose, tolerance)

        if not success:
            self.get_logger().error("Failed to reach pose!")
        return success
    
    def spin_until_done(self, future, timeout: float = 10.0) -> bool:
        """
        Wait for a ROS2 future without calling spin() directly.
        Safe to use from a secondary thread while MultiThreadedExecutor
        is already spinning on the main thread.
        """
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for future!")
                return False
            time.sleep(0.05)
        return True

    def call_move_relative(self, client, transform: Transform, timeout: float = 10.0) -> bool:
        """Helper: build + send a MoveRelative request and wait for the result."""
        request = MoveRelative.Request()
        request.relative_motion = transform
        future = client.call_async(request)
        if not self.spin_until_done(future, timeout=timeout):
            return False
        if future.result() is not None:
            self.get_logger().info(f"MoveRelative response: {future.result()}")
            return True
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return False

    # ──────────────────────────────────────────────────────────────────────────
    # ORCHESTRATION ENTRY POINT
    # ──────────────────────────────────────────────────────────────────────────

    def orchestrate(self):
        if self.mode == "forward":
            self.task1(move_connector=True, move_screw1=True, move_screw2=True)
        # self.task1(move_connector=True, move_screw1=True)
        else:
            self.task1_inverted()
        # self.demo_learned_for_connector()

    # ──────────────────────────────────────────────────────────────────────────
    # TASK 1
    # ──────────────────────────────────────────────────────────────────────────
    def task1(self, move_connector: bool = True, move_screw1: bool = True, move_screw2: bool = True):

        move_relative_client = self.create_client(MoveRelative, "/move_relative")
        self.get_logger().info("Waiting for MoveRelative service...")
        while not move_relative_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveRelative service...")

        # ── OPEN GRIPPER ─────────────────────────────────────────────
        input("Press Enter to open gripper")
        self.gripper.open()

        # ── 1. CONNECTOR ─────────────────────────────────────────────
        if move_connector:
            input("Press Enter to start routine 1")

            # TOP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
            end_pose.pose.position.z = -0.05
            self.execute_and_wait(end_pose, max_vel=velocity)

            # DOWN
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # GRASP
            self.gripper.close()
            time.sleep(2)

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
            end_pose.pose.position.z = -0.08
            self.execute_and_wait(end_pose, max_vel=velocity)

            # LEARNED SKILL
            start_pose = end_pose

            connector_pose = PoseStamped()
            connector_pose.header.frame_id = self.kit1_connector_deposit_frame_name
            connector_pose.pose.position.z = -0.10

            input("Press enter to execute learned skill")
            self.execute_learned_skill.execute_skill(
                "connector_to_deposit",
                initial_pose=start_pose,
                final_pose=connector_pose,
                use_learned_initial=True,
                use_learned_final=True,
                max_vel=velocity,
            )
            input("Press enter to move down to connector deposit")

            # DOWN DEPOSIT
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # RELEASE
            self.gripper.open()

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
            end_pose.pose.position.z = -0.12
            self.execute_and_wait(end_pose, max_vel=velocity)

        # ── 2. SCREW 1 ─────────────────────────────────────────────
        if move_screw1:

            # TOP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw1_frame_name
            end_pose.pose.position.z = -0.05
            self.execute_and_wait(end_pose, max_vel=velocity)

            # DOWN
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw1_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # GRASP
            self.gripper.close()

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw1_frame_name
            end_pose.pose.position.z = -0.08
            self.execute_and_wait(end_pose, max_vel=velocity)

            # MOVE TO DEPOSIT TOP
            # end_pose = PoseStamped()
            # end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
            # end_pose.pose.position.z = -0.05
            # self.execute_and_wait(end_pose, max_vel=velocity)

            input("Press enter to execute learned skill")
            start_pose = end_pose
            connector_pose = PoseStamped()
            connector_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
            connector_pose.pose.position.z = -0.05
            self.execute_learned_skill.execute_skill(
                "screw1_to_deposit",
                initial_pose=start_pose,
                final_pose=connector_pose,
                use_learned_initial=True,
                use_learned_final=True,
                max_vel=velocity,
            )
            input("Press enter to move down to connector deposit")


            # DOWN
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # RELEASE
            self.gripper.open()

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
            end_pose.pose.position.z = -0.12
            self.execute_and_wait(end_pose, max_vel=velocity)

        # ── 3. SCREW 2 ─────────────────────────────────────────────
        if move_screw2:

            # TOP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw2_frame_name
            end_pose.pose.position.z = -0.05
            self.execute_and_wait(end_pose, max_vel=velocity)

            # DOWN
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw2_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # GRASP
            self.gripper.close()

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw2_frame_name
            end_pose.pose.position.z = -0.08
            self.execute_and_wait(end_pose, max_vel=velocity)

            # MOVE TO DEPOSIT TOP
            # end_pose = PoseStamped()
            # end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
            # end_pose.pose.position.z = -0.05
            # self.execute_and_wait(end_pose, max_vel=velocity)
            input("Press enter to execute learned skill")
            start_pose = end_pose
            connector_pose = PoseStamped()
            connector_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
            connector_pose.pose.position.z = -0.05
            self.execute_learned_skill.execute_skill(
                "screw2_to_deposit",
                initial_pose=start_pose,
                final_pose=connector_pose,
                use_learned_initial=True,
                use_learned_final=True,
                max_vel=velocity,
            )
            input("Press enter to move down to connector deposit")

            # DOWN
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
            self.execute_and_wait(end_pose, max_vel=velocity)

            # RELEASE
            self.gripper.open()

            # UP
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
            end_pose.pose.position.z = -0.08
            self.execute_and_wait(end_pose, max_vel=velocity)


    def task1_inverted(self):

        self.get_logger().info("Starting Task 1 INVERTED: Screws → Connector")

        move_relative_client = self.create_client(MoveRelative, "/move_relative")
        self.get_logger().info("Waiting for MoveRelative service...")
        while not move_relative_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveRelative service...")

        # ── OPEN GRIPPER ──────────────────────────────────────────────
        input("Press Enter to open gripper")
        self.gripper.open()

        # ==============================================================
        # 🔩 1. SCREW 2 (DEPOSIT → ORIGINAL)
        # ==============================================================
        input("Start INVERTED routine: Screw 2")

        # ABOVE DEPOSIT
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        end_pose.pose.position.z = -0.05
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        end_pose.pose.position.z = 0.003
        self.execute_and_wait(end_pose, max_vel=velocity)

        # GRASP
        self.gripper.close()

        # UP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_deposit_frame_name
        end_pose.pose.position.z = -0.08
        self.execute_and_wait(end_pose, max_vel=velocity)

        # MOVE TO ORIGINAL TOP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_frame_name
        end_pose.pose.position.z = -0.05
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_frame_name
        self.execute_and_wait(end_pose, max_vel=velocity)

        # RELEASE
        self.gripper.open()

        # UP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw2_frame_name
        end_pose.pose.position.z = -0.08
        self.execute_and_wait(end_pose, max_vel=velocity)

        # ==============================================================
        # 🔩 2. SCREW 1 (DEPOSIT → ORIGINAL)
        # ==============================================================
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        end_pose.pose.position.z = -0.05
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        end_pose.pose.position.z = 0.003
        self.execute_and_wait(end_pose, max_vel=velocity)

        # GRASP
        self.gripper.close()

        # UP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_deposit_frame_name
        end_pose.pose.position.z = -0.08
        self.execute_and_wait(end_pose, max_vel=velocity)

        # MOVE TO ORIGINAL TOP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        end_pose.pose.position.z = -0.05
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        self.execute_and_wait(end_pose, max_vel=velocity)

        # RELEASE
        self.gripper.open()

        # UP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_screw1_frame_name
        end_pose.pose.position.z = -0.08
        self.execute_and_wait(end_pose, max_vel=velocity)

        # ==============================================================
        # 🔌 3. CONNECTOR (DEPOSIT → GRASP)
        # ==============================================================
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        end_pose.pose.position.z = -0.06
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        end_pose.pose.position.z = 0.005
        self.execute_and_wait(end_pose, max_vel=velocity)

        # GRASP
        self.gripper.close()
        time.sleep(2)

        # UP (extra high)
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_deposit_frame_name
        end_pose.pose.position.z = -0.20
        self.execute_and_wait(end_pose, max_vel=velocity)

        # MOVE ABOVE GRASP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
        end_pose.pose.position.z = -0.05
        self.execute_and_wait(end_pose, max_vel=velocity)

        # DOWN
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
        self.execute_and_wait(end_pose, max_vel=velocity)

        # RELEASE
        self.gripper.open()

        # UP
        end_pose = PoseStamped()
        end_pose.header.frame_id = self.kit1_connector_grasp_frame_name
        end_pose.pose.position.z = -0.08
        self.execute_and_wait(end_pose, max_vel=velocity)

def main():
    rclpy.init()
    orchestrator = Orchestrator()

    executor = MultiThreadedExecutor()
    executor.add_node(orchestrator)

    # Orchestration runs in a daemon thread; executor processes ROS2 callbacks
    # on the main thread — spin_until_done() safely polls futures from the
    # orchestration thread without calling spin() a second time.
    threading.Thread(target=orchestrator.orchestrate, daemon=True).start()

    try:
        executor.spin()
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
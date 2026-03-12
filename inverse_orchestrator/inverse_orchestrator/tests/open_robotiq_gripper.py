#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class Orchestrator(Node):

    def __init__(self):
        super().__init__("orchestrator")
        self.pub = self.create_publisher(
            JointTrajectory,
            "/robotiq_gripper_controller/joint_trajectory",
            10,
        )


    
    def test_opening_gripper(self):
        msg = JointTrajectory()
        msg.joint_names = ["robotiq_85_left_knuckle_joint"]

        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [point]

        self.get_logger().info("Publishing open command to Robotiq gripper")
        self.pub.publish(msg)




    
def main():
    rclpy.init()
    orchestrator = Orchestrator()
    # Give DDS a short time to discover the controller subscriber.
    rclpy.spin_once(orchestrator, timeout_sec=0.2)
    orchestrator.test_opening_gripper()
    rclpy.spin_once(orchestrator, timeout_sec=0.2)
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os

class RobotDescriptionDumper(Node):

    def __init__(self):
        super().__init__('robot_description_dumper')

        # Parameter for output file
        self.declare_parameter('output_file', 'robot_description.urdf')
        self.output_file = self.get_parameter('output_file').value

        # ✅ Correct QoS for latched topic
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.listener_callback,
            qos_profile
        )

        self.received = False
        self.get_logger().info(
            f'Waiting for latched /robot_description... Will save to: {self.output_file}'
        )

    def listener_callback(self, msg: String):
        if self.received:
            return

        try:
            with open(self.output_file, 'w') as f:
                f.write(msg.data)

            self.get_logger().info(
                f'URDF saved to {os.path.abspath(self.output_file)}'
            )
            self.received = True

        except Exception as e:
            self.get_logger().error(f'Failed to write file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionDumper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
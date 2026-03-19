import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFToPose(Node):
    def __init__(self):
        super().__init__("tf_to_pose")

        self.parent_frame = "base_link"
        self.child_frame = "target_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseStamped, "/target_link_pose", 10)

        self.create_timer(0.008, self.timer_callback)  # 20 Hz

        self.get_logger().info(
            f"TFToPose node started, publishing {self.parent_frame} -> {self.child_frame} on /target_link_pose"
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),  # latest available
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform: {e}", throttle_duration_sec=2.0)
            return

        msg = PoseStamped()
        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.parent_frame
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z
        msg.pose.orientation = transform.transform.rotation

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Publish whether the right hand is too close to the ArUco marker."""

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Duration, Time
from std_msgs.msg import Bool, Float64
from tf2_ros import Buffer, TransformException, TransformListener


HAND_TOPIC = "/camera_1/smpl/right_hand"
TARGET_FRAME = "aruco_frame"


def quaternion_to_rotation(x, y, z, w):
    """Convert a ROS-order quaternion (xyzw) to a rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class HandProximityMonitor(Node):
    def __init__(self):
        super().__init__("hand_proximity_monitor")
        # Hysteresis: "near" below near_distance, "clear" again only above clear_distance
        self.near_distance = self.declare_parameter("near_distance", 0.20).value
        self.clear_distance = self.declare_parameter("clear_distance", 0.25).value
        # No hand detection for this long -> report near (unknown is unsafe)
        self.stale_timeout = self.declare_parameter("stale_timeout", 0.5).value
        
        if not 0.0 < self.near_distance <= self.clear_distance:
            raise ValueError("Require 0 < near_distance <= clear_distance")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.near_pub = self.create_publisher(Bool, "/right_hand_near_aruco", 10)
        self.distance_pub = self.create_publisher(Float64, "/right_hand_aruco_distance", 10)
        self.create_subscription(PointStamped, HAND_TOPIC, self.on_hand, 10)
        self.create_timer(0.1, self.check_stale)

        self.near = True  # unsafe until the first valid measurement
        self.last_measurement = None
        self.get_logger().info(
            f"Monitoring {HAND_TOPIC} vs {TARGET_FRAME}: near < {self.near_distance:.2f} m, "
            f"clear > {self.clear_distance:.2f} m, stale after {self.stale_timeout:.1f} s"
        )

    def on_hand(self, msg: PointStamped):
        try:
            # Latest available transform: aruco_frame is static w.r.t. the camera
            tf = self.tf_buffer.lookup_transform(TARGET_FRAME, msg.header.frame_id, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f"No TF {msg.header.frame_id} -> {TARGET_FRAME} yet: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        point = np.array([msg.point.x, msg.point.y, msg.point.z])
        point_aruco = quaternion_to_rotation(q.x, q.y, q.z, q.w) @ point + np.array([t.x, t.y, t.z])
        # The marker is the origin of aruco_frame, so the norm is the distance
        distance = float(np.linalg.norm(point_aruco))

        if self.near and distance > self.clear_distance:
            self.set_near(False, f"Right hand clear of marker ({distance:.3f} m)")
        elif not self.near and distance < self.near_distance:
            self.set_near(True, f"Right hand too close to marker ({distance:.3f} m)")

        self.last_measurement = self.get_clock().now()
        self.distance_pub.publish(Float64(data=distance))
        self.near_pub.publish(Bool(data=self.near))

    def check_stale(self):
        now = self.get_clock().now()
        if (self.last_measurement is not None
                and now - self.last_measurement < Duration(seconds=self.stale_timeout)):
            return
        if not self.near:
            self.set_near(True, "No recent right hand detection: assuming too close")
        self.near_pub.publish(Bool(data=True))

    def set_near(self, near, reason):
        self.near = near
        if near:
            self.get_logger().warning(reason)
        else:
            self.get_logger().info(reason)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandProximityMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

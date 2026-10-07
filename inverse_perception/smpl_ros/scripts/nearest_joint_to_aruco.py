#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MARKER_TOPIC = "/smpl_markers"
TARGET_FRAME = "aruco_frame"

# SMPL joint order (24 joints)
SMPL_JOINT_NAMES = [
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hand", "right_hand",
]
# Joints published by SMPLRviz::publish_upper_body, in marker point order
UPPER_BODY = [0, 3, 6, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]


def quaternion_to_rotation(x, y, z, w):
    """Convert a ROS-order quaternion (xyzw) to a rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class NearestJointToAruco(Node):
    def __init__(self):
        super().__init__("nearest_joint_to_aruco")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.distance_pub = self.create_publisher(Float64, "~/distance", 10)
        self.point_pub = self.create_publisher(PointStamped, "~/nearest_point", 10)
        self.create_subscription(MarkerArray, MARKER_TOPIC, self.on_markers, 10)
        self.get_logger().info(
            f"Listening on {MARKER_TOPIC}: nearest joint to {TARGET_FRAME} published on "
            f"{self.distance_pub.topic_name} and {self.point_pub.topic_name}"
        )

    def on_markers(self, msg: MarkerArray):
        # The joints are the SPHERE_LIST marker (the LINE_LIST is the skeleton)
        joints = next((m for m in msg.markers if m.type == Marker.SPHERE_LIST), None)
        if joints is None or not joints.points:
            return

        try:
            # Latest available transform: aruco_frame is static w.r.t. the camera
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, joints.header.frame_id, Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"No TF {joints.header.frame_id} -> {TARGET_FRAME} yet: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        rotation = quaternion_to_rotation(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z])

        points = np.array([[p.x, p.y, p.z] for p in joints.points])
        # Undetected joints come as NaN, missing ones as exact zeros: skip both
        valid = np.isfinite(points).all(axis=1) & np.any(points != 0.0, axis=1)
        if not valid.any():
            return

        # Joints in aruco_frame: the marker is the origin, so the norm is the distance
        points_aruco = points @ rotation.T + translation
        distances = np.linalg.norm(points_aruco, axis=1)
        distances[~valid] = np.inf
        nearest = int(np.argmin(distances))

        if len(points) == len(UPPER_BODY):
            name = SMPL_JOINT_NAMES[UPPER_BODY[nearest]]
        elif len(points) == len(SMPL_JOINT_NAMES):
            name = SMPL_JOINT_NAMES[nearest]
        else:
            name = f"joint_{nearest}"

        self.distance_pub.publish(Float64(data=float(distances[nearest])))
        point = PointStamped()
        point.header.stamp = joints.header.stamp
        point.header.frame_id = TARGET_FRAME
        point.point.x, point.point.y, point.point.z = map(float, points_aruco[nearest])
        self.point_pub.publish(point)

        self.get_logger().info(
            f"Nearest joint to {TARGET_FRAME}: {name} at {distances[nearest]:.3f} m",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NearestJointToAruco()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

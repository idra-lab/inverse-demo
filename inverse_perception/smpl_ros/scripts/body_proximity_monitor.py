#!/usr/bin/env python3
"""Publish whether any tracked SMPL joint is too close to a target.

use_goal_pose false (default): the target is the origin of aruco_frame.
use_goal_pose true:  the target is the position of the latest PoseStamped on
                     goal_topic, in its own header.frame_id; it can change at
                     any time (orientation is ignored).
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Duration, Time
from std_msgs.msg import Bool, Float64, String
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


def joint_name(index, count):
    """Name of the index-th marker point, given how many points were published."""
    if count == len(UPPER_BODY):
        return SMPL_JOINT_NAMES[UPPER_BODY[index]]
    if count == len(SMPL_JOINT_NAMES):
        return SMPL_JOINT_NAMES[index]
    return f"joint_{index}"


class BodyProximityMonitor(Node):
    def __init__(self):
        super().__init__("body_proximity_monitor")
        # Hysteresis: "near" below near_distance, "clear" again only above clear_distance
        self.near_distance = self.declare_parameter("near_distance", 0.20).value
        self.clear_distance = self.declare_parameter("clear_distance", 0.25).value
        # No body detection for this long -> report near (unknown is unsafe)
        self.stale_timeout = self.declare_parameter("stale_timeout", 0.5).value
        if not 0.0 < self.near_distance <= self.clear_distance:
            raise ValueError("Require 0 < near_distance <= clear_distance")
        self.use_goal_pose = self.declare_parameter("use_goal_pose", False).value
        goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        # 0 = a goal stays valid until replaced (e.g. one-shot RViz goals)
        self.goal_timeout = self.declare_parameter("goal_timeout", 0.0).value
        # A goal moving more than this counts as a new target: re-evaluate from unsafe
        self.goal_reset_distance = self.declare_parameter("goal_reset_distance", 0.05).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.use_goal_pose:
            self.target_name = goal_topic
            self.near_pub = self.create_publisher(Bool, "/body_near_goal", 10)
            self.distance_pub = self.create_publisher(Float64, "/body_goal_distance", 10)
            self.joint_pub = self.create_publisher(String, "/body_nearest_joint_goal", 10)
            self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        else:
            self.target_name = TARGET_FRAME
            self.near_pub = self.create_publisher(Bool, "/body_near_aruco", 10)
            self.distance_pub = self.create_publisher(Float64, "/body_aruco_distance", 10)
            self.joint_pub = self.create_publisher(String, "/body_nearest_joint", 10)
        self.goal = None  # (frame_id, position) of the latest goal
        self.goal_received = None
        self.create_subscription(MarkerArray, MARKER_TOPIC, self.on_markers, 10)
        self.create_timer(0.1, self.check_stale)

        self.near = True  # unsafe until the first valid measurement
        self.last_measurement = None
        self.get_logger().info(
            f"Monitoring {MARKER_TOPIC} vs {self.target_name}: near < {self.near_distance:.2f} m, "
            f"clear > {self.clear_distance:.2f} m, stale after {self.stale_timeout:.1f} s"
        )

    def on_goal(self, msg: PoseStamped):
        if not msg.header.frame_id:
            self.get_logger().error("Ignoring goal with empty frame_id", throttle_duration_sec=5.0)
            return
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        moved = (self.goal is None
                 or self.goal[0] != msg.header.frame_id
                 or np.linalg.norm(position - self.goal[1]) > self.goal_reset_distance)
        self.goal = (msg.header.frame_id, position)
        self.goal_received = self.get_clock().now()
        if moved:
            self.get_logger().info(
                f"New goal in {msg.header.frame_id}: {np.round(position, 3).tolist()}"
            )
            if not self.near:
                self.set_near(True, "New goal: re-evaluating distance")

    def current_target(self):
        """(frame, position) of the target, or None if there is no valid one."""
        if not self.use_goal_pose:
            return TARGET_FRAME, np.zeros(3)
        if self.goal is None:
            self.get_logger().warning(
                f"No goal received on {self.target_name} yet", throttle_duration_sec=5.0
            )
            return None
        if (self.goal_timeout > 0.0
                and self.get_clock().now() - self.goal_received > Duration(seconds=self.goal_timeout)):
            self.get_logger().warning("Goal expired", throttle_duration_sec=5.0)
            return None
        return self.goal

    def on_markers(self, msg: MarkerArray):
        # The joints are the SPHERE_LIST marker (the LINE_LIST is the skeleton)
        joints = next((m for m in msg.markers if m.type == Marker.SPHERE_LIST), None)
        if joints is None or not joints.points:
            return

        points = np.array([[p.x, p.y, p.z] for p in joints.points])
        # Undetected joints come as NaN, missing ones as exact zeros: skip both
        valid = np.isfinite(points).all(axis=1) & np.any(points != 0.0, axis=1)
        if not valid.any():
            return

        # No valid target: skip, the stale check then reports near
        target = self.current_target()
        if target is None:
            return
        target_frame, target_position = target

        try:
            # Latest available transform (the camera is static in the target frame)
            tf = self.tf_buffer.lookup_transform(target_frame, joints.header.frame_id, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f"No TF {joints.header.frame_id} -> {target_frame} yet: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        rotation = quaternion_to_rotation(q.x, q.y, q.z, q.w)
        # Joints in the target frame, then distance to the target position
        points_target = points @ rotation.T + np.array([t.x, t.y, t.z])
        distances = np.linalg.norm(points_target - target_position, axis=1)
        distances[~valid] = np.inf
        nearest = int(np.argmin(distances))
        distance = float(distances[nearest])
        name = joint_name(nearest, len(points))

        if self.near and distance > self.clear_distance:
            self.set_near(False, f"Body clear of target (nearest: {name} at {distance:.3f} m)")
        elif not self.near and distance < self.near_distance:
            self.set_near(True, f"{name} too close to target ({distance:.3f} m)")

        self.last_measurement = self.get_clock().now()
        self.distance_pub.publish(Float64(data=distance))
        self.joint_pub.publish(String(data=name))
        self.near_pub.publish(Bool(data=self.near))

    def check_stale(self):
        now = self.get_clock().now()
        if (self.last_measurement is not None
                and now - self.last_measurement < Duration(seconds=self.stale_timeout)):
            return
        if not self.near:
            self.set_near(True, "No recent measurement (body or goal missing): assuming too close")
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
        node = BodyProximityMonitor()
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

#!/usr/bin/env python3
"""Publish whether the tracked human body is too close to one or more targets.

Each target is the origin of a TF frame:
  use_obs_frames false (default): one target, "bus_bar", at aruco_frame.
  use_obs_frames true:  one target per name in obs_objects, at the TF frame
                        "obs(<name>)", e.g. obs(bus_bar), obs(front_connector);
                        an entry that is already a frame, e.g. via(bus_bar), is used as is.

For every skeleton received on /smpl_markers, the node finds, for each target,
the SMPL joint nearest to it and compares that distance with the thresholds.
The state of all targets is published as a single std_msgs/String, e.g.
    "-bus_bar.free, front_connector.free, rear_connector.free"
  "<name>.free"   the body is away from <name>
  "-<name>.free"  the body is too close to <name>, or its state is unknown
                  (no body detected, no TF, or no recent data): unknown is unsafe

Subscribed:
  /smpl_markers (visualization_msgs/MarkerArray): joints from zed_smpl_tracking
  TF: camera frame of the markers -> each target frame
Published (topic names depend on use_obs_frames: aruco / obs):
  state:         /body_near_aruco          | /seed_ur10_services/state  (String)
  distance:      /body_aruco_distance      | /body_obs_distance         (Float64)
  nearest joint: /body_nearest_joint       | /body_nearest_joint_obs    (String)
  The distance and joint are the nearest over all targets, for debugging.

Parameters:
  near_distance, clear_distance [m]  hysteresis thresholds (see below)
  clear_hold [s]                     time above clear_distance before "free"
  stale_timeout [s]                  max age of a measurement before "unknown"
  use_obs_frames (bool)              target selection (see above)
  obs_objects (string list)          object names used when use_obs_frames is true
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Duration, Time
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MARKER_TOPIC = "/smpl_markers"
# Target frame when use_obs_frames is false: the ArUco marker on the table
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

# Hysteresis thresholds [m], defaults of the near_distance/clear_distance
# parameters (values set in the launch file take precedence):
#  - a target becomes "not free" when the nearest joint gets closer than
#    NEAR_DISTANCE;
#  - it becomes "free" again only when the nearest joint is farther than
#    CLEAR_DISTANCE.
# In between the state does not change, so tracking noise around a single
# threshold cannot make the output flicker between free and not free.
NEAR_DISTANCE = 0.50
CLEAR_DISTANCE = 0.55
# Time hysteresis [s], default of the clear_hold parameter: a "not free" target
# becomes "free" only after the nearest joint has stayed farther than
# CLEAR_DISTANCE for this long. Any measurement at or below CLEAR_DISTANCE (or
# a stale target) restarts the wait.
CLEAR_HOLD = 2.0


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


def obs_frame(name):
    """TF frame of an obs_objects entry: a full frame name like "via(bus_bar)"
    is used as is, a bare name like "front_connector" becomes obs(<name>)."""
    return name if "(" in name else f"obs({name})"


class BodyProximityMonitor(Node):
    """Tracks one near/free state per target and publishes them as one string."""

    def __init__(self):
        super().__init__("body_proximity_monitor")
        # ---- Parameters ----
        # Hysteresis thresholds, see NEAR_DISTANCE / CLEAR_DISTANCE
        self.near_distance = self.declare_parameter("near_distance", NEAR_DISTANCE).value
        self.clear_distance = self.declare_parameter("clear_distance", CLEAR_DISTANCE).value
        # Time hysteresis, see CLEAR_HOLD
        self.clear_hold = self.declare_parameter("clear_hold", CLEAR_HOLD).value
        # A target not measured for this long (no body, no TF) is reported as
        # not free: unknown is treated as unsafe
        self.stale_timeout = self.declare_parameter("stale_timeout", 0.5).value
        if not 0.0 < self.near_distance <= self.clear_distance:
            raise ValueError("Require 0 < near_distance <= clear_distance")
        if self.clear_hold < 0.0:
            raise ValueError("Require clear_hold >= 0")
        use_obs_frames = self.declare_parameter("use_obs_frames", False).value
        obs_objects = self.declare_parameter(
            "obs_objects", ["via(bus_bar)", "front_connector", "rear_connector"]
        ).value

        # ---- Targets and output topics ----
        # Targets as (name, TF frame); the target point is the origin of its frame.
        # The name is what appears in the state string ("<name>.free").
        if use_obs_frames:
            self.targets = [(name, obs_frame(name)) for name in obs_objects]
            state_topic = "/seed_ur10_services/state_facts"
            self.distance_pub = self.create_publisher(Float64, "/body_obs_distance", 10)
            self.joint_pub = self.create_publisher(String, "/body_nearest_joint_obs", 10)
        else:
            self.targets = [("bus_bar", TARGET_FRAME)]
            state_topic = "/body_near_aruco"
            self.distance_pub = self.create_publisher(Float64, "/body_aruco_distance", 10)
            self.joint_pub = self.create_publisher(String, "/body_nearest_joint", 10)
        if not self.targets:
            raise ValueError("No targets to monitor")

        # ---- ROS interfaces ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.create_subscription(MarkerArray, MARKER_TOPIC, self.on_markers, 10)
        # Runs even when no skeleton arrives, so a lost body is still reported
        self.create_timer(0.1, self.check_stale)

        # ---- State, per target name ----
        # near[name]: True = "not free". Starts True: unsafe until the first
        # valid measurement of each target.
        # last_measurement[name]: time of the last successful distance check.
        # clear_since[name]: while not free, since when the nearest joint has
        # been continuously above clear_distance (None = not above it).
        self.near = {name: True for name, _ in self.targets}
        self.last_measurement = {name: None for name, _ in self.targets}
        self.clear_since = {name: None for name, _ in self.targets}
        self.get_logger().info(
            f"Monitoring {MARKER_TOPIC} vs {[frame for _, frame in self.targets]}: "
            f"near < {self.near_distance:.2f} m, clear > {self.clear_distance:.2f} m "
            f"for {self.clear_hold:.1f} s, "
            f"stale after {self.stale_timeout:.1f} s, state on {state_topic}"
        )

    def on_markers(self, msg: MarkerArray):
        """New skeleton: update every target's state and publish.

        For each target: transform the joints into the target frame, take the
        nearest valid joint, then apply the hysteresis to its distance.
        """
        # The joints are the SPHERE_LIST marker (the LINE_LIST is the skeleton)
        joints = next((m for m in msg.markers if m.type == Marker.SPHERE_LIST), None)
        if joints is None or not joints.points:
            return

        points = np.array([[p.x, p.y, p.z] for p in joints.points])
        # Undetected joints come as NaN, missing ones as exact zeros: skip both
        valid = np.isfinite(points).all(axis=1) & np.any(points != 0.0, axis=1)
        if not valid.any():
            return

        # A target whose TF is missing is skipped: it is not marked as measured,
        # so check_stale() turns it to "not free" after stale_timeout
        now = self.get_clock().now()
        overall = None  # (distance, joint) of the nearest joint over all targets
        for name, frame in self.targets:
            try:
                # Latest available transform (the target frames are static)
                tf = self.tf_buffer.lookup_transform(frame, joints.header.frame_id, Time())
            except TransformException as exc:
                # Not measured: this target goes stale, so it reports near
                self.get_logger().warning(
                    f"No TF {joints.header.frame_id} -> {frame} yet: {exc}",
                    throttle_duration_sec=5.0,
                )
                continue

            q = tf.transform.rotation
            t = tf.transform.translation
            rotation = quaternion_to_rotation(q.x, q.y, q.z, q.w)
            # Joints in the target frame: the target is the origin, so the norm is the distance
            points_target = points @ rotation.T + np.array([t.x, t.y, t.z])
            distances = np.linalg.norm(points_target, axis=1)
            distances[~valid] = np.inf
            nearest = int(np.argmin(distances))
            distance = float(distances[nearest])
            joint = joint_name(nearest, len(points))

            # Hysteresis: leave "not free" only after staying above
            # clear_distance for clear_hold, enter it only below near_distance;
            # in between keep the state
            if self.near[name]:
                if distance > self.clear_distance:
                    if self.clear_since[name] is None:
                        self.clear_since[name] = now
                    if now - self.clear_since[name] >= Duration(seconds=self.clear_hold):
                        self.set_near(
                            name, False,
                            f"Body clear of {frame} for {self.clear_hold:.1f} s "
                            f"(nearest: {joint} at {distance:.3f} m)",
                        )
                else:
                    # Back at or below clear_distance: restart the wait
                    self.clear_since[name] = None
            elif distance < self.near_distance:
                self.set_near(name, True, f"{joint} too close to {frame} ({distance:.3f} m)")
            self.last_measurement[name] = now
            if overall is None or distance < overall[0]:
                overall = (distance, joint)

        # No target could be measured (all TFs missing): nothing new to publish
        if overall is None:
            return
        self.distance_pub.publish(Float64(data=overall[0]))
        self.joint_pub.publish(String(data=overall[1]))
        self.publish_state()

    def check_stale(self):
        """Timer: mark targets without a recent measurement as not free.

        Covers a person leaving the camera view, tracking loss, the camera node
        stopping or a target TF disappearing. Publishes the state whenever any
        target is stale, so consumers keep receiving it even without skeletons.
        """
        now = self.get_clock().now()
        stale = [
            name for name, last in self.last_measurement.items()
            if last is None or now - last >= Duration(seconds=self.stale_timeout)
        ]
        if not stale:
            return
        for name in stale:
            if not self.near[name]:
                self.set_near(name, True, f"No recent measurement for {name}: assuming too close")
        self.publish_state()

    def publish_state(self):
        """Publish e.g. "-bus_bar.free, front_connector.free" in target order."""
        state = ", ".join(
            f"-{name}.free" if self.near[name] else f"{name}.free" for name, _ in self.targets
        )
        self.state_pub.publish(String(data=state))

    def set_near(self, name, near, reason):
        """Change one target's state and log why (only called on a change)."""
        self.near[name] = near
        self.clear_since[name] = None
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

from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import rclpy


class SMPLModel:
    """Handles SMPL model visualization markers and TF2 transforms."""

    def __init__(self, node, marker_topic="smpl_model_markers"):
        self.node = node
        self.subscriber = self.node.create_subscription(
            MarkerArray,
            marker_topic,
            self.marker_callback,
            10,
        )
        self.node.get_logger().info(
            f"Subscribed to SMPL model markers on {marker_topic}"
        )

        # SMPL data
        self.triangles = None
        self.keypoints = None

        # === TF2 setup ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.latest_frame_position = None  # stores position of frame_id wrt zed_camera_frame

    def marker_callback(self, msg: MarkerArray):
        """Callback to process incoming MarkerArray messages."""
        triangles = []
        self.keypoints = list()
        for marker in msg.markers:
            if marker.type == marker.TRIANGLE_LIST:
                self.node.get_logger().info(
                    f"Processing TRIANGLE_LIST marker with ID: {marker.id}"
                )
                for point in marker.points:
                    triangles.append([point.x, point.y, point.z])
            if marker.ns == "keypoints":
                for point in marker.points:
                    self.keypoints.append(np.array([point.x, point.y, point.z]))

        # Convert to numpy array
        if triangles:
            self.triangles = np.array(triangles).reshape(-1, 3, 3)

    # === TF2 helper ===
    def update_frame_position(self, frame_id="world", reference_frame="zed_camera_frame"):
        """
        Looks up the TF transform from reference_frame -> frame_id
        and stores the position (translation) of frame_id in reference_frame coordinates.
        """
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                reference_frame, frame_id, rclpy.time.Time()
            )
            t = transform.transform.translation
            self.latest_frame_position = np.array([t.x, t.y, t.z])
            self.node.get_logger().info(
                f"Transform {frame_id} -> {reference_frame}: position = {self.latest_frame_position}"
            )
            return self.latest_frame_position
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(
                f"Could not transform {frame_id} to {reference_frame}: {e}"
            )
            return None

    def get_keypoints_shortest_distance(self, frame_id="world"):
        """Returns the shortest distance from the SMPL model to the origin in the specified frame."""
        self.update_frame_position(frame_id=frame_id)

        if (self.keypoints is None) or (self.latest_frame_position is None):
            self.node.get_logger().warn("No SMPL model data available.")
            return None

        # Update TF position
        # self.node.get_logger().info(f"Frame pos: {self.latest_frame_position}")

        min_distance = float("inf")
        for pt in self.keypoints:
            distance = np.linalg.norm(pt - self.latest_frame_position)
            if distance < min_distance:
                min_distance = distance

        # self.node.get_logger().info(
        #     f"Shortest distance from SMPL model to origin in frame {frame_id}: {min_distance}"
        # )
        return min_distance

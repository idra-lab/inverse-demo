from visualization_msgs.msg import MarkerArray
import numpy as np


class SMPLModel:
    """Handles SMPL model visualization markers."""

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
        self.triangles = None

    def marker_callback(self, msg):
        """Callback to process incoming MarkerArray messages."""
        self.node.get_logger().info(
            f"Received SMPL model markers with {len(msg.markers)} markers."
        )
        # Further processing of the markers can be done here.

        # Decode msg which is a TRIANGLE_LIST MarkerArray representing the SMPL model
        # triangles = np.zeros((0, 3, 3))  # Initialize an empty array for triangles
        triangles = []
        for marker in msg.markers:
            if marker.type == marker.TRIANGLE_LIST:
                self.node.get_logger().info(
                    f"Processing TRIANGLE_LIST marker with ID: {marker.id}"
                )
                for point in marker.points:
                    triangles.append([point.x, point.y, point.z])
        # Convert to numpy array
        self.triangles = np.array(triangles).reshape(-1, 3, 3)

    def get_shortest_distance(self, frame_id="world"):
        """Returns the shortest distance from the SMPL model to the origin in the specified frame."""
        if self.triangles is None:
            self.node.get_logger().warn("No SMPL model data available.")
            return None

        min_distance = float("inf")
        for triangle in self.triangles:
            for vertex in triangle:
                distance = np.linalg.norm(vertex)
                if distance < min_distance:
                    min_distance = distance

        self.node.get_logger().warn(
            f"Shortest distance from SMPL model to origin in frame {frame_id}: {min_distance}"
        )
        return min_distance

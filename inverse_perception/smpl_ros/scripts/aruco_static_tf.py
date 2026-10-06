#!/usr/bin/env python3
"""Average a fixed ArUco marker's pose and publish it on /tf_static."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# Configuration: marker size is the outer black square, excluding white margins.
ARUCO_DICTIONARY = cv2.aruco.DICT_6X6_250
MARKER_ID = 0
MARKER_SIZE_M = 0.100
SAMPLE_COUNT = 50
IMAGE_TOPIC = "/camera_1/zed/image"
CAMERA_INFO_TOPIC = "/camera_1/zed/camera_info"
MARKER_FRAME = "aruco_frame"
# ZED VIEW::LEFT is rectified: use CameraInfo.P and no lens distortion.
IMAGE_IS_RECTIFIED = True
MAX_REPROJECTION_ERROR_PX = 3.0


def rotation_to_quaternion(rotation):
    """Convert a rotation matrix to a normalized ROS-order quaternion (xyzw)."""
    trace = np.trace(rotation)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = np.array([
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            s / 4.0,
        ])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        q = np.zeros(4)
        q[i] = s / 4.0
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
    return q / np.linalg.norm(q)


def average_quaternions(quaternions):
    """Markley mean: q and -q represent the same rotation."""
    samples = np.asarray(quaternions)
    _, vectors = np.linalg.eigh(samples.T @ samples)
    mean = vectors[:, -1]
    return mean if mean[3] >= 0.0 else -mean


class ArucoStaticTF(Node):
    def __init__(self):
        super().__init__("aruco_static_tf")
        if SAMPLE_COUNT < 1 or MARKER_SIZE_M <= 0:
            raise ValueError("SAMPLE_COUNT and MARKER_SIZE_M must be positive")
        self.bridge = CvBridge()
        self.broadcaster = StaticTransformBroadcaster(self)
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        # Support both the older Ubuntu ROS OpenCV and newer OpenCV APIs.
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters()
            )
            self.detect = self.detector.detectMarkers
        else:
            parameters = cv2.aruco.DetectorParameters_create()
            self.detect = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters
            )
        half = MARKER_SIZE_M / 2.0
        # IPPE_SQUARE order: top-left, top-right, bottom-right, bottom-left.
        # Marker origin is its center; +X right, +Y up, +Z out of its face.
        self.object_points = np.array([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float64)
        self.camera_info = None
        self.parent_frame = None
        self.translations = []
        self.quaternions = []
        self.published = False
        self.info_subscription = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, qos_profile_sensor_data
        )
        self.image_subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Waiting for marker {MARKER_ID}: averaging {SAMPLE_COUNT} valid poses "
            f"from {IMAGE_TOPIC}. Keep camera and marker stationary."
        )

    def on_camera_info(self, msg):
        if not self.published:
            self.camera_info = msg

    def on_image(self, msg):
        if self.published or self.camera_info is None:
            return
        info = self.camera_info
        if not msg.header.frame_id or msg.header.frame_id != info.header.frame_id:
            self.get_logger().warning("Image/CameraInfo frame IDs must match and be nonempty.", throttle_duration_sec=5.0)
            return
        if (msg.width, msg.height) != (info.width, info.height):
            self.get_logger().warning("Image/CameraInfo resolutions do not match.", throttle_duration_sec=5.0)
            return
        if msg.header.frame_id == MARKER_FRAME:
            self.get_logger().error("Camera and marker frame names must differ.", throttle_duration_sec=5.0)
            return
        if IMAGE_IS_RECTIFIED:
            matrix = np.asarray(info.p, dtype=np.float64).reshape(3, 4)[:, :3].copy()
            distortion = np.zeros(5, dtype=np.float64)
        else:
            matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(info.d, dtype=np.float64)
            if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
                self.get_logger().error("Unsupported raw-image distortion model.", throttle_duration_sec=5.0)
                return
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            self.get_logger().warning("Waiting for valid camera intrinsics.", throttle_duration_sec=5.0)
            return
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            corners, ids, _ = self.detect(gray)
            if ids is None:
                return
            matches = np.flatnonzero(ids.reshape(-1) == MARKER_ID)
            if len(matches) != 1:
                return
            image_points = corners[int(matches[0])].reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points, matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                return
            rotation, _ = cv2.Rodrigues(rvec)
            camera_points = (rotation @ self.object_points.T).T + tvec.reshape(3)
            if np.any(camera_points[:, 2] <= 0):
                return
            projected, _ = cv2.projectPoints(
                self.object_points, rvec, tvec, matrix, distortion
            )
            error = np.sqrt(np.mean(np.sum(
                (projected.reshape(4, 2) - image_points) ** 2, axis=1
            )))
            if error > MAX_REPROJECTION_ERROR_PX:
                return
        except (cv2.error, RuntimeError, ValueError) as exc:
            self.get_logger().warning(f"Marker processing failed: {exc}", throttle_duration_sec=5.0)
            return
        if self.parent_frame != msg.header.frame_id:
            self.translations.clear()
            self.quaternions.clear()
            self.parent_frame = msg.header.frame_id
        self.translations.append(tvec.reshape(3))
        self.quaternions.append(rotation_to_quaternion(rotation))
        count = len(self.translations)
        if count % 10 == 0 or count == SAMPLE_COUNT:
            self.get_logger().info(f"Collected {count}/{SAMPLE_COUNT} marker poses")
        if count == SAMPLE_COUNT:
            self.publish_transform()

    def publish_transform(self):
        translation = np.mean(self.translations, axis=0)
        quaternion = average_quaternions(self.quaternions)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = MARKER_FRAME
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.broadcaster.sendTransform(transform)
        self.published = True
        self.destroy_subscription(self.image_subscription)
        self.destroy_subscription(self.info_subscription)
        self.get_logger().info(
            f"Published static TF {self.parent_frame} -> {MARKER_FRAME}: "
            f"translation={translation.tolist()} m, quaternion={quaternion.tolist()}. "
            "Restart this node to recalibrate."
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoStaticTF()
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

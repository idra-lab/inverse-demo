#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R_convert
class CharucoTracker(Node):
    def __init__(self):
        super().__init__('charuco_tracker')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('charuco_square_length', 0.035)  # meters
        self.declare_parameter('charuco_marker_length', 0.028)  # meters
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('marker_frame', 'charuco_board')
        self.declare_parameter('charuco_rows', 4)
        self.declare_parameter('charuco_cols', 4)
        # calibration params
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.charuco_square_length = self.get_parameter('charuco_square_length').get_parameter_value().double_value
        self.charuco_marker_length = self.get_parameter('charuco_marker_length').get_parameter_value().double_value
        self.charuco_rows = self.get_parameter('charuco_rows').get_parameter_value().integer_value
        self.charuco_cols = self.get_parameter('charuco_cols').get_parameter_value().integer_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_frame = self.get_parameter('marker_frame').get_parameter_value().string_value

        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.k1 = self.get_parameter('k1').get_parameter_value().double_value
        self.k2 = self.get_parameter('k2').get_parameter_value().double_value
        self.p1 = self.get_parameter('p1').get_parameter_value().double_value
        self.p2 = self.get_parameter('p2').get_parameter_value().double_value
        self.k3 = self.get_parameter('k3').get_parameter_value().double_value
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([[self.k1, self.k2, self.p1, self.p2, self.k3]], dtype=np.float32)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Define Charuco board
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard((self.charuco_rows,self.charuco_cols), self.charuco_square_length, self.charuco_marker_length, self.dictionary)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.02  # Lower this to detect smaller markers at a distance
        self.parameters.maxMarkerPerimeterRate = 0.2   # Raise this if markers are not fully captured
        self.parameters.minMarkerDistanceRate = 0.04   # Lower this if markers are close to each other
        self.parameters.minOtsuStdDev = 5.0            # Control noise in thresholding
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.2
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Use subpixel refinement
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # stamp = msg.header.stamp
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # cv2.imshow("Gray", gray)
        # cv2.waitKey(1)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        self.get_logger().info(f"Detected markers: {ids}")
        if ids is not None:
            # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # self.get_logger().info(f"Detected some markers: {ids.flatten()}")
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board,
                                                                                   cameraMatrix=self.camera_matrix,
                                                                                   distCoeffs=self.dist_coeffs)
            if ret > 1: #self.charuco_rows:
                success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, self.camera_matrix, self.dist_coeffs, None, None)

                if success:
                    # draw the board
                    # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.charuco_rows * self.charuco_square_length)
                    # resize frame to HD
                    frame = cv2.resize(frame, (1280, 720))
                    cv2.imshow("Charuco Board", frame)
                    cv2.waitKey(1)
                    stamp = self.get_clock().now().to_msg()
                    self.publish_transform(rvec, tvec, stamp)
                    return
            # else:
                # self.get_logger().info("Not enough markers detected")
        frame = cv2.resize(frame, (1280, 720))
        cv2.imshow("Charuco Board", frame)
        cv2.waitKey(1)

    def publish_transform(self, rvec, tvec, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.marker_frame
        transform.transform.translation = Vector3(x=tvec[0][0], y=tvec[1][0], z=tvec[2][0])

        rot_mat, _ = cv2.Rodrigues(rvec)
        q = self.rotation_matrix_to_quaternion(rot_mat)
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = R_convert.from_matrix(R).as_quat()
        return q


def main(args=None):
    rclpy.init(args=args)
    node = CharucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

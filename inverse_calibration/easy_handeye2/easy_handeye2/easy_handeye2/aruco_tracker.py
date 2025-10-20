#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R
from rosidl_runtime_py import set_message_fields, message_to_yaml

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_length', 0.022)  # meters
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('marker_frame', 'aruco_marker')
        
        # Calibration parameters
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)
        self.declare_parameter('save_calibration', False)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
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

        self.get_logger().info(f"Camera Matrix: \n{self.camera_matrix}")
        self.get_logger().info(f"Distortion Coefficients: \n{self.dist_coeffs}")

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Define ArUco dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.last_rvec = None
        self.last_tvec = None
        self.img_received = False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is not None and self.marker_id in ids:
            index = np.where(ids == self.marker_id)[0][0]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[index]], self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            if rvec is not None and tvec is not None:
                # Draw the marker and axes
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_length / 2)
                #resize to HD
                frame = cv2.resize(frame, (1280, 720))
                cv2.imshow("Aruco Marker", frame)
                cv2.waitKey(1)
                stamp = msg.header.stamp
                self.last_rvec = rvec[0]
                self.last_tvec = tvec[0] 
                self.publish_transform(self.last_rvec, self.last_tvec, stamp)
        else:
            frame = cv2.resize(frame, (1280, 720))
            cv2.imshow("Aruco Marker", frame)
            cv2.waitKey(1)
        
        if not self.img_received:
            self.timer = self.create_timer(0.01, self.timer_callback)
            self.img_received = True
    
    def timer_callback(self):
        if self.last_rvec is not None and self.last_tvec is not None:
            self.publish_transform(self.last_rvec, self.last_tvec, self.get_clock().now().to_msg())
        else:
            self.get_logger().warn("No marker detected yet.")
        
    def publish_transform(self, rvec, tvec, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.marker_frame
        transform.transform.translation = Vector3(x=tvec[0][0], y=tvec[0][1], z=tvec[0][2])

        rot_mat, _ = cv2.Rodrigues(rvec)
        q = self.rotation_matrix_to_quaternion(rot_mat)
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # self.get_logger().info(f"Position: {tvec.flatten()}, Rotation: {q}")
        self.tf_broadcaster.sendTransform(transform)

    
    @staticmethod
    def rotation_matrix_to_quaternion(rotmat):
        q = R.from_matrix(rotmat).as_quat()
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

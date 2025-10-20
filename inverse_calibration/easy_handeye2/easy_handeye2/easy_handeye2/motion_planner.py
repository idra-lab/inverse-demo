#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath import UnitQuaternion
from spatialmath.base import tr2eul
import time
class MotionPlanner(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # Declare parameters
        super().__init__("linear_trajectory_publisher")

        self.declare_parameter("camera_to_calibrate", "cam1")
        self.declare_parameter("robot_base_frame", "lbr_link_0")
        self.declare_parameter("robot_effector_frame", "lbr_link_ee")
        self.declare_parameter("topic_name", "/lbr/target_framee")
        self.declare_parameter("waiting_time", 3.0)

        self.camera_to_calibrate = self.get_parameter("camera_to_calibrate").get_parameter_value().string_value
        self.base = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self.end_effector = self.get_parameter("robot_effector_frame").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        
        # Initialize the publisher for PoseStamped messages
        self.publisher_ = self.create_publisher(PoseStamped, self.topic_name, 10)

        # Initialize tf2 for transforming coordinates
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_freq = 1000  # 1 kHz
        self.waiting_time = self.get_parameter("waiting_time").get_parameter_value().double_value
        # Position increment

        self.starting_position = np.array([0.58181, -0.16668, 0.53191, -0.049209, 0.99814, -0.003696, 0.035745])
    
        # Normalize quaternion part
        self.starting_position[3:7] = self.starting_position[3:7] / np.linalg.norm(self.starting_position[3:7])

        # convert into RPY
        self.initial_RPY = R.from_quat(self.starting_position[3:7]).as_euler("xyz")

        self.starting_position = self.starting_position
        self.target_keypoints = np.array([
            self.starting_position.tolist(),


        ])

        if self.camera_to_calibrate == "cam2":
            combinations = [(-35, 0, 0),
                            (35, 0, 0),
                            (0, -35, 0),
                            (0, 35, 0),
                            (0, 0, -35),
                            (0, 0, 20),
                            (0, 0, 0)]
        elif self.camera_to_calibrate == "cam1":
            combinations = [(-30, 0, 0),
                            (20, 0, 0),
                            (0, -15, 0),
                            (0, 20, 0),
                            (0, 0, -40),
                            (0, 0, 50),
                            (0, 0, 0)]
        else:
            raise ValueError("Invalid camera_to_calibrate parameter. Use 'cam1' or 'cam2'.")

        self.target_keypoints = []
        for roll_offset, pitch_offset, yaw_offset in combinations:
            offset_rpy = self.initial_RPY + np.radians([roll_offset, pitch_offset, yaw_offset])
            offset_quat = R.from_euler("xyz", offset_rpy).as_quat()
            target_point = np.hstack((self.starting_position[:3], offset_quat))
            self.target_keypoints.append(target_point)
        self.target_keypoints = np.array(self.target_keypoints)

        self.initial_orientation = None
        self.initial_position = None
        self.velocity = 0.03  # Speed of the linear trajectory
        self.ang_velocity = 0.1  # Speed of the angular trajectory
        self.traj_idx = 0
        self.timer = self.create_timer(1.0 / self.pub_freq, self.publish_trajectory)


    def get_current_transform(self):
        try:
            transform_msg: TransformStamped = self.tf_buffer.lookup_transform(
                self.base, self.end_effector, rclpy.time.Time()
            )
            return transform_msg
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return None
    
    def compute_ctraj(self):
        # Create a SE3 object from the quaternion and translation vector
        # traj list is Nx7
        traj_list = np.empty((0, 7))
                    # add waiting time
        for i in range(int(self.waiting_time * self.pub_freq)):
            traj_list = np.vstack((traj_list, np.zeros((1, 7))))
            traj_list[i, 0:3] = self.initial_position
            traj_list[i, 3:7] = self.initial_orientation


        for i in range(len(self.target_keypoints)):
            # lin_distance_between_waypoints = self.velocity / self.pub_freq
            # number_of_samples_lin = int(
            #     np.linalg.norm(self.step_size[i]) / lin_distance_between_waypoints
            # )
            # ang_distance_between_waypoints = self.ang_velocity / self.pub_freq
            # number_of_samples_ang = int(
            #     np.linalg.norm(self.step_orientation[i]) / ang_distance_between_waypoints
            # )
            number_of_samples = 5000 #max(number_of_samples_lin, number_of_samples_ang)
            self.get_logger().info("Number of samples: " + str(number_of_samples))
            if i==0:
                T_start = SE3.Rt(
                    UnitQuaternion(
                        s=self.initial_orientation[3], v=self.initial_orientation[:3], norm=True
                    ).R,
                    self.initial_position,
                )
            else:
                # use last_point as the new start point
                T_start = traj_list[-1]
                T_start = SE3.Rt(
                    UnitQuaternion(
                        s=T_start[6], v=T_start[3:6], norm=True
                    ).R,
                    T_start[:3],
                )
            # self.get_logger().info("Start position: " + str(T_start.t) + str(self.step_size[i]))
            # pos_end = self.step_size[i] + T_start.t
            # orient_end_RPY = self.step_orientation[i] + R.from_matrix(T_start.R).as_euler("xyz")
            # orient_end = R.from_euler("xyz", orient_end_RPY).as_quat()
            pose_end = self.target_keypoints[i]
            pos_end = pose_end[:3]
            orient_end = pose_end[3:7]

            T_end = SE3.Rt(
                UnitQuaternion(
                    s=orient_end[3], v=orient_end[:3], norm=True
                ).R,
                pos_end,
            )
            self.get_logger().info("Start position: " + str(T_start.t))
            self.get_logger().info("End position: " + str(T_end.t))


            tg = rtb.ctraj(T_start, T_end, t=number_of_samples)
            print("Computed trajectory, len is ", len(tg))

            traj = np.zeros((len(tg) + int(self.waiting_time * self.pub_freq), 7))
            # Convert to numpy xyz qx qy qz qw
            for j in range(len(tg)):
                traj[j, 0:3] = tg[j].t
                traj[j, 3:7] = R.from_matrix(tg[j].R).as_quat()
            
            # add waiting time
            for j in range(int(self.waiting_time * self.pub_freq)):
                traj[j + len(tg)] = traj[len(tg) - 1]
            
            traj_list = np.vstack((traj_list, traj))

            print("Trajectory shape: ", traj_list.shape)


        self.trajectory = traj_list
        print("Trajectory ", self.trajectory)
        time.sleep(1.0)


    
    
    def publish_trajectory(self):
        transform_msg = self.get_current_transform()
        if transform_msg is None:
            return

        if self.initial_orientation is None:
            self.initial_orientation = np.array(
                [
                    transform_msg.transform.rotation.x,
                    transform_msg.transform.rotation.y,
                    transform_msg.transform.rotation.z,
                    transform_msg.transform.rotation.w,
                ]
            )
            self.initial_position = np.array(
                [
                    transform_msg.transform.translation.x,
                    transform_msg.transform.translation.y,
                    transform_msg.transform.translation.z,
                ]
            )
            error = np.linalg.norm(self.initial_position - self.starting_position[:3])
            if error > 0.2:
                self.get_logger().warn("Error is too high: " + str(error))
                self.get_logger().error("Initial position is not correct, exiting...")
                exit(1)
            else:
                self.get_logger().info("Initial position is correct, continuing...")
            # if self.camera_to_calibrate == "cam2":
            #     if np.linalg.norm(self.initial_position - self.starting_position2[:3]) > 0.1:
            #         self.get_logger().warn("Initial position is not correct, exiting...")
            #         exit(1)
            #     else:
            #         self.get_logger().info("Initial position is correct, continuing...")
            self.commanded_x = self.initial_position[0]
            self.commanded_y = self.initial_position[1]
            self.commanded_z = self.initial_position[2]
            self.commanded_qx = self.initial_orientation[0]
            self.commanded_qy = self.initial_orientation[1]
            self.commanded_qz = self.initial_orientation[2]
            self.commanded_qw = self.initial_orientation[3]

            self.compute_ctraj()
            self.get_logger().info("Publishing trajectory...")


        self.current_pose = (
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z,
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z,
            transform_msg.transform.rotation.w,
        )

        if self.traj_idx >= len(self.trajectory):
            self.get_logger().info("Trajectory completed.")
            exit(0)
        traj_point = self.trajectory[self.traj_idx]
        self.traj_idx += 1

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = self.base
        target_pose.pose.position.x = traj_point[0]
        target_pose.pose.position.y = traj_point[1]
        target_pose.pose.position.z = traj_point[2]
        target_pose.pose.orientation.x = traj_point[3]
        target_pose.pose.orientation.y = traj_point[4]
        target_pose.pose.orientation.z = traj_point[5]
        target_pose.pose.orientation.w = traj_point[6]

        # Publish the message
        # self.get_logger().info("Publishing target pose: " + str(target_pose.pose.position))
        self.publisher_.publish(target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

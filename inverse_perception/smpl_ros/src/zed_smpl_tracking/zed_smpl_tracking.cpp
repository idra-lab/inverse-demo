#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include "smpl_msgs/msg/smpl.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "utils/json.hpp"
#include "zed_smpl_tracking/ClientPublisher.hpp"
#include "zed_smpl_tracking/bodyConverter.hpp"
#include "zed_smpl_tracking/utils.hpp"
#include "smpl_ros_viewer/smpl_rviz.hpp"

// SMPL -> ROS: equivalent to right-multiplying each row by
// {{0,1,0},{0,0,1},{1,0,0}}, expressed as a left-multiply transpose.
static const Eigen::Matrix3d SMPL_TO_ROS =
    (Eigen::Matrix3d() << 0.0, 0.0, 1.0,
     1.0, 0.0, 0.0,
     0.0, 1.0, 0.0)
        .finished();

void publish_compressed_image_msg(
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub,
    const cv::Mat &image,
    const std::string &frame_id)
{
  auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  msg->header.stamp = rclcpp::Clock().now();
  msg->header.frame_id = frame_id;
  msg->format = "jpeg";
  cv::imencode(".jpg", image, msg->data, {cv::IMWRITE_JPEG_QUALITY, 80});
  pub->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Build a sensor_msgs::msg::CameraInfo from ZED calibration parameters.
// ---------------------------------------------------------------------------
sensor_msgs::msg::CameraInfo build_camera_info(
    const sl::CameraParameters &cam_params,
    const std::string &frame_id,
    const rclcpp::Time &stamp)
{
  sensor_msgs::msg::CameraInfo info;

  info.header.stamp    = stamp;
  info.header.frame_id = frame_id;

  info.width  = static_cast<uint32_t>(cam_params.image_size.width);
  info.height = static_cast<uint32_t>(cam_params.image_size.height);

  // Distortion model: plumb_bob (k1 k2 p1 p2 k3)
  info.distortion_model = "plumb_bob";
  info.d.resize(5);
  for (int i = 0; i < 5; ++i)
    info.d[i] = static_cast<double>(cam_params.disto[i]);

  const double fx = static_cast<double>(cam_params.fx);
  const double fy = static_cast<double>(cam_params.fy);
  const double cx = static_cast<double>(cam_params.cx);
  const double cy = static_cast<double>(cam_params.cy);

  // Intrinsic matrix K (3×3)
  info.k = {fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0};

  // Rectification matrix R – identity (stream is already rectified)
  info.r = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};

  // Projection matrix P (3×4) – monocular, Tx = 0
  info.p = {fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0};

  return info;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("smpl_single_camera_node");

  // ------------------------------------------------------------------ //
  //  ROS Parameters
  // ------------------------------------------------------------------ //
  node->declare_parameter<int>("serial_number", 0);
  const int serial_number = node->get_parameter("serial_number").as_int();

  node->declare_parameter<std::string>("frame_id", "zed_camera_frame");
  const std::string frame_id = node->get_parameter("frame_id").as_string();

  // ------------------------------------------------------------------ //
  //  Publishers — NOTE: all topic names are RELATIVE (no leading slash)
  //  so they are automatically prefixed with the node namespace set by
  //  the launch file (e.g. "camera_1", "camera_2").
  // ------------------------------------------------------------------ //
  auto smpl_pub =
      node->create_publisher<smpl_msgs::msg::Smpl>("smpl_params", 10);
  auto image_pub =
      node->create_publisher<sensor_msgs::msg::Image>("zed/image", 10);
  auto image_compressed_pub =
      node->create_publisher<sensor_msgs::msg::CompressedImage>(
          "zed/image/compressed", 10);
  auto depth_pub =
      node->create_publisher<sensor_msgs::msg::Image>("zed/depth", 10);
  auto camera_info_pub =
      node->create_publisher<sensor_msgs::msg::CameraInfo>(
          "zed/camera_info", 10);
  auto depth_camera_info_pub =
      node->create_publisher<sensor_msgs::msg::CameraInfo>(
          "zed/depth/camera_info", 10);

  SMPLRviz rviz(node, frame_id);

  // ------------------------------------------------------------------ //
  //  Open camera
  // ------------------------------------------------------------------ //
  sl::InputType input_type;
  if (serial_number > 0)
  {
    RCLCPP_INFO(node->get_logger(),
                "Opening ZED with serial number: %d", serial_number);
    input_type.setFromSerialNumber(static_cast<unsigned int>(serial_number));
  }
  else
  {
    RCLCPP_INFO(node->get_logger(),
                "No serial number specified — opening first available ZED");
  }

  ClientPublisher client;
  if (!client.open(input_type,
                   sl::COORDINATE_SYSTEM::IMAGE,
                   sl::RESOLUTION::HD2K,
                   0))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open ZED");
    return 1;
  }

  // Read and cache intrinsics (stable for the whole session)
  const sl::CameraInformation cam_info_sdk =
      client.zed.getCameraInformation();
  const sl::CameraParameters &left_cam =
      cam_info_sdk.camera_configuration.calibration_parameters.left_cam;

  RCLCPP_INFO(node->get_logger(),
              "ZED intrinsics | fx=%.2f fy=%.2f cx=%.2f cy=%.2f res=%dx%d",
              left_cam.fx, left_cam.fy,
              left_cam.cx, left_cam.cy,
              left_cam.image_size.width,
              left_cam.image_size.height);

  RCLCPP_INFO(node->get_logger(),
              "ZED running | frame_id: '%s'", frame_id.c_str());

  rclcpp::Rate rate(15);

  sl::Bodies bodies;
  sl::BodyTrackingRuntimeParameters body_runtime;
  body_runtime.detection_confidence_threshold = 40;

  while (rclcpp::ok())
  {
    if (!client.grab())
    {
      RCLCPP_WARN(node->get_logger(), "Grab failed");
      rate.sleep();
      continue;
    }

    const rclcpp::Time stamp = node->now();

    // CameraInfo is the same for image and depth (same left sensor)
    const auto cam_info_msg = build_camera_info(left_cam, frame_id, stamp);

    // ---------------- IMAGE ----------------
    sl::Mat zed_image;
    if (client.zed.retrieveImage(zed_image, sl::VIEW::LEFT) ==
        sl::ERROR_CODE::SUCCESS)
    {
      cv::Mat cvImage(
          zed_image.getHeight(),
          zed_image.getWidth(),
          CV_8UC4,
          zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
      cv::cvtColor(cvImage, cvImage, cv::COLOR_BGRA2BGR);
      publish_image_msg(image_pub, cvImage, frame_id);
      publish_compressed_image_msg(image_compressed_pub, cvImage, frame_id);
      camera_info_pub->publish(cam_info_msg);
    }

    // ---------------- DEPTH ----------------
    sl::Mat zed_depth;
    if (client.zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH) ==
        sl::ERROR_CODE::SUCCESS)
    {
      cv::Mat cvDepth(
          zed_depth.getHeight(),
          zed_depth.getWidth(),
          CV_32FC1,
          zed_depth.getPtr<sl::uchar1>(sl::MEM::CPU));

      auto depth_msg = cv_bridge::CvImage(
                           std_msgs::msg::Header{},
                           "32FC1",
                           cvDepth)
                           .toImageMsg();
      depth_msg->header.stamp    = stamp;
      depth_msg->header.frame_id = frame_id;
      depth_pub->publish(*depth_msg);
      depth_camera_info_pub->publish(cam_info_msg);
    }

    // ---------------- HUMAN ----------------
    if (client.zed.retrieveBodies(bodies, body_runtime) ==
            sl::ERROR_CODE::SUCCESS &&
        !bodies.body_list.empty())
    {
      const auto &body = bodies.body_list[0];
      if (body.keypoint.size() == 0)
      {
        RCLCPP_WARN(node->get_logger(), "Empty keypoints");
        rclcpp::spin_some(node);
        rate.sleep();
        continue;
      }

      Eigen::Matrix<double, 24, 3> kp_raw;
      for (int smpl_idx = 0; smpl_idx < 24; ++smpl_idx)
      {
        int zed_idx = SMPL_TO_ZED[smpl_idx];
        if (zed_idx < static_cast<int>(body.keypoint.size()))
          kp_raw.row(smpl_idx) << body.keypoint[zed_idx].x,
              body.keypoint[zed_idx].y,
              body.keypoint[zed_idx].z;
        else
          kp_raw.row(smpl_idx).setZero();
      }

      Eigen::Matrix<double, 24, 3> kp = kp_raw;
      rviz.publish_upper_body(kp, stamp);

      auto bodies_out = extractBodyData({bodies.body_list[0]}, SMPL_TO_ZED);
      auto smpl_msg = buildSMPLMessage(bodies_out[0], smpl_to_ros_transform(), {});
      // smpl_pub->publish(smpl_msg);
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  client.close();
  rclcpp::shutdown();
  return 0;
}
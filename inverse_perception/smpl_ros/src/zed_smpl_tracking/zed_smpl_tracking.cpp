#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("smpl_single_camera_node");

  auto smpl_pub =
      node->create_publisher<smpl_msgs::msg::Smpl>("/smpl_params", 10);
  auto image_pub =
      node->create_publisher<sensor_msgs::msg::Image>("/zed/image", 10);
  auto image_compressed_pub =
      node->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/zed/image/compressed", 10);
  auto depth_pub =
      node->create_publisher<sensor_msgs::msg::Image>("/zed/depth", 10);

  // SMPLRviz uses the same frame_id as the camera so keypoints
  // are in the same coordinate space as the depth/image data.
  std::string frame_id = "zed_camera_frame";
  SMPLRviz rviz(node, frame_id);

  ClientPublisher client;
  if (!client.open(sl::InputType(),
                   sl::COORDINATE_SYSTEM::IMAGE,
                   sl::RESOLUTION::HD2K,
                   0))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open ZED");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "ZED running...");
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
      RCLCPP_INFO(node->get_logger(), "Image OK");
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
      depth_msg->header.stamp = stamp;
      depth_msg->header.frame_id = frame_id;
      depth_pub->publish(*depth_msg);
      RCLCPP_INFO(node->get_logger(), "Depth OK");
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

      // Remap from ZED order to SMPL order using SMPL_TO_ZED lookup table.
      // SMPL_TO_ZED[smpl_idx] = zed_idx, so:
      //   kp_raw.row(smpl_idx) = body.keypoint[zed_idx]
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

      // Apply SMPL -> ROS axis transform
      Eigen::Matrix<double, 24, 3> kp = kp_raw;

      rviz.publish_upper_body(kp, stamp);

      auto bodies_out = extractBodyData({bodies.body_list[0]}, SMPL_TO_ZED);
      auto smpl_msg = buildSMPLMessage(bodies_out[0], smpl_to_ros_transform(), {});
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "SMPL msg betas size: " << smpl_msg.betas.size());
      // smpl_pub->publish(smpl_msg);
      RCLCPP_INFO(node->get_logger(), "SMPL OK");
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }

  client.close();
  rclcpp::shutdown();
  return 0;
}
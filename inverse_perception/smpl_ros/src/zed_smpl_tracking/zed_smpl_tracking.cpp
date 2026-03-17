#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "smpl_msgs/msg/smpl.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "utils/json.hpp"
#include "zed_smpl_tracking/ClientPublisher.hpp"
#include "zed_smpl_tracking/bodyConverter.hpp"
#include "zed_smpl_tracking/utils.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("smpl_single_camera_node");

  auto smpl_pub =
      node->create_publisher<smpl_msgs::msg::Smpl>("/smpl_params", 10);
  auto image_pub =
      node->create_publisher<sensor_msgs::msg::Image>("/zed/image", 10);

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

  rclcpp::Rate rate(15); // match camera FPS
  std::string frame_id = "zed_camera_frame";
  sl::Bodies bodies;
  sl::BodyTrackingRuntimeParameters body_runtime;
  body_runtime.detection_confidence_threshold = 40;

  while (rclcpp::ok())
  {

    // 🔴 THIS is the key line
    if (!client.grab())
    {
      RCLCPP_WARN(node->get_logger(), "Grab failed");
      rate.sleep();
      continue;
    }

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

      RCLCPP_INFO(node->get_logger(), "Image OK");
    }

    // ---------------- HUMAN ----------------

    if (client.zed.retrieveBodies(bodies, body_runtime) ==
            sl::ERROR_CODE::SUCCESS &&
        !bodies.body_list.empty()) {

    }

    if (client.zed.retrieveBodies(bodies, body_runtime) ==
            sl::ERROR_CODE::SUCCESS &&
        !bodies.body_list.empty())
    {

      const auto &body = bodies.body_list[0];

      // // 🔴 critical checks
      if (body.keypoint.size() == 0)
      {
        RCLCPP_WARN(node->get_logger(), "Empty keypoints");
        continue;
      }
      auto bodies_out =
          extractBodyData({bodies.body_list[0]}, SMPL_TO_ZED);

      auto msg =
          buildSMPLMessage(bodies_out[0], smpl_to_ros_transform(), {});

      RCLCPP_INFO_STREAM(node->get_logger(), "SMPL msg betas size: " << msg.betas.size());

      // smpl_pub->publish(msg);

      RCLCPP_INFO(node->get_logger(), "SMPL OK");
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  client.close();
  rclcpp::shutdown();
  return 0;
}
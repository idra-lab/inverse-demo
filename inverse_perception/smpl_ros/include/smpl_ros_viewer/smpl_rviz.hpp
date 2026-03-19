#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Core>
#include <array>
#include <string>
#include "utils/constants.hpp"
#include <unordered_set>
class SMPLRviz
{
public:
  SMPLRviz(rclcpp::Node::SharedPtr node, std::string frame_id = "map")
      : node_(node), frame_id_(std::move(frame_id))
  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/smpl_markers", 10);
    RCLCPP_INFO(node_->get_logger(), "SMPLRviz initialized.");
  }

  void publish(const Eigen::Matrix<double, 24, 3> &kp, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(make_keypoints(kp, stamp));
    arr.markers.push_back(make_skeleton(kp, stamp));
    marker_pub_->publish(arr);
  }

  visualization_msgs::msg::Marker make_keypoints(
      const Eigen::Matrix<double, 24, 3> &kp, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = stamp;
    m.ns = "smpl";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.04;
    m.color.a = 1.0f;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;

    m.points.reserve(24);
    for (int i = 0; i < 24; ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = kp(i, 0);
      pt.y = kp(i, 1);
      pt.z = kp(i, 2);
      m.points.push_back(pt);
    }
    return m;
  }

  visualization_msgs::msg::Marker make_skeleton(
      const Eigen::Matrix<double, 24, 3> &kp, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = stamp;
    m.ns = "smpl";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.a = 1.0f;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;

    for (int i = 0; i < 24; ++i)
    {
      int parent = SMPL_PARENTS[i];
      if (parent < 0)
        continue;
      geometry_msgs::msg::Point p, q;
      p.x = kp(parent, 0);
      p.y = kp(parent, 1);
      p.z = kp(parent, 2);
      q.x = kp(i, 0);
      q.y = kp(i, 1);
      q.z = kp(i, 2);
      m.points.push_back(p);
      m.points.push_back(q);
    }
    return m;
  }

  void publish_upper_body(const Eigen::Matrix<double, 24, 3> &kp, const rclcpp::Time &stamp)
  {
    static const std::vector<int> UPPER = {
        0, 3, 6, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(make_keypoints_subset(kp, stamp, UPPER));
    arr.markers.push_back(make_skeleton_subset(kp, stamp, UPPER));
    marker_pub_->publish(arr);
  }

  visualization_msgs::msg::Marker make_keypoints_subset(
      const Eigen::Matrix<double, 24, 3> &kp,
      const rclcpp::Time &stamp,
      const std::vector<int> &indices)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = stamp;
    m.ns = "smpl";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.04;
    m.color.a = 1.0f;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;

    for (int i : indices)
    {
      geometry_msgs::msg::Point pt;
      pt.x = kp(i, 0);
      pt.y = kp(i, 1);
      pt.z = kp(i, 2);
      m.points.push_back(pt);
    }
    return m;
  }

  visualization_msgs::msg::Marker make_skeleton_subset(
      const Eigen::Matrix<double, 24, 3> &kp,
      const rclcpp::Time &stamp,
      const std::vector<int> &indices)
  {
    // Build a set for O(1) lookup
    const std::unordered_set<int> upper_set(indices.begin(), indices.end());

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = stamp;
    m.ns = "smpl";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.a = 1.0f;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;

    for (int i : indices)
    {
      int parent = SMPL_PARENTS[i];
      // Draw bone only if both child and parent are in the upper body set
      if (parent < 0 || upper_set.find(parent) == upper_set.end())
        continue;
      geometry_msgs::msg::Point p, q;
      p.x = kp(parent, 0);
      p.y = kp(parent, 1);
      p.z = kp(parent, 2);
      q.x = kp(i, 0);
      q.y = kp(i, 1);
      q.z = kp(i, 2);
      m.points.push_back(p);
      m.points.push_back(q);
    }
    return m;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string frame_id_;
};
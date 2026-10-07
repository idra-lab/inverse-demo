// Copyright (c) 2026 Inverse
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// KDL uses a base-frame geometric Jacobian at the configured tip origin,
// ordered [linear; angular], matching the controller's twist convention.
class RobotKinematics {
public:
  bool init(const std::string &urdf_xml, const std::string &base_link,
            const std::string &tip_link,
            const std::vector<std::string> &joint_names,
            const rclcpp::Logger &logger);
  Eigen::Isometry3d forwardKinematics(const Eigen::VectorXd &q) const;
  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q) const;
  Eigen::MatrixXd jacobianPseudoInverse(const Eigen::VectorXd &q,
                                        double damping_max,
                                        double singularity_threshold) const;
  const Eigen::VectorXd &velocityLimits() const { return velocity_limits_; }

private:
  KDL::Chain chain_;
  // Chain index -> controller joint index.
  std::vector<std::size_t> joint_indices_;
  Eigen::VectorXd velocity_limits_;
};

Eigen::Vector3d quaternionLog(const Eigen::Quaterniond &quaternion);
Eigen::Quaterniond quaternionExp(const Eigen::Vector3d &rotation_vector);
Eigen::Matrix<double, 6, 1> poseError(const Eigen::Isometry3d &current,
                                      const Eigen::Isometry3d &desired);
std::string
getRobotDescription(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                    const std::string &source);

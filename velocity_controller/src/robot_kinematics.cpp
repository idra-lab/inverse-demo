// Copyright (c) 2026 Inverse
// SPDX-License-Identifier: Apache-2.0
#include "velocity_controller/robot_kinematics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include <Eigen/SVD>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/parameter_client.hpp>
#include <urdf/model.h>

namespace {
KDL::JntArray chainPositions(const Eigen::VectorXd &q,
                             const std::vector<std::size_t> &indices) {
  KDL::JntArray positions(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    positions(i) = q(static_cast<Eigen::Index>(indices[i]));
  }
  return positions;
}
} // namespace

bool RobotKinematics::init(const std::string &urdf_xml,
                           const std::string &base_link,
                           const std::string &tip_link,
                           const std::vector<std::string> &joint_names,
                           const rclcpp::Logger &logger) {
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(urdf_xml) ||
      !kdl_parser::treeFromUrdfModel(model, tree) ||
      !tree.getChain(base_link, tip_link, chain_)) {
    RCLCPP_ERROR(logger, "Cannot construct kinematic chain from '%s' to '%s'.",
                 base_link.c_str(), tip_link.c_str());
    return false;
  }
  if (chain_.getNrOfJoints() != joint_names.size() ||
      std::unordered_set<std::string>(joint_names.begin(), joint_names.end())
              .size() != joint_names.size()) {
    RCLCPP_ERROR(
        logger,
        "Configured joints must match the movable joints in the chain.");
    return false;
  }
  joint_indices_.clear();
  velocity_limits_.resize(joint_names.size());
  for (const auto &segment : chain_.segments) {
    const auto &joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    const auto found =
        std::find(joint_names.begin(), joint_names.end(), joint.getName());
    const auto urdf_joint = model.getJoint(joint.getName());
    if (found == joint_names.end() || !urdf_joint || !urdf_joint->limits ||
        urdf_joint->mimic) {
      RCLCPP_ERROR(logger,
                   "Joint '%s' is missing, lacks limits, or is a mimic joint.",
                   joint.getName().c_str());
      return false;
    }
    const auto index =
        static_cast<std::size_t>(std::distance(joint_names.begin(), found));
    joint_indices_.push_back(index);
    velocity_limits_(static_cast<Eigen::Index>(index)) =
        urdf_joint->limits->velocity;
  }
  return true;
}

Eigen::Isometry3d
RobotKinematics::forwardKinematics(const Eigen::VectorXd &q) const {
  KDL::ChainFkSolverPos_recursive solver(chain_);
  KDL::Frame frame;
  if (solver.JntToCart(chainPositions(q, joint_indices_), frame) < 0) {
    throw std::runtime_error("KDL forward kinematics failed");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    result.translation()(row) = frame.p(row);
    for (int col = 0; col < 3; ++col) {
      result.linear()(row, col) = frame.M(row, col);
    }
  }
  return result;
}

Eigen::MatrixXd RobotKinematics::jacobian(const Eigen::VectorXd &q) const {
  KDL::ChainJntToJacSolver solver(chain_);
  KDL::Jacobian chain_jacobian(chain_.getNrOfJoints());
  if (solver.JntToJac(chainPositions(q, joint_indices_), chain_jacobian) < 0) {
    throw std::runtime_error("KDL Jacobian calculation failed");
  }
  Eigen::MatrixXd result(6, joint_indices_.size());
  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    result.col(static_cast<Eigen::Index>(joint_indices_[i])) =
        chain_jacobian.data.col(i);
  }
  return result;
}

Eigen::MatrixXd
RobotKinematics::jacobianPseudoInverse(const Eigen::VectorXd &q,
                                       double damping_max,
                                       double singularity_threshold) const {
  const Eigen::MatrixXd j = jacobian(q);
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeThinU |
                                                     Eigen::ComputeThinV);
  const auto &singular_values = svd.singularValues();
  const double sigma_min = singular_values.minCoeff();
  const double ratio = sigma_min / singularity_threshold;
  const double damping_squared =
      sigma_min < singularity_threshold
          ? damping_max * damping_max * (1.0 - ratio * ratio)
          : 0.0;
  Eigen::VectorXd inverse(singular_values.size());
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    const double sigma = singular_values(i);
    inverse(i) = sigma == 0.0 && damping_squared == 0.0
                     ? 0.0
                     : sigma / (sigma * sigma + damping_squared);
  }
  return svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
}

Eigen::Vector3d quaternionLog(const Eigen::Quaterniond &quaternion) {
  Eigen::Quaterniond q = quaternion.normalized();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  const double norm = q.vec().norm();
  if (norm < 1e-12) {
    return 2.0 * q.vec();
  }
  return (2.0 * std::atan2(norm, q.w()) / norm) * q.vec();
}

Eigen::Quaterniond quaternionExp(const Eigen::Vector3d &rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.5 * rotation_vector.x(),
                              0.5 * rotation_vector.y(),
                              0.5 * rotation_vector.z())
        .normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

Eigen::Matrix<double, 6, 1> poseError(const Eigen::Isometry3d &current,
                                      const Eigen::Isometry3d &desired) {
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = current.translation() - desired.translation();
  error.tail<3>() = quaternionLog(
      Eigen::Quaterniond(current.linear() * desired.linear().transpose()));
  return error;
}

std::string
getRobotDescription(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                    const std::string &source) {
  // A separate node lets this synchronous configure-time request spin without
  // adding the controller's already-managed lifecycle node to another executor.
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.context(node->get_node_base_interface()->get_context());
  options.use_global_arguments(false);
  auto client_node = std::make_shared<rclcpp::Node>(
      std::string(node->get_name()) + "_description_client",
      node->get_namespace(), options);
  auto client =
      std::make_shared<rclcpp::SyncParametersClient>(client_node, source);
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Robot description parameter service '%s' unavailable.",
                 source.c_str());
    return {};
  }
  try {
    const auto parameters = client->get_parameters({"robot_description"}, 5s);
    if (parameters.size() == 1 && parameters.front().get_type() ==
                                      rclcpp::ParameterType::PARAMETER_STRING) {
      return parameters.front().as_string();
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "Robot description request failed: %s",
                 error.what());
  }
  return {};
}

// Copyright (c) 2026 Inverse
// SPDX-License-Identifier: Apache-2.0
#include <controller_interface/controller_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "velocity_controller/robot_kinematics.hpp"

namespace {
const std::string robot = R"(
<robot name="test_arm">
  <link name="base"/>
  <link name="first"/>
  <link name="second"/>
  <link name="tip"/>
  <joint name="yaw" type="revolute">
    <parent link="base"/><child link="first"/>
    <origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="2"/>
  </joint>
  <joint name="pitch" type="revolute">
    <parent link="first"/><child link="second"/>
    <origin xyz="0.4 0 0.1"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="3"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="second"/><child link="tip"/>
    <origin xyz="0.3 0.1 0.2" rpy="0.2 0.1 0.4"/>
  </joint>
</robot>)";

TEST(Kinematics, JacobianMatchesFiniteDifferenceInConfiguredJointOrder) {
  RobotKinematics kinematics;
  ASSERT_TRUE(kinematics.init(robot, "base", "tip", {"pitch", "yaw"},
                              rclcpp::get_logger("kinematics_test")));
  EXPECT_DOUBLE_EQ(kinematics.velocityLimits()(0), 3.0);
  EXPECT_DOUBLE_EQ(kinematics.velocityLimits()(1), 2.0);
  Eigen::Vector2d q(0.4, -0.6);
  const auto pose = kinematics.forwardKinematics(q);
  const auto jacobian = kinematics.jacobian(q);
  constexpr double step = 1e-7;
  for (int joint = 0; joint < 2; ++joint) {
    auto displaced = q;
    displaced(joint) += step;
    const auto next = kinematics.forwardKinematics(displaced);
    Eigen::Matrix<double, 6, 1> derivative;
    derivative.head<3>() = (next.translation() - pose.translation()) / step;
    derivative.tail<3>() = quaternionLog(Eigen::Quaterniond(
                               next.linear() * pose.linear().transpose())) /
                           step;
    EXPECT_TRUE(derivative.isApprox(jacobian.col(joint), 1e-6));
  }
  const Eigen::MatrixXd recovered =
      kinematics.jacobianPseudoInverse(q, 0.05, 1e-6) * jacobian;
  EXPECT_TRUE(recovered.isApprox(Eigen::Matrix2d::Identity(), 1e-10));
  // Large damping must attenuate, while retaining finite output.
  const auto damped = kinematics.jacobianPseudoInverse(q, 10.0, 100.0);
  EXPECT_TRUE(damped.allFinite());
  EXPECT_LT((damped * jacobian).norm(), recovered.norm());
}

TEST(Kinematics, PoseErrorUsesCurrentMinusDesiredInBaseFrame) {
  Eigen::Isometry3d desired = Eigen::Isometry3d::Identity();
  desired.linear() =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Isometry3d current = desired;
  current.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  current.linear() =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      desired.linear();
  const auto error = poseError(current, desired);
  EXPECT_TRUE(error.head<3>().isApprox(current.translation(), 1e-10));
  EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d(0, 0, 0.2), 1e-10));
}

TEST(Packaging, PluginCanBeLoaded) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  const auto controller = loader.createSharedInstance(
      "velocity_controller/CartesianPositionController");
  ASSERT_NE(controller, nullptr);
}
} // namespace

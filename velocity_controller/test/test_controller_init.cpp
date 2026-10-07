// Copyright (c) 2026 Inverse
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "velocity_controller/cartesian_position_controller.hpp"

namespace {
class ControllerInit : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ControllerInit, InitializesWithoutOverrides) {
  CartesianPositionController controller;
  ASSERT_EQ(controller.init("default_controller"),
            controller_interface::return_type::OK);
  EXPECT_EQ(controller.get_node()->get_parameter("base_link").as_string(),
            "base_link");
  EXPECT_EQ(controller.get_node()->get_parameter("tip_link").as_string(),
            "tool0");
  EXPECT_EQ(controller.get_node()->get_parameter("command_mode").as_string(),
            "position");
}

TEST_F(ControllerInit, PreservesAutomaticallyDeclaredOverrides) {
  const std::vector<rclcpp::Parameter> overrides{
      {"joints", std::vector<std::string>{"joint_1", "joint_2"}},
      {"base_link", "test_base"},
      {"tip_link", "target_link"},
      {"speed_scaling_topic", "/test_speed_scaling"},
      {"robot_description_source", "test_robot_state_publisher"},
      {"command_mode", "velocity"},
      {"P_gain", 0.0},
  };
  auto options = rclcpp::NodeOptions()
                     .allow_undeclared_parameters(true)
                     .automatically_declare_parameters_from_overrides(true)
                     .parameter_overrides(overrides);
  CartesianPositionController controller;
  ASSERT_EQ(controller.init("override_controller", "", options),
            controller_interface::return_type::OK);
  for (const auto &parameter : overrides) {
    EXPECT_EQ(controller.get_node()->get_parameter(parameter.get_name()),
              parameter);
  }
}
}  // namespace

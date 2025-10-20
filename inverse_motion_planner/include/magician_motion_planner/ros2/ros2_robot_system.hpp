#ifndef MAGICIAN_MOTION_PLANNER_ROS2_ROBOT_SYSTEM_HPP
#define MAGICIAN_MOTION_PLANNER_ROS2_ROBOT_SYSTEM_HPP

#include <optional>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mdv/macros.hpp>
#include <mdv/utils/logging.hpp>
#include <rclcpp/node.hpp>

#include "magician_motion_planner/interfaces/parameters_interface.hpp"
#include "magician_motion_planner/interfaces/robot_system_interface.hpp"

class Ros2RobotSystem : public RobotSystemInterface {
public:
    Ros2RobotSystem(
            mdv::Logger::SharedPtr     logger,
            rclcpp::Node*              node,
            PlannerParameterInterface* parameters
    );

    MDV_NODISCARD Eigen::Affine3d get_transformation(
            const std::string& from_link, const std::string& to_link
    ) const override;

    /**
     * @brief Yields the current position of the end-effector displayed in base link.
     */
    MDV_NODISCARD Se3Pose current_ee_position() const override;

    // clang-format off
    MDV_NODISCARD mdv::Logger&               logger() const noexcept { assert(_logger); return *_logger; }
    MDV_NODISCARD rclcpp::Node&              node() noexcept         { assert(_node); return *_node; }
    MDV_NODISCARD PlannerParameterInterface& parameters() const noexcept   { assert(_parameters); return *_parameters; }

    // clang-format on
private:
    mdv::Logger::SharedPtr             _logger     = nullptr;
    rclcpp::Node*                      _node       = nullptr;
    mutable PlannerParameterInterface* _parameters = nullptr;

    tf2_ros::Buffer            _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    using PoseStamped      = geometry_msgs::msg::PoseStamped;
    using PoseSubscription = rclcpp::Subscription<PoseStamped>::SharedPtr;
    PoseSubscription       _reference_sub      = nullptr;
    std::optional<Se3Pose> _cartesio_reference = std::nullopt;
};

#endif  // MAGICIAN_MOTION_PLANNER_ROS2_ROBOT_SYSTEM_HPP

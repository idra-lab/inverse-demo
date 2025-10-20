#include "magician_motion_planner/ros2/ros2_robot_system.hpp"

#include <gsl/assert>

#include <mdv/ros2/conversions.hpp>

Ros2RobotSystem::Ros2RobotSystem(
        mdv::Logger::SharedPtr     logger,
        rclcpp::Node*              node,
        PlannerParameterInterface* parameters
) :
        _logger(std::move(logger)),
        _node(node),
        _parameters(parameters),
        _tf_buffer(node->get_clock()),
        _tf_listener(_tf_buffer) {
    Expects(_logger);
    Expects(node);
    Expects(parameters);

    const std::string ref_topic =
            fmt::format("/cartesian/{}/current_reference", parameters->get_ee_link());
    _reference_sub = node->create_subscription<PoseStamped>(
            ref_topic,
            rclcpp::QoS(1),
            [this](const PoseStamped::ConstSharedPtr msg) {
                _cartesio_reference = mdv::ros2::get_pose(msg->pose);
            }
    );
}

Eigen::Affine3d
Ros2RobotSystem::get_transformation(
        const std::string& from_link, const std::string& to_link
) const {
    if (from_link.empty()) {
        logger().error(
                "Provided empty 'from_link' parameter to get_transformation() call!"
        );
        throw std::runtime_error(
                "Invalid 'from_link' parameter in Ros2RobotSystem::get_transformation "
                "call"
        );
    }

    if (to_link.empty()) {
        logger().error(
                "Provided empty 'to_link' parameter to get_transformation() call!"
        );
        throw std::runtime_error(
                "Invalid 'to_link' parameter in Ros2RobotSystem::get_transformation "
                "call"
        );
    }

    assert(!from_link.empty());
    assert(!to_link.empty());

    if (from_link == to_link) return Eigen::Affine3d::Identity();

    logger().debug("Retrieving transform from '{}' to '{}'", from_link, to_link);
    const Eigen::Affine3d res = mdv::ros2::get_transform(_tf_buffer.lookupTransform(
            to_link,
            from_link,
            tf2::TimePointZero,
            tf2::Duration(std::chrono::seconds(10))
    ));

    return res;
}

Ros2RobotSystem::Se3Pose
Ros2RobotSystem::current_ee_position() const {
    const std::string ee_link   = parameters().get_ee_link();
    const std::string base_link = parameters().get_base_link();
    const auto tf_pose = Se3Pose::from_affine(get_transformation(ee_link, base_link));
    return _cartesio_reference.value_or(tf_pose);
}

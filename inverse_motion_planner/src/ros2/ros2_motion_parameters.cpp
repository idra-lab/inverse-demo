#include "inverse_motion_planner/ros2/ros2_motion_parameters.hpp"

#include <gsl/assert>

namespace defaults {

const std::string base_link = "base_link";
const std::string ee_link   = "tcp";
const double      dt        = 0.001;  // 10ms

}  // namespace defaults

Ros2PlannerParameters::Ros2PlannerParameters(
        mdv::Logger::SharedPtr logger, rclcpp::Node* node
) :
        _logger(std::move(logger)), _node(node), _dt(defaults::dt) {
    Expects(_logger);
    Expects(_node);
    update_parameters(true);
}

void
Ros2PlannerParameters::update_parameters(bool verbose) {
    if (verbose) {
        logger().debug("Parameter name for the robot base link: 'base_link'");
        logger().debug("Parameter name for the robot end-effector link: 'ee_link'");
        logger().debug(
                "Parameter name for the integration timestep/period: 'integration_dt'"
        );
    };

    _base_link = node().declare_parameter("base_link", defaults::base_link);
    _ee_link   = node().declare_parameter("ee_link", defaults::ee_link);
    _dt        = node().declare_parameter("integration_dt", defaults::dt);

    if (verbose) {
        logger().info("base_link parameter = {}", _base_link);
        logger().info("ee_link parameter = {}", _ee_link);
        logger().info("integration_dt parameter = {}", _dt);
    }
}

std::string
Ros2PlannerParameters::get_base_link() const {
    return _base_link;
}

std::string
Ros2PlannerParameters::get_ee_link() const {
    return _ee_link;
}

double
Ros2PlannerParameters::get_dt() const {
    return _dt;
}

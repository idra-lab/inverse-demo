#ifndef MAGICIAN_MOTION_PLANNER_ROS2_MOTION_PARAMETERS_HPP
#define MAGICIAN_MOTION_PLANNER_ROS2_MOTION_PARAMETERS_HPP

#include <mdv/utils/logging.hpp>
#include <rclcpp/node.hpp>

#include "magician_motion_planner/interfaces/parameters_interface.hpp"

class Ros2PlannerParameters : public PlannerParameterInterface {
public:
    Ros2PlannerParameters(mdv::Logger::SharedPtr logger, rclcpp::Node* node);

    MDV_NODISCARD std::string get_base_link() const override;
    MDV_NODISCARD std::string get_ee_link() const override;
    MDV_NODISCARD double      get_dt() const override;

    void update_parameters(bool verbose = true);

    // clang-format off
    MDV_NODISCARD mdv::Logger&  logger() const noexcept { assert(_logger); return *_logger; }
    MDV_NODISCARD rclcpp::Node& node() noexcept         { assert(_node); return *_node; }

    // clang-format on
private:
    mdv::Logger::SharedPtr _logger = nullptr;
    rclcpp::Node*          _node   = nullptr;

    std::string _base_link;
    std::string _ee_link;
    double      _dt;
};

#endif  // MAGICIAN_MOTION_PLANNER_ROS2_MOTION_PARAMETERS_HPP

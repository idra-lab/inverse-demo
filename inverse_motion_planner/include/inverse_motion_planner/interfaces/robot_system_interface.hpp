#ifndef INVERSE_MOTION_PLANNER_ROBOT_SYSTEM_INTERFACE_HPP
#define INVERSE_MOTION_PLANNER_ROBOT_SYSTEM_INTERFACE_HPP

#include <Eigen/Geometry>
#include <string>

#include <mdv/macros.hpp>
#include <mdv/ros2/se3.hpp>

class RobotSystemInterface {
public:
    virtual ~RobotSystemInterface() = default;

    using Se3Pose = mdv::ros2::SE3;

    /*
     * @brief Yields the transform from "from_link" to "to_link" as an Eigen affine
     * tranformation
     */
    MDV_NODISCARD virtual Eigen::Affine3d get_transformation(
            const std::string& from_link, const std::string& to_link
    ) const = 0;

    /**
     * @brief Yields the current position of the end-effector displayed in base link.
     */
    MDV_NODISCARD virtual Se3Pose current_ee_position() const = 0;

};  // class ParameterInterface


#endif  // INVERSE_MOTION_PLANNER_ROBOT_SYSTEM_INTERFACE_HPP

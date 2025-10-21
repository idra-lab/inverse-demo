#ifndef INVERSE_MOTION_PLANNER_HPP
#define INVERSE_MOTION_PLANNER_HPP

#include <memory>

#include <mdv/macros.hpp>
#include <mdv/ros2/se3.hpp>

#include "inverse_motion_planner/components/motion_queue.hpp"
#include "inverse_motion_planner/interfaces/parameters_interface.hpp"
#include "inverse_motion_planner/interfaces/robot_system_interface.hpp"

class MotionPlanner {
public:
    using Se3Pose   = Motion::Se3Pose;
    using Se3Framed = mdv::ros2::SE3Framed;

    MotionPlanner(
            mdv::Logger::SharedPtr     logger,
            RobotSystemInterface*      system_interface,
            PlannerParameterInterface* parameters
    );

    /**
     * @brief Calls the step function on the ongoing motion, and if possible steps on
     * the next planned motion.
     *
     */
    Se3Pose step();

    MDV_NODISCARD Se3Pose
    display_in(const Se3Framed& pose, const std::string& link_name) const;

    MDV_NODISCARD Se3Pose display_in_base(const Se3Framed& pose) const;

    // clang-format off
    MDV_NODISCARD mdv::Logger&                     logger() const noexcept      { assert(_logger); return *_logger; }
    MDV_NODISCARD RobotSystemInterface&            system() noexcept            { assert(_system); return *_system; }
    MDV_NODISCARD const RobotSystemInterface&      system() const noexcept      { assert(_system); return *_system; }
    MDV_NODISCARD PlannerParameterInterface&       parameters()  noexcept       { assert(_parameters); return *_parameters; }
    MDV_NODISCARD const PlannerParameterInterface& parameters() const noexcept  { assert(_parameters); return *_parameters; }
    MDV_NODISCARD MotionQueue&                     motion_queue() noexcept      { assert(_motion_queue); return *_motion_queue; }
    MDV_NODISCARD Motion&                          current_motion() noexcept    { return motion_queue().current_motion(); }

    // clang-format on

private:
    mutable mdv::Logger::SharedPtr _logger     = nullptr;
    RobotSystemInterface*          _system     = nullptr;
    PlannerParameterInterface*     _parameters = nullptr;

    std::unique_ptr<MotionQueue> _motion_queue;
};


#endif  // INVERSE_MOTION_PLANNER_HPP

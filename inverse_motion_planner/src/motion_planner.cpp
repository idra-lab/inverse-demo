#include "magician_motion_planner/motion_planner.hpp"

#include <cassert>
#include <gsl/assert>

MotionPlanner::MotionPlanner(
        mdv::Logger::SharedPtr     logger,
        RobotSystemInterface*      system_interface,
        PlannerParameterInterface* parameters
) :
        _logger(std::move(logger)), _system(system_interface), _parameters(parameters) {
    Expects(_logger);
    Expects(_system);
    Expects(_parameters);

    _motion_queue =
            std::make_unique<MotionQueue>(system().current_ee_position(), _logger);
    Ensures(_motion_queue);
}

MotionPlanner::Se3Pose
MotionPlanner::step() {
    if (current_motion().is_completed()) motion_queue().try_step_motion();
    current_motion().step();
    return current_motion().current_reference_pose();
};

MotionPlanner::Se3Pose
MotionPlanner::display_in(const Se3Framed& pose, const std::string& link_name) const {
    const Eigen::Affine3d transform =
            system().get_transformation(pose.frame_name, link_name);
    return transform * static_cast<const Se3Pose&>(pose);
};

MotionPlanner::Se3Pose
MotionPlanner::display_in_base(const Se3Framed& pose) const {
    return display_in(pose, parameters().get_base_link());
}

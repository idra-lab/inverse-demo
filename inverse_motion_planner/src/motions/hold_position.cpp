#include "magician_motion_planner/motions/hold_position.hpp"

#include <mdv/ros2/conversions.hpp>

#include "magician_motion_planner/motions/dmp_motion_interface.hpp"

HoldPositionMotion::HoldPositionMotion(
        const Se3Pose&         desired_hold_pose,
        const double           hold_time,
        const double           dt,
        const DmpParameters&   parameters,
        mdv::Logger::SharedPtr logger
) :
        DmpMotionInterface(parameters, dt, std::move(logger)), _hold_time(hold_time) {
    _dmp.goal_state.y() = desired_hold_pose;
}

HoldPositionMotion::Se3Pose
HoldPositionMotion::initial_pose() const {
    return _dmp.goal_state.y();
}

std::string
HoldPositionMotion::describe() const {
    return fmt::format(
            "holding position {} for {} seconds",
            mdv::ros2::describe(final_pose()),
            _hold_time
    );
}

void
HoldPositionMotion::step() {
    DmpMotionInterface::step();
    _elapsed_time += _dmp.dt;
}

bool
HoldPositionMotion::is_completed() const {
    return _elapsed_time > _hold_time;
}

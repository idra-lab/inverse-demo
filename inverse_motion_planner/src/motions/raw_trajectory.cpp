#include "magician_motion_planner/motions/raw_trajectory.hpp"

#include <Eigen/Geometry>

#include <mdv/utils/logging.hpp>
#include <range/v3/all.hpp>

#include "magician_motion_planner/components/motion.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

using Se3Pose = Motion::Se3Pose;
using Twist   = Motion::Twist;

RawTrajectoryMotion::RawTrajectoryMotion(
        const Se3Trajectory&             traj,
        const std::string&               traj_frame,
        const PlannerParameterInterface& params,
        const RobotSystemInterface&      robot
) {
    // Get transformation
    const Eigen::Affine3d transform =
            robot.get_transformation(traj_frame, params.get_base_link());

    const auto display_in_base = [&transform](const Se3Pose& pose) -> Se3Pose {
        return transform * pose;
    };
    _traj        = traj | rv::transform(display_in_base) | rs::to_vector;
    _current_pos = _traj.begin();
}

void
RawTrajectoryMotion::set_current_reference(
        const Se3Pose& new_reference, const Twist& vel_reference
) {
    const double dist = (new_reference.pos - initial_pose().pos).norm();
    if (dist > 0.01) {
        throw std::runtime_error(
                "RawTrajectoryMotion::set_current_reference called with too high "
                "reference error - error: "
                + std::to_string(dist)
        );
    }
}

Se3Pose
RawTrajectoryMotion::current_reference_pose() const {
    return *_current_pos;
}

Twist
RawTrajectoryMotion::current_reference_twist() const {
    return {
            Eigen::Vector3d::Zero(),
            Eigen::Vector4d::Zero(),
    };
}

Se3Pose
RawTrajectoryMotion::initial_pose() const {
    return _traj.front();
}

Se3Pose
RawTrajectoryMotion::final_pose() const {
    return _traj.back();
}

bool
RawTrajectoryMotion::is_completed() const {
    return (_current_pos + 1) == _traj.end();
}

std::string
RawTrajectoryMotion::describe() const {
    return fmt::format("Raw trajectory with {} samples", _traj.size());
}

void
RawTrajectoryMotion::step() {
    if (!is_completed()) ++_current_pos;
}

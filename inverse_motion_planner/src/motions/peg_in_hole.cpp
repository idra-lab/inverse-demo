#include "inverse_motion_planner/motions/peg_in_hole.hpp"

#include <Eigen/Geometry>

#include <mdv/utils/logging.hpp>
#include <range/v3/all.hpp>

#include "inverse_motion_planner/components/motion.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

using Se3Pose = Motion::Se3Pose;
using Twist   = Motion::Twist;

PegInHole::PegInHole() {};
void
PegInHole::set_current_reference(
        const Se3Pose& new_reference, const Twist& vel_reference
) {
    const double dist = (new_reference.pos - initial_pose().pos).norm();
    if (dist > 0.01) {
        throw std::runtime_error(
                "PegInHole::set_current_reference called with too high "
                "reference error - error: "
                + std::to_string(dist)
        );
    }
    _spiral_centre = new_reference.pos;
}

std::vector<Eigen::Vector3d>
PegInHole::generate_spiral(
    int num_points,
    int turns,
    double spacing
) const {
    std::vector<Eigen::Vector3d> spiral_points;
    spiral_points.reserve(num_points);

    const double theta_max = 2.0 * M_PI * turns;
    const double d_theta = theta_max / static_cast<double>(num_points);

    const double z = spiral_centre.z();

    for (int i = 0; i < num_points; ++i) {
        double theta = i * d_theta;
        // Archimedean spiral: r = a + b * theta
        double r = spiral_radius + spacing * theta;

        double x = spiral_centre.x() + r * std::cos(theta);
        double y = spiral_centre.y() + r * std::sin(theta);

        spiral_points.emplace_back(x, y, z);
    }

    return spiral_points;
}

Se3Pose
PegInHole::current_reference_pose() const {
    return *_current_pos;
}

// Twist
// PegInHole::current_reference_twist() const {
//     return {
//             Eigen::Vector3d::Zero(),
//             Eigen::Vector4d::Zero(),
//     };
// }

// Se3Pose
// PegInHole::initial_pose() const {
//     return _traj.front();
// }

// Se3Pose
// PegInHole::final_pose() const {
//     return _traj.back();
// }

// bool
// PegInHole::is_completed() const {
//     return (_current_pos + 1) == _traj.end();
// }

// std::string
// PegInHole::describe() const {
//     return fmt::format("Raw trajectory with {} samples", _traj.size());
// }

// void
// PegInHole::step() {
//     if (!is_completed()) ++_current_pos;
// }

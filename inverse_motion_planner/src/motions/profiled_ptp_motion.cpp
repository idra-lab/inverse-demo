#include "inverse_motion_planner/motions/profiled_ptp_motion.hpp"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <limits>
#include <memory>
#include <stdexcept>

#include <mdv/macros.hpp>
#include <mdv/ros2/conversions.hpp>
#include <ruckig/ruckig.hpp>

namespace {

constexpr double zero_distance_tolerance = 1e-6;

// Hard-coded velocity limits (for safety purposes)
constexpr double max_velocity_th         = 0.25;
constexpr double max_angular_velocity_th = M_PI_4;  // 45deg/s rotation

double
rotation_distance(const Eigen::Quaterniond& from, const Eigen::Quaterniond& to) {
    const double q1_dot_q2 = from.coeffs().dot(to.coeffs());
    const double dot       = std::clamp(std::abs(q1_dot_q2), 0.0, 1.0);
    return 2.0 * std::acos(dot);  // NOLINT magic numbers: known math formula
}

}  // namespace

struct ProfiledPtpMotion::PathProgress {
    ruckig::Ruckig<1>          otg;
    ruckig::InputParameter<1>  input;
    ruckig::OutputParameter<1> output;
    double                     s = 0.0;  // current abscissa

    PathProgress(
            const Se3Pose& from,
            const Se3Pose& target,
            const double   dt,
            double         max_velocity,
            double         max_angular_velocity
    ) :
            otg(dt) {
        max_velocity         = std::min(max_velocity, max_velocity_th);
        max_angular_velocity = std::min(max_angular_velocity, max_angular_velocity_th);

        // ruckig struct init
        input.current_position[0]     = 0.0;
        input.current_velocity[0]     = 0.0;
        input.current_acceleration[0] = 0.0;

        input.target_position[0]     = 1.0;
        input.target_velocity[0]     = 0.0;
        input.target_acceleration[0] = 0.0;

        // Helper variables
        const double linear_distance  = (target.pos - from.pos).norm();
        const double angular_distance = rotation_distance(from.ori, target.ori);
        if (linear_distance <= zero_distance_tolerance
            && angular_distance <= zero_distance_tolerance) {
            s = 1.0;
            return;
        }

        // Max normalised velocity computation
        double max_normalized_velocity = std::numeric_limits<double>::infinity();
        if (linear_distance > zero_distance_tolerance) {
            max_normalized_velocity =
                    std::min(max_normalized_velocity, max_velocity / linear_distance);
        }
        if (angular_distance > zero_distance_tolerance) {
            max_normalized_velocity = std::min(
                    max_normalized_velocity, max_angular_velocity / angular_distance
            );
        }
        input.max_velocity[0] = max_normalized_velocity;

        // Max acceleration velocity computation
        input.max_acceleration[0] = std::min(
                max_normalized_velocity
                        / ProfiledPtpMotion::default_acceleration_ramp_time,
                ProfiledPtpMotion::default_max_normalized_acceleration
        );

        // Ruckig 0.9 requires finite jerk to compute a position trajectory.
        input.max_jerk[0] = input.max_acceleration[0]
                           / ProfiledPtpMotion::default_jerk_ramp_time;
    }

    MDV_NODISCARD double
    step() {
        if (s >= 1.0) return 1.0;

        const auto result = otg.update(input, output);

        if (result < ruckig::Working) [[unlikely]] {
            const std::string msg = fmt::format("Ruckig failed with result {}", result);
            throw std::runtime_error(msg);
        }

        s = std::clamp(output.new_position[0], 0.0, 1.0);

        if (output.time >= output.trajectory.get_duration()
            || result == ruckig::Finished) {
            s = 1.0;
        }

        output.pass_to_input(input);
        return s;
    }
};

ProfiledPtpMotion::ProfiledPtpMotion(
        const Se3Pose& from,
        const Se3Pose& to,  // NOLINT no need of std::move
        const double   dt,
        const double   max_velocity,
        const double   max_angular_velocity
) :
        _initial(from),
        _target(to),
        _dt(dt),
        _max_velocity(max_velocity),
        _max_angular_velocity(max_angular_velocity) {
    // Required for correctness of fast-pimpl
    static_assert(sizeof(PathProgress) <= path_progress_storage_size);
    static_assert(alignof(PathProgress) <= alignof(std::max_align_t));

    if (_dt <= 0.0 || _max_velocity <= 0.0 || _max_angular_velocity <= 0.0) {
        throw std::invalid_argument("ProfiledPtpMotion requires positive limits and dt"
        );
    }

    // NOLINTNEXTLINE
    auto* path_progress = reinterpret_cast<PathProgress*>(_path_progress_storage);
    std::construct_at(
            path_progress, from, _target, _dt, _max_velocity, _max_angular_velocity
    );
}

ProfiledPtpMotion::~ProfiledPtpMotion() {
    // NOLINTNEXTLINE
    auto* path_progress = reinterpret_cast<PathProgress*>(_path_progress_storage);
    std::destroy_at(path_progress);
}

void
ProfiledPtpMotion::set_current_reference(
        const Se3Pose& new_reference, const Twist& /*vel_reference*/
) {
    // Replace the path progress object => recompute interpolation properties
    // NOLINTNEXTLINE
    auto* path_progress = reinterpret_cast<PathProgress*>(_path_progress_storage);
    std::destroy_at(path_progress);
    std::construct_at(
            path_progress,
            new_reference,
            _target,
            _dt,
            _max_velocity,
            _max_angular_velocity
    );

    _initial = new_reference;
    _s       = 0.0;
}

ProfiledPtpMotion::Se3Pose
ProfiledPtpMotion::final_pose() const {
    return _target;
}

ProfiledPtpMotion::Se3Pose
ProfiledPtpMotion::initial_pose() const {
    return _initial;
}

ProfiledPtpMotion::Se3Pose
ProfiledPtpMotion::current_reference_pose() const {
    if (_s >= 1.0) return _target;
    return pose_at(_s);
}

ProfiledPtpMotion::Twist
ProfiledPtpMotion::current_reference_twist() const {
    return {Eigen::Vector3d::Zero(), Eigen::Vector4d::Zero()};
}

void
ProfiledPtpMotion::step() {
    _s = path_progress().step();
}

bool
ProfiledPtpMotion::is_completed() const {
    return _s >= 1.0;
}

std::string
ProfiledPtpMotion::describe() const {
    return fmt::format(
            "profiled point-to-point motion from {} to {}",
            mdv::ros2::describe(initial_pose()),
            mdv::ros2::describe(final_pose())
    );
}

ProfiledPtpMotion::PathProgress&
ProfiledPtpMotion::path_progress() noexcept {
    // NOLINTNEXTLINE
    return *std::launder(reinterpret_cast<PathProgress*>(_path_progress_storage));
}

const ProfiledPtpMotion::PathProgress&
ProfiledPtpMotion::path_progress() const noexcept {
    // NOLINTNEXTLINE
    return *std::launder(reinterpret_cast<const PathProgress*>(_path_progress_storage));
}

ProfiledPtpMotion::Se3Pose
ProfiledPtpMotion::pose_at(const double s) const {
    const auto& p0      = _initial.pos;
    const auto  delta_p = _target.pos - _initial.pos;
    const auto& q0      = _initial.ori;
    const auto& q1      = _target.ori;
    return {p0 + s * delta_p, q0.slerp(s, q1)};
}

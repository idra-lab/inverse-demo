#include "inverse_motion_planner/motions/rhytmic_dmp_motion.hpp"

#include <Eigen/Geometry>

#include <mdv/ros2/conversions.hpp>
#include <mdv/utils/logging.hpp>
#include <range/v3/all.hpp>

#include "inverse_motion_planner/components/motion.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

using Se3Pose = Motion::Se3Pose;
using Twist   = Motion::Twist;

RhytmicDmpMotion::Se3Pose
average(const RhytmicDmpMotion::Se3Path& path) {
    using Quat      = Eigen::Quaterniond;
    using Vec7      = Eigen::Vector<double, 7>;
    Vec7       zero = Vec7::Zero();
    const auto add  = [](Vec7 sum, const Se3Pose& pose) -> Vec7 {
        sum.head<3>() = pose.pos;
        sum.tail<4>() = pose.ori.coeffs();
        return sum;
    };
    const Vec7 res = rs::accumulate(path, zero, add) / double(path.size());
    return {res.head<3>(), Quat{res.tail<4>().normalized()}};
}

RhytmicDmpMotion::RhytmicDmpMotion(
        const Se3Path&              path,
        const Se3Pose&              centre,
        double                      dt,
        const RhytmicDmpParameters& params,
        mdv::Logger::SharedPtr      logger
) :
        _dmp(params.n_basis,
             std::make_shared<Se3DmpBase::TransformationSystem>(
                     params.alpha, params.beta
             ),
             std::move(logger)) {
    using namespace std::chrono_literals;
    Se3Path newpath = path;

    const auto& log = _dmp.dmp().logger();
    log.info(
            "Learning RhytmicDmp motion from {} samples - centre: {} - y0: {}",
            path.size(),
            mdv::ros2::describe(centre),
            mdv::ros2::describe(path.front())
    );

    for (std::size_t i = 1; i < newpath.size(); ++i) {
        if (newpath[i].ori.coeffs().dot(newpath[i - 1].ori.coeffs()) < 0.0)
            newpath[i].ori.coeffs() *= -1.0;
    }
    const Demonstration demo = Demonstration::builder()
                                       .assign_position(newpath)
                                       .velocity_automatic_differentiation()
                                       .acceleration_automatic_differentiation()
                                       .set_sampling_period(0.001)
                                       .create();
    const auto& goal = centre;
    _dmp.dmp().learn(demo, 1.0, goal);
    _dmp.dmp().tau = 1.0;
    _dmp.set_sampling_period(dt);

    auto integrated_trajectory = _dmp.dmp().integrate(
            demo.front().y(),
            demo.front().yd(),
            goal,
            1.0,
            200,
            std::chrono::milliseconds(5)
    );

    auto get_sample_velocity = [](const auto& sample) -> double {
        return sample.yd().pos.norm();
    };
    const double dmp_max_vel =
            rs::max(integrated_trajectory | rv::transform(get_sample_velocity));
    _dmp.dmp().tau = dmp_max_vel / params.max_vel;
    _dmp.dmp().logger().debug(
            "Optimal tau: {} (with tau = 1 the max. vel. is {}; maximum vel. allowed: "
            "{})",
            _dmp.dmp().tau,
            dmp_max_vel,
            params.max_vel
    );

    _dmp.goal_state.y() = goal;
    _initial_pose       = demo.front().y();

    _n_revs      = 0;
    _target_revs = params.num_revolutions;
}

void
RhytmicDmpMotion::set_current_reference(
        const Se3Pose& new_reference, const Twist& vel_reference
) {
    _dmp.curr_state.y()  = new_reference;
    _dmp.curr_state.yd() = vel_reference;

    if (initial_pose().ori.coeffs().dot(new_reference.ori.coeffs()) < 0.0)
        _dmp.curr_state.y().ori.coeffs() *= -1.0;
}

Se3Pose
RhytmicDmpMotion::current_reference_pose() const {
    return _dmp.curr_state.y();
}

Twist
RhytmicDmpMotion::current_reference_twist() const {
    return _dmp.curr_state.yd();
}

Se3Pose
RhytmicDmpMotion::initial_pose() const {
    return _initial_pose;
}

Se3Pose
RhytmicDmpMotion::final_pose() const {
    return _initial_pose;
}

bool
RhytmicDmpMotion::is_completed() const {
    return _n_revs >= _target_revs;
}

std::string
RhytmicDmpMotion::describe() const {
    mdv::ros2::describe(initial_pose());
    return fmt::format(
            "Periodic trajectory: initial pose {} - center {}",
            mdv::ros2::describe(initial_pose()),
            mdv::ros2::describe(_dmp.goal_state.y())
    );
}

void
RhytmicDmpMotion::step() {
    const double s_old = _dmp.s;
    _dmp.step();
    if (_dmp.s < s_old) ++_n_revs;
}

#include "inverse_motion_planner/motions/discrete_dmp_motion.hpp"

#include <mdv/ros2/conversions.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view/transform.hpp>
#include <rclcpp/parameter_map.hpp>

#include "inverse_motion_planner/components/interpolation.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <mdv/rerun.hpp>
#include <rerun.hpp>
#endif  // MDV_WITH_RERUN_SDK

namespace rs = ::ranges;
namespace rv = ::ranges::views;

void try_rerun_plot_trajectories(const auto& demonstration, const auto& integration);

DiscreteDmpMotion::DiscreteDmpMotion(
        const Se3Path&         path,
        const double           dt,
        const DmpParameters&   parameters,
        mdv::Logger::SharedPtr logger
) :
        DmpMotionInterface(parameters, dt, std::move(logger)) {
    Se3Path newpath = path;

    for (std::size_t i = 1; i < newpath.size(); ++i) {
        if (newpath[i].ori.coeffs().dot(newpath[i - 1].ori.coeffs()) < 0.0)
            newpath[i].ori.coeffs() *= -1.0;
    }

    const auto demo = Demonstration::builder(newpath.size())
                              .assign_position(newpath)
                              .velocity_automatic_differentiation()
                              .acceleration_automatic_differentiation()
                              .set_sampling_period(0.01)
                              .create();
    _dmp.dmp().learn(demo);

    // Optimise tau
    _dmp.goal_state.y() = demo.back().y();
    _dmp.dmp().tau      = 1.0;

    if ((demo.front().y().pos - demo.back().y().pos).norm() < 1e-2) return;

    optimise_tau(demo.front().y(), demo.back().y(), parameters.max_vel);
};

DiscreteDmpMotion::DiscreteDmpMotion(
        const Eigen::MatrixXd& dmp_weights,
        const Se3Pose&         y0,
        const Se3Pose&         g,
        double                 dt,
        const DmpParameters&   parameters,
        mdv::Logger::SharedPtr logger
) :
        DmpMotionInterface(parameters, dt, std::move(logger)) {
    _dmp.dmp().weights() = dmp_weights;
    _dmp.goal_state.y()  = g;
    optimise_tau(y0, g, parameters.max_vel);
}

void
DiscreteDmpMotion::optimise_tau(
        const Se3Pose& y0, const Se3Pose& g, const double v_max
) {
    // Optimise tau
    _dmp.dmp().tau = 1.0;
    auto integrated_trajectory =
            _dmp.dmp().integrate(y0, g, 200, std::chrono::milliseconds(5));
    auto get_sample_velocity = [](const auto& sample) -> double {
        return sample.yd().pos.norm();
    };
    const double dmp_max_vel =
            rs::max(integrated_trajectory | rv::transform(get_sample_velocity));
    _dmp.dmp().tau = dmp_max_vel / v_max;
    _dmp.dmp().logger().debug(
            "Optimal tau: {} (with tau = 1 the max. vel. is {}; maximum vel. allowed: "
            "{})",
            _dmp.dmp().tau,
            dmp_max_vel,
            v_max
    );
}

DiscreteDmpMotion::UniquePtr
DiscreteDmpMotion::linear_interpolation(
        const Se3Pose&                         from,
        const Se3Pose&                         to,
        const double                           dt,
        const DmpParameters&                   parameters,
        const NormalisedPathProfilerInterface& profiler,
        mdv::Logger::SharedPtr                 logger
) {
    logger->debug(
            "Creating linear interpolation trajectory from {} to {}",
            mdv::ros2::describe(from),
            mdv::ros2::describe(to)
    );
    const Se3Path path = linear_ptp_interpolation(from, to, profiler, 60);
    return std::make_unique<DiscreteDmpMotion>(path, dt, parameters, std::move(logger));
};

DiscreteDmpMotion::Se3Pose
DiscreteDmpMotion::initial_pose() const {
    return _initial_pose;
}

bool
DiscreteDmpMotion::is_completed() const {
    constexpr double max_s       = 0.3;
    constexpr double max_pos_err = 0.01;
    constexpr double vel_th      = 0.05;
    const double     pos_err = (current_reference_pose().pos - final_pose().pos).norm();
    const double     vel     = (_dmp.curr_state.yd().pos).norm();
    return (_dmp.s < max_s) && (pos_err < max_pos_err) && (vel < vel_th);
}

std::string
DiscreteDmpMotion::describe() const {
    return fmt::format(
            "discrete dmp motion from {} to {}",
            mdv::ros2::describe(initial_pose()),
            mdv::ros2::describe(final_pose())
    );
}

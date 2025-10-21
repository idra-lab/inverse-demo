#include "inverse_motion_planner/motions/discrete_dmp_motion.hpp"

#include <mdv/ros2/conversions.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view/transform.hpp>

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
    _dmp.dmp().tau      = 1.0;
    _dmp.goal_state.y() = demo.back().y();
    _initial_pose       = demo.front().y();

    if ((demo.front().y().pos - demo.back().y().pos).norm() < 1e-2) return;

    auto integrated_trajectory = _dmp.dmp().integrate(
            demo.front().y(), demo.back().y(), 200, std::chrono::milliseconds(5)
    );
    auto get_sample_velocity = [](const auto& sample) -> double {
        return sample.yd().pos.norm();
    };
    const double dmp_max_vel =
            rs::max(integrated_trajectory | rv::transform(get_sample_velocity));
    _dmp.dmp().tau = dmp_max_vel / parameters.max_vel;
    _dmp.dmp().logger().debug(
            "Optimal tau: {} (with tau = 1 the max. vel. is {}; maximum vel. allowed: "
            "{})",
            _dmp.dmp().tau,
            dmp_max_vel,
            parameters.max_vel
    );

    try_rerun_plot_trajectories(demo, integrated_trajectory);
};

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

void
try_rerun_plot_trajectories(const auto& demonstration, const auto& integration) {
#if 0
#ifdef MDV_WITH_RERUN_SDK
    // Rerun
    using rerun::Scalars;
    using rerun::archetypes::SeriesLines;
    using rerun::archetypes::SeriesPoints;
    using rerun::components::Color;

    const Color c1(237, 135, 150);
    const Color c2(166, 218, 149);
    const Color c3(138, 173, 244);
    const Color c4(238, 212, 159);

    rerun::RecordingStream rec("DmpLearning");
    mdv::RerunConverter    rr_converter;
    if (rec.spawn().is_ok()) {
        // clang-format off
        rec.log_static("integration/position/px",    SeriesLines().with_colors(c2).with_names("x"));
        rec.log_static("integration/position/py",    SeriesLines().with_colors(c3).with_names("y"));
        rec.log_static("integration/position/pz",    SeriesLines().with_colors(c4).with_names("z"));
        rec.log_static("integration/orientation/qw", SeriesLines().with_colors(c1).with_names("qw"));
        rec.log_static("integration/orientation/qx", SeriesLines().with_colors(c2).with_names("qx"));
        rec.log_static("integration/orientation/qy", SeriesLines().with_colors(c3).with_names("qy"));
        rec.log_static("integration/orientation/qz", SeriesLines().with_colors(c4).with_names("qz"));
        rec.log_static("integration/s3/qw", SeriesPoints().with_colors(c1).with_names("qw"));
        rec.log_static("integration/s3/qx", SeriesPoints().with_colors(c2).with_names("qx"));
        rec.log_static("integration/s3/qy", SeriesPoints().with_colors(c3).with_names("qy"));
        rec.log_static("integration/s3/qz", SeriesPoints().with_colors(c4).with_names("qz"));
        
        rec.log_static("demonstration/position/px",    SeriesPoints().with_colors(c2).with_names("x"));
        rec.log_static("demonstration/position/py",    SeriesPoints().with_colors(c3).with_names("y"));
        rec.log_static("demonstration/position/pz",    SeriesPoints().with_colors(c4).with_names("z"));
        rec.log_static("demonstration/orientation/qw", SeriesPoints().with_colors(c1).with_names("qw"));
        rec.log_static("demonstration/orientation/qx", SeriesPoints().with_colors(c2).with_names("qx"));
        rec.log_static("demonstration/orientation/qy", SeriesPoints().with_colors(c3).with_names("qy"));
        rec.log_static("demonstration/orientation/qz", SeriesPoints().with_colors(c4).with_names("qz"));

        const auto& traj = integration;

        const double dt_demo = 0.1;
        const double dt_integration= dt_demo * double(demonstration.size()) / double(traj.size());

        for(long i = 0; i < demonstration.size(); ++i) {
            rec.set_time_sequence("tick", i);
            rec.set_time_seconds("time", double(i) * dt_demo);
            rec.log("demonstration/position/px",    Scalars(demonstration[i].y().pos(0)));
            rec.log("demonstration/position/py",    Scalars(demonstration[i].y().pos(1)));
            rec.log("demonstration/position/pz",    Scalars(demonstration[i].y().pos(2)));
            rec.log("demonstration/orientation/qw", Scalars(demonstration[i].y().ori.w()));
            rec.log("demonstration/orientation/qx", Scalars(demonstration[i].y().ori.x()));
            rec.log("demonstration/orientation/qy", Scalars(demonstration[i].y().ori.y()));
            rec.log("demonstration/orientation/qz", Scalars(demonstration[i].y().ori.z()));
        }
        for(long i = 0; i < traj.size(); ++i) {
            assert(mdv::condition::is_unit_norm(traj[i].y().ori.coeffs()));
            rec.set_time_sequence("tick", i);
            rec.set_time_seconds("time", double(i) * dt_integration);
            rec.log("integration/position/px",    Scalars(traj[i].y().pos(0)));
            rec.log("integration/position/py",    Scalars(traj[i].y().pos(1)));
            rec.log("integration/position/pz",    Scalars(traj[i].y().pos(2)));
            rec.log("integration/orientation/qw", Scalars(traj[i].y().ori.w()));
            rec.log("integration/orientation/qx", Scalars(traj[i].y().ori.x()));
            rec.log("integration/orientation/qy", Scalars(traj[i].y().ori.y()));
            rec.log("integration/orientation/qz", Scalars(traj[i].y().ori.z()));
        }
        // clang-format on
    }
#endif  // MDV_WITH_RERUN_SDK
#endif
}

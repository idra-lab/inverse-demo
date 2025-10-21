#ifndef INVERSE_MOTION_PLANNER_DISCRETE_DMP_MOTION_HPP
#define INVERSE_MOTION_PLANNER_DISCRETE_DMP_MOTION_HPP

#include <mdv/utils/logging.hpp>

#include "inverse_motion_planner/components/path_profiler.hpp"
#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

class DiscreteDmpMotion : public DmpMotionInterface {
public:
    static constexpr double default_max_velocity = 0.25;

    using UniquePtr = std::unique_ptr<DiscreteDmpMotion>;

    DiscreteDmpMotion(
            const Se3Path&         path,
            double                 dt,
            const DmpParameters&   parameters = DmpParameters(),
            mdv::Logger::SharedPtr logger     = mdv::get_default_logger()
    );

    DiscreteDmpMotion(
            const Eigen::MatrixXd& dmp_weights, 
            const Se3Pose& y0,
            const Se3Pose& g,
            double                 dt,
            const DmpParameters&   parameters = DmpParameters(),
            mdv::Logger::SharedPtr logger     = mdv::get_default_logger()
    );

    static UniquePtr linear_interpolation(
            const Se3Pose&                         from,
            const Se3Pose&                         to,
            double                                 dt,
            const DmpParameters&                   parameters = DmpParameters(),
            const NormalisedPathProfilerInterface& profiler   = default_path_profiler(),
            mdv::Logger::SharedPtr                 logger = mdv::get_default_logger()

    );

    MDV_NODISCARD Se3Pose initial_pose() const final;

    MDV_NODISCARD bool is_completed() const override;

    MDV_NODISCARD std::string describe() const override;

    void optimise_tau(const Se3Pose& y0, const Se3Pose& g, double v_max);

private:
    Se3Pose _initial_pose;
};


#endif  // INVERSE_MOTION_PLANNER_DISCRETE_DMP_MOTION_HPP

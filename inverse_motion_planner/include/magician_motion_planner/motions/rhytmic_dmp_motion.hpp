#ifndef MAGICIAN_MOTION_PLANNER_RHYTMIC_DMP_MOTION_HPP
#define MAGICIAN_MOTION_PLANNER_RHYTMIC_DMP_MOTION_HPP

#include <string>

#include <mdv/containers/demonstration.hpp>
#include <mdv/dmp/rhythmic_dmp.hpp>
#include <mdv/riemann_geometry/se3.hpp>

#include "magician_motion_planner/components/motion.hpp"

struct RhytmicDmpParameters {
    double      alpha           = 48.0;
    double      beta            = 12.0;
    std::size_t n_basis         = 30;
    double      max_vel         = 0.05;
    std::size_t num_revolutions = 1;
};

class RhytmicDmpMotion : public Motion {
public:
    using Se3Path       = std::vector<Se3Pose>;
    using Demonstration = mdv::Demonstration<mdv::riemann::SE3, 2, double>;
    using Se3DmpBase    = mdv::RhytmicDmp<mdv::riemann::SE3>;
    using Se3Dmp        = mdv::IntegrableRhytmicDmp<Se3DmpBase>;

    RhytmicDmpMotion(
            const Se3Path&              traj,
            const Se3Pose&              centre,
            double                      dt,
            const RhytmicDmpParameters& params = RhytmicDmpParameters{},
            mdv::Logger::SharedPtr      logger = mdv::get_default_logger()
    );

    void set_current_reference(const Se3Pose& new_reference, const Twist& vel_reference)
            final;

    MDV_NODISCARD Se3Pose final_pose() const final;
    MDV_NODISCARD Se3Pose initial_pose() const final;
    MDV_NODISCARD Se3Pose current_reference_pose() const final;
    MDV_NODISCARD Twist   current_reference_twist() const final;
    void                  step() override;
    MDV_NODISCARD bool    is_completed() const final;
    MDV_NODISCARD std::string describe() const final;

private:
    Se3Dmp      _dmp;
    Se3Pose     _initial_pose;
    std::size_t _n_revs;
    std::size_t _target_revs;
};

#endif  // MAGICIAN_MOTION_PLANNER_RHYTMIC_DMP_MOTION_HPP

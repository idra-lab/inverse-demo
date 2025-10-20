#ifndef MAGICIAN_MOTION_PLANNER_DMP_MOTION_INTERFACE_HPP
#define MAGICIAN_MOTION_PLANNER_DMP_MOTION_INTERFACE_HPP

#include <mdv/dmp/dmp.hpp>
#include <mdv/macros.hpp>
#include <mdv/riemann_geometry/se3.hpp>
#include <mdv/utils/logging.hpp>

#include "magician_motion_planner/components/motion.hpp"

struct DmpParameters;

class DmpMotionInterface : public Motion {
public:
    using Se3Path       = std::vector<Se3Pose>;
    using Demonstration = mdv::Demonstration<mdv::riemann::SE3, 2, double>;
    using Se3DmpBase    = mdv::Dmp<mdv::riemann::SE3>;
    using Se3Dmp        = mdv::IntegrableDmp<Se3DmpBase>;

    DmpMotionInterface(
            const DmpParameters&   parameters,
            double                 dt,
            mdv::Logger::SharedPtr logger = mdv::get_default_logger()
    );

    void set_current_reference(const Se3Pose& new_reference, const Twist& vel_reference)
            final;

    MDV_NODISCARD Se3Pose current_reference_pose() const final;

    MDV_NODISCARD Twist current_reference_twist() const final;

    MDV_NODISCARD Se3Pose final_pose() const final;

    void step() override;

    // clang-format off
    MDV_NODISCARD Se3Dmp&       dmp()       { return _dmp; }
    MDV_NODISCARD const Se3Dmp& dmp() const { return _dmp; }

    // clang-format on

protected:
    Se3Dmp _dmp;
};

struct DmpParameters {
    using basis_size_t   = DmpMotionInterface::Se3DmpBase::basis_size_t;
    double       alpha   = 48.0;
    double       beta    = 12.0;
    double       gamma   = 3.0;
    basis_size_t n_basis = 12;
    double       max_vel = 0.25;
};


#endif  // MAGICIAN_MOTION_PLANNER_DMP_MOTION_INTERFACE_HPP

#include "inverse_motion_planner/components/path_profiler.hpp"

#include <Eigen/Dense>

#include <mdv/dmp/dmp_utilities.hpp>
#include <range/v3/all.hpp>


namespace rs = ::ranges;
namespace rv = ::ranges::views;

class DefaultProfiler : public NormalisedPathProfilerInterface {
public:
    ~DefaultProfiler() override = default;

    MDV_NODISCARD std::vector<double>
                  build_profile(std::size_t n_samples) const override {
        return mdv::poly_5th(
                Eigen::VectorXd::LinSpaced(static_cast<long>(n_samples), 0.0, 1.0)
        );
    }
};

NormalisedPathProfilerInterface&
default_path_profiler() {
    static DefaultProfiler default_profiler;
    return default_profiler;
}

std::vector<double>
ConstantVelocityPathProfiler::build_profile(std::size_t n_samples) const {
    const Eigen::VectorXd res =
            Eigen::VectorXd::LinSpaced(static_cast<long>(n_samples), 0.0, 1.0);
    return res | rs::to<std::vector<double>>;
}

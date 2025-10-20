#include "magician_motion_planner/components/path_profiler.hpp"

#include <Eigen/Dense>

#include <range/v3/all.hpp>

namespace rs = ::ranges;
namespace rv = ::ranges::views;

class DefaultProfiler : public NormalisedPathProfilerInterface {
public:
    ~DefaultProfiler() override = default;

    MDV_NODISCARD std::vector<double>
                  build_profile(std::size_t n_samples) const override {
        std::size_t pre_post_samples = n_samples / 10 - 1;
        std::size_t midsamples       = n_samples - 2 * pre_post_samples;
        assert(pre_post_samples < n_samples);

        const Eigen::VectorXd ramp =
                Eigen::VectorXd::LinSpaced(static_cast<long>(midsamples), 0.0, 1.0);

        const auto begin = rv::repeat(0.0) | rv::take(pre_post_samples);
        const auto end   = rv::repeat(1.0) | rv::take(pre_post_samples);
        return rv::concat(begin, ramp, end) | rs::to_vector;
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

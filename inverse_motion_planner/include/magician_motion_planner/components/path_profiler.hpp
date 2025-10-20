#ifndef MAGICIAN_MOTION_PLANNER_PATH_PROFILER_HPP
#define MAGICIAN_MOTION_PLANNER_PATH_PROFILER_HPP

#include <vector>

#include <mdv/macros.hpp>

class NormalisedPathProfilerInterface {
public:
    virtual ~NormalisedPathProfilerInterface() = default;

    MDV_NODISCARD virtual std::vector<double> build_profile(std::size_t n_samples
    ) const = 0;
};

class ConstantVelocityPathProfiler : public NormalisedPathProfilerInterface {
public:
    ~ConstantVelocityPathProfiler() override = default;

    MDV_NODISCARD std::vector<double> build_profile(std::size_t n_samples
    ) const override;
};

NormalisedPathProfilerInterface& default_path_profiler();


#endif  // MAGICIAN_MOTION_PLANNER_PATH_PROFILER_HPP

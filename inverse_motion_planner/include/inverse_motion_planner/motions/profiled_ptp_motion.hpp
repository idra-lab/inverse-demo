#ifndef MAGICIAN_MOTION_PLANNER_PROFILED_PTP_MOTION_HPP
#define MAGICIAN_MOTION_PLANNER_PROFILED_PTP_MOTION_HPP

#include <cstddef>
#include <string>
#include <vector>

#include <mdv/macros.hpp>

#include "inverse_motion_planner/components/motion.hpp"

class ProfiledPtpMotion : public Motion {
public:
    static constexpr double default_max_angular_velocity        = 0.15;
    static constexpr double default_acceleration_ramp_time      = 2.0;
    static constexpr double default_jerk_ramp_time               = 0.1;
    static constexpr double default_max_normalized_acceleration = 1.0;

    ProfiledPtpMotion(
            const Se3Pose& from,
            const Se3Pose& to,
            double         dt,
            double         max_velocity,
            double         max_angular_velocity = default_max_angular_velocity
    );
    ~ProfiledPtpMotion() final;

    void set_current_reference(const Se3Pose& new_reference, const Twist& vel_reference)
            final;

    MDV_NODISCARD Se3Pose final_pose() const final;
    MDV_NODISCARD Se3Pose initial_pose() const final;
    MDV_NODISCARD Se3Pose current_reference_pose() const final;
    MDV_NODISCARD Twist   current_reference_twist() const final;
    void                  step() final;
    MDV_NODISCARD bool    is_completed() const final;
    MDV_NODISCARD std::string describe() const final;

private:
    struct PathProgress;

    MDV_NODISCARD PathProgress&       path_progress() noexcept;
    MDV_NODISCARD const PathProgress& path_progress() const noexcept;
    MDV_NODISCARD Se3Pose             pose_at(double s) const;

    static constexpr std::size_t path_progress_storage_size = 4096;

    Se3Pose _initial;
    Se3Pose _target;
    double  _dt;
    double  _max_velocity;
    double  _max_angular_velocity;
    double  _s = 0.0;  // current curvilinear abscissa coordinate

    alignas(std::max_align_t
    ) std::byte _path_progress_storage[path_progress_storage_size] = {};
};

#endif  // MAGICIAN_MOTION_PLANNER_PROFILED_PTP_MOTION_HPP

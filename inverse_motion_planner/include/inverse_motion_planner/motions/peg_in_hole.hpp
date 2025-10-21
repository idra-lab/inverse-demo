#ifndef INVERSE_MOTION_PLANNER_RAW_TRAJECTORY_MOTION_HPP
#define INVERSE_MOTION_PLANNER_RAW_TRAJECTORY_MOTION_HPP

#include <string>

#include <mdv/riemann_geometry/se3.hpp>

#include "inverse_motion_planner/components/motion.hpp"
#include "inverse_motion_planner/interfaces/parameters_interface.hpp"
#include "inverse_motion_planner/interfaces/robot_system_interface.hpp"

class PegInHole : public Motion {
public:
    using Se3Trajectory = std::vector<Se3Pose>;

    PegInHole();

    void generate_spiral(const Se3Pose& centre);
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
    Se3Trajectory           _traj;
    Se3Trajectory::iterator _current_pos;
    Eigen::Vector3d _spiral_centre;
    double _spiral_radius;
    double _spacing;
};

#endif  // INVERSE_MOTION_PLANNER_RAW_TRAJECTORY_MOTION_HPP

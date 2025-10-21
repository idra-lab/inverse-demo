#ifndef INVERSE_MOTION_PLANNER_INTERPOLATION_HPP
#define INVERSE_MOTION_PLANNER_INTERPOLATION_HPP

#include <vector>

#include <mdv/mesh/fwd.hpp>

#include "inverse_motion_planner/components/motion.hpp"
#include "inverse_motion_planner/components/path_profiler.hpp"

std::vector<Motion::Se3Pose> linear_ptp_interpolation(
        const Motion::Se3Pose&                 from,
        const Motion::Se3Pose&                 to,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
);

/**
 * @brief Constructs a point-to-point motion within 2 points lying on the mesh.
 *
 * The default strategy for the orientation is to encode the z axis to be normal to the
 * surface, and the x axis tangent to the velocity of the curve.
 */
std::vector<Motion::Se3Pose> mesh_ptp_interpolation(
        const mdv::mesh::Mesh&                 mesh,
        const Eigen::Vector3d&                 from,
        const Eigen::Vector3d&                 to,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
);

std::vector<Motion::Se3Pose> mesh_ptp_interpolation_multipoint(
        const mdv::mesh::Mesh&                 mesh,
        const std::vector<Eigen::Vector3d>&    waypoints,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
);

namespace meshmotion {

/**
 * @brief Given a geodesic on a mesh, it encodes its rotation with the z axis normal to
 * the face, and the x axis directed with the tangent velocity. Optionally, the z axis
 * normal can be flipped.
 *
 */
std::vector<Motion::Se3Pose> encode_tangent_velocity_orientation(
        const mdv::mesh::Geodesic& position_traj,
        const mdv::mesh::Mesh&     mesh,
        bool                       flip_z_axis = true
);

/**
 * @brief Rotates each pose on the trajectory around the z axis in order to minimise the
 * rotation of the xy axis w.r.t the previous pose. For the first pose, q0ref is used as
 * reference quaternion.
 *
 */
void minimise_rotation_around_z_axis(
        std::vector<Motion::Se3Pose>& pos_traj, const Eigen::Quaterniond& q0ref
);

/**
 * @brief Given a point on a mesh, creates a pose with the z axis entering the face and
 * the x/y axis that minimise the rotation along the z axis w.r.t. a reference rotation
 * q0ref.
 *
 */
Motion::Se3Pose minimise_rotation_around_z_axis(
        const mdv::mesh::Point& pt, const Eigen::Quaterniond& q0ref
);


}  // namespace meshmotion


#endif  // INVERSE_MOTION_PLANNER_INTERPOLATION_HPP

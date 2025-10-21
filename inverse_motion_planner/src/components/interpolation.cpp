#include "inverse_motion_planner/components/interpolation.hpp"

#include <mdv/mesh/algorithm.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/point.hpp>
#include <range/v3/all.hpp>

#include "inverse_motion_planner/motion_planner.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <mdv/rerun.hpp>
#include <rerun.hpp>
#endif  // MDV_WITH_RERUN_SDK

namespace rs = ::ranges;
namespace rv = ::ranges::views;

namespace {
void try_rerun_plot_mesh_trajectory(
        const mdv::mesh::Mesh&           mesh,
        const mdv::mesh::Geodesic&       path,
        const mdv::mesh::CartesianPoint& y0,
        const mdv::mesh::CartesianPoint& g0
);
}

std::vector<Motion::Se3Pose>
linear_ptp_interpolation(
        const Motion::Se3Pose&                 from,
        const Motion::Se3Pose&                 to,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
) {
    const std::vector<double>    s_coords = profiler.build_profile(n_samples);
    std::vector<Motion::Se3Pose> path;
    path.reserve(s_coords.size());

    for (const double s : s_coords) {
        path.emplace_back(
                from.pos + s * (to.pos - from.pos), from.ori.slerp(s, to.ori)
        );
    }
    return path;
}

std::vector<Motion::Se3Pose>
mesh_ptp_interpolation(
        const mdv::mesh::Mesh&                 mesh,
        const Eigen::Vector3d&                 from_cartesian,
        const Eigen::Vector3d&                 to_cartesian,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
) {
    using mdv::mesh::Geodesic;
    using mdv::mesh::Point;

    const Point    from_mesh = Point::from_cartesian(mesh, from_cartesian);
    const Point    to_mesh   = Point::from_cartesian(mesh, to_cartesian);
    const Geodesic geodesic  = mesh.build_geodesic(from_mesh, to_mesh);

    // Position computation
    // try_rerun_plot_mesh_trajectory(mesh, {}, from_cartesian, to_cartesian);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const std::vector<double> s_coords = profiler.build_profile(n_samples);
    const auto position_path = mdv::mesh::geodesic_resample(geodesic, s_coords);

    try_rerun_plot_mesh_trajectory(mesh, position_path, from_cartesian, to_cartesian);

    return meshmotion::encode_tangent_velocity_orientation(position_path, mesh, true);
}

std::vector<Motion::Se3Pose>
mesh_ptp_interpolation_multipoint(
        const mdv::mesh::Mesh&                 mesh,
        const std::vector<Eigen::Vector3d>&    waypoints,
        const NormalisedPathProfilerInterface& profiler,
        std::size_t                            n_samples
) {
    using mdv::mesh::Geodesic;
    using mdv::mesh::Point;

    std::vector<Geodesic> path_entries =
            waypoints | rv::sliding(2)
            | rv::transform([&mesh](const auto& pts) -> mdv::mesh::Geodesic {
                  return mesh.build_geodesic(
                          Point::from_cartesian(mesh, pts[0]),
                          Point::from_cartesian(mesh, pts[1])
                  );
              })
            | rs::to<std::vector<Geodesic>>;
    Geodesic path = path_entries | rv::join | rs::to<Geodesic>;
    rs::unique(path);

    const auto s_coords       = profiler.build_profile(n_samples);
    const auto resampled_path = mdv::mesh::geodesic_resample(path, s_coords);

    return meshmotion::encode_tangent_velocity_orientation(resampled_path, mesh, true);
}

namespace meshmotion {

/**
 * @brief Given a geodesic on a mesh, it encodes its rotation with the z axis normal to
 * the face, and the x axis directed with the tangent velocity. Optionally, the z axis
 * normal can be flipped.
 *
 */
std::vector<Motion::Se3Pose>
encode_tangent_velocity_orientation(
        const mdv::mesh::Geodesic& position_path,
        const mdv::mesh::Mesh&     mesh,
        bool                       flip_z_axis
) {
    using mdv::mesh::Point;

    std::vector<Motion::Se3Pose> path;
    path.reserve(position_path.size());

    // For the orientation:
    // 1. compute the sequence of vectors normal to the surface
    // 2. compute the (optional) tangent vector component at each point
    // 3. fill missing tangent vectors
    // 4. ensure that x is normal to z
    // 5. joint position and orientation data

    // step 1
    const double z_multi     = flip_z_axis ? -1.0 : 1.0;
    const auto   take_normal = [&mesh, z_multi](const auto& pos) -> Eigen::Vector3d {
        const auto pt = Point::from_cartesian(mesh, pos);
        return z_multi * pt.face().normal();
    };
    const std::vector<Eigen::Vector3d> zs =
            position_path | rv::transform(take_normal) | rs::to_vector;

    // step 2
    const auto take_finite_diff = [](const auto& rng
                                  ) -> std::optional<Eigen::Vector3d> {
        const Eigen::Vector3d& p1   = rng[0];
        const Eigen::Vector3d& p2   = rng[1];
        const auto             diff = (p2 - p1);
        if (diff.norm() < 1e-7) return std::nullopt;
        return (p2 - p1).normalized();
    };
    const std::vector<std::optional<Eigen::Vector3d>> xs_opt =
            rv::concat(position_path, rv::single(position_path.back())) | rv::sliding(2)
            | rv::transform(take_finite_diff) | rs::to_vector;
    assert(xs_opt.size() == zs.size());

    // step 3
    const auto first_x =
            rs::find_if(xs_opt, [](const auto& x) { return x.has_value(); });
    if (first_x == xs_opt.end())
        throw std::runtime_error("invalid tangential component finite difference");
    Eigen::Vector3d              valid_dir = (*first_x).value();
    std::vector<Eigen::Vector3d> xs(xs_opt.size());
    for (std::size_t i = 0; i < xs_opt.size(); ++i) {
        if (xs_opt[i].has_value()) valid_dir = xs_opt[i].value();
        xs[i] = valid_dir;
    }

    // step 4
    for (std::size_t i = 0; i < xs_opt.size(); ++i) {
        using Mat3      = Eigen::Matrix3d;
        const Mat3 proj = Mat3::Identity() - zs[i] * zs[i].transpose();
        xs[i]           = (proj * xs[i]).normalized();
    }

    auto construct_pose = [](auto&& rng) -> MotionPlanner::Se3Pose {
        auto& [x, z, pos] = rng;
        Eigen::Matrix3d rot;
        rot.col(0) = x;
        rot.col(1) = z.cross(x);
        rot.col(2) = z;
        assert(std::abs(rot.determinant() - 1.0) < 1e-7);
        // pose.ori   = Eigen::Quaterniond(rot).normalized();
        return MotionPlanner::Se3Pose(pos, Eigen::Quaterniond(rot));
    };

    return rv::zip(xs, zs, position_path) | rv::transform(construct_pose)
           | rs::to_vector;
}

void
minimise_rotation_around_z_axis(
        std::vector<Motion::Se3Pose>& pos_traj, const Eigen::Quaterniond& q0ref
) {
    using Quat = Eigen::Quaterniond;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    const auto ux = Vec3::UnitX();
    const auto uz = Vec3::UnitZ();

    Quat qprev = q0ref;
    for (std::size_t i = 0; i < pos_traj.size(); ++i) {
        const Quat qcurr  = pos_traj[i].ori;
        const Vec3 z_mesh = qcurr * uz;
        const Mat3 proj   = Mat3::Identity() - z_mesh * z_mesh.transpose();
        pos_traj[i].ori   = Quat::FromTwoVectors(qcurr * ux, proj * qprev * ux) * qcurr;
        qprev             = pos_traj[i].ori;
    }
}

Motion::Se3Pose
minimise_rotation_around_z_axis(const mdv::mesh::Point& pt, const Eigen::Quaterniond& q0ref) {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    const Vec3 z    = -pt.face().normal();
    const Vec3 xref = q0ref * Vec3::UnitX();
    const Vec3 x    = ((Mat3::Identity() - z * z.transpose()) * xref).normalized();
    const Vec3 y    = z.cross(x);

    Mat3 rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    return {pt.position(), Eigen::Quaterniond(rot)};
}

}  // namespace meshmotion

namespace {
void
try_rerun_plot_mesh_trajectory(
        const mdv::mesh::Mesh&           mesh,
        const mdv::mesh::Geodesic&       path,
        const mdv::mesh::CartesianPoint& y0,
        const mdv::mesh::CartesianPoint& g0
) {
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

    static rerun::RecordingStream rec("MeshTrajectory");
    static int                    time_idx = 0;
    mdv::RerunConverter           rr_converter;

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(mesh.num_vertices());
    for (const auto& v : mesh.vertices()) vertices.emplace_back(v.position());

    if (rec.spawn().is_ok()) {
        rec.set_time_sequence("generation_id", time_idx);
        rec.log_static("mesh", rr_converter(mesh));
        if (!path.empty()) rec.log("path", rr_converter(path));
        rec.log("initial_point", rr_converter.as_points({y0, g0}));
        rec.log("vertices", rr_converter.as_points(vertices));
        time_idx++;
    }
#endif  // MDV_WITH_RERUN_SDK
}
}  // namespace

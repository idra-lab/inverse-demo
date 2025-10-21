#include "inverse_motion_planner/components/skill_database.hpp"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <range/v3/all.hpp>

namespace rs = ::ranges;
namespace rv = ::ranges::views;
using json   = nlohmann::json;

std::string
SkillDatabase::default_database() {
    using std::filesystem::path;
    const path share_dir =
            ament_index_cpp::get_package_share_directory("inverse_motion_planner");
    return (share_dir / "motion.json").string();
}

namespace {
json
pose_to_json(const Motion::Se3Pose& pose) {
    json j;
    j["position"]    = {pose.pos(0), pose.pos(1), pose.pos(2)};
    j["orientation"] = {pose.ori.w(), pose.ori.x(), pose.ori.y(), pose.ori.z()};
    return j;
}

Motion::Se3Pose
json_to_pose(const json& j) {
    Eigen::Vector3d pos = {
            j["position"][0].get<double>(),
            j["position"][1].get<double>(),
            j["position"][2].get<double>()};
    Eigen::Quaterniond ori = {
            j["orientation"][0].get<double>(),
            j["orientation"][1].get<double>(),
            j["orientation"][2].get<double>(),
            j["orientation"][3].get<double>()};
    return Motion::Se3Pose{pos, ori};
}

json
matrix_to_json(const Eigen::MatrixXd& mat) {
    json                j;
    std::vector<double> data;
    data.reserve(mat.size());
    std::copy(mat.data(), mat.data() + mat.size(), std::back_inserter(data));
    j["cols"] = mat.cols();
    j["rows"] = mat.rows();
    j["data"] = data;
    return j;
}

Eigen::MatrixXd
json_to_matrix(const json& j) {
    const long rows = j["rows"].get<long>();
    const long cols = j["cols"].get<long>();
    const long size = rows * cols;

    std::vector<double> data = j["data"];
    Eigen::MatrixXd     res(rows, cols);
    rs::copy(data, res.data());
    return res;
}

json
skill_to_json(const SkillDatabase::SkillData& data) {
    json j;
    j["initial_pose"] = pose_to_json(data.initial_pose);
    j["final_pose"]   = pose_to_json(data.final_pose);
    j["dmp_weights"]  = matrix_to_json(data.dmp_weights);
    j["alpha"]        = data.dmp_params.alpha;
    j["beta"]         = data.dmp_params.beta;
    j["gamma"]        = data.dmp_params.gamma;
    j["n_basis"]      = data.dmp_weights.rows();
    return j;
}

SkillDatabase::SkillData
json_to_skill(const json& j) {
    return {
            .initial_pose = json_to_pose(j["initial_pose"]),
            .final_pose   = json_to_pose(j["final_pose"]),
            .dmp_weights  = json_to_matrix(j["dmp_weights"]),
            .dmp_params{
                        .alpha   = j["alpha"].get<double>(),
                        .beta    = j["beta"].get<double>(),
                        .gamma   = j["gamma"].get<double>(),
                        .n_basis = j["n_basis"].get<std::size_t>()}
    };
}

}  // namespace

SkillDatabase::SkillDatabase(
        const std::string& db_file, mdv::Logger::SharedPtr logger_
) :
        _logger(std::move(logger_)) {
    if (!std::filesystem::exists(db_file)) {
        logger().warn("Database file {} is not existing!", db_file);
        return;
    }

    read_database(db_file);
};

void
SkillDatabase::write_database(const std::string& db_file) const {
    logger().info("Writing skill database to file {}", db_file);

    json j;
    for (const auto& [skill_name, skill_data] : _db) {
        j[skill_name] = skill_to_json(skill_data);
        logger().debug("Exported skill {}", skill_name);
    }

    std::ofstream file(db_file);
    file << j.dump(4);
    logger().info("Database written to file");
}

void
SkillDatabase::read_database(const std::string& db_file) {
    logger().info("Reading skill database from file {}", db_file);

    assert(std::filesystem::exists(db_file));
    std::ifstream file(db_file);
    json          j;
    file >> j;

    _db.clear();
    for (auto& e : j.items()) {
        _db[e.key()] = json_to_skill(e.value());
        logger().info("Loaded skill {}", e.key());
    }
    logger().info("Loaded {} skills", _db.size());
}

void
SkillDatabase::add_skill(const std::string& skill_name, const SkillData& data) {
    _db[skill_name] = data;
}

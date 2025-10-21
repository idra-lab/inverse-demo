#ifndef INVERSE_MOTION_PLANNER_SKILL_DATABASE_HPP
#define INVERSE_MOTION_PLANNER_SKILL_DATABASE_HPP

#include <Eigen/Dense>
#include <map>

#include <mdv/utils/logging.hpp>

#include "inverse_motion_planner/components/motion.hpp"
#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

class SkillDatabase {
public:
    struct SkillData {
        Motion::Se3Pose initial_pose;
        Motion::Se3Pose final_pose;
        Eigen::MatrixXd dmp_weights;
        DmpParameters   dmp_params;
    };

    SkillDatabase(
            const std::string&     db_file,
            mdv::Logger::SharedPtr logger = mdv::get_default_logger()
    );

    void write_database(const std::string& db_file) const;

    void read_database(const std::string& db_file);

    void add_skill(const std::string& skill_name, const SkillData& data);

    std::unique_ptr<Motion> get_skill(const std::string& skill_name);

    // clang-format off
    mdv::Logger& logger() const { assert(_logger); return *_logger; }

    // clang-format on

private:
    mutable mdv::Logger::SharedPtr   _logger;
    std::map<std::string, SkillData> _db;
};  // class SkillDatabase

#endif  // INVERSE_MOTION_PLANNER_SKILL_DATABASE_HPP

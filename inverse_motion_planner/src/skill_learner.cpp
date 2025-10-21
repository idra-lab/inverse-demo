#include "inverse_motion_planner/skill_learner.hpp"

#include <chrono>
#include <rmw/qos_profiles.h>

#include <mdv/ros2/conversions.hpp>
#include <mdv/ros2/logger.hpp>
#include <mdv/utils/logging_extras.hpp>
#include <range/v3/all.hpp>
#include <rcl_interfaces/msg/detail/parameter__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/parameter.hpp>

#include "inverse_motion_planner/components/skill_database.hpp"
#include "inverse_motion_planner/motions/discrete_dmp_motion.hpp"
#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

SkillLearner::SkillLearner() : rclcpp::Node("skill_learner") {
    _logger     = std::make_shared<mdv::ros2::RosLogger>(get_logger());
    _parameters = std::make_unique<Ros2PlannerParameters>(_logger, this);
    _system     = std::make_unique<Ros2RobotSystem>(_logger, this, _parameters.get());
    logger().info("Interfaces initialised!");

    _base_link = _parameters->get_base_link();
    _ee_link   = _parameters->get_ee_link();
    logger().info("base link: {}", _base_link);
    logger().info("end-effector link: {}", _ee_link);

    _cli_cbk_group =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _server_cbk_group =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const std::string node_name =
            declare_parameter("node_name", "left_cartesian_impedance_controller");

    auto cbk = [this](const LearnSkill::Request::ConstSharedPtr req,
                      LearnSkill::Response::SharedPtr           resp) {
        on_learn_skill_request(req, resp);
    };
    _learn_skill_server = create_service<LearnSkill>(
            "learn_skill", cbk, rmw_qos_profile_services_default, _server_cbk_group
    );

    _skill_db =
            std::make_unique<SkillDatabase>(SkillDatabase::default_database(), _logger);
}

void
SkillLearner::on_learn_skill_request(
        const LearnSkill::Request::ConstSharedPtr& req,
        LearnSkill::Response::SharedPtr&           resp
) {
    resp->success = false;
    std::vector<Se3Pose> full_demo;
    full_demo.reserve(10000);

    logger().info(
            "Starting listening for a demonstration of {}s",
            req->registration_duration_secs
    );
    const auto sampling_period = std::chrono::milliseconds(10);

    const auto timer = create_wall_timer(sampling_period, [this, &full_demo]() {
        const auto pose = _system->current_ee_position();
        full_demo.push_back(pose);
    });

    using std::chrono::high_resolution_clock;
    const auto start      = high_resolution_clock::now();
    bool       can_record = true;
    while (can_record) {
        const auto   stop  = high_resolution_clock::now();
        const double delta = std::chrono::duration<double>(stop - start).count();
        can_record         = delta < req->registration_duration_secs;
    }
    timer->reset();
    logger().info("Recorded {} entries in the demonstration", full_demo.size());

    const auto y0 = full_demo.front();
    const auto g  = full_demo.back();

    const auto first_sample = std::find_if(
            full_demo.cbegin(),
            full_demo.cend(),
            [y0](const auto& y) -> bool { return (y0.pos - y.pos).norm() > 1e-2; }
    );
    const auto first_sample_id = std::max<long>(
            std::distance(full_demo.cbegin(), first_sample) - 30, 0
    );
    logger().info("First sample id: {}", first_sample_id);

    const auto last_sample = rs::find_if(
            full_demo.crbegin(),
            full_demo.crend(),
            [g](const auto& y) -> bool { return (g.pos - y.pos).norm() > 1e-2; }
    );
    const auto last_sample_id = std::max<std::size_t>(
            std::distance(full_demo.crbegin(), last_sample) + 30, full_demo.size()
    );
    logger().info("Last sample id: {}", last_sample_id);

    std::vector<Se3Pose> demo;
    for (std::size_t i = first_sample_id; i < last_sample_id; ++i)
        demo.emplace_back(full_demo[i]);

    if (demo.size() < 20) {
        resp->success = false;
        logger().error("Processed demonstration has {} samples!", demo.size());
        return;
    }

    DmpParameters p;
    p.n_basis = req->num_basis;
    DiscreteDmpMotion motion(demo, 0.001, p, _logger);

    SkillDatabase::SkillData skill_data;
    skill_data.initial_pose = demo.front();
    skill_data.final_pose   = demo.back();
    skill_data.dmp_params   = p;
    skill_data.dmp_weights  = motion.dmp().dmp().weights();
    _skill_db->add_skill(req->skill_name, skill_data);
    _skill_db->write_database(SkillDatabase::default_database());

    resp->success      = true;
    resp->initial_pose = mdv::ros2::to_pose_message(skill_data.initial_pose);
    resp->final_pose   = mdv::ros2::to_pose_message(skill_data.final_pose);
    resp->total_demonstration_time =
            std::chrono::duration<double>(sampling_period).count() * demo.size();
}

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto                                     node = std::make_shared<SkillLearner>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

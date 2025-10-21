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
}
void
SkillLearner::on_learn_skill_request(
        const LearnSkill::Request::ConstSharedPtr& req,
        LearnSkill::Response::SharedPtr&           resp
) {
    std::vector<Se3Pose> path;
    path.reserve(10000);

    const auto timer =
            create_wall_timer(std::chrono::milliseconds(100), [this, &path]() {
                const auto pose = _system->current_ee_position();
                path.push_back(pose);
                logger().info("{}", mdv::ros2::describe(pose));
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

    logger().info("Recorded {} entries", path.size());
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

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

    _params_getter_cli = create_client<GetParameters>(
            fmt::format("/{}/get_parameters", node_name),
            rmw_qos_profile_services_default,
            _cli_cbk_group
    );
    if (!_params_getter_cli->wait_for_service(std::chrono::seconds(3))) {
        logger().error(
                "Unable to connect to service {}",
                _params_getter_cli->get_service_name()
        );
        std::terminate();
    }
    logger().info(
            "Reading parameters through service {}",
            _params_getter_cli->get_service_name()
    );

    _params_setter_cli = create_client<SetParameters>(
            fmt::format("/{}/get_parameters", node_name),
            rmw_qos_profile_services_default,
            _cli_cbk_group
    );
    if (!_params_setter_cli->wait_for_service(std::chrono::seconds(3))) {
        logger().error(
                "Unable to connect to service {}",
                _params_setter_cli->get_service_name()
        );
        std::terminate();
    }
    logger().info(
            "Reading parameters through service {}",
            _params_setter_cli->get_service_name()
    );

    _stiffness_initial_values = read_stiffness_values();
    logger().info(
            "Backed up stiffness values: {}",
            mdv::eigen_to_str(_stiffness_initial_values)
    );

    auto cbk = [this](const LearnSkill::Request::ConstSharedPtr req,
                      LearnSkill::Response::SharedPtr           resp) {
        on_learn_skill_request(req, resp);
    };
    _learn_skill_server = create_service<LearnSkill>(
            "learn_skill", cbk, rmw_qos_profile_services_default, _server_cbk_group
    );
}

SkillLearner::Vec6
SkillLearner::read_stiffness_values() const {
    using rcl_interfaces::msg::Parameter;

    if (_params_getter_cli == nullptr || !_params_getter_cli->service_is_ready()) {
        logger().error("Service is not ready!");
        return Vec6::Zero();
    }

    Parameter p;
    auto      req = std::make_shared<GetParameters::Request>();
    p.value.type  = 3;
    req->names    = {
               "stiffness.trans_x",
               "stiffness.trans_y",
               "stiffness.trans_z",
               "stiffness.rot_x",
               "stiffness.rot_y",
               "stiffness.rot_z"};

    auto future = _params_getter_cli->async_send_request(req);

    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        logger().error("Unable to retrieve parameters in 2 seconds");
        return Vec6::Zero();
    }

    const auto resp = future.get();

    Vec6 res;
    std::transform(
            resp->values.begin(),
            resp->values.end(),
            res.data(),
            [](const auto& p) -> double { return p.double_value; }
    );
    return res;
}

void
SkillLearner::gcomp_enable() {
    set_stiffness_values(Vec6::Zero());
}

void
SkillLearner::gcomp_disable() {
    set_stiffness_values(_stiffness_initial_values);
}

void
SkillLearner::set_stiffness_values(const Vec6& stiffness) {
    auto req = std::make_shared<SetParameters::Request>();
    using rcl_interfaces::msg::Parameter;
    Parameter p;

    p.value.type = 3;

    p.name               = "stiffness.trans_x";
    p.value.double_value = stiffness(0);
    req->parameters.push_back(p);

    p.name               = "stiffness.trans_y";
    p.value.double_value = stiffness(1);
    req->parameters.push_back(p);

    p.name               = "stiffness.trans_z";
    p.value.double_value = stiffness(2);
    req->parameters.push_back(p);

    p.name               = "stiffness.rot_x";
    p.value.double_value = stiffness(3);
    req->parameters.push_back(p);

    p.name               = "stiffness.rot_y";
    p.value.double_value = stiffness(4);
    req->parameters.push_back(p);

    p.name               = "stiffness.rot_z";
    p.value.double_value = stiffness(5);
    req->parameters.push_back(p);

    auto future = _params_setter_cli->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        logger().error("Didn't hear a reply in 5 seconds");
        return;
    }
}

void
SkillLearner::on_learn_skill_request(
        const LearnSkill::Request::ConstSharedPtr& req,
        LearnSkill::Response::SharedPtr&           resp
) {
    std::vector<Se3Pose> path;
    path.reserve(10000);

    const auto timer =
            create_wall_timer(std::chrono::milliseconds(10), [this, &path]() {
                path.push_back(_system->current_ee_position());
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
    node->gcomp_enable();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

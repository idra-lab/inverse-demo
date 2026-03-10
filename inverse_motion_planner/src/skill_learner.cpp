#include "inverse_motion_planner/skill_learner.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include <rmw/qos_profiles.h>

#include <mdv/ros2/conversions.hpp>
#include <mdv/ros2/logger.hpp>
#include <mdv/utils/logging_extras.hpp>
#include <range/v3/all.hpp>

#include "inverse_motion_planner/components/skill_database.hpp"
#include "inverse_motion_planner/motions/discrete_dmp_motion.hpp"
#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

namespace rs = ::ranges;

SkillLearner::SkillLearner() : rclcpp::Node("skill_learner")
{
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

    auto cbk =
        [this](const LearnSkill::Request::ConstSharedPtr req,
               LearnSkill::Response::SharedPtr resp)
        {
            on_learn_skill_request(req, resp);
        };

    _learn_skill_server = create_service<LearnSkill>(
        "learn_skill",
        cbk,
        rmw_qos_profile_services_default,
        _server_cbk_group);

    _skill_db =
        std::make_unique<SkillDatabase>(SkillDatabase::default_database(), _logger);
}

void SkillLearner::on_learn_skill_request(
    const LearnSkill::Request::ConstSharedPtr& req,
    LearnSkill::Response::SharedPtr& resp)
{
    resp->success = false;

    if (req->registration_duration_secs <= 0.0)
    {
        logger().error("registration_duration_secs must be > 0");
        return;
    }

    if (req->num_basis == 0)
    {
        logger().error("num_basis must be > 0");
        return;
    }

    logger().info(
        "Starting listening for a demonstration of {} s",
        req->registration_duration_secs);

    constexpr auto sampling_period = std::chrono::milliseconds(5);
    const auto duration =
        std::chrono::duration<double>(req->registration_duration_secs);

    std::vector<Se3Pose> demo_copy;
    demo_copy.reserve(static_cast<std::size_t>(
        req->registration_duration_secs / 0.005 + 100.0));

    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < duration)
    {
        try
        {
            demo_copy.push_back(_system->current_ee_position());
        }
        catch (const std::exception& e)
        {
            logger().error("Failed to read current end-effector pose: {}", e.what());
            return;
        }
        catch (...)
        {
            logger().error("Failed to read current end-effector pose: unknown exception");
            return;
        }

        std::this_thread::sleep_for(sampling_period);
    }

    logger().info("Recorded {} entries in the demonstration", demo_copy.size());

    if (demo_copy.empty())
    {
        logger().error("No samples recorded");
        return;
    }

    const auto y0 = demo_copy.front();
    const auto g  = demo_copy.back();

    logger().info("y0: {}", mdv::ros2::describe(y0));
    logger().info("g: {}", mdv::ros2::describe(g));

    const auto first_it = std::find_if(
        demo_copy.begin(),
        demo_copy.end(),
        [&y0](const auto& y)
        {
            return (y.pos - y0.pos).norm() > 5e-3;
        });

    std::size_t first_sample_id = 0;
    if (first_it != demo_copy.end())
    {
        const auto idx = static_cast<std::size_t>(
            std::distance(demo_copy.begin(), first_it));
        first_sample_id = (idx > 100) ? (idx - 100) : 0;
    }

    const auto last_rit = std::find_if(
        demo_copy.rbegin(),
        demo_copy.rend(),
        [&g](const auto& y)
        {
            return (y.pos - g.pos).norm() > 1e-3;
        });

    std::size_t last_sample_id = demo_copy.size();
    if (last_rit != demo_copy.rend())
    {
        const auto reverse_idx = static_cast<std::size_t>(
            std::distance(demo_copy.rbegin(), last_rit));

        const auto idx_from_front = demo_copy.size() - reverse_idx;
        last_sample_id = std::min(idx_from_front + 100, demo_copy.size());
    }

    if (first_sample_id >= last_sample_id)
    {
        logger().error(
            "Invalid crop range: first_sample_id={} last_sample_id={}",
            first_sample_id,
            last_sample_id);
        return;
    }

    logger().info("First sample id: {}", first_sample_id);
    logger().info("Last sample id: {}", last_sample_id);

    std::vector<Se3Pose> demo;
    demo.reserve(last_sample_id - first_sample_id);

    for (std::size_t i = first_sample_id; i < last_sample_id; ++i)
        demo.push_back(demo_copy[i]);

    if (demo.size() < 20)
    {
        logger().error("Processed demonstration has only {} samples", demo.size());
        return;
    }

    for (std::size_t i = demo.size() - 1; i > 0; --i)
    {
        auto& curr = demo[i];
        auto& prev = demo[i - 1];

        if (prev.ori.coeffs().dot(curr.ori.coeffs()) < 0.0)
            prev.ori.coeffs() *= -1.0;
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

    resp->success = true;
    resp->initial_pose = mdv::ros2::to_pose_message(skill_data.initial_pose);
    resp->final_pose   = mdv::ros2::to_pose_message(skill_data.final_pose);
    resp->total_demonstration_time =
        std::chrono::duration<double>(sampling_period).count() *
        static_cast<double>(demo.size());

    logger().info(
        "Skill '{}' learned successfully with {} samples",
        req->skill_name,
        demo.size());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SkillLearner>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
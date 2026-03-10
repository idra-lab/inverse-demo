#ifndef INVERSE_MOTION_PLANNER_SKILL_LEARNER_HPP
#define INVERSE_MOTION_PLANNER_SKILL_LEARNER_HPP

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <inverse_msgs/srv/learn_skill.hpp>
#include <mdv/macros.hpp>
#include <mdv/utils/logging.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include "inverse_motion_planner/components/motion.hpp"
#include "inverse_motion_planner/components/skill_database.hpp"
#include "inverse_motion_planner/motion_planner.hpp"
#include "inverse_motion_planner/ros2/ros2_motion_parameters.hpp"
#include "inverse_motion_planner/ros2/ros2_robot_system.hpp"

class SkillLearner : public rclcpp::Node {
public:
    using Se3Pose   = MotionPlanner::Se3Pose;
    using Se3Framed = MotionPlanner::Se3Framed;
    using Vec6      = Eigen::Vector<double, 6>;

    SkillLearner();

    // clang-format off
    MDV_NODISCARD mdv::Logger&   logger() const noexcept { assert(_logger); return *_logger; }

    // clang-format on

    void gcomp_enable();
    void gcomp_disable();

    Vec6 read_stiffness_values() const;

    void set_stiffness_values(const Vec6& stiffness);

    mutable mdv::Logger::SharedPtr         _logger     = nullptr;
    std::unique_ptr<Ros2PlannerParameters> _parameters = nullptr;
    std::unique_ptr<Ros2RobotSystem>       _system     = nullptr;
    std::unique_ptr<SkillDatabase>         _skill_db   = nullptr;

    std::string _base_link;
    std::string _ee_link;

    rclcpp::CallbackGroup::SharedPtr _cli_cbk_group    = nullptr;
    rclcpp::CallbackGroup::SharedPtr _server_cbk_group = nullptr;
    rclcpp::CallbackGroup::SharedPtr _timer_cbk_group  = nullptr;

    using GetParameters                  = rcl_interfaces::srv::GetParameters;
    using ParamGetterClient              = rclcpp::Client<GetParameters>::SharedPtr;
    ParamGetterClient _params_getter_cli = nullptr;

    using SetParameters                  = rcl_interfaces::srv::SetParameters;
    using ParamSetterClient              = rclcpp::Client<SetParameters>::SharedPtr;
    ParamSetterClient _params_setter_cli = nullptr;

    using LearnSkill                     = inverse_msgs::srv::LearnSkill;
    using LearnSkillServer               = rclcpp::Service<LearnSkill>::SharedPtr;
    LearnSkillServer _learn_skill_server = nullptr;

    void on_learn_skill_request(
            const LearnSkill::Request::ConstSharedPtr& req,
            LearnSkill::Response::SharedPtr&           resp
    );

    std::string _db_file;

    Vec6 _stiffness_initial_values;
};

#endif  // INVERSE_MOTION_PLANNER_SKILL_LEARNER_HPP

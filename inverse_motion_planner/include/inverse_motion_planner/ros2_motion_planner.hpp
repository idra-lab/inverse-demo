#ifndef INVERSE_MOTION_PLANNER_ROS2_PLANNER_HPP
#define INVERSE_MOTION_PLANNER_ROS2_PLANNER_HPP

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mdv/macros.hpp>
#include <mdv/utils/logging.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <inverse_msgs/srv/move_relative.hpp>
#include <inverse_msgs/srv/point_to_point_motion.hpp>
#include <inverse_msgs/srv/reach_position.hpp>
#include <inverse_msgs/srv/hold_position.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "inverse_motion_planner/motion_planner.hpp"
#include "inverse_motion_planner/ros2/ros2_motion_parameters.hpp"
#include "inverse_motion_planner/ros2/ros2_robot_system.hpp"

class Ros2MotionPlanner : public rclcpp::Node {
public:
    using Se3Pose     = MotionPlanner::Se3Pose;
    using Se3Framed   = MotionPlanner::Se3Framed;

    Ros2MotionPlanner();

    // clang-format off
    MDV_NODISCARD mdv::Logger&   logger() const noexcept { assert(_logger); return *_logger; }
    MDV_NODISCARD MotionPlanner& planner() noexcept      { assert(_planner); return *_planner; }

    // clang-format on

private:
    mutable mdv::Logger::SharedPtr             _logger       = nullptr;
    std::unique_ptr<Ros2PlannerParameters>     _parameters   = nullptr;
    std::unique_ptr<Ros2RobotSystem>           _system       = nullptr;
    std::unique_ptr<MotionPlanner>             _planner      = nullptr;

    double _f_des           = 0.0;
    double _f_des_topic     = 0.0;
    bool   _use_topic_des_f = false;

    Motion::Callback activate_admittance_callback();
    Motion::Callback deactivate_admittance_callback();

    Motion::Callback activate_sander_callback();
    Motion::Callback deactivate_sander_callback();


    //  ____        _                   _ _
    // / ___| _   _| |__  ___  ___ _ __(_) |__   ___ _ __ ___
    // \___ \| | | | '_ \/ __|/ __| '__| | '_ \ / _ \ '__/ __|
    //  ___) | |_| | |_) \__ \ (__| |  | | |_) |  __/ |  \__ \
    // |____/ \__,_|_.__/|___/\___|_|  |_|_.__/ \___|_|  |___/
    //
    using Float64                 = std_msgs::msg::Float64;
    using Float64Subscription     = rclcpp::Subscription<Float64>::SharedPtr;
    Float64Subscription _fdes_sub = nullptr;

    //   ____ _ _            _
    //  / ___| (_) ___ _ __ | |_ ___
    // | |   | | |/ _ \ '_ \| __/ __|
    // | |___| | |  __/ | | | |_\__ \
    //  \____|_|_|\___|_| |_|\__|___/
    //
    rclcpp::CallbackGroup::SharedPtr _client_cbk_group = nullptr;

    //  ____                  _
    // / ___|  ___ _ ____   _(_) ___ ___  ___
    // \___ \ / _ \ '__\ \ / / |/ __/ _ \/ __|
    //  ___) |  __/ |   \ V /| | (_|  __/\__ \
    // |____/ \___|_|    \_/ |_|\___\___||___/
    //
    using SetBoolSrv    = std_srvs::srv::SetBool;
    using SetBoolServer = rclcpp::Service<SetBoolSrv>::SharedPtr;
    void on_broadcast_state_request(
            const SetBoolSrv::Request::ConstSharedPtr& request,
            SetBoolSrv::Response::SharedPtr&           response
    );
    SetBoolServer _broadcast_state_server = nullptr;

    using TriggerSrv    = std_srvs::srv::Trigger;
    using TriggerServer = rclcpp::Service<TriggerSrv>::SharedPtr;
    void on_safestop_request(
            const TriggerSrv::Request::ConstSharedPtr& request,
            TriggerSrv::Response::SharedPtr&           response
    );
    TriggerServer _safestop_server = nullptr;

    using ReachPositionSrv    = inverse_msgs::srv::ReachPosition;
    using ReachPositionServer = rclcpp::Service<ReachPositionSrv>::SharedPtr;
    void on_reachposition_request(
            const ReachPositionSrv::Request::ConstSharedPtr& request,
            ReachPositionSrv::Response::SharedPtr&           response
    );
    ReachPositionServer _reachposition_server = nullptr;

    using PointToPointMotionSrv    = inverse_msgs::srv::PointToPointMotion;
    using PointToPointMotionServer = rclcpp::Service<PointToPointMotionSrv>::SharedPtr;
    void on_ptp_motion_request(
            const PointToPointMotionSrv::Request::ConstSharedPtr& request,
            PointToPointMotionSrv::Response::SharedPtr&           response
    );
    PointToPointMotionServer _ptp_motion_server = nullptr;

    using HoldPositionSrv    = inverse_msgs::srv::HoldPosition;
    using HoldPositionServer = rclcpp::Service<HoldPositionSrv>::SharedPtr;
    void on_hold_position_request(
            const HoldPositionSrv::Request::ConstSharedPtr& request,
            HoldPositionSrv::Response::SharedPtr&           response
    );
    HoldPositionServer _hold_position_server = nullptr;


    // using RhytmicDmpSrv    = inverse_msgs::srv::RhytmicDmpMotion;
    // using RhytmicDmpServer = rclcpp::Service<RhytmicDmpSrv>::SharedPtr;
    // void on_rhytmic_dmp_request(
    //         const RhytmicDmpSrv::Request::ConstSharedPtr& request,
    //         RhytmicDmpSrv::Response::SharedPtr&           response
    // );
    // RhytmicDmpServer _rhytmic_dmp_server = nullptr;

    using MoveRelativeSrv    = inverse_msgs::srv::MoveRelative;
    using MoveRelativeServer = rclcpp::Service<MoveRelativeSrv>::SharedPtr;
    void on_move_relative_request(
            const MoveRelativeSrv::Request::ConstSharedPtr& request,
            MoveRelativeSrv::Response::SharedPtr&           response
    );
    MoveRelativeServer _move_relative_server = nullptr;


    //  ____        _     _ _     _
    // |  _ \ _   _| |__ | (_)___| |__   ___ _ __ ___
    // | |_) | | | | '_ \| | / __| '_ \ / _ \ '__/ __|
    // |  __/| |_| | |_) | | \__ \ | | |  __/ |  \__ \
    // |_|    \__,_|_.__/|_|_|___/_| |_|\___|_|  |___/
    //
    using Timer = rclcpp::TimerBase::SharedPtr;

    using PoseStamped   = geometry_msgs::msg::PoseStamped;
    using PosePublisher = rclcpp::Publisher<PoseStamped>::SharedPtr;
    rclcpp::CallbackGroup::SharedPtr _reference_group     = nullptr;
    PosePublisher                    _reference_publisher = nullptr;
    Timer                            _reference_timer     = nullptr;

    void setup_reference_publisher(const std::string& link_name);
    void activate_reference_broadcasting();
    void deactivate_reference_broadcasting();
};

#endif  // INVERSE_MOTION_PLANNER_ROS2_PLANNER_HPP

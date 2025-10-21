#include "inverse_motion_planner/ros2_motion_planner.hpp"

#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/assert>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mdv/ros2/conversions.hpp>
#include <mdv/ros2/logger.hpp>
#include <mdv/utils/logging.hpp>
#include <range/v3/all.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "inverse_motion_planner/components/interpolation.hpp"
#include "inverse_motion_planner/components/path_profiler.hpp"
#include "inverse_motion_planner/components/skill_database.hpp"
#include "inverse_motion_planner/motions/discrete_dmp_motion.hpp"
#include "inverse_motion_planner/motions/hold_position.hpp"
#include "inverse_motion_planner/motions/raw_trajectory.hpp"
#include "inverse_motion_planner/motions/rhytmic_dmp_motion.hpp"
#include "inverse_motion_planner/ros2/ros2_motion_parameters.hpp"
#include "inverse_motion_planner/ros2/ros2_robot_system.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

Ros2MotionPlanner::Ros2MotionPlanner() : rclcpp::Node("motion_planner") {
    _logger = std::make_shared<mdv::ros2::RosLogger>(get_logger());

    Ensures(_logger);

    const bool debug_prints = declare_parameter("debug_prints", false);
    logger().info("debug_prints = {}", debug_prints);
    if (debug_prints) logger().set_log_level(mdv::Logger::LogLevel::Debug);

    const bool debug_lib = declare_parameter("debug_lib", false);
    logger().info("debug_lib = {}", debug_lib);
    if (debug_lib) mdv::set_default_logger(_logger);

    const std::string out_topic = declare_parameter("frame_topic_name", "desired_pose");
    logger().debug("'frama_topic_name' = {}", out_topic);

    logger().debug("Initialising Ros2PlannerParameters");
    _parameters = std::make_unique<Ros2PlannerParameters>(_logger, this);
    logger().debug("Initialising Ros2RobotSystem");
    _system = std::make_unique<Ros2RobotSystem>(_logger, this, _parameters.get());
    Ensures(_parameters);
    Ensures(_system);

    logger().debug("Initialising MotionPlanner");
    _planner =
            std::make_unique<MotionPlanner>(_logger, _system.get(), _parameters.get());
    Ensures(_planner);

    logger().debug("Creating ImpedanceToAdmittance");
    const std::string base_link = planner().parameters().get_base_link();
    const std::string ee_link   = planner().parameters().get_ee_link();
    const std::string ft_link   = declare_parameter("ft_link", ee_link);

    //  ____        _                   _ _
    // / ___| _   _| |__  ___  ___ _ __(_) |__   ___ _ __ ___
    // \___ \| | | | '_ \/ __|/ __| '__| | '_ \ / _ \ '__/ __|
    //  ___) | |_| | |_) \__ \ (__| |  | | |_) |  __/ |  \__ \
    // |____/ \__,_|_.__/|___/\___|_|  |_|_.__/ \___|_|  |___/
    //
    const std::string fdes_topic = fmt::format(
            "/cartesian/{}/desired_force", planner().parameters().get_ee_link()
    );
    _fdes_sub = create_subscription<Float64>(
            fdes_topic,
            rclcpp::QoS(1).keep_last(1).durability_volatile(),
            [this](const Float64::ConstSharedPtr& msg) { _f_des_topic = msg->data; }
    );
    assert(_fdes_sub);
    logger().info(
            "Listening for desired force on topic {}", _fdes_sub->get_topic_name()
    );

    //   ____ _ _            _
    //  / ___| (_) ___ _ __ | |_ ___
    // | |   | | |/ _ \ '_ \| __/ __|
    // | |___| | |  __/ | | | |_\__ \
    //  \____|_|_|\___|_| |_|\__|___/
    //
    _client_cbk_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    //  ____                  _
    // / ___|  ___ _ ____   _(_) ___ ___  ___
    // \___ \ / _ \ '__\ \ / / |/ __/ _ \/ __|
    //  ___) |  __/ |   \ V /| | (_|  __/\__ \
    // |____/ \___|_|    \_/ |_|\___\___||___/
    //
    auto broadcast_state_lambda = [this](const SetBoolSrv::Request::ConstSharedPtr req,
                                         SetBoolSrv::Response::SharedPtr res) {
        on_broadcast_state_request(req, res);
    };
    _broadcast_state_server =
            create_service<SetBoolSrv>("set_broadcast_state", broadcast_state_lambda);
    assert(_broadcast_state_server);
    logger().info(
            "Exposed service on topic {} with type std_srvs::srv::SetBool",
            _broadcast_state_server->get_service_name()
    );

    auto safestop_lambda = [this](const TriggerSrv::Request::ConstSharedPtr req,
                                  TriggerSrv::Response::SharedPtr           res) {
        on_safestop_request(req, res);
    };
    _safestop_server = create_service<TriggerSrv>("safe_stop", safestop_lambda);
    assert(_safestop_server);
    logger().info(
            "Exposed service on topic {} with type std_srvs::srv::Trigger",
            _safestop_server->get_service_name()
    );

    auto reachposition_lambda =
            [this](const ReachPositionSrv::Request::ConstSharedPtr req,
                   ReachPositionSrv::Response::SharedPtr           res) {
                on_reachposition_request(req, res);
            };
    _reachposition_server =
            create_service<ReachPositionSrv>("reach_position", reachposition_lambda);
    assert(_reachposition_server);
    logger().info(
            "Exposed service on topic {} with type inverse_msgs::srv::ReachPosition",
            _reachposition_server->get_service_name()
    );

    auto ptp_motion_lambda =
            [this](const PointToPointMotionSrv::Request::ConstSharedPtr req,
                   PointToPointMotionSrv::Response::SharedPtr           res) {
                on_ptp_motion_request(req, res);
            };
    _ptp_motion_server = create_service<PointToPointMotionSrv>(
            "execute_ptp_motion", ptp_motion_lambda
    );
    assert(_ptp_motion_server);
    logger().info(
            "Exposed service on topic {} with type "
            "inverse_msgs::srv::PointToPointMotion",
            _ptp_motion_server->get_service_name()
    );

    auto hold_position_lambda =
            [this](const HoldPositionSrv::Request::ConstSharedPtr req,
                   HoldPositionSrv::Response::SharedPtr           res) {
                on_hold_position_request(req, res);
            };
    _hold_position_server =
            create_service<HoldPositionSrv>("hold_position", hold_position_lambda);
    assert(_hold_position_server);
    logger().info(
            "Exposed service on topic {} with type "
            "inverse_msgs::srv::HoldPosition",
            _hold_position_server->get_service_name()
    );

    auto move_relative_lambda =
            [this](const MoveRelativeSrv::Request::ConstSharedPtr req,
                   MoveRelativeSrv::Response::SharedPtr           res) {
                on_move_relative_request(req, res);
            };
    _move_relative_server =
            create_service<MoveRelativeSrv>("move_relative", move_relative_lambda);
    assert(_move_relative_server);
    logger().info(
            "Exposed service on topic {} with type "
            "inverse_msgs::srv::MoveRelative",
            _move_relative_server->get_service_name()
    );

    //  ____        _     _ _     _
    // |  _ \ _   _| |__ | (_)___| |__   ___ _ __ ___
    // | |_) | | | | '_ \| | / __| '_ \ / _ \ '__/ __|
    // |  __/| |_| | |_) | | \__ \ | | |  __/ |  \__ \
    // |_|    \__,_|_.__/|_|_|___/_| |_|\___|_|  |___/
    //
    _reference_group =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    setup_reference_publisher(planner().parameters().get_ee_link());

    using std::filesystem::path;
    const path default_db =
            path(ament_index_cpp::get_package_share_directory("inverse_motion_planner"))
            / "motion.json";
    _db_file  = declare_parameter("skill_database", default_db.string());
    _skill_db = std::make_unique<SkillDatabase>(_db_file, _logger);
};

//   ____      _ _ _                _
//  / ___|__ _| | | |__   __ _  ___| | _____
// | |   / _` | | | '_ \ / _` |/ __| |/ / __|
// | |__| (_| | | | |_) | (_| | (__|   <\__ \
//  \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/
//
void
Ros2MotionPlanner::on_broadcast_state_request(
        const SetBoolSrv::Request::ConstSharedPtr& request,
        SetBoolSrv::Response::SharedPtr&           response
) {
    response->success = false;
    if (request->data) {
        logger().info("Received request to start reference broadcasting");
        activate_reference_broadcasting();
    } else {
        logger().info("Received request to stop reference broadcasting");
        deactivate_reference_broadcasting();
    }
    response->success = true;
}

void
Ros2MotionPlanner::on_safestop_request(
        const TriggerSrv::Request::ConstSharedPtr& /*request*/,
        TriggerSrv::Response::SharedPtr& response
) {
    response->success = false;
    logger().info("Received request to stop all motions");
    planner().motion_queue().clear();
    planner().motion_queue().stop_motion();
    response->success = true;
}

void
Ros2MotionPlanner::on_reachposition_request(
        const ReachPositionSrv::Request::ConstSharedPtr& request,
        ReachPositionSrv::Response::SharedPtr&           response
) {
    response->success = false;

    const Se3Framed framed_goal = mdv::ros2::get_pose(request->desired_pos);
    logger().info("Received request to reach {}", mdv::ros2::describe(framed_goal));
    const Se3Pose goal = planner().display_in_base(framed_goal);

    DmpParameters params;
    params.max_vel = request->max_vel;

    if (request->immediate_execution) {
        logger().warn("Position shall be reached immediately!");

        const auto curr_ref = planner().current_motion().current_reference_pose();
        auto       new_plan = DiscreteDmpMotion::linear_interpolation(
                curr_ref, goal, planner().parameters().get_dt(), params
        );

        // TODO:
        planner().motion_queue().clear();
        planner().motion_queue().stop_motion();
        planner().motion_queue().append_motion(std::move(new_plan));


    } else {
        auto new_plan = DiscreteDmpMotion::linear_interpolation(
                planner().motion_queue().get_queue_final_pose(),
                goal,
                planner().parameters().get_dt(),
                params
        );
        planner().motion_queue().append_motion(std::move(new_plan));
    }
    response->success = true;
}

void
Ros2MotionPlanner::on_ptp_motion_request(
        const PointToPointMotionSrv::Request::ConstSharedPtr& request,
        PointToPointMotionSrv::Response::SharedPtr&           response
) {
    response->success = false;

    const Se3Framed y0 = mdv::ros2::get_pose(request->y0);
    const Se3Framed g  = mdv::ros2::get_pose(request->g);
    logger().info(
            "Received request for a point-to-point motion from {} to {}",
            mdv::ros2::describe(y0),
            mdv::ros2::describe(g)
    );

    DmpParameters params;
    params.max_vel = request->max_vel;

    if (request->plan_y0_motion) {
        logger().info("Adding motion to reach the initial configuration");
        auto plan = DiscreteDmpMotion::linear_interpolation(
                planner().motion_queue().get_queue_final_pose(),
                planner().display_in_base(y0),
                planner().parameters().get_dt(),
                params
        );
        planner().motion_queue().append_motion(std::move(plan));
    }

    auto plan = DiscreteDmpMotion::linear_interpolation(
            planner().display_in_base(y0),
            planner().display_in_base(g),
            planner().parameters().get_dt(),
            params
    );
    planner().motion_queue().append_motion(std::move(plan));

    response->success = true;
}

void
Ros2MotionPlanner::on_hold_position_request(
        const HoldPositionSrv::Request::ConstSharedPtr& request,
        HoldPositionSrv::Response::SharedPtr&           response
) {
    response->success = false;

    auto plan = std::make_unique<HoldPositionMotion>(
            planner().motion_queue().get_queue_final_pose(),
            request->wait_sec,
            planner().parameters().get_dt(),
            DmpParameters(),
            _logger
    );
    planner().motion_queue().append_motion(std::move(plan));

    response->success = true;
}

void
Ros2MotionPlanner::on_move_relative_request(
        const MoveRelativeSrv::Request::ConstSharedPtr& req,
        MoveRelativeSrv::Response::SharedPtr&           resp
) {
    resp->success = false;

    const std::string     base_link    = planner().parameters().get_base_link();
    const std::string     ee_link      = planner().parameters().get_ee_link();
    const Eigen::Affine3d in_transform = mdv::ros2::get_transform(req->relative_motion);

    const Se3Framed y0{planner().motion_queue().get_queue_final_pose(), base_link};

    // tf_rf_ee: transformation from ee to the ref. frame. where the input transform is
    // defined
    Eigen::Affine3d tf_rf_ee = Eigen::Affine3d::Identity();
    // TODO: introduce frame link argument
    // if (req->frame_link != "")
    //     tf_rf_ee = planner().system().get_transformation(ee_link, req->frame_link);

    // compute the transform in the end-effector ling
    const Eigen::Affine3d transform_in_ee = tf_rf_ee * in_transform;

    const Se3Framed g{
            Se3Pose::from_affine(y0.to_affine() * transform_in_ee), base_link};

    DmpParameters params;
    params.max_vel = req->max_vel;
    auto plan      = DiscreteDmpMotion::linear_interpolation(
            y0, g, planner().parameters().get_dt(), params
    );

    planner().motion_queue().append_motion(std::move(plan));
    resp->success = true;
}

//  ____        _     _ _     _
// |  _ \ _   _| |__ | (_)___| |__   ___ _ __
// | |_) | | | | '_ \| | / __| '_ \ / _ \ '__|
// |  __/| |_| | |_) | | \__ \ | | |  __/ |
// |_|    \__,_|_.__/|_|_|___/_| |_|\___|_|
//  _____                 _   _
// |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
// | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
// |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
// |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//
void
Ros2MotionPlanner::setup_reference_publisher(const std::string& link_name) {
    const std::string topic_name = get_parameter("frame_topic_name").as_string();
    _reference_publisher         = create_publisher<PoseStamped>(
            topic_name, rclcpp::QoS(1).durability_volatile()
    );
    assert(_reference_publisher);
    logger().info(
            "Publishing reference pose on topic {}",
            _reference_publisher->get_topic_name()
    );
}

void
Ros2MotionPlanner::activate_reference_broadcasting() {
    const double dt = planner().parameters().get_dt();
    logger().info("Starting broadcasting reference with period {}ms", 1000 * dt);

    logger().info(
            "Current reference: {}",
            mdv::ros2::describe(planner().system().current_ee_position())
    );
    planner().current_motion().set_current_reference(
            planner().system().current_ee_position(), Motion::Twist()
    );

    auto step_and_publish_reference = [this]() {
        if (_use_topic_des_f) _f_des = _f_des_topic;

        auto       ref        = Se3Pose(planner().step());
        const auto ref_framed = Se3Framed(ref, planner().parameters().get_base_link());
        _reference_publisher->publish(mdv::ros2::to_pose_message(ref_framed));
    };

    assert(_reference_group);
    _reference_timer = create_wall_timer(
            std::chrono::duration<double>(dt),
            step_and_publish_reference,
            _reference_group
    );
}

void
Ros2MotionPlanner::deactivate_reference_broadcasting() {
    logger().info("Stopped reference broadcasting");
    _reference_timer.reset();
    planner().motion_queue().stop_motion();
}

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ros2MotionPlanner>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

#ifndef INVERSE_MOTION_PLANNER_GRIPPER_MOTION_HPP
#define INVERSE_MOTION_PLANNER_GRIPPER_MOTION_HPP

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "inverse_motion_planner/components/motion.hpp"

// A pose-holding primitive. Only a successful action result completes it;
// failures hold the queue until the operator stops/replaces the motion.
class GripperMotion : public Motion {
public:
    using Action = control_msgs::action::GripperCommand;
    using Client = rclcpp_action::Client<Action>;

    GripperMotion(
            const Se3Pose&    pose,
            Client::SharedPtr client,
            double            position,
            double            max_effort,
            double            timeout_sec,
            bool              allow_stalled,
            std::string       name,
            rclcpp::Logger    logger
    );
    ~GripperMotion() override;

    void set_current_reference(const Se3Pose& pose, const Twist&) override;

    Se3Pose
    final_pose() const override {
        return _pose;
    }

    Se3Pose
    initial_pose() const override {
        return _pose;
    }

    Se3Pose
    current_reference_pose() const override {
        return _pose;
    }

    Twist
    current_reference_twist() const override {
        return {};
    }

    void        step() override;
    bool        is_completed() const override;
    std::string describe() const override;

private:
    struct State;
    std::shared_ptr<State> _state;
    Se3Pose                _pose;
};

class PickMotion final : public GripperMotion {
public:
    PickMotion(
            const Se3Pose&    pose,
            Client::SharedPtr client,
            double            closed_position,
            double            max_effort,
            double            timeout_sec,
            bool              allow_stalled,
            rclcpp::Logger    logger
    ) :
            GripperMotion(
                    pose,
                    std::move(client),
                    closed_position,
                    max_effort,
                    timeout_sec,
                    allow_stalled,
                    "Pick (close gripper)",
                    logger
            ) {}
};

class PlaceMotion final : public GripperMotion {
public:
    PlaceMotion(
            const Se3Pose&    pose,
            Client::SharedPtr client,
            double            open_position,
            double            max_effort,
            double            timeout_sec,
            rclcpp::Logger    logger
    ) :
            GripperMotion(
                    pose,
                    std::move(client),
                    open_position,
                    max_effort,
                    timeout_sec,
                    false,
                    "Place (open gripper)",
                    logger
            ) {}
};

#endif

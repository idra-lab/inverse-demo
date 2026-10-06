#include "inverse_motion_planner/motions/gripper_motion.hpp"

#include <chrono>
#include <mutex>
#include <stdexcept>

struct GripperMotion::State {
    using time_point = std::chrono::steady_clock::time_point;
    using ClientGoal = rclcpp_action::ClientGoalHandle<Action>::SharedPtr;

    Client::SharedPtr client;
    rclcpp::Logger    logger;
    std::string       name;
    double            position;
    double            max_effort;
    double            timeout_sec;
    bool              allow_stalled;
    std::mutex        mutex;
    bool              started   = false;
    bool              succeeded = false;
    bool              stopped   = false;
    time_point        start_time;
    ClientGoal        goal;

    // Called with mutex held. Also handles goals accepted after a timeout/stop.
    void
    cancel() {
        if (!goal) return;
        try {
            client->async_cancel_goal(goal);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "%s: cancellation failed: %s", name.c_str(), e.what());
        }
    }

    void
    fail(const std::string& reason) {
        stopped = true;
        RCLCPP_ERROR(
                logger,
                "%s: %s; holding pose and queue until safe_stop",
                name.c_str(),
                reason.c_str()
        );
        cancel();
    }

    State(Client::SharedPtr client_,
          rclcpp::Logger    logger_,
          std::string       name_,
          double            position_,
          double            max_effort_,
          double            timeout_sec_,
          bool              allow_stalled_) :
            client(std::move(client_)),
            logger(logger_),
            name(std::move(name_)),
            position(position_),
            max_effort(max_effort_),
            timeout_sec(timeout_sec_),
            allow_stalled(allow_stalled_) {}
};

GripperMotion::GripperMotion(
        const Se3Pose&    pose,
        Client::SharedPtr client,
        double            position,
        double            max_effort,
        double            timeout_sec,
        bool              allow_stalled,
        std::string       name,
        rclcpp::Logger    logger
) :
        _state(std::make_shared<State>(
                std::move(client),
                logger,
                std::move(name),
                position,
                max_effort,
                timeout_sec,
                allow_stalled
        )),
        _pose(pose) {
    if (!_state->client || timeout_sec <= 0.0) {
        throw std::invalid_argument(
                "Gripper motion requires a client and positive timeout"
        );
    }
}

GripperMotion::~GripperMotion() {
    std::lock_guard<std::mutex> lock(_state->mutex);
    if (!_state->succeeded && !_state->stopped) {
        _state->stopped = true;
        _state->cancel();
    }
}

void
GripperMotion::set_current_reference(const Se3Pose& pose, const Twist&) {
    _pose = pose;
}

bool
GripperMotion::is_completed() const {
    std::lock_guard<std::mutex> lock(_state->mutex);
    return _state->succeeded;
}

std::string
GripperMotion::describe() const {
    return _state->name;
}

void
GripperMotion::step() {
    // Callbacks own only shared state, never the Motion: late results/acceptance
    // are safe even when safe_stop destroys the active primitive.
    auto                        state = _state;
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->stopped || state->succeeded) return;
    const auto now = std::chrono::steady_clock::now();
    if (state->started) {
        if (std::chrono::duration<double>(now - state->start_time).count()
            >= state->timeout_sec) {
            state->fail("gripper action timed out");
        }
        return;
    }

    state->started    = true;
    state->start_time = now;
    if (!state->client->action_server_is_ready()) {
        state->fail("gripper action server unavailable");
        return;
    }

    Action::Goal goal;
    goal.command.position   = state->position;
    goal.command.max_effort = state->max_effort;
    Client::SendGoalOptions options;
    options.goal_response_callback = [state](auto handle) {
        std::lock_guard<std::mutex> guard(state->mutex);
        state->goal = handle;
        if (state->stopped) {
            state->cancel();
        } else if (!handle) {
            state->fail("gripper goal rejected");
        }
    };
    options.result_callback = [state](const auto& result) {
        std::lock_guard<std::mutex> guard(state->mutex);
        state->goal.reset();
        if (state->stopped) return;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result
            && (result.result->reached_goal
                || (state->allow_stalled && result.result->stalled))) {
            state->succeeded = true;
            RCLCPP_INFO(state->logger, "%s completed", state->name.c_str());
        } else {
            state->fail("gripper did not complete successfully");
        }
    };
    try {
        state->client->async_send_goal(goal, options);
    } catch (const std::exception& e) { state->fail(e.what()); }
}

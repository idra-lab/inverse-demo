#include "inverse_motion_planner/components/motion_queue.hpp"

#include <gsl/assert>
#include <mutex>
#include <utility>

#include <mdv/ros2/se3.hpp>

class KeepPosition : public Motion {
public:
    KeepPosition(Se3Pose pose) : _pose(std::move(pose)) {}

    ~KeepPosition() override = default;

    void
    set_current_reference(const Se3Pose& pose_reference, const Twist& /*vel_reference*/)
            override {
        _pose = pose_reference;
    };

    MDV_NODISCARD Se3Pose
    final_pose() const final {
        return _pose;
    }

    MDV_NODISCARD Se3Pose
    initial_pose() const final {
        return _pose;
    }

    MDV_NODISCARD Se3Pose
    current_reference_pose() const final {
        return _pose;
    }

    MDV_NODISCARD Twist
    current_reference_twist() const final {
        return {};
    }

    void step() final{};

    MDV_NODISCARD bool
    is_completed() const final {
        return true;
    };

    MDV_NODISCARD std::string
                  describe() const final {
        return fmt::format("Keeping pose {}", mdv::ros2::describe(_pose));
    }

private:
    Se3Pose _pose;
};

MotionQueue::MotionQueue(const Se3Pose& initial_pose, mdv::Logger::SharedPtr logger) :
        _logger(std::move(logger)) {
    Expects(_logger);
    KeepPosition mot(initial_pose);
    _curr_motion = std::make_unique<KeepPosition>(initial_pose);

    assert(_curr_motion);
    assert(_motion_queue.empty());
}

void
MotionQueue::append_motion(Motion::UniquePtr&& motion) {
    std::lock_guard<std::mutex> queue_mutex_lock(_queue_mutex);
    logger().info("Appending new motion to the queue: {}", motion->describe());
    _motion_queue.emplace(std::move(motion));
}

void
MotionQueue::clear() {
    logger().warn("Deleting all planned motion in the queue!");
    std::lock_guard<std::mutex> queue_mutex_lock(_queue_mutex);
    logger().warn("Number of motion enqueued: {}", _motion_queue.size());
    while (!_motion_queue.empty()) _motion_queue.pop();
    logger().warn("Motion plan queu cleared!");
}

void
MotionQueue::stop_motion() {
    logger().debug("Called MotionQueue::stop_motion()");
    auto new_plan =
            std::make_unique<KeepPosition>(current_motion().current_reference_pose());
    _curr_motion = std::move(new_plan);
}

MotionQueue::Se3Pose
MotionQueue::get_queue_final_pose() const {
    std::lock_guard<std::mutex> queue_mutex_lock(_queue_mutex);
    if (_motion_queue.empty()) return _curr_motion->final_pose();
    return _motion_queue.back()->final_pose();
}

std::size_t
MotionQueue::length() {
    std::lock_guard<std::mutex> queue_mutex_lock(_queue_mutex);
    return _motion_queue.size();
}

void
MotionQueue::try_step_motion() {
    if (_queue_mutex.try_lock()) {
        pop_from_queue();
        _queue_mutex.unlock();
    }
}

void
MotionQueue::pop_from_queue() {
    if (_motion_queue.empty()) return;

    _curr_motion->call_motion_completion_hooks();

    // Retrieve new plan
    auto new_plan = std::move(_motion_queue.front());
    _motion_queue.pop();

    // Ensure reference continuity
    new_plan->set_current_reference(
            _curr_motion->current_reference_pose(),
            _curr_motion->current_reference_twist()
    );
    _curr_motion = std::move(new_plan);
    // TODO: check consistency with plan requirements?

    logger().info("Switched to new motion plan: {}", _curr_motion->describe());
    _curr_motion->call_motion_start_hooks();
}

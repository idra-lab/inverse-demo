#ifndef INVERSE_MOTION_PLANNER_MOTION_QUEUE_HPP
#define INVERSE_MOTION_PLANNER_MOTION_QUEUE_HPP

#include <mutex>
#include <queue>

#include <mdv/utils/logging.hpp>

#include "inverse_motion_planner/components/motion.hpp"

class MotionQueue {
public:
    using Se3Pose = Motion::Se3Pose;

    MotionQueue(
            const Se3Pose&         initial_pose,
            mdv::Logger::SharedPtr logger = mdv::get_default_logger()
    );

    /**
     * @brief Puts the provided motion at the end of the queue
     *
     */
    void append_motion(Motion::UniquePtr&& motion);

    /**
     * @brief Clears the current motion queue
     *
     */
    void clear();

    /**
     * @brief Stops the ongoing motion
     *
     */
    void stop_motion();

    /**
     * @brief Get the final pose of the last motion in the queue
     *
     */
    MDV_NODISCARD Se3Pose get_queue_final_pose() const;

    /**
     * @std Numers of motion plan that are in the queue awaiting for execution
     */
    MDV_NODISCARD std::size_t length();

    /**
     * @brief Tries to acquire the mutex, and if it succeed it replaces ongoing motion
     * with new one.
     *
     */
    void try_step_motion();

    /**
     * @brief Tryies to pop the front element from the motion queue in order to execute
     * the motion. If the queue is empty, it does nothing.
     *
     * This function implicitly assumes that the queue mutex is locked, and does not
     * automatically release it at the end of the call. The management of the mutex is
     * up to the caller.
     */
    void pop_from_queue();

    // clang-format off
    MDV_NODISCARD Motion&      current_motion() noexcept { assert(_curr_motion.get());  return *_curr_motion.get(); }
    MDV_NODISCARD mdv::Logger& logger() const noexcept   { assert(_logger); return *_logger; }

    // clang-format on


private:
    mutable mdv::Logger::SharedPtr _logger      = nullptr;
    Motion::UniquePtr              _curr_motion = nullptr;
    std::queue<Motion::UniquePtr>  _motion_queue;
    mutable std::mutex             _queue_mutex;
};


#endif  // INVERSE_MOTION_PLANNER_MOTION_QUEUE_HPP

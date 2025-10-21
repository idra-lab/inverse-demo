#ifndef INVERSE_MOTION_PLANNER_MOTION_HPP
#define INVERSE_MOTION_PLANNER_MOTION_HPP

#include <functional>
#include <memory>

#include <mdv/macros.hpp>
#include <mdv/riemann_geometry/se3.hpp>

class Motion {
public:
    using Ptr       = Motion*;
    using UniquePtr = std::unique_ptr<Motion>;
    using SharedPtr = std::shared_ptr<Motion>;

    using Se3Pose = mdv::riemann::SE3::Point;
    using Twist   = mdv::riemann::SE3::TangentVector;

    using Callback       = std::function<void()>;
    using LifecycleHooks = std::vector<Callback>;

    Motion()                         = default;
    virtual ~Motion()                = default;
    Motion(const Motion&)            = delete;
    Motion(Motion&&)                 = delete;
    Motion& operator=(const Motion&) = delete;
    Motion& operator=(Motion&&)      = delete;

    virtual void set_current_reference(
            const Se3Pose& pose_reference, const Twist& vel_reference
    ) = 0;

    MDV_NODISCARD virtual Se3Pose final_pose() const = 0;

    MDV_NODISCARD virtual Se3Pose initial_pose() const = 0;

    MDV_NODISCARD virtual Se3Pose current_reference_pose() const = 0;

    MDV_NODISCARD virtual Twist current_reference_twist() const = 0;

    virtual void step() = 0;

    MDV_NODISCARD virtual bool is_completed() const = 0;

    MDV_NODISCARD virtual std::string describe() const = 0;

    /**
     * @brief Adds a callback that is called when the motion starts getting
     * executed.
     *
     */
    void add_motion_start_hook(Callback&& cbk);

    /**
     * @brief Adds a callback that is called when the motion is completed.
     *
     */
    void add_motion_completion_hook(Callback&& cbk);

    void call_motion_start_hooks();

    void call_motion_completion_hooks();

private:
    LifecycleHooks _on_motion_start_hooks;
    LifecycleHooks _on_motion_completion_hooks;
};
#endif  // INVERSE_MOTION_PLANNER_MOTION_HPP

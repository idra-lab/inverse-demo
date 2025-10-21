#ifndef INVERSE_MOTION_PLANNER_HOLD_POSITION_MOTION_HPP
#define INVERSE_MOTION_PLANNER_HOLD_POSITION_MOTION_HPP

#include <mdv/utils/logging.hpp>

#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

class HoldPositionMotion : public DmpMotionInterface {
public:
    HoldPositionMotion(
            const Se3Pose&         desired_hold_pose,
            double                 hold_time,
            double                 dt,
            const DmpParameters&   parameters = DmpParameters(),
            mdv::Logger::SharedPtr logger     = mdv::get_default_logger()
    );

    MDV_NODISCARD Se3Pose initial_pose() const final;

    MDV_NODISCARD bool is_completed() const override;

    MDV_NODISCARD std::string describe() const override;

    void step() final;

private:
    double _hold_time    = 0.0;
    double _elapsed_time = 0.0;
};


#endif  // INVERSE_MOTION_PLANNER_HOLD_POSITION_MOTION_HPP

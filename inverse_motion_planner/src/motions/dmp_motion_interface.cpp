#include "inverse_motion_planner/motions/dmp_motion_interface.hpp"

DmpMotionInterface::DmpMotionInterface(
        const DmpParameters& parameters, const double dt, mdv::Logger::SharedPtr logger
) :

        _dmp(parameters.n_basis,
             std::make_shared<Se3DmpBase::TransformationSystem>(
                     parameters.alpha, parameters.beta
             ),
             std::make_shared<Se3DmpBase::CoordinateSystem>(parameters.gamma),
             std::move(logger)) {
    _dmp.set_sampling_period(dt);
}

void
DmpMotionInterface::set_current_reference(
        const Se3Pose& new_reference, const Twist& vel_reference
) {
    _dmp.curr_state.y()  = new_reference;
    _dmp.curr_state.yd() = vel_reference;

    if (initial_pose().ori.coeffs().dot(new_reference.ori.coeffs()) < 0.0)
        _dmp.curr_state.y().ori.coeffs() *= -1.0;
}

DmpMotionInterface::Se3Pose
DmpMotionInterface::final_pose() const {
    return _dmp.goal_state.y();
}

DmpMotionInterface::Se3Pose
DmpMotionInterface::current_reference_pose() const {
    return _dmp.curr_state.y();
}

DmpMotionInterface::Twist
DmpMotionInterface::current_reference_twist() const {
    return _dmp.curr_state.yd();
}

void
DmpMotionInterface::step() {
    _dmp.step();
}

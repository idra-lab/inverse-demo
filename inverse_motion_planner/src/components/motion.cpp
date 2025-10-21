#include "inverse_motion_planner/components/motion.hpp"

void
Motion::add_motion_start_hook(Callback&& cbk) {
    _on_motion_start_hooks.emplace_back(std::move(cbk));
}

void
Motion::add_motion_completion_hook(Callback&& cbk) {
    _on_motion_completion_hooks.emplace_back(std::move(cbk));
}

void
Motion::call_motion_start_hooks() {
    for (const auto& hook : _on_motion_start_hooks) hook();
}

void
Motion::call_motion_completion_hooks() {
    for (const auto& hook : _on_motion_completion_hooks) hook();
}

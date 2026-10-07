// Copyright (c) 2026 ASL
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

#include "inverse_msgs/msg/cartesian_trajectory_point.hpp"
#include "velocity_controller/robot_kinematics.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Cartesian motion control, commanded through the `velocity` command
 * interface (not `position`/resolved-rate-integrated-to-`position` like the
 * two controllers this replaces):
 *
 *   xdot_cmd = -P_gain * pose_error + target_velocity
 *   qdot_cmd = J^+(q) * xdot_cmd
 *
 * clamped to the URDF's velocity limits and scaled by `speed_scaling`.
 * `P_gain` (default `1.0`) is a starting point, not a tuned value —
 * live-tunable with `ros2 param set` while running, same as
 * `jacobian_damping_max`/`singularity_threshold`. This is a plain
 * proportional law; replace it in `update()` (see the `// TODO` marker
 * there) if something more involved is needed.
 *
 * Takes a single `inverse_msgs/msg/CartesianTrajectoryPoint` on
 * `~/commands` (`pose`, `velocity`, `acceleration` -- `base_link` frame --
 * and `time_from_start`; `acceleration` is unused by this controller, it
 * exists so `CartesianImpedanceController` can share the same message type)
 * instead of the three independent topics (`~/target_pose`,
 * `~/target_velocity`, and previously nothing for acceleration) an earlier
 * version used: a subscriber reading `pose`/`velocity` out of two
 * independently-timestamped `RealtimeBuffer`s has no way to know whether the
 * values it just read actually belong together, and a raw-to-raw
 * reference-shaping segment (see below) specifically needs a
 * pose/velocity/time triple that's known to be mutually consistent. One
 * message makes that structural instead of best-effort -- see
 * `msg/CartesianTrajectoryPoint.msg`'s doc comment. Defaults to the pose
 * measured at activation (FK of the initial joint state) with zero
 * velocity, so the arm doesn't move until a new command is published.
 *
 * The read-only `command_mode` parameter (`"position"`, `"velocity"`, or
 * `"position_velocity"`, default `"position"`) determines which of `pose`/
 * `velocity` drives `qdot_cmd` — mirrors `JointPositionController`'s
 * `command_mode` exactly, since both now take a single synchronized message
 * with the same three modes:
 *
 *  - `"position"`: only the pose term drives `qdot_cmd`; `velocity` is
 *    ignored.
 *  - `"velocity"`: only the feedforward twist drives `qdot_cmd`; `P_gain`
 *    is forced to `0.0` at configure time (logged), so `pose` is ignored.
 *  - `"position_velocity"`: both terms active.
 *
 * `kinematics_` provides `forwardKinematics()`, `jacobian()`,
 * `jacobianPseudoInverse()` -- variable damping (Nakamura & Hanafusa's
 * singularity-robust inverse), live-tunable via `jacobian_damping_max`
 * (damping magnitude applied exactly at a singularity) and
 * `singularity_threshold` (the smallest-singular-value threshold below
 * which damping ramps up from 0); see `RobotKinematics::
 * jacobianPseudoInverse()`'s doc comment for why a single fixed damping
 * value can't be both accurate away from singularities and safe at one --
 * and `inverseKinematics()` for whatever the control law needs.
 *
 * This controller commands joint velocities directly (not an integrated
 * position reference), so on real UR hardware it needs to know about the
 * driver's safety speed scaling the same way `scaled_joint_trajectory_
 * controller` does — see `ur_robot_driver`'s `speed_scaling/speed_scaling_
 * factor` state interface. That interface is only exported by `ur_robot_
 * driver`'s own hardware interface (not by this workspace's Isaac/Gazebo
 * backends), so rather than claiming it directly and failing to configure on
 * those backends, this controller always subscribes to the scaling factor
 * published by `speed_scaling_state_broadcaster` on `speed_scaling_topic`
 * and exposes it (`speed_scaling_factor_buffer_`) for the control law to
 * apply when `use_speed_scaling` is true (1.0/no scaling if that broadcaster
 * isn't running, or the option is off).
 *
 * Reference trajectory shaping: the pose/twist actually used in the control
 * law above are not simply the raw values from `~/commands` -- how they're
 * derived depends on `command_mode`:
 *
 *  - `"velocity"`: same one-raw-sample-delay FIFO as `"position_velocity"`
 * below, but a plain LINEAR segment between two consecutive raw twists instead
 * of a pose-matching cubic (there's no pose term to shape -- `P_gain` is forced
 * to `0.0`): once twist_k is received, `xdot_d` ramps linearly from twist_{k-1}
 * to twist_k over `T` seconds, `T` being the actual difference between the two
 * samples' publisher-assigned `time_from_start`, exactly like
 *    `"position_velocity"`'s `T`. Unlike that mode's rotation-vector
 * construction, this is a plain 6-vector lerp -- the twist is just a vector
 * being forwarded, not a derivative of a tracked orientation, so there's no
 * anchor frame to reason about. This replaces forwarding `xdot_d` directly from
 * the latest raw message, which produced a step discontinuity in the commanded
 * velocity at every publish tick whenever consecutive samples differed.
 * Watchdog: zeros `xdot_d` directly on staleness (no deceleration ramp, same
 * reasoning as `JointPositionController`'s `"velocity"` mode) and resets the
 * one-sample-delay buffering so the next two fresh samples start a clean
 * segment.
 *  - `"position"`: `target_pose_used` is evaluated from **plain linear
 *    interpolation** -- translation moves at the constant rate
 *    `(target - current) / trajectory_duration`; orientation follows a
 *    constant-angular-rate blend of the rotation vector
 *    (`RobotKinematics::quaternionLog`/`quaternionExp`, re-anchored to the
 *    current orientation at every replan), exactly a standard SLERP from
 *    current to target. This keeps the pose itself smooth (no step
 *    discontinuities feeding the pose-error term). The twist `xdot_d`,
 *    however, is always `0`: this mode commands **pure P-control** with no
 *    twist feedforward at all. Earlier iterations fed the segment's implied
 *    constant twist forward, but that constant jumps to a new value at
 *    every replan -- most sharply right around a direction change, where
 *    consecutive targets' implied twists can point opposite ways -- which
 *    showed up as exactly the same kind of feedforward-driven
 *    chatter/spikes this whole reference-shaping design exists to avoid.
 *    Dropping the feedforward removes that failure mode entirely, at the
 *    cost of relying on `P_gain` alone to track a moving target.
 *  - `"position_velocity"`: mirrors `JointPositionController`'s
 *    `"position_velocity"` mode exactly. Every `~/commands` message is
 *    pushed onto `raw_command_queue_` directly from `commandCallback()`
 *    (not detected later by polling `desired_state_buffer_` at `update()`'s
 *    rate), so no sample is lost even if several arrive within one
 *    `update()` cycle.
 *
 *    `update()` consumes that queue with a **one-raw-sample delay**: two
 *    consecutive raw samples `(P_{k-1}, P_k)` become the boundary
 *    conditions of a cubic Hermite segment (translation: per-axis cubic;
 *    orientation: cubic on a rotation vector anchored to `P_{k-1}`'s
 *    orientation, so it starts at `r=0`) executed over T seconds, matching
 *    `P_k`'s pose AND twist exactly -- `trans_v0`/`rot_v0` are the *raw*
 *    commanded twist from `P_{k-1}`, not a centered finite difference or
 *    previous-segment evaluation: an earlier version estimated `v0` that
 *    way, and separately an even earlier version ran boundary twists
 *    through Fritsch-Carlson monotone tangent limiting -- both approaches
 *    broke down the same way `JointPositionController`'s did (see its doc
 *    comment), so this controller now shares its exact fix: use the
 *    literal raw per-message twist as the boundary derivative. For
 *    `trans_v0`/`trans_vf` that means unchanged, straight off the sample;
 *    `rot_v0`/`rot_vf` still come directly from the raw commanded angular
 *    velocity with no estimation, but *do* get rotated into the segment's
 *    anchor frame first (`new_anchor.inverse() * raw_angular_velocity`,
 *    see `startRawHermiteSegment()`) -- the raw angular velocity is in the
 *    base frame, but the cubic is on the anchor-relative rotation vector,
 *    so using it unrotated makes the boundary derivative point along the
 *    wrong axis whenever the anchor orientation isn't near identity
 *    (visible as the tracked orientation rotating around extra axes beyond
 *    the commanded one). Exact at `t=0` (the exponential map's derivative
 *    at `r=0` is the identity, and `new_anchor` IS `P_{k-1}`'s
 *    orientation); a first-order approximation at `t=T` reusing the same
 *    anchor, accurate for the modest rotations typical of one segment.
 *    Deliberately a cubic (matching pose/twist only), not a quintic
 *    matching acceleration too -- this control law never consumes an
 *    acceleration feedforward, so matching it across segments buys nothing
 *    while making the construction more sensitive to segment-to-segment
 *    inconsistency (a quintic's highest-order coefficients scale as
 *    `1/T^4`/`1/T^5`; the cubic's worst term is only `1/T^3`).
 *
 *    T is NOT the trajectory_duration parameter but the actual difference
 *    between the two raw samples' publisher-assigned `time_from_start`
 *    (`T = P_k.trajectory_time - P_{k-1}.trajectory_time`, see
 *    `RawCommand::trajectory_time`) -- deriving T from what the publisher
 *    says it intended, rather than assuming a fixed trajectory_duration,
 *    makes the segment's math insensitive to executor/DDS scheduling
 *    jitter between publish and receipt, same construction as
 *    `JointPositionController`'s. If `T <= 0` (the publisher restarted its
 *    trajectory time counter, sent samples out of order, or sent a
 *    duplicate), the raw-command stream is treated as restarting:
 *    `raw_command_queue_` is cleared and the offending sample becomes the
 *    new P0 instead of being blended against the old (now numerically
 *    meaningless) T -- see `update()`'s "position_velocity" branch.
 *    `trajectory_duration` is only used as the initial placeholder segment
 *    duration at `on_activate()` and by `"position"` mode's plain linear
 *    interpolation, which has no raw sample timestamps to derive a
 *    duration from.
 *
 *    The playback clock (`traj_start_time_`) is re-anchored to the current
 *    time only for the very first segment (after activation, or after a
 *    stale-command reset below); every later segment advances it by
 *    exactly `traj_duration_` instead of snapping to "now" (`playback_
 *    started_` tracks this) -- otherwise small per-cycle scheduling
 *    lateness in `update()` accumulates into a systematic slowdown of the
 *    segment cadence relative to the raw publisher's, which is exactly what
 *    let the queue grow without bound even with a correctly-matched
 *    `trajectory_duration`.
 *
 *    If no `~/commands` message arrives for longer than `command_timeout`
 *    seconds (tracked via `last_command_stamp_buffer_`), the reference does
 *    not drop `xdot_d` to 0 immediately: the instant staleness is first
 *    detected, whatever pose/twist the reference trajectory was actually
 *    commanding at that moment becomes the start of a one-shot
 *    deceleration segment -- built with the same cubic Hermite machinery,
 *    targeting `xdot_d=0` after `stale_deceleration_time` seconds, with the
 *    end pose chosen so each cubic degenerates to a plain
 *    constant-deceleration ramp (exactly `JointPositionController`'s
 *    construction, applied to translation and to the re-anchored rotation
 *    vector independently). `raw_command_queue_` is cleared and
 *    `previous_raw_desired_`/`playback_started_` are reset at the same
 *    time: the next two fresh samples become a brand new `P0 -> P1` segment
 *    instead of a continuation, since blending a fresh sample against a
 *    stale waypoint or playback schedule would produce a velocity spike
 *    sized by however long the gap was.
 *
 * An earlier version instead independently low-pass filtered position,
 * orientation (via slerp), and twist with separate exponential smoothers.
 * That's fine when the twist is being tracked in its own right (`"velocity"`
 * mode), but breaks for `"position_velocity"` mode the same way it did in
 * `JointPositionController`: the twist is supposed to be the literal
 * derivative of the pose, and filtering them separately makes them drift out
 * of that relationship during any transient, which showed up as tracking
 * overshoot. The polynomial constructions above keep that relationship
 * exact by construction instead.
 *
 * `P_gain`, `jacobian_damping_max`, `singularity_threshold`,
 * `command_timeout`, `trajectory_duration`, `stale_deceleration_time`,
 * `use_speed_scaling`, and `velocity_scale` can be changed live
 * with `ros2 param set` while the controller is running; `joints`/
 * `base_link`/`tip_link`/`speed_scaling_topic` are read-only.
 * `velocity_scale` (default `1.0`, must be in `(0, 1]`) multiplies
 * `RobotKinematics::velocityLimits()` before the final `qdot_cmd` clamp --
 * e.g. set to `0.2` to run the arm at 20% of its URDF velocity limits
 * without editing the URDF, for a first test on real hardware.
 *
 * Diagnostic output: `target_pose_used`/`xdot_d` (the same values fed into
 * the control law above) are published every cycle as a
 * geometry_msgs/PoseStamped on `~/interpolated_pose` and a
 * geometry_msgs/TwistStamped on `~/interpolated_twist` -- e.g. for plotting
 * in PlotJuggler/rqt_plot against `CartesianPoseBroadcaster`'s `~/pose`/
 * `~/twist` to see the effect of `trajectory_duration` and the
 * reference-shaping construction directly, rather than inferring it.
 * `det(J)` (well-defined since this is a non-redundant 6-DOF arm, so `J`
 * is square) is published as a std_msgs/Float64 on `~/jacobian_determinant`
 * every cycle, so you can watch how close the current configuration is to
 * a singularity without inferring it from joint angles. It's a coarse
 * indicator, not the same quantity `jacobianPseudoInverse()`'s variable
 * damping actually reacts to (that's `J`'s smallest singular value): `det(J)`
 * can be small while every singular value is only moderately small (e.g.
 * several axes each contributing a modest factor), and conversely a `det(J)`
 * that looks "not that small" can still mean one singular value is
 * uncomfortably close to zero if another is correspondingly large -- useful
 * for a quick visual/trend check, but don't tune `singularity_threshold`
 * against this value.
 */
class CartesianPositionController
    : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

private:
  // A ~/commands sample, decomposed into the Eigen types the rest of this class
  // works with. Used both as "the latest received command"
  // (desired_state_buffer_, for "position"/"velocity" modes) and as one FIFO
  // entry (raw_command_queue_, for "position_velocity" mode) -- same dual role
  // as JointPositionController::DesiredState.
  struct RawCommand {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};
    uint64_t sequence{0};

    // Trajectory time (seconds) this sample's pose/twist were evaluated at by
    // the publisher, taken directly from the raw message's time_from_start --
    // NOT the time this sample was received. Used in "position_velocity" mode
    // to compute a raw-to-raw segment's actual intended duration instead of
    // assuming trajectory_duration; see startRawHermiteSegment() and
    // JointPositionController::DesiredState::trajectory_time for the identical
    // construction.
    double trajectory_time{0.0};
  };

  std::vector<std::string> joint_names_;
  std::size_t num_joints_{0};
  std::string base_link_;
  std::string tip_link_;
  std::string speed_scaling_topic_;
  std::string command_mode_;

  RobotKinematics kinematics_;

  // Reference trajectory state (RT-thread owned, read/written exclusively
  // inside update()) -- see "Reference trajectory shaping" above. Unused in
  // "velocity" mode. Translation is a plain cubic in base_link coordinates;
  // orientation is a cubic on a rotation vector re-anchored to
  // traj_orientation_anchor_ at every replan (so the vector is always 0 at t=0
  // -- see replanTrajectory()'s doc comment for why that makes the boundary
  // velocity transfer exactly across replans).
  Eigen::Vector3d traj_trans_c0_{Eigen::Vector3d::Zero()},
      traj_trans_c1_{Eigen::Vector3d::Zero()},
      traj_trans_c2_{Eigen::Vector3d::Zero()},
      traj_trans_c3_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d traj_rot_c0_{Eigen::Vector3d::Zero()},
      traj_rot_c1_{Eigen::Vector3d::Zero()},
      traj_rot_c2_{Eigen::Vector3d::Zero()},
      traj_rot_c3_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond traj_orientation_anchor_{Eigen::Quaterniond::Identity()};
  rclcpp::Time traj_start_time_;
  double traj_duration_{0.0};

  // Pointer to the newest RawCommand already observed by the RT loop. Used only
  // to detect when a new command has arrived (in "position" mode).
  std::shared_ptr<RawCommand> last_planned_desired_;

  // Used only in "position_velocity" mode. traj_start_time_ is initialized
  // at on_activate() to a hold-pose placeholder, which can be arbitrarily
  // stale by the time the first real raw-to-raw Hermite segment starts.
  // This flag marks that first segment (and the first one after a
  // stale-command reset) so update() can re-anchor traj_start_time_ to the
  // current time exactly once; every segment after that preserves the
  // fixed playback timeline by advancing traj_start_time_ by
  // traj_duration_ instead of resetting it, so scheduling lateness cannot
  // accumulate -- see the class doc comment.
  bool playback_started_{false};

  // Used only in "position_velocity" mode. Set once a one-shot
  // deceleration segment has been built for the current stale episode (see
  // the class doc comment's "Reference trajectory shaping" section), so
  // update() doesn't rebuild it -- and reset a shrinking v0 back toward
  // zero -- every cycle while still stale. Reset to false as soon as the
  // command stream is fresh again.
  bool decel_planned_{false};

  // --------------------------------------------------------------------------
  // One-sample-delay raw-command buffering ("position_velocity" mode only)
  //
  // previous_raw_desired_:
  //   Previous raw command sample, i.e. the start of the currently executing
  //   (or most recently executed) Hermite segment.
  //
  // raw_command_queue_ / raw_command_queue_mutex_:
  //   FIFO of raw command samples received from commandCallback() but not
  //   yet consumed as a segment endpoint. commandCallback() pushes to the
  //   back; update() pops from the front once the current segment
  //   finishes, so no sample is skipped even if several arrive while one
  //   segment is executing.
  // --------------------------------------------------------------------------

  std::shared_ptr<RawCommand> previous_raw_desired_;

  std::deque<std::shared_ptr<RawCommand>> raw_command_queue_;
  std::mutex raw_command_queue_mutex_;

  // --------------------------------------------------------------------------
  // One-sample-delay raw-TWIST buffering ("velocity" mode only)
  //
  // Same dual-role pattern as previous_raw_desired_/raw_command_queue_ above,
  // but for the plain linear xdot_d(t) = vel_traj_c0_ + t*vel_traj_c1_ segment
  // described in the class doc comment's "velocity" section. Kept separate from
  // traj_trans_c*_/traj_rot_c*_ above (never active at the same time, since
  // command_mode_ is fixed per instance) because those are specifically a
  // translation cubic plus an anchored rotation-vector cubic, not a plain
  // 6-vector -- reusing them would need an anchor/rotation story this mode
  // doesn't have.
  // --------------------------------------------------------------------------

  std::shared_ptr<RawCommand> previous_raw_velocity_;

  std::deque<std::shared_ptr<RawCommand>> raw_velocity_queue_;
  std::mutex raw_velocity_queue_mutex_;

  Eigen::Matrix<double, 6, 1> vel_traj_c0_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> vel_traj_c1_{Eigen::Matrix<double, 6, 1>::Zero()};
  rclcpp::Time vel_traj_start_time_;
  double vel_traj_duration_{0.0};
  bool vel_playback_started_{false};
  bool vel_stale_reset_done_{false};

  realtime_tools::RealtimeBuffer<std::shared_ptr<RawCommand>>
      desired_state_buffer_;
  realtime_tools::RealtimeBuffer<double> speed_scaling_factor_buffer_;
  realtime_tools::RealtimeBuffer<double> p_gain_buffer_;
  // See RobotKinematics::jacobianPseudoInverse()'s doc comment for the
  // variable-damping formula these two feed -- jacobian_damping_max is NOT
  // applied as a constant, only ramped up to as singularity_threshold_buffer_
  // is approached.
  realtime_tools::RealtimeBuffer<double> jacobian_damping_max_buffer_;
  realtime_tools::RealtimeBuffer<double> singularity_threshold_buffer_;
  realtime_tools::RealtimeBuffer<double> command_timeout_buffer_;
  realtime_tools::RealtimeBuffer<double> trajectory_duration_buffer_;
  realtime_tools::RealtimeBuffer<bool> use_speed_scaling_buffer_;

  // Duration (seconds) of the one-shot deceleration ramp built when the
  // command stream first goes stale in "position_velocity" mode -- see the
  // class doc comment's "Reference trajectory shaping" section.
  realtime_tools::RealtimeBuffer<double> stale_deceleration_time_buffer_;

  // Multiplies RobotKinematics::velocityLimits() before the final qdot_cmd
  // clamp -- (0, 1], default 1.0 (no scaling). Live-tunable so the URDF's
  // per-joint velocity limits can be turned down without editing the URDF, e.g.
  // for a first test on real hardware.
  realtime_tools::RealtimeBuffer<double> velocity_scale_buffer_;

  // Updated on every ~/commands message. Drives the "velocity" mode twist
  // watchdog and the "position_velocity" mode command_stale check -- see the
  // class doc comment.
  realtime_tools::RealtimeBuffer<rclcpp::Time> last_command_stamp_buffer_;

  rclcpp::Subscription<inverse_msgs::msg::CartesianTrajectoryPoint>::SharedPtr
      commands_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      speed_scaling_subscriber_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  // Diagnostic output -- see the class doc comment's last paragraph.
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
      interpolated_pose_publisher_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>
      interpolated_twist_publisher_;
  // det(J) -- only meaningful because this is a non-redundant 6-DOF arm, so J
  // is square; see the class doc comment's "Diagnostic output" paragraph for
  // what it's for and its limitations.
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>
      jacobian_determinant_publisher_;

  Eigen::Vector3d evalTransPosition(double t) const;
  Eigen::Vector3d evalTransVelocity(double t) const;
  Eigen::Vector3d evalRotVector(double t) const;
  Eigen::Vector3d evalRotVelocity(double t) const;
  // "position" mode only -- plain linear/constant-rate-blend replan, zero
  // twist feedforward. See the class doc comment.
  void replanTrajectory(const Eigen::Isometry3d &target,
                        const Eigen::Matrix<double, 6, 1> &vf,
                        const rclcpp::Time &time);

  // "position_velocity" mode only -- cubic Hermite segment directly between
  // two raw command samples, matching p1's pose AND twist exactly. v0/v1
  // are read straight off p0/p1, no estimation -- but the angular
  // components ARE rotated into the segment's anchor frame first (the raw
  // twist is base-frame, the cubic is on the anchor-relative rotation
  // vector), exact at t=0 and a first-order approximation at t=T -- see
  // the class doc comment's "position_velocity" section. Over the
  // caller-supplied duration T -- the actual difference between the two
  // samples' publisher-assigned trajectory_time (see RawCommand), not the
  // trajectory_duration parameter. The caller is responsible for
  // validating T (see update()'s "position_velocity" branch: T <= 0 means
  // the publisher restarted its trajectory time counter or sent samples
  // out of order). Does NOT touch traj_start_time_/playback_started_ --
  // the caller (update()) is responsible for scheduling the segment's
  // start time, same convention as
  // JointPositionController::startRawHermiteSegment().
  void startRawHermiteSegment(const RawCommand &p0, const RawCommand &p1,
                              double T);

  // "velocity" mode only -- plain linear segment directly between two
  // consecutive raw commanded twists, matching v1 exactly at t=T. Analogue of
  // startRawHermiteSegment() above without a pose to match. Does NOT touch
  // vel_traj_start_time_/vel_playback_started_ -- the caller schedules the
  // segment's start time.
  [[nodiscard]] Eigen::Matrix<double, 6, 1> evalVelLinear(double t) const;
  void startRawLinearVelocitySegment(const Eigen::Matrix<double, 6, 1> &v0,
                                     const Eigen::Matrix<double, 6, 1> &v1,
                                     double T);

  void commandCallback(
      const inverse_msgs::msg::CartesianTrajectoryPoint::SharedPtr msg);
  void speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult
  onParameterUpdate(const std::vector<rclcpp::Parameter> &parameters);
};

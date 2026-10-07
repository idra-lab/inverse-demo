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

#include "velocity_controller/cartesian_position_controller.hpp"

#include <algorithm>
#include <exception>

namespace {
// Solves for a per-axis cubic Hermite polynomial q(t) = c0 + c1*t + c2*t^2 +
// c3*t^3 matching q(0)=q0, qdot(0)=v0, q(duration)=qf, qdot(duration)=vf.
// Defined below; forward-declared here so update()'s stale-command deceleration
// segment (built inline, not via startRawHermiteSegment()) can call it too.
void solveCubic(const Eigen::Vector3d &q0, const Eigen::Vector3d &v0,
                const Eigen::Vector3d &qf, const Eigen::Vector3d &vf,
                double duration, Eigen::Vector3d &c0, Eigen::Vector3d &c1,
                Eigen::Vector3d &c2, Eigen::Vector3d &c3);
} // namespace

controller_interface::InterfaceConfiguration
CartesianPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::return_type
CartesianPositionController::update(const rclcpp::Time &time,
                                    const rclcpp::Duration & /*period*/) {
  Eigen::VectorXd q(static_cast<Eigen::Index>(num_joints_));
  Eigen::VectorXd qdot(static_cast<Eigen::Index>(num_joints_));
  for (std::size_t i = 0; i < num_joints_; ++i) {
    q(static_cast<Eigen::Index>(i)) = state_interfaces_[2 * i].get_value();
    qdot(static_cast<Eigen::Index>(i)) =
        state_interfaces_[2 * i + 1].get_value();
  }

  const Eigen::Isometry3d transform = kinematics_.forwardKinematics(q);
  const double command_timeout = *command_timeout_buffer_.readFromRT();

  Eigen::Isometry3d target_pose_used = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 1> xdot_d = Eigen::Matrix<double, 6, 1>::Zero();

  if (command_mode_ == "velocity") {
    // target_pose_used below is unused (P_gain forced to 0 in this mode) --
    // only xdot_d matters. Same one-raw-sample-delay FIFO as
    // "position_velocity" below, but a plain LINEAR segment between two
    // consecutive raw twists instead of a pose-matching cubic -- see the class
    // doc comment's "velocity" section.
    auto desired = *desired_state_buffer_.readFromRT();
    target_pose_used.translation() = desired->translation;
    target_pose_used.linear() = desired->orientation.toRotationMatrix();

    const bool command_stale =
        (time - *last_command_stamp_buffer_.readFromRT()).seconds() >
        command_timeout;

    if (command_stale) {
      if (!vel_stale_reset_done_) {
        {
          std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
          raw_velocity_queue_.clear();
        }
        previous_raw_velocity_.reset();
        vel_playback_started_ = false;
        vel_stale_reset_done_ = true;
      }
    } else {
      vel_stale_reset_done_ = false;

      if (!previous_raw_velocity_) {
        std::shared_ptr<RawCommand> first;
        {
          std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
          if (!raw_velocity_queue_.empty()) {
            first = raw_velocity_queue_.front();
            raw_velocity_queue_.pop_front();
          }
        }
        if (first) {
          previous_raw_velocity_ = first;
        }
      }

      bool queue_has_next = false;
      {
        std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
        queue_has_next = !raw_velocity_queue_.empty();
      }

      if (previous_raw_velocity_ && queue_has_next) {
        const bool need_new_segment =
            !vel_playback_started_ ||
            (time - vel_traj_start_time_).seconds() >= vel_traj_duration_;
        if (need_new_segment) {
          std::shared_ptr<RawCommand> next;
          {
            std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
            next = raw_velocity_queue_.front();
            raw_velocity_queue_.pop_front();
          }

          const double T =
              next->trajectory_time - previous_raw_velocity_->trajectory_time;

          if (T <= 1e-6) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Raw trajectory_time did not advance (%.6f -> %.6f, "
                        "T=%.6f); treating "
                        "seq=%lu as the start of a new velocity stream.",
                        previous_raw_velocity_->trajectory_time,
                        next->trajectory_time, T,
                        static_cast<unsigned long>(next->sequence));
            {
              std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
              raw_velocity_queue_.clear();
            }
            previous_raw_velocity_ = next;
            vel_playback_started_ = false;
          } else {
            if (!vel_playback_started_) {
              vel_traj_start_time_ = time;
              vel_playback_started_ = true;
            } else {
              vel_traj_start_time_ =
                  vel_traj_start_time_ +
                  rclcpp::Duration::from_seconds(vel_traj_duration_);
            }
            startRawLinearVelocitySegment(previous_raw_velocity_->twist,
                                          next->twist, T);
            previous_raw_velocity_ = next;
          }
        }
      }
    }

    const double vel_tau =
        vel_playback_started_
            ? std::clamp((time - vel_traj_start_time_).seconds(), 0.0,
                         vel_traj_duration_)
            : 0.0;
    xdot_d = vel_playback_started_ ? evalVelLinear(vel_tau)
                                   : Eigen::Matrix<double, 6, 1>::Zero();
    if (command_stale) {
      xdot_d = Eigen::Matrix<double, 6, 1>::Zero();
    }

  } else if (command_mode_ == "position") {
    // Track a cubic-degenerate (linear/constant-rate-blend) reference
    // trajectory from wherever it currently is toward each newly published
    // ~/commands pose. xdot_d stays zero (pure P-control) -- see the class doc
    // comment's "Reference trajectory shaping" section.
    auto desired = *desired_state_buffer_.readFromRT();
    if (desired != last_planned_desired_) {
      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() = desired->translation;
      target.linear() = desired->orientation.toRotationMatrix();
      replanTrajectory(target, Eigen::Matrix<double, 6, 1>::Zero(), time);
      last_planned_desired_ = desired;
    }
    const double tau =
        std::clamp((time - traj_start_time_).seconds(), 0.0, traj_duration_);
    target_pose_used.translation() = evalTransPosition(tau);
    target_pose_used.linear() =
        (traj_orientation_anchor_ * quaternionExp(evalRotVector(tau)))
            .toRotationMatrix();

  } else {
    // "position_velocity": raw samples are pushed onto raw_command_queue_
    // directly from commandCallback() (not detected here by polling
    // desired_state_buffer_), so no sample is lost even if several ~/commands
    // messages arrive within one update() cycle.

    const bool command_stale =
        (time - *last_command_stamp_buffer_.readFromRT()).seconds() >
        command_timeout;

    if (command_stale) {
      if (!decel_planned_) {
        // Rising edge into staleness. Drop everything buffered instead of
        // holding it for whenever publishing resumes: otherwise the first fresh
        // sample after the gap would get Hermite-blended against a waypoint
        // (and a playback schedule) that's now stale, producing a velocity
        // spike sized by however long the gap was. The next two fresh samples
        // become a brand new P0 -> P1 segment instead of a continuation -- see
        // the class doc comment.
        {
          std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
          raw_command_queue_.clear();
        }
        previous_raw_desired_.reset();
        playback_started_ = false;

        // Build a one-shot deceleration segment from whatever the reference
        // trajectory was actually commanding at this instant, instead of
        // dropping xdot_d straight to 0 -- see the class doc comment's
        // "Reference trajectory shaping" section. Re-anchoring the rotation
        // vector to the current orientation (new_anchor) makes rot0 = 0, so
        // choosing rot1 = 0.5*rot_v0*T (mirroring trans1 = trans0 +
        // 0.5*trans_v0*T) makes each cubic degenerate to a plain
        // constant-deceleration ramp (c3=0): qdot(t) = v0*(1-t/T), same
        // construction as JointPositionController's.
        const double tau0 = std::clamp((time - traj_start_time_).seconds(), 0.0,
                                       traj_duration_);
        const Eigen::Vector3d trans0 = evalTransPosition(tau0);
        const Eigen::Vector3d trans_v0 = evalTransVelocity(tau0);
        const Eigen::Quaterniond new_anchor =
            traj_orientation_anchor_ * quaternionExp(evalRotVector(tau0));
        const Eigen::Vector3d rot_v0 = evalRotVelocity(tau0);

        const double decel_time = *stale_deceleration_time_buffer_.readFromRT();

        const Eigen::Vector3d trans1 = trans0 + 0.5 * trans_v0 * decel_time;
        const Eigen::Vector3d rot1 = 0.5 * rot_v0 * decel_time;

        solveCubic(trans0, trans_v0, trans1, Eigen::Vector3d::Zero(),
                   decel_time, traj_trans_c0_, traj_trans_c1_, traj_trans_c2_,
                   traj_trans_c3_);
        solveCubic(Eigen::Vector3d::Zero(), rot_v0, rot1,
                   Eigen::Vector3d::Zero(), decel_time, traj_rot_c0_,
                   traj_rot_c1_, traj_rot_c2_, traj_rot_c3_);

        traj_orientation_anchor_ = new_anchor;
        traj_start_time_ = time;
        traj_duration_ = decel_time;
        decel_planned_ = true;

        RCLCPP_WARN(get_node()->get_logger(),
                    "Command stream stale; decelerating to a stop over %.3fs.",
                    decel_time);
      }

      const double tau =
          std::clamp((time - traj_start_time_).seconds(), 0.0, traj_duration_);
      target_pose_used.translation() = evalTransPosition(tau);
      target_pose_used.linear() =
          (traj_orientation_anchor_ * quaternionExp(evalRotVector(tau)))
              .toRotationMatrix();
      xdot_d.head<3>() = evalTransVelocity(tau);
      // evalRotVelocity(tau) is the LOCAL (anchor-relative) rotation-vector
      // rate -- rotate it into the base frame (anchor * local_rate) before
      // using it as xdot_d's angular component, same reasoning as
      // startRawHermiteSegment()'s rot_v0/rot_vf.
      xdot_d.tail<3>() =
          traj_orientation_anchor_.toRotationMatrix() * evalRotVelocity(tau);

    } else {
      decel_planned_ = false;

      // ------------------------------------------------------------------------
      // Get the first raw waypoint (P0) if we don't have one yet -- e.g. right
      // after activation, or right after a stale-command reset above.
      // ------------------------------------------------------------------------

      if (!previous_raw_desired_) {
        std::shared_ptr<RawCommand> first;
        {
          std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
          if (!raw_command_queue_.empty()) {
            first = raw_command_queue_.front();
            raw_command_queue_.pop_front();
          }
        }
        if (first) {
          previous_raw_desired_ = first;
          RCLCPP_INFO(get_node()->get_logger(),
                      "Stored first raw command: seq=%lu",
                      static_cast<unsigned long>(first->sequence));
        }
      }

      // ------------------------------------------------------------------------
      // Only start/advance a segment once a second waypoint (P1) is actually
      // queued, and only pop it once one is actually due -- a burst that
      // arrived during one segment is drained one segment at a time rather than
      // being collapsed/overwritten.
      // ------------------------------------------------------------------------

      bool queue_has_next = false;
      {
        std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
        queue_has_next = !raw_command_queue_.empty();
      }

      if (previous_raw_desired_ && queue_has_next) {
        const bool need_new_segment =
            !playback_started_ ||
            (time - traj_start_time_).seconds() >= traj_duration_;

        if (need_new_segment) {
          std::shared_ptr<RawCommand> next;
          std::size_t queue_size_after_pop = 0;
          {
            std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
            next = raw_command_queue_.front();
            raw_command_queue_.pop_front();
            queue_size_after_pop = raw_command_queue_.size();
          }

          // Diagnostic: nominal setup (publish rate matching
          // trajectory_duration) should keep this around 0-1. Steady growth
          // means update() is consuming segments slower than commands arrive --
          // a separate timing problem.
          RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              "raw_command_queue_ size after pop: %zu", queue_size_after_pop);

          // Diagnostic: the sequence numbers should differ by exactly 1.
          const uint64_t seq0 = previous_raw_desired_->sequence;
          const uint64_t seq1 = next->sequence;
          if (seq1 != seq0 + 1) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "SKIPPED RAW COMMAND(S): starting segment %lu -> %lu",
                         static_cast<unsigned long>(seq0),
                         static_cast<unsigned long>(seq1));
          } else {
            RCLCPP_INFO(get_node()->get_logger(), "Starting segment %lu -> %lu",
                        static_cast<unsigned long>(seq0),
                        static_cast<unsigned long>(seq1));
          }

          // The segment's actual intended duration is the difference between
          // the two raw samples' publisher-assigned trajectory time, not the
          // trajectory_duration parameter
          // -- see the class doc comment's "position_velocity" section.
          const double T =
              next->trajectory_time - previous_raw_desired_->trajectory_time;

          if (T <= 1e-6) {
            // The publisher restarted its trajectory time counter, sent samples
            // out of order, or sent a duplicate -- the old segment's timeline
            // is no longer numerically meaningful. Treat this sample as the
            // first point (P0) of a brand new stream instead of blending it
            // against a bogus T.
            RCLCPP_WARN(get_node()->get_logger(),
                        "Raw trajectory_time did not advance (%.6f -> %.6f, "
                        "T=%.6f); treating "
                        "seq=%lu as the start of a new trajectory stream.",
                        previous_raw_desired_->trajectory_time,
                        next->trajectory_time, T,
                        static_cast<unsigned long>(next->sequence));

            {
              std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
              raw_command_queue_.clear();
            }

            previous_raw_desired_ = next;
            playback_started_ = false;

          } else {
            if (!playback_started_) {
              // First segment after activation or after a stale-command reset:
              // traj_start_time_ is a stale placeholder, so re-anchor to the
              // current time instead of clamping tau straight to
              // traj_duration_.
              traj_start_time_ = time;
              playback_started_ = true;
            } else {
              // Every later segment stays on the fixed playback timeline
              // instead of re-anchoring to "now", so per-cycle scheduling
              // lateness can't accumulate into a systematic slowdown of the
              // segment cadence relative to the raw publisher's.
              traj_start_time_ = traj_start_time_ +
                                 rclcpp::Duration::from_seconds(traj_duration_);
            }

            startRawHermiteSegment(*previous_raw_desired_, *next, T);

            previous_raw_desired_ = next;
          }
        }
      }

      const double tau =
          std::clamp((time - traj_start_time_).seconds(), 0.0, traj_duration_);
      target_pose_used.translation() = evalTransPosition(tau);
      target_pose_used.linear() =
          (traj_orientation_anchor_ * quaternionExp(evalRotVector(tau)))
              .toRotationMatrix();
      xdot_d.head<3>() = evalTransVelocity(tau);
      // evalRotVelocity(tau) is the LOCAL (anchor-relative) rotation-vector
      // rate -- rotate it into the base frame (anchor * local_rate) before
      // using it as xdot_d's angular component, same reasoning as
      // startRawHermiteSegment()'s rot_v0/rot_vf.
      xdot_d.tail<3>() =
          traj_orientation_anchor_.toRotationMatrix() * evalRotVelocity(tau);
    }
  }

  if (interpolated_pose_publisher_ && interpolated_pose_publisher_->trylock()) {
    const Eigen::Vector3d translation = target_pose_used.translation();
    const Eigen::Quaterniond orientation(target_pose_used.rotation());
    interpolated_pose_publisher_->msg_.header.stamp = time;
    interpolated_pose_publisher_->msg_.pose.position.x = translation.x();
    interpolated_pose_publisher_->msg_.pose.position.y = translation.y();
    interpolated_pose_publisher_->msg_.pose.position.z = translation.z();
    interpolated_pose_publisher_->msg_.pose.orientation.x = orientation.x();
    interpolated_pose_publisher_->msg_.pose.orientation.y = orientation.y();
    interpolated_pose_publisher_->msg_.pose.orientation.z = orientation.z();
    interpolated_pose_publisher_->msg_.pose.orientation.w = orientation.w();
    interpolated_pose_publisher_->unlockAndPublish();
  }
  if (interpolated_twist_publisher_ &&
      interpolated_twist_publisher_->trylock()) {
    interpolated_twist_publisher_->msg_.header.stamp = time;
    interpolated_twist_publisher_->msg_.twist.linear.x = xdot_d(0);
    interpolated_twist_publisher_->msg_.twist.linear.y = xdot_d(1);
    interpolated_twist_publisher_->msg_.twist.linear.z = xdot_d(2);
    interpolated_twist_publisher_->msg_.twist.angular.x = xdot_d(3);
    interpolated_twist_publisher_->msg_.twist.angular.y = xdot_d(4);
    interpolated_twist_publisher_->msg_.twist.angular.z = xdot_d(5);
    interpolated_twist_publisher_->unlockAndPublish();
  }

  const double p_gain = *p_gain_buffer_.readFromRT();
  const double jacobian_damping_max =
      *jacobian_damping_max_buffer_.readFromRT();
  const double singularity_threshold =
      *singularity_threshold_buffer_.readFromRT();
  const double speed_scaling = *use_speed_scaling_buffer_.readFromRT()
                                   ? *speed_scaling_factor_buffer_.readFromRT()
                                   : 1.0;
  const double velocity_scale = *velocity_scale_buffer_.readFromRT();

  const Eigen::MatrixXd jacobian = kinematics_.jacobian(q);
  const Eigen::Matrix<double, 6, 1> pose_error =
      poseError(transform, target_pose_used);

  if (jacobian_determinant_publisher_ &&
      jacobian_determinant_publisher_->trylock()) {
    // Non-redundant 6-DOF arm -- jacobian is square, so det() is well-defined.
    // See the class doc comment's "Diagnostic output" paragraph for what this
    // is (and isn't) useful for.
    jacobian_determinant_publisher_->msg_.data = jacobian.determinant();
    jacobian_determinant_publisher_->unlockAndPublish();
  }

  // Plain proportional law with a feedforward twist, mapped to joint space
  // with the damped Jacobian pseudo-inverse. Replace this with something
  // more involved if needed -- pose_error follows the same "current -
  // desired" sign convention as CartesianImpedanceController's
  // poseError()/damping term (hence the minus sign below); q/qdot/
  // transform/target_pose_used are all already computed above.
  // kinematics_.inverseKinematics(q, target, q_out) is available too, for a
  // law built around per-cycle IK instead of a Jacobian mapping.
  const Eigen::Matrix<double, 6, 1> xdot_cmd = -p_gain * pose_error + xdot_d;
  Eigen::VectorXd qdot_cmd =
      kinematics_.jacobianPseudoInverse(q, jacobian_damping_max,
                                        singularity_threshold) *
      xdot_cmd;

  qdot_cmd *= speed_scaling;
  qdot_cmd = qdot_cmd.cwiseMax(-velocity_scale * kinematics_.velocityLimits())
                 .cwiseMin(velocity_scale * kinematics_.velocityLimits());

  for (std::size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(qdot_cmd(static_cast<Eigen::Index>(i)));
  }
  return controller_interface::return_type::OK;
}

Eigen::Vector3d CartesianPositionController::evalTransPosition(double t) const {
  return traj_trans_c0_ +
         t * (traj_trans_c1_ + t * (traj_trans_c2_ + t * traj_trans_c3_));
}

Eigen::Vector3d CartesianPositionController::evalTransVelocity(double t) const {
  return traj_trans_c1_ + t * (2.0 * traj_trans_c2_ + t * 3.0 * traj_trans_c3_);
}

Eigen::Vector3d CartesianPositionController::evalRotVector(double t) const {
  return traj_rot_c0_ +
         t * (traj_rot_c1_ + t * (traj_rot_c2_ + t * traj_rot_c3_));
}

Eigen::Vector3d CartesianPositionController::evalRotVelocity(double t) const {
  return traj_rot_c1_ + t * (2.0 * traj_rot_c2_ + t * 3.0 * traj_rot_c3_);
}

namespace {
void solveCubic(const Eigen::Vector3d &q0, const Eigen::Vector3d &v0,
                const Eigen::Vector3d &qf, const Eigen::Vector3d &vf,
                double duration, Eigen::Vector3d &c0, Eigen::Vector3d &c1,
                Eigen::Vector3d &c2, Eigen::Vector3d &c3) {
  const double t1 = duration, t2 = t1 * t1, t3 = t2 * t1;
  c0 = q0;
  c1 = v0;
  c2 = (3.0 * (qf - q0) - (2.0 * v0 + vf) * t1) / t2;
  c3 = (2.0 * (q0 - qf) + (v0 + vf) * t1) / t3;
}

} // namespace

void CartesianPositionController::replanTrajectory(
    const Eigen::Isometry3d &target, const Eigen::Matrix<double, 6, 1> & /*vf*/,
    const rclcpp::Time &time) {
  // Position at the start of the new segment comes from evaluating the
  // *previous* segment at this same instant -- not the raw measured pose -- so
  // restarting mid-segment never introduces a position discontinuity, same
  // reasoning as JointPositionController::replanTrajectory().
  const double tau0 =
      std::clamp((time - traj_start_time_).seconds(), 0.0, traj_duration_);
  const Eigen::Vector3d trans0 = evalTransPosition(tau0);

  // Re-anchor the rotation-vector representation to the orientation the
  // reference trajectory is actually at right now (new_anchor), so the new
  // segment starts at r=0 -- see startRawHermiteSegment()'s doc comment for why
  // that matters when a boundary derivative is reused across the re-anchor.
  const Eigen::Quaterniond new_anchor =
      traj_orientation_anchor_ * quaternionExp(evalRotVector(tau0));
  const Eigen::Vector3d rot_f = quaternionLog(Eigen::Quaterniond(
      new_anchor.inverse() * Eigen::Quaterniond(target.rotation())));

  const double duration = *trajectory_duration_buffer_.readFromRT();

  // Plain linear interpolation (translation) / constant-rate blend of the
  // rotation vector (orientation -- equivalent to a standard SLERP from the
  // current orientation to the target, since exp(t * rot_f/duration) traces a
  // constant-angular-speed geodesic). Deliberately NOT twist-matched -- this
  // mode has no real target twist to match; forcing xdot_d to zero at every
  // waypoint boundary (what a twist-matched cubic would do here) makes the
  // reference repeatedly decelerate to a full stop and reaccelerate whenever
  // it's actually tracking a continuously moving stream of pose-only commands,
  // which shows up as velocity vibration. See the class doc comment.
  traj_trans_c0_ = trans0;
  traj_trans_c1_ = (target.translation() - trans0) / duration;
  traj_trans_c2_ = traj_trans_c3_ = Eigen::Vector3d::Zero();
  traj_rot_c0_ = Eigen::Vector3d::Zero();
  traj_rot_c1_ = rot_f / duration;
  traj_rot_c2_ = traj_rot_c3_ = Eigen::Vector3d::Zero();

  traj_orientation_anchor_ = new_anchor;
  traj_start_time_ = time;
  traj_duration_ = duration;
}

Eigen::Matrix<double, 6, 1>
CartesianPositionController::evalVelLinear(double t) const {
  return vel_traj_c0_ + t * vel_traj_c1_;
}

void CartesianPositionController::startRawLinearVelocitySegment(
    const Eigen::Matrix<double, 6, 1> &v0,
    const Eigen::Matrix<double, 6, 1> &v1, double T) {
  vel_traj_c0_ = v0;
  vel_traj_c1_ = (v1 - v0) / T;
  // vel_traj_start_time_ is scheduled by the caller (update()), not here -- see
  // vel_playback_started_ for why.
  vel_traj_duration_ = T;
}

void CartesianPositionController::startRawHermiteSegment(const RawCommand &p0,
                                                         const RawCommand &p1,
                                                         double T) {
  // The new segment's rotation-vector anchor is p0's orientation exactly (not
  // evaluated from whatever the previous segment happened to be at some tau0):
  // a raw-to-raw segment always starts exactly at p0, so r=0 at t=0 by
  // construction, same as JointPositionController::startRawHermiteSegment()
  // using q0 = previous_raw_desired_->q directly.
  const Eigen::Quaterniond new_anchor = p0.orientation;
  const Eigen::Vector3d rot_f =
      quaternionLog(Eigen::Quaterniond(new_anchor.inverse() * p1.orientation));

  // p0/p1's raw commanded angular velocity is in the base frame, but
  // traj_rot_c* is a cubic on the LOCAL (anchor-relative) rotation vector --
  // rotate both into the anchor's frame first (new_anchor^-1 * v), don't use
  // the raw base-frame vector directly. At t=0 this is exact (the exponential
  // map's derivative at r=0 is the identity, mapping anchor-frame angular
  // velocity to rotation-vector-rate exactly, since new_anchor IS p0's
  // orientation); at t=T it's a first-order approximation reusing the same
  // anchor, accurate for the modest rotations typical of one segment. Skipping
  // this transform entirely (an earlier version did) makes the boundary
  // velocity's axis wrong whenever the anchor orientation isn't close to
  // identity, which shows up as the interpolated orientation visibly rotating
  // around the wrong axis/multiple axes instead of just the commanded one.
  const Eigen::Vector3d rot_v0 = new_anchor.inverse() * p0.twist.tail<3>();
  const Eigen::Vector3d rot_vf = new_anchor.inverse() * p1.twist.tail<3>();

  solveCubic(p0.translation, p0.twist.head<3>(), p1.translation,
             p1.twist.head<3>(), T, traj_trans_c0_, traj_trans_c1_,
             traj_trans_c2_, traj_trans_c3_);
  solveCubic(Eigen::Vector3d::Zero(), rot_v0, rot_f, rot_vf, T, traj_rot_c0_,
             traj_rot_c1_, traj_rot_c2_, traj_rot_c3_);

  traj_orientation_anchor_ = new_anchor;

  // traj_start_time_ is scheduled by the caller (update()), not here -- see its
  // doc comment and playback_started_ for why.
  traj_duration_ = T;
}

CallbackReturn CartesianPositionController::on_init() {
  try {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    // Humble may already have declared YAML overrides when creating the node.
    // Preserve those values rather than attempting to declare them again.
    if (!get_node()->has_parameter("joints")) {
      get_node()->declare_parameter<std::vector<std::string>>("joints", {},
                                                              read_only);
    }
    if (!get_node()->has_parameter("base_link")) {
      get_node()->declare_parameter<std::string>("base_link", "base_link",
                                                 read_only);
    }
    if (!get_node()->has_parameter("tip_link")) {
      get_node()->declare_parameter<std::string>("tip_link", "tool0", read_only);
    }
    if (!get_node()->has_parameter("speed_scaling_topic")) {
      get_node()->declare_parameter<std::string>(
          "speed_scaling_topic", "/speed_scaling_state_broadcaster/speed_scaling",
          read_only);
    }
    if (!get_node()->has_parameter("robot_description_source")) {
      get_node()->declare_parameter<std::string>(
          "robot_description_source", "robot_state_publisher", read_only);
    }
    if (!get_node()->has_parameter("command_mode")) {
      get_node()->declare_parameter<std::string>("command_mode", "position",
                                                 read_only);
    }

    auto_declare<double>("jacobian_damping_max", 0.05);
    auto_declare<double>("singularity_threshold", 0.05);
    auto_declare<double>("command_timeout", 0.1);
    auto_declare<double>("trajectory_duration", 0.5);
    auto_declare<bool>("use_speed_scaling", true);
    auto_declare<double>("P_gain", 1.0);
    auto_declare<double>("stale_deceleration_time", 0.2);
    auto_declare<double>("velocity_scale", 1.0);
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return CallbackReturn::FAILURE;
  }
  num_joints_ = joint_names_.size();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  tip_link_ = get_node()->get_parameter("tip_link").as_string();
  speed_scaling_topic_ =
      get_node()->get_parameter("speed_scaling_topic").as_string();
  jacobian_damping_max_buffer_.writeFromNonRT(
      get_node()->get_parameter("jacobian_damping_max").as_double());
  const double singularity_threshold =
      get_node()->get_parameter("singularity_threshold").as_double();
  if (singularity_threshold <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'singularity_threshold' must be > 0, got %f.",
                 singularity_threshold);
    return CallbackReturn::FAILURE;
  }
  singularity_threshold_buffer_.writeFromNonRT(singularity_threshold);
  command_timeout_buffer_.writeFromNonRT(
      get_node()->get_parameter("command_timeout").as_double());
  const double trajectory_duration =
      get_node()->get_parameter("trajectory_duration").as_double();
  if (trajectory_duration <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'trajectory_duration' must be > 0, got %f.",
                 trajectory_duration);
    return CallbackReturn::FAILURE;
  }
  trajectory_duration_buffer_.writeFromNonRT(trajectory_duration);

  const double stale_deceleration_time =
      get_node()->get_parameter("stale_deceleration_time").as_double();
  if (stale_deceleration_time <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'stale_deceleration_time' must be > 0, got %f.",
                 stale_deceleration_time);
    return CallbackReturn::FAILURE;
  }
  stale_deceleration_time_buffer_.writeFromNonRT(stale_deceleration_time);

  const double velocity_scale =
      get_node()->get_parameter("velocity_scale").as_double();
  if (velocity_scale <= 0.0 || velocity_scale > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'velocity_scale' must be in (0, 1], got %f.", velocity_scale);
    return CallbackReturn::FAILURE;
  }
  velocity_scale_buffer_.writeFromNonRT(velocity_scale);

  use_speed_scaling_buffer_.writeFromNonRT(
      get_node()->get_parameter("use_speed_scaling").as_bool());

  command_mode_ = get_node()->get_parameter("command_mode").as_string();
  if (command_mode_ != "position" && command_mode_ != "velocity" &&
      command_mode_ != "position_velocity") {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'command_mode' must be 'position', 'velocity', or "
                 "'position_velocity', got '%s'.",
                 command_mode_.c_str());
    return CallbackReturn::FAILURE;
  }

  double p_gain = get_node()->get_parameter("P_gain").as_double();
  if (command_mode_ == "velocity") {
    p_gain = 0.0;
    get_node()->set_parameter(rclcpp::Parameter("P_gain", p_gain));
    RCLCPP_INFO(get_node()->get_logger(),
                "command_mode='velocity': forcing P_gain to 0.0 (pure velocity "
                "feedforward, "
                "no position term).");
  }
  p_gain_buffer_.writeFromNonRT(p_gain);

  const std::string urdf_xml = getRobotDescription(
      get_node(),
      get_node()->get_parameter("robot_description_source").as_string());
  if (urdf_xml.empty() ||
      !kinematics_.init(urdf_xml, base_link_, tip_link_, joint_names_,
                        get_node()->get_logger())) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to initialize robot kinematics.");
    return CallbackReturn::FAILURE;
  }

  commands_subscriber_ =
      get_node()
          ->create_subscription<inverse_msgs::msg::CartesianTrajectoryPoint>(
              "~/commands", rclcpp::SystemDefaultsQoS(),
              std::bind(&CartesianPositionController::commandCallback, this,
                        std::placeholders::_1));

  // ur_robot_driver's hardware interface exports
  // speed_scaling/speed_scaling_factor as a state interface, but this
  // workspace's Isaac/Gazebo backends don't, so this is a plain topic
  // subscription (published by speed_scaling_state_broadcaster, 0-100) rather
  // than a claimed state interface: it degrades to no scaling instead of
  // failing to configure when the broadcaster isn't running. Always subscribed;
  // use_speed_scaling only toggles whether the received value is applied, so it
  // can be flipped live.
  speed_scaling_factor_buffer_.writeFromNonRT(1.0);
  speed_scaling_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::Float64>(
          speed_scaling_topic_, rclcpp::SystemDefaultsQoS(),
          std::bind(&CartesianPositionController::speedScalingCallback, this,
                    std::placeholders::_1));

  interpolated_pose_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          "~/interpolated_pose", rclcpp::SystemDefaultsQoS()));
  interpolated_pose_publisher_->msg_.header.frame_id = base_link_;

  interpolated_twist_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/interpolated_twist", rclcpp::SystemDefaultsQoS()));
  interpolated_twist_publisher_->msg_.header.frame_id = base_link_;

  jacobian_determinant_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
      get_node()->create_publisher<std_msgs::msg::Float64>(
          "~/jacobian_determinant", rclcpp::SystemDefaultsQoS()));

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CartesianPositionController::onParameterUpdate, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  Eigen::VectorXd q(static_cast<Eigen::Index>(num_joints_));
  for (std::size_t i = 0; i < num_joints_; ++i) {
    q(static_cast<Eigen::Index>(i)) = state_interfaces_[2 * i].get_value();
  }
  const Eigen::Isometry3d activation_pose = kinematics_.forwardKinematics(q);

  auto hold = std::make_shared<RawCommand>();
  hold->translation = activation_pose.translation();
  hold->orientation = Eigen::Quaterniond(activation_pose.rotation());
  desired_state_buffer_.writeFromNonRT(hold);

  // Seed a degenerate (stationary) reference trajectory at the activation pose,
  // and mark `hold` as already "planned" so update() doesn't immediately replan
  // against it.
  traj_trans_c0_ = activation_pose.translation();
  traj_trans_c1_ = traj_trans_c2_ = traj_trans_c3_ = Eigen::Vector3d::Zero();
  traj_rot_c0_ = traj_rot_c1_ = traj_rot_c2_ = traj_rot_c3_ =
      Eigen::Vector3d::Zero();
  traj_orientation_anchor_ = Eigen::Quaterniond(activation_pose.rotation());
  traj_start_time_ = get_node()->now();
  traj_duration_ = *trajectory_duration_buffer_.readFromRT();
  last_planned_desired_ = hold;

  previous_raw_desired_.reset();
  playback_started_ = false;
  decel_planned_ = false;
  last_command_stamp_buffer_.writeFromNonRT(get_node()->now());

  {
    std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
    raw_command_queue_.clear();
  }

  vel_traj_c0_ = Eigen::Matrix<double, 6, 1>::Zero();
  vel_traj_c1_ = Eigen::Matrix<double, 6, 1>::Zero();
  vel_traj_start_time_ = get_node()->now();
  vel_traj_duration_ = 0.0;
  vel_playback_started_ = false;
  vel_stale_reset_done_ = false;

  previous_raw_velocity_.reset();
  {
    std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
    raw_velocity_queue_.clear();
  }

  return CallbackReturn::SUCCESS;
}

void CartesianPositionController::commandCallback(
    const inverse_msgs::msg::CartesianTrajectoryPoint::SharedPtr msg) {
  auto desired = std::make_shared<RawCommand>();
  desired->translation = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  desired->orientation =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z)
          .normalized();
  desired->twist << msg->velocity.linear.x, msg->velocity.linear.y,
      msg->velocity.linear.z, msg->velocity.angular.x, msg->velocity.angular.y,
      msg->velocity.angular.z;

  static uint64_t sequence = 0;
  desired->sequence = sequence++;

  // Trajectory time this sample's pose/twist were evaluated at by the
  // publisher, NOT when it was received -- see RawCommand::trajectory_time and
  // the class doc comment's "position_velocity" section.
  desired->trajectory_time =
      static_cast<double>(msg->time_from_start.sec) +
      1e-9 * static_cast<double>(msg->time_from_start.nanosec);

  desired_state_buffer_.writeFromNonRT(desired);
  last_command_stamp_buffer_.writeFromNonRT(get_node()->now());

  if (command_mode_ == "position_velocity") {
    // Every command is pushed directly onto the FIFO -- see the class doc
    // comment.
    std::lock_guard<std::mutex> lock(raw_command_queue_mutex_);
    raw_command_queue_.push_back(desired);

  } else if (command_mode_ == "velocity") {
    std::lock_guard<std::mutex> lock(raw_velocity_queue_mutex_);
    raw_velocity_queue_.push_back(desired);
  }
}

void CartesianPositionController::speedScalingCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  // speed_scaling_state_broadcaster publishes the factor as a 0-100 percentage.
  const double factor = std::clamp(msg->data / 100.0, 0.0, 1.0);
  speed_scaling_factor_buffer_.writeFromNonRT(factor);
}

rcl_interfaces::msg::SetParametersResult
CartesianPositionController::onParameterUpdate(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "jacobian_damping_max") {
      const double value = param.as_double();
      if (value < 0.0) {
        result.successful = false;
        result.reason = "jacobian_damping_max must be >= 0";
        return result;
      }
      jacobian_damping_max_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "singularity_threshold") {
      const double value = param.as_double();
      if (value <= 0.0) {
        result.successful = false;
        result.reason = "singularity_threshold must be > 0";
        return result;
      }
      singularity_threshold_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "command_timeout") {
      const double value = param.as_double();
      if (value < 0.0) {
        result.successful = false;
        result.reason = "command_timeout must be >= 0";
        return result;
      }
      command_timeout_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "trajectory_duration") {
      const double value = param.as_double();
      if (value <= 0.0) {
        result.successful = false;
        result.reason = "trajectory_duration must be > 0";
        return result;
      }
      trajectory_duration_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "use_speed_scaling") {
      use_speed_scaling_buffer_.writeFromNonRT(param.as_bool());
    } else if (param.get_name() == "P_gain") {
      const double value = param.as_double();
      if (value < 0.0) {
        result.successful = false;
        result.reason = "P_gain must be >= 0";
        return result;
      }
      p_gain_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "stale_deceleration_time") {
      const double value = param.as_double();
      if (value <= 0.0) {
        result.successful = false;
        result.reason = "stale_deceleration_time must be > 0";
        return result;
      }
      stale_deceleration_time_buffer_.writeFromNonRT(value);
    } else if (param.get_name() == "velocity_scale") {
      const double value = param.as_double();
      if (value <= 0.0 || value > 1.0) {
        result.successful = false;
        result.reason = "velocity_scale must be in (0, 1]";
        return result;
      }
      velocity_scale_buffer_.writeFromNonRT(value);
    }
  }
  return result;
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(CartesianPositionController,
                       controller_interface::ControllerInterface)

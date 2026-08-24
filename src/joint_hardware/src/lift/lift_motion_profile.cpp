#include "joint_hardware/lift/lift_motion_profile.hpp"

#include <algorithm>
#include <cmath>

namespace joint_hardware::lift
{

LiftMotionProfile::LiftMotionProfile(double cycle_sec)
: cycle_sec_(cycle_sec), otg_(cycle_sec)
{
  input_.synchronization = ruckig::Synchronization::None;
}

bool LiftMotionProfile::configure(const LiftMotionLimits & limits, std::string & error)
{
  if (!std::isfinite(limits.min_position_m) || !std::isfinite(limits.max_position_m) ||
    limits.min_position_m >= limits.max_position_m)
  {
    error = "position limits must be finite and ordered";
    return false;
  }
  if (!std::isfinite(limits.max_velocity_mps) || limits.max_velocity_mps <= 0.0 ||
    limits.max_velocity_mps > 0.060)
  {
    error = "max velocity must be in (0, 0.060] m/s";
    return false;
  }
  if (!std::isfinite(limits.max_acceleration_mps2) || limits.max_acceleration_mps2 <= 0.0 ||
    !std::isfinite(limits.max_jerk_mps3) || limits.max_jerk_mps3 <= 0.0 ||
    !std::isfinite(limits.default_velocity_scale) || limits.default_velocity_scale <= 0.0 ||
    limits.default_velocity_scale > 1.0)
  {
    error = "acceleration, jerk and velocity scale must be positive; scale must not exceed 1";
    return false;
  }
  limits_ = limits;
  configured_ = true;
  apply_limits(limits_.default_velocity_scale);
  error.clear();
  return true;
}

void LiftMotionProfile::reset(
  double position_m, double velocity_mps, double acceleration_mps2)
{
  const double position = std::clamp(position_m, limits_.min_position_m, limits_.max_position_m);
  input_.current_position[0] = position;
  input_.current_velocity[0] = std::clamp(
    velocity_mps, -limits_.max_velocity_mps, limits_.max_velocity_mps);
  input_.current_acceleration[0] = std::clamp(
    acceleration_mps2, -limits_.max_acceleration_mps2, limits_.max_acceleration_mps2);
  input_.target_position[0] = position;
  input_.target_velocity[0] = 0.0;
  input_.target_acceleration[0] = 0.0;
  input_.control_interface = ruckig::ControlInterface::Position;
  apply_limits(limits_.default_velocity_scale);
  otg_.reset();
  mode_ = LiftProfileMode::hold;
  state_ = {position, input_.current_velocity[0], input_.current_acceleration[0], 0.0, true, true};
}

bool LiftMotionProfile::set_position_target(
  double position_m, double target_velocity_mps, double target_acceleration_mps2,
  double velocity_scale, double acceleration_limit_mps2)
{
  if (!configured_ || !std::isfinite(position_m) || !std::isfinite(target_velocity_mps) ||
    !std::isfinite(target_acceleration_mps2) || !std::isfinite(acceleration_limit_mps2))
  {
    return false;
  }
  input_.control_interface = ruckig::ControlInterface::Position;
  input_.target_position[0] = std::clamp(
    position_m, limits_.min_position_m, limits_.max_position_m);
  input_.target_velocity[0] = std::clamp(
    target_velocity_mps, -limits_.max_velocity_mps, limits_.max_velocity_mps);
  input_.target_acceleration[0] = std::clamp(
    target_acceleration_mps2, -limits_.max_acceleration_mps2, limits_.max_acceleration_mps2);
  apply_limits(normalized_scale(velocity_scale));
  if (acceleration_limit_mps2 > 0.0) {
    input_.max_acceleration[0] = std::min(
      input_.max_acceleration[0], acceleration_limit_mps2);
  }
  otg_.reset();
  mode_ = LiftProfileMode::position;
  return true;
}

bool LiftMotionProfile::set_velocity_target(double velocity_mps, double velocity_scale)
{
  if (!configured_ || !std::isfinite(velocity_mps)) {
    return false;
  }
  const double scale = normalized_scale(velocity_scale);
  input_.control_interface = ruckig::ControlInterface::Velocity;
  apply_limits(scale);
  input_.target_velocity[0] = limit_aware_velocity(std::clamp(
      velocity_mps, -input_.max_velocity[0], input_.max_velocity[0]));
  input_.target_acceleration[0] = 0.0;
  otg_.reset();
  mode_ = LiftProfileMode::velocity;
  return true;
}

void LiftMotionProfile::request_soft_stop()
{
  input_.control_interface = ruckig::ControlInterface::Velocity;
  input_.target_velocity[0] = 0.0;
  input_.target_acceleration[0] = 0.0;
  apply_limits(limits_.default_velocity_scale);
  otg_.reset();
  mode_ = LiftProfileMode::soft_stop;
}

void LiftMotionProfile::hold(double position_m)
{
  reset(position_m, 0.0, 0.0);
}

LiftMotionSample LiftMotionProfile::update(double cycle_sec)
{
  if (!configured_ || !std::isfinite(cycle_sec) || cycle_sec <= 0.0) {
    state_.valid = false;
    return state_;
  }
  if (mode_ == LiftProfileMode::hold) {
    state_.finished = true;
    return state_;
  }
  cycle_sec_ = std::clamp(cycle_sec, 0.001, 0.1);
  otg_.delta_time = cycle_sec_;
  if (mode_ == LiftProfileMode::velocity) {
    input_.target_velocity[0] = limit_aware_velocity(input_.target_velocity[0]);
  }
  const auto result = otg_.update(input_, output_);
  if (result < ruckig::Result::Working) {
    state_.valid = false;
    return state_;
  }
  output_.pass_to_input(input_);
  state_.position_m = std::clamp(
    output_.new_position[0], limits_.min_position_m, limits_.max_position_m);
  state_.velocity_mps = output_.new_velocity[0];
  state_.acceleration_mps2 = output_.new_acceleration[0];
  state_.jerk_mps3 = output_.new_jerk[0];
  state_.finished = result == ruckig::Result::Finished;
  state_.valid = std::isfinite(state_.position_m) && std::isfinite(state_.velocity_mps) &&
    std::isfinite(state_.acceleration_mps2) && std::isfinite(state_.jerk_mps3);

  if (state_.position_m <= limits_.min_position_m && state_.velocity_mps < 0.0) {
    state_.velocity_mps = 0.0;
    state_.acceleration_mps2 = 0.0;
    reset(limits_.min_position_m);
  } else if (state_.position_m >= limits_.max_position_m && state_.velocity_mps > 0.0) {
    state_.velocity_mps = 0.0;
    state_.acceleration_mps2 = 0.0;
    reset(limits_.max_position_m);
  }
  return state_;
}

double LiftMotionProfile::normalized_scale(double requested) const noexcept
{
  if (!std::isfinite(requested) || requested <= 0.0) {
    return limits_.default_velocity_scale;
  }
  return std::clamp(requested, 0.01, 1.0);
}

double LiftMotionProfile::limit_aware_velocity(double requested) const noexcept
{
  if (requested > 0.0) {
    const double distance = std::max(0.0, limits_.max_position_m - input_.current_position[0]);
    const double braking_limit = std::sqrt(2.0 * limits_.max_acceleration_mps2 * distance);
    return std::min(requested, braking_limit);
  }
  if (requested < 0.0) {
    const double distance = std::max(0.0, input_.current_position[0] - limits_.min_position_m);
    const double braking_limit = std::sqrt(2.0 * limits_.max_acceleration_mps2 * distance);
    return std::max(requested, -braking_limit);
  }
  return 0.0;
}

void LiftMotionProfile::apply_limits(double velocity_scale)
{
  const double scale = normalized_scale(velocity_scale);
  input_.max_velocity[0] = limits_.max_velocity_mps * scale;
  input_.max_acceleration[0] = limits_.max_acceleration_mps2;
  input_.max_jerk[0] = limits_.max_jerk_mps3;
}

}  // namespace joint_hardware::lift

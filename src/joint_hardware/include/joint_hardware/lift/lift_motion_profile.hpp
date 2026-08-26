#ifndef JOINT_HARDWARE__LIFT__LIFT_MOTION_PROFILE_HPP_
#define JOINT_HARDWARE__LIFT__LIFT_MOTION_PROFILE_HPP_

#include <string>

#include "ruckig/ruckig.hpp"

namespace joint_hardware::lift
{

struct LiftMotionLimits
{
  double min_position_m{-1.0};
  double max_position_m{0.0};
  double max_velocity_mps{0.020};
  double max_acceleration_mps2{0.033333333};
  double max_jerk_mps3{0.4};
  double default_velocity_scale{0.80};
};

struct LiftMotionSample
{
  double position_m{0.0};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double jerk_mps3{0.0};
  bool finished{true};
  bool valid{true};
};

enum class LiftProfileMode
{
  hold,
  position,
  velocity,
  soft_stop,
};

class LiftMotionProfile
{
public:
  explicit LiftMotionProfile(double cycle_sec = 0.01);

  bool configure(const LiftMotionLimits & limits, std::string & error);
  void reset(double position_m, double velocity_mps = 0.0, double acceleration_mps2 = 0.0);
  bool set_position_target(
    double position_m, double target_velocity_mps = 0.0,
    double target_acceleration_mps2 = 0.0, double velocity_scale = 0.0,
    double acceleration_limit_mps2 = 0.0);
  bool set_velocity_target(double velocity_mps, double velocity_scale = 0.0);
  void request_soft_stop();
  void hold(double position_m);
  LiftMotionSample update(double cycle_sec);

  LiftProfileMode mode() const noexcept {return mode_;}
  const LiftMotionLimits & limits() const noexcept {return limits_;}
  const LiftMotionSample & state() const noexcept {return state_;}
  double target_position_m() const noexcept {return input_.target_position[0];}
  double target_velocity_mps() const noexcept {return input_.target_velocity[0];}

private:
  double normalized_scale(double requested) const noexcept;
  double limit_aware_velocity(double requested) const noexcept;
  void apply_limits(double velocity_scale);

  double cycle_sec_{0.01};
  LiftMotionLimits limits_{};
  LiftMotionSample state_{};
  LiftProfileMode mode_{LiftProfileMode::hold};
  ruckig::Ruckig<1> otg_;
  ruckig::InputParameter<1> input_;
  ruckig::OutputParameter<1> output_;
  bool configured_{false};
};

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__LIFT_MOTION_PROFILE_HPP_

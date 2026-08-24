#include <algorithm>
#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "joint_hardware/lift/lift_motion_profile.hpp"

namespace
{

joint_hardware::lift::LiftMotionProfile configured_profile()
{
  joint_hardware::lift::LiftMotionProfile profile(0.01);
  joint_hardware::lift::LiftMotionLimits limits;
  std::string error;
  EXPECT_TRUE(profile.configure(limits, error)) << error;
  profile.reset(0.0);
  return profile;
}

TEST(LiftMotionProfile, RejectsLimitsAboveMechanicalVelocity)
{
  joint_hardware::lift::LiftMotionProfile profile;
  joint_hardware::lift::LiftMotionLimits limits;
  limits.max_velocity_mps = 0.061;
  std::string error;
  EXPECT_FALSE(profile.configure(limits, error));
}

TEST(LiftMotionProfile, PositionProfileRespectsVelocityAccelerationAndJerk)
{
  auto profile = configured_profile();
  ASSERT_TRUE(profile.set_position_target(-0.10));
  double previous_acceleration = 0.0;
  for (int cycle = 0; cycle < 500; ++cycle) {
    const auto sample = profile.update(0.01);
    ASSERT_TRUE(sample.valid);
    EXPECT_LE(std::abs(sample.velocity_mps), 0.060 * 0.80 + 1.0e-8);
    EXPECT_LE(std::abs(sample.acceleration_mps2), 0.10 + 1.0e-8);
    EXPECT_LE(std::abs(sample.jerk_mps3), 0.4 + 1.0e-6);
    if (cycle > 0) {
      EXPECT_LE(std::abs(sample.acceleration_mps2 - previous_acceleration), 0.004 + 1.0e-6);
    }
    previous_acceleration = sample.acceleration_mps2;
  }
}

TEST(LiftMotionProfile, DefaultVelocityScaleIsEightyPercent)
{
  auto profile = configured_profile();
  ASSERT_TRUE(profile.set_position_target(-0.5));
  double peak_velocity = 0.0;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    peak_velocity = std::max(peak_velocity, std::abs(profile.update(0.01).velocity_mps));
  }
  EXPECT_LE(peak_velocity, 0.048 + 1.0e-8);
  EXPECT_GT(peak_velocity, 0.040);
}

TEST(LiftMotionProfile, JogCannotDriveBeyondLowerSoftwareLimit)
{
  auto profile = configured_profile();
  profile.reset(-0.999);
  ASSERT_TRUE(profile.set_velocity_target(-0.060, 1.0));
  for (int cycle = 0; cycle < 500; ++cycle) {
    const auto sample = profile.update(0.01);
    EXPECT_GE(sample.position_m, -1.0);
  }
  EXPECT_GE(profile.state().position_m, -1.0);
  EXPECT_NEAR(profile.state().velocity_mps, 0.0, 1.0e-9);
}

TEST(LiftMotionProfile, JogCannotDriveBeyondUpperSoftwareLimit)
{
  auto profile = configured_profile();
  profile.reset(-0.001);
  ASSERT_TRUE(profile.set_velocity_target(0.060, 1.0));
  for (int cycle = 0; cycle < 500; ++cycle) {
    const auto sample = profile.update(0.01);
    EXPECT_LE(sample.position_m, 0.0);
  }
  EXPECT_LE(profile.state().position_m, 0.0);
  EXPECT_NEAR(profile.state().velocity_mps, 0.0, 1.0e-9);
}

TEST(LiftMotionProfile, SoftStopUsesAccelerationAndJerkLimits)
{
  auto profile = configured_profile();
  profile.reset(-0.5);
  ASSERT_TRUE(profile.set_velocity_target(0.040, 1.0));
  for (int cycle = 0; cycle < 200; ++cycle) {
    (void)profile.update(0.01);
  }
  const double velocity_before = profile.state().velocity_mps;
  ASSERT_GT(velocity_before, 0.01);
  profile.request_soft_stop();
  const auto first = profile.update(0.01);
  EXPECT_GT(first.velocity_mps, 0.0);
  EXPECT_LE(std::abs(first.acceleration_mps2), 0.10 + 1.0e-8);
  EXPECT_LE(std::abs(first.jerk_mps3), 0.4 + 1.0e-6);
  for (int cycle = 0; cycle < 500; ++cycle) {
    (void)profile.update(0.01);
  }
  EXPECT_NEAR(profile.state().velocity_mps, 0.0, 1.0e-6);
}

TEST(LiftMotionProfile, SixHundredRpmPerSecondEqualsPointOneMetresPerSecondSquared)
{
  constexpr double lead_mm_per_rev = 10.0;
  constexpr double slew_rpm_per_sec = 600.0;
  const double acceleration_mps2 = slew_rpm_per_sec * lead_mm_per_rev / 60000.0;
  EXPECT_DOUBLE_EQ(acceleration_mps2, 0.10);
}

TEST(LiftMotionProfile, PositionTargetIsClampedBeforeRuckig)
{
  auto profile = configured_profile();
  ASSERT_TRUE(profile.set_position_target(-2.0));
  EXPECT_DOUBLE_EQ(profile.target_position_m(), -1.0);
  ASSERT_TRUE(profile.set_position_target(1.0));
  EXPECT_DOUBLE_EQ(profile.target_position_m(), 0.0);
}

TEST(LiftMotionProfile, PerCommandAccelerationCanOnlyTightenGlobalLimit)
{
  auto profile = configured_profile();
  profile.reset(-0.5);
  ASSERT_TRUE(profile.set_position_target(-0.2, 0.0, 0.0, 1.0, 0.03));
  for (int cycle = 0; cycle < 300; ++cycle) {
    const auto sample = profile.update(0.01);
    ASSERT_TRUE(sample.valid);
    EXPECT_LE(std::abs(sample.acceleration_mps2), 0.03 + 1.0e-9);
  }

  profile.reset(-0.5);
  ASSERT_TRUE(profile.set_position_target(-0.2, 0.0, 0.0, 1.0, 1.0));
  for (int cycle = 0; cycle < 300; ++cycle) {
    const auto sample = profile.update(0.01);
    ASSERT_TRUE(sample.valid);
    EXPECT_LE(std::abs(sample.acceleration_mps2), 0.10 + 1.0e-9);
  }
}

}  // namespace

#include <gtest/gtest.h>

#include "joint_hardware/lift/lift_units.hpp"

namespace
{

TEST(LiftUnits, UsesValidatedFeedUnitsAndSign)
{
  joint_hardware::lift::LiftUnitConfig config;
  std::string error;
  ASSERT_TRUE(joint_hardware::lift::validate_lift_units(config, error)) << error;
  EXPECT_NEAR(
    joint_hardware::lift::position_units_to_m(10000, 0, config), -0.003333333333, 1e-12);
  EXPECT_NEAR(
    joint_hardware::lift::velocity_units_to_mps(10000, config), -0.003333333333, 1e-12);
  EXPECT_EQ(joint_hardware::lift::wire_rpm_to_velocity_units(360.0, config), 60000);
  EXPECT_NEAR(joint_hardware::lift::max_velocity_mps(360.0, config), 0.020, 1e-12);
}

TEST(LiftUnits, ConvertsGearedEffectiveCarriageTravel)
{
  joint_hardware::lift::LiftUnitConfig config;
  config.lead_mm_per_rev = 10.0 / 3.0;

  EXPECT_NEAR(
    joint_hardware::lift::position_units_to_m(10000, 0, config), -0.003333333333, 1e-12);
  EXPECT_NEAR(joint_hardware::lift::max_velocity_mps(360.0, config), 0.020, 1e-12);
}

TEST(LiftUnits, RejectsUnverifiedFeedRatio)
{
  joint_hardware::lift::LiftUnitConfig config;
  config.feed_units_numerator = 20000;
  std::string error;
  EXPECT_FALSE(joint_hardware::lift::validate_lift_units(config, error));
  EXPECT_NE(error.find("6092"), std::string::npos);
}

TEST(LiftUnits, AppliesManualGearRatioOnlyWhenFeedEqualsEncoder)
{
  double units = 0.0;
  std::string error;
  ASSERT_TRUE(joint_hardware::lift::derive_command_units_per_rev(
    131072, 2, 1, 131072, 0, units, error)) << error;
  EXPECT_DOUBLE_EQ(units, 65536.0);

  ASSERT_TRUE(joint_hardware::lift::derive_command_units_per_rev(
    131072, 2, 1, 10000, 0, units, error)) << error;
  EXPECT_DOUBLE_EQ(units, 10000.0);

  ASSERT_TRUE(joint_hardware::lift::derive_command_units_per_rev(
    131072, 2, 1, 10000, 20000, units, error)) << error;
  EXPECT_DOUBLE_EQ(units, 20000.0);
}

TEST(LiftUnits, RejectsNonIntegralFeedObject)
{
  joint_hardware::lift::LiftUnitConfig config;
  config.feed_units_denominator = 2;
  std::string error;
  EXPECT_FALSE(joint_hardware::lift::validate_lift_units(config, error));
  EXPECT_NE(error.find("subindex"), std::string::npos);
}

}  // namespace

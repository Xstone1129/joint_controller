#include <gtest/gtest.h>

#include "joint_hardware/lift/cia402.hpp"

namespace
{

using joint_hardware::lift::Cia402Controller;
using joint_hardware::lift::Cia402Inputs;
using joint_hardware::lift::Cia402State;

Cia402Inputs healthy(uint16_t status)
{
  Cia402Inputs inputs;
  inputs.status_word = status;
  inputs.mode_display = 9;
  inputs.link_operational = true;
  inputs.pdo_fresh = true;
  inputs.working_counter_ok = true;
  return inputs;
}

TEST(Cia402, ParsesAndEnablesSequence)
{
  EXPECT_EQ(joint_hardware::lift::parse_cia402_status(0x0040), Cia402State::switch_on_disabled);
  EXPECT_EQ(joint_hardware::lift::parse_cia402_status(0x0021), Cia402State::ready_to_switch_on);
  EXPECT_EQ(joint_hardware::lift::parse_cia402_status(0x0023), Cia402State::switched_on);
  EXPECT_EQ(joint_hardware::lift::parse_cia402_status(0x0027), Cia402State::operation_enabled);

  Cia402Controller controller(3);
  auto decision = controller.step(healthy(0x0040));
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);
  decision = controller.step(healthy(0x0021));
  EXPECT_EQ(decision.control_word, 0x0007);
  decision = controller.step(healthy(0x0023));
  EXPECT_EQ(decision.control_word, 0x000f);
  decision = controller.step(healthy(0x0027));
  EXPECT_TRUE(decision.allow_motion);
}

TEST(Cia402, DropoutForcesSafeStop)
{
  Cia402Controller controller;
  auto inputs = healthy(0x0027);
  auto decision = controller.step(inputs);
  ASSERT_TRUE(decision.allow_motion);
  inputs.link_operational = false;
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);

  inputs.link_operational = true;
  inputs.pdo_fresh = false;
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);

  inputs.pdo_fresh = true;
  inputs.working_counter_ok = false;
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);

  inputs.working_counter_ok = true;
  inputs.mode_display = 8;
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);
}

TEST(Cia402, AcceptsHomingModeWhenItIsExplicitlyExpected)
{
  Cia402Controller controller;
  auto inputs = healthy(0x0027);
  inputs.mode_display = 6;
  inputs.expected_mode = 6;
  const auto decision = controller.step(inputs);
  EXPECT_TRUE(decision.mode_ok);
  EXPECT_TRUE(decision.health_ok);
  EXPECT_TRUE(decision.allow_motion);
  EXPECT_EQ(decision.control_word, 0x000f);
}

TEST(Cia402, BrakeFalseRequestsControlledDisable)
{
  Cia402Controller controller;
  auto inputs = healthy(0x0027);
  inputs.request_enable = false;
  const auto decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_FALSE(decision.allow_motion);
}

TEST(Cia402, QuickStopClearsQuickStopBitBeforeDisable)
{
  Cia402Controller controller;
  auto inputs = healthy(0x0027);
  inputs.request_enable = false;
  inputs.request_quick_stop = true;
  const auto decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x000b);
  EXPECT_FALSE(decision.allow_motion);

  inputs.status_word = 0x0007;
  const auto active = controller.step(inputs);
  EXPECT_EQ(active.control_word, 0x000b);
  EXPECT_FALSE(active.allow_motion);
}

TEST(Cia402, FaultResetIsFinite)
{
  Cia402Controller controller(2, 2);
  auto inputs = healthy(0x0008);
  inputs.error_code = 0x2310;
  auto decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0080);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0080);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  decision = controller.step(inputs);
  EXPECT_EQ(decision.control_word, 0x0006);
  EXPECT_TRUE(decision.recovery_exhausted);
  EXPECT_FALSE(decision.allow_motion);
}

}  // namespace

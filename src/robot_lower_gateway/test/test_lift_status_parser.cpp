#include <gtest/gtest.h>

#include <string>
#include <limits>

#include "robot_lower_gateway/lift_status_parser.hpp"

namespace
{

const char * kDriverStatus =
  R"json({
  "feedback_fresh":true,
  "ethercat_operational":true,
  "working_counter":3,
  "working_counter_ok":true,
  "initialized":true,
  "cia402_state":"ready_to_switch_on",
  "status_word":1585,
  "error_code":0,
  "mode_display":9,
  "power_enable_command":true,
  "power_enabled":false,
  "brake_unlocked":false,
  "estop_latched":false,
  "motion_blocked":false,
  "quick_stop_active":false,
  "position":-0.00486,
  "velocity":0.0,
  "command_position":-0.00486,
  "command_velocity":0.0,
  "command_acceleration":0.1,
  "target_rpm":0.0,
  "target_velocity_units":0,
  "velocity_mode_active":false,
  "fault_reason":"none"
})json";

TEST(LiftStatusParser, PreservesRequestedAndActualPowerSeparately)
{
  robot_lower_gateway::LiftDriverStatus status;
  std::string error;
  ASSERT_TRUE(robot_lower_gateway::parseLiftDriverStatus(kDriverStatus, status, error)) << error;
  EXPECT_TRUE(status.command_enabled);
  EXPECT_FALSE(status.enabled);
  EXPECT_EQ(status.cia402_state, "ready_to_switch_on");
  EXPECT_EQ(status.working_counter, 3);
  EXPECT_DOUBLE_EQ(status.position, -0.00486);
  EXPECT_TRUE(status.pdo_target_available);
  EXPECT_EQ(status.target_velocity_units, 0);
  EXPECT_FALSE(robot_lower_gateway::liftPowerStateMatches(status, true));
  EXPECT_FALSE(robot_lower_gateway::liftPowerStateMatches(status, false));

  status.enabled = true;
  status.brake_unlocked = true;
  status.cia402_state = "operation_enabled";
  status.status_word = 5687;
  EXPECT_TRUE(robot_lower_gateway::liftPowerStateMatches(status, true));

  status.command_enabled = false;
  status.enabled = false;
  status.brake_unlocked = false;
  status.cia402_state = "ready_to_switch_on";
  status.status_word = 1585;
  EXPECT_TRUE(robot_lower_gateway::liftPowerStateMatches(status, false));
}

TEST(LiftStatusParser, AcceptsOlderDriverStatusWithoutPdoTargetDiagnostics)
{
  robot_lower_gateway::LiftDriverStatus status;
  std::string error;
  std::string older_driver = kDriverStatus;
  const auto first = older_driver.find("  \"target_rpm\":0.0,\n");
  ASSERT_NE(first, std::string::npos);
  older_driver.erase(first, std::string("  \"target_rpm\":0.0,\n").size());
  const auto second = older_driver.find("  \"target_velocity_units\":0,\n");
  ASSERT_NE(second, std::string::npos);
  older_driver.erase(second, std::string("  \"target_velocity_units\":0,\n").size());
  const auto third = older_driver.find("  \"velocity_mode_active\":false,\n");
  ASSERT_NE(third, std::string::npos);
  older_driver.erase(third, std::string("  \"velocity_mode_active\":false,\n").size());
  ASSERT_TRUE(robot_lower_gateway::parseLiftDriverStatus(older_driver, status, error)) << error;
  EXPECT_FALSE(status.pdo_target_available);
}

TEST(LiftStatusParser, RejectsMissingActualPowerFeedback)
{
  robot_lower_gateway::LiftDriverStatus status;
  std::string error;
  const std::string incomplete = R"json({"feedback_fresh":true})json";
  EXPECT_FALSE(robot_lower_gateway::parseLiftDriverStatus(incomplete, status, error));
  EXPECT_FALSE(error.empty());
}

TEST(LiftStatusParser, ParsesControllerState)
{
  robot_lower_gateway::LiftControlStatus status;
  std::string error;
  ASSERT_TRUE(
    robot_lower_gateway::parseLiftControlStatus(
      R"json({"mode":"hold","trajectory_active":false,"jog_active":false})json",
      status, error)) << error;
  EXPECT_EQ(status.mode, "hold");
  EXPECT_FALSE(status.trajectory_active);
  EXPECT_FALSE(status.jog_active);
}

TEST(LiftStatusParser, RejectsNonFinitePosition)
{
  robot_lower_gateway::LiftDriverStatus status;
  std::string error;
  std::string invalid = kDriverStatus;
  const auto position = invalid.find("-0.00486");
  ASSERT_NE(position, std::string::npos);
  invalid.replace(position, 8, "\"nan\"");
  EXPECT_FALSE(robot_lower_gateway::parseLiftDriverStatus(invalid, status, error));
  EXPECT_NE(error.find("position"), std::string::npos);
}

robot_lower_gateway::LiftDriverStatus healthyReadyStatus()
{
  robot_lower_gateway::LiftDriverStatus status;
  status.feedback_fresh = true;
  status.ethercat_operational = true;
  status.working_counter_ok = true;
  status.initialized = true;
  status.cia402_state = "ready_to_switch_on";
  status.status_word = 1585;
  status.mode_display = robot_lower_gateway::kExpectedLiftModeDisplay;
  return status;
}

TEST(LiftStatusSafety, ReadyToSwitchOnDoesNotRequireEnableOrBrakeRelease)
{
  auto status = healthyReadyStatus();
  status.enabled = false;
  status.brake_unlocked = false;
  status.motion_blocked = true;
  status.fault_reason = "CiA 402 fault or mode mismatch";

  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_FALSE(safety.fault);
  EXPECT_FALSE(safety.motion_blocked);
  EXPECT_TRUE(safety.fault_reason.empty());
}

TEST(LiftStatusSafety, RecoveredPdoFailureDoesNotRemainLatched)
{
  auto status = healthyReadyStatus();
  status.motion_blocked = true;
  status.fault_reason = "PDO exchange failed";

  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_FALSE(safety.fault);
  EXPECT_FALSE(safety.motion_blocked);
  EXPECT_TRUE(safety.fault_reason.empty());
}

TEST(LiftStatusSafety, CurrentPdoFailureRemainsFailClosed)
{
  auto status = healthyReadyStatus();
  status.working_counter_ok = false;
  status.motion_blocked = true;
  status.fault_reason = "PDO exchange failed";

  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_EQ(safety.fault_reason, "PDO exchange failed");
  std::string reason;
  EXPECT_FALSE(robot_lower_gateway::liftEnablePreconditionsMet(status, true, reason));
  EXPECT_NE(reason.find("WorkingCounter"), std::string::npos);
}

TEST(LiftStatusSafety, EnableRequiresRunningWorkspaceAndHealthyCurrentState)
{
  auto status = healthyReadyStatus();
  std::string reason;
  EXPECT_FALSE(robot_lower_gateway::liftEnablePreconditionsMet(status, false, reason));
  EXPECT_NE(reason.find("workspace RUNNING"), std::string::npos);

  EXPECT_TRUE(robot_lower_gateway::liftEnablePreconditionsMet(status, true, reason));
  EXPECT_TRUE(reason.empty());

  status.motion_blocked = true;
  status.fault_reason = "feedback coordinate jump exceeded max_feedback_jump_m";
  EXPECT_FALSE(robot_lower_gateway::liftEnablePreconditionsMet(status, true, reason));
  EXPECT_EQ(reason, status.fault_reason);
}

TEST(LiftStatusSafety, PowerConfirmationUsesMaskedCia402State)
{
  auto status = healthyReadyStatus();
  status.command_enabled = true;
  status.enabled = true;
  status.brake_unlocked = true;
  status.cia402_state = "operation_enabled";
  status.status_word = 5687;
  EXPECT_EQ(
    status.status_word & robot_lower_gateway::kCia402StateMask,
    robot_lower_gateway::kCia402OperationEnabled);
  EXPECT_TRUE(robot_lower_gateway::liftPowerStateMatches(status, true));

  status.status_word = 1585;
  EXPECT_FALSE(robot_lower_gateway::liftPowerStateMatches(status, true));
}

TEST(LiftStatusSafety, MotionRequiresConfirmedEnableAndBrakeRelease)
{
  auto status = healthyReadyStatus();
  std::string reason;
  EXPECT_FALSE(robot_lower_gateway::liftMotionPreconditionsMet(status, true, reason));
  EXPECT_NE(reason.find("Operation Enabled"), std::string::npos);

  status.command_enabled = true;
  status.enabled = true;
  status.brake_unlocked = true;
  status.cia402_state = "operation_enabled";
  status.status_word = 5687;
  EXPECT_TRUE(robot_lower_gateway::liftMotionPreconditionsMet(status, true, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(LiftCommandValidation, RequiresSingleFiniteTargetAndPositiveLimits)
{
  std::string reason;
  EXPECT_TRUE(robot_lower_gateway::liftCommandRequestValid(
      {"joint_motor"}, {-0.001}, 0.005, 0.05, "joint_motor", reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(robot_lower_gateway::liftCommandRequestValid(
      {"wrong"}, {-0.001}, 0.005, 0.05, "joint_motor", reason));
  EXPECT_FALSE(robot_lower_gateway::liftCommandRequestValid(
      {"joint_motor"}, {}, 0.005, 0.05, "joint_motor", reason));
  EXPECT_FALSE(robot_lower_gateway::liftCommandRequestValid(
      {"joint_motor"}, {std::numeric_limits<double>::infinity()}, 0.005, 0.05,
      "joint_motor", reason));
  EXPECT_FALSE(robot_lower_gateway::liftCommandRequestValid(
      {"joint_motor"}, {-0.001}, 0.0, 0.05, "joint_motor", reason));
  EXPECT_FALSE(robot_lower_gateway::liftCommandRequestValid(
      {"joint_motor"}, {-0.001}, 0.005, -0.05, "joint_motor", reason));
}

TEST(LiftStatusSafety, ReportsActualModeMismatchPrecisely)
{
  auto status = healthyReadyStatus();
  status.mode_display = 8;
  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_FALSE(safety.fault);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_NE(safety.fault_reason.find("expected_mode=9"), std::string::npos);
  EXPECT_NE(safety.fault_reason.find("actual_mode=8"), std::string::npos);
}

TEST(LiftStatusSafety, Cia402FaultBlocksWithSpecificDetails)
{
  auto status = healthyReadyStatus();
  status.cia402_state = "fault";
  status.status_word = 8;
  status.motion_blocked = true;
  status.fault_reason = "CiA 402 fault or mode mismatch";
  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_TRUE(safety.fault);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_NE(safety.fault_reason.find("state=fault"), std::string::npos);
  EXPECT_NE(safety.fault_reason.find("status_word=8"), std::string::npos);
  EXPECT_NE(safety.fault_reason.find("error_code=0"), std::string::npos);
  EXPECT_EQ(safety.fault_reason.find("or mode mismatch"), std::string::npos);
}

TEST(LiftStatusSafety, NonzeroErrorCodeBlocksEvenOutsideFaultState)
{
  auto status = healthyReadyStatus();
  status.error_code = 0x2310;
  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_TRUE(safety.fault);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_NE(safety.fault_reason.find("state=ready_to_switch_on"), std::string::npos);
  EXPECT_NE(safety.fault_reason.find("error_code=8976"), std::string::npos);
}

TEST(LiftStatusSafety, EstopQuickStopAndExplicitDriverLatchRemainBlocked)
{
  auto status = healthyReadyStatus();
  status.estop_latched = true;
  status.quick_stop_active = true;
  auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_NE(safety.fault_reason.find("emergency stop"), std::string::npos);
  EXPECT_NE(safety.fault_reason.find("quick stop"), std::string::npos);

  status.estop_latched = false;
  status.quick_stop_active = false;
  status.motion_blocked = true;
  status.fault_reason = "feedback coordinate jump exceeded max_feedback_jump_m";
  safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_TRUE(safety.motion_blocked);
  EXPECT_EQ(safety.fault_reason, status.fault_reason);
}

TEST(LiftStatusSafety, OperationEnabledWithoutFaultIsNotBlocked)
{
  auto status = healthyReadyStatus();
  status.cia402_state = "operation_enabled";
  status.status_word = 39;
  status.command_enabled = true;
  status.enabled = true;
  status.brake_unlocked = true;
  const auto safety = robot_lower_gateway::evaluateLiftSafety(status);
  EXPECT_FALSE(safety.fault);
  EXPECT_FALSE(safety.motion_blocked);
  EXPECT_TRUE(safety.fault_reason.empty());
}

}  // namespace

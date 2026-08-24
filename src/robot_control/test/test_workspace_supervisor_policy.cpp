#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "robot_control/workspace_supervisor_policy.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"

namespace
{
using ArmPowerStatus = robot_control_msg::msg::ArmPowerStatus;
using WorkspaceStatus = robot_control_msg::msg::WorkspaceStatus;
namespace policy = robot_control::workspace_supervisor_policy;

ArmPowerStatus make_power_status(bool enabled)
{
  ArmPowerStatus status;
  for (int index = 1; index <= 7; ++index) {
    status.joint_names.emplace_back("ljoint" + std::to_string(index));
  }
  for (int index = 1; index <= 7; ++index) {
    status.joint_names.emplace_back("rjoint" + std::to_string(index));
  }
  status.status_codes.assign(
    14, enabled ? ArmPowerStatus::ENABLED_STATUS : 33);
  status.enabled.assign(14, enabled);
  status.command_enabled = enabled;
  status.all_enabled = enabled;
  return status;
}

TEST(WorkspaceSupervisorPolicy, StartingRealWithAllDrivesDisabledCanBecomeRunning)
{
  const auto issues = policy::collect_startup_arm_power_issues(make_power_status(false), true);
  const auto result = policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, issues);

  EXPECT_TRUE(issues.empty());
  EXPECT_EQ(result.state, WorkspaceStatus::RUNNING);
  EXPECT_EQ(result.mode, WorkspaceStatus::REAL);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.message, "workspace status heartbeat");
}

TEST(WorkspaceSupervisorPolicy, StartingRealRejectsAnyEnabledDrive)
{
  auto power = make_power_status(false);
  power.enabled[4] = true;
  const auto issues = policy::collect_startup_arm_power_issues(power, true);
  const auto result = policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, issues);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_NE(issues.front().find("ljoint5"), std::string::npos);
  EXPECT_EQ(result.state, WorkspaceStatus::ERROR);
  EXPECT_EQ(result.mode, WorkspaceStatus::REAL);
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.message.find("confirmed power-off"), std::string::npos);
}

TEST(WorkspaceSupervisorPolicy, RunningRealRemainsHealthyWhilePoweredOff)
{
  const auto issues = policy::collect_startup_arm_power_issues(make_power_status(false), false);
  const auto result = policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, issues);

  EXPECT_EQ(result.state, WorkspaceStatus::RUNNING);
  EXPECT_EQ(result.mode, WorkspaceStatus::REAL);
  EXPECT_TRUE(result.accepted);
}

TEST(WorkspaceSupervisorPolicy, RunningRealRemainsHealthyWithFourteenEnabledDrives)
{
  const auto issues = policy::collect_startup_arm_power_issues(make_power_status(true), false);
  const auto result = policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, issues);

  EXPECT_TRUE(issues.empty());
  EXPECT_EQ(result.state, WorkspaceStatus::RUNNING);
  EXPECT_EQ(result.mode, WorkspaceStatus::REAL);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.message, "workspace status heartbeat");
}

TEST(WorkspaceSupervisorPolicy, RunningRealStillRejectsStaleHardwareFeedback)
{
  const std::vector<std::string> issues{"REAL hardware feedback stale: age=1200ms"};
  const auto result = policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, issues);

  EXPECT_EQ(result.state, WorkspaceStatus::ERROR);
  EXPECT_EQ(result.mode, WorkspaceStatus::REAL);
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.message.find("feedback stale"), std::string::npos);
}

TEST(WorkspaceSupervisorPolicy, StopCompletesOnlyInStoppedUnknownState)
{
  const auto disabled = make_power_status(false);
  EXPECT_TRUE(policy::collect_startup_arm_power_issues(disabled, true).empty());

  const auto result = policy::evaluate_workspace_health(false, WorkspaceStatus::REAL, {});
  EXPECT_EQ(result.state, WorkspaceStatus::STOPPED);
  EXPECT_EQ(result.mode, WorkspaceStatus::UNKNOWN);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.message, "workspace status heartbeat");
}

TEST(WorkspaceSupervisorPolicy, NewStartReappliesPowerOffGate)
{
  const auto enabled = make_power_status(true);
  EXPECT_TRUE(policy::collect_startup_arm_power_issues(enabled, false).empty());

  const auto new_start_issues = policy::collect_startup_arm_power_issues(enabled, true);
  const auto result =
    policy::evaluate_workspace_health(true, WorkspaceStatus::REAL, new_start_issues);
  EXPECT_FALSE(new_start_issues.empty());
  EXPECT_EQ(result.state, WorkspaceStatus::ERROR);
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.message.find("confirmed power-off"), std::string::npos);
}
}  // namespace

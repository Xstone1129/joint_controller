#include <string>
#include <utility>
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
TEST(WorkspaceSupervisorPolicy, ArmFeedbackArrayIssueAcceptsFourteenAxes)
{
  EXPECT_TRUE(policy::arm_feedback_array_issue(make_power_status(false)).empty());
  EXPECT_TRUE(policy::collect_arm_drive_issues(make_power_status(false)).empty());
}

TEST(WorkspaceSupervisorPolicy, ArmFeedbackArrayIssueRejectsPartialArrays)
{
  auto power = make_power_status(false);
  power.joint_names.pop_back();
  const auto issue = policy::arm_feedback_array_issue(power);

  EXPECT_NE(issue.find("invalid array sizes"), std::string::npos);
  EXPECT_NE(issue.find("names=13"), std::string::npos);
  EXPECT_NE(issue.find("(expected 14 each)"), std::string::npos);
}

TEST(WorkspaceSupervisorPolicy, ArmDriveIssuesReportMappingMismatch)
{
  auto power = make_power_status(false);
  std::swap(power.joint_names[0], power.joint_names[1]);
  const auto issues = policy::collect_arm_drive_issues(power);

  ASSERT_EQ(issues.size(), 2U);
  EXPECT_NE(issues[0].find("mapping mismatch at index 0"), std::string::npos);
  EXPECT_NE(issues[0].find("got=ljoint2 expected=ljoint1"), std::string::npos);
  EXPECT_NE(issues[1].find("mapping mismatch at index 1"), std::string::npos);
}

TEST(WorkspaceSupervisorPolicy, ArmDriveIssuesAggregateUnavailableDrivesInOrder)
{
  auto power = make_power_status(false);
  power.status_codes[1] = 0;
  power.status_codes[8] = 0;
  const auto issues = policy::collect_arm_drive_issues(power);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_NE(
    issues.front().find(
      "REAL EtherCAT drives unavailable: "
      "ljoint2(status=0/unavailable), rjoint2(status=0/unavailable)"),
    std::string::npos);
}
}  // namespace

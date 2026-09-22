#ifndef ROBOT_CONTROL__WORKSPACE_SUPERVISOR_POLICY_HPP_
#define ROBOT_CONTROL__WORKSPACE_SUPERVISOR_POLICY_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"

namespace robot_control::workspace_supervisor_policy
{
// Number of arm axes the lower workspace must always report, in the fixed
// ljoint1..rjoint7 order.
constexpr std::size_t kArmAxisCount = 14;

inline const std::array<const char *, kArmAxisCount> & expected_arm_joint_names()
{
  static const std::array<const char *, kArmAxisCount> names = {
    "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
    "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7",
  };
  return names;
}

// Returns an empty string when the per-axis arrays are well formed. A non-empty
// result is fatal for the sample: the arrays cannot be indexed safely, so the
// caller must stop checking this message.
inline std::string arm_feedback_array_issue(
  const robot_control_msg::msg::ArmPowerStatus & status)
{
  if (status.joint_names.size() != kArmAxisCount ||
    status.status_codes.size() != kArmAxisCount ||
    status.enabled.size() != kArmAxisCount)
  {
    return "REAL drive feedback has invalid array sizes: names=" +
           std::to_string(status.joint_names.size()) + " status_codes=" +
           std::to_string(status.status_codes.size()) + " enabled=" +
           std::to_string(status.enabled.size()) + " (expected 14 each)";
  }
  return {};
}

// Per-drive issues that only depend on the message contents: joint names that
// do not match the expected order, followed by one aggregated issue listing the
// drives whose status code is 0 (unavailable). Requires the sizes to have been
// validated by arm_feedback_array_issue() first.
inline std::vector<std::string> collect_arm_drive_issues(
  const robot_control_msg::msg::ArmPowerStatus & status)
{
  const auto & expected_joint_names = expected_arm_joint_names();
  std::vector<std::string> issues;
  std::vector<std::string> unavailable_drives;
  for (std::size_t index = 0; index < expected_joint_names.size(); ++index) {
    if (status.joint_names[index] != expected_joint_names[index]) {
      issues.emplace_back(
        "REAL drive mapping mismatch at index " + std::to_string(index) + ": got=" +
        status.joint_names[index] + " expected=" + expected_joint_names[index]);
    }
    if (status.status_codes[index] == 0) {
      unavailable_drives.emplace_back(
        status.joint_names[index] + "(status=0/unavailable)");
    }
  }
  if (!unavailable_drives.empty()) {
    std::string names;
    for (std::size_t index = 0; index < unavailable_drives.size(); ++index) {
      if (index != 0U) {
        names += ", ";
      }
      names += unavailable_drives[index];
    }
    issues.emplace_back("REAL EtherCAT drives unavailable: " + names);
  }
  return issues;
}

inline std::vector<std::string> collect_startup_arm_power_issues(
  const robot_control_msg::msg::ArmPowerStatus & status,
  bool require_startup_power_off)
{
  if (!require_startup_power_off) {
    return {};
  }

  std::vector<std::string> enabled_drives;
  const auto count = std::min(status.joint_names.size(), status.enabled.size());
  for (std::size_t index = 0; index < count; ++index) {
    if (status.enabled[index]) {
      enabled_drives.emplace_back(status.joint_names[index]);
    }
  }

  if (!status.command_enabled && !status.all_enabled && enabled_drives.empty()) {
    return {};
  }

  std::string names;
  for (std::size_t index = 0; index < enabled_drives.size(); ++index) {
    if (index != 0U) {
      names += ", ";
    }
    names += enabled_drives[index];
  }
  return {
    "REAL startup is not in a confirmed power-off state: command_enabled=" +
    std::string(status.command_enabled ? "true" : "false") + " all_enabled=" +
    std::string(status.all_enabled ? "true" : "false") + " enabled_drives=[" + names + "]"};
}

inline robot_control_msg::msg::WorkspaceStatus evaluate_workspace_health(
  bool active, std::uint8_t mode, const std::vector<std::string> & issues,
  const std::string & healthy_message = "workspace status heartbeat")
{
  robot_control_msg::msg::WorkspaceStatus status;
  status.mode = active ? mode : robot_control_msg::msg::WorkspaceStatus::UNKNOWN;
  status.state = active ?
    (issues.empty() ? robot_control_msg::msg::WorkspaceStatus::RUNNING :
    robot_control_msg::msg::WorkspaceStatus::ERROR) :
    (issues.empty() ? robot_control_msg::msg::WorkspaceStatus::STOPPED :
    robot_control_msg::msg::WorkspaceStatus::ERROR);
  status.accepted = issues.empty();
  if (issues.empty()) {
    status.message = healthy_message;
  } else {
    status.message = "workspace unhealthy: ";
    for (std::size_t index = 0; index < issues.size(); ++index) {
      if (index != 0U) {
        status.message += "; ";
      }
      status.message += issues[index];
    }
  }
  return status;
}

}  // namespace robot_control::workspace_supervisor_policy

#endif  // ROBOT_CONTROL__WORKSPACE_SUPERVISOR_POLICY_HPP_

// Readiness collection for the workspace supervisor.
//
// "Ready" is deliberately expensive to prove: a stack is only accepted when the
// systemd unit is active, every required service server and publisher exists,
// the unique nodes appear exactly once, and (for REAL) the hardware feedback is
// fresh, complete and in the expected power state. Each failure is turned into
// a human readable issue string because that string is the only diagnostic the
// upper machine receives when a start request is rejected.

#include "robot_control/workspace_supervisor_ubuntu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robot_control/workspace_supervisor_policy.hpp"

using namespace std::chrono_literals;

namespace robot_control
{
using namespace workspace_supervisor;

void WorkspaceSupervisorUbuntu::begin_real_hardware_observation()
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(power_status_mutex_);
    has_power_status_ = false;
    last_power_status_ = ArmPowerStatus{};
    power_status_received_at_ = std::chrono::steady_clock::time_point{};
    real_lifecycle_started_at_ = now;
  }
  {
    std::lock_guard<std::mutex> lock(lift_status_mutex_);
    has_lift_status_ = false;
    last_lift_status_.clear();
    lift_status_received_at_ = std::chrono::steady_clock::time_point{};
  }
}

std::vector<std::string> WorkspaceSupervisorUbuntu::collect_lift_hardware_issues(
  bool require_startup_power_off)
{
  std::string status;
  std::chrono::steady_clock::time_point received_at;
  std::chrono::steady_clock::time_point lifecycle_started_at;
  bool received = false;
  {
    std::lock_guard<std::mutex> lock(lift_status_mutex_);
    received = has_lift_status_;
    status = last_lift_status_;
    received_at = lift_status_received_at_;
  }
  {
    std::lock_guard<std::mutex> lock(power_status_mutex_);
    lifecycle_started_at = real_lifecycle_started_at_;
  }

  std::vector<std::string> issues;
  if (!received || received_at < lifecycle_started_at) {
    issues.emplace_back("REAL lift feedback unavailable: no fresh driver_status sample");
    return issues;
  }
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - received_at);
  if (age > std::chrono::milliseconds(hardware_feedback_timeout_ms_)) {
    issues.emplace_back("REAL lift feedback stale: age=" + std::to_string(age.count()) + "ms");
    return issues;
  }

  if (!json_bool(status, "initialized")) {
    issues.emplace_back("REAL lift hardware is not initialized");
  }
  if (!json_bool(status, "ethercat_operational")) {
    issues.emplace_back("REAL lift EtherCAT link is not operational");
  }
  if (!json_bool(status, "feedback_fresh")) {
    issues.emplace_back("REAL lift PDO feedback is unavailable or stale");
  }
  if (!json_bool(status, "working_counter_ok")) {
    issues.emplace_back("REAL lift PDO working counter is incomplete");
  }
  const long status_word = json_integer(status, "status_word", 0);
  const long error_code = json_integer(status, "error_code", -1);
  const long mode = json_integer(status, "mode_display", -1);
  if (status_word == 0) {
    issues.emplace_back("REAL lift status_word=0/unavailable");
  }
  if (error_code != 0) {
    issues.emplace_back("REAL lift error_code=" + std::to_string(error_code));
  }
  if (mode != 9) {
    issues.emplace_back("REAL lift mode_display=" + std::to_string(mode) + " (expected 9)");
  }
  if (require_startup_power_off &&
    (json_bool(status, "power_enable_command") || json_bool(status, "power_enabled") ||
    status.find("\"cia402_state\":\"operation_enabled\"") != std::string::npos))
  {
    issues.emplace_back("REAL lift startup is not in a confirmed power-off state");
  }
  return issues;
}

std::vector<std::string> WorkspaceSupervisorUbuntu::collect_real_hardware_issues(
  bool require_startup_power_off)
{
  std::vector<std::string> issues;
  const auto driver_processes = collect_igh_driver_processes();
  if (driver_processes.size() != 1) {
    issues.emplace_back(
      "REAL EtherCAT backend process count=" + std::to_string(driver_processes.size()) +
      " (expected 1 IGH driver)");
  }

  ArmPowerStatus status;
  std::chrono::steady_clock::time_point received_at;
  std::chrono::steady_clock::time_point lifecycle_started_at;
  bool received = false;
  {
    std::lock_guard<std::mutex> lock(power_status_mutex_);
    received = has_power_status_;
    status = last_power_status_;
    received_at = power_status_received_at_;
    lifecycle_started_at = real_lifecycle_started_at_;
  }

  if (!received ||
    (lifecycle_started_at != std::chrono::steady_clock::time_point{} &&
    received_at < lifecycle_started_at))
  {
    issues.emplace_back("REAL hardware feedback unavailable: no fresh /arm/power_status sample");
    return issues;
  }

  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - received_at);
  if (age > std::chrono::milliseconds(hardware_feedback_timeout_ms_)) {
    issues.emplace_back(
      "REAL hardware feedback stale: age=" + std::to_string(age.count()) + "ms");
    return issues;
  }

  // Array validation and per-drive checks are pure functions of the message, so
  // they live in the policy header and are unit tested there.
  const auto array_issue =
    workspace_supervisor_policy::arm_feedback_array_issue(status);
  if (!array_issue.empty()) {
    issues.emplace_back(array_issue);
    return issues;
  }
  const auto drive_issues =
    workspace_supervisor_policy::collect_arm_drive_issues(status);
  issues.insert(issues.end(), drive_issues.begin(), drive_issues.end());

  const auto startup_power_issues =
    robot_control::workspace_supervisor_policy::collect_startup_arm_power_issues(
    status, require_startup_power_off);
  issues.insert(issues.end(), startup_power_issues.begin(), startup_power_issues.end());
  const auto lift_issues = collect_lift_hardware_issues(require_startup_power_off);
  issues.insert(issues.end(), lift_issues.begin(), lift_issues.end());
  return issues;
}

std::vector<std::string> WorkspaceSupervisorUbuntu::collect_readiness_issues(
  const char * unit, bool require_startup_power_off)
{
  std::vector<std::string> issues;
  if (!unit_is_active(unit)) {
    issues.emplace_back(std::string("unit inactive: ") + unit);
    return issues;
  }

  try {
    // The global service graph includes clients. Readiness requires actual
    // service servers, otherwise the always-on gateway clients can make an
    // incomplete stack look ready.
    const auto service_servers = collect_service_server_graph();
    const auto & services = service_servers.types;
    const auto require_service = [&services, &issues](
      const char * name, const char * expected_type)
      {
        const auto entry = services.find(name);
        if (entry == services.end()) {
          issues.emplace_back(std::string("missing service ") + name);
          return;
        }
        if (std::find(entry->second.begin(), entry->second.end(), expected_type) ==
          entry->second.end())
        {
          issues.emplace_back(std::string("wrong type for service ") + name);
        }
      };

    require_service("/set_arm_control_mode", "robot_control_msg/srv/SetArmControlMode");
    require_service("/set_robot_power", "robot_control_msg/srv/SetRobotPower");
    require_service("/arm_absolute_control", "robot_control_msg/srv/JointAbsoluteControl");
    require_service("/arm/joint_batch_control", "robot_control_msg/srv/JointBatchControl");
    require_service(
      "/cartesian_absolute_control", "robot_control_msg/srv/CartesianAbsoluteControl");
    require_service(
      "/cartesian_increment_control", "robot_control_msg/srv/CartesianIncrementControl");

    constexpr const char * required_topics[] = {
      "/arm/power_status",
      "/arm/control_mode_status",
      "/arm/joint_states",
      "/arm_tcp_pose",
      "/arm/arm_controller/motion_status",
      "/arm_cartesian_path_execution_status",
      "/arm/joint_batch_execution_status",
      "/joint/lift/driver_status",
    };
    for (const char * topic : required_topics) {
      if (count_publishers(topic) == 0) {
        issues.emplace_back(std::string("no publisher for ") + topic);
      }
    }

    const auto node_names = get_node_names();
    constexpr const char * unique_nodes[] = {
      "/cartesian_single_control_srv",
      "/cartesian_moveL_path",
      "/cartesian_path_absolute_control_srv",
      "/cartesian_path_increment_control_srv",
      "/lift/controller_manager",
      "/lift/lift_controller",
    };
    for (const char * node : unique_nodes) {
      const auto count = std::count(node_names.begin(), node_names.end(), node);
      if (count != 1) {
        issues.emplace_back(
          std::string("node count ") + node + "=" + std::to_string(count) + " (expected 1)");
      }
    }

    if (std::strcmp(unit, kRealStackUnit) == 0) {
      const auto hardware_issues = collect_real_hardware_issues(require_startup_power_off);
      issues.insert(issues.end(), hardware_issues.begin(), hardware_issues.end());
    }
  } catch (const std::exception & error) {
    issues.emplace_back(std::string("ROS graph query failed: ") + error.what());
  }
  return issues;
}

std::vector<std::string> WorkspaceSupervisorUbuntu::collect_unmanaged_stack_issues()
{
  std::vector<std::string> issues;
  try {
    // The global service graph includes clients. Query each node's server
    // endpoints so the always-on gateway does not look like an orphaned
    // native control stack while only its clients are present.
    const auto service_servers = collect_service_server_graph();
    constexpr const char * native_services[] = {
      "/set_robot_power",
      "/set_arm_control_mode",
      "/arm_absolute_control",
      "/arm/joint_batch_control",
      "/cartesian_absolute_control",
      "/cartesian_increment_control",
    };
    for (const char * service : native_services) {
      const auto entry = service_servers.types.find(service);
      if (entry != service_servers.types.end()) {
        std::string issue = std::string("native service server remains visible: ") + service;
        const auto owners = service_servers.owners.find(service);
        if (owners != service_servers.owners.end() && !owners->second.empty()) {
          issue += " (owner=" + join(owners->second) + ")";
        }
        issues.emplace_back(issue);
      }
    }

    const auto node_names = get_node_names();
    constexpr const char * control_nodes[] = {
      "/arm/controller_manager",
      "/arm/erobot_controller_arm",
      "/arm/command_processor",
      "/lift/controller_manager",
      "/lift/lift_controller",
    };
    for (const char * node : control_nodes) {
      if (std::find(node_names.begin(), node_names.end(), node) != node_names.end()) {
        issues.emplace_back(std::string("control node remains visible: ") + node);
      }
    }

    if (count_publishers("/arm/power_status") != 0) {
      issues.emplace_back("power feedback publisher remains visible");
    }
    if (count_publishers("/joint/lift/driver_status") != 0) {
      issues.emplace_back("lift driver feedback publisher remains visible");
    }

    for (const auto & process : collect_managed_stack_processes()) {
      issues.emplace_back(describe_process(process));
    }
  } catch (const std::exception & error) {
    issues.emplace_back(std::string("ROS graph query failed: ") + error.what());
  }
  return issues;
}

std::vector<std::string> WorkspaceSupervisorUbuntu::collect_stopped_issues()
{
  std::vector<std::string> issues;
  constexpr const char * managed_units[] = {
    kSimulationStackUnit,
    kRealStackUnit,
    kLegacyStackUnit,
    kBootRobotUnit,
    kRvizUnit,
  };
  for (const char * unit : managed_units) {
    if (unit_is_active(unit)) {
      issues.emplace_back(std::string("active unit: ") + unit);
    } else if (unit_is_failed(unit)) {
      issues.emplace_back(std::string("failed unit: ") + unit);
    }
  }

  auto unmanaged = collect_unmanaged_stack_issues();
  issues.insert(issues.end(), unmanaged.begin(), unmanaged.end());
  return issues;
}

bool WorkspaceSupervisorUbuntu::wait_for_stack_stopped(
  std::vector<std::string> & final_issues)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(cleanup_timeout_ms_);
  const auto required_stability = std::chrono::milliseconds(cleanup_stability_ms_);
  auto clean_since = std::chrono::steady_clock::time_point{};
  auto next_log = std::chrono::steady_clock::now();

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    final_issues = collect_stopped_issues();
    const auto now = std::chrono::steady_clock::now();
    if (final_issues.empty()) {
      if (clean_since == std::chrono::steady_clock::time_point{}) {
        clean_since = now;
      }
      if (now - clean_since >= required_stability) {
        final_issues = collect_stopped_issues();
        return final_issues.empty();
      }
    } else {
      clean_since = std::chrono::steady_clock::time_point{};
    }

    if (now >= next_log) {
      RCLCPP_INFO(
        get_logger(), "Waiting for workspace cleanup/stability: %s",
        final_issues.empty() ? "units, processes and ROS graph clean; verifying stability" :
        join(final_issues).c_str());
      next_log = now + 2s;
    }
    std::this_thread::sleep_for(250ms);
  }
  final_issues = collect_stopped_issues();
  if (final_issues.empty()) {
    final_issues.emplace_back(
      "cleanup did not remain continuously clean for " +
      std::to_string(cleanup_stability_ms_) + "ms before timeout");
  }
  return false;
}

bool WorkspaceSupervisorUbuntu::wait_for_stack_ready(
  const char * unit, std::vector<std::string> & final_issues)
{
  const auto wait_started_at = std::chrono::steady_clock::now();
  const auto deadline = wait_started_at +
    std::chrono::milliseconds(startup_timeout_ms_);
  const auto required_stability = std::chrono::milliseconds(readiness_stability_ms_);
  auto next_log = std::chrono::steady_clock::now();
  auto ready_since = std::chrono::steady_clock::time_point{};
  bool real_hardware_observation_logged = false;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    final_issues = collect_readiness_issues(unit, true);
    if (final_issues.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (ready_since == std::chrono::steady_clock::time_point{}) {
        ready_since = now;
      }
      if (now - ready_since >= required_stability) {
        final_issues = collect_readiness_issues(unit, true);
        return final_issues.empty() && unit_is_active(unit);
      }
    } else {
      ready_since = std::chrono::steady_clock::time_point{};
    }
    if (std::strcmp(unit, kRealStackUnit) == 0 &&
      std::chrono::steady_clock::now() - wait_started_at >=
      std::chrono::milliseconds(real_hardware_readiness_timeout_ms_))
    {
      const auto hardware_issues = collect_real_hardware_issues(true);
      if (!hardware_issues.empty() && !real_hardware_observation_logged) {
        // EtherCAT can legitimately need longer than the initial observation
        // window to finish DC synchronization and establish complete PDO WKC.
        // This is diagnostic only: acceptance still requires all readiness
        // gates to remain healthy, while a real failure expires at the bounded
        // overall startup deadline below.
        RCLCPP_WARN(
          get_logger(),
          "REAL hardware is not ready after %d ms; continuing observation until "
          "the %d ms startup deadline: %s",
          real_hardware_readiness_timeout_ms_, startup_timeout_ms_,
          join(hardware_issues).c_str());
        real_hardware_observation_logged = true;
      }
    }
    if (!unit_is_active(unit)) {
      final_issues = collect_readiness_issues(unit, true);
      return false;
    }
    if (std::chrono::steady_clock::now() >= next_log) {
      RCLCPP_INFO(
        get_logger(), "Waiting for workspace stack readiness/stability: %s",
        final_issues.empty() ? "endpoints ready; verifying unit stability" :
        join(final_issues).c_str());
      next_log = std::chrono::steady_clock::now() + 2s;
    }
    std::this_thread::sleep_for(250ms);
  }
  final_issues = collect_readiness_issues(unit, true);
  if (final_issues.empty()) {
    final_issues.emplace_back(
      "workspace readiness did not remain stable for " +
      std::to_string(readiness_stability_ms_) + "ms before timeout");
  }
  return false;
}

}  // namespace robot_control

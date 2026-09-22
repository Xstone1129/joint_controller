// Workspace supervisor node (lower machine): ROS interface, the START/STOP
// command state machine and status publishing.
//
// Everything else this node needs is implemented by theme in the sibling
// translation units listed in workspace_supervisor_ubuntu.hpp.

#include "robot_control/workspace_supervisor_ubuntu.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "robot_control/workspace_supervisor_policy.hpp"

using namespace std::chrono_literals;

namespace robot_control
{
using namespace workspace_supervisor;

WorkspaceSupervisorUbuntu::WorkspaceSupervisorUbuntu()
: Node("workspace_supervisor_ubuntu")
{
  allowed_source_ = declare_parameter<std::string>(
    "allowed_source", "jetson_192_168_2_10");
  status_source_ = declare_parameter<std::string>(
    "status_source", kDefaultStatusSource);
  startup_timeout_ms_ = declare_parameter<int>("startup_timeout_ms", 45000);
  cleanup_timeout_ms_ = declare_parameter<int>("cleanup_timeout_ms", 30000);
  cleanup_stability_ms_ = declare_parameter<int>("cleanup_stability_ms", 1000);
  hardware_feedback_timeout_ms_ =
    declare_parameter<int>("hardware_feedback_timeout_ms", 1000);
  real_hardware_readiness_timeout_ms_ =
    declare_parameter<int>("real_hardware_readiness_timeout_ms", 15000);
  const auto declared_readiness_stability_ms =
    declare_parameter<int>("readiness_stability_ms", 3000);
  readiness_stability_ms_ = declared_readiness_stability_ms > 0 ?
    static_cast<int>(declared_readiness_stability_ms) : 0;
  cleanup_timeout_ms_ = std::max(cleanup_timeout_ms_, 1000);
  cleanup_stability_ms_ = std::max(cleanup_stability_ms_, 0);
  hardware_feedback_timeout_ms_ = std::max(hardware_feedback_timeout_ms_, 100);
  real_hardware_readiness_timeout_ms_ =
    std::max(real_hardware_readiness_timeout_ms_, 1000);

  const auto command_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto status_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  status_publisher_ = create_publisher<WorkspaceStatus>(
    "/workspace/status", status_qos);
  command_subscription_ = create_subscription<WorkspaceControl>(
    "/workspace/control", command_qos,
    [this](const WorkspaceControl::SharedPtr msg) {handle_command(msg);});
  power_status_subscription_ = create_subscription<ArmPowerStatus>(
    "/arm/power_status", status_qos,
    [this](const ArmPowerStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(power_status_mutex_);
      last_power_status_ = *msg;
      power_status_received_at_ = std::chrono::steady_clock::now();
      has_power_status_ = true;
    });
  lift_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10).reliable().durability_volatile(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(lift_status_mutex_);
      last_lift_status_ = msg->data;
      lift_status_received_at_ = std::chrono::steady_clock::now();
      has_lift_status_ = true;
    });

  const bool active_at_startup = any_stack_is_active();
  current_mode_.store(active_at_startup ? detect_active_mode() : WorkspaceStatus::UNKNOWN);
  if (active_at_startup) {
    const auto issues = collect_readiness_issues(unit_for_mode(current_mode_.load()));
    current_state_.store(issues.empty() ? WorkspaceStatus::RUNNING : WorkspaceStatus::ERROR);
    if (issues.empty() && current_mode_.load() == WorkspaceStatus::SIMULATION) {
      start_simulation_rviz();
    }
    publish_status(
      0, issues.empty(), issues.empty() ? "workspace supervisor ready" :
      "workspace supervisor found an unhealthy active stack: " + join(issues));
  } else {
    const auto unmanaged_issues = collect_unmanaged_stack_issues();
    current_state_.store(
      unmanaged_issues.empty() ? WorkspaceStatus::STOPPED : WorkspaceStatus::ERROR);
    publish_status(
      0, unmanaged_issues.empty(), unmanaged_issues.empty() ? "workspace supervisor ready" :
      "workspace supervisor found unmanaged control processes: " + join(unmanaged_issues));
  }

  status_timer_ = create_wall_timer(2s, [this]() {refresh_status();});

  RCLCPP_INFO(
    get_logger(),
    "Workspace supervisor ready; allowed_source='%s', unit='%s', "
    "startup_timeout=%dms, readiness_stability=%dms, cleanup_timeout=%dms, "
    "cleanup_stability=%dms, hardware_feedback_timeout=%dms, "
    "real_hardware_readiness_timeout=%dms",
    allowed_source_.c_str(), "joint-controller-stack-{sim,real}.service",
    startup_timeout_ms_, readiness_stability_ms_, cleanup_timeout_ms_,
    cleanup_stability_ms_, hardware_feedback_timeout_ms_,
    real_hardware_readiness_timeout_ms_);
}

WorkspaceSupervisorUbuntu::~WorkspaceSupervisorUbuntu()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

void WorkspaceSupervisorUbuntu::handle_command(const WorkspaceControl::SharedPtr msg)
{
  if (!allowed_source_.empty() && msg->source != allowed_source_) {
    RCLCPP_WARN(
      get_logger(), "Rejected command seq=%u from source='%s'",
      msg->seq, msg->source.c_str());
    publish_status(msg->seq, false, "source is not allowed");
    return;
  }

  if (msg->command == WorkspaceControl::STATUS) {
    if (!operation_in_progress_.load()) {
      const bool active = any_stack_is_active();
      current_mode_.store(active ? detect_active_mode() : WorkspaceStatus::UNKNOWN);
      if (active) {
        const auto issues = collect_readiness_issues(unit_for_mode(current_mode_.load()));
        current_state_.store(issues.empty() ? WorkspaceStatus::RUNNING : WorkspaceStatus::ERROR);
        publish_status(
          msg->seq, issues.empty(),
          issues.empty() ? "status requested" : "status requested; unhealthy: " + join(issues));
        return;
      }
      const auto unmanaged_issues = collect_unmanaged_stack_issues();
      if (!unmanaged_issues.empty()) {
        current_state_.store(WorkspaceStatus::ERROR);
        publish_status(
          msg->seq, false,
          "status requested; unmanaged control processes: " + join(unmanaged_issues));
        return;
      }
      current_state_.store(WorkspaceStatus::STOPPED);
    }
    publish_status(msg->seq, true, "status requested");
    return;
  }

  if (msg->command != WorkspaceControl::START && msg->command != WorkspaceControl::STOP) {
    publish_status(msg->seq, false, "unsupported command");
    return;
  }

  if (msg->command == WorkspaceControl::START &&
      msg->mode != WorkspaceControl::SIMULATION && msg->mode != WorkspaceControl::REAL) {
    publish_status(msg->seq, false, "unsupported workspace mode");
    return;
  }

  std::lock_guard<std::mutex> lock(operation_mutex_);
  if (operation_in_progress_.load()) {
    publish_status(msg->seq, false, "another workspace operation is in progress");
    return;
  }

  const bool active = any_stack_is_active();
  const auto unmanaged_issues = active ? std::vector<std::string>{} :
    collect_unmanaged_stack_issues();
  if (msg->command == WorkspaceControl::START && !unmanaged_issues.empty()) {
    current_state_.store(WorkspaceStatus::ERROR);
    current_mode_.store(WorkspaceStatus::UNKNOWN);
    publish_status(
      msg->seq, false,
      "start rejected; unmanaged control processes detected: " + join(unmanaged_issues));
    return;
  }
  if (msg->command == WorkspaceControl::STOP && !active && unmanaged_issues.empty() &&
    !any_managed_unit_is_failed() && !unit_is_active(kRvizUnit))
  {
    stop_simulation_rviz();
    current_state_.store(
      unmanaged_issues.empty() ? WorkspaceStatus::STOPPED : WorkspaceStatus::ERROR);
    current_mode_.store(WorkspaceStatus::UNKNOWN);
    publish_status(
      msg->seq, unmanaged_issues.empty(), unmanaged_issues.empty() ?
      "workspace is already stopped" :
      "stop rejected; unmanaged control processes require local cleanup: " + join(unmanaged_issues));
    return;
  }

  if (worker_.joinable()) {
    worker_.join();
  }

  const bool starting = msg->command == WorkspaceControl::START;
  const std::string action = starting ? "start" : "stop";
  const std::uint8_t operation_mode = starting ? msg->mode : current_mode_.load();
  current_state_.store(starting ? WorkspaceStatus::STARTING : WorkspaceStatus::STOPPING);
  if (starting) {
    current_mode_.store(operation_mode);
  }
  operation_in_progress_.store(true);
  operation_command_seq_.store(msg->seq);
  publish_status(msg->seq, true, action + " accepted");

  worker_ = std::thread([this, action, command_seq = msg->seq, starting, operation_mode]() {
    const char * target_unit = unit_for_mode(operation_mode);
    RCLCPP_INFO(
      get_logger(), "Executing workspace %s mode=%s for command seq=%u",
      action.c_str(), operation_mode == WorkspaceStatus::REAL ? "REAL" : "SIMULATION", command_seq);

    int result = 0;
    std::vector<std::string> cleanup_issues;
    if (starting) {
      bool stopped_previous_stack = false;
      if (!unit_is_active(target_unit) && unit_is_failed(target_unit)) {
        result = run_systemctl("stop", target_unit);
      }
      if (operation_mode != WorkspaceStatus::SIMULATION) {
        stop_simulation_rviz();
      }
      if (result == 0 &&
        (unit_is_active(kBootRobotUnit) || unit_is_failed(kBootRobotUnit)))
      {
        result = run_systemctl("stop", kBootRobotUnit);
        stopped_previous_stack = true;
      }
      if (result == 0 && operation_mode != WorkspaceStatus::SIMULATION &&
        (unit_is_active(kSimulationStackUnit) || unit_is_failed(kSimulationStackUnit)))
      {
        result = run_systemctl("stop", kSimulationStackUnit);
        stopped_previous_stack = true;
      }
      if (result == 0 && operation_mode != WorkspaceStatus::REAL &&
        (unit_is_active(kRealStackUnit) || unit_is_failed(kRealStackUnit)))
      {
        result = run_systemctl("stop", kRealStackUnit);
        stopped_previous_stack = true;
      }
      if (result == 0 && (unit_is_active(kLegacyStackUnit) || unit_is_failed(kLegacyStackUnit))) {
        result = run_systemctl("stop", kLegacyStackUnit);
        stopped_previous_stack = true;
      }
      reset_failed_managed_units();
      if (result == 0 && stopped_previous_stack &&
        !wait_for_stack_stopped(cleanup_issues))
      {
        result = 125;
      }
      if (result == 0 && operation_mode == WorkspaceStatus::REAL) {
        begin_real_hardware_observation();
      }
      if (result == 0 && !unit_is_active(target_unit)) {
        result = run_systemctl("start", target_unit);
      }
    } else {
      stop_simulation_rviz();
      if (!any_stack_is_active() && !collect_unmanaged_stack_issues().empty()) {
        RCLCPP_WARN(
          get_logger(),
          "No managed stack unit is active; running local cleanup for unmanaged control processes");
        result = run_cleanup_script();
      }
      if (unit_is_active(kBootRobotUnit) || unit_is_failed(kBootRobotUnit)) {
        if (result == 0) {
          result = run_systemctl("stop", kBootRobotUnit);
        }
      }
      if (result == 0 &&
        (unit_is_active(kSimulationStackUnit) || unit_is_failed(kSimulationStackUnit)))
      {
        result = run_systemctl("stop", kSimulationStackUnit);
      }
      if (result == 0 && (unit_is_active(kRealStackUnit) || unit_is_failed(kRealStackUnit))) {
        result = run_systemctl("stop", kRealStackUnit);
      }
      if (result == 0 && (unit_is_active(kLegacyStackUnit) || unit_is_failed(kLegacyStackUnit))) {
        result = run_systemctl("stop", kLegacyStackUnit);
      }
      reset_failed_managed_units();
    }
    std::vector<std::string> readiness_issues;
    bool succeeded = false;
    if (starting && result == 0) {
      succeeded = wait_for_stack_ready(target_unit, readiness_issues);
      if (!succeeded) {
        const int stop_result =
          (unit_is_active(target_unit) || unit_is_failed(target_unit)) ?
          run_systemctl("stop", target_unit) : 0;
        if (stop_result != 0) {
          readiness_issues.emplace_back(
            "failed to stop incomplete stack: systemctl exit=" + std::to_string(stop_result));
        }
        reset_failed_managed_units();
        std::vector<std::string> failed_start_cleanup_issues;
        if (!wait_for_stack_stopped(failed_start_cleanup_issues)) {
          readiness_issues.emplace_back(
            "incomplete stack cleanup timed out: " + join(failed_start_cleanup_issues));
        }
      }
    } else if (!starting) {
      succeeded = result == 0 && wait_for_stack_stopped(cleanup_issues);
    }

    if (succeeded) {
      if (starting && operation_mode == WorkspaceStatus::SIMULATION) {
        start_simulation_rviz();
      }
      current_state_.store(starting ? WorkspaceStatus::RUNNING : WorkspaceStatus::STOPPED);
      current_mode_.store(starting ? operation_mode : WorkspaceStatus::UNKNOWN);
      publish_status(command_seq, true, action + " completed");
      RCLCPP_INFO(get_logger(), "Workspace %s completed", action.c_str());
    } else {
      current_state_.store(WorkspaceStatus::ERROR);
      current_mode_.store(starting ? operation_mode : WorkspaceStatus::UNKNOWN);
      std::string details;
      if (starting && !readiness_issues.empty()) {
        details = "; readiness: " + join(readiness_issues);
      }
      if (!cleanup_issues.empty()) {
        details += "; cleanup: " + join(cleanup_issues);
      }
      publish_status(
        command_seq, false,
        action + " failed (mode=" +
        std::string(operation_mode == WorkspaceStatus::REAL ? "REAL" : "SIMULATION") +
        ", systemctl exit=" + std::to_string(result) + ")" + details);
      RCLCPP_ERROR(
        get_logger(), "Workspace %s failed: systemctl exit=%d%s",
        action.c_str(), result, details.c_str());
    }
    operation_in_progress_.store(false);
  });
}

void WorkspaceSupervisorUbuntu::refresh_status()
{
  if (operation_in_progress_.load()) {
    publish_status(
      operation_command_seq_.load(), true,
      current_state_.load() == WorkspaceStatus::STARTING ?
      "start in progress" : "stop in progress");
    return;
  }

  const bool active = any_stack_is_active();
  std::vector<std::string> issues;
  const auto observed_mode = active ? detect_active_mode() : WorkspaceStatus::UNKNOWN;
  if (active) {
    issues = collect_readiness_issues(unit_for_mode(observed_mode));
  } else {
    issues = collect_unmanaged_stack_issues();
  }
  const auto previous_state = current_state_.load();
  const auto previous_mode = current_mode_.load();
  const bool initially_changed =
    (active ? WorkspaceStatus::RUNNING : WorkspaceStatus::STOPPED) != previous_state ||
    observed_mode != previous_mode;
  const auto evaluation =
    robot_control::workspace_supervisor_policy::evaluate_workspace_health(
    active, observed_mode, issues,
    initially_changed ? "workspace state changed outside supervisor" :
    "workspace status heartbeat");
  current_state_.store(evaluation.state);
  current_mode_.store(evaluation.mode);
  publish_status(
    0, evaluation.accepted, evaluation.message);
}

void WorkspaceSupervisorUbuntu::publish_status(
  std::uint32_t command_seq, bool accepted, const std::string & message)
{
  WorkspaceStatus status;
  status.stamp = this->now();
  status.command_seq = command_seq;
  status.state = current_state_.load();
  status.mode = current_mode_.load();
  status.accepted = accepted;
  status.message = message;
  status.source = status_source_;
  status_publisher_->publish(status);
}

}  // namespace robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_control::WorkspaceSupervisorUbuntu>());
  rclcpp::shutdown();
  return 0;
}

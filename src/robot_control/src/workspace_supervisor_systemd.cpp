// systemd unit control for the workspace supervisor.
//
// Everything here forks /usr/bin/systemctl and waits for it with a bounded
// timeout, plus the unit <-> workspace-mode mapping and the simulation RViz
// unit that is part of the simulation stack.

#include "robot_control/workspace_supervisor_ubuntu.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace robot_control
{
using namespace workspace_supervisor;

int WorkspaceSupervisorUbuntu::run_systemctl(const std::string & action, const char * unit) const
{
  const pid_t child = fork();
  if (child < 0) {
    RCLCPP_ERROR(get_logger(), "fork() failed: %s", std::strerror(errno));
    return -1;
  }

  if (child == 0) {
    if (action == "is-active" || action == "is-failed") {
      execl(kSystemctl, "systemctl", action.c_str(), "--quiet", unit, nullptr);
    } else if (action == "start" || action == "stop" || action == "reset-failed") {
      execl(kSystemctl, "systemctl", action.c_str(), unit, nullptr);
    }
    _exit(127);
  }

  // systemctl is called from the ROS executor and must never be allowed to
  // block it indefinitely when PID 1/DBus is unavailable. Status probes are
  // short; start/stop may wait for the unit's normal shutdown timeout.
  const auto timeout = action == "is-active" || action == "is-failed" ? 2s : 60s;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  for (;;) {
    const pid_t result = waitpid(child, &status, WNOHANG);
    if (result == child) {
      break;
    }
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "waitpid() failed: %s", std::strerror(errno));
      kill(child, SIGKILL);
      waitpid(child, nullptr, 0);
      return -1;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(), "systemctl %s %s timed out after %ld ms",
        action.c_str(), unit, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
      kill(child, SIGKILL);
      waitpid(child, nullptr, 0);
      return 124;
    }
    std::this_thread::sleep_for(20ms);
  }

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

int WorkspaceSupervisorUbuntu::run_cleanup_script() const
{
  const pid_t child = fork();
  if (child < 0) {
    RCLCPP_ERROR(get_logger(), "fork() for cleanup script failed: %s", std::strerror(errno));
    return -1;
  }

  if (child == 0) {
    execl(kCleanupScript, kCleanupScript, nullptr);
    _exit(127);
  }

  const auto timeout = 45s;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  for (;;) {
    const pid_t result = waitpid(child, &status, WNOHANG);
    if (result == child) {
      break;
    }
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "waitpid() for cleanup script failed: %s", std::strerror(errno));
      kill(child, SIGKILL);
      waitpid(child, nullptr, 0);
      return -1;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(get_logger(), "workspace cleanup script timed out after 45 seconds");
      kill(child, SIGKILL);
      waitpid(child, nullptr, 0);
      return 124;
    }
    std::this_thread::sleep_for(20ms);
  }

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

bool WorkspaceSupervisorUbuntu::unit_is_active() const
{
  return any_stack_is_active();
}

bool WorkspaceSupervisorUbuntu::unit_is_active(const char * unit) const
{
  return run_systemctl("is-active", unit) == 0;
}

bool WorkspaceSupervisorUbuntu::unit_is_failed(const char * unit) const
{
  return run_systemctl("is-failed", unit) == 0;
}

bool WorkspaceSupervisorUbuntu::any_stack_is_active() const
{
  return unit_is_active(kSimulationStackUnit) ||
         unit_is_active(kRealStackUnit) ||
         unit_is_active(kLegacyStackUnit) ||
         unit_is_active(kBootRobotUnit);
}

bool WorkspaceSupervisorUbuntu::any_managed_unit_is_failed() const
{
  return unit_is_failed(kSimulationStackUnit) ||
         unit_is_failed(kRealStackUnit) ||
         unit_is_failed(kLegacyStackUnit) ||
         unit_is_failed(kBootRobotUnit) ||
         unit_is_failed(kRvizUnit);
}

void WorkspaceSupervisorUbuntu::reset_failed_managed_units() const
{
  constexpr const char * managed_units[] = {
    kSimulationStackUnit,
    kRealStackUnit,
    kLegacyStackUnit,
    kBootRobotUnit,
    kRvizUnit,
  };
  for (const char * unit : managed_units) {
    if (unit_is_failed(unit)) {
      const int result = run_systemctl("reset-failed", unit);
      if (result != 0) {
        RCLCPP_WARN(
          get_logger(), "Failed to reset stopped unit %s: systemctl exit=%d", unit, result);
      }
    }
  }
}

void WorkspaceSupervisorUbuntu::start_simulation_rviz() const
{
  if (unit_is_active(kRvizUnit)) {
    return;
  }
  const int result = run_systemctl("start", kRvizUnit);
  if (result != 0) {
    RCLCPP_WARN(
      get_logger(),
      "Simulation stack is ready, but RViz failed to start: systemctl exit=%d",
      result);
  } else {
    RCLCPP_INFO(get_logger(), "Simulation RViz start requested");
  }
}

void WorkspaceSupervisorUbuntu::stop_simulation_rviz() const
{
  if (!unit_is_active(kRvizUnit) && !unit_is_failed(kRvizUnit)) {
    return;
  }
  const int result = run_systemctl("stop", kRvizUnit);
  if (result != 0) {
    RCLCPP_WARN(get_logger(), "RViz failed to stop: systemctl exit=%d", result);
  }
}

std::uint8_t WorkspaceSupervisorUbuntu::detect_active_mode() const
{
  if (unit_is_active(kRealStackUnit)) {
    return WorkspaceStatus::REAL;
  }
  if (unit_is_active(kSimulationStackUnit) || unit_is_active(kLegacyStackUnit)) {
    return WorkspaceStatus::SIMULATION;
  }
  if (unit_is_active(kBootRobotUnit)) {
    return WorkspaceStatus::SIMULATION;
  }
  return WorkspaceStatus::UNKNOWN;
}

const char * WorkspaceSupervisorUbuntu::unit_for_mode(std::uint8_t mode) const
{
  return mode == WorkspaceControl::REAL ? kRealStackUnit : kSimulationStackUnit;
}

}  // namespace robot_control

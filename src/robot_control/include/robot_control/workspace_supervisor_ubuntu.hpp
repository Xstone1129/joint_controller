#ifndef ROBOT_CONTROL__WORKSPACE_SUPERVISOR_UBUNTU_HPP_
#define ROBOT_CONTROL__WORKSPACE_SUPERVISOR_UBUNTU_HPP_

// Workspace supervisor node (lower machine).
//
// The node itself only owns the ROS interface and the START/STOP state
// machine; its implementation is split by theme so each file stays small:
//
//   workspace_supervisor_ubuntu.cpp     ROS interface, command state machine,
//                                       status publishing, main()
//   workspace_supervisor_systemd.cpp    systemd unit control, RViz, unit/mode
//                                       mapping
//   workspace_supervisor_probe.cpp      /proc process probing and the ROS
//                                       service-server graph
//   workspace_supervisor_readiness.cpp  hardware/endpoint readiness collection
//                                       and the bounded wait loops
//
// Pure decision logic that does not need ROS state lives in
// workspace_supervisor_policy.hpp so it can be unit tested.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/workspace_control.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_control
{
namespace workspace_supervisor
{
// Managed systemd units. The REAL and SIMULATION stacks are mutually
// exclusive; robot.service is the legacy boot unit and joint-controller-stack
// is the legacy generic unit kept only so a stale boot configuration is still
// stopped and reported instead of being left behind unmanaged.
constexpr char kSystemctl[] = "/usr/bin/systemctl";
constexpr char kLegacyStackUnit[] = "joint-controller-stack.service";
constexpr char kSimulationStackUnit[] = "joint-controller-stack-sim.service";
constexpr char kRealStackUnit[] = "joint-controller-stack-real.service";
constexpr char kRvizUnit[] = "joint-controller-rviz.service";
constexpr char kBootRobotUnit[] = "robot.service";
constexpr char kCleanupScript[] = "/home/user/joint_controller/stop.sh";
constexpr char kDefaultStatusSource[] = "ubuntu_192_168_2_20";

struct ServiceServerGraph
{
  std::map<std::string, std::vector<std::string>> types;
  std::map<std::string, std::vector<std::string>> owners;
};

struct ManagedProcess
{
  pid_t pid;
  std::string command;
};

inline std::string fully_qualified_node_name(
  const std::string & node_name, const std::string & node_namespace)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  return node_namespace.back() == '/' ?
         node_namespace + node_name : node_namespace + "/" + node_name;
}

// The lower workspace publishes its lift driver diagnostics as a JSON string.
// These two helpers read single keys out of that text without pulling in a JSON
// dependency; they are intentionally tolerant and return the fallback when the
// key is absent or malformed.
inline bool json_bool(const std::string & text, const char * key)
{
  return text.find(std::string("\"") + key + "\":true") != std::string::npos;
}

inline long json_integer(const std::string & text, const char * key, long fallback = 0)
{
  const std::string prefix = std::string("\"") + key + "\":";
  const auto begin = text.find(prefix);
  if (begin == std::string::npos) {
    return fallback;
  }
  try {
    return std::stol(text.substr(begin + prefix.size()), nullptr, 0);
  } catch (...) {
    return fallback;
  }
}
}  // namespace workspace_supervisor

class WorkspaceSupervisorUbuntu : public rclcpp::Node
{
public:
  using WorkspaceControl = robot_control_msg::msg::WorkspaceControl;
  using WorkspaceStatus = robot_control_msg::msg::WorkspaceStatus;
  using ArmPowerStatus = robot_control_msg::msg::ArmPowerStatus;

  WorkspaceSupervisorUbuntu();
  ~WorkspaceSupervisorUbuntu() override;

private:
  // --- systemd unit control (workspace_supervisor_systemd.cpp) -------------
  int run_systemctl(const std::string & action, const char * unit) const;
  int run_cleanup_script() const;
  bool unit_is_active() const;
  bool unit_is_active(const char * unit) const;
  bool unit_is_failed(const char * unit) const;
  bool any_stack_is_active() const;
  bool any_managed_unit_is_failed() const;
  void reset_failed_managed_units() const;
  void start_simulation_rviz() const;
  void stop_simulation_rviz() const;
  std::uint8_t detect_active_mode() const;
  const char * unit_for_mode(std::uint8_t mode) const;

  // --- process and ROS graph probing (workspace_supervisor_probe.cpp) ------
  static std::string join(const std::vector<std::string> & values);
  static std::string read_process_command(pid_t pid);
  static std::vector<workspace_supervisor::ManagedProcess> collect_processes_matching(
    const std::vector<std::string> & fragments);
  static std::vector<workspace_supervisor::ManagedProcess> collect_managed_stack_processes();
  static std::vector<workspace_supervisor::ManagedProcess> collect_igh_driver_processes();
  static std::string describe_process(
    const workspace_supervisor::ManagedProcess & process);
  workspace_supervisor::ServiceServerGraph collect_service_server_graph();

  // --- readiness (workspace_supervisor_readiness.cpp) ----------------------
  void begin_real_hardware_observation();
  std::vector<std::string> collect_lift_hardware_issues(bool require_startup_power_off);
  std::vector<std::string> collect_real_hardware_issues(bool require_startup_power_off = false);
  std::vector<std::string> collect_readiness_issues(
    const char * unit, bool require_startup_power_off = false);
  std::vector<std::string> collect_unmanaged_stack_issues();
  std::vector<std::string> collect_stopped_issues();
  bool wait_for_stack_stopped(std::vector<std::string> & final_issues);
  bool wait_for_stack_ready(const char * unit, std::vector<std::string> & final_issues);

  // --- command state machine and status (workspace_supervisor_ubuntu.cpp) --
  void handle_command(const WorkspaceControl::SharedPtr msg);
  void refresh_status();
  void publish_status(std::uint32_t command_seq, bool accepted, const std::string & message);

  std::string allowed_source_;
  std::string status_source_;
  int startup_timeout_ms_{45000};
  int readiness_stability_ms_{3000};
  int cleanup_timeout_ms_{30000};
  int cleanup_stability_ms_{1000};
  int hardware_feedback_timeout_ms_{1000};
  int real_hardware_readiness_timeout_ms_{15000};
  std::atomic<std::uint8_t> current_state_{WorkspaceStatus::STOPPED};
  std::atomic<std::uint8_t> current_mode_{WorkspaceStatus::UNKNOWN};
  std::atomic<bool> operation_in_progress_{false};
  std::atomic<std::uint32_t> operation_command_seq_{0};
  std::mutex operation_mutex_;
  std::mutex power_status_mutex_;
  std::mutex lift_status_mutex_;
  std::thread worker_;
  ArmPowerStatus last_power_status_;
  bool has_power_status_{false};
  std::chrono::steady_clock::time_point power_status_received_at_{};
  std::chrono::steady_clock::time_point real_lifecycle_started_at_{};
  std::string last_lift_status_;
  bool has_lift_status_{false};
  std::chrono::steady_clock::time_point lift_status_received_at_{};
  rclcpp::Publisher<WorkspaceStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<WorkspaceControl>::SharedPtr command_subscription_;
  rclcpp::Subscription<ArmPowerStatus>::SharedPtr power_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lift_status_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__WORKSPACE_SUPERVISOR_UBUNTU_HPP_

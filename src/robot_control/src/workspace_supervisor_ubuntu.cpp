#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_control/workspace_supervisor_policy.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/workspace_control.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
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

std::string fully_qualified_node_name(
  const std::string & node_name, const std::string & node_namespace)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  return node_namespace.back() == '/' ?
         node_namespace + node_name : node_namespace + "/" + node_name;
}

bool json_bool(const std::string & text, const char * key)
{
  return text.find(std::string("\"") + key + "\":true") != std::string::npos;
}

long json_integer(const std::string & text, const char * key, long fallback = 0)
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
}  // namespace

class WorkspaceSupervisorUbuntu : public rclcpp::Node
{
public:
  using WorkspaceControl = robot_control_msg::msg::WorkspaceControl;
  using WorkspaceStatus = robot_control_msg::msg::WorkspaceStatus;
  using ArmPowerStatus = robot_control_msg::msg::ArmPowerStatus;

  WorkspaceSupervisorUbuntu()
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

  ~WorkspaceSupervisorUbuntu() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  int run_systemctl(const std::string & action, const char * unit) const
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

  int run_cleanup_script() const
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

  bool unit_is_active() const
  {
    return any_stack_is_active();
  }

  bool unit_is_active(const char * unit) const
  {
    return run_systemctl("is-active", unit) == 0;
  }

  bool unit_is_failed(const char * unit) const
  {
    return run_systemctl("is-failed", unit) == 0;
  }

  bool any_stack_is_active() const
  {
    return unit_is_active(kSimulationStackUnit) ||
           unit_is_active(kRealStackUnit) ||
           unit_is_active(kLegacyStackUnit) ||
           unit_is_active(kBootRobotUnit);
  }

  bool any_managed_unit_is_failed() const
  {
    return unit_is_failed(kSimulationStackUnit) ||
           unit_is_failed(kRealStackUnit) ||
           unit_is_failed(kLegacyStackUnit) ||
           unit_is_failed(kBootRobotUnit) ||
           unit_is_failed(kRvizUnit);
  }

  void reset_failed_managed_units() const
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

  void start_simulation_rviz() const
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

  void stop_simulation_rviz() const
  {
    if (!unit_is_active(kRvizUnit) && !unit_is_failed(kRvizUnit)) {
      return;
    }
    const int result = run_systemctl("stop", kRvizUnit);
    if (result != 0) {
      RCLCPP_WARN(get_logger(), "RViz failed to stop: systemctl exit=%d", result);
    }
  }

  std::uint8_t detect_active_mode() const
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

  const char * unit_for_mode(std::uint8_t mode) const
  {
    return mode == WorkspaceControl::REAL ? kRealStackUnit : kSimulationStackUnit;
  }

  static std::string join(const std::vector<std::string> & values)
  {
    std::ostringstream stream;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index != 0) {
        stream << ", ";
      }
      stream << values[index];
    }
    return stream.str();
  }

  static std::string read_process_command(pid_t pid)
  {
    std::ifstream stream("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
    std::string command(
      (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    std::replace(command.begin(), command.end(), '\0', ' ');
    while (!command.empty() && command.back() == ' ') {
      command.pop_back();
    }
    return command;
  }

  static std::vector<ManagedProcess> collect_processes_matching(
    const std::vector<std::string> & fragments)
  {
    std::vector<ManagedProcess> matches;
    DIR * proc = opendir("/proc");
    if (proc == nullptr) {
      return matches;
    }

    while (const auto * entry = readdir(proc)) {
      if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
        continue;
      }
      char * end = nullptr;
      const long parsed_pid = std::strtol(entry->d_name, &end, 10);
      if (end == nullptr || *end != '\0' || parsed_pid <= 1) {
        continue;
      }
      const auto command = read_process_command(static_cast<pid_t>(parsed_pid));
      if (command.empty()) {
        continue;
      }
      const bool matched = std::any_of(
        fragments.begin(), fragments.end(), [&command](const std::string & fragment) {
          return command.find(fragment) != std::string::npos;
        });
      if (matched) {
        matches.push_back({static_cast<pid_t>(parsed_pid), command});
      }
    }
    closedir(proc);
    std::sort(
      matches.begin(), matches.end(),
      [](const ManagedProcess & left, const ManagedProcess & right) {
        return left.pid < right.pid;
      });
    return matches;
  }

  static std::vector<ManagedProcess> collect_managed_stack_processes()
  {
    return collect_processes_matching({
      "/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver",
      "ros2 launch erobot_controller load_controller_arm.launch.py",
      "ros2 launch robot_control robot_control.launch.py",
      "ros2 launch joint_hardware lift_ethercat.launch.py",
      "/controller_manager/ros2_control_node",
      "/controller_manager/spawner",
      "/lift/controller_manager/ros2_control_node",
      "/robot_state_publisher/robot_state_publisher",
      "/robot_control/cartesian_single_control_srv",
      "/robot_control/cartesian_path_absolute_control_srv",
      "/robot_control/cartesian_path_increment_control_srv",
      "/robot_control/joint_absolute_control_srv",
      "/robot_control/joint_batch_control_srv",
      "/robot_control/cartesian_moveL_path",
      "/robot_control_msg/arm_control_mode_service",
    });
  }

  static std::vector<ManagedProcess> collect_igh_driver_processes()
  {
    return collect_processes_matching({
      "/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver"});
  }

  static std::string describe_process(const ManagedProcess & process)
  {
    return "residual process pid=" + std::to_string(process.pid) +
           " cmd='" + process.command + "'";
  }

  ServiceServerGraph collect_service_server_graph()
  {
    ServiceServerGraph graph;
    const auto nodes = get_node_graph_interface()->get_node_names_and_namespaces();
    for (const auto & node : nodes) {
      const auto services = get_service_names_and_types_by_node(node.first, node.second);
      const auto owner = fully_qualified_node_name(node.first, node.second);
      for (const auto & service : services) {
        auto & types = graph.types[service.first];
        for (const auto & type : service.second) {
          if (std::find(types.begin(), types.end(), type) == types.end()) {
            types.push_back(type);
          }
        }

        auto & owners = graph.owners[service.first];
        if (std::find(owners.begin(), owners.end(), owner) == owners.end()) {
          owners.push_back(owner);
        }
      }
    }
    return graph;
  }

  void begin_real_hardware_observation()
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

  std::vector<std::string> collect_lift_hardware_issues(bool require_startup_power_off)
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

  std::vector<std::string> collect_real_hardware_issues(bool require_startup_power_off = false)
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

    constexpr std::array<const char *, 14> expected_joint_names = {
      "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
      "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7",
    };
    if (status.joint_names.size() != expected_joint_names.size() ||
      status.status_codes.size() != expected_joint_names.size() ||
      status.enabled.size() != expected_joint_names.size())
    {
      issues.emplace_back(
        "REAL drive feedback has invalid array sizes: names=" +
        std::to_string(status.joint_names.size()) + " status_codes=" +
        std::to_string(status.status_codes.size()) + " enabled=" +
        std::to_string(status.enabled.size()) + " (expected 14 each)");
      return issues;
    }

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
      issues.emplace_back(
        "REAL EtherCAT drives unavailable: " + join(unavailable_drives));
    }
    const auto startup_power_issues =
      robot_control::workspace_supervisor_policy::collect_startup_arm_power_issues(
      status, require_startup_power_off);
    issues.insert(issues.end(), startup_power_issues.begin(), startup_power_issues.end());
    const auto lift_issues = collect_lift_hardware_issues(require_startup_power_off);
    issues.insert(issues.end(), lift_issues.begin(), lift_issues.end());
    return issues;
  }

  std::vector<std::string> collect_readiness_issues(
    const char * unit, bool require_startup_power_off = false)
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

  std::vector<std::string> collect_unmanaged_stack_issues()
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

  std::vector<std::string> collect_stopped_issues()
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

  bool wait_for_stack_stopped(std::vector<std::string> & final_issues)
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

  bool wait_for_stack_ready(
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

  void handle_command(const WorkspaceControl::SharedPtr msg)
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

  void refresh_status()
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

  void publish_status(
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorkspaceSupervisorUbuntu>());
  rclcpp::shutdown();
  return 0;
}

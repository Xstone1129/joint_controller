#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_lower_gateway/backend.hpp"
#include "robot_lower_gateway/feedback_policy.hpp"
#include "robot_lower_gateway/native_service_forwarder.hpp"
#include "robot_control_msg/srv/cartesian_absolute_control.hpp"
#include "robot_control_msg/srv/cartesian_increment_control.hpp"
#include "robot_control_msg/srv/joint_absolute_control.hpp"
#include "robot_control_msg/srv/joint_batch_control.hpp"
#include "robot_control_msg/srv/set_arm_control_mode.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace robot_lower_gateway
{

class RobotLowerGatewayNode final : public rclcpp::Node
{
public:
  using SetRobotPower = robot_control_msg::srv::SetRobotPower;
  using SetArmControlMode = robot_control_msg::srv::SetArmControlMode;
  using JointBatchControl = robot_control_msg::srv::JointBatchControl;
  using JointAbsoluteControl = robot_control_msg::srv::JointAbsoluteControl;
  using CartesianIncrementControl = robot_control_msg::srv::CartesianIncrementControl;
  using CartesianAbsoluteControl = robot_control_msg::srv::CartesianAbsoluteControl;

  RobotLowerGatewayNode()
  : Node("ubuntu_lower_gateway"),
    backend_loader_("robot_lower_gateway", "robot_lower_gateway::Backend")
  {
    const auto backend_plugin = declare_parameter<std::string>(
      "backend_plugin", "robot_lower_gateway/ErobotBackend");

    robot_lower_gateway::BackendConfiguration configuration;
    const std::vector<std::string> no_joint_names;
    configuration.canonical_joint_names =
      declare_parameter<std::vector<std::string>>("canonical_joint_names", no_joint_names);
    configuration.native_joint_names =
      declare_parameter<std::vector<std::string>>("native_joint_names", no_joint_names);
    configuration.joint_state_topic =
      declare_parameter<std::string>("topics.joint_state", "/arm/joint_states");
    configuration.power_status_topic =
      declare_parameter<std::string>("topics.power_status", "/arm/power_status");
    configuration.control_mode_status_topic = declare_parameter<std::string>(
      "topics.control_mode_status", "/arm/control_mode_status");
    configuration.motion_status_topic = declare_parameter<std::string>(
      "topics.motion_status", "/arm/arm_controller/motion_status");
    configuration.tcp_pose_topic =
      declare_parameter<std::string>("topics.tcp_pose", "/arm_tcp_pose");
    configuration.execution_status_topic = declare_parameter<std::string>(
      "topics.execution_status", "/arm_cartesian_path_execution_status");
    configuration.workspace_status_topic = declare_parameter<std::string>(
      "topics.workspace_status", "/workspace/status");
    const auto enabled_status_codes =
      declare_parameter<std::vector<std::int64_t>>("drive_status.enabled_codes", {39});
    const auto unavailable_status_codes =
      declare_parameter<std::vector<std::int64_t>>("drive_status.unavailable_codes", {0});

    // ROS parameters expose integer arrays as int64; narrow only after validation.
    configuration.enabled_status_codes = narrowCodes(enabled_status_codes, "enabled_codes");
    configuration.unavailable_status_codes = narrowCodes(
      unavailable_status_codes, "unavailable_codes");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    joint_state_stale_sec_ = millisecondsParameter("stale_timeout_ms.joint_state", 250);
    power_status_stale_sec_ = millisecondsParameter("stale_timeout_ms.power_status", 500);
    control_mode_stale_sec_ = millisecondsParameter("stale_timeout_ms.control_mode", 500);
    motion_status_stale_sec_ = millisecondsParameter("stale_timeout_ms.motion_status", 500);
    tcp_pose_stale_sec_ = millisecondsParameter("stale_timeout_ms.tcp_pose", 500);
    execution_status_stale_sec_ = millisecondsParameter("stale_timeout_ms.execution", 3000);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 200.0) {
      throw std::invalid_argument("publish_rate_hz must be in (0, 200]");
    }

    configureCommandProxy();

    backend_ = backend_loader_.createSharedInstance(backend_plugin);
    backend_->configure(*this, configuration);
    capabilities_ = backend_->capabilities();

    canonical_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(period, [this]() {publishSnapshot();});

    RCLCPP_INFO(
      get_logger(),
      "Lower gateway started; backend=%s joints=%zu output=%s command_proxy=%s",
      backend_->name().c_str(), configuration.canonical_joint_names.size(),
      "~/joint_states", enable_command_proxy_ ? "enabled" : "disabled");
  }

private:
  static std::vector<std::int32_t> narrowCodes(
    const std::vector<std::int64_t> & values, const std::string & parameter_name)
  {
    std::vector<std::int32_t> result;
    result.reserve(values.size());
    for (const auto value : values) {
      if (value < std::numeric_limits<std::int32_t>::min() ||
        value > std::numeric_limits<std::int32_t>::max())
      {
        throw std::invalid_argument(parameter_name + " contains a value outside int32 range");
      }
      result.push_back(static_cast<std::int32_t>(value));
    }
    return result;
  }

  double millisecondsParameter(const std::string & name, std::int64_t default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<double>(value) / 1000.0;
  }

  static diagnostic_msgs::msg::KeyValue keyValue(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
  }

  static std::string boolText(bool value)
  {
    return value ? "true" : "false";
  }

  static std::string proxyServiceName(
    const std::string & prefix, const std::string & leaf)
  {
    if (prefix.empty() || prefix == "~" || prefix == "~/") {
      return "~/" + leaf;
    }
    if (prefix.back() == '/') {
      return prefix + leaf;
    }
    return prefix + "/" + leaf;
  }

  static std::chrono::milliseconds commandTimeout(
    rclcpp::Node & node, const std::string & name, std::int64_t default_value)
  {
    const auto value = node.declare_parameter<std::int64_t>(name, default_value);
    if (value <= 0 || value > 600000) {
      throw std::invalid_argument(name + " must be in (0, 600000] milliseconds");
    }
    return std::chrono::milliseconds(value);
  }

  bool acquireCommandLock(std::unique_lock<std::mutex> & lock)
  {
    lock = std::unique_lock<std::mutex>(command_mutex_, std::defer_lock);
    return lock.try_lock();
  }

  void configureCommandProxy()
  {
    enable_command_proxy_ = declare_parameter<bool>("enable_command_proxy", false);
    command_proxy_prefix_ = declare_parameter<std::string>(
      "command_proxy_prefix", "~/");
    if (command_proxy_prefix_ == "/") {
      throw std::invalid_argument(
              "command_proxy_prefix='/' is unsafe; use '~/' or an explicit namespace");
    }

    native_power_service_ = declare_parameter<std::string>(
      "native_services.power", "/set_robot_power");
    native_mode_service_ = declare_parameter<std::string>(
      "native_services.mode", "/set_arm_control_mode");
    native_joint_batch_service_ = declare_parameter<std::string>(
      "native_services.joint_batch", "/arm/joint_batch_control");
    native_joint_absolute_service_ = declare_parameter<std::string>(
      "native_services.joint_absolute", "/arm_absolute_control");
    native_cartesian_increment_service_ = declare_parameter<std::string>(
      "native_services.cartesian_increment", "/cartesian_increment_control");
    native_cartesian_absolute_service_ = declare_parameter<std::string>(
      "native_services.cartesian_absolute", "/cartesian_absolute_control");

    power_timeout_ = commandTimeout(*this, "command_timeout_ms.power", 30000);
    mode_timeout_ = commandTimeout(*this, "command_timeout_ms.mode", 5000);
    joint_timeout_ = commandTimeout(*this, "command_timeout_ms.joint", 30000);
    cartesian_timeout_ = commandTimeout(*this, "command_timeout_ms.cartesian", 60000);

    if (!enable_command_proxy_) {
      return;
    }

    // Keep proxy callbacks reentrant so a second request can be rejected by
    // command_mutex_ immediately instead of waiting behind a long motion call.
    command_service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    power_client_ = create_client<SetRobotPower>(
      native_power_service_, rmw_qos_profile_services_default, client_callback_group_);
    mode_client_ = create_client<SetArmControlMode>(
      native_mode_service_, rmw_qos_profile_services_default, client_callback_group_);
    joint_batch_client_ = create_client<JointBatchControl>(
      native_joint_batch_service_, rmw_qos_profile_services_default, client_callback_group_);
    joint_absolute_client_ = create_client<JointAbsoluteControl>(
      native_joint_absolute_service_, rmw_qos_profile_services_default, client_callback_group_);
    cartesian_increment_client_ = create_client<CartesianIncrementControl>(
      native_cartesian_increment_service_, rmw_qos_profile_services_default,
      client_callback_group_);
    cartesian_absolute_client_ = create_client<CartesianAbsoluteControl>(
      native_cartesian_absolute_service_, rmw_qos_profile_services_default,
      client_callback_group_);

    power_proxy_service_ = create_service<SetRobotPower>(
      proxyServiceName(command_proxy_prefix_, "set_robot_power"),
      [this](const std::shared_ptr<SetRobotPower::Request> request,
      std::shared_ptr<SetRobotPower::Response> response) {
        handlePowerProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    mode_proxy_service_ = create_service<SetArmControlMode>(
      proxyServiceName(command_proxy_prefix_, "set_control_mode"),
      [this](const std::shared_ptr<SetArmControlMode::Request> request,
      std::shared_ptr<SetArmControlMode::Response> response) {
        handleModeProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    joint_batch_proxy_service_ = create_service<JointBatchControl>(
      proxyServiceName(command_proxy_prefix_, "joint_batch_control"),
      [this](const std::shared_ptr<JointBatchControl::Request> request,
      std::shared_ptr<JointBatchControl::Response> response) {
        handleJointBatchProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    joint_absolute_proxy_service_ = create_service<JointAbsoluteControl>(
      proxyServiceName(command_proxy_prefix_, "joint_absolute_control"),
      [this](const std::shared_ptr<JointAbsoluteControl::Request> request,
      std::shared_ptr<JointAbsoluteControl::Response> response) {
        handleJointAbsoluteProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    cartesian_increment_proxy_service_ = create_service<CartesianIncrementControl>(
      proxyServiceName(command_proxy_prefix_, "cartesian_increment_control"),
      [this](const std::shared_ptr<CartesianIncrementControl::Request> request,
      std::shared_ptr<CartesianIncrementControl::Response> response) {
        handleCartesianIncrementProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    cartesian_absolute_proxy_service_ = create_service<CartesianAbsoluteControl>(
      proxyServiceName(command_proxy_prefix_, "cartesian_absolute_control"),
      [this](const std::shared_ptr<CartesianAbsoluteControl::Request> request,
      std::shared_ptr<CartesianAbsoluteControl::Response> response) {
        handleCartesianAbsoluteProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Command proxy enabled under '%s'; native power='%s' mode='%s' joint_batch='%s' "
      "joint_absolute='%s' cartesian_increment='%s' cartesian_absolute='%s'",
      command_proxy_prefix_.c_str(), native_power_service_.c_str(), native_mode_service_.c_str(),
      native_joint_batch_service_.c_str(), native_joint_absolute_service_.c_str(),
      native_cartesian_increment_service_.c_str(), native_cartesian_absolute_service_.c_str());
  }

  static void rejectBusy(SetRobotPower::Response & response)
  {
    response.success = false;
    response.message = "another lower gateway command is already in progress";
  }

  static void rejectBusy(SetArmControlMode::Response & response)
  {
    response.success = false;
    response.message = "another lower gateway command is already in progress";
  }

  static void rejectBusy(JointBatchControl::Response & response)
  {
    response.accepted = false;
    response.message = "another lower gateway command is already in progress";
  }

  static void rejectBusy(JointAbsoluteControl::Response & response)
  {
    response.success = false;
    response.message = "another lower gateway command is already in progress";
  }

  static void rejectBusy(CartesianIncrementControl::Response & response)
  {
    response.success = false;
    response.message = "another lower gateway command is already in progress";
  }

  static void rejectBusy(CartesianAbsoluteControl::Response & response)
  {
    response.success = false;
    response.message = "another lower gateway command is already in progress";
  }

  void handlePowerProxy(
    const std::shared_ptr<SetRobotPower::Request> & request,
    std::shared_ptr<SetRobotPower::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<SetRobotPower>(
      request, power_client_, native_power_service_, power_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Power proxy completed: success=%s message='%s'",
      result->success ? "true" : "false", result->message.c_str());
  }

  void handleModeProxy(
    const std::shared_ptr<SetArmControlMode::Request> & request,
    std::shared_ptr<SetArmControlMode::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<SetArmControlMode>(
      request, mode_client_, native_mode_service_, mode_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->active_mode = result->active_mode;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Control mode proxy completed: success=%s active_mode=%u",
      result->success ? "true" : "false", static_cast<unsigned>(result->active_mode));
  }

  void handleJointBatchProxy(
    const std::shared_ptr<JointBatchControl::Request> & request,
    std::shared_ptr<JointBatchControl::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<JointBatchControl>(
      request, joint_batch_client_, native_joint_batch_service_, joint_timeout_, error);
    if (!result) {
      response->accepted = false;
      response->command_seq = request->command_seq;
      response->message = error;
      return;
    }
    response->accepted = result->accepted;
    response->command_seq = result->command_seq;
    response->target_joints = result->target_joints;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Joint batch proxy completed: accepted=%s seq=%u",
      result->accepted ? "true" : "false", result->command_seq);
  }

  void handleJointAbsoluteProxy(
    const std::shared_ptr<JointAbsoluteControl::Request> & request,
    std::shared_ptr<JointAbsoluteControl::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<JointAbsoluteControl>(
      request, joint_absolute_client_, native_joint_absolute_service_, joint_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Joint absolute proxy completed: success=%s",
      result->success ? "true" : "false");
  }

  void handleCartesianIncrementProxy(
    const std::shared_ptr<CartesianIncrementControl::Request> & request,
    std::shared_ptr<CartesianIncrementControl::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<CartesianIncrementControl>(
      request, cartesian_increment_client_, native_cartesian_increment_service_,
      cartesian_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Cartesian increment proxy completed: success=%s",
      result->success ? "true" : "false");
  }

  void handleCartesianAbsoluteProxy(
    const std::shared_ptr<CartesianAbsoluteControl::Request> & request,
    std::shared_ptr<CartesianAbsoluteControl::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<CartesianAbsoluteControl>(
      request, cartesian_absolute_client_, native_cartesian_absolute_service_,
      cartesian_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Cartesian absolute proxy completed: success=%s",
      result->success ? "true" : "false");
  }

  static std::string ageText(double age_sec)
  {
    if (!std::isfinite(age_sec)) {
      return "WAITING";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << age_sec * 1000.0 << " ms";
    return stream.str();
  }

  static std::string jointPositionsText(
    const robot_lower_gateway::BackendSnapshot & snapshot)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    for (std::size_t index = 0;
      index < snapshot.joint_names.size() && index < snapshot.joint_positions.size(); ++index)
    {
      if (index != 0) {
        stream << ' ';
      }
      stream << snapshot.joint_names[index] << '=' << snapshot.joint_positions[index];
    }
    return stream.str();
  }

  static std::string driveStatusText(
    const robot_lower_gateway::BackendSnapshot & snapshot)
  {
    std::ostringstream stream;
    for (std::size_t index = 0;
      index < snapshot.joint_names.size() && index < snapshot.drive_states.size(); ++index)
    {
      if (index != 0) {
        stream << ' ';
      }
      stream << snapshot.joint_names[index] << '=';
      switch (snapshot.drive_states[index]) {
        case robot_lower_gateway::DriveState::kUnavailable:
          stream << "UNAVAILABLE";
          break;
        case robot_lower_gateway::DriveState::kNotEnabled:
          stream << "NOT_ENABLED";
          break;
        case robot_lower_gateway::DriveState::kEnabled:
          stream << "ENABLED";
          break;
        default:
          stream << "UNKNOWN";
          break;
      }
      if (index < snapshot.raw_drive_status_codes.size()) {
        stream << '(' << snapshot.raw_drive_status_codes[index] << ')';
      }
    }
    return stream.str();
  }

  static void checkFeedback(
    bool received, double age_sec, double stale_sec, const std::string & name,
    std::vector<std::string> & issues)
  {
    if (!received) {
      issues.push_back(name + " waiting");
    } else if (!std::isfinite(age_sec) || age_sec > stale_sec) {
      issues.push_back(name + " stale");
    }
  }

  void publishSnapshot()
  {
    const auto snapshot = backend_->snapshot();

    const bool joints_fresh = snapshot.joint_state_received && snapshot.joint_state_valid &&
      snapshot.joint_state_complete && snapshot.joint_state_age_sec <= joint_state_stale_sec_;
    if (joints_fresh) {
      sensor_msgs::msg::JointState message;
      message.header.stamp = now();
      message.name = snapshot.joint_names;
      message.position = snapshot.joint_positions;
      message.velocity = snapshot.joint_velocities;
      message.effort = snapshot.joint_efforts;
      canonical_joint_state_publisher_->publish(message);
    }

    std::vector<std::string> issues;
    checkFeedback(
      snapshot.joint_state_received, snapshot.joint_state_age_sec,
      joint_state_stale_sec_, "joint_state", issues);
    if (snapshot.joint_state_received && !snapshot.joint_state_valid) {
      issues.push_back("joint_state INVALID: " + snapshot.joint_state_error);
    }
    if (capabilities_.observes_power) {
      checkFeedback(
        snapshot.power_status_received, snapshot.power_status_age_sec,
        power_status_stale_sec_, "power_status", issues);
    }
    if (capabilities_.observes_control_mode) {
      checkFeedback(
        snapshot.control_mode_received, snapshot.control_mode_age_sec,
        control_mode_stale_sec_, "control_mode", issues);
    }
    if (capabilities_.observes_motion) {
      checkFeedback(
        snapshot.motion_status_received, snapshot.motion_status_age_sec,
        motion_status_stale_sec_, "motion_status", issues);
    }
    if (capabilities_.observes_tcp_pose) {
      checkFeedback(
        snapshot.tcp_pose_received, snapshot.tcp_pose_age_sec,
        tcp_pose_stale_sec_, "tcp_pose", issues);
    }
    if (capabilities_.observes_execution) {
      if (!snapshot.execution_status_received) {
        issues.push_back("execution_status waiting for current workspace generation");
      } else if (executionStatusIsStale(
          true, snapshot.execution_state, snapshot.execution_status_age_sec,
          execution_status_stale_sec_))
      {
        issues.push_back("execution_status stale while active");
      }
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "ubuntu_lower_gateway/backend";
    status.hardware_id = backend_->name();
    status.level = issues.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    if (issues.empty()) {
      status.message = "all configured feedback is fresh";
    } else {
      std::ostringstream stream;
      for (std::size_t index = 0; index < issues.size(); ++index) {
        if (index != 0) {
          stream << "; ";
        }
        stream << issues[index];
      }
      status.message = stream.str();
    }

    status.values.push_back(keyValue("read_only", boolText(!enable_command_proxy_)));
    status.values.push_back(keyValue("command_proxy_enabled", boolText(enable_command_proxy_)));
    status.values.push_back(keyValue("joint_state_age", ageText(snapshot.joint_state_age_sec)));
    status.values.push_back(
      keyValue(
        "joint_state_complete", boolText(
          snapshot.joint_state_complete)));
    status.values.push_back(keyValue("joint_state_valid", boolText(snapshot.joint_state_valid)));
    status.values.push_back(keyValue("joint_state_error", snapshot.joint_state_error));
    status.values.push_back(
      keyValue("joint_velocity_count", std::to_string(snapshot.joint_velocities.size())));
    status.values.push_back(
      keyValue("joint_effort_count", std::to_string(snapshot.joint_efforts.size())));
    status.values.push_back(keyValue("joint_positions_rad", jointPositionsText(snapshot)));
    status.values.push_back(keyValue("power_status_age", ageText(snapshot.power_status_age_sec)));
    status.values.push_back(
      keyValue(
        "power_command_enabled",
        boolText(snapshot.power_command_enabled)));
    status.values.push_back(keyValue("all_drives_enabled", boolText(snapshot.all_drives_enabled)));
    status.values.push_back(keyValue("drive_status", driveStatusText(snapshot)));
    status.values.push_back(keyValue("control_mode_age", ageText(snapshot.control_mode_age_sec)));
    status.values.push_back(
      keyValue(
        "position_command_ready", boolText(snapshot.position_command_ready)));
    status.values.push_back(keyValue("motion_status_age", ageText(snapshot.motion_status_age_sec)));
    status.values.push_back(keyValue("is_moving", boolText(snapshot.is_moving)));
    status.values.push_back(keyValue("goal_reached", boolText(snapshot.goal_reached)));
    status.values.push_back(keyValue("tcp_pose_age", ageText(snapshot.tcp_pose_age_sec)));
    status.values.push_back(
      keyValue(
        "execution_status_age", ageText(snapshot.execution_status_age_sec)));
    status.values.push_back(keyValue("execution_state", std::to_string(snapshot.execution_state)));
    status.values.push_back(
      keyValue(
        "execution_requires_freshness",
        boolText(executionStatusRequiresFreshness(snapshot.execution_state))));
    status.values.push_back(
      keyValue("workspace_generation", std::to_string(snapshot.workspace_generation)));
    status.values.push_back(
      keyValue("workspace_command_seq", std::to_string(snapshot.workspace_command_seq)));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);

    const auto current_level = array.status.front().level;
    const auto & current_message = array.status.front().message;
    if (!last_diagnostic_valid_ || current_level != last_diagnostic_level_ ||
      current_message != last_diagnostic_message_)
    {
      if (current_level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        RCLCPP_INFO(get_logger(), "Backend feedback healthy");
      } else {
        RCLCPP_WARN(get_logger(), "Backend feedback unhealthy: %s", current_message.c_str());
      }
      last_diagnostic_valid_ = true;
      last_diagnostic_level_ = current_level;
      last_diagnostic_message_ = current_message;
    }
  }

  pluginlib::ClassLoader<robot_lower_gateway::Backend> backend_loader_;
  std::shared_ptr<robot_lower_gateway::Backend> backend_;
  robot_lower_gateway::BackendCapabilities capabilities_;

  double publish_rate_hz_{20.0};
  double joint_state_stale_sec_{0.25};
  double power_status_stale_sec_{0.5};
  double control_mode_stale_sec_{0.5};
  double motion_status_stale_sec_{0.5};
  double tcp_pose_stale_sec_{0.5};
  double execution_status_stale_sec_{3.0};

  bool enable_command_proxy_{false};
  std::string command_proxy_prefix_{"~/"};
  std::string native_power_service_;
  std::string native_mode_service_;
  std::string native_joint_batch_service_;
  std::string native_joint_absolute_service_;
  std::string native_cartesian_increment_service_;
  std::string native_cartesian_absolute_service_;
  std::chrono::milliseconds power_timeout_{30000};
  std::chrono::milliseconds mode_timeout_{5000};
  std::chrono::milliseconds joint_timeout_{30000};
  std::chrono::milliseconds cartesian_timeout_{60000};

  std::mutex command_mutex_;
  rclcpp::CallbackGroup::SharedPtr command_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<SetRobotPower>::SharedPtr power_client_;
  rclcpp::Client<SetArmControlMode>::SharedPtr mode_client_;
  rclcpp::Client<JointBatchControl>::SharedPtr joint_batch_client_;
  rclcpp::Client<JointAbsoluteControl>::SharedPtr joint_absolute_client_;
  rclcpp::Client<CartesianIncrementControl>::SharedPtr cartesian_increment_client_;
  rclcpp::Client<CartesianAbsoluteControl>::SharedPtr cartesian_absolute_client_;
  rclcpp::Service<SetRobotPower>::SharedPtr power_proxy_service_;
  rclcpp::Service<SetArmControlMode>::SharedPtr mode_proxy_service_;
  rclcpp::Service<JointBatchControl>::SharedPtr joint_batch_proxy_service_;
  rclcpp::Service<JointAbsoluteControl>::SharedPtr joint_absolute_proxy_service_;
  rclcpp::Service<CartesianIncrementControl>::SharedPtr cartesian_increment_proxy_service_;
  rclcpp::Service<CartesianAbsoluteControl>::SharedPtr cartesian_absolute_proxy_service_;

  bool last_diagnostic_valid_{false};
  std::uint8_t last_diagnostic_level_{0};
  std::string last_diagnostic_message_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    canonical_joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace robot_lower_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_lower_gateway::RobotLowerGatewayNode>();
    // A proxy callback waits for a response from a native service. Keep at
    // least one executor thread available for the client response callback.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_lower_gateway"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

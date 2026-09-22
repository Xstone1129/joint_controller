#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <iomanip>
#include <iterator>
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
#include "robot_lower_gateway/heavy_gateway_v1.hpp"
#include "robot_lower_gateway/lift_status_parser.hpp"
#include "robot_lower_gateway/native_service_forwarder.hpp"
#include "robot_control_msg/msg/lift_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "robot_control_msg/srv/cartesian_absolute_control.hpp"
#include "robot_control_msg/srv/cartesian_increment_control.hpp"
#include "robot_control_msg/srv/joint_absolute_control.hpp"
#include "robot_control_msg/srv/joint_batch_control.hpp"
#include "robot_control_msg/srv/selected_joint_control.hpp"
#include "robot_control_msg/srv/set_arm_control_mode.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

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
  using SelectedJointControl = robot_control_msg::srv::SelectedJointControl;
  using SetBool = std_srvs::srv::SetBool;
  using Trigger = std_srvs::srv::Trigger;

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
    workspace_status_topic_ = configuration.workspace_status_topic;
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
    lift_status_stale_sec_ = millisecondsParameter("stale_timeout_ms.lift_status", 500);
    lift_joint_state_stale_sec_ = millisecondsParameter(
      "stale_timeout_ms.lift_joint_state", 500);

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
    configureLiftFeedback();
    heavy_gateway_v1_ = std::make_unique<HeavyGatewayV1>(*this);

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
    native_lift_power_service_ = declare_parameter<std::string>(
      "native_services.lift_power", "/lift_brake_command");
    native_lift_command_service_ = declare_parameter<std::string>(
      "native_services.lift_command", "/joint/lift/command");
    native_lift_stop_service_ = declare_parameter<std::string>(
      "native_services.lift_stop", "/joint/lift/stop");
    native_lift_hold_service_ = declare_parameter<std::string>(
      "native_services.lift_hold", "/joint/lift/hold");
    native_lift_set_drive_zero_service_ = declare_parameter<std::string>(
      "native_services.lift_set_drive_zero", "/lift_set_drive_zero");
    // Host-frame zero: persists the current 6064h as the ROS coordinate zero.
    // Unlike the LD3M P00.15=9 maintenance zero it takes effect immediately and
    // needs no drive power cycle, which is what an operator means by
    // "set the current position as zero".
    native_lift_host_zero_service_ = declare_parameter<std::string>(
      "native_services.lift_host_zero", "/lift_reset_zero");

    power_timeout_ = commandTimeout(*this, "command_timeout_ms.power", 30000);
    mode_timeout_ = commandTimeout(*this, "command_timeout_ms.mode", 5000);
    joint_timeout_ = commandTimeout(*this, "command_timeout_ms.joint", 30000);
    cartesian_timeout_ = commandTimeout(*this, "command_timeout_ms.cartesian", 60000);
    lift_power_timeout_ = commandTimeout(*this, "command_timeout_ms.lift_power", 30000);
    lift_command_timeout_ = commandTimeout(*this, "command_timeout_ms.lift_command", 5000);
    lift_safety_timeout_ = commandTimeout(*this, "command_timeout_ms.lift_safety", 5000);
    lift_set_drive_zero_timeout_ = commandTimeout(
      *this, "command_timeout_ms.lift_set_drive_zero", 10000);
    lift_host_zero_timeout_ = commandTimeout(
      *this, "command_timeout_ms.lift_host_zero", 10000);

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
    lift_power_client_ = create_client<SetBool>(
      native_lift_power_service_, rmw_qos_profile_services_default, client_callback_group_);
    lift_command_client_ = create_client<SelectedJointControl>(
      native_lift_command_service_, rmw_qos_profile_services_default, client_callback_group_);
    lift_stop_client_ = create_client<Trigger>(
      native_lift_stop_service_, rmw_qos_profile_services_default, client_callback_group_);
    lift_hold_client_ = create_client<Trigger>(
      native_lift_hold_service_, rmw_qos_profile_services_default, client_callback_group_);
    lift_host_zero_client_ = create_client<Trigger>(
      native_lift_host_zero_service_, rmw_qos_profile_services_default,
      command_service_callback_group_);
    lift_set_drive_zero_client_ = create_client<Trigger>(
      native_lift_set_drive_zero_service_, rmw_qos_profile_services_default,
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
    lift_power_proxy_service_ = create_service<SetBool>(
      proxyServiceName(command_proxy_prefix_, "lift/set_power"),
      [this](const std::shared_ptr<SetBool::Request> request,
      std::shared_ptr<SetBool::Response> response) {
        handleLiftPowerProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    lift_command_proxy_service_ = create_service<SelectedJointControl>(
      proxyServiceName(command_proxy_prefix_, "lift/command"),
      [this](const std::shared_ptr<SelectedJointControl::Request> request,
      std::shared_ptr<SelectedJointControl::Response> response) {
        handleLiftCommandProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    lift_stop_proxy_service_ = create_service<Trigger>(
      proxyServiceName(command_proxy_prefix_, "lift/stop"),
      [this](const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
        handleLiftStopProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    lift_hold_proxy_service_ = create_service<Trigger>(
      proxyServiceName(command_proxy_prefix_, "lift/hold"),
      [this](const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
        handleLiftHoldProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    lift_set_drive_zero_proxy_service_ = create_service<Trigger>(
      proxyServiceName(command_proxy_prefix_, "lift/set_drive_zero"),
      [this](const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
        handleLiftSetDriveZeroProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);
    lift_host_zero_proxy_service_ = create_service<Trigger>(
      proxyServiceName(command_proxy_prefix_, "lift/set_host_zero"),
      [this](const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
        handleLiftHostZeroProxy(request, response);
      }, rmw_qos_profile_services_default, command_service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Command proxy enabled under '%s'; native power='%s' mode='%s' joint_batch='%s' "
      "joint_absolute='%s' cartesian_increment='%s' cartesian_absolute='%s' "
      "lift_power='%s' lift_command='%s' lift_stop='%s' lift_hold='%s' "
      "lift_set_drive_zero='%s'",
      command_proxy_prefix_.c_str(), native_power_service_.c_str(), native_mode_service_.c_str(),
      native_joint_batch_service_.c_str(), native_joint_absolute_service_.c_str(),
      native_cartesian_increment_service_.c_str(), native_cartesian_absolute_service_.c_str(),
      native_lift_power_service_.c_str(), native_lift_command_service_.c_str(),
      native_lift_stop_service_.c_str(), native_lift_hold_service_.c_str(),
      native_lift_set_drive_zero_service_.c_str());
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

  static void rejectBusy(SelectedJointControl::Response & response)
  {
    response.accepted = false;
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

  void handleLiftPowerProxy(
    const std::shared_ptr<SetBool::Request> & request,
    std::shared_ptr<SetBool::Response> response)
  {
    // Enabling is serialized with motion commands. Disabling deliberately is
    // not: a fail-safe lift power/brake request must remain callable while an
    // unrelated arm service is waiting for completion.
    std::unique_lock<std::mutex> command_lock;
    if (request->data && !acquireCommandLock(command_lock)) {
      response->success = false;
      response->message = "another lower gateway command is already in progress";
      return;
    }
    if (request->data) {
      std::string precondition_error;
      if (!liftCommandPreconditionsMet(precondition_error)) {
        // A latched controller fault must not need a human: the same native STOP
        // that a fresh lease acquisition sends clears it.  Without this, every
        // `lift reset` (whose zero write disables the drive by design) left the
        // controller in fault and the next `lift power on` was refused until the
        // operator ran `lift lease reset`.
        const bool fault_latched = precondition_error.find("controller_mode=") != std::string::npos;
        if (fault_latched && lift_stop_client_ && lift_stop_client_->service_is_ready()) {
          auto retry_request = std::make_shared<Trigger::Request>();
          std::string retry_error;
          const auto retry_result = callNativeServiceBounded<Trigger>(
            retry_request, lift_stop_client_, native_lift_stop_service_,
            lift_safety_timeout_, retry_error);
          if (retry_result) {
            RCLCPP_WARN(
              get_logger(),
              "lift power enable found %s; sent a recovery STOP (result=%s) and retrying",
              precondition_error.c_str(), retry_result->message.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            liftCommandPreconditionsMet(precondition_error);
          }
        }
        if (!precondition_error.empty()) {
          response->success = false;
          response->message = precondition_error;
          return;
        }
      }
    }
    std::string hold_warning;
    if (!request->data) {
      auto hold_request = std::make_shared<Trigger::Request>();
      std::string hold_error;
      const auto hold_result = callNativeServiceBounded<Trigger>(
        hold_request, lift_hold_client_, native_lift_hold_service_, lift_safety_timeout_,
        hold_error);
      if (!hold_result || !hold_result->success) {
        hold_warning = "pre-disable HOLD was not confirmed: " +
          (hold_error.empty() && hold_result ? hold_result->message : hold_error);
      }
    }

    std::uint64_t generation_before = 0;
    {
      std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
      generation_before = lift_driver_status_generation_;
    }
    std::string error;
    const auto result = callNativeServiceBounded<SetBool>(
      request, lift_power_client_, native_lift_power_service_, lift_power_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    if (!result->success) {
      response->success = false;
      response->message = result->message;
      return;
    }

    std::string controller_stop_warning;
    if (!request->data) {
      auto stop_request = std::make_shared<Trigger::Request>();
      std::string stop_error;
      const auto stop_result = callNativeServiceBounded<Trigger>(
        stop_request, lift_stop_client_, native_lift_stop_service_, lift_safety_timeout_,
        stop_error);
      if (!stop_result || !stop_result->success) {
        controller_stop_warning = "controller STOP request failed: " +
          (stop_error.empty() && stop_result ? stop_result->message : stop_error);
      }
    }

    std::string feedback;
    if (waitForLiftPowerState(
        request->data, generation_before, lift_power_timeout_, feedback))
    {
      response->success = true;
      response->message = std::string("lift power ") +
        (request->data ? "enabled" : "disabled") + " and confirmed; " + feedback;
      if (!hold_warning.empty()) {
        response->message += "; warning: " + hold_warning;
      }
      if (!controller_stop_warning.empty()) {
        response->message += "; warning: " + controller_stop_warning;
      }
    } else if (request->data) {
      auto rollback_request = std::make_shared<SetBool::Request>();
      rollback_request->data = false;
      {
        std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
        generation_before = lift_driver_status_generation_;
      }
      std::string rollback_error;
      const auto rollback_result = callNativeServiceBounded<SetBool>(
        rollback_request, lift_power_client_, native_lift_power_service_,
        lift_power_timeout_, rollback_error);
      auto stop_request = std::make_shared<Trigger::Request>();
      std::string stop_error;
      (void)callNativeServiceBounded<Trigger>(
        stop_request, lift_stop_client_, native_lift_stop_service_, lift_safety_timeout_,
        stop_error);
      std::string rollback_feedback;
      const bool rollback_confirmed = rollback_result && rollback_result->success &&
        waitForLiftPowerState(
        false, generation_before, lift_power_timeout_, rollback_feedback);
      response->success = false;
      response->message = "lift power enable confirmation failed: " + feedback + "; " +
        (rollback_confirmed ?
        "automatic disable rollback confirmed" :
        "automatic disable rollback NOT confirmed: " +
        (rollback_error.empty() ? rollback_feedback : rollback_error));
    } else {
      response->success = false;
      response->message = "lift power disable was accepted but not confirmed: " + feedback;
    }
    RCLCPP_INFO(
      get_logger(), "Lift power proxy completed: requested=%s success=%s message='%s'",
      request->data ? "true" : "false", response->success ? "true" : "false",
      response->message.c_str());
  }

  bool waitForLiftPowerState(
    bool enabled, std::uint64_t generation_before, std::chrono::milliseconds timeout,
    std::string & description)
  {
    std::unique_lock<std::mutex> lock(lift_feedback_mutex_);
    const auto matches = [this, enabled, generation_before]() {
        if (lift_driver_status_generation_ <= generation_before ||
          !lift_driver_status_valid_)
        {
          return false;
        }
        return liftPowerStateMatches(lift_driver_status_, enabled);
      };
    const bool confirmed = lift_feedback_condition_.wait_for(lock, timeout, matches);
    std::ostringstream stream;
    stream << "generation=" << lift_driver_status_generation_
           << " valid=" << boolText(lift_driver_status_valid_)
           << " command_enabled=" << boolText(lift_driver_status_.command_enabled)
           << " enabled=" << boolText(lift_driver_status_.enabled)
           << " brake_unlocked=" << boolText(lift_driver_status_.brake_unlocked)
           << " cia402_state=" << lift_driver_status_.cia402_state
           << " status_word=" << lift_driver_status_.status_word
           << " error_code=" << lift_driver_status_.error_code;
    if (!lift_driver_status_error_.empty()) {
      stream << " parser_error='" << lift_driver_status_error_ << "'";
    }
    description = stream.str();
    return confirmed;
  }

  void handleLiftCommandProxy(
    const std::shared_ptr<SelectedJointControl::Request> & request,
    std::shared_ptr<SelectedJointControl::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      rejectBusy(*response);
      return;
    }
    std::string validation_error;
    if (!liftCommandRequestValid(
        request->joint_names, request->values, request->vel, request->acc,
        lift_joint_name_, validation_error))
    {
      response->accepted = false;
      response->message = validation_error;
      return;
    }
    std::string precondition_error;
    if (!liftMotionPreconditionsMetCurrent(precondition_error)) {
      response->accepted = false;
      response->message = precondition_error;
      return;
    }
    auto native_request = std::make_shared<SelectedJointControl::Request>(*request);
    native_request->joint_names[0] = native_lift_joint_name_;
    std::string error;
    const auto result = callNativeServiceBounded<SelectedJointControl>(
      native_request, lift_command_client_, native_lift_command_service_,
      lift_command_timeout_, error);
    if (!result) {
      response->accepted = false;
      response->message = error;
      return;
    }
    response->accepted = result->accepted;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Lift command proxy completed: accepted=%s relative=%s",
      result->accepted ? "true" : "false", request->relative ? "true" : "false");
  }

  void handleLiftStopProxy(
    const std::shared_ptr<Trigger::Request> & request,
    std::shared_ptr<Trigger::Response> response)
  {
    forwardLiftSafetyRequest(request, response, lift_stop_client_, native_lift_stop_service_);
  }

  void handleLiftHoldProxy(
    const std::shared_ptr<Trigger::Request> & request,
    std::shared_ptr<Trigger::Response> response)
  {
    forwardLiftSafetyRequest(request, response, lift_hold_client_, native_lift_hold_service_);
  }

  void handleLiftHostZeroProxy(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      response->success = false;
      response->message = "another lower gateway command is already in progress";
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<Trigger>(
      request, lift_host_zero_client_, native_lift_host_zero_service_,
      lift_host_zero_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
  }

  void handleLiftSetDriveZeroProxy(
    const std::shared_ptr<Trigger::Request> & request,
    std::shared_ptr<Trigger::Response> response)
  {
    std::unique_lock<std::mutex> command_lock;
    if (!acquireCommandLock(command_lock)) {
      response->success = false;
      response->message = "another lower gateway command is already in progress";
      return;
    }
    std::string error;
    const auto result = callNativeServiceBounded<Trigger>(
      request, lift_set_drive_zero_client_, native_lift_set_drive_zero_service_,
      lift_set_drive_zero_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Lift drive-zero proxy completed: success=%s message='%s'",
      result->success ? "true" : "false", result->message.c_str());
  }

  void forwardLiftSafetyRequest(
    const std::shared_ptr<Trigger::Request> & request,
    std::shared_ptr<Trigger::Response> response,
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & native_service)
  {
    std::string error;
    const auto result = callNativeServiceBounded<Trigger>(
      request, client, native_service, lift_safety_timeout_, error);
    if (!result) {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = result->success;
    response->message = result->message;
    RCLCPP_INFO(
      get_logger(), "Lift safety proxy '%s' completed: success=%s message='%s'",
      native_service.c_str(), result->success ? "true" : "false", result->message.c_str());
  }

  bool liftCommandPreconditionsMet(std::string & error)
  {
    LiftDriverStatus driver;
    LiftControlStatus control;
    bool workspace_running = false;
    {
      std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
      const auto steady_now = std::chrono::steady_clock::now();
      workspace_running = lift_workspace_state_received_ &&
        lift_workspace_state_ == robot_control_msg::msg::WorkspaceStatus::RUNNING;
      if (!lift_driver_status_received_ || !lift_driver_status_valid_) {
        error = "lift command requires valid driver status";
        return false;
      }
      if (steady_now - lift_driver_status_time_ >
        std::chrono::duration<double>(lift_status_stale_sec_))
      {
        error = "lift command requires fresh driver status";
        return false;
      }
      if (!lift_control_status_received_ || !lift_control_status_valid_ ||
        steady_now - lift_control_status_time_ >
        std::chrono::duration<double>(lift_status_stale_sec_))
      {
        error = "lift command requires fresh controller status";
        return false;
      }
      if (!lift_joint_state_received_ || !lift_joint_state_valid_ ||
        steady_now - lift_joint_state_time_ >
        std::chrono::duration<double>(lift_joint_state_stale_sec_))
      {
        error = "lift command requires fresh joint_motor feedback";
        return false;
      }
      driver = lift_driver_status_;
      control = lift_control_status_;
    }
    if (!liftEnablePreconditionsMet(driver, workspace_running, error)) {
      return false;
    }
    if (control.mode == "fault" || control.mode == "estop") {
      error = "lift command rejected while controller_mode=" + control.mode;
      return false;
    }
    error.clear();
    return true;
  }

  bool liftMotionPreconditionsMetCurrent(std::string & error)
  {
    if (!liftCommandPreconditionsMet(error)) {
      return false;
    }
    LiftDriverStatus driver;
    bool workspace_running = false;
    {
      std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
      driver = lift_driver_status_;
      workspace_running = lift_workspace_state_received_ &&
        lift_workspace_state_ == robot_control_msg::msg::WorkspaceStatus::RUNNING;
    }
    return robot_lower_gateway::liftMotionPreconditionsMet(
      driver, workspace_running, error);
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

  void configureLiftFeedback()
  {
    lift_joint_name_ = declare_parameter<std::string>("lift.canonical_joint_name", "joint_motor");
    native_lift_joint_name_ = declare_parameter<std::string>(
      "lift.native_joint_name", "joint_motor");
    native_lift_driver_status_topic_ = declare_parameter<std::string>(
      "topics.lift_driver_status", "/joint/lift/driver_status");
    native_lift_control_status_topic_ = declare_parameter<std::string>(
      "topics.lift_control_status", "/joint/lift/control_status");
    native_lift_joint_state_topic_ = declare_parameter<std::string>(
      "topics.lift_joint_state", "/lift/joint_states");
    if (lift_joint_name_.empty() || native_lift_joint_name_.empty()) {
      throw std::invalid_argument("lift joint names must not be empty");
    }

    lift_status_publisher_ = create_publisher<robot_control_msg::msg::LiftStatus>(
      "~/lift/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    lift_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/lift/joint_states", rclcpp::SensorDataQoS());
    lift_driver_status_subscription_ = create_subscription<std_msgs::msg::String>(
      native_lift_driver_status_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        LiftDriverStatus parsed;
        std::string error;
        const bool valid = message && parseLiftDriverStatus(message->data, parsed, error);
        std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
        lift_driver_status_received_ = true;
        lift_driver_status_valid_ = valid;
        lift_driver_status_error_ = valid ? std::string() : error;
        if (valid) {
          lift_driver_status_ = std::move(parsed);
        }
        ++lift_driver_status_generation_;
        lift_driver_status_time_ = std::chrono::steady_clock::now();
        lift_feedback_condition_.notify_all();
      });
    // lift_controller intentionally publishes this monitoring stream as best effort.
    // A best-effort reader remains compatible with both best-effort and reliable writers.
    lift_control_status_subscription_ = create_subscription<std_msgs::msg::String>(
      native_lift_control_status_topic_, rclcpp::QoS(1).best_effort(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        LiftControlStatus parsed;
        std::string error;
        const bool valid = message && parseLiftControlStatus(message->data, parsed, error);
        std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
        lift_control_status_received_ = true;
        lift_control_status_valid_ = valid;
        lift_control_status_error_ = valid ? std::string() : error;
        if (valid) {
          lift_control_status_ = std::move(parsed);
        }
        lift_control_status_time_ = std::chrono::steady_clock::now();
      });
    lift_joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      native_lift_joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        bool valid = false;
        std::string error = "lift joint state message is empty";
        double position = 0.0;
        double velocity = 0.0;
        bool velocity_available = false;
        if (message) {
          const auto first = std::find(
            message->name.begin(), message->name.end(), native_lift_joint_name_);
          if (first == message->name.end()) {
            error = "native lift joint is missing from joint state";
          } else if (std::find(std::next(first), message->name.end(), native_lift_joint_name_) !=
          message->name.end())
          {
            error = "native lift joint is duplicated in joint state";
          } else {
            const auto index = static_cast<std::size_t>(std::distance(
              message->name.begin(), first));
            if (index >= message->position.size() || !std::isfinite(message->position[index])) {
              error = "native lift position is missing or non-finite";
            } else if (!message->velocity.empty() &&
            (index >= message->velocity.size() || !std::isfinite(message->velocity[index])))
            {
              error = "native lift velocity is incomplete or non-finite";
            } else {
              valid = true;
              error.clear();
              position = message->position[index];
              velocity_available = !message->velocity.empty();
              velocity = velocity_available ? message->velocity[index] : 0.0;
            }
          }
        }
        std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
        lift_joint_state_received_ = true;
        lift_joint_state_valid_ = valid;
        lift_joint_state_error_ = error;
        if (valid) {
          lift_joint_position_ = position;
          lift_joint_velocity_ = velocity;
          lift_joint_velocity_available_ = velocity_available;
        }
        lift_joint_state_time_ = std::chrono::steady_clock::now();
      });
    lift_workspace_status_subscription_ =
      create_subscription<robot_control_msg::msg::WorkspaceStatus>(
      workspace_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const robot_control_msg::msg::WorkspaceStatus::SharedPtr message) {
        if (!message) {
          return;
        }
        std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
        const bool entering_stopped =
          message->state == robot_control_msg::msg::WorkspaceStatus::STOPPED &&
          (!lift_workspace_state_received_ ||
          lift_workspace_state_ != robot_control_msg::msg::WorkspaceStatus::STOPPED);
        lift_workspace_state_received_ = true;
        lift_workspace_state_ = message->state;
        if (!entering_stopped) {
          return;
        }
        lift_driver_status_ = LiftDriverStatus{};
        lift_control_status_ = LiftControlStatus{};
        lift_driver_status_received_ = false;
        lift_driver_status_valid_ = false;
        lift_control_status_received_ = false;
        lift_control_status_valid_ = false;
        lift_joint_state_received_ = false;
        lift_joint_state_valid_ = false;
        lift_joint_velocity_available_ = false;
        lift_joint_position_ = 0.0;
        lift_joint_velocity_ = 0.0;
        lift_driver_status_error_.clear();
        lift_control_status_error_.clear();
        lift_joint_state_error_.clear();
        lift_driver_status_time_ = {};
        lift_control_status_time_ = {};
        lift_joint_state_time_ = {};
        ++lift_driver_status_generation_;
        lift_feedback_condition_.notify_all();
      });
  }

  static double feedbackAge(
    bool received, const std::chrono::steady_clock::time_point & time,
    const std::chrono::steady_clock::time_point & now)
  {
    if (!received) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(now - time).count();
  }

  diagnostic_msgs::msg::DiagnosticStatus publishLiftSnapshot()
  {
    LiftDriverStatus driver;
    LiftControlStatus control;
    bool driver_received;
    bool driver_valid;
    bool control_received;
    bool control_valid;
    bool joint_received;
    bool joint_valid;
    bool joint_velocity_available;
    double joint_position;
    double joint_velocity;
    double driver_age;
    double control_age;
    double joint_age;
    std::uint64_t driver_generation;
    std::string driver_error;
    std::string control_error;
    std::string joint_error;
    const auto steady_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(lift_feedback_mutex_);
      driver = lift_driver_status_;
      control = lift_control_status_;
      driver_received = lift_driver_status_received_;
      driver_valid = lift_driver_status_valid_;
      control_received = lift_control_status_received_;
      control_valid = lift_control_status_valid_;
      joint_received = lift_joint_state_received_;
      joint_valid = lift_joint_state_valid_;
      joint_velocity_available = lift_joint_velocity_available_;
      joint_position = lift_joint_position_;
      joint_velocity = lift_joint_velocity_;
      driver_error = lift_driver_status_error_;
      control_error = lift_control_status_error_;
      joint_error = lift_joint_state_error_;
      driver_generation = lift_driver_status_generation_;
      driver_age = feedbackAge(driver_received, lift_driver_status_time_, steady_now);
      control_age = feedbackAge(control_received, lift_control_status_time_, steady_now);
      joint_age = feedbackAge(joint_received, lift_joint_state_time_, steady_now);
    }

    const bool driver_fresh = driver_received && driver_valid &&
      driver_age <= lift_status_stale_sec_;
    const bool control_fresh = control_received && control_valid &&
      control_age <= lift_status_stale_sec_;
    const bool joint_fresh = joint_received && joint_valid &&
      joint_age <= lift_joint_state_stale_sec_;
    const bool valid = driver_fresh && control_fresh && joint_fresh && driver.feedback_fresh &&
      driver.ethercat_operational && driver.working_counter_ok && driver.initialized;

    std::vector<std::string> issues;
    if (!driver_received) {
      issues.emplace_back("lift driver status waiting");
    } else if (!driver_valid) {
      issues.emplace_back("lift driver status invalid: " + driver_error);
    } else if (driver_age > lift_status_stale_sec_) {
      issues.emplace_back("lift driver status stale");
    }
    if (!control_received) {
      issues.emplace_back("lift control status waiting");
    } else if (!control_valid) {
      issues.emplace_back("lift control status invalid: " + control_error);
    } else if (control_age > lift_status_stale_sec_) {
      issues.emplace_back("lift control status stale");
    }
    if (!joint_received) {
      issues.emplace_back("lift joint state waiting");
    } else if (!joint_valid) {
      issues.emplace_back("lift joint state invalid: " + joint_error);
    } else if (joint_age > lift_joint_state_stale_sec_) {
      issues.emplace_back("lift joint state stale");
    }
    if (driver_fresh && !driver.feedback_fresh) {
      issues.emplace_back("lift PDO feedback stale");
    }
    if (driver_fresh && !driver.ethercat_operational) {
      issues.emplace_back("lift EtherCAT not operational");
    }
    if (driver_fresh && !driver.working_counter_ok) {
      issues.emplace_back("lift WorkingCounter invalid");
    }
    if (driver_fresh && !driver.initialized) {
      issues.emplace_back("lift hardware not initialized");
    }

    robot_control_msg::msg::LiftStatus message;
    message.stamp = now();
    message.joint_name = lift_joint_name_;
    message.valid = valid;
    message.feedback_fresh = valid;
    message.ethercat_operational = driver_fresh && driver.ethercat_operational;
    message.working_counter_ok = driver_fresh && driver.working_counter_ok;
    message.initialized = driver_fresh && driver.initialized;
    message.command_enabled = driver_fresh && driver.command_enabled;
    message.enabled = driver_fresh && driver.enabled;
    message.brake_unlocked = driver_fresh && driver.brake_unlocked;
    message.estop = driver_fresh && driver.estop_latched;
    message.quick_stop = driver_fresh && driver.quick_stop_active;
    message.trajectory_active = control_fresh && control.trajectory_active;
    message.jog_active = control_fresh && control.jog_active;
    const auto lift_safety = evaluateLiftSafety(driver);
    message.fault = driver_fresh && lift_safety.fault;
    message.motion_blocked = !valid || (driver_fresh && lift_safety.motion_blocked);
    message.status_word = driver.status_word;
    message.error_code = static_cast<std::int32_t>(driver.error_code);
    message.cia402_state = driver.cia402_state;
    message.controller_mode = control_fresh ? control.mode : std::string();
    message.fault_reason = driver_fresh ? lift_safety.fault_reason : std::string();
    // A controller-latched fault used to surface with fault_reason='' because this
    // field is composed from the drive-side safety status, which stays empty when
    // the drive has no fault code of its own (603Fh = 0).  Prefer the controller's
    // own reason so a latched fault is never reported without an explanation.
    if (message.fault_reason.empty() && control_fresh && !control.fault_reason.empty()) {
      message.fault_reason = control.fault_reason;
    }
    if (issues.empty()) {
      if (message.motion_blocked) {
        message.message = message.fault_reason.empty() ?
          "lift motion blocked by the current safety state" :
          "lift motion blocked: " + message.fault_reason;
      } else {
        std::ostringstream diagnostic;
        diagnostic << "lift feedback ready"
                   << "; measured_position=" << joint_position
                   << "; measured_velocity=" << joint_velocity
                   << "; command_position=" << driver.command_position
                   << "; command_velocity=" << driver.command_velocity
                   << "; controller_mode=" << message.controller_mode;
        if (driver.pdo_target_available) {
          diagnostic << "; pdo_target_rpm=" << driver.target_rpm
                     << "; pdo_target_velocity_units=" << driver.target_velocity_units
                     << "; velocity_mode_active=" <<
            (driver.velocity_mode_active ? "true" : "false");
        } else {
          diagnostic << "; pdo_target=unavailable";
        }
        message.message = diagnostic.str();
      }
    } else {
      std::ostringstream stream;
      for (std::size_t index = 0; index < issues.size(); ++index) {
        if (index != 0) {
          stream << "; ";
        }
        stream << issues[index];
      }
      message.message = stream.str();
      if (message.fault_reason.empty()) {
        message.fault_reason = message.message;
      } else {
        message.fault_reason += "; " + message.message;
      }
    }
    lift_status_publisher_->publish(message);
    const auto status_generation = ++lift_status_generation_;
    if (lift_last_publish_steady_time_.time_since_epoch().count() != 0) {
      const auto interval = std::chrono::duration<double>(
        steady_now - lift_last_publish_steady_time_).count();
      if (interval > 0.0) {
        lift_measured_publish_rate_hz_ = 1.0 / interval;
      }
    }
    lift_last_publish_steady_time_ = steady_now;
    lift_last_publish_ros_time_ = message.stamp;

    if (joint_fresh) {
      sensor_msgs::msg::JointState joint_message;
      joint_message.header.stamp = now();
      joint_message.name = {lift_joint_name_};
      joint_message.position = {joint_position};
      if (joint_velocity_available) {
        joint_message.velocity = {joint_velocity};
      }
      lift_joint_state_publisher_->publish(joint_message);
    }

    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "ubuntu_lower_gateway/lift";
    diagnostic.hardware_id = lift_joint_name_;
    diagnostic.level = valid && !message.motion_blocked ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.message = message.message;
    diagnostic.values.push_back(keyValue("driver_feedback_age", ageText(driver_age)));
    diagnostic.values.push_back(keyValue("control_feedback_age", ageText(control_age)));
    diagnostic.values.push_back(keyValue("joint_feedback_age", ageText(joint_age)));
    diagnostic.values.push_back(keyValue("driver_parse_valid", boolText(driver_valid)));
    diagnostic.values.push_back(keyValue("driver_parse_error", driver_error));
    diagnostic.values.push_back(keyValue("control_parse_valid", boolText(control_valid)));
    diagnostic.values.push_back(keyValue("control_parse_error", control_error));
    diagnostic.values.push_back(keyValue("joint_parse_valid", boolText(joint_valid)));
    diagnostic.values.push_back(keyValue("joint_parse_error", joint_error));
    diagnostic.values.push_back(
      keyValue("driver_generation", std::to_string(driver_generation)));
    diagnostic.values.push_back(
      keyValue("status_generation", std::to_string(status_generation)));
    diagnostic.values.push_back(
      keyValue("published_count", std::to_string(status_generation)));
    diagnostic.values.push_back(
      keyValue(
        "last_publish_time",
        std::to_string(lift_last_publish_ros_time_.sec) + "." +
        std::to_string(lift_last_publish_ros_time_.nanosec)));
    diagnostic.values.push_back(
      keyValue("publish_rate_hz", std::to_string(lift_measured_publish_rate_hz_)));
    diagnostic.values.push_back(keyValue("valid", boolText(message.valid)));
    diagnostic.values.push_back(
      keyValue("feedback_fresh", boolText(message.feedback_fresh)));
    diagnostic.values.push_back(
      keyValue("ethercat_operational", boolText(message.ethercat_operational)));
    diagnostic.values.push_back(
      keyValue("working_counter", std::to_string(driver_fresh ? driver.working_counter : 0)));
    diagnostic.values.push_back(
      keyValue("working_counter_ok", boolText(message.working_counter_ok)));
    diagnostic.values.push_back(keyValue("initialized", boolText(message.initialized)));
    diagnostic.values.push_back(keyValue("enabled", boolText(message.enabled)));
    diagnostic.values.push_back(
      keyValue("brake_unlocked", boolText(message.brake_unlocked)));
    diagnostic.values.push_back(keyValue("fault", boolText(message.fault)));
    diagnostic.values.push_back(keyValue("fault_reason", message.fault_reason));
    diagnostic.values.push_back(keyValue("degrade_reason", valid ? "" : message.message));
    return diagnostic;
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

    const auto lift_diagnostic = publishLiftSnapshot();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    array.status.push_back(lift_diagnostic);
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
  double lift_status_stale_sec_{0.5};
  double lift_joint_state_stale_sec_{0.5};

  bool enable_command_proxy_{false};
  std::string command_proxy_prefix_{"~/"};
  std::string native_power_service_;
  std::string native_mode_service_;
  std::string native_joint_batch_service_;
  std::string native_joint_absolute_service_;
  std::string native_cartesian_increment_service_;
  std::string native_cartesian_absolute_service_;
  std::string native_lift_power_service_;
  std::string native_lift_command_service_;
  std::string native_lift_stop_service_;
  std::string native_lift_hold_service_;
  std::string native_lift_set_drive_zero_service_;
  std::string native_lift_host_zero_service_;
  std::chrono::milliseconds power_timeout_{30000};
  std::chrono::milliseconds mode_timeout_{5000};
  std::chrono::milliseconds joint_timeout_{30000};
  std::chrono::milliseconds cartesian_timeout_{60000};
  std::chrono::milliseconds lift_power_timeout_{30000};
  std::chrono::milliseconds lift_command_timeout_{5000};
  std::chrono::milliseconds lift_safety_timeout_{5000};
  std::chrono::milliseconds lift_set_drive_zero_timeout_{10000};
  std::chrono::milliseconds lift_host_zero_timeout_{10000};

  std::mutex command_mutex_;
  rclcpp::CallbackGroup::SharedPtr command_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<SetRobotPower>::SharedPtr power_client_;
  rclcpp::Client<SetArmControlMode>::SharedPtr mode_client_;
  rclcpp::Client<JointBatchControl>::SharedPtr joint_batch_client_;
  rclcpp::Client<JointAbsoluteControl>::SharedPtr joint_absolute_client_;
  rclcpp::Client<CartesianIncrementControl>::SharedPtr cartesian_increment_client_;
  rclcpp::Client<CartesianAbsoluteControl>::SharedPtr cartesian_absolute_client_;
  rclcpp::Client<SetBool>::SharedPtr lift_power_client_;
  rclcpp::Client<SelectedJointControl>::SharedPtr lift_command_client_;
  rclcpp::Client<Trigger>::SharedPtr lift_stop_client_;
  rclcpp::Client<Trigger>::SharedPtr lift_hold_client_;
  rclcpp::Client<Trigger>::SharedPtr lift_set_drive_zero_client_;
  rclcpp::Client<Trigger>::SharedPtr lift_host_zero_client_;
  rclcpp::Service<SetRobotPower>::SharedPtr power_proxy_service_;
  rclcpp::Service<SetArmControlMode>::SharedPtr mode_proxy_service_;
  rclcpp::Service<JointBatchControl>::SharedPtr joint_batch_proxy_service_;
  rclcpp::Service<JointAbsoluteControl>::SharedPtr joint_absolute_proxy_service_;
  rclcpp::Service<CartesianIncrementControl>::SharedPtr cartesian_increment_proxy_service_;
  rclcpp::Service<CartesianAbsoluteControl>::SharedPtr cartesian_absolute_proxy_service_;
  rclcpp::Service<SetBool>::SharedPtr lift_power_proxy_service_;
  rclcpp::Service<SelectedJointControl>::SharedPtr lift_command_proxy_service_;
  rclcpp::Service<Trigger>::SharedPtr lift_stop_proxy_service_;
  rclcpp::Service<Trigger>::SharedPtr lift_hold_proxy_service_;
  rclcpp::Service<Trigger>::SharedPtr lift_set_drive_zero_proxy_service_;
  rclcpp::Service<Trigger>::SharedPtr lift_host_zero_proxy_service_;

  std::string lift_joint_name_{"joint_motor"};
  std::string native_lift_joint_name_{"joint_motor"};
  std::string native_lift_driver_status_topic_;
  std::string native_lift_control_status_topic_;
  std::string native_lift_joint_state_topic_;
  std::string workspace_status_topic_;
  std::mutex lift_feedback_mutex_;
  std::condition_variable lift_feedback_condition_;
  std::uint64_t lift_driver_status_generation_{0};
  std::uint64_t lift_status_generation_{0};
  double lift_measured_publish_rate_hz_{0.0};
  std::chrono::steady_clock::time_point lift_last_publish_steady_time_;
  builtin_interfaces::msg::Time lift_last_publish_ros_time_;
  bool lift_workspace_state_received_{false};
  std::uint8_t lift_workspace_state_{robot_control_msg::msg::WorkspaceStatus::STOPPED};
  LiftDriverStatus lift_driver_status_;
  LiftControlStatus lift_control_status_;
  bool lift_driver_status_received_{false};
  bool lift_driver_status_valid_{false};
  bool lift_control_status_received_{false};
  bool lift_control_status_valid_{false};
  bool lift_joint_state_received_{false};
  bool lift_joint_state_valid_{false};
  bool lift_joint_velocity_available_{false};
  double lift_joint_position_{0.0};
  double lift_joint_velocity_{0.0};
  std::string lift_driver_status_error_;
  std::string lift_control_status_error_;
  std::string lift_joint_state_error_;
  std::chrono::steady_clock::time_point lift_driver_status_time_;
  std::chrono::steady_clock::time_point lift_control_status_time_;
  std::chrono::steady_clock::time_point lift_joint_state_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lift_driver_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lift_control_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lift_joint_state_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::WorkspaceStatus>::SharedPtr
    lift_workspace_status_subscription_;
  rclcpp::Publisher<robot_control_msg::msg::LiftStatus>::SharedPtr lift_status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lift_joint_state_publisher_;

  bool last_diagnostic_valid_{false};
  std::uint8_t last_diagnostic_level_{0};
  std::string last_diagnostic_message_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    canonical_joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<HeavyGatewayV1> heavy_gateway_v1_;
};

}  // namespace robot_lower_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_lower_gateway::RobotLowerGatewayNode>();
    // A proxy callback waits for a response from a native service. Keep at
    // least one executor thread available for the client response callback.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
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

#include "robot_lower_gateway/heavy_gateway_v1.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "robot_lower_gateway/junior_gateway_contract.hpp"
#include "robot_lower_gateway/native_service_forwarder.hpp"

namespace robot_lower_gateway
{
namespace
{

using namespace std::chrono_literals;
namespace contract = junior_gateway::junior_heavy::v1;

constexpr std::uint16_t kProtocolMajor = 1;
constexpr std::uint16_t kProtocolMinor = 0;
constexpr char kLayoutSha256[] =
  "d410df9c8dc715bbea59a1684bb1420ef1dc679185bbfce46642b25d58786e35";
constexpr char kInternalCommandTopic[] =
  "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command";
constexpr char kLeaseGateTopic[] =
  "/ubuntu_lower_gateway/internal/heavy/v1/lease_active";
constexpr char kArmAckTopic[] =
  "/ubuntu_lower_gateway/internal/heavy/v1/arm_applied_sequence";
constexpr char kLiftAckTopic[] =
  "/ubuntu_lower_gateway/internal/heavy/v1/lift_applied_sequence";

template<typename DurationT>
bool isFresh(
  bool received, const std::chrono::steady_clock::time_point & stamp,
  const std::chrono::steady_clock::time_point & now, DurationT timeout)
{
  return received && stamp.time_since_epoch().count() != 0 && now >= stamp &&
         now - stamp <= timeout;
}

std::chrono::milliseconds positiveMilliseconds(
  rclcpp::Node & node, const std::string & name, std::int64_t default_value)
{
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0 || value > 600000) {
    throw std::invalid_argument(name + " must be in (0, 600000] milliseconds");
  }
  return std::chrono::milliseconds(value);
}

double positiveRate(rclcpp::Node & node, const std::string & name, double default_value)
{
  const auto value = node.declare_parameter<double>(name, default_value);
  if (!std::isfinite(value) || value <= 0.0 || value > 1000.0) {
    throw std::invalid_argument(name + " must be finite and in (0, 1000]");
  }
  return value;
}

}  // namespace

HeavyGatewayV1::HeavyGatewayV1(rclcpp::Node & node)
: node_(node), logger_(node.get_logger()), started_at_(std::chrono::steady_clock::now())
{
  configureParameters();
  configureRosInterfaces();
  RCLCPP_INFO(
    logger_,
    "Junior Heavy Gateway V1 ready: resources=15 layout_crc32=0x%08x schema=%s",
    contract::kLayoutCrc32, contract::kInterfaceSchemaSha256.data());
}

void HeavyGatewayV1::configureParameters()
{
  command_rate_hz_ = positiveRate(node_, "heavy_v1.command_rate_hz", 100.0);
  controller_rate_hz_ = positiveRate(node_, "heavy_v1.controller_rate_hz", 100.0);
  feedback_rate_hz_ = positiveRate(node_, "heavy_v1.feedback_rate_hz", 100.0);
  status_rate_hz_ = positiveRate(node_, "heavy_v1.status_rate_hz", 10.0);
  command_hold_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.command_hold_timeout_ms", 100);
  feedback_fault_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.feedback_fault_timeout_ms", 300);
  workspace_status_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.workspace_status_timeout_ms", 3000);
  execution_lease_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.execution_lease_timeout_ms", 500);
  startup_feedback_grace_ = positiveMilliseconds(
    node_, "heavy_v1.startup_feedback_grace_ms", 2000);
  controller_ack_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.controller_ack_timeout_ms", 200);
  controller_ack_retry_interval_ = positiveMilliseconds(
    node_, "heavy_v1.controller_ack_retry_interval_ms", 50);
  service_timeout_ = positiveMilliseconds(node_, "heavy_v1.service_timeout_ms", 5000);
  lift_enable_timeout_ = positiveMilliseconds(
    node_, "heavy_v1.lift_enable_timeout_ms", 1000);
  arm_max_velocity_rad_s_ = positiveRate(
    node_, "heavy_v1.arm_max_velocity_rad_s", 0.3);
  arm_max_acceleration_rad_s2_ = positiveRate(
    node_, "heavy_v1.arm_max_acceleration_rad_s2", 1.0);
  lift_min_position_m_ = node_.declare_parameter<double>(
    "heavy_v1.lift_min_position_m", -1.0);
  lift_max_position_m_ = node_.declare_parameter<double>(
    "heavy_v1.lift_max_position_m", 0.0);
  lift_position_tolerance_m_ = node_.declare_parameter<double>(
    "heavy_v1.lift_position_tolerance_m", 0.002);
  lift_max_velocity_mps_ = positiveRate(
    node_, "heavy_v1.lift_max_velocity_mps", 0.060);
  lift_max_acceleration_mps2_ = positiveRate(
    node_, "heavy_v1.lift_max_acceleration_mps2", 0.10);
  allow_lift_only_follow_ = node_.declare_parameter<bool>(
    "heavy_v1.allow_lift_only_follow", false);
  lift_only_arm_hold_position_tolerance_rad_ = node_.declare_parameter<double>(
    "heavy_v1.lift_only_arm_hold_position_tolerance_rad", 0.001);
  brake_lock_velocity_threshold_mps_ = positiveRate(
    node_, "heavy_v1.brake_lock_velocity_threshold_mps", 0.001);
  if (brake_lock_velocity_threshold_mps_ > lift_max_velocity_mps_) {
    throw std::invalid_argument(
            "heavy_v1.brake_lock_velocity_threshold_mps exceeds lift maximum velocity");
  }
  if (controller_ack_retry_interval_ >= controller_ack_timeout_) {
    throw std::invalid_argument(
            "heavy_v1.controller_ack_retry_interval_ms must be less than "
            "heavy_v1.controller_ack_timeout_ms");
  }
  if (!std::isfinite(lift_min_position_m_) || !std::isfinite(lift_max_position_m_) ||
    lift_min_position_m_ >= lift_max_position_m_)
  {
    throw std::invalid_argument("heavy_v1 lift position limits must be finite and ordered");
  }
  if (lift_min_position_m_ < -1.0 || lift_max_position_m_ > 0.0 ||
    lift_max_velocity_mps_ > 0.060 || lift_max_acceleration_mps2_ > 0.10)
  {
    throw std::invalid_argument(
      "heavy_v1 lift limits exceed the native lift controller safety limits");
  }
  if (!std::isfinite(lift_position_tolerance_m_) || lift_position_tolerance_m_ < 0.0 ||
    lift_position_tolerance_m_ > 0.01)
  {
    throw std::invalid_argument(
      "heavy_v1.lift_position_tolerance_m must be finite and in [0, 0.01]");
  }
  if (!std::isfinite(lift_only_arm_hold_position_tolerance_rad_) ||
    lift_only_arm_hold_position_tolerance_rad_ <= 0.0 ||
    lift_only_arm_hold_position_tolerance_rad_ > 0.01)
  {
    throw std::invalid_argument(
            "heavy_v1.lift_only_arm_hold_position_tolerance_rad must be in (0, 0.01]");
  }

  const std::vector<std::string> default_arm_names = {
    "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
    "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
  arm_native_names_ = node_.declare_parameter<std::vector<std::string>>(
    "heavy_v1.arm_native_names", default_arm_names);
  lift_native_name_ = node_.declare_parameter<std::string>(
    "heavy_v1.lift_native_name", "joint_motor");
  heavy_lift_brake_native_service_ = node_.declare_parameter<std::string>(
    "heavy_v1.heavy_lift_brake_native_service", "");
  if (arm_native_names_.size() != 14 || lift_native_name_.empty()) {
    throw std::invalid_argument("heavy_v1 requires 14 arm_native_names and one lift name");
  }
}

void HeavyGatewayV1::configureRosInterfaces()
{
  command_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  watchdog_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  feedback_publication_callback_group_ = node_.create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  status_publication_callback_group_ = node_.create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  service_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  feedback_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions command_options;
  command_options.callback_group = command_callback_group_;
  rclcpp::SubscriptionOptions feedback_options;
  feedback_options.callback_group = feedback_callback_group_;

  rclcpp::QoS command_qos(rclcpp::KeepLast(1));
  command_qos.best_effort().durability_volatile();
  command_qos.deadline(rclcpp::Duration(0, 20000000));
  command_qos.lifespan(rclcpp::Duration(0, 50000000));
  rclcpp::QoS feedback_qos(rclcpp::KeepLast(1));
  feedback_qos.best_effort().durability_volatile();
  feedback_qos.deadline(rclcpp::Duration(0, 20000000));
  rclcpp::QoS status_qos(rclcpp::KeepLast(1));
  status_qos.reliable().transient_local();
  rclcpp::QoS internal_command_qos(rclcpp::KeepLast(1));
  // The external 100 Hz command stream remains BEST_EFFORT. Once a frame is
  // admitted, however, the one-shot controller transaction must not be lost:
  // aligned HOLD in particular has no preceding command that can replace it.
  internal_command_qos.reliable().durability_volatile();
  rclcpp::QoS gate_qos(rclcpp::KeepLast(1));
  gate_qos.reliable().transient_local();

  feedback_publisher_ = node_.create_publisher<Feedback>(
    contract::kFeedbackTopic.data(), feedback_qos);
  status_publisher_ = node_.create_publisher<Status>(contract::kStatusTopic.data(), status_qos);
  internal_command_publisher_ = node_.create_publisher<Command>(
    kInternalCommandTopic, internal_command_qos);
  lease_gate_publisher_ = node_.create_publisher<std_msgs::msg::Bool>(kLeaseGateTopic, gate_qos);
  command_subscription_ = node_.create_subscription<Command>(
    contract::kCommandTopic.data(), command_qos,
    [this](const Command::SharedPtr message) {handleCommand(message);}, command_options);

  arm_joint_subscription_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    "/arm/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {handleArmJointState(message);},
    feedback_options);
  lift_joint_subscription_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    "/lift/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {handleLiftJointState(message);},
    feedback_options);
  arm_power_subscription_ = node_.create_subscription<robot_control_msg::msg::ArmPowerStatus>(
    "/arm/power_status", status_qos,
    [this](const robot_control_msg::msg::ArmPowerStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      arm_power_ = *message;
      arm_power_received_ = true;
      arm_power_time_ = std::chrono::steady_clock::now();
    }, feedback_options);
  lift_status_subscription_ = node_.create_subscription<robot_control_msg::msg::LiftStatus>(
    "/ubuntu_lower_gateway/lift/status", status_qos,
    [this](const robot_control_msg::msg::LiftStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      lift_status_ = *message;
      lift_status_received_ = true;
      lift_status_time_ = std::chrono::steady_clock::now();
      // Before this gateway has committed a native transition, LiftStatus is
      // authoritative.  Once a lock was committed, ignore an older retained
      // "unlocked" sample: accepting it would reopen the 100 Hz HOLD path and
      // submit the same physical lock repeatedly.
      if (!brake_lock_commanded_) {
        lift_brake_locked_ = !message->brake_unlocked;
      }
      state_condition_.notify_all();
    }, feedback_options);
  workspace_subscription_ = node_.create_subscription<robot_control_msg::msg::WorkspaceStatus>(
    "/workspace/status", status_qos,
    [this](const robot_control_msg::msg::WorkspaceStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool entering_simulation =
        message->state == robot_control_msg::msg::WorkspaceStatus::RUNNING &&
        message->mode == robot_control_msg::msg::WorkspaceStatus::SIMULATION &&
        (!workspace_received_ ||
        workspace_status_.state != robot_control_msg::msg::WorkspaceStatus::RUNNING ||
        workspace_status_.mode != robot_control_msg::msg::WorkspaceStatus::SIMULATION);
      workspace_status_ = *message;
      workspace_received_ = true;
      workspace_time_ = std::chrono::steady_clock::now();
      if (entering_simulation) {
        // Never label feedback from a previous REAL or SIM run as current SIM data.
        // New source samples must re-establish every validity bit.
        joints_ = JointSnapshot{};
        arm_power_ = robot_control_msg::msg::ArmPowerStatus{};
        lift_status_ = robot_control_msg::msg::LiftStatus{};
        arm_power_received_ = false;
        lift_status_received_ = false;
        arm_power_time_ = {};
        lift_status_time_ = {};
      }
    }, feedback_options);
  control_mode_subscription_ =
    node_.create_subscription<robot_control_msg::msg::ArmControlModeStatus>(
    "/arm/control_mode_status", status_qos,
    [this](const robot_control_msg::msg::ArmControlModeStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      control_mode_status_ = *message;
      control_mode_received_ = true;
      control_mode_time_ = std::chrono::steady_clock::now();
    }, feedback_options);
  arm_ack_subscription_ = node_.create_subscription<std_msgs::msg::UInt64>(
    kArmAckTopic, rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::UInt64::SharedPtr message) {handleArmAck(message);}, command_options);
  lift_ack_subscription_ = node_.create_subscription<std_msgs::msg::UInt64>(
    kLiftAckTopic, rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::UInt64::SharedPtr message) {handleLiftAck(message);}, command_options);

  capabilities_service_ = node_.create_service<Capabilities>(
    contract::kCapabilitiesService.data(),
    [this](const Capabilities::Request::SharedPtr request,
    Capabilities::Response::SharedPtr response) {handleCapabilities(request, response);},
    rmw_qos_profile_services_default, service_callback_group_);
  acquire_service_ = node_.create_service<AcquireLease>(
    contract::kAcquireLeaseService.data(),
    [this](const AcquireLease::Request::SharedPtr request,
    AcquireLease::Response::SharedPtr response) {handleAcquire(request, response);},
    rmw_qos_profile_services_default, service_callback_group_);
  release_service_ = node_.create_service<ReleaseLease>(
    contract::kReleaseLeaseService.data(),
    [this](const ReleaseLease::Request::SharedPtr request,
    ReleaseLease::Response::SharedPtr response) {handleRelease(request, response);},
    rmw_qos_profile_services_default, service_callback_group_);
  lift_brake_service_ = node_.create_service<LiftBrake>(
    contract::kLiftBrakeService.data(),
    [this](const LiftBrake::Request::SharedPtr request,
    LiftBrake::Response::SharedPtr response) {handleLiftBrake(request, response);},
    rmw_qos_profile_services_default, service_callback_group_);

  arm_power_client_ = node_.create_client<robot_control_msg::srv::SetRobotPower>(
    contract::kTorqueService.data(), rmw_qos_profile_services_default, service_callback_group_);
  if (!heavy_lift_brake_native_service_.empty()) {
    heavy_lift_brake_client_ = node_.create_client<std_srvs::srv::SetBool>(
      heavy_lift_brake_native_service_, rmw_qos_profile_services_default, service_callback_group_);
  }

  watchdog_timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / controller_rate_hz_)),
    [this]() {watchdogAndFeedbackTick();}, watchdog_callback_group_);
  feedback_timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / feedback_rate_hz_)),
    [this]() {publishFeedback();}, feedback_publication_callback_group_);
  status_timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / status_rate_hz_)),
    [this]() {publishStatus();}, status_publication_callback_group_);
  publishLeaseGate(false);
}

void HeavyGatewayV1::handleCapabilities(
  const Capabilities::Request::SharedPtr request, Capabilities::Response::SharedPtr response)
{
  response->compatible = request->requested_protocol_major == kProtocolMajor;
  response->protocol_major = kProtocolMajor;
  response->protocol_minor = kProtocolMinor;
  response->layout_crc32 = contract::kLayoutCrc32;
  response->layout_sha256 = kLayoutSha256;
  response->resource_count = contract::kResourceCount;
  response->command_rate_hz = command_rate_hz_;
  response->feedback_rate_hz = feedback_rate_hz_;
  response->command_timeout_ms = command_hold_timeout_.count();
  response->lease_timeout_ms = execution_lease_timeout_.count();
  response->supports_position = true;
  response->supports_velocity_feedforward = true;
  response->supports_acceleration_feedforward = true;
  response->supports_effort_command = false;
  response->supports_mit_stream = false;
  response->has_end_effectors = false;
  response->hardware_calibration_required = false;
  response->arm_power_is_torque_enable = true;
  response->arm_power_affects_lift = false;
  response->lift_brake_state_required = false;
  response->lift_brake_command_optional = true;
  response->interface_schema_sha256 = std::string(contract::kInterfaceSchemaSha256);
  response->message = response->compatible ?
    "Junior Heavy Gateway V1 compatible; per-axis position, velocity, and acceleration feedforward supported"
    :
    "requested protocol major is incompatible";
}

std::string HeavyGatewayV1::generateLeaseId(const std::string & owner)
{
  std::random_device random;
  std::ostringstream stream;
  stream << owner << '-' << std::hex << random() << '-'
         << std::chrono::steady_clock::now().time_since_epoch().count() << '-'
         << lease_generation_;
  return stream.str();
}

void HeavyGatewayV1::handleAcquire(
  const AcquireLease::Request::SharedPtr request, AcquireLease::Response::SharedPtr response)
{
  Command alignment;
  bool enable_lift = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    std::string reason;
    if (request->owner_id.empty()) {
      response->message = "owner_id must not be empty";
      return;
    }
    if (request->protocol_major != kProtocolMajor) {
      response->message = "protocol major mismatch";
      return;
    }
    if (request->protocol_minor > kProtocolMinor) {
      response->message = "protocol minor is newer than supported";
      return;
    }
    if (request->layout_crc32 != contract::kLayoutCrc32) {
      response->message = "layout CRC mismatch";
      return;
    }
    if (lease_active_ || lease_acquisition_in_progress_) {
      response->message = "execution lease already held by owner='" + owner_id_ +
        "' lease='" + lease_id_ + "' state=" + std::to_string(lease_.state) +
        "; lower detail=" + lease_.detail +
        "; release the reported owner/lease before acquire";
      return;
    }
    if (!workspaceReadyLocked(now, reason) || !feedbackReadyLocked(now, reason)) {
      response->message = "execution lease unavailable: " + reason;
      return;
    }
    if (!controllerEndpointsReadyLocked(reason)) {
      response->message = "execution lease unavailable: " + reason;
      return;
    }
    if (!liftEnableReadyLocked(now, reason)) {
      response->message = "execution lease unavailable: " + reason;
      return;
    }
    if (!liftPowerEnabledLocked(now) && !heavy_lift_brake_client_) {
      response->message =
        "execution lease unavailable: lift power is disabled and the native enable service is not configured";
      return;
    }
    const bool real_workspace = workspace_status_.mode ==
      robot_control_msg::msg::WorkspaceStatus::REAL;
    lease_acquisition_in_progress_ = true;
    enable_lift = real_workspace && !liftPowerEnabledLocked(now);
    if (enable_lift) {
      brake_transition_in_progress_ = true;
      detail_ = "preparing zero-velocity lift enable before lease grant";
    }
  }

  if (enable_lift && !executeBrakeAction({true, "lease preparation: lift power enable"})) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lease_acquisition_in_progress_ = false;
      response->message = "execution lease unavailable: lift power enable was not confirmed";
    }
    publishStatus(true);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    lease_acquisition_in_progress_ = false;
    const auto now = std::chrono::steady_clock::now();
    std::string reason;
    const bool real_workspace = workspace_status_.mode ==
      robot_control_msg::msg::WorkspaceStatus::REAL;
    if (lease_active_ || !workspaceReadyLocked(now, reason) || !feedbackReadyLocked(now, reason) ||
      !controllerEndpointsReadyLocked(reason) || !liftEnableReadyLocked(now, reason) ||
      (real_workspace && !liftPowerEnabledLocked(now)))
    {
      response->message = "execution lease unavailable after lift preparation: " +
        (reason.empty() ? "lift power is not confirmed" : reason);
      return;
    }
    ++lease_generation_;
    initial_sequence_ = lease_generation_ * 1000000ULL + 1ULL;
    last_command_sequence_ = initial_sequence_ - 1ULL;
    owner_id_ = request->owner_id;
    lease_id_ = generateLeaseId(owner_id_);
    lease_active_ = true;
    aligned_hold_ = false;
    first_user_command_for_lease_ = true;
    last_mode_ = Command::MODE_STOP;
    // The lease watchdog starts with the first accepted command.  Acquiring a
    // lease may legitimately spend time aligning the initial HOLD.
    lease_deadline_ = {};
    last_command_time_ = {};
    state_ = Status::STATE_MONITOR;
    detail_ = "lease granted; aligning HOLD to current feedback";
    alignment = makeHoldCommandLocked(last_command_sequence_);
    pending_command_sequence_ = ++controller_command_generation_;
    pending_external_command_sequence_ = 0;
    pending_lift_target_m_ = alignment.position[0];
    alignment.sequence = pending_command_sequence_;
    pending_internal_command_ = alignment;
    last_pending_command_publish_time_ = now;
    controller_ack_retry_count_ = 0;
    pending_command_valid_ = true;
    pending_command_is_user_ = false;
    staged_command_valid_ = false;
    revoke_after_ack_ = false;
    arm_ack_sequence_ = 0;
    lift_ack_sequence_ = 0;
    pending_command_time_ = now;
    recordLeaseEventLocked("granted", "", now);
    RCLCPP_INFO(
      logger_,
      "Heavy V1 acquire granted: request_owner='%s' active_owner='%s' active_lease='%s' "
      "initial_sequence=%lu",
      request->owner_id.c_str(), owner_id_.c_str(), lease_id_.c_str(), initial_sequence_);
    response->granted = true;
    response->lease_id = lease_id_;
    response->lease_timeout_ms = execution_lease_timeout_.count();
    response->initial_sequence = initial_sequence_;
    response->message = "lease granted; wait for LEASED_HOLD before FOLLOW_POSITION";
  }
  publishLeaseGate(true);
  publishInternalCommand(alignment);
  publishStatus(true);
}

bool HeavyGatewayV1::validateCommandLocked(
  const Command & command, const std::chrono::steady_clock::time_point & now,
  std::string & reason, bool allow_lift_power_disabled)
{
  if (command.protocol_major != kProtocolMajor || command.protocol_minor > kProtocolMinor) {
    reason = "protocol version mismatch";
    return false;
  }
  if (command.layout_crc32 != contract::kLayoutCrc32) {
    reason = "layout CRC mismatch";
    return false;
  }
  if (!lease_active_ || command.owner_id != owner_id_ || command.lease_id != lease_id_) {
    reason = "owner or lease mismatch";
    return false;
  }
  if (command.mode > Command::MODE_STOP) {
    reason = "unsupported command mode";
    return false;
  }
  constexpr std::uint8_t allowed_fields =
    Command::FIELD_POSITION | Command::FIELD_VELOCITY | Command::FIELD_ACCELERATION;
  if ((command.field_mask & ~allowed_fields) != 0U) {
    reason = "unsupported field mask";
    return false;
  }
  if ((command.mode == Command::MODE_HOLD || command.mode == Command::MODE_FOLLOW_POSITION) &&
    (command.field_mask & Command::FIELD_POSITION) == 0U)
  {
    reason = "HOLD and FOLLOW_POSITION require position field";
    return false;
  }
  for (std::size_t index = 0; index < contract::kResourceCount; ++index) {
    if (command.mode != Command::MODE_STOP &&
      (command.field_mask & Command::FIELD_POSITION) != 0U &&
      !std::isfinite(command.position[index]))
    {
      reason = "non-finite position at resource " + std::to_string(index);
      return false;
    }
    if ((command.field_mask & Command::FIELD_VELOCITY) != 0U &&
      !std::isfinite(command.velocity[index]))
    {
      reason = "non-finite velocity at resource " + std::to_string(index);
      return false;
    }
    if ((command.field_mask & Command::FIELD_ACCELERATION) != 0U &&
      !std::isfinite(command.acceleration[index]))
    {
      reason = "non-finite acceleration at resource " + std::to_string(index);
      return false;
    }
    if (command.mode != Command::MODE_STOP &&
      (command.field_mask & Command::FIELD_VELOCITY) != 0U) {
      const double limit = index == 0 ? lift_max_velocity_mps_ : arm_max_velocity_rad_s_;
      if (std::abs(command.velocity[index]) > limit) {
        std::ostringstream stream;
        stream << "velocity exceeds resource limit at resource " << index << ": value=" <<
          command.velocity[index] << " limit=" << limit;
        reason = stream.str();
        return false;
      }
    }
    if (command.mode != Command::MODE_STOP &&
      (command.field_mask & Command::FIELD_ACCELERATION) != 0U) {
      const double limit = index == 0 ? lift_max_acceleration_mps2_ : arm_max_acceleration_rad_s2_;
      if (std::abs(command.acceleration[index]) > limit) {
        std::ostringstream stream;
        stream << "acceleration exceeds resource limit at resource " << index << ": value=" <<
          command.acceleration[index] << " limit=" << limit;
        reason = stream.str();
        return false;
      }
    }
  }
  const double lift_tolerance = command.mode == Command::MODE_HOLD ?
    lift_position_tolerance_m_ : 0.0;
  if (command.mode != Command::MODE_STOP &&
    (command.field_mask & Command::FIELD_POSITION) != 0U &&
    (command.position[0] < lift_min_position_m_ - lift_tolerance ||
     command.position[0] > lift_max_position_m_ + lift_tolerance))
  {
    std::ostringstream stream;
    stream << "lift position exceeds software travel: target=" << command.position[0]
           << " min=" << lift_min_position_m_ << " max=" << lift_max_position_m_
           << " tolerance=" << lift_tolerance << " feedback=" << joints_.position[0]
           << " sequence=" << command.sequence;
    reason = stream.str();
    return false;
  }
  if (command.mode == Command::MODE_FOLLOW_POSITION && !aligned_hold_) {
    reason = "aligned HOLD has not been confirmed";
    return false;
  }
  if (!workspaceReadyLocked(now, reason) || !feedbackReadyLocked(now, reason)) {
    return false;
  }
  if (command.mode == Command::MODE_FOLLOW_POSITION) {
    const bool simulation = workspace_status_.mode ==
      robot_control_msg::msg::WorkspaceStatus::SIMULATION;
    if (simulation) {
      // SIM has no energized drives by design.  It still requires the same
      // lease, command, feedback, and software-limit validation above, but
      // cannot satisfy REAL's physical torque/brake feedback gate.
      return true;
    }
    const bool all_arm_enabled = arm_power_.all_enabled && arm_power_.command_enabled &&
      arm_power_.enabled.size() == 14 &&
      std::all_of(
      arm_power_.enabled.begin(), arm_power_.enabled.end(), [](bool enabled) {
        return enabled;
      });
    if (!all_arm_enabled) {
      if (!allow_lift_only_follow_) {
        reason = "FOLLOW_POSITION requires all 14 arm drives enabled";
        return false;
      }
      std::string hold_reason;
      if (!armHoldCommandLocked(command, hold_reason)) {
        reason = "lift-only FOLLOW_POSITION requires safe arm hold: " + hold_reason;
        return false;
      }
    }
    if (lift_status_.motion_blocked || lift_status_.estop || lift_status_.error_code != 0)
    {
      reason = "FOLLOW_POSITION requires an unblocked lift drive";
      return false;
    }
    // `lift_brake_locked_` is latched only after a physical lock request or
    // from the canonical LiftStatus before any transition.  A FOLLOW command
    // cannot be handed to the lower controller while that latch is set if
    // this process has no native route to release it.  Otherwise the lift
    // controller correctly withholds its acknowledgement and the gateway
    // misleadingly reports a controller-ack timeout.
    if (lift_brake_locked_ && !heavy_lift_brake_client_) {
      reason =
        "FOLLOW_POSITION requires lift brake release, but native lift brake service is not configured";
      return false;
    }
    if (!allow_lift_power_disabled && !liftPowerEnabledLocked(now)) {
      reason = "FOLLOW_POSITION requires confirmed Operation Enabled and brake release";
      return false;
    }
  }
  return true;
}

bool HeavyGatewayV1::armHoldCommandLocked(
  const Command & command, std::string & reason) const
{
  constexpr std::uint8_t kRequiredFields =
    Command::FIELD_POSITION | Command::FIELD_VELOCITY | Command::FIELD_ACCELERATION;
  constexpr double kZeroTolerance = 1e-9;

  if ((command.field_mask & kRequiredFields) != kRequiredFields) {
    reason = "position, velocity, and acceleration fields are required";
    return false;
  }

  for (std::size_t index = 1; index < command.position.size(); ++index) {
    if (std::abs(command.position[index] - joints_.position[index]) >
      lift_only_arm_hold_position_tolerance_rad_)
    {
      reason = "arm target differs from measured position at resource " + std::to_string(index);
      return false;
    }
    if (std::abs(command.velocity[index]) > kZeroTolerance ||
      std::abs(command.acceleration[index]) > kZeroTolerance)
    {
      reason = "arm velocity and acceleration must be zero at resource " +
        std::to_string(index);
      return false;
    }
  }
  return true;
}

std::uint64_t HeavyGatewayV1::newestAdmittedCommandSequenceLocked() const
{
  std::uint64_t sequence = last_command_sequence_;
  if (pending_command_valid_ && pending_command_is_user_) {
    sequence = std::max(sequence, pending_external_command_sequence_);
  }
  if (staged_command_valid_) {
    sequence = std::max(sequence, staged_command_.sequence);
  }
  return sequence;
}

HeavyGatewayV1::Command HeavyGatewayV1::dispatchUserCommandLocked(
  const Command & command, const std::chrono::steady_clock::time_point & now)
{
  Command effective_command = command;
  if (effective_command.mode == Command::MODE_HOLD ||
    effective_command.mode == Command::MODE_STOP) {
    effective_command.position[0] = clampLiftSafetyTargetLocked(effective_command.position[0]);
  }
  ++accepted_commands_;
  last_command_rejection_reason_.clear();
  last_command_sequence_ = effective_command.sequence;
  lease_deadline_ = now + execution_lease_timeout_;
  last_command_time_ = now;
  lease_.watchdog_hold_captured = false;
  pending_command_sequence_ = ++controller_command_generation_;
  pending_external_command_sequence_ = effective_command.sequence;
  pending_lift_target_m_ = effective_command.position[0];
  pending_command_valid_ = true;
  pending_command_is_user_ = true;
  revoke_after_ack_ = false;
  arm_ack_sequence_ = 0;
  lift_ack_sequence_ = 0;
  pending_command_time_ = now;
  const bool follow_to_hold = last_mode_ == Command::MODE_FOLLOW_POSITION &&
    effective_command.mode == Command::MODE_HOLD;
  const bool first_hold_for_lease = first_user_command_for_lease_ &&
    effective_command.mode == Command::MODE_HOLD;
  if (follow_to_hold) {
    queueBrakeLockLocked("FOLLOW_POSITION_to_HOLD", false);
  } else if (first_hold_for_lease) {
    queueBrakeLockLocked("first_HOLD_for_owner_or_lease", false);
  } else if (effective_command.mode == Command::MODE_STOP) {
    queueBrakeLockLocked("MODE_STOP", true);
  }
  first_user_command_for_lease_ = false;
  last_mode_ = effective_command.mode;
  if (effective_command.mode == Command::MODE_FOLLOW_POSITION) {
    state_ = Status::STATE_FOLLOWING;
    detail_ = "FOLLOW_POSITION submitted to lower controllers";
  } else if (effective_command.mode == Command::MODE_STOP) {
    state_ = Status::STATE_STOPPING;
    detail_ = "STOP submitted to lower controller safety paths";
  } else {
    state_ = Status::STATE_LEASED_HOLD;
    detail_ = "HOLD submitted to lower controllers";
  }
  last_controller_submission_result_ =
    "accepted command forwarded; awaiting arm and lift acknowledgement";
  recordCommandDiagnosticLocked(effective_command, "accepted", "", now);
  auto internal_command = effective_command;
  internal_command.sequence = pending_command_sequence_;
  pending_internal_command_ = internal_command;
  last_pending_command_publish_time_ = now;
  controller_ack_retry_count_ = 0;
  return internal_command;
}

void HeavyGatewayV1::handleCommand(const Command::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::optional<BrakeAction> release_before_follow;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto received_now = std::chrono::steady_clock::now();
    ++received_commands_;
    lease_.last_received_sequence = message->sequence;
    lease_.last_received_command_time = received_now;
    const auto header_stamp = rclcpp::Time(message->header.stamp);
    last_command_header_delay_ms_ = header_stamp.nanoseconds() > 0 ?
      (node_.now() - header_stamp).seconds() * 1000.0 : kNoCommandAgeMs;
    // FOLLOW is never forwarded with a known locked brake.  Do not unlock on
    // the basis of ownership alone: an invalid, stale, or out-of-range FOLLOW
    // must remain a pure rejection and cannot actuate the native brake.
    std::string preflight_reason;
    if (message->mode == Command::MODE_FOLLOW_POSITION && lease_active_ &&
      !pending_command_valid_ && !staged_command_valid_ &&
      message->owner_id == owner_id_ && message->lease_id == lease_id_ &&
      message->sequence > newestAdmittedCommandSequenceLocked() && lift_brake_locked_ &&
      !brake_transition_in_progress_ && heavy_lift_brake_client_ &&
      validateCommandLocked(
        *message, std::chrono::steady_clock::now(), preflight_reason, true) &&
      liftEnableReadyLocked(std::chrono::steady_clock::now(), preflight_reason))
    {
      brake_transition_in_progress_ = true;
      release_before_follow = BrakeAction{true, "MODE_FOLLOW_POSITION brake release"};
    }
  }
  if (release_before_follow && !executeBrakeAction(*release_before_follow)) {
    return;
  }
  bool forward = false;
  bool status_changed = false;
  Command internal_command;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    std::string reason;
    if (state_ == Status::STATE_FAULT) {
      reason = "gateway is in controller-acknowledgement fault; explicit release is required";
      ++rejected_commands_;
      ++command_rejection_reasons_[reason];
      last_command_rejection_reason_ = reason;
      // Keep the first controller fault visible.  Fresh traffic after an ACK
      // failure is diagnostic evidence, not a reason to replace the timeout
      // detail with a generic command rejection.
      recordCommandDiagnosticLocked(*message, "rejected", reason, now);
      status_changed = true;
    } else if (!validateCommandLocked(*message, now, reason)) {
      ++rejected_commands_;
      ++command_rejection_reasons_[reason];
      last_command_rejection_reason_ = reason;
      last_controller_submission_result_ = "not submitted: " + reason;
      detail_ = "command rejected: " + reason;
      recordCommandDiagnosticLocked(*message, "rejected", reason, now);
      status_changed = true;
    } else if (message->sequence < newestAdmittedCommandSequenceLocked()) {
      ++rejected_commands_;
      ++out_of_order_commands_;
      detail_ = "out-of-order command rejected";
      ++command_rejection_reasons_["sequence is below first allowed/last accepted sequence"];
      last_command_rejection_reason_ =
        "sequence is below first allowed/last accepted sequence";
      last_controller_submission_result_ = "not submitted: out-of-order sequence";
      recordCommandDiagnosticLocked(
        *message, "rejected", "sequence is below first allowed/last accepted sequence", now);
      status_changed = true;
    } else if (message->sequence == newestAdmittedCommandSequenceLocked()) {
      ++duplicate_commands_;
      detail_ = "duplicate command ignored";
      recordCommandDiagnosticLocked(*message, "duplicate ignored", "", now);
      status_changed = true;
    } else if (pending_command_valid_) {
      if (message->mode == Command::MODE_FOLLOW_POSITION && lift_brake_locked_) {
        reason = "FOLLOW_POSITION rejected while a controller command is pending and lift brake is locked";
        ++rejected_commands_;
        ++command_rejection_reasons_[reason];
        last_command_rejection_reason_ = reason;
        last_controller_submission_result_ = "not submitted: " + reason;
        detail_ = "command rejected: " + reason;
        recordCommandDiagnosticLocked(*message, "rejected", reason, now);
      } else {
        // Keep only the newest valid command while the lower controllers own
        // the current transaction.  A staged frame is not accepted and must
        // not refresh the lease until it is actually dispatched after ACK.
        staged_command_ = *message;
        staged_command_valid_ = true;
        last_controller_submission_result_ =
          "command staged; awaiting prior arm and lift acknowledgement";
        detail_ = "command staged; lower-controller acknowledgement pending";
        recordCommandDiagnosticLocked(*message, "staged", "", now);
      }
      status_changed = true;
    } else {
      internal_command = dispatchUserCommandLocked(*message, now);
      forward = true;
      status_changed = true;
    }
  }
  if (forward) {
    publishInternalCommand(internal_command);
  }
  if (status_changed) {
    publishStatus(true);
  }
}

void HeavyGatewayV1::recordLeaseEventLocked(
  const std::string & event, const std::string & reason,
  const std::chrono::steady_clock::time_point & now) const
{
  RCLCPP_INFO(
    logger_,
    "Heavy V1 lease event=%s reason='%s' owner='%s' lease='%s' initial_sequence=%lu "
    "last_accepted_sequence=%lu applied_sequence=%lu command_age_ms=%.1f "
    "lease_remaining_ms=%.1f received=%lu accepted=%lu rejected=%lu duplicate=%lu out_of_order=%lu "
    "workspace_state=%u arm_power_received=%s lift_status_received=%s controller_submission='%s'",
    event.c_str(), reason.c_str(), owner_id_.c_str(), lease_id_.c_str(), initial_sequence_,
    last_command_sequence_, applied_command_sequence_, commandAgeMsLocked(now),
    leaseRemainingMsLocked(now), received_commands_, accepted_commands_, rejected_commands_,
    duplicate_commands_, out_of_order_commands_, static_cast<unsigned int>(workspace_status_.state),
    arm_power_received_ ? "true" : "false",
    lift_status_received_ ? "true" : "false", last_controller_submission_result_.c_str());
}

void HeavyGatewayV1::clearHeavyLeaseLocked(
  const std::string & reason, const std::chrono::steady_clock::time_point & now)
{
  if (!lease_active_) {
    return;
  }
  recordLeaseEventLocked("cleared", reason, now);
  last_lease_clear_reason_ = reason;
  lease_active_ = false;
  aligned_hold_ = false;
  owner_id_.clear();
  lease_id_.clear();
  // A revoked lease must not be revived by a delayed acknowledgement for the
  // safety HOLD which was submitted immediately before this transition.
  pending_command_valid_ = false;
  pending_command_is_user_ = false;
  staged_command_valid_ = false;
  pending_external_command_sequence_ = 0;
  pending_internal_command_ = Command{};
  last_pending_command_publish_time_ = {};
  controller_ack_retry_count_ = 0;
  arm_ack_sequence_ = 0;
  lift_ack_sequence_ = 0;
  revoke_after_ack_ = false;
  pending_lease_clear_reason_.clear();
  state_ = Status::STATE_MONITOR;
  detail_ = "lease cleared: " + reason;
}

void HeavyGatewayV1::recordCommandDiagnosticLocked(
  const Command & command, const std::string & outcome, const std::string & reason,
  const std::chrono::steady_clock::time_point & now)
{
  if (last_command_diagnostic_time_.time_since_epoch().count() != 0 &&
    now - last_command_diagnostic_time_ < std::chrono::seconds(1))
  {
    return;
  }
  last_command_diagnostic_time_ = now;
  const auto header_stamp = rclcpp::Time(command.header.stamp);
  const double header_age_ms = header_stamp.nanoseconds() > 0 ?
    (node_.now() - header_stamp).seconds() * 1000.0 : -1.0;
  std::ostringstream rejected_summary;
  bool first = true;
  for (const auto & entry : command_rejection_reasons_) {
    if (!first) {
      rejected_summary << "; ";
    }
    first = false;
    rejected_summary << entry.first << '=' << entry.second;
  }
  RCLCPP_INFO(
    logger_,
    "Heavy V1 command %s: owner='%s' lease='%s' sequence=%lu mode=%u field_mask=0x%02x "
    "protocol=%u.%u layout=0x%08x active_owner='%s' active_lease='%s' "
    "first_allowed_or_last=%lu header_age_ms=%.1f reason='%s' reject_summary='%s' "
    "lift={valid=%s fresh=%s ethercat_op=%s wkc_ok=%s initialized=%s command_enabled=%s "
    "enabled=%s brake_unlocked=%s cia402='%s' status_word=%u error_code=%d fault_reason='%s'}",
    outcome.c_str(), command.owner_id.c_str(), command.lease_id.c_str(), command.sequence,
    command.mode, command.field_mask, command.protocol_major, command.protocol_minor,
    command.layout_crc32, owner_id_.c_str(), lease_id_.c_str(), last_command_sequence_,
    header_age_ms, reason.c_str(), rejected_summary.str().c_str(),
    lift_status_.valid ? "true" : "false", lift_status_.feedback_fresh ? "true" : "false",
    lift_status_.ethercat_operational ? "true" : "false",
    lift_status_.working_counter_ok ? "true" : "false",
    lift_status_.initialized ? "true" : "false",
    lift_status_.command_enabled ? "true" : "false",
    lift_status_.enabled ? "true" : "false", lift_status_.brake_unlocked ? "true" : "false",
    lift_status_.cia402_state.c_str(), static_cast<unsigned int>(lift_status_.status_word),
    static_cast<int>(lift_status_.error_code),
    lift_status_.fault_reason.c_str());
}

HeavyGatewayV1::Command HeavyGatewayV1::makeHoldCommandLocked(std::uint64_t sequence) const
{
  Command command;
  command.header.stamp = node_.now();
  command.protocol_major = kProtocolMajor;
  command.protocol_minor = kProtocolMinor;
  command.layout_crc32 = contract::kLayoutCrc32;
  command.owner_id = owner_id_;
  command.lease_id = lease_id_;
  command.sequence = sequence;
  command.mode = Command::MODE_HOLD;
  command.field_mask = Command::FIELD_POSITION;
  command.position = joints_.position;
  command.position[0] = clampLiftSafetyTargetLocked(command.position[0]);
  command.velocity.fill(0.0);
  command.acceleration.fill(0.0);
  return command;
}

double HeavyGatewayV1::clampLiftSafetyTargetLocked(double target) const
{
  if (target > lift_max_position_m_ &&
    target <= lift_max_position_m_ + lift_position_tolerance_m_)
  {
    return lift_max_position_m_;
  }
  if (target < lift_min_position_m_ &&
    target >= lift_min_position_m_ - lift_position_tolerance_m_)
  {
    return lift_min_position_m_;
  }
  return target;
}

void HeavyGatewayV1::beginSafetyHoldLocked(const std::string & reason, bool revoke_lease)
{
  if (!lease_active_) {
    return;
  }
  const bool command_watchdog_hold = !revoke_lease &&
    reason == "command_timeout: FOLLOWING to HOLD";
  if (command_watchdog_hold && !lease_.watchdog_hold_captured) {
    lease_.watchdog_hold_position = joints_.position;
    lease_.watchdog_hold_captured = true;
  }
  auto command = makeHoldCommandLocked(last_command_sequence_);
  if (lease_.watchdog_hold_captured && (command_watchdog_hold || revoke_lease)) {
    command.position = lease_.watchdog_hold_position;
  }
  auto internal_command = command;
  pending_command_sequence_ = ++controller_command_generation_;
  pending_external_command_sequence_ = 0;
  pending_lift_target_m_ = internal_command.position[0];
  internal_command.sequence = pending_command_sequence_;
  pending_internal_command_ = internal_command;
  last_pending_command_publish_time_ = std::chrono::steady_clock::now();
  controller_ack_retry_count_ = 0;
  pending_command_valid_ = true;
  pending_command_is_user_ = false;
  staged_command_valid_ = false;
  revoke_after_ack_ = revoke_lease;
  pending_lease_clear_reason_ = revoke_lease ? reason : "";
  arm_ack_sequence_ = 0;
  lift_ack_sequence_ = 0;
  pending_command_time_ = std::chrono::steady_clock::now();
  last_controller_submission_result_ =
    "safety HOLD forwarded; awaiting arm and lift acknowledgement";
  state_ = revoke_lease ? Status::STATE_STOPPING : Status::STATE_LEASED_HOLD;
  detail_ = reason;
  if (!revoke_lease && reason == "command_timeout: FOLLOWING to HOLD") {
    ++command_watchdog_hold_count_;
  }
  // A timeout or resource fault is a physical safety boundary, not a new
  // remote HOLD keepalive.  Queue exactly one native lock request.
  if (revoke_lease) {
    queueBrakeLockLocked(reason, true);
  } else if (last_mode_ == Command::MODE_FOLLOW_POSITION) {
    queueBrakeLockLocked(reason, false);
  }
  last_mode_ = Command::MODE_HOLD;
  publishInternalCommand(internal_command);
}

void HeavyGatewayV1::queueBrakeLockLocked(const std::string & reason, bool force)
{
  if (lift_brake_locked_ || brake_transition_in_progress_ || brake_lock_pending_) {
    return;
  }
  brake_lock_pending_ = true;
  brake_lock_force_ = force;
  pending_brake_reason_ = reason;
}

std::optional<HeavyGatewayV1::BrakeAction> HeavyGatewayV1::takePendingBrakeActionLocked(
  const std::chrono::steady_clock::time_point & now)
{
  if (!brake_lock_pending_ || brake_transition_in_progress_ || lift_brake_locked_) {
    return std::nullopt;
  }
  if (!heavy_lift_brake_client_) {
    brake_lock_pending_ = false;
    last_brake_reason_ = pending_brake_reason_ + ": native lift brake service is not configured";
    detail_ = last_brake_reason_;
    return std::nullopt;
  }
  const bool lift_feedback_fresh = isFresh(
    lift_status_received_, lift_status_time_, now, feedback_fault_timeout_) &&
    isFresh(joints_.lift_received, joints_.lift_time, now, feedback_fault_timeout_);
  const bool stationary = lift_feedback_fresh &&
    std::abs(joints_.velocity[0]) <= brake_lock_velocity_threshold_mps_ &&
    !lift_status_.trajectory_active && !lift_status_.jog_active;
  if (!brake_lock_force_ && !stationary) {
    return std::nullopt;
  }
  BrakeAction action;
  action.release = false;
  action.reason = pending_brake_reason_;
  brake_lock_pending_ = false;
  brake_lock_force_ = false;
  pending_brake_reason_.clear();
  brake_transition_in_progress_ = true;
  return action;
}

bool HeavyGatewayV1::executeBrakeAction(const BrakeAction & action)
{
  auto native_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  native_request->data = action.release;
  std::string error;
  const auto native_response = callNativeServiceBounded<std_srvs::srv::SetBool>(
    native_request, heavy_lift_brake_client_, heavy_lift_brake_native_service_, service_timeout_, error);
  bool confirmed = native_response && native_response->success;
  std::string confirmation_error;
  bool real_workspace = false;
  if (confirmed && action.release) {
    std::unique_lock<std::mutex> lock(mutex_);
    real_workspace = workspace_status_.mode == robot_control_msg::msg::WorkspaceStatus::REAL;
    if (real_workspace) {
      confirmed = state_condition_.wait_for(lock, lift_enable_timeout_, [this]() {
        return liftPowerEnabledLocked(std::chrono::steady_clock::now());
      });
      if (!confirmed) {
        confirmation_error = "Operation Enabled confirmation timed out";
      }
    }
  }
  if (!confirmed && action.release && native_response && native_response->success) {
    auto rollback_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    rollback_request->data = false;
    std::string rollback_error;
    (void)callNativeServiceBounded<std_srvs::srv::SetBool>(
      rollback_request, heavy_lift_brake_client_, heavy_lift_brake_native_service_, service_timeout_,
      rollback_error);
  }
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    brake_transition_in_progress_ = false;
    if (confirmed) {
      // A successful native reply commits this gateway's transition latch.
      // Canonical LiftStatus remains independently visible to the caller.
      lift_brake_locked_ = !action.release;
      brake_lock_commanded_ = !action.release;
      ++brake_transition_count_;
      last_brake_reason_ = action.reason;
      last_physical_brake_time_ = now;
      detail_ = std::string(action.release ? "lift brake release requested: " :
        "lift brake lock requested: ") + action.reason;
      if (action.release) {
        RCLCPP_INFO(
          logger_,
          "Heavy V1 lift brake released: reason='%s' transition=%lu",
          action.reason.c_str(), static_cast<unsigned long>(brake_transition_count_));
      } else {
        RCLCPP_WARN(
          logger_,
          "Heavy V1 lift brake LOCKED (drive disabled): reason='%s' transition=%lu",
          action.reason.c_str(), static_cast<unsigned long>(brake_transition_count_));
      }
    } else {
      last_brake_reason_ = action.reason + ": " +
        (confirmation_error.empty() ? (native_response ? native_response->message : error) :
        confirmation_error);
      detail_ = "lift brake transition failed: " + last_brake_reason_;
      RCLCPP_ERROR(
        logger_,
        "Heavy V1 lift brake transition FAILED: reason='%s'",
        last_brake_reason_.c_str());
    }
  }
  publishStatus(true);
  return confirmed;
}

bool HeavyGatewayV1::liftEnableReadyLocked(
  const std::chrono::steady_clock::time_point & now, std::string & reason) const
{
  if (!workspaceReadyLocked(now, reason) || !feedbackReadyLocked(now, reason)) {
    return false;
  }
  if (lift_status_.estop || lift_status_.motion_blocked || lift_status_.error_code != 0) {
    reason = "lift is blocked by an emergency stop, motion latch, or drive error";
    return false;
  }
  reason.clear();
  return true;
}

bool HeavyGatewayV1::liftPowerEnabledLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  return isFresh(lift_status_received_, lift_status_time_, now, feedback_fault_timeout_) &&
         lift_status_.valid && lift_status_.feedback_fresh &&
         lift_status_.ethercat_operational && lift_status_.working_counter_ok &&
         lift_status_.initialized && lift_status_.command_enabled && lift_status_.enabled &&
         lift_status_.brake_unlocked &&
         lift_status_.cia402_state == "operation_enabled" &&
         !lift_status_.estop && !lift_status_.motion_blocked && lift_status_.error_code == 0;
}

bool HeavyGatewayV1::controllerEndpointsReadyLocked(std::string & reason) const
{
  const auto command_subscriptions = internal_command_publisher_->get_subscription_count();
  const auto arm_ack_publishers = arm_ack_subscription_->get_publisher_count();
  const auto lift_ack_publishers = lift_ack_subscription_->get_publisher_count();
  if (command_subscriptions < 2 || arm_ack_publishers < 1 || lift_ack_publishers < 1) {
    std::ostringstream stream;
    stream << "lower controller Heavy endpoints are not ready: command_subscriptions="
           << command_subscriptions << "/2 arm_ack_publishers=" << arm_ack_publishers
           << "/1 lift_ack_publishers=" << lift_ack_publishers << "/1";
    reason = stream.str();
    return false;
  }
  reason.clear();
  return true;
}

std::string HeavyGatewayV1::brakeDiagnosticLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  std::ostringstream stream;
  stream << " command_mode=" << static_cast<unsigned int>(last_mode_)
         << " lift_brake_locked=" << (lift_brake_locked_ ? "true" : "false")
         << " brake_transition_count=" << brake_transition_count_
         << " last_brake_reason='" << last_brake_reason_ << "'"
         << " last_physical_brake_age_ms=";
  if (last_physical_brake_time_.time_since_epoch().count() == 0) {
    stream << -1;
  } else {
    stream << std::chrono::duration<double, std::milli>(now - last_physical_brake_time_).count();
  }
  stream << " command_watchdog_hold_count=" << command_watchdog_hold_count_
         << " lease_expiry_count=" << lease_expiry_count_
         << " received_commands=" << received_commands_
         << " last_received_sequence=" << lease_.last_received_sequence
         << " received_command_age_ms=" << receivedCommandAgeMsLocked(now)
         << " last_accepted_sequence=" << lease_.last_accepted_sequence
         << " applied_sequence=" << lease_.applied_command_sequence
         << " command_header_delay_ms=" << last_command_header_delay_ms_
         << " controller_ack_delay_ms=" << last_controller_ack_delay_ms_
         << " controller_publish_latency_ms=" << last_controller_publish_latency_ms_
         << " pending_controller_generation=" <<
    (pending_command_valid_ ? pending_command_sequence_ : 0)
         << " arm_ack_generation=" << arm_ack_sequence_
         << " lift_ack_generation=" << lift_ack_sequence_
         << " last_arm_ack_received=" << last_arm_ack_received_sequence_
         << " last_lift_ack_received=" << last_lift_ack_received_sequence_
         << " ignored_arm_ack_count=" << ignored_arm_ack_count_
         << " ignored_lift_ack_count=" << ignored_lift_ack_count_
         << " controller_command_subscriptions=" <<
    internal_command_publisher_->get_subscription_count()
         << " arm_ack_publishers=" << arm_ack_subscription_->get_publisher_count()
         << " lift_ack_publishers=" << lift_ack_subscription_->get_publisher_count()
         << " lift_valid=" << (lift_status_.valid ? "true" : "false")
         << " lift_feedback_fresh=" << (lift_status_.feedback_fresh ? "true" : "false")
         << " lift_ethercat_operational=" <<
    (lift_status_.ethercat_operational ? "true" : "false")
         << " lift_wkc_ok=" << (lift_status_.working_counter_ok ? "true" : "false")
         << " lift_initialized=" << (lift_status_.initialized ? "true" : "false")
         << " lift_command_enabled=" << (lift_status_.command_enabled ? "true" : "false")
         << " lift_enabled=" << (lift_status_.enabled ? "true" : "false")
         << " lift_brake_unlocked=" << (lift_status_.brake_unlocked ? "true" : "false")
         << " lift_cia402_state='" << lift_status_.cia402_state << "'"
         << " lift_status_word=" << lift_status_.status_word
         << " lift_error_code=" << lift_status_.error_code
         << " lift_fault_reason='" << lift_status_.fault_reason << "'";
  return stream.str();
}

void HeavyGatewayV1::handleArmAck(const std_msgs::msg::UInt64::SharedPtr message)
{
  if (message) {
    handleAck(true, message->data);
  }
}

void HeavyGatewayV1::handleLiftAck(const std_msgs::msg::UInt64::SharedPtr message)
{
  if (message) {
    handleAck(false, message->data);
  }
}

void HeavyGatewayV1::handleAck(bool arm, std::uint64_t sequence)
{
  bool release_gate = false;
  bool forward_staged_command = false;
  Command staged_internal_command;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    (arm ? last_arm_ack_received_sequence_ : last_lift_ack_received_sequence_) = sequence;
    if (!pending_command_valid_ || sequence != pending_command_sequence_) {
      ++(arm ? ignored_arm_ack_count_ : ignored_lift_ack_count_);
      RCLCPP_WARN_THROTTLE(
        logger_, *node_.get_clock(), 2000,
        "Heavy V1 ignored %s acknowledgement: received_generation=%lu "
        "pending_valid=%s pending_generation=%lu active_owner='%s' active_lease='%s'",
        arm ? "arm" : "lift", sequence, pending_command_valid_ ? "true" : "false",
        pending_command_sequence_, owner_id_.c_str(), lease_id_.c_str());
      return;
    }
    (arm ? arm_ack_sequence_ : lift_ack_sequence_) = sequence;
    if (arm_ack_sequence_ != pending_command_sequence_ ||
      lift_ack_sequence_ != pending_command_sequence_)
    {
      return;
    }
    pending_command_valid_ = false;
    if (pending_command_is_user_) {
      applied_command_sequence_ = pending_external_command_sequence_;
    }
    last_controller_ack_delay_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - pending_command_time_).count();
    last_controller_submission_result_ = "arm and lift acknowledgement confirmed";
    RCLCPP_INFO_THROTTLE(
      logger_, *node_.get_clock(), 1000,
      "Heavy V1 controller acknowledgement confirmed: controller_generation=%lu "
      "external_sequence=%lu ack_latency_ms=%.3f arm_ack=%lu lift_ack=%lu "
      "target_lift=%.9f actual_lift=%.9f brake_locked=%s brake_unlocked=%s "
      "cia402='%s' status_word=%u error_code=%d ethercat_op=%s wkc_ok=%s initialized=%s",
      pending_command_sequence_, pending_external_command_sequence_, last_controller_ack_delay_ms_,
      arm_ack_sequence_, lift_ack_sequence_, pending_lift_target_m_, joints_.position[0],
      lift_brake_locked_ ? "true" : "false",
      lift_status_.brake_unlocked ? "true" : "false", lift_status_.cia402_state.c_str(),
      static_cast<unsigned int>(lift_status_.status_word), static_cast<int>(lift_status_.error_code),
      lift_status_.ethercat_operational ? "true" : "false",
      lift_status_.working_counter_ok ? "true" : "false",
      lift_status_.initialized ? "true" : "false");
    if (!aligned_hold_) {
      aligned_hold_ = true;
      state_ = Status::STATE_LEASED_HOLD;
      detail_ = "aligned HOLD confirmed by arm and lift controllers";
    } else if (state_ == Status::STATE_STOPPING && !revoke_after_ack_) {
      state_ = Status::STATE_LEASED_HOLD;
      detail_ = "STOP completed in HOLD";
    }
    if (revoke_after_ack_) {
      const std::string reason = pending_lease_clear_reason_.empty() ?
        "internal_state_invariant_failure" : pending_lease_clear_reason_;
      clearHeavyLeaseLocked(reason, std::chrono::steady_clock::now());
      release_gate = true;
    } else if (staged_command_valid_) {
      const auto staged_now = std::chrono::steady_clock::now();
      staged_internal_command = dispatchUserCommandLocked(staged_command_, staged_now);
      staged_command_valid_ = false;
      forward_staged_command = true;
    }
    state_condition_.notify_all();
  }
  if (release_gate) {
    publishLeaseGate(false);
  }
  if (forward_staged_command) {
    publishInternalCommand(staged_internal_command);
  }
  publishStatus(true);
}

void HeavyGatewayV1::handleRelease(
  const ReleaseLease::Request::SharedPtr request, ReleaseLease::Response::SharedPtr response)
{
  bool released_without_hold = false;
  std::string release_detail;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!lease_active_ || request->owner_id != owner_id_ || request->lease_id != lease_id_) {
      response->message = "owner or lease mismatch";
      return;
    }

    // During a workspace STOP/START transition the lower controllers are
    // intentionally unavailable.  Submitting a HOLD in that window cannot
    // produce an acknowledgement and used to convert a harmless stale lease
    // cleanup into a sticky gateway FAULT.  The workspace stop path already
    // owns the physical shutdown; clear the remote lease without waiting for
    // an ACK, while preserving the reason in diagnostics.
    const auto now = std::chrono::steady_clock::now();
    std::string workspace_reason;
    const bool workspace_state_unavailable =
      !workspace_received_ ||
      !isFresh(workspace_received_, workspace_time_, now, workspace_status_timeout_) ||
      !workspace_status_.accepted ||
      workspace_status_.state != robot_control_msg::msg::WorkspaceStatus::RUNNING;
    if (workspace_state_unavailable) {
      if (!workspaceReadyLocked(now, workspace_reason)) {
        release_detail = "workspace_stop: " + workspace_reason;
      } else {
        release_detail = "workspace_stop: workspace is not RUNNING";
      }
      last_controller_submission_result_ = "not submitted: " + release_detail;
      clearHeavyLeaseLocked(release_detail, now);
      released_without_hold = true;
    } else {
      beginSafetyHoldLocked("explicit_release", true);
      const bool released = state_condition_.wait_for(
        lock, controller_ack_timeout_, [this]() {return !lease_active_;});
      if (!released) {
        // A missing HOLD acknowledgement must not leave a revocation pending
        // forever: pending revocation disables the lease expiry path, so the
        // lease would stay active with a stale owner and every later acquire
        // from a new owner would be refused as "owner or lease mismatch".
        // The physical stop is still owned by the upper watchdog; release the
        // remote lease deterministically and report why.
        const std::string detail = "explicit_release_ack_timeout: controller HOLD ack timeout";
        last_controller_submission_result_ = "not submitted: " + detail;
        clearHeavyLeaseLocked(detail, std::chrono::steady_clock::now());
        release_detail = detail;   // consumed by the released_without_hold reply below
        released_without_hold = true;
      }
    }
  }

  if (released_without_hold) {
    publishLeaseGate(false);
    publishStatus(true);
    response->released = true;
    response->message = "lease released; HOLD not submitted because " + release_detail;
    return;
  }

  if (request->request_arm_torque_off) {
    auto power_request = std::make_shared<robot_control_msg::srv::SetRobotPower::Request>();
    power_request->enable = false;
    std::string error;
    const auto power_response = callNativeServiceBounded<robot_control_msg::srv::SetRobotPower>(
      power_request, arm_power_client_, contract::kTorqueService.data(), service_timeout_, error);
    if (!power_response || !power_response->success) {
      response->released = false;
      response->message = "lease released, but arm torque-off failed: " +
        (power_response ? power_response->message : error);
      return;
    }
    response->message = "lease released and arm torque-off confirmed";
  } else {
    response->message = "lease released after HOLD confirmation";
  }
  response->released = true;
}

void HeavyGatewayV1::handleLiftBrake(
  const LiftBrake::Request::SharedPtr request, LiftBrake::Response::SharedPtr response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!heavy_lift_brake_client_) {
      response->accepted = false;
      response->message =
        "Heavy lift brake interface is not configured; request was not routed to the Junior lift board";
      return;
    }
    if (!request->release && lift_brake_locked_) {
      response->accepted = true;
      response->message = "lift brake is already locked; no physical transition issued";
      return;
    }
    if (request->release && !lift_brake_locked_) {
      response->accepted = true;
      response->message = "lift brake is already released; no physical transition issued";
      return;
    }
    if (brake_transition_in_progress_) {
      response->accepted = false;
      response->message = "lift brake transition is already in progress";
      return;
    }
    if (request->release) {
      const auto now = std::chrono::steady_clock::now();
      std::string reason;
      if (!workspaceReadyLocked(now, reason) || !feedbackReadyLocked(now, reason) ||
        lift_status_.estop || lift_status_.motion_blocked || lift_status_.error_code != 0)
      {
        response->accepted = false;
        response->message = "Heavy lift brake release rejected: " + reason;
        return;
      }
    }
    brake_transition_in_progress_ = true;
  }
  const BrakeAction action{request->release, request->release ?
      "explicit_brake_release" : "explicit_brake_lock"};
  response->accepted = executeBrakeAction(action);
  response->message = response->accepted ?
    (request->release ? "lift brake release transition accepted" : "lift brake lock transition accepted") :
    "native lift brake transition failed";
}

void HeavyGatewayV1::handleArmJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::array<double, 14> positions{};
  std::array<double, 14> velocities{};
  std::array<bool, 14> seen{};
  std::uint16_t position_mask = 0;
  std::uint16_t velocity_mask = 0;
  for (std::size_t source = 0; source < message->name.size(); ++source) {
    const auto found = std::find(
      arm_native_names_.begin(), arm_native_names_.end(), message->name[source]);
    if (found == arm_native_names_.end()) {
      continue;
    }
    const auto index = static_cast<std::size_t>(std::distance(arm_native_names_.begin(), found));
    if (seen[index]) {
      position_mask &= static_cast<std::uint16_t>(~(1U << (index + 1U)));
      velocity_mask &= static_cast<std::uint16_t>(~(1U << (index + 1U)));
      continue;
    }
    seen[index] = true;
    if (source < message->position.size() && std::isfinite(message->position[source])) {
      positions[index] = message->position[source];
      position_mask |= static_cast<std::uint16_t>(1U << (index + 1U));
    }
    if (source < message->velocity.size() && std::isfinite(message->velocity[source])) {
      velocities[index] = message->velocity[source];
      velocity_mask |= static_cast<std::uint16_t>(1U << (index + 1U));
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < 14; ++index) {
    joints_.position[index + 1] = positions[index];
    joints_.velocity[index + 1] = velocities[index];
  }
  joints_.position_valid_mask =
    (joints_.position_valid_mask & 0x0001U) | position_mask;
  joints_.velocity_valid_mask =
    (joints_.velocity_valid_mask & 0x0001U) | velocity_mask;
  joints_.arm_received = true;
  joints_.arm_time = std::chrono::steady_clock::now();
}

void HeavyGatewayV1::handleLiftJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (!message) {
    return;
  }
  bool seen = false;
  bool duplicate = false;
  bool position_valid = false;
  bool velocity_valid = false;
  double position = 0.0;
  double velocity = 0.0;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (message->name[index] != lift_native_name_) {
      continue;
    }
    if (seen) {
      duplicate = true;
      break;
    }
    seen = true;
    if (index < message->position.size() && std::isfinite(message->position[index])) {
      position = message->position[index];
      position_valid = true;
    }
    if (index < message->velocity.size() && std::isfinite(message->velocity[index])) {
      velocity = message->velocity[index];
      velocity_valid = true;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  joints_.position[0] = position;
  joints_.velocity[0] = velocity;
  if (position_valid && !duplicate) {
    joints_.position_valid_mask |= 0x0001U;
  } else {
    joints_.position_valid_mask &= 0xfffeU;
  }
  if (velocity_valid && !duplicate) {
    joints_.velocity_valid_mask |= 0x0001U;
  } else {
    joints_.velocity_valid_mask &= 0xfffeU;
  }
  joints_.lift_received = true;
  joints_.lift_time = std::chrono::steady_clock::now();
}

bool HeavyGatewayV1::workspaceReadyLocked(
  const std::chrono::steady_clock::time_point & now, std::string & reason) const
{
  if (!isFresh(workspace_received_, workspace_time_, now, workspace_status_timeout_)) {
    reason = "workspace status unavailable or stale";
    return false;
  }
  if (!workspace_status_.accepted ||
    workspace_status_.state != robot_control_msg::msg::WorkspaceStatus::RUNNING)
  {
    reason = "workspace is not RUNNING";
    return false;
  }
  if (!isFresh(control_mode_received_, control_mode_time_, now, feedback_fault_timeout_) ||
    control_mode_status_.active_mode != robot_control_msg::msg::ArmControlModeStatus::POSITION ||
    !control_mode_status_.position_command_ready)
  {
    reason = "arm POSITION control mode is not ready";
    return false;
  }
  return true;
}

bool HeavyGatewayV1::feedbackReadyLocked(
  const std::chrono::steady_clock::time_point & now, std::string & reason) const
{
  if (!jointFeedbackCompleteLocked(now)) {
    reason = "joint position or velocity feedback is incomplete or stale";
    return false;
  }
  if (!isFresh(arm_power_received_, arm_power_time_, now, feedback_fault_timeout_) ||
    arm_power_.enabled.size() != 14 || arm_power_.status_codes.size() != 14)
  {
    reason = "arm power feedback is incomplete or stale";
    return false;
  }
  if (!isFresh(lift_status_received_, lift_status_time_, now, feedback_fault_timeout_) ||
    !lift_status_.valid || !lift_status_.feedback_fresh)
  {
    reason = "lift drive feedback is invalid or stale";
    return false;
  }
  return true;
}

bool HeavyGatewayV1::jointFeedbackCompleteLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  return isFresh(joints_.arm_received, joints_.arm_time, now, feedback_fault_timeout_) &&
    (joints_.position_valid_mask & 0x7ffeU) == 0x7ffeU &&
    (joints_.velocity_valid_mask & 0x7ffeU) == 0x7ffeU &&
    isFresh(joints_.lift_received, joints_.lift_time, now, feedback_fault_timeout_) &&
    (joints_.position_valid_mask & 0x0001U) != 0U &&
    (joints_.velocity_valid_mask & 0x0001U) != 0U;
}

double HeavyGatewayV1::commandAgeMsLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  if (last_command_time_.time_since_epoch().count() == 0) {
    return kNoCommandAgeMs;
  }
  return std::chrono::duration<double, std::milli>(now - last_command_time_).count();
}

double HeavyGatewayV1::receivedCommandAgeMsLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  if (lease_.last_received_command_time.time_since_epoch().count() == 0) {
    return kNoCommandAgeMs;
  }
  return std::chrono::duration<double, std::milli>(
    now - lease_.last_received_command_time).count();
}

double HeavyGatewayV1::leaseRemainingMsLocked(
  const std::chrono::steady_clock::time_point & now) const
{
  if (!lease_active_ || now >= lease_deadline_) {
    return 0.0;
  }
  return std::chrono::duration<double, std::milli>(lease_deadline_ - now).count();
}

void HeavyGatewayV1::updateStateWithoutLeaseLocked(
  const std::chrono::steady_clock::time_point & now)
{
  if (lease_active_) {
    return;
  }

  // A workspace start is an expected transitional state.  Keep the gateway
  // offline while the supervisor brings up controllers and hardware instead
  // of advertising a sticky FAULT that makes upper-level startup look like a
  // failure.  Command submission remains gated by workspaceReadyLocked().
  if (workspace_received_ &&
    isFresh(workspace_received_, workspace_time_, now, workspace_status_timeout_) &&
    workspace_status_.state == robot_control_msg::msg::WorkspaceStatus::STARTING)
  {
    state_ = Status::STATE_OFFLINE;
    detail_ = "workspace start in progress; waiting for RUNNING and complete feedback";
    return;
  }

  std::string reason;
  if (workspaceReadyLocked(now, reason) && feedbackReadyLocked(now, reason)) {
    state_ = Status::STATE_MONITOR;
    detail_ = "lease cleared: " + last_lease_clear_reason_ +
      "; complete lower-controller feedback; no execution lease";
    if (!last_command_rejection_reason_.empty()) {
      detail_ += "; last command rejected: " + last_command_rejection_reason_;
    }
  } else if (now - started_at_ <= startup_feedback_grace_) {
    state_ = Status::STATE_OFFLINE;
    detail_ = "lease cleared: " + last_lease_clear_reason_ + "; startup feedback grace: " + reason;
  } else {
    state_ = Status::STATE_FAULT;
    detail_ = "lease cleared: " + last_lease_clear_reason_ +
      "; lower-controller feedback unavailable: " + reason;
  }
}

void HeavyGatewayV1::watchdogAndFeedbackTick()
{
  bool status_changed = false;
  bool release_gate = false;
  bool retry_pending_command = false;
  Command retry_command;
  std::optional<BrakeAction> brake_action;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    std::string reason;
    if (lease_active_ && !pending_command_valid_ && state_ == Status::STATE_FOLLOWING &&
      now - last_command_time_ > command_hold_timeout_)
    {
      beginSafetyHoldLocked("command_timeout: FOLLOWING to HOLD", false);
      status_changed = true;
    }
    // An in-flight revocation must NOT be able to disable lease expiry: if the
    // safety HOLD that carries the revocation is never acknowledged, gating the
    // expiry on !revoke_after_ack_ leaves lease_active_ true forever.  The lower
    // then rejects every acquire from a new owner ("owner or lease mismatch") and
    // the only recovery was a full stack restart.  Expiry now always applies; it
    // performs the same safety HOLD plus revoke that the pending revocation
    // wanted, so the outcome is unchanged and bounded in time.
    if (lease_active_ && last_command_time_.time_since_epoch().count() != 0 &&
      now >= lease_deadline_)
    {
      ++lease_expiry_count_;
      RCLCPP_ERROR(
        logger_,
        "Heavy V1 execution lease EXPIRED: timeout_ms=%lu expiry_count=%lu owner='%s' lease='%s' "
        "last_command_age_ms=%.1f received=%lu accepted=%lu rejected=%lu duplicate=%lu "
        "out_of_order=%lu last_received_sequence=%lu last_accepted_sequence=%lu",
        static_cast<unsigned long>(execution_lease_timeout_.count()),
        static_cast<unsigned long>(lease_expiry_count_), owner_id_.c_str(), lease_id_.c_str(),
        commandAgeMsLocked(now), received_commands_, accepted_commands_, rejected_commands_,
        duplicate_commands_, out_of_order_commands_, lease_.last_received_sequence,
        lease_.last_accepted_sequence);
      beginSafetyHoldLocked(
        "execution lease expired after " +
        std::to_string(execution_lease_timeout_.count()) + " ms without an accepted command", true);
      status_changed = true;
    }
    if (lease_active_ && revoke_after_ack_ && pending_command_valid_ &&
      now - pending_command_time_ > controller_ack_timeout_)
    {
      // Bounded fallback for a revocation whose HOLD is never acknowledged.
      // clearHeavyLeaseLocked() otherwise only runs from the acknowledgement
      // handler, so one lost acknowledgement left the lease active forever:
      // the lower kept reporting LEASED_HOLD with a zero remaining lease, a new
      // owner could never acquire, and only a full restart helped.
      const std::string reason = pending_lease_clear_reason_.empty() ?
        "internal_state_invariant_failure" : pending_lease_clear_reason_;
      RCLCPP_WARN(
        logger_,
        "Heavy V1 lease revocation timed out waiting for the controller HOLD acknowledgement "
        "(%.1f ms); clearing owner='%s' lease='%s' without it",
        std::chrono::duration<double, std::milli>(now - pending_command_time_).count(),
        owner_id_.c_str(), lease_id_.c_str());
      clearHeavyLeaseLocked(reason + " (ack timeout fallback)", now);
      pending_command_valid_ = false;
      pending_command_is_user_ = false;
      staged_command_valid_ = false;
      revoke_after_ack_ = false;
      release_gate = true;
      status_changed = true;
    }
    if (lease_active_ && pending_command_valid_ &&
      now - pending_command_time_ > controller_ack_timeout_ && state_ != Status::STATE_FAULT)
    {
      state_ = Status::STATE_FAULT;
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        now - pending_command_time_).count();
      std::ostringstream timeout;
      timeout << "lower controller apply acknowledgement timeout: controller_generation="
              << pending_command_sequence_ << " external_sequence="
              << pending_external_command_sequence_ << " elapsed_ms=" << elapsed_ms
              << " arm_ack=" << arm_ack_sequence_ << " lift_ack=" << lift_ack_sequence_
              << " missing=";
      if (arm_ack_sequence_ != pending_command_sequence_ &&
        lift_ack_sequence_ != pending_command_sequence_)
      {
        timeout << "arm,lift";
      } else if (arm_ack_sequence_ != pending_command_sequence_) {
        timeout << "arm";
      } else {
        timeout << "lift";
      }
      timeout << " command_subscriptions=" << internal_command_publisher_->get_subscription_count()
              << " arm_ack_publishers=" << arm_ack_subscription_->get_publisher_count()
              << " lift_ack_publishers=" << lift_ack_subscription_->get_publisher_count()
              << " target_lift=" << pending_lift_target_m_
              << " actual_lift=" << joints_.position[0]
              << " brake_locked=" << (lift_brake_locked_ ? "true" : "false")
              << " brake_unlocked=" << (lift_status_.brake_unlocked ? "true" : "false")
              << " cia402='" << lift_status_.cia402_state << "'"
              << " status_word=" << lift_status_.status_word
              << " error_code=" << lift_status_.error_code
              << " ethercat_op=" <<
        (lift_status_.ethercat_operational ? "true" : "false")
              << " wkc_ok=" << (lift_status_.working_counter_ok ? "true" : "false")
              << " initialized=" << (lift_status_.initialized ? "true" : "false")
              << " owner='" << owner_id_ << "' lease='" << lease_id_ << "'"
              << " last_accepted=" << last_command_sequence_
              << " last_applied=" << lease_.applied_command_sequence
              << " command_age_ms=" << commandAgeMsLocked(now)
              << "; lease gate remains active";
      detail_ = timeout.str();
      last_controller_submission_result_ = timeout.str();
      RCLCPP_ERROR(logger_, "%s", timeout.str().c_str());
      // A lower-controller acknowledgement failure is a safety boundary.  The
      // queue/latch makes this exactly one native lock request, even though
      // the watchdog continues to observe the fault on later ticks.
      queueBrakeLockLocked("controller_apply_ack_timeout", true);
      // Do not leave the resident gateway advertising an unconfirmed lease.
      // The lower command was already submitted and timed out; revoke the
      // remote lease now, keep the timeout detail, and let the queued native
      // brake lock/STOP safety path continue independently.
      clearHeavyLeaseLocked(
        "automatic release: controller HOLD ack timeout; lease revoked after STOP/HOLD submission",
        now);
      release_gate = true;
      status_changed = true;
    }
    if (lease_active_ && pending_command_valid_ &&
      now - pending_command_time_ < controller_ack_timeout_ &&
      now - last_pending_command_publish_time_ >= controller_ack_retry_interval_)
    {
      retry_command = pending_internal_command_;
      retry_pending_command = retry_command.sequence == pending_command_sequence_;
      if (retry_pending_command) {
        ++controller_ack_retry_count_;
      }
    }
    std::string workspace_reason;
    if (lease_active_ && !revoke_after_ack_ && !workspaceReadyLocked(now, workspace_reason))
    {
      const std::string lease_clear_reason = "workspace_stop: " + workspace_reason;
      beginSafetyHoldLocked(lease_clear_reason, true);
      clearHeavyLeaseLocked(lease_clear_reason, now);
      release_gate = true;
      status_changed = true;
    } else if (lease_active_ && !feedbackReadyLocked(now, reason) &&
      state_ != Status::STATE_FAULT && state_ != Status::STATE_STOPPING)
    {
      const std::string lease_clear_reason = "resource_fault: " + reason;
      // Submit the native safe HOLD with the currently coherent snapshot,
      // then revoke the Heavy lease without waiting for its acknowledgement.
      // A missing lower-controller acknowledgement must never leave a remote
      // owner holding a lease while feedback is incomplete.
      beginSafetyHoldLocked(lease_clear_reason, true);
      clearHeavyLeaseLocked(lease_clear_reason, now);
      release_gate = true;
      status_changed = true;
    }
    updateStateWithoutLeaseLocked(now);
    brake_action = takePendingBrakeActionLocked(now);
  }
  if (release_gate) {
    publishLeaseGate(false);
  }
  if (retry_pending_command) {
    publishInternalCommand(retry_command, true);
  }
  if (status_changed) {
    publishStatus(true);
  }
  if (brake_action) {
    executeBrakeAction(*brake_action);
  }
}

void HeavyGatewayV1::publishFeedback()
{
  Feedback message;
  std::chrono::steady_clock::time_point steady_now;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Take the clock sample after acquiring the snapshot lock.  A source
    // callback may otherwise update its timestamp between the clock sample
    // and this lock acquisition, making a fresh sample look "from the
    // future" and incorrectly trimming the arm validity bits to 0x0001.
    steady_now = std::chrono::steady_clock::now();
    const bool joint_feedback_complete = jointFeedbackCompleteLocked(steady_now);
    if (lease_active_ && !joint_feedback_complete) {
      // Do not expose a partially valid resource set while a lease is active.
      // watchdogAndFeedbackTick() has already submitted the bounded safety HOLD
      // and will revoke the lease before the next fault feedback is published.
      return;
    }
    if (last_feedback_publish_time_.time_since_epoch().count() != 0) {
      const auto interval = std::chrono::duration<double>(
        steady_now - last_feedback_publish_time_).count();
      if (interval > 0.0) {
        measured_feedback_rate_hz_ = 1.0 / interval;
      }
    }
    last_feedback_publish_time_ = steady_now;
    message.header.stamp = node_.now();
    message.protocol_major = kProtocolMajor;
    message.protocol_minor = kProtocolMinor;
    message.layout_crc32 = contract::kLayoutCrc32;
    message.feedback_sequence = ++feedback_sequence_;
    message.applied_command_sequence = lease_.applied_command_sequence;
    message.active_owner_id = lease_.active ? lease_.owner_id : "";
    message.active_lease_id = lease_.active ? lease_.lease_id : "";
    message.workspace_state = Feedback::WORKSPACE_STOPPED;
    if (isFresh(workspace_received_, workspace_time_, steady_now, workspace_status_timeout_) &&
      workspace_status_.state == robot_control_msg::msg::WorkspaceStatus::RUNNING)
    {
      message.workspace_state = workspace_status_.mode ==
        robot_control_msg::msg::WorkspaceStatus::REAL ?
        Feedback::WORKSPACE_REAL : Feedback::WORKSPACE_SIMULATION;
    }
    message.control_mode = isFresh(
      control_mode_received_, control_mode_time_, steady_now, feedback_fault_timeout_) &&
      control_mode_status_.active_mode == robot_control_msg::msg::ArmControlModeStatus::POSITION ?
      Feedback::MODE_POSITION : Feedback::MODE_UNKNOWN;
    message.position_valid_mask = joints_.position_valid_mask;
    message.velocity_valid_mask = joints_.velocity_valid_mask;
    if (!isFresh(joints_.arm_received, joints_.arm_time, steady_now, feedback_fault_timeout_)) {
      message.position_valid_mask &= 0x0001U;
      message.velocity_valid_mask &= 0x0001U;
    }
    if (!isFresh(joints_.lift_received, joints_.lift_time, steady_now, feedback_fault_timeout_)) {
      message.position_valid_mask &= 0x7ffeU;
      message.velocity_valid_mask &= 0x7ffeU;
    }
    message.position = joints_.position;
    message.velocity = joints_.velocity;
    message.arm_torque_enabled_mask = 0;
    message.resource_fault_mask = 0;
    message.fault_code.fill(0);
    if (isFresh(arm_power_received_, arm_power_time_, steady_now, feedback_fault_timeout_) &&
      arm_power_.enabled.size() == 14 && arm_power_.status_codes.size() == 14)
    {
      for (std::size_t index = 0; index < 14; ++index) {
        if (arm_power_.enabled[index]) {
          message.arm_torque_enabled_mask |= static_cast<std::uint16_t>(1U << (index + 1U));
        }
        const auto code = arm_power_.status_codes[index];
        if (code == 0 || code == 8) {
          message.resource_fault_mask |= static_cast<std::uint16_t>(1U << (index + 1U));
          message.fault_code[index + 1] = code == 0 ? -1 : code;
        }
      }
    }
    message.lift_drive_enabled = isFresh(
      lift_status_received_, lift_status_time_, steady_now, feedback_fault_timeout_) &&
      lift_status_.valid && lift_status_.enabled;
    if (!isFresh(
        lift_status_received_, lift_status_time_, steady_now, feedback_fault_timeout_) ||
      !lift_status_.valid || lift_status_.error_code != 0)
    {
      message.resource_fault_mask |= 0x0001U;
      message.fault_code[0] = lift_status_.error_code == 0 ? -1 : lift_status_.error_code;
    }
    if (!joint_feedback_complete) {
      if ((message.position_valid_mask & 0x7ffeU) != 0x7ffeU ||
        (message.velocity_valid_mask & 0x7ffeU) != 0x7ffeU)
      {
        message.resource_fault_mask |= 0x7ffeU;
        for (std::size_t index = 1; index < message.fault_code.size(); ++index) {
          if (message.fault_code[index] == 0) {
            message.fault_code[index] = -1;
          }
        }
      }
      if ((message.position_valid_mask & 0x0001U) == 0U ||
        (message.velocity_valid_mask & 0x0001U) == 0U)
      {
        message.resource_fault_mask |= 0x0001U;
        if (message.fault_code[0] == 0) {
          message.fault_code[0] = -1;
        }
      }
    }
    message.command_age_ms = commandAgeMsLocked(steady_now);
  }
  feedback_publisher_->publish(message);
}

void HeavyGatewayV1::publishStatus(bool force)
{
  Status message;
  const auto steady_now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto minimum_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    if (!force && last_status_publish_time_.time_since_epoch().count() != 0 &&
      steady_now - last_status_publish_time_ < minimum_period)
    {
      return;
    }
    last_status_publish_time_ = steady_now;
    message.header.stamp = node_.now();
    message.protocol_major = kProtocolMajor;
    message.protocol_minor = kProtocolMinor;
    message.layout_crc32 = contract::kLayoutCrc32;
    message.state = lease_.state;
    message.active_owner_id = lease_.active ? lease_.owner_id : "";
    message.active_lease_id = lease_.active ? lease_.lease_id : "";
    message.lease_remaining_ms = leaseRemainingMsLocked(steady_now);
    message.command_age_ms = commandAgeMsLocked(steady_now);
    message.publish_rate_hz = measured_feedback_rate_hz_;
    message.accepted_commands = accepted_commands_;
    message.rejected_commands = rejected_commands_;
    message.duplicate_commands = duplicate_commands_;
    message.out_of_order_commands = out_of_order_commands_;
    message.detail = lease_.detail + "; controller_submission='" +
      last_controller_submission_result_ + "';" + brakeDiagnosticLocked(steady_now);
  }
  status_publisher_->publish(message);
}

void HeavyGatewayV1::publishInternalCommand(const Command & command, bool retry)
{
  const auto start = std::chrono::steady_clock::now();
  internal_command_publisher_->publish(command);
  const double publish_latency_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  // Safety HOLD may call this helper while already owning mutex_. Publishing
  // must stay non-blocking and must never recursively acquire that mutex.
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  last_controller_publish_latency_ms_ = publish_latency_ms;
  const auto subscriptions = internal_command_publisher_->get_subscription_count();
  if (pending_command_valid_ && pending_command_sequence_ == command.sequence) {
    last_pending_command_publish_time_ = std::chrono::steady_clock::now();
    std::ostringstream result;
    result << "internal controller command " << (retry ? "re-published" : "published")
           << ": generation=" << command.sequence
           << " matched_subscriptions=" << subscriptions
           << " publish_latency_ms=" << publish_latency_ms
           << " retry_count=" << controller_ack_retry_count_
           << "; awaiting arm and lift acknowledgement";
    last_controller_submission_result_ = result.str();
  }
  RCLCPP_INFO_THROTTLE(
    logger_, *node_.get_clock(), 1000,
    "Heavy V1 controller submission: controller_generation=%lu external_sequence=%lu "
    "mode=%u target_lift=%.9f actual_lift=%.9f matched_subscriptions=%zu "
    "publish_latency_ms=%.3f arm_ack=%lu lift_ack=%lu brake_locked=%s brake_unlocked=%s "
    "cia402='%s' status_word=%u error_code=%d ethercat_op=%s wkc_ok=%s initialized=%s "
    "lift_feedback='%s'",
    command.sequence, pending_external_command_sequence_, static_cast<unsigned int>(command.mode),
    command.position[0], joints_.position[0], subscriptions, publish_latency_ms,
    arm_ack_sequence_, lift_ack_sequence_, lift_brake_locked_ ? "true" : "false",
    lift_status_.brake_unlocked ? "true" : "false", lift_status_.cia402_state.c_str(),
    static_cast<unsigned int>(lift_status_.status_word), static_cast<int>(lift_status_.error_code),
    lift_status_.ethercat_operational ? "true" : "false",
    lift_status_.working_counter_ok ? "true" : "false",
    lift_status_.initialized ? "true" : "false", lift_status_.message.c_str());
}

void HeavyGatewayV1::publishLeaseGate(bool active)
{
  std_msgs::msg::Bool message;
  message.data = active;
  lease_gate_publisher_->publish(message);
}

}  // namespace robot_lower_gateway

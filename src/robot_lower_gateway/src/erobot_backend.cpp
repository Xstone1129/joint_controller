#include "robot_lower_gateway/erobot_backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "robot_lower_gateway/feedback_policy.hpp"
#include "robot_lower_gateway/joint_state_mapping.hpp"

namespace robot_lower_gateway
{

std::string ErobotBackend::name() const
{
  return "erobot";
}

robot_lower_gateway::BackendCapabilities ErobotBackend::capabilities() const
{
  robot_lower_gateway::BackendCapabilities result;
  result.observes_joint_state = true;
  result.observes_power = true;
  result.observes_control_mode = true;
  result.observes_motion = true;
  result.observes_tcp_pose = true;
  result.observes_execution = true;
  return result;
}

void ErobotBackend::configure(
  rclcpp::Node & node,
  const robot_lower_gateway::BackendConfiguration & configuration)
{
  if (configuration.canonical_joint_names.empty()) {
    throw std::invalid_argument("canonical_joint_names must not be empty");
  }
  if (configuration.canonical_joint_names.size() != configuration.native_joint_names.size()) {
    throw std::invalid_argument("canonical_joint_names and native_joint_names sizes differ");
  }

  std::unordered_set<std::string> canonical_names;
  std::unordered_set<std::string> native_names;
  for (std::size_t index = 0; index < configuration.canonical_joint_names.size(); ++index) {
    if (!canonical_names.insert(configuration.canonical_joint_names[index]).second) {
      throw std::invalid_argument(
              "duplicate canonical joint name: " +
              configuration.canonical_joint_names[index]);
    }
    if (!native_names.insert(configuration.native_joint_names[index]).second) {
      throw std::invalid_argument(
              "duplicate native joint name: " +
              configuration.native_joint_names[index]);
    }
  }

  configuration_ = configuration;
  snapshot_.joint_names = configuration.canonical_joint_names;
  snapshot_.joint_positions.assign(
    configuration.canonical_joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  snapshot_.raw_drive_status_codes.assign(configuration.canonical_joint_names.size(), 0);
  snapshot_.drive_states.assign(
    configuration.canonical_joint_names.size(),
    robot_lower_gateway::DriveState::kUnavailable);

  native_joint_index_.clear();
  for (std::size_t index = 0; index < configuration.native_joint_names.size(); ++index) {
    native_joint_index_.emplace(configuration.native_joint_names[index], index);
  }

  joint_state_subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
    configuration.joint_state_topic,
    rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {handleJointState(message);});

  const auto latched_status_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  power_status_subscription_ = node.create_subscription<robot_control_msg::msg::ArmPowerStatus>(
    configuration.power_status_topic, latched_status_qos,
    [this](const robot_control_msg::msg::ArmPowerStatus::SharedPtr message) {
      handlePowerStatus(message);
    });
  control_mode_subscription_ =
    node.create_subscription<robot_control_msg::msg::ArmControlModeStatus>(
    configuration.control_mode_status_topic, latched_status_qos,
    [this](const robot_control_msg::msg::ArmControlModeStatus::SharedPtr message) {
      handleControlMode(message);
    });
  motion_status_subscription_ = node.create_subscription<robot_control_msg::msg::ArmMotionStatus>(
    configuration.motion_status_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const robot_control_msg::msg::ArmMotionStatus::SharedPtr message) {
      handleMotionStatus(message);
    });
  tcp_pose_subscription_ = node.create_subscription<robot_control_msg::msg::EndEffectorPose>(
    configuration.tcp_pose_topic,
    rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile(),
    [this](const robot_control_msg::msg::EndEffectorPose::SharedPtr message) {
      handleTcpPose(message);
    });
  execution_status_subscription_ =
    node.create_subscription<robot_control_msg::msg::CartesianExecutionStatus>(
    configuration.execution_status_topic, latched_status_qos,
    [this](const robot_control_msg::msg::CartesianExecutionStatus::SharedPtr message) {
      handleExecutionStatus(message);
    });
  workspace_status_subscription_ =
    node.create_subscription<robot_control_msg::msg::WorkspaceStatus>(
    configuration.workspace_status_topic, latched_status_qos,
    [this](const robot_control_msg::msg::WorkspaceStatus::SharedPtr message) {
      handleWorkspaceStatus(message);
    });
}

robot_lower_gateway::BackendSnapshot ErobotBackend::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = snapshot_;
  result.joint_state_age_sec = ageSeconds(joint_state_time_, result.joint_state_received);
  result.power_status_age_sec = ageSeconds(power_status_time_, result.power_status_received);
  result.control_mode_age_sec = ageSeconds(control_mode_time_, result.control_mode_received);
  result.motion_status_age_sec = ageSeconds(motion_status_time_, result.motion_status_received);
  result.tcp_pose_age_sec = ageSeconds(tcp_pose_time_, result.tcp_pose_received);
  result.execution_status_age_sec =
    ageSeconds(execution_status_time_, result.execution_status_received);
  return result;
}

double ErobotBackend::ageSeconds(const SteadyTime & then, bool received)
{
  if (!received) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - then).count();
}

bool ErobotBackend::containsCode(
  const std::vector<std::int32_t> & values, std::int32_t code) const
{
  return std::find(values.begin(), values.end(), code) != values.end();
}

robot_lower_gateway::DriveState ErobotBackend::normalizeDriveState(
  std::int32_t status_code, bool enabled) const
{
  if (containsCode(configuration_.unavailable_status_codes, status_code)) {
    return robot_lower_gateway::DriveState::kUnavailable;
  }
  if (enabled && containsCode(configuration_.enabled_status_codes, status_code)) {
    return robot_lower_gateway::DriveState::kEnabled;
  }
  if (!enabled) {
    return robot_lower_gateway::DriveState::kNotEnabled;
  }
  return robot_lower_gateway::DriveState::kUnknown;
}

void ErobotBackend::handleJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  const auto mapped = mapJointState(
    *message, configuration_.canonical_joint_names, configuration_.native_joint_names);
  std::lock_guard<std::mutex> lock(mutex_);
  if (mapped.valid) {
    snapshot_.joint_positions = mapped.positions;
    snapshot_.joint_velocities = mapped.velocities;
    snapshot_.joint_efforts = mapped.efforts;
  }
  snapshot_.joint_state_received = true;
  snapshot_.joint_state_complete = mapped.complete;
  snapshot_.joint_state_valid = mapped.valid;
  snapshot_.joint_state_error = mapped.error;
  joint_state_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handlePowerStatus(
  const robot_control_msg::msg::ArmPowerStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::unordered_map<std::string, std::size_t> source_indices;
  for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
    source_indices.emplace(message->joint_names[index], index);
  }

  bool every_drive_enabled = !configuration_.native_joint_names.empty();
  for (std::size_t target_index = 0;
    target_index < configuration_.native_joint_names.size(); ++target_index)
  {
    const auto source = source_indices.find(configuration_.native_joint_names[target_index]);
    if (source == source_indices.end() || source->second >= message->status_codes.size() ||
      source->second >= message->enabled.size())
    {
      snapshot_.raw_drive_status_codes[target_index] = 0;
      snapshot_.drive_states[target_index] = robot_lower_gateway::DriveState::kUnavailable;
      every_drive_enabled = false;
      continue;
    }

    const auto status_code = message->status_codes[source->second];
    const auto drive_state = normalizeDriveState(status_code, message->enabled[source->second]);
    snapshot_.raw_drive_status_codes[target_index] = status_code;
    snapshot_.drive_states[target_index] = drive_state;
    every_drive_enabled = every_drive_enabled &&
      drive_state == robot_lower_gateway::DriveState::kEnabled;
  }

  snapshot_.power_status_received = true;
  snapshot_.power_command_enabled = message->command_enabled;
  snapshot_.all_drives_enabled = every_drive_enabled && message->all_enabled;
  power_status_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handleControlMode(
  const robot_control_msg::msg::ArmControlModeStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control_mode_received = true;
  snapshot_.requested_control_mode = message->requested_mode;
  snapshot_.active_control_mode = message->active_mode;
  snapshot_.position_command_ready = message->position_command_ready;
  control_mode_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handleMotionStatus(
  const robot_control_msg::msg::ArmMotionStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.motion_status_received = true;
  snapshot_.is_moving = message->is_moving;
  snapshot_.goal_reached = message->goal_reached;
  motion_status_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handleTcpPose(
  const robot_control_msg::msg::EndEffectorPose::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.tcp_pose_received = true;
  snapshot_.left_tcp_pose = message->left_ee_pose;
  snapshot_.right_tcp_pose = message->right_ee_pose;
  tcp_pose_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handleExecutionStatus(
  const robot_control_msg::msg::CartesianExecutionStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.execution_status_received = true;
  snapshot_.execution_state = message->state;
  snapshot_.planned_points = message->planned_points;
  snapshot_.planned_duration_sec = message->planned_duration_sec;
  snapshot_.execution_message = message->message;
  execution_status_time_ = std::chrono::steady_clock::now();
}

void ErobotBackend::handleWorkspaceStatus(
  const robot_control_msg::msg::WorkspaceStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (isNewWorkspaceStarting(
      message->state, message->command_seq,
      starting_command_seen_, last_starting_command_seq_))
  {
    starting_command_seen_ = true;
    last_starting_command_seq_ = message->command_seq;
    ++snapshot_.workspace_generation;
    snapshot_.execution_status_received = false;
    snapshot_.execution_state = robot_control_msg::msg::CartesianExecutionStatus::IDLE;
    snapshot_.planned_points = 0;
    snapshot_.planned_duration_sec = 0.0;
    snapshot_.execution_message.clear();
    execution_status_time_ = SteadyTime{};
  }

  snapshot_.workspace_status_received = true;
  snapshot_.workspace_command_seq = message->command_seq;
  snapshot_.workspace_state = message->state;
  snapshot_.workspace_mode = message->mode;
}

}  // namespace robot_lower_gateway

PLUGINLIB_EXPORT_CLASS(
  robot_lower_gateway::ErobotBackend,
  robot_lower_gateway::Backend)

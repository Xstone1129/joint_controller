#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace joint_hardware
{

namespace
{

std::string trim_copy(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  return text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
}

bool parse_int_text(const std::string & text, int & value)
{
  try {
    std::size_t used = 0;
    const int parsed = std::stoi(text, &used, 0);
    if (used != text.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double_text(const std::string & text, double & value)
{
  try {
    std::size_t used = 0;
    const double parsed = std::stod(text, &used);
    if (used != text.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_bool_text(const std::string & text, bool & value)
{
  std::string normalized = text;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    value = true;
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    value = false;
    return true;
  }
  return false;
}

bool load_flat_yaml(
  const std::string & path, std::unordered_map<std::string, std::string> & values)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return false;
  }
  values.clear();
  std::string line;
  while (std::getline(stream, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.resize(comment);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim_copy(line.substr(0, colon));
    const std::string value = trim_copy(line.substr(colon + 1));
    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }
  return true;
}

bool hardware_string(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  std::string & value)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return false;
  }
  value = it->second;
  return true;
}

bool hardware_int(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  int & value)
{
  std::string text;
  return hardware_string(info, key, text) && parse_int_text(text, value);
}

bool yaml_int(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key,
  int & value)
{
  const auto it = values.find(key);
  return it != values.end() && parse_int_text(it->second, value);
}

bool yaml_double(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key,
  double & value)
{
  const auto it = values.find(key);
  return it != values.end() && parse_double_text(it->second, value);
}

bool yaml_bool(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key,
  bool & value)
{
  const auto it = values.find(key);
  return it != values.end() && parse_bool_text(it->second, value);
}

bool load_joint_limits(
  const std::string & urdf_path,
  const std::string & joint_name,
  double & lower,
  double & upper)
{
  urdf::Model model;
  if (!model.initFile(urdf_path)) {
    return false;
  }
  const auto joint = model.getJoint(joint_name);
  if (!joint || !joint->limits) {
    return false;
  }
  const double lo = static_cast<double>(joint->limits->lower);
  const double hi = static_cast<double>(joint->limits->upper);
  if (!std::isfinite(lo) || !std::isfinite(hi)) {
    return false;
  }
  lower = std::min(lo, hi);
  upper = std::max(lo, hi);
  return true;
}

}  // namespace

hardware_interface::CallbackReturn WaistHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1U || info_.joints.front().name != "joint_qugan") {
    RCLCPP_ERROR(logger_, "WaistHardware requires exactly one joint named joint_qugan.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hardware_string(info_, "waist_port", waist_port_);
  hardware_int(info_, "waist_node_id", waist_node_id_);
  hardware_int(info_, "serial_baudrate", serial_baudrate_);
  hardware_int(info_, "bridge_channel", bridge_channel_);
  hardware_int(info_, "bridge_bitrate", bridge_bitrate_);

  int integer_value = 0;
  if (hardware_int(info_, "can_nom_baud", integer_value) && integer_value > 0) {
    can_nom_baud_ = static_cast<uint32_t>(integer_value);
  }
  if (hardware_int(info_, "can_dat_baud", integer_value) && integer_value > 0) {
    can_dat_baud_ = static_cast<uint32_t>(integer_value);
  }
  if (hardware_int(info_, "right_tool_channel", integer_value)) {
    bridge_route_config_.right_tool_channel = static_cast<uint8_t>(std::clamp(integer_value, 0, 2));
  }
  if (hardware_int(info_, "left_tool_channel", integer_value)) {
    bridge_route_config_.left_tool_channel = static_cast<uint8_t>(std::clamp(integer_value, 0, 2));
  }
  if (hardware_int(info_, "waist_channel", integer_value)) {
    bridge_route_config_.waist_channel = static_cast<uint8_t>(std::clamp(integer_value, 0, 2));
    bridge_channel_ = integer_value;
  }
  if (hardware_int(info_, "right_tool_bitrate", integer_value) && integer_value > 0) {
    bridge_route_config_.right_tool_bitrate = static_cast<uint32_t>(integer_value);
  }
  if (hardware_int(info_, "left_tool_bitrate", integer_value) && integer_value > 0) {
    bridge_route_config_.left_tool_bitrate = static_cast<uint32_t>(integer_value);
  }
  if (hardware_int(info_, "waist_bitrate", integer_value) && integer_value > 0) {
    bridge_route_config_.waist_bitrate = static_cast<uint32_t>(integer_value);
    bridge_bitrate_ = integer_value;
  }

  std::string config_path;
  if (!hardware_string(info_, "waist_hardware_config", config_path)) {
    try {
      config_path = ament_index_cpp::get_package_share_directory("joint_hardware") +
        "/config/waist_hardware.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Could not resolve waist config: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::unordered_map<std::string, std::string> config;
  if (!load_flat_yaml(config_path, config)) {
    RCLCPP_ERROR(logger_, "Failed to load waist config: %s", config_path.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  yaml_int(config, "waist_timeout_ms", waist_timeout_ms_);
  yaml_int(config, "waist_init_timeout_ms", waist_init_timeout_ms_);
  yaml_int(config, "waist_init_retry_count", waist_init_retry_count_);
  yaml_int(config, "waist_profile_vel", waist_profile_vel_);
  yaml_int(config, "waist_profile_acc", waist_profile_acc_);
  yaml_double(config, "waist_command_resend_hz", waist_command_resend_hz_);
  yaml_double(config, "waist_feedback_sync_hz", waist_feedback_sync_hz_);
  yaml_double(config, "command_epsilon_qugan", command_epsilon_qugan_);
  yaml_double(config, "qugan_sign", qugan_sign_);
  yaml_double(config, "joint_qugan_min_rad", joint_qugan_min_rad_);
  yaml_double(config, "joint_qugan_max_rad", joint_qugan_max_rad_);
  yaml_bool(
    config, "waist_defer_bridge_release_on_deactivate",
    waist_defer_bridge_release_on_deactivate_);

  waist_node_id_ = std::clamp(waist_node_id_, 1, 127);
  bridge_channel_ = std::clamp(bridge_channel_, 0, 2);
  waist_timeout_ms_ = std::clamp(waist_timeout_ms_, 50, 3000);
  waist_init_timeout_ms_ = std::clamp(waist_init_timeout_ms_, 100, 3000);
  waist_init_retry_count_ = std::clamp(waist_init_retry_count_, 1, 10);
  waist_profile_vel_ = std::clamp(waist_profile_vel_, 1, 40);
  waist_profile_acc_ = std::clamp(waist_profile_acc_, 1, 10000);
  waist_command_resend_hz_ = std::max(1.0, waist_command_resend_hz_);
  waist_feedback_sync_hz_ = std::clamp(waist_feedback_sync_hz_, 1.0, 500.0);
  command_epsilon_qugan_ = std::max(1e-5, command_epsilon_qugan_);
  qugan_sign_ = qugan_sign_ < 0.0 ? -1.0 : 1.0;

  if (!std::isfinite(joint_qugan_min_rad_) || !std::isfinite(joint_qugan_max_rad_) ||
    joint_qugan_min_rad_ >= joint_qugan_max_rad_)
  {
    RCLCPP_ERROR(
      logger_, "Invalid joint_qugan limits [%.6f, %.6f].",
      joint_qugan_min_rad_, joint_qugan_max_rad_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (waist_port_.empty()) {
    RCLCPP_ERROR(logger_, "waist_port is required for the USB-FDCAN waist driver.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string urdf_path;
  const auto urdf_it = config.find("urdf_path");
  if (urdf_it != config.end()) {
    urdf_path = urdf_it->second;
  }
  if (urdf_path.empty()) {
    urdf_path.clear();
  }
  if (
    !urdf_path.empty() &&
    !load_joint_limits(urdf_path, "joint_qugan", joint_qugan_min_rad_, joint_qugan_max_rad_))
  {
    RCLCPP_WARN(
      logger_, "Failed to load joint_qugan limits from %s; using [%.3f, %.3f].",
      urdf_path.c_str(), joint_qugan_min_rad_, joint_qugan_max_rad_);
  }

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_velocity_commands_.assign(info_.joints.size(), 0.0);
  hw_acceleration_commands_.assign(info_.joints.size(), 0.0);
  prev_positions_.assign(info_.joints.size(), 0.0);
  idx_joint_qugan_ = 0;

  const auto & waist_joint = info_.joints[static_cast<std::size_t>(idx_joint_qugan_)];
  const auto has_interface = [](const auto & interfaces, const std::string & name) {
      return std::any_of(
        interfaces.begin(), interfaces.end(),
        [&name](const auto & interface) {return interface.name == name;});
    };
  if (!has_interface(waist_joint.command_interfaces, "position") ||
    !has_interface(waist_joint.command_interfaces, "velocity") ||
    !has_interface(waist_joint.command_interfaces, "acceleration") ||
    !has_interface(waist_joint.state_interfaces, "position") ||
    !has_interface(waist_joint.state_interfaces, "velocity"))
  {
    RCLCPP_ERROR(
      logger_, "joint_qugan requires position/velocity/acceleration commands and "
      "position/velocity states.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_,
    "Waist configured: USB-FDCAN port=%s channel=%d node=0x%02X init_timeout=%dms retries=%d "
    "profile_vel=%d profile_acc=%d command_hz=%.1f feedback_sync_hz=%.1f "
    "range=[%.3f, %.3f]",
    waist_port_.c_str(), bridge_channel_ + 1, waist_node_id_, waist_init_timeout_ms_,
    waist_init_retry_count_,
    waist_profile_vel_, waist_profile_acc_, waist_command_resend_hz_, waist_feedback_sync_hz_,
    joint_qugan_min_rad_, joint_qugan_max_rad_);

  service_node_ = rclcpp::Node::make_shared("waist_hardware_service");
  feedback_status_pub_ = service_node_->create_publisher<std_msgs::msg::String>(
    "/joint/waist/driver_status", rclcpp::QoS(10));
  waist_clear_error_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/waist_clear_error",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = clear_waist_error(response->message);
    });
  waist_set_zero_position_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/waist_set_zero_position",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = set_waist_zero_position(response->message);
    });

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WaistHardware::init_waist()
{
  const auto started = std::chrono::steady_clock::now();
  bool ok = false;
  for (int attempt = 1; attempt <= waist_init_retry_count_; ++attempt) {
    ok = init_waist_once(waist_init_timeout_ms_, true);
    if (ok) {
      break;
    }
    if (attempt < waist_init_retry_count_) {
      RCLCPP_WARN(
        logger_, "Waist initialization attempt %d/%d failed; retrying in 200 ms.",
        attempt, waist_init_retry_count_);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started).count();
  if (ok) {
    RCLCPP_INFO(logger_, "Waist initialized in %lld ms.", static_cast<long long>(elapsed_ms));
  } else {
    RCLCPP_ERROR(
      logger_, "Waist initialization failed after %lld ms.",
      static_cast<long long>(elapsed_ms));
  }
  return ok;
}

bool WaistHardware::init_waist_once(int timeout_ms, bool verbose)
{
  // The drive does not publish 0x300 feedback continuously while disabled.
  // The vendor workbook requires an empty 0x080 CAN-FD+BRS sync frame to
  // request one feedback sample without changing torque or position.
  const std::array<uint8_t, 8> sync_payload{};
  if (!send_frame(waist_can::kSyncId, sync_payload, 0)) {
    if (verbose) {
      RCLCPP_ERROR(logger_, "Waist init: failed to send vendor 0x080 sync frame.");
    }
    return false;
  }
  if (!poll_waist_feedback(timeout_ms) || !poll_waist_position(0)) {
    if (verbose) {
      RCLCPP_ERROR(logger_, "Waist init: no valid 0x300 feedback within %d ms.", timeout_ms);
    }
    return false;
  }

  if (manufacturer_error_active()) {
    const auto clear_high = waist_can::build_clear_error_frame_command(waist_node_id_, true);
    const auto clear_low = waist_can::build_clear_error_frame_command(waist_node_id_, false);
    if (!send_frame(clear_high.can_id, clear_high.data, clear_high.dlc)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!send_frame(clear_low.can_id, clear_low.data, clear_low.dlc)) {
      return false;
    }

    const auto clear_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < clear_deadline) {
      (void)poll_waist_feedback(50);
      if (waist_feedback_valid_ && !manufacturer_error_active()) {
        break;
      }
    }
    if (manufacturer_error_active()) {
      if (verbose) {
        RCLCPP_ERROR(
          logger_, "Waist init: clear-error failed, state=0x%02X error=0x%04X.",
          static_cast<unsigned int>(waist_feedback_state_),
          static_cast<unsigned int>(waist_feedback_err_code_));
      }
      return false;
    }
  }

  // Match the proven waist_console E path exactly. The custom 0x100 command
  // is accepted only after this drive has entered CiA-402 operation-enabled.
  // Each transition is sent once; there is no retrying state machine.
  const int sdo_timeout_ms = std::max(50, timeout_ms);
  if (!sdo_write_i32(
      waist_can::kIndexTargetVelocity, 0x00, 0, sdo_timeout_ms))
  {
    if (verbose) {
      RCLCPP_ERROR(logger_, "Waist init: zero target velocity was not acknowledged.");
    }
    return false;
  }
  if (!sdo_write_i8(
      waist_can::kIndexMode, 0x00, waist_can::kModeProfileVelocity, sdo_timeout_ms))
  {
    if (verbose) {
      RCLCPP_ERROR(logger_, "Waist init: profile-velocity mode was not acknowledged.");
    }
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  constexpr std::pair<uint16_t, const char *> kEnableSequence[] = {
    {waist_can::kControlwordShutdown, "shutdown"},
    {waist_can::kControlwordSwitchOn, "switch-on"},
    {waist_can::kControlwordEnableOperation, "enable-operation"},
  };
  for (const auto & [controlword, stage] : kEnableSequence) {
    if (!sdo_write_u16(
        waist_can::kIndexControlword, 0x00, controlword, sdo_timeout_ms))
    {
      if (verbose) {
        RCLCPP_ERROR(
          logger_, "Waist init: console-style %s transition was not acknowledged.", stage);
      }
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
  }

  const int32_t safe_target_raw = waist_feedback_pos_raw_;
  const auto safe_command = waist_can::build_position_frame_command(
    waist_node_id_, safe_target_raw, waist_profile_acc_, waist_profile_vel_);
  if (!send_frame(safe_command.can_id, safe_command.data, safe_command.dlc)) {
    if (verbose) {
      RCLCPP_ERROR(logger_, "Waist init: safe enable frame failed to send.");
    }
    return false;
  }

  const auto enable_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < enable_deadline) {
    (void)poll_waist_feedback(50);
    if (manufacturer_error_active()) {
      if (verbose) {
        RCLCPP_ERROR(
          logger_, "Waist init: enable reported error state=0x%02X error=0x%04X.",
          static_cast<unsigned int>(waist_feedback_state_),
          static_cast<unsigned int>(waist_feedback_err_code_));
      }
      return false;
    }
    const bool enabled =
      (waist_feedback_state_ & waist_can::kFeedbackStateEnabled) != 0U;
    const bool brake_released =
      (waist_feedback_state_ & waist_can::kFeedbackStateBrakeReleased) != 0U;
    if (waist_feedback_valid_ && enabled && brake_released) {
      (void)poll_waist_position(0);
      return waist_position_valid_;
    }
  }

  if (verbose) {
    RCLCPP_ERROR(
      logger_, "Waist init: enable confirmation timed out, state=0x%02X error=0x%04X.",
      static_cast<unsigned int>(waist_feedback_state_),
      static_cast<unsigned int>(waist_feedback_err_code_));
  }
  return false;
}

bool WaistHardware::clear_waist_error(std::string & detail)
{
  if (!can_transport_ready()) {
    detail = "waist transport not ready";
    return false;
  }

  const auto clear_high = waist_can::build_clear_error_frame_command(waist_node_id_, true);
  const auto clear_low = waist_can::build_clear_error_frame_command(waist_node_id_, false);
  if (!send_frame(clear_high.can_id, clear_high.data, clear_high.dlc)) {
    detail = "clear-error high pulse send failed";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!send_frame(clear_low.can_id, clear_low.data, clear_low.dlc)) {
    detail = "clear-error low pulse send failed";
    return false;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(waist_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    (void)poll_waist_feedback(50);
    if (waist_feedback_valid_ && !manufacturer_error_active()) {
      waist_initialized_ = init_waist_once(waist_init_timeout_ms_, false);
      detail = waist_initialized_ ? "waist error cleared and drive enabled" :
        "waist error cleared but enable confirmation failed";
      return waist_initialized_;
    }
  }

  char message[128];
  std::snprintf(
    message, sizeof(message), "clear-error timeout: state=0x%02X error=0x%04X",
    static_cast<unsigned int>(waist_feedback_state_),
    static_cast<unsigned int>(waist_feedback_err_code_));
  detail = message;
  return false;
}

bool WaistHardware::set_waist_zero_position(std::string & detail)
{
  if (!can_transport_ready()) {
    detail = "waist transport not ready";
    return false;
  }

  int32_t before_raw = 0;
  const bool before_ok = sdo_read_i32(
    waist_can::kIndexActualPosition, 0x00, before_raw, waist_timeout_ms_);
  if (!sdo_write_u16(
      waist_can::kIndexControlword,
      0x00,
      waist_can::kControlwordDisableVoltage,
      waist_timeout_ms_))
  {
    detail = "waist zero failed: disable command was not acknowledged";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!sdo_write_u32(
      waist_can::kIndexSetZeroPosition, 0x00, 1U, waist_timeout_ms_))
  {
    detail = "waist zero failed: 0x2531:00 was not acknowledged";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int32_t after_raw = 0;
  const bool after_ok = sdo_read_i32(
    waist_can::kIndexActualPosition, 0x00, after_raw, waist_timeout_ms_);
  waist_initialized_ = false;
  waist_feedback_valid_ = false;
  waist_position_valid_ = false;
  hw_commands_[idx_joint_qugan_] = 0.0;
  hw_velocity_commands_[idx_joint_qugan_] = 0.0;

  char message[192];
  std::snprintf(
    message, sizeof(message),
    "waist zero set; before_raw=%s%d after_raw=%s%d; drive remains disabled",
    before_ok ? "" : "unavailable/", before_raw,
    after_ok ? "" : "unavailable/", after_raw);
  detail = message;
  RCLCPP_WARN(logger_, "%s", detail.c_str());
  return true;
}

}  // namespace joint_hardware

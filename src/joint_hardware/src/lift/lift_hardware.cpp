#include "joint_hardware/lift_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_hardware/lift/pdo_mapping.hpp"

namespace joint_hardware
{

namespace
{

bool parse_bool(const std::string & text, bool & value)
{
  std::string normalized;
  normalized.reserve(text.size());
  for (const char character : text) {
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
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

bool get_string(
  const hardware_interface::HardwareInfo & info, const std::string & key, std::string & value)
{
  const auto iterator = info.hardware_parameters.find(key);
  if (iterator == info.hardware_parameters.end()) {
    return false;
  }
  value = iterator->second;
  return true;
}

bool get_int(
  const hardware_interface::HardwareInfo & info, const std::string & key, int & value)
{
  std::string text;
  if (!get_string(info, key, text)) {
    return false;
  }
  try {
    std::size_t used = 0;
    const auto parsed = std::stoll(text, &used, 0);
    if (used != text.size() || parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max())
    {
      return false;
    }
    value = static_cast<int>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool get_uint32(
  const hardware_interface::HardwareInfo & info, const std::string & key, uint32_t & value)
{
  std::string text;
  if (!get_string(info, key, text)) {
    return false;
  }
  try {
    std::size_t used = 0;
    const auto parsed = std::stoull(text, &used, 0);
    if (used != text.size() || parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool get_double(
  const hardware_interface::HardwareInfo & info, const std::string & key, double & value)
{
  std::string text;
  if (!get_string(info, key, text)) {
    return false;
  }
  try {
    std::size_t used = 0;
    const auto parsed = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

template<typename T>
bool get_optional(
  const hardware_interface::HardwareInfo & info, const std::string & key, T & value,
  bool (* parser)(const hardware_interface::HardwareInfo &, const std::string &, T &))
{
  if (info.hardware_parameters.find(key) == info.hardware_parameters.end()) {
    return true;
  }
  return parser(info, key, value);
}

bool get_bool_optional(
  const hardware_interface::HardwareInfo & info, const std::string & key, bool & value)
{
  std::string text;
  if (!get_string(info, key, text)) {
    return true;
  }
  return parse_bool(text, value);
}

std::string link_state_string(lift::EthercatLinkState state)
{
  switch (state) {
    case lift::EthercatLinkState::offline:
      return "offline";
    case lift::EthercatLinkState::init:
      return "init";
    case lift::EthercatLinkState::pre_operational:
      return "pre_operational";
    case lift::EthercatLinkState::safe_operational:
      return "safe_operational";
    case lift::EthercatLinkState::operational:
      return "operational";
  }
  return "unknown";
}

lift::EthercatLinkState link_state_from_code(uint8_t code)
{
  switch (static_cast<lift::EthercatLinkState>(code)) {
    case lift::EthercatLinkState::offline:
      return lift::EthercatLinkState::offline;
    case lift::EthercatLinkState::init:
      return lift::EthercatLinkState::init;
    case lift::EthercatLinkState::pre_operational:
      return lift::EthercatLinkState::pre_operational;
    case lift::EthercatLinkState::safe_operational:
      return lift::EthercatLinkState::safe_operational;
    case lift::EthercatLinkState::operational:
      return lift::EthercatLinkState::operational;
  }
  return lift::EthercatLinkState::offline;
}

const char * error_reason_text(uint8_t code)
{
  switch (code) {
    case 1:
      return "PDO exchange failed";
    case 2:
      return "feedback coordinate jump exceeded max_feedback_jump_m";
    case 3:
      return "feedback position is outside software limits";
    case 4:
      return "zero offset persistence failed";
    case 5:
      return "actual velocity exceeded the configured mechanical limit";
    case 6:
      return "non-finite ros2_control command";
    case 7:
      return "CiA 402 fault or mode mismatch";
    case 8:
      return "emergency stop latched";
    default:
      return "";
  }
}

std::string cia_state_string(lift::Cia402State state)
{
  switch (state) {
    case lift::Cia402State::not_ready_to_switch_on:
      return "not_ready";
    case lift::Cia402State::switch_on_disabled:
      return "switch_on_disabled";
    case lift::Cia402State::ready_to_switch_on:
      return "ready_to_switch_on";
    case lift::Cia402State::switched_on:
      return "switched_on";
    case lift::Cia402State::operation_enabled:
      return "operation_enabled";
    case lift::Cia402State::quick_stop_active:
      return "quick_stop_active";
    case lift::Cia402State::fault_reaction_active:
      return "fault_reaction_active";
    case lift::Cia402State::fault:
      return "fault";
    case lift::Cia402State::unknown:
      return "unknown";
  }
  return "unknown";
}

const char * brake_stop_phase_string(uint8_t phase) noexcept
{
  switch (phase) {
    case 0:
      return "disabled";
    case 1:
      return "idle";
    case 2:
      return "quick_stop";
  }
  return "unknown";
}

template<typename T>
bool decode_u32_le(const std::vector<uint8_t> & bytes, T & result)
{
  static_assert(sizeof(T) == sizeof(uint32_t), "SDO decoder expects uint32_t");
  if (bytes.empty() || bytes.size() > sizeof(uint32_t)) {
    return false;
  }
  uint32_t value = 0;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    value |= static_cast<uint32_t>(bytes[i]) << (8U * i);
  }
  result = static_cast<T>(value);
  return true;
}

std::vector<uint8_t> encode_u32_le(uint32_t value)
{
  return {
    static_cast<uint8_t>(value & 0xffU), static_cast<uint8_t>((value >> 8U) & 0xffU),
    static_cast<uint8_t>((value >> 16U) & 0xffU), static_cast<uint8_t>((value >> 24U) & 0xffU)};
}

std::vector<uint8_t> encode_i32_le(int32_t value)
{
  return encode_u32_le(static_cast<uint32_t>(value));
}

}  // namespace

LiftHardware::LiftHardware()
: backend_(lift::make_lift_ethercat_backend("unavailable")), cia402_(3)
{
}

LiftHardware::LiftHardware(std::unique_ptr<lift::LiftEthercatBackend> backend)
: backend_(std::move(backend)), backend_injected_(true), cia402_(3)
{
  if (!backend_) {
    backend_ = lift::make_lift_ethercat_backend("unavailable");
  }
}

LiftHardware::~LiftHardware()
{
  stop_service_thread();
  if (backend_) {
    (void)backend_->stop();
  }
}

hardware_interface::CallbackReturn LiftHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.empty()) {
    RCLCPP_ERROR(logger_, "Lift hardware requires the joint_motor joint");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!backend_injected_) {
    backend_ = lift::make_lift_ethercat_backend(backend_name_);
  }
  initialize_interfaces();
  if (idx_joint_motor_ < 0) {
    RCLCPP_ERROR(logger_, "Lift hardware requires a joint named joint_motor");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool LiftHardware::parse_parameters()
{
  std::string text;
  if (get_string(info_, "ethercat_backend", text)) {
    backend_name_ = text;
  }
  if (get_string(info_, "motor_id", text)) {
    motor_id_ = text;
  }
  if (get_string(info_, "zero_offset_file", text)) {
    zero_offset_file_ = text;
  }
  if (!get_bool_optional(info_, "use_persistent_zero_offset", use_persistent_zero_offset_)) {
    RCLCPP_ERROR(logger_, "use_persistent_zero_offset must be boolean");
    return false;
  }
  if (get_string(info_, "ethercat_interface", text)) {
    master_config_.interface_name = text;
  }
  if (!get_optional(info_, "ethercat_master_index", master_config_.master_index, get_uint32)) {
    RCLCPP_ERROR(logger_, "ethercat_master_index must be an unsigned integer");
    return false;
  }

  int integer = 10;
  if (!get_optional(info_, "ethercat_cycle_ms", integer, get_int) || integer <= 0) {
    RCLCPP_ERROR(logger_, "ethercat_cycle_ms must be a positive integer");
    return false;
  }
  master_config_.cycle_period = std::chrono::milliseconds(integer);
  if (master_config_.cycle_period != std::chrono::milliseconds(10)) {
    RCLCPP_ERROR(logger_, "The lift EtherCAT cycle must be exactly 10 ms (100 Hz)");
    return false;
  }
  if (get_optional(info_, "feedback_timeout_ms", feedback_timeout_ms_, get_int) &&
    feedback_timeout_ms_ <= 0)
  {
    RCLCPP_ERROR(logger_, "feedback_timeout_ms must be positive");
    return false;
  }
  if (!get_optional(info_, "expected_working_counter", expected_working_counter_, get_uint32) ||
    expected_working_counter_ == 0)
  {
    RCLCPP_ERROR(logger_, "expected_working_counter must be positive");
    return false;
  }
  master_config_.expected_working_counter = expected_working_counter_;
  integer = 0;
  if (get_optional(info_, "slave_alias", integer, get_int)) {
    if (integer < 0 || integer > std::numeric_limits<uint16_t>::max()) {
      RCLCPP_ERROR(logger_, "slave_alias is outside uint16 range");
      return false;
    }
    slave_config_.alias = static_cast<uint16_t>(integer);
  } else {
    return false;
  }
  integer = 0;
  if (get_optional(info_, "slave_position", integer, get_int)) {
    if (integer < 0 || integer > std::numeric_limits<uint16_t>::max()) {
      RCLCPP_ERROR(logger_, "slave_position is outside uint16 range");
      return false;
    }
    slave_config_.position = static_cast<uint16_t>(integer);
  } else {
    return false;
  }
  get_optional(info_, "slave_vendor_id", slave_config_.vendor_id, get_uint32);
  get_optional(info_, "slave_product_code", slave_config_.product_code, get_uint32);
  get_optional(info_, "pdo_rx_logical_address", slave_config_.rx_pdo_logical_address, get_uint32);
  get_optional(info_, "pdo_tx_logical_address", slave_config_.tx_pdo_logical_address, get_uint32);
  uint32_t dc_assign_activate = slave_config_.dc_assign_activate;
  if (!get_optional(info_, "dc_assign_activate", dc_assign_activate, get_uint32) ||
    dc_assign_activate > std::numeric_limits<uint16_t>::max())
  {
    RCLCPP_ERROR(logger_, "dc_assign_activate must fit uint16");
    return false;
  }
  slave_config_.dc_assign_activate = static_cast<uint16_t>(dc_assign_activate);
  if (!get_optional(
      info_, "dc_sync0_shift_ns", slave_config_.dc_sync0_shift_ns, get_int))
  {
    RCLCPP_ERROR(logger_, "dc_sync0_shift_ns must be an integer");
    return false;
  }
  uint32_t rx_pdo_bytes = slave_config_.rx_pdo_bytes;
  uint32_t tx_pdo_bytes = slave_config_.tx_pdo_bytes;
  if (!get_optional(info_, "pdo_rx_bytes", rx_pdo_bytes, get_uint32) ||
    !get_optional(info_, "pdo_tx_bytes", tx_pdo_bytes, get_uint32) ||
    rx_pdo_bytes > std::numeric_limits<uint16_t>::max() ||
    tx_pdo_bytes > std::numeric_limits<uint16_t>::max())
  {
    RCLCPP_ERROR(logger_, "PDO byte lengths must fit uint16");
    return false;
  }
  slave_config_.rx_pdo_bytes = static_cast<uint16_t>(rx_pdo_bytes);
  slave_config_.tx_pdo_bytes = static_cast<uint16_t>(tx_pdo_bytes);
  if (slave_config_.rx_pdo_bytes != sizeof(lift::LiftRxPdo) ||
    slave_config_.tx_pdo_bytes != sizeof(lift::LiftTxPdo))
  {
    RCLCPP_ERROR(logger_, "LD3M CSV PDO sizes must be Rx=8 bytes and Tx=17 bytes");
    return false;
  }

  command_units_parameter_set_ =
    info_.hardware_parameters.find("command_units_per_rev") != info_.hardware_parameters.end();
  if (!get_optional(
      info_, "command_units_per_rev", configured_command_units_per_rev_,
      get_uint32) ||
    configured_command_units_per_rev_ == 0)
  {
    RCLCPP_ERROR(logger_, "command_units_per_rev must be positive");
    return false;
  }
  unit_config_.command_units_per_rev = configured_command_units_per_rev_;
  if (!get_optional(
      info_, "encoder_counts_per_rev", unit_config_.encoder_counts_per_rev,
      get_uint32) ||
    unit_config_.encoder_counts_per_rev == 0)
  {
    RCLCPP_ERROR(logger_, "encoder_counts_per_rev must be positive");
    return false;
  }
  if (!get_optional(info_, "lift_sign", unit_config_.lift_sign, get_double) ||
    !get_optional(info_, "lead_mm_per_rev", unit_config_.lead_mm_per_rev, get_double))
  {
    RCLCPP_ERROR(logger_, "lift_sign and lead_mm_per_rev must be finite numbers");
    return false;
  }
  if (!get_optional(info_, "position_min_m", position_min_m_, get_double) ||
    !get_optional(info_, "position_max_m", position_max_m_, get_double) ||
    !get_optional(
      info_, "reset_max_search_travel_m", reset_max_search_travel_m_, get_double))
  {
    RCLCPP_ERROR(logger_, "position software limits must be finite numbers");
    return false;
  }
  if (!std::isfinite(position_min_m_) || !std::isfinite(position_max_m_)) {
    RCLCPP_ERROR(logger_, "position_min_m and position_max_m must be finite numbers");
    return false;
  }
  if (position_min_m_ > position_max_m_) {
    std::swap(position_min_m_, position_max_m_);
  }
  if (!std::isfinite(reset_max_search_travel_m_) || reset_max_search_travel_m_ <= 0.0) {
    RCLCPP_ERROR(logger_, "reset_max_search_travel_m must be finite and positive");
    return false;
  }

  if (!get_optional(info_, "max_rpm", max_rpm_, get_int)) {
    RCLCPP_ERROR(logger_, "max_rpm is invalid");
    return false;
  }
  if (max_rpm_ <= 0) {
    RCLCPP_ERROR(logger_, "max_rpm must be positive");
    return false;
  }
  if (!get_optional(info_, "kp_rpm_per_m", kp_rpm_per_m_, get_double) ||
    !get_optional(info_, "kd_rpm_per_mps", kd_rpm_per_mps_, get_double) ||
    !get_optional(info_, "deadband_m", deadband_m_, get_double) ||
    !get_optional(info_, "slowdown_distance_m", slowdown_distance_m_, get_double) ||
    !get_optional(info_, "stop_window_m", stop_window_m_, get_double) ||
    !get_optional(info_, "stop_target_stable_ms", stop_target_stable_ms_, get_int) ||
    !get_optional(info_, "stop_window_max_rpm", stop_window_max_rpm_, get_int) ||
    !get_optional(info_, "feedback_filter_alpha", feedback_filter_alpha_, get_double) ||
    !get_optional(info_, "velocity_slew_rpm_per_s", velocity_slew_rpm_per_s_, get_double) ||
    !get_optional(info_, "stop_slew_rpm_per_s", stop_slew_rpm_per_s_, get_double) ||
    !get_optional(info_, "brake_accel_rpm_per_s", brake_accel_rpm_per_s_, get_double) ||
    !get_optional(info_, "overshoot_guard_window_m", overshoot_guard_window_m_, get_double) ||
    !get_optional(info_, "overshoot_recovery_max_rpm", overshoot_recovery_max_rpm_, get_int) ||
    !get_optional(
      info_, "stationary_velocity_threshold_mps", stationary_velocity_threshold_mps_,
      get_double) ||
    !get_optional(info_, "command_epsilon_m", command_epsilon_m_, get_double) ||
    !get_optional(info_, "max_feedback_jump_m", max_feedback_jump_m_, get_double) ||
    !get_optional(info_, "motion_command_period_ms", motion_command_period_ms_, get_int) ||
    !get_optional(info_, "reset_velocity_rpm", reset_velocity_rpm_, get_int) ||
    !get_optional(info_, "reset_velocity_period_ms", reset_velocity_period_ms_, get_int) ||
    !get_optional(info_, "homing_method", homing_method_, get_int) ||
    !get_optional(info_, "homing_speed_high_units_s", homing_speed_high_units_s_, get_int) ||
    !get_optional(info_, "homing_speed_low_units_s", homing_speed_low_units_s_, get_int) ||
    !get_optional(
      info_, "homing_acceleration_units_s2", homing_acceleration_units_s2_, get_int) ||
    !get_optional(info_, "homing_offset_units", homing_offset_units_, get_int) ||
    !get_optional(info_, "homing_timeout_ms", homing_timeout_ms_, get_int) ||
    !get_optional(info_, "drive_zero_timeout_ms", drive_zero_timeout_ms_, get_int) ||
    !get_optional(
      info_, "position_limit_recovery_max_rpm", position_limit_recovery_max_rpm_, get_int) ||
    !get_optional(info_, "max_fault_reset_attempts", max_fault_reset_attempts_, get_int) ||
    !get_optional(info_, "fault_reset_backoff_ms", fault_reset_backoff_ms_, get_int) ||
    !get_optional(
      info_, "mode_mismatch_debounce_cycles", mode_mismatch_debounce_cycles_, get_int) ||
    !get_optional(info_, "brake_release_wait_ms", brake_release_wait_ms_, get_int) ||
    !get_optional(info_, "brake_p04_37_ms", brake_p04_37_ms_, get_int) ||
    !get_optional(info_, "brake_p04_39_rpm", brake_p04_39_rpm_, get_int) ||
    !get_optional(info_, "brake_p06_14_ms", brake_p06_14_ms_, get_int) ||
    !get_optional(info_, "limit_switch_positive_bit", limit_switch_positive_bit_, get_int) ||
    !get_optional(info_, "limit_switch_negative_bit", limit_switch_negative_bit_, get_int))
  {
    RCLCPP_ERROR(logger_, "Invalid numeric lift controller parameter");
    return false;
  }
  // The lift_velocity_* names are the public tuning contract. Legacy short
  // names remain accepted so an installed real-hardware launch is not broken.
  if (!get_optional(info_, "lift_velocity_kp_rpm_per_m", kp_rpm_per_m_, get_double) ||
    !get_optional(info_, "lift_velocity_kd_rpm_per_mps", kd_rpm_per_mps_, get_double) ||
    !get_optional(info_, "lift_velocity_deadband_m", deadband_m_, get_double) ||
    !get_optional(
      info_, "lift_velocity_slowdown_distance_m", slowdown_distance_m_, get_double) ||
    !get_optional(info_, "lift_velocity_stop_window_m", stop_window_m_, get_double) ||
    !get_optional(
      info_, "lift_velocity_stop_target_stable_ms", stop_target_stable_ms_, get_int) ||
    !get_optional(
      info_, "lift_velocity_stop_window_max_rpm", stop_window_max_rpm_, get_int) ||
    !get_optional(
      info_, "lift_velocity_feedback_filter_alpha", feedback_filter_alpha_, get_double) ||
    !get_optional(
      info_, "lift_velocity_command_epsilon_rpm", command_epsilon_rpm_, get_double) ||
    !get_optional(
      info_, "lift_velocity_slew_rpm_per_s", velocity_slew_rpm_per_s_, get_double) ||
    !get_optional(
      info_, "lift_velocity_stop_slew_rpm_per_s", stop_slew_rpm_per_s_, get_double) ||
    !get_optional(
      info_, "lift_velocity_brake_accel_rpm_per_s", brake_accel_rpm_per_s_, get_double) ||
    !get_optional(
      info_, "lift_velocity_overshoot_guard_window_m", overshoot_guard_window_m_, get_double) ||
    !get_optional(
      info_, "lift_velocity_overshoot_recovery_max_rpm",
      overshoot_recovery_max_rpm_, get_int) ||
    !get_optional(
      info_, "lift_stationary_velocity_threshold_mps",
      stationary_velocity_threshold_mps_, get_double) ||
    !get_optional(info_, "lift_motion_command_period_ms", motion_command_period_ms_, get_int) ||
    !get_optional(info_, "lift_startup_motion_guard_ms", startup_motion_guard_ms_, get_int) ||
    !get_optional(info_, "reset_velocity_timeout_ms", reset_velocity_timeout_ms_, get_int) ||
    !get_optional(
      info_, "max_feedback_velocity_mps", max_feedback_velocity_mps_, get_double) ||
    !get_optional(
      info_, "feedback_velocity_tolerance_mps", feedback_velocity_tolerance_mps_, get_double))
  {
    RCLCPP_ERROR(logger_, "Invalid lift_velocity_* safety parameter");
    return false;
  }
  get_string(info_, "brake_p05_06_mode", brake_p05_06_mode_);
  get_string(info_, "brake_p05_10_mode", brake_p05_10_mode_);
  if (!get_bool_optional(info_, "brake_control_enabled", brake_control_enabled_) ||
    !get_bool_optional(info_, "limit_switch_enabled", limit_switch_enabled_) ||
    !get_bool_optional(info_, "limit_switch_active_high", limit_switch_active_high_) ||
    !get_bool_optional(info_, "direct_stop_in_stop_window", direct_stop_in_stop_window_) ||
    !get_bool_optional(info_, "auto_fault_reset", auto_fault_reset_) ||
    !get_bool_optional(
      info_, "lift_velocity_direct_stop_in_stop_window", direct_stop_in_stop_window_) ||
    !get_bool_optional(
      info_, "lift_startup_motion_guard_enabled", startup_motion_guard_enabled_) ||
    !get_bool_optional(
      info_, "reset_velocity_debug_enabled", reset_velocity_debug_enabled_))
  {
    RCLCPP_ERROR(logger_, "Invalid boolean lift hardware parameter");
    return false;
  }
  if (feedback_filter_alpha_ < 0.0 || feedback_filter_alpha_ > 1.0 ||
    deadband_m_ < 0.0 || slowdown_distance_m_ < 0.0 || stop_window_m_ < 0.0 ||
    motion_command_period_ms_ <= 0 || reset_velocity_period_ms_ <= 0 ||
    max_fault_reset_attempts_ < 0 || stop_window_max_rpm_ < 0 ||
    overshoot_recovery_max_rpm_ < 0 || max_feedback_jump_m_ <= 0.0 ||
    brake_release_wait_ms_ < -1 || brake_p04_37_ms_ < 0 || brake_p04_39_rpm_ < 0 ||
    brake_p06_14_ms_ < 0 || limit_switch_positive_bit_ < -1 ||
    limit_switch_positive_bit_ > 31 || limit_switch_negative_bit_ < -1 ||
    limit_switch_negative_bit_ > 31 || command_epsilon_rpm_ < 0.0 ||
    startup_motion_guard_ms_ < 0 || reset_velocity_timeout_ms_ <= 0 ||
    homing_method_ < -6 || homing_method_ > 37 || homing_method_ == 0 ||
    homing_method_ == 36 ||
    homing_speed_high_units_s_ <= 0 || homing_speed_low_units_s_ <= 0 ||
    homing_acceleration_units_s2_ <= 0 || homing_timeout_ms_ <= 0 ||
    drive_zero_timeout_ms_ <= 0 ||
    max_feedback_velocity_mps_ <= 0.0 ||
    feedback_velocity_tolerance_mps_ < 0.0 || kp_rpm_per_m_ < 0.0 ||
    kd_rpm_per_mps_ < 0.0 || velocity_slew_rpm_per_s_ < 0.0 ||
    stop_slew_rpm_per_s_ < 0.0 || brake_accel_rpm_per_s_ < 0.0 ||
    stationary_velocity_threshold_mps_ <= 0.0 || fault_reset_backoff_ms_ < 0 ||
    mode_mismatch_debounce_cycles_ <= 0 || mode_mismatch_debounce_cycles_ > 1000 ||
    position_limit_recovery_max_rpm_ <= 0 || position_limit_recovery_max_rpm_ > max_rpm_ ||
    fault_reset_backoff_ms_ % motion_command_period_ms_ != 0)
  {
    RCLCPP_ERROR(logger_, "Invalid lift controller timing or filter parameter");
    return false;
  }
  if (brake_release_wait_ms_ < 0) {
    RCLCPP_WARN(
      logger_,
      "brake_release_wait_ms is unset; motion stays inhibited until P04.38 release timing is confirmed");
  }
  if (!limit_switch_enabled_) {
    RCLCPP_WARN(
      logger_,
      "No physical limit switch is enabled. Software limits, PDO freshness and CiA 402 faults are the primary protection");
  } else if (limit_switch_positive_bit_ < 0 && limit_switch_negative_bit_ < 0) {
    RCLCPP_ERROR(
      logger_,
      "limit_switch_enabled requires limit_switch_positive_bit and/or "
      "limit_switch_negative_bit; do not guess 60FDh bit assignments");
    return false;
  }
  cia402_ = lift::Cia402Controller(
    static_cast<uint32_t>(max_fault_reset_attempts_),
    static_cast<uint32_t>(fault_reset_backoff_ms_ / motion_command_period_ms_));
  return true;
}

void LiftHardware::initialize_interfaces()
{
  const std::size_t count = info_.joints.size();
  hw_positions_.assign(count, 0.0);
  hw_velocities_.assign(count, 0.0);
  hw_status_.assign(count, 0.0);
  hw_error_codes_.assign(count, 0.0);
  hw_modes_.assign(count, 0.0);
  hw_brake_unlocked_.assign(count, 0.0);
  hw_digital_inputs_.assign(count, 0.0);
  hw_power_enabled_.assign(count, 0.0);
  hw_commands_.assign(count, 0.0);
  hw_velocity_commands_.assign(count, 0.0);
  hw_acceleration_commands_.assign(count, 0.0);
  hw_power_commands_.assign(count, 0.0);

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint = info_.joints[joint_index];
    if (joint.name == "joint_motor") {
      idx_joint_motor_ = static_cast<int>(joint_index);
    }
    for (const auto & state : joint.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION && !state.initial_value.empty()) {
        try {
          hw_positions_[joint_index] = std::stod(state.initial_value);
        } catch (...) {
          RCLCPP_ERROR(logger_, "Invalid initial position for joint '%s'", joint.name.c_str());
        }
      }
    }
    for (const auto & command : joint.command_interfaces) {
      if (command.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_command_ = true;
      }
      if (command.name == hardware_interface::HW_IF_ACCELERATION) {
        has_acceleration_command_ = true;
      }
    }
  }
  if (idx_joint_motor_ >= 0) {
    hw_commands_[idx_joint_motor_] = hw_positions_[idx_joint_motor_];
  }
}

hardware_interface::CallbackReturn LiftHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (idx_joint_motor_ < 0 || !backend_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!configure_backend() || !load_zero_offset()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  configured_ = true;
  active_.store(false, std::memory_order_release);
  initialized_.store(false, std::memory_order_release);
  // brake_control_enabled only authorizes the compatibility service. Never
  // release a vertical-axis brake implicitly during configure/startup.
  brake_request_enable_.store(false, std::memory_order_release);
  service_disable_latched_.store(false, std::memory_order_release);
  brake_stop_requested_.store(false, std::memory_order_release);
  brake_stop_timeout_.store(false, std::memory_order_release);
  brake_stop_phase_ = BrakeStopPhase::disabled;
  brake_stop_phase_atomic_.store(
    static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
  brake_stop_start_time_ = std::chrono::steady_clock::time_point{};
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(false, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  homing_active_.store(false, std::memory_order_release);
  homing_start_requested_.store(false, std::memory_order_release);
  homing_complete_.store(false, std::memory_order_release);
  homing_failed_.store(false, std::memory_order_release);
  drive_zero_pending_.store(false, std::memory_order_release);
  zero_request_pending_.store(false, std::memory_order_release);
  zero_disable_observed_.store(false, std::memory_order_release);
  zero_offset_apply_pending_.store(false, std::memory_order_release);
  first_write_sync_pending_.store(false, std::memory_order_release);
  motion_blocked_.store(false, std::memory_order_release);
  latched_motion_reason_code_.store(0, std::memory_order_release);
  transport_fault_active_.store(false, std::memory_order_release);
  transport_recovery_healthy_cycles_.store(0, std::memory_order_release);
  error_reason_code_.store(0, std::memory_order_release);
  position_limit_violation_.store(false, std::memory_order_release);
  limit_recovery_active_.store(false, std::memory_order_release);
  mode_mismatch_cycles_.store(0, std::memory_order_release);
  feedback_continuity_.configure(
    max_feedback_jump_m_,
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::milliseconds(feedback_timeout_ms_)).count());
  feedback_continuity_reset_pending_.store(false, std::memory_order_release);
  reset_feedback_continuity();
  last_target_wire_rpm_ = 0.0;
  last_target_position_m_ = std::numeric_limits<double>::quiet_NaN();
  hw_commands_[idx_joint_motor_] = hw_positions_[idx_joint_motor_];
  hw_velocity_commands_[idx_joint_motor_] = 0.0;
  hw_acceleration_commands_[idx_joint_motor_] = 0.0;
  hw_power_commands_[idx_joint_motor_] = 0.0;
  power_command_requested_.store(false, std::memory_order_release);
  power_enabled_.store(false, std::memory_order_release);
  previous_power_command_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool LiftHardware::bootstrap_first_feedback()
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  constexpr uint32_t kRequiredStableSamples = 5;
  uint32_t stable_samples = 0;
  double candidate_position = 0.0;
  lift::LiftTxPdo last_sample{};
  lift::EthercatLinkState last_link = lift::EthercatLinkState::offline;
  uint32_t last_working_counter = 0;
  bool last_pdo_fresh = false;
  bool received_sample = false;
  uint32_t fault_reset_attempts = 0;
  auto next_fault_reset_time = std::chrono::steady_clock::time_point{};
  lift::LiftRxPdo safe_output{};
  safe_output.control_word = 0x0006;
  safe_output.target_velocity_units_per_s = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    lift::LiftTxPdo sample{};
    bool read_ok = false;
    bool write_ok = false;
    {
      std::lock_guard<std::mutex> lock(backend_mutex_);
      read_ok = backend_->read_pdo(sample);
      // The EtherLab master advances its AL state only when the application
      // queues and sends a domain.  Bootstrap cannot wait for the first OP
      // PDO without continuing to send a deterministic disabled command.
      if (read_ok) {
        safe_output.control_word = 0x0006;
        const auto now = std::chrono::steady_clock::now();
        const bool reset_allowed = auto_fault_reset_ && sample.error_code != 0U &&
          std::abs(sample.actual_velocity_units_per_s) == 0 &&
          fault_reset_attempts < max_fault_reset_attempts_ && now >= next_fault_reset_time;
        if (reset_allowed) {
          // A PDI watchdog fault is sticky.  This is the same bounded reset
          // permitted by the normal CiA402 state machine before any explicit
          // power-enable request, while velocity remains forced to zero.
          safe_output.control_word = 0x0080;
          ++fault_reset_attempts;
          next_fault_reset_time = now + std::chrono::milliseconds(fault_reset_backoff_ms_);
          RCLCPP_WARN(
            logger_, "Lift bootstrap sending bounded fault reset %u/%u for 603F=0x%04x",
            fault_reset_attempts, max_fault_reset_attempts_, sample.error_code);
        }
        write_ok = backend_->write_pdo(safe_output);
      }
    }
    if (read_ok && write_ok) {
      const auto link = backend_->link_state();
      const bool fresh = backend_->pdo_fresh() &&
        link == lift::EthercatLinkState::operational &&
        backend_->working_counter() >= expected_working_counter_;
      last_sample = sample;
      last_link = link;
      last_working_counter = backend_->working_counter();
      last_pdo_fresh = backend_->pdo_fresh();
      received_sample = true;
      link_state_atomic_.store(static_cast<uint8_t>(link), std::memory_order_release);
      working_counter_atomic_.store(backend_->working_counter(), std::memory_order_release);
      if (fresh) {
        const double position = lift::position_units_to_m(
          sample.actual_position_units, zero_offset_units_, unit_config_);
        const double velocity = lift::velocity_units_to_mps(
          sample.actual_velocity_units_per_s, unit_config_);
        const bool valid_stationary_sample = std::isfinite(position) &&
          std::isfinite(velocity) && sample.mode_display == 9 &&
          lift::parse_cia402_status(sample.status_word) != lift::Cia402State::unknown &&
          std::abs(velocity) <= stationary_velocity_threshold_mps_;
        if (!valid_stationary_sample) {
          stable_samples = 0;
        } else {
          if (stable_samples == 0 ||
            std::abs(position - candidate_position) > max_feedback_jump_m_)
          {
            candidate_position = position;
            stable_samples = 1;
          } else {
            candidate_position = position;
            ++stable_samples;
          }
        }
        if (stable_samples >= kRequiredStableSamples) {
          rx_pdo_ = sample;
          feedback_position_units_.store(sample.actual_position_units, std::memory_order_release);
          status_word_.store(sample.status_word, std::memory_order_release);
          error_code_.store(sample.error_code, std::memory_order_release);
          mode_display_.store(sample.mode_display, std::memory_order_release);
          digital_inputs_.store(sample.digital_inputs, std::memory_order_release);
          hw_positions_[idx_joint_motor_] = position;
          hw_velocities_[idx_joint_motor_] = velocity;
          hw_commands_[idx_joint_motor_] = position;
          hw_velocity_commands_[idx_joint_motor_] = 0.0;
          hw_acceleration_commands_[idx_joint_motor_] = 0.0;
          hw_power_commands_[idx_joint_motor_] = 0.0;
          feedback_position_m_.store(position, std::memory_order_release);
          feedback_velocity_mps_.store(velocity, std::memory_order_release);
          filtered_velocity_mps_ = velocity;
          const auto now = std::chrono::steady_clock::now();
          last_feedback_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            std::memory_order_release);
          feedback_fresh_.store(true, std::memory_order_release);
          update_state_interfaces();
          return true;
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(backend_mutex_);
      (void)backend_->write_pdo(safe_output);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(
    logger_,
    "Lift bootstrap details: received=%s link=%d pdo_fresh=%s wc=%u status=0x%04x "
    "error=0x%04x mode=%d velocity=%d stable_samples=%u",
    received_sample ? "true" : "false", static_cast<int>(last_link),
    last_pdo_fresh ? "true" : "false", last_working_counter, last_sample.status_word,
    last_sample.error_code, static_cast<int>(last_sample.mode_display),
    last_sample.actual_velocity_units_per_s, stable_samples);
  return false;
}

bool LiftHardware::configure_backend()
{
  if (!backend_->initialize(master_config_)) {
    RCLCPP_ERROR(
      logger_, "EtherCAT backend initialize failed: %s",
      backend_->error_message().c_str());
    return false;
  }
  if (!backend_->configure_slave(slave_config_)) {
    RCLCPP_ERROR(
      logger_, "EtherCAT slave configuration failed: %s",
      backend_->error_message().c_str());
    return false;
  }
  std::string pdo_error;
  if (!lift::configure_lift_csv_pdos(*backend_, pdo_error)) {
    RCLCPP_ERROR(logger_, "Failed to configure LD3M CSV PDO mapping: %s", pdo_error.c_str());
    return false;
  }
  const std::vector<uint8_t> csv_mode{9};
  if (!backend_->write_sdo(0x6060, 0, csv_mode)) {
    RCLCPP_ERROR(logger_, "Failed to write 6060h=9 (CSV): %s", backend_->error_message().c_str());
    return false;
  }
  std::vector<uint8_t> actual_mode;
  if (!backend_->read_sdo(
      0x6061, 0,
      actual_mode) || actual_mode.size() != 1 || actual_mode[0] != 9)
  {
    RCLCPP_ERROR(logger_, "Drive did not confirm 6061h=9 (CSV)");
    return false;
  }
  uint32_t encoder_resolution = 0;
  if (!read_validated_sdo(0x608f, 1, encoder_resolution)) {
    return false;
  }
  unit_config_.encoder_counts_per_rev = encoder_resolution;
  unit_config_.encoder_counts_denominator = 1;
  if (!read_validated_sdo(0x6091, 1, unit_config_.gear_ratio_numerator) ||
    !read_validated_sdo(0x6091, 2, unit_config_.gear_ratio_denominator) ||
    !read_validated_sdo(0x6092, 1, unit_config_.feed_units_numerator))
  {
    return false;
  }

  // P00.08/2008:00 is the drive's highest-priority command-unit setting.  It
  // is read during configuration only; no SDO access is performed in read().
  if (!read_validated_sdo(0x2008, 0, unit_config_.p00_08_command_units_per_rev, true)) {
    return false;
  }

  double units_per_rev = 0.0;
  std::string units_error;
  if (!lift::derive_command_units_per_rev(
      unit_config_.encoder_counts_per_rev, unit_config_.gear_ratio_numerator,
      unit_config_.gear_ratio_denominator, unit_config_.feed_units_numerator,
      unit_config_.p00_08_command_units_per_rev, units_per_rev, units_error))
  {
    RCLCPP_ERROR(logger_, "Validated command-unit conversion is invalid: %s", units_error.c_str());
    return false;
  }
  unit_config_.effective_command_units_per_rev = units_per_rev;
  unit_config_.command_units_per_rev = static_cast<uint32_t>(std::llround(units_per_rev));
  if (command_units_parameter_set_ &&
    std::abs(static_cast<double>(configured_command_units_per_rev_) - units_per_rev) >
    1e-9 * std::max(1.0, units_per_rev))
  {
    RCLCPP_ERROR(
      logger_, "Configured command_units_per_rev=%u disagrees with validated effective value %.9g",
      configured_command_units_per_rev_, units_per_rev);
    return false;
  }
  std::string error;
  if (!lift::validate_lift_units(unit_config_, error)) {
    RCLCPP_ERROR(logger_, "Lift unit validation failed: %s", error.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger_,
    "Validated CiA402 units: 608F=%u counts/rev, 6091=%u/%u, 6092=%u units/rev, 2008=%u, effective=%.9g",
    unit_config_.encoder_counts_per_rev, unit_config_.gear_ratio_numerator,
    unit_config_.gear_ratio_denominator, unit_config_.feed_units_numerator,
    unit_config_.p00_08_command_units_per_rev, units_per_rev);
  if (!validate_brake_parameters()) {
    return false;
  }
  RCLCPP_INFO(
    logger_,
    "Brake timing check: P04.37=%d ms P04.38=%d ms P04.39=%d rpm P06.14=%d ms P05.06=%s P05.10=%s",
    brake_p04_37_ms_, brake_release_wait_ms_, brake_p04_39_rpm_, brake_p06_14_ms_,
    brake_p05_06_mode_.c_str(), brake_p05_10_mode_.c_str());
  return true;
}

bool LiftHardware::validate_brake_parameters()
{
  uint32_t p04_37 = 0;
  uint32_t p04_38 = 0;
  uint32_t p04_39 = 0;
  uint32_t p06_14 = 0;
  uint32_t p05_06 = 0;
  uint32_t p05_10 = 0;
  if (!read_validated_sdo(0x2437, 0, p04_37, true) ||
    !read_validated_sdo(0x2438, 0, p04_38, true) ||
    !read_validated_sdo(0x2439, 0, p04_39, true) ||
    !read_validated_sdo(0x2614, 0, p06_14, true) ||
    !read_validated_sdo(0x2506, 0, p05_06, true) ||
    !read_validated_sdo(0x2510, 0, p05_10, true))
  {
    RCLCPP_ERROR(
      logger_,
      "Failed to read LD3M brake timing objects 2437h/2438h/2439h/2614h/2506h/2510h");
    return false;
  }
  if (p04_37 > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    p04_38 > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    p04_39 > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    p06_14 > static_cast<uint32_t>(std::numeric_limits<int>::max()))
  {
    RCLCPP_ERROR(logger_, "LD3M brake timing SDO value exceeds the supported integer range");
    return false;
  }
  if (p04_37 > 3000U || p04_38 > 3000U || p04_39 < 30U || p04_39 > 3000U ||
    p06_14 > 3000U || p05_06 < 3U || p05_06 > 4U || p05_10 < 4U || p05_10 > 5U)
  {
    RCLCPP_ERROR(
      logger_,
      "LD3M brake SDO values are outside the manual ranges: P04.37=%u P04.38=%u "
      "P04.39=%u P06.14=%u P05.06=%u P05.10=%u",
      p04_37, p04_38, p04_39, p06_14, p05_06, p05_10);
    return false;
  }
  brake_release_wait_observed_ms_ = static_cast<int>(p04_38);
  const auto mismatch = [this](const char * name, int expected, uint32_t actual) {
      if (expected < 0) {
        return false;
      }
      if (static_cast<uint32_t>(expected) != actual) {
        RCLCPP_ERROR(
          logger_, "LD3M %s mismatch: configured %d, drive reports %u", name, expected, actual);
        return true;
      }
      return false;
    };
  if (mismatch("P04.37", brake_p04_37_ms_, p04_37) ||
    mismatch("P04.39", brake_p04_39_rpm_, p04_39) ||
    mismatch("P06.14", brake_p06_14_ms_, p06_14) ||
    mismatch("P04.38", brake_release_wait_ms_, p04_38))
  {
    return false;
  }

  const auto expected_mode = [](const std::string & text, uint32_t & value) {
      if (text.empty() || text == "drive_default") {
        return false;
      }
      try {
        std::size_t used = 0;
        const auto parsed = std::stoul(text, &used, 0);
        if (used != text.size() || parsed > std::numeric_limits<uint16_t>::max()) {
          return false;
        }
        value = static_cast<uint32_t>(parsed);
        return true;
      } catch (...) {
        return false;
      }
    };
  uint32_t expected_p05_06 = 0;
  uint32_t expected_p05_10 = 0;
  if (expected_mode(brake_p05_06_mode_, expected_p05_06) && expected_p05_06 != p05_06) {
    RCLCPP_ERROR(
      logger_, "LD3M P05.06 mismatch: configured %s, drive reports %u",
      brake_p05_06_mode_.c_str(), p05_06);
    return false;
  }
  if (expected_mode(brake_p05_10_mode_, expected_p05_10) && expected_p05_10 != p05_10) {
    RCLCPP_ERROR(
      logger_, "LD3M P05.10 mismatch: configured %s, drive reports %u",
      brake_p05_10_mode_.c_str(), p05_10);
    return false;
  }
  RCLCPP_INFO(
    logger_,
    "Validated LD3M brake objects: P04.37=%u ms P04.38=%u ms P04.39=%u rpm "
    "P06.14=%u ms P05.06=%u P05.10=%u",
    p04_37, p04_38, p04_39, p06_14, p05_06, p05_10);
  if (p05_06 != 3U || p05_10 != 4U) {
    RCLCPP_WARN(
      logger_,
      "LD3M P05.06/P05.10 are not the commonly documented servo-brake defaults "
      "(3/4); verify normal-disable and alarm-stop behavior for this vertical axis");
  }
  if (brake_release_wait_ms_ < 0) {
    RCLCPP_WARN(
      logger_,
      "P04.38 is readable as %d ms but has not been explicitly confirmed; motion remains inhibited",
      brake_release_wait_observed_ms_);
  }
  return true;
}

bool LiftHardware::read_validated_sdo(
  uint16_t index, uint8_t subindex, uint32_t & value, bool allow_zero)
{
  std::vector<uint8_t> bytes;
  if (!backend_->read_sdo(index, subindex, bytes) || !decode_u32_le(bytes, value) ||
    (!allow_zero && value == 0))
  {
    RCLCPP_ERROR(
      logger_, "Failed to validate SDO 0x%04X:%02X: %s", index, subindex,
      backend_->error_message().c_str());
    return false;
  }
  return true;
}

bool LiftHardware::load_zero_offset()
{
  if (!use_persistent_zero_offset_) {
    zero_offset_units_ = 0;
    zero_offset_units_atomic_.store(0, std::memory_order_release);
    zero_offset_loaded_ = false;
    RCLCPP_INFO(
      logger_, "Persistent lift zero offset disabled; using direct LD3M 6064h coordinates");
    return true;
  }

  lift::ZeroOffsetRecord record;
  std::string error;
  const auto result = lift::ZeroOffsetStore::load(
    zero_offset_file_, motor_id_, slave_config_.alias, slave_config_.position, record, error);
  if (result == lift::ZeroOffsetLoadResult::invalid) {
    RCLCPP_ERROR(logger_, "Refuse lift startup because zero offset is invalid: %s", error.c_str());
    return false;
  }
  if (result == lift::ZeroOffsetLoadResult::loaded) {
    zero_offset_units_ = record.zero_offset_units;
    zero_offset_units_atomic_.store(zero_offset_units_, std::memory_order_release);
    zero_offset_loaded_ = true;
    RCLCPP_INFO(
      logger_, "Loaded host zero offset %d units for %s", zero_offset_units_,
      motor_id_.c_str());
  } else {
    zero_offset_units_ = 0;
    zero_offset_units_atomic_.store(0, std::memory_order_release);
    zero_offset_loaded_ = false;
    RCLCPP_WARN(
      logger_,
      "No host zero offset file found; using configured origin until /lift_reset_zero");
  }
  return true;
}

hardware_interface::CallbackReturn LiftHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_ || !backend_->start()) {
    RCLCPP_ERROR(logger_, "EtherCAT backend start failed: %s", backend_->error_message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  tx_pdo_ = lift::LiftRxPdo{};
  rx_pdo_ = lift::LiftTxPdo{};
  active_.store(false, std::memory_order_release);
  brake_enable_time_ = std::chrono::steady_clock::now();
  previous_operation_enabled_ = false;
  previous_power_command_ = false;
  hw_power_commands_[idx_joint_motor_] = 0.0;
  power_command_requested_.store(false, std::memory_order_release);
  power_enabled_.store(false, std::memory_order_release);
  service_disable_latched_.store(false, std::memory_order_release);
  link_state_atomic_.store(
    static_cast<uint8_t>(backend_->link_state()), std::memory_order_release);
  working_counter_atomic_.store(backend_->working_counter(), std::memory_order_release);
  last_feedback_ns_.store(0, std::memory_order_release);
  feedback_fresh_.store(false, std::memory_order_release);
  transport_fault_active_.store(false, std::memory_order_release);
  transport_recovery_healthy_cycles_.store(0, std::memory_order_release);
  latched_motion_reason_code_.store(0, std::memory_order_release);
  error_reason_code_.store(0, std::memory_order_release);
  feedback_position_units_.store(0, std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  brake_stop_requested_.store(false, std::memory_order_release);
  brake_stop_timeout_.store(false, std::memory_order_release);
  brake_stop_phase_ = BrakeStopPhase::disabled;
  brake_stop_phase_atomic_.store(
    static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
  brake_stop_start_time_ = std::chrono::steady_clock::time_point{};
  zero_disable_observed_.store(false, std::memory_order_release);
  zero_offset_apply_pending_.store(false, std::memory_order_release);
  cia402_.reset();
  mode_mismatch_cycles_.store(0, std::memory_order_release);
  position_limit_violation_.store(false, std::memory_order_release);
  limit_recovery_active_.store(false, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(false, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  homing_active_.store(false, std::memory_order_release);
  homing_start_requested_.store(false, std::memory_order_release);
  homing_complete_.store(false, std::memory_order_release);
  homing_failed_.store(false, std::memory_order_release);
  drive_zero_pending_.store(false, std::memory_order_release);
  last_target_wire_rpm_ = 0.0;
  last_target_position_m_ = std::numeric_limits<double>::quiet_NaN();
  feedback_continuity_reset_pending_.store(false, std::memory_order_release);
  reset_feedback_continuity();
  if (!bootstrap_first_feedback()) {
    RCLCPP_ERROR(
      logger_, "Lift activation timed out waiting for the first fresh Operational PDO: %s",
      backend_->error_message().c_str());
    (void)backend_->stop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  // bootstrap_first_feedback() rejects startup artifacts using five stable PDO
  // samples. The first ros2_control read belongs to the new controller epoch
  // and establishes its own continuity baseline; it is never compared with a
  // sample from activation or a previous calibration.
  reset_feedback_continuity();
  const auto now = std::chrono::steady_clock::now();
  startup_guard_until_ = startup_motion_guard_enabled_ ?
    now + std::chrono::milliseconds(startup_motion_guard_ms_) : now;
  first_write_sync_pending_.store(true, std::memory_order_release);
  initialized_.store(true, std::memory_order_release);
  active_.store(true, std::memory_order_release);
  start_service_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  initialized_.store(false, std::memory_order_release);
  reset_feedback_continuity();
  brake_request_enable_.store(false, std::memory_order_release);
  service_disable_latched_.store(true, std::memory_order_release);
  hw_power_commands_[idx_joint_motor_] = 0.0;
  power_command_requested_.store(false, std::memory_order_release);
  power_enabled_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  brake_stop_timeout_.store(false, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(false, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  homing_active_.store(false, std::memory_order_release);
  homing_start_requested_.store(false, std::memory_order_release);
  homing_complete_.store(false, std::memory_order_release);
  homing_failed_.store(false, std::memory_order_release);
  drive_zero_pending_.store(false, std::memory_order_release);
  // Give the drive one explicit Quick Stop frame before the final disable.
  // EtherLab's stop() also emits a final 0x0006 frame after this bounded pair.
  tx_pdo_.control_word = 0x000b;
  tx_pdo_.target_velocity_units_per_s = 0;
  if (backend_) {
    (void)backend_->write_pdo(tx_pdo_);
    tx_pdo_.control_word = 0x0006;
    (void)backend_->write_pdo(tx_pdo_);
    (void)backend_->stop();
  }
  brake_stop_phase_ = BrakeStopPhase::disabled;
  brake_stop_phase_atomic_.store(
    static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
  mode_mismatch_cycles_.store(0, std::memory_order_release);
  position_limit_violation_.store(false, std::memory_order_release);
  limit_recovery_active_.store(false, std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  stop_service_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LiftHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & state : info_.joints[i].state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_positions_[i]);
      } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_velocities_[i]);
      } else if (state.name == "status") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_status_[i]);
      } else if (state.name == "error_code") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_error_codes_[i]);
      } else if (state.name == "mode") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_modes_[i]);
      } else if (state.name == "brake_unlocked") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_brake_unlocked_[i]);
      } else if (state.name == "digital_inputs") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_digital_inputs_[i]);
      } else if (state.name == "power_enable") {
        interfaces.emplace_back(info_.joints[i].name, state.name, &hw_power_enabled_[i]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> LiftHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & command : info_.joints[i].command_interfaces) {
      if (command.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(info_.joints[i].name, command.name, &hw_commands_[i]);
      } else if (command.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(info_.joints[i].name, command.name, &hw_velocity_commands_[i]);
      } else if (command.name == hardware_interface::HW_IF_ACCELERATION) {
        interfaces.emplace_back(info_.joints[i].name, command.name, &hw_acceleration_commands_[i]);
      } else if (command.name == "power_enable") {
        interfaces.emplace_back(info_.joints[i].name, command.name, &hw_power_commands_[i]);
      }
    }
  }
  return interfaces;
}

hardware_interface::return_type LiftHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load(std::memory_order_acquire)) {
    return hardware_interface::return_type::ERROR;
  }
  lift::LiftTxPdo sample{};
  // PDO reception is separate from transmission.  write() sends the command
  // calculated by ros2_control later in this same 10 ms cycle, so a normal
  // position or velocity update does not wait for a second read period.
  bool read_ok = false;
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    read_ok = backend_->read_pdo(sample);
  }
  if (!read_ok) {
    handle_transport_fault("PDO read failed");
    return hardware_interface::return_type::ERROR;
  }
  rx_pdo_ = sample;
  feedback_position_units_.store(rx_pdo_.actual_position_units, std::memory_order_release);
  const auto now = std::chrono::steady_clock::now();
  const auto current_link = backend_->link_state();
  link_state_atomic_.store(
    static_cast<uint8_t>(current_link), std::memory_order_release);
  working_counter_atomic_.store(backend_->working_counter(), std::memory_order_release);
  const bool fresh = backend_->pdo_fresh() &&
    current_link == lift::EthercatLinkState::operational &&
    backend_->working_counter() >= expected_working_counter_;
  update_transport_health(fresh);
  if (fresh) {
    last_feedback_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
      std::memory_order_release);
  }
  feedback_fresh_.store(fresh, std::memory_order_release);
  status_word_.store(rx_pdo_.status_word, std::memory_order_release);
  error_code_.store(rx_pdo_.error_code, std::memory_order_release);
  mode_display_.store(rx_pdo_.mode_display, std::memory_order_release);
  digital_inputs_.store(rx_pdo_.digital_inputs, std::memory_order_release);
  if (homing_active_.load(std::memory_order_acquire)) {
    if (rx_pdo_.error_code != 0U || (rx_pdo_.status_word & 0x2000U) != 0U) {
      homing_failed_.store(true, std::memory_order_release);
    }
    const bool homing_done = (rx_pdo_.status_word & 0x1400U) == 0x1400U;
    if (homing_done) {
      homing_complete_.store(true, std::memory_order_release);
    }
  }
  const double position = lift::position_units_to_m(
    rx_pdo_.actual_position_units, zero_offset_units_, unit_config_);
  const double velocity = lift::velocity_units_to_mps(
    rx_pdo_.actual_velocity_units_per_s,
    unit_config_);
  if (!std::isfinite(position) || !std::isfinite(velocity)) {
    latch_motion_fault(6);
    return hardware_interface::return_type::ERROR;
  }
  filtered_velocity_mps_ = feedback_filter_alpha_ * velocity +
    (1.0 - feedback_filter_alpha_) * filtered_velocity_mps_;
  feedback_velocity_mps_.store(filtered_velocity_mps_, std::memory_order_release);
  feedback_position_m_.store(position, std::memory_order_release);
  const int64_t sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
  if (feedback_continuity_reset_pending_.exchange(false, std::memory_order_acq_rel)) {
    reset_feedback_continuity();
  }
  observe_feedback_continuity(
    position, sample_time_ns,
    fresh && !transport_fault_active_.load(std::memory_order_acquire));
  const bool reset_search_active = reset_velocity_request_.load(std::memory_order_acquire);
  const double reset_search_start =
    reset_search_start_position_m_.load(std::memory_order_acquire);
  const bool reset_upper_override = reset_search_active &&
    position >= reset_search_start - command_epsilon_m_ &&
    position <= reset_search_start + reset_max_search_travel_m_ + command_epsilon_m_;
  const bool position_outside_limits = position < position_min_m_ - command_epsilon_m_ ||
    (position > position_max_m_ + command_epsilon_m_ && !reset_upper_override);
  const bool was_outside_limits = position_limit_violation_.exchange(
    position_outside_limits, std::memory_order_acq_rel);
  if (position_outside_limits) {
    limit_recovery_active_.store(false, std::memory_order_release);
    if (!motion_blocked_.load(std::memory_order_acquire)) {
      error_reason_code_.store(3, std::memory_order_release);
      if (!was_outside_limits) {
        // Stop once on entry. The photoelectric reset may cross the normal
        // upper bound only while its bounded, watchdog-backed takeover is
        // active. A later explicit enable may recover only toward
        // the configured software range; repeatedly forcing this request here
        // would make that safe recovery impossible.
        brake_request_enable_.store(false, std::memory_order_release);
        brake_stop_requested_.store(true, std::memory_order_release);
        brake_unlocked_.store(false, std::memory_order_release);
        reset_velocity_request_.store(false, std::memory_order_release);
        reset_velocity_stop_pending_.store(true, std::memory_order_release);
        reset_hold_request_.store(false, std::memory_order_release);
        last_target_wire_rpm_ = 0.0;
      }
    }
  } else {
    limit_recovery_active_.store(false, std::memory_order_release);
  }
  if (std::abs(velocity) > max_feedback_velocity_mps_ + feedback_velocity_tolerance_mps_) {
    latch_motion_fault(5);
  }
  if (idx_joint_motor_ >= 0) {
    hw_positions_[idx_joint_motor_] = position;
    hw_velocities_[idx_joint_motor_] = velocity;
  }
  update_state_interfaces();
  return hardware_interface::return_type::OK;
}

void LiftHardware::handle_transport_fault(const char * reason)
{
  tx_pdo_.target_velocity_units_per_s = 0;
  tx_pdo_.control_word = 0x0006;
  brake_stop_requested_.store(true, std::memory_order_release);
  brake_stop_phase_ = BrakeStopPhase::disabled;
  brake_stop_phase_atomic_.store(
    static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  feedback_fresh_.store(false, std::memory_order_release);
  if (idx_joint_motor_ >= 0) {
    hw_power_enabled_[idx_joint_motor_] = 0.0;
  }
  filtered_velocity_mps_ = 0.0;
  feedback_velocity_mps_.store(0.0, std::memory_order_release);
  (void)reason;
  transport_fault_active_.store(true, std::memory_order_release);
  transport_recovery_healthy_cycles_.store(0, std::memory_order_release);
  invalidate_feedback_continuity();
  if (!motion_blocked_.load(std::memory_order_acquire)) {
    error_reason_code_.store(1, std::memory_order_release);
  }
}

uint32_t LiftHardware::feedback_source_code() const noexcept
{
  if (backend_name_ == "mock") {
    return 1U;
  }
  if (backend_name_ == "etherlab") {
    return 2U;
  }
  if (backend_name_ == "real_sdk") {
    return 3U;
  }
  return 0U;
}

void LiftHardware::reset_feedback_continuity() noexcept
{
  const uint32_t source = feedback_source_code();
  feedback_continuity_.reset(source);
  feedback_baseline_valid_.store(false, std::memory_order_release);
  feedback_epoch_.store(feedback_continuity_.epoch(), std::memory_order_release);
  feedback_source_atomic_.store(source, std::memory_order_release);
}

void LiftHardware::invalidate_feedback_continuity() noexcept
{
  const uint32_t source = feedback_source_code();
  feedback_continuity_.invalidate(source);
  feedback_baseline_valid_.store(false, std::memory_order_release);
  feedback_epoch_.store(feedback_continuity_.epoch(), std::memory_order_release);
  feedback_source_atomic_.store(source, std::memory_order_release);
}

void LiftHardware::observe_feedback_continuity(
  double position_m, int64_t sample_time_ns, bool fresh) noexcept
{
  const uint32_t source = feedback_source_code();
  const auto result = feedback_continuity_.observe(
    position_m, sample_time_ns, source, fresh);
  feedback_baseline_valid_.store(
    feedback_continuity_.baseline_valid(), std::memory_order_release);
  feedback_epoch_.store(result.epoch, std::memory_order_release);
  feedback_source_atomic_.store(result.source, std::memory_order_release);
  if (result.state != lift::FeedbackContinuityState::jump_detected) {
    return;
  }
  jump_previous_position_m_.store(result.previous_position_m, std::memory_order_release);
  jump_current_position_m_.store(result.current_position_m, std::memory_order_release);
  jump_delta_m_.store(result.delta_m, std::memory_order_release);
  jump_interval_ms_.store(result.interval_ms, std::memory_order_release);
  jump_epoch_.store(result.epoch, std::memory_order_release);
  jump_source_.store(result.source, std::memory_order_release);
  latch_motion_fault(2);
}

std::string LiftHardware::detailed_error_reason(uint8_t code) const
{
  if (code != 2U) {
    return error_reason_text(code);
  }
  const auto source_name = [this]() {
      switch (jump_source_.load(std::memory_order_acquire)) {
        case 1U:
          return "mock";
        case 2U:
          return "etherlab";
        case 3U:
          return "real_sdk";
        default:
          return "unknown";
      }
    };
  std::ostringstream reason;
  reason << std::setprecision(9)
         << "feedback coordinate jump: previous="
         << jump_previous_position_m_.load(std::memory_order_acquire)
         << "m current=" << jump_current_position_m_.load(std::memory_order_acquire)
         << "m delta=" << jump_delta_m_.load(std::memory_order_acquire)
         << "m threshold=" << max_feedback_jump_m_
         << "m interval=" << jump_interval_ms_.load(std::memory_order_acquire)
         << "ms source=" << source_name()
         << " epoch=" << jump_epoch_.load(std::memory_order_acquire);
  return reason.str();
}

void LiftHardware::update_transport_health(bool healthy) noexcept
{
  if (!healthy) {
    transport_fault_active_.store(true, std::memory_order_release);
    transport_recovery_healthy_cycles_.store(0, std::memory_order_release);
    if (!motion_blocked_.load(std::memory_order_acquire)) {
      error_reason_code_.store(1, std::memory_order_release);
    }
    return;
  }
  if (!transport_fault_active_.load(std::memory_order_acquire)) {
    return;
  }
  const uint32_t next = transport_recovery_healthy_cycles_.fetch_add(
    1, std::memory_order_acq_rel) + 1U;
  if (next < transport_recovery_required_cycles_) {
    return;
  }
  transport_fault_active_.store(false, std::memory_order_release);
  transport_recovery_healthy_cycles_.store(0, std::memory_order_release);
  if (!motion_blocked_.load(std::memory_order_acquire) &&
    !estop_latched_.load(std::memory_order_acquire))
  {
    error_reason_code_.store(
      position_limit_violation_.load(std::memory_order_acquire) ? 3 : 0,
      std::memory_order_release);
  }
}

void LiftHardware::latch_motion_fault(uint8_t reason_code) noexcept
{
  motion_blocked_.store(true, std::memory_order_release);
  latched_motion_reason_code_.store(reason_code, std::memory_order_release);
  error_reason_code_.store(reason_code, std::memory_order_release);
  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(true, std::memory_order_release);
  reset_hold_target_m_.store(
    feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
  reset_hold_request_.store(true, std::memory_order_release);
}

bool LiftHardware::feedback_is_fresh() const noexcept
{
  const int64_t stamp = last_feedback_ns_.load(std::memory_order_acquire);
  if (!feedback_fresh_.load(std::memory_order_acquire) || stamp == 0) {
    return false;
  }
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return now_ns >= stamp && now_ns - stamp <=
         std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::milliseconds(feedback_timeout_ms_)).count();
}

bool LiftHardware::compute_motion_command(double & target_wire_rpm, double period_s)
{
  target_wire_rpm = 0.0;
  limit_recovery_active_.store(false, std::memory_order_release);
  if (idx_joint_motor_ < 0 || !feedback_is_fresh() || motion_blocked_.load(
      std::memory_order_acquire))
  {
    return false;
  }
  const double feedback_position = hw_positions_[idx_joint_motor_];
  const double feedback_velocity = filtered_velocity_mps_;
  if (!std::isfinite(hw_commands_[idx_joint_motor_]) ||
    (has_velocity_command_ && !std::isfinite(hw_velocity_commands_[idx_joint_motor_])) ||
    (has_acceleration_command_ &&
    !std::isfinite(hw_acceleration_commands_[idx_joint_motor_])))
  {
    latch_motion_fault(6);
    return false;
  }
  const double sign = std::abs(unit_config_.lift_sign) > 1e-9 ? unit_config_.lift_sign : -1.0;
  if (reset_velocity_request_.load(std::memory_order_acquire)) {
    const double joint_rpm = std::clamp(
      static_cast<double>(std::max(1, std::abs(reset_velocity_rpm_))),
      0.0, static_cast<double>(max_rpm_));
    target_wire_rpm = joint_rpm / sign;
    return true;
  }

  const bool hold_requested = reset_hold_request_.load(std::memory_order_acquire);
  double target = hold_requested ?
    reset_hold_target_m_.load(std::memory_order_acquire) : hw_commands_[idx_joint_motor_];
  if (!std::isfinite(target)) {
    return false;
  }
  const double unclamped_target = target;
  target = std::clamp(target, position_min_m_, position_max_m_);
  if (std::abs(target - unclamped_target) > 1e-12) {
    hw_commands_[idx_joint_motor_] = target;
    last_unclamped_target_m_.store(unclamped_target, std::memory_order_release);
    last_clamped_target_m_.store(target, std::memory_order_release);
    target_clamp_pending_.store(true, std::memory_order_release);
  }

  const double error = target - feedback_position;
  const double abs_error = std::abs(error);
  const double commanded_velocity = !hold_requested && has_velocity_command_ &&
    std::isfinite(hw_velocity_commands_[idx_joint_motor_]) ?
    hw_velocity_commands_[idx_joint_motor_] : 0.0;
  const bool below_lower_limit = feedback_position < position_min_m_ - command_epsilon_m_;
  const bool above_upper_limit = feedback_position > position_max_m_ + command_epsilon_m_;
  const bool position_limit_violation = below_lower_limit || above_upper_limit;
  if (!std::isfinite(last_target_position_m_) ||
    std::abs(target - last_target_position_m_) > command_epsilon_m_)
  {
    last_target_position_m_ = target;
    target_change_time_ = std::chrono::steady_clock::now();
  }
  const bool terminal_target = target_change_time_.time_since_epoch().count() != 0 &&
    std::chrono::steady_clock::now() - target_change_time_ >=
    std::chrono::milliseconds(std::max(0, stop_target_stable_ms_)) &&
    std::abs(commanded_velocity) <= stationary_velocity_threshold_mps_;
  const bool within_deadband = abs_error <= deadband_m_;
  const bool trajectory_velocity_active = std::abs(commanded_velocity) >
    stationary_velocity_threshold_mps_;
  const bool motion_active = !within_deadband || trajectory_velocity_active;
  if (!motion_active) {
    target_wire_rpm = 0.0;
    last_target_wire_rpm_ = 0.0;
    return true;
  }

  const double rpm_to_mps = std::abs(unit_config_.lead_mm_per_rev) / 60000.0;
  const double feedforward_rpm = rpm_to_mps > 1e-9 ? commanded_velocity / rpm_to_mps : 0.0;
  double joint_rpm = feedforward_rpm + error * kp_rpm_per_m_ +
    (commanded_velocity - feedback_velocity) * kd_rpm_per_mps_;
  joint_rpm = std::clamp(joint_rpm, -static_cast<double>(max_rpm_), static_cast<double>(max_rpm_));

  if (terminal_target && slowdown_distance_m_ > deadband_m_ && abs_error < slowdown_distance_m_) {
    double max_joint_rpm = 0.0;
    if (brake_accel_rpm_per_s_ > 0.0 && rpm_to_mps > 1e-9) {
      const double remaining = std::max(0.0, abs_error - deadband_m_);
      const double acceleration_mps2 = brake_accel_rpm_per_s_ * rpm_to_mps;
      max_joint_rpm = std::sqrt(std::max(0.0, 2.0 * acceleration_mps2 * remaining)) / rpm_to_mps;
    } else {
      const double ramp = (abs_error - deadband_m_) /
        (slowdown_distance_m_ - deadband_m_);
      max_joint_rpm = static_cast<double>(max_rpm_) * std::clamp(ramp, 0.0, 1.0);
    }
    joint_rpm = std::clamp(joint_rpm, -max_joint_rpm, max_joint_rpm);
  }
  if (terminal_target && abs_error <= overshoot_guard_window_m_) {
    const double recovery = static_cast<double>(std::min(overshoot_recovery_max_rpm_, max_rpm_));
    joint_rpm = std::clamp(joint_rpm, -recovery, recovery);
    if (joint_rpm * error <= 0.0) {
      joint_rpm = 0.0;
    }
  }
  const bool within_stop_window = terminal_target && abs_error <= stop_window_m_;
  if (within_stop_window) {
    if (direct_stop_in_stop_window_) {
      joint_rpm = 0.0;
    } else {
      const double stop_limit = static_cast<double>(std::min(stop_window_max_rpm_, max_rpm_));
      joint_rpm = std::clamp(joint_rpm, -stop_limit, stop_limit);
      if (std::abs(feedback_velocity) >= stationary_velocity_threshold_mps_ &&
        joint_rpm * feedback_velocity < 0.0)
      {
        joint_rpm = 0.0;
      }
    }
  }

  const bool software_limit_stop =
    (feedback_position <= position_min_m_ + command_epsilon_m_ && joint_rpm < 0.0) ||
    (feedback_position >= position_max_m_ - command_epsilon_m_ && joint_rpm > 0.0);
  const uint32_t digital_inputs = digital_inputs_.load(std::memory_order_acquire);
  const auto limit_asserted = [this, digital_inputs](int bit) {
      if (bit < 0) {
        return false;
      }
      const bool high = ((digital_inputs >> static_cast<unsigned int>(bit)) & 1U) != 0U;
      return limit_switch_active_high_ ? high : !high;
    };
  const bool positive_limit = limit_switch_enabled_ &&
    limit_asserted(limit_switch_positive_bit_);
  const bool negative_limit = limit_switch_enabled_ &&
    limit_asserted(limit_switch_negative_bit_);
  const bool limit_stop = software_limit_stop ||
    (positive_limit && joint_rpm > 0.0) || (negative_limit && joint_rpm < 0.0);
  if (limit_stop)
  {
    joint_rpm = 0.0;
  }

  const double dt = std::clamp(period_s, 0.001, 0.1);
  if (std::abs(joint_rpm) <= 1e-9 && std::abs(last_target_wire_rpm_ * sign) > 1e-9 &&
    stop_slew_rpm_per_s_ > 0.0 && !limit_stop)
  {
    const double max_delta = stop_slew_rpm_per_s_ * dt;
    const double previous_joint_rpm = last_target_wire_rpm_ * sign;
    if (std::abs(previous_joint_rpm) > max_delta) {
      joint_rpm = previous_joint_rpm - std::copysign(max_delta, previous_joint_rpm);
    }
  } else if (motion_active && velocity_slew_rpm_per_s_ > 0.0) {
    const double previous_joint_rpm = last_target_wire_rpm_ * sign;
    const double max_delta = velocity_slew_rpm_per_s_ * dt;
    joint_rpm =
      std::clamp(joint_rpm, previous_joint_rpm - max_delta, previous_joint_rpm + max_delta);
  }
  joint_rpm = std::clamp(joint_rpm, -static_cast<double>(max_rpm_), static_cast<double>(max_rpm_));

  // Position-limit recovery requires an explicit velocity command toward the
  // valid range. This prevents an enable-only request or a clamped HOLD target
  // from moving a vertical axis unexpectedly.
  if (position_limit_violation) {
    const bool explicit_inward_command =
      (below_lower_limit && commanded_velocity > 1.0e-9) ||
      (above_upper_limit && commanded_velocity < -1.0e-9);
    const bool computed_inward_motion =
      (below_lower_limit && joint_rpm > 0.0) ||
      (above_upper_limit && joint_rpm < 0.0);
    if (!explicit_inward_command || !computed_inward_motion) {
      joint_rpm = 0.0;
    } else {
      const double recovery_limit = static_cast<double>(position_limit_recovery_max_rpm_);
      joint_rpm = std::clamp(joint_rpm, -recovery_limit, recovery_limit);
    }
  }
  target_wire_rpm = joint_rpm / sign;
  last_target_wire_rpm_ = target_wire_rpm;
  limit_recovery_active_.store(
    position_limit_violation && std::abs(joint_rpm) > command_epsilon_rpm_,
    std::memory_order_release);
  return true;
}

hardware_interface::return_type LiftHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return hardware_interface::return_type::ERROR;
  }
  apply_pending_zero_offset();
  const bool reset_velocity_stop_pending =
    reset_velocity_stop_pending_.load(std::memory_order_acquire);
  const auto current_link = backend_->link_state();
  link_state_atomic_.store(static_cast<uint8_t>(current_link), std::memory_order_release);
  working_counter_atomic_.store(backend_->working_counter(), std::memory_order_release);
  const bool link_operational = current_link == lift::EthercatLinkState::operational;
  const bool fresh = feedback_is_fresh();
  const auto now = std::chrono::steady_clock::now();
  if (reset_velocity_request_.load(std::memory_order_acquire) &&
    reset_velocity_start_time_.time_since_epoch().count() != 0 &&
    now - reset_velocity_start_time_ >= std::chrono::milliseconds(reset_velocity_timeout_ms_))
  {
    reset_velocity_request_.store(false, std::memory_order_release);
    reset_velocity_stop_pending_.store(true, std::memory_order_release);
    reset_hold_target_m_.store(
      feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
    reset_hold_request_.store(true, std::memory_order_release);
  }
  const bool estop = estop_latched_.load(std::memory_order_acquire);
  if (estop) {
    brake_request_enable_.store(false, std::memory_order_release);
    brake_stop_requested_.store(true, std::memory_order_release);
    reset_velocity_request_.store(false, std::memory_order_release);
  }
  const bool power_command = std::isfinite(hw_power_commands_[idx_joint_motor_]) &&
    hw_power_commands_[idx_joint_motor_] >= 0.5;
  power_command_requested_.store(power_command, std::memory_order_release);
  if (power_command != previous_power_command_) {
    if (power_command && !estop && !motion_blocked_.load(std::memory_order_acquire)) {
      // A new false-to-true edge is required after an external safety disable.
      service_disable_latched_.store(false, std::memory_order_release);
      brake_stop_requested_.store(false, std::memory_order_release);
    } else if (!power_command) {
      brake_stop_requested_.store(true, std::memory_order_release);
    }
    previous_power_command_ = power_command;
  }
  const bool enable_requested = !estop && brake_control_enabled_ &&
    !service_disable_latched_.load(std::memory_order_acquire) &&
    (power_command || brake_request_enable_.load(std::memory_order_acquire));
  const bool stop_requested = estop || brake_stop_requested_.load(std::memory_order_acquire);
  lift::Cia402Inputs inputs;
  inputs.status_word = rx_pdo_.status_word;
  inputs.error_code = rx_pdo_.error_code;
  inputs.mode_display = rx_pdo_.mode_display;
  inputs.link_operational = link_operational;
  inputs.pdo_fresh = fresh && backend_->pdo_fresh() &&
    !transport_fault_active_.load(std::memory_order_acquire);
  inputs.working_counter_ok = backend_->working_counter() >= expected_working_counter_;
  const int8_t expected_mode = homing_active_.load(std::memory_order_acquire) ? 6 : 9;
  inputs.expected_mode = expected_mode;
  const bool mode_ok = rx_pdo_.mode_display == expected_mode;
  if (fresh && rx_pdo_.error_code != 0U) {
    // A drive error is always exposed immediately and blocks the current
    // enable sequence. Before any enable request, allow the bounded CiA 402
    // auto-reset state machine to recover without leaving a permanent motion
    // latch. A fault while enable is requested or active remains latched.
    mode_mismatch_cycles_.store(0, std::memory_order_release);
    if (enable_requested || power_enabled_.load(std::memory_order_acquire)) {
      latch_motion_fault(7);
    } else if (!motion_blocked_.load(std::memory_order_acquire) && !estop) {
      error_reason_code_.store(7, std::memory_order_release);
    }
  } else if (fresh && enable_requested && !mode_ok) {
    // 6061h can briefly expose the old mode while the drive changes state.
    // Do not permanently lock out the lift on one transient PDO, but fail
    // safe when the mismatch persists while enable was explicitly requested.
    const uint32_t previous = mode_mismatch_cycles_.load(std::memory_order_acquire);
    const uint32_t next = std::min<uint32_t>(
      previous + 1U, static_cast<uint32_t>(mode_mismatch_debounce_cycles_));
    mode_mismatch_cycles_.store(next, std::memory_order_release);
    if (next >= static_cast<uint32_t>(mode_mismatch_debounce_cycles_)) {
      latch_motion_fault(7);
    } else if (!motion_blocked_.load(std::memory_order_acquire) && !estop) {
      error_reason_code_.store(7, std::memory_order_release);
    }
  } else if (!inputs.link_operational || !inputs.pdo_fresh || !inputs.working_counter_ok) {
    mode_mismatch_cycles_.store(0, std::memory_order_release);
    if (!motion_blocked_.load(std::memory_order_acquire)) {
      error_reason_code_.store(1, std::memory_order_release);
    }
  } else if (!motion_blocked_.load(std::memory_order_acquire) && !estop) {
    mode_mismatch_cycles_.store(0, std::memory_order_release);
    error_reason_code_.store(
      position_limit_violation_.load(std::memory_order_acquire) ? 3 : 0,
      std::memory_order_release);
  }

  // The service only flips atomics.  This small state machine is the sole
  // owner of the PDO control word, so the next 10 ms cycle sends a deterministic
  // Quick Stop without racing the service callback.
  if (stop_requested && brake_stop_phase_ == BrakeStopPhase::idle) {
    brake_stop_phase_ = BrakeStopPhase::quick_stop;
    brake_stop_phase_atomic_.store(
      static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
    brake_stop_start_time_ = now;
    brake_stop_timeout_.store(false, std::memory_order_release);
  } else if (!stop_requested && brake_stop_phase_ == BrakeStopPhase::quick_stop) {
    // An explicit unlock request exits Quick Stop on the next cycle.  The
    // normal CiA 402 sequence still requires the drive to report its state
    // before a non-zero velocity is allowed.
    brake_stop_phase_ = BrakeStopPhase::idle;
    brake_stop_phase_atomic_.store(
      static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
    brake_stop_start_time_ = std::chrono::steady_clock::time_point{};
  } else if (!stop_requested && brake_stop_phase_ == BrakeStopPhase::disabled &&
    enable_requested)
  {
    brake_stop_phase_ = BrakeStopPhase::idle;
    brake_stop_phase_atomic_.store(
      static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
  }
  const bool quick_stop_requested = brake_stop_phase_ == BrakeStopPhase::quick_stop;

  // A disabled brake-control interface must be fail-safe: it may not request
  // Operation enabled implicitly.  This keeps the default launch from
  // releasing a vertical-axis brake before the operator explicitly opts in.
  inputs.request_enable = enable_requested && !quick_stop_requested;
  inputs.request_quick_stop = quick_stop_requested;
  inputs.request_fault_reset = auto_fault_reset_ && !estop;
  auto decision = cia402_.step(inputs);
  fault_reset_attempts_.store(cia402_.fault_reset_attempts(), std::memory_order_release);
  recovery_exhausted_.store(decision.recovery_exhausted, std::memory_order_release);

  if (homing_active_.load(std::memory_order_acquire) && decision.allow_motion) {
    // HM starts on a rising edge of 6040h bit 4. Keeping the bit set is
    // intentional: clearing it pauses the LD3M homing trajectory.
    decision.control_word = static_cast<uint16_t>(decision.control_word | 0x0010U);
    homing_start_requested_.store(false, std::memory_order_release);
  }
  if (quick_stop_requested) {
    // P04.39 is the drive's configured speed threshold for brake triggering.
    // Use the raw 606Ch sample here instead of the filtered velocity so a
    // freshly reported stop is not held back by the outer-loop filter.
    const double raw_velocity_mps = std::abs(lift::velocity_units_to_mps(
        rx_pdo_.actual_velocity_units_per_s, unit_config_));
    const double brake_threshold_mps = std::abs(lift::rpm_to_mps(
        static_cast<double>(brake_p04_39_rpm_), unit_config_));
    const double stop_threshold_mps = std::max(
      stationary_velocity_threshold_mps_, brake_threshold_mps);
    const bool low_speed = raw_velocity_mps <= stop_threshold_mps;
    const bool stop_timed_out = brake_stop_start_time_.time_since_epoch().count() != 0 &&
      now - brake_stop_start_time_ >= std::chrono::milliseconds(std::max(0, brake_p06_14_ms_));
    if (low_speed || stop_timed_out || decision.control_word == 0x0006) {
      if (stop_timed_out && !low_speed) {
        brake_stop_timeout_.store(true, std::memory_order_release);
      }
      brake_stop_phase_ = BrakeStopPhase::disabled;
      brake_stop_phase_atomic_.store(
        static_cast<uint8_t>(brake_stop_phase_), std::memory_order_release);
      brake_stop_start_time_ = std::chrono::steady_clock::time_point{};
      decision.control_word = 0x0006;
    }
  }

  tx_pdo_.control_word = decision.control_word;
  tx_pdo_.torque_feedforward = 0;

  if (decision.allow_motion && !previous_operation_enabled_) {
    brake_enable_time_ = now;
  }
  previous_operation_enabled_ = decision.allow_motion;
  bool release_wait_elapsed = brake_release_wait_ms_ >= 0;
  if (release_wait_elapsed) {
    release_wait_elapsed = now - brake_enable_time_ >=
      std::chrono::milliseconds(brake_release_wait_ms_);
  }
  const bool inferred_unlocked = decision.allow_motion && release_wait_elapsed &&
    brake_stop_phase_ == BrakeStopPhase::idle &&
    !motion_blocked_.load(std::memory_order_acquire);
  brake_unlocked_.store(inferred_unlocked, std::memory_order_release);

  double wire_rpm = 0.0;
  const double period_s = period.seconds() > 0.0 ? period.seconds() : 0.01;
  const bool startup_guard_active = startup_motion_guard_enabled_ && now < startup_guard_until_;
  const bool first_write_sync = first_write_sync_pending_.exchange(
    false, std::memory_order_acq_rel);
  if (startup_guard_active || first_write_sync || estop) {
    hw_commands_[idx_joint_motor_] = feedback_position_m_.load(std::memory_order_acquire);
    hw_velocity_commands_[idx_joint_motor_] = 0.0;
    hw_acceleration_commands_[idx_joint_motor_] = 0.0;
  }
  if (inferred_unlocked && decision.allow_motion &&
    !homing_active_.load(std::memory_order_acquire))
  {
    (void)compute_motion_command(wire_rpm, period_s);
  }
  if (!decision.allow_motion || !inferred_unlocked || !fresh ||
    motion_blocked_.load(std::memory_order_acquire) || estop || startup_guard_active ||
    first_write_sync)
  {
    wire_rpm = 0.0;
    last_target_wire_rpm_ = 0.0;
  }
  tx_pdo_.target_velocity_units_per_s = lift::wire_rpm_to_velocity_units(wire_rpm, unit_config_);
  command_position_atomic_.store(hw_commands_[idx_joint_motor_], std::memory_order_release);
  command_velocity_atomic_.store(
    hw_velocity_commands_[idx_joint_motor_], std::memory_order_release);
  command_acceleration_atomic_.store(
    hw_acceleration_commands_[idx_joint_motor_], std::memory_order_release);
  if (reset_velocity_stop_pending) {
    // Stop reset-velocity takeover on this transmission. The captured hold
    // target is allowed to take effect from the following cycle, but the old
    // takeover speed is never sent again after the service returns.
    tx_pdo_.target_velocity_units_per_s = 0;
    tx_pdo_.torque_feedforward = 0;
    wire_rpm = 0.0;
    last_target_wire_rpm_ = 0.0;
  }
  target_wire_rpm_atomic_.store(wire_rpm, std::memory_order_release);
  target_velocity_units_atomic_.store(
    tx_pdo_.target_velocity_units_per_s, std::memory_order_release);

  if (zero_request_pending_.load(std::memory_order_acquire) &&
    !decision.allow_motion && decision.control_word == 0x0006)
  {
    // The service thread may persist the offset only after this cycle has
    // observed the requested controlled disable.  No file I/O occurs here.
    zero_disable_observed_.store(true, std::memory_order_release);
  }
  bool write_ok = false;
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    write_ok = backend_->write_pdo(tx_pdo_);
  }
  if (!write_ok) {
    handle_transport_fault("PDO write failed");
    return hardware_interface::return_type::ERROR;
  }
  if (reset_velocity_stop_pending) {
    reset_velocity_stop_pending_.store(false, std::memory_order_release);
  }
  update_state_interfaces();
  return hardware_interface::return_type::OK;
}

bool LiftHardware::complete_pending_zero()
{
  if (!active_.load(std::memory_order_acquire) ||
    !zero_request_pending_.load(std::memory_order_acquire) ||
    !zero_disable_observed_.load(std::memory_order_acquire) ||
    brake_unlocked_.load(std::memory_order_acquire) || !feedback_is_fresh() ||
    std::abs(feedback_velocity_mps_.load(std::memory_order_acquire)) >
    stationary_velocity_threshold_mps_)
  {
    return false;
  }

  if (!use_persistent_zero_offset_) {
    zero_offset_units_ = 0;
    zero_offset_units_atomic_.store(0, std::memory_order_release);
    zero_offset_loaded_ = false;
    zero_request_pending_.store(false, std::memory_order_release);
    zero_disable_observed_.store(false, std::memory_order_release);
    error_reason_code_.store(0, std::memory_order_release);
    RCLCPP_INFO(logger_, "Host zero request completed without a persistent offset");
    return true;
  }

  lift::ZeroOffsetRecord record;
  record.motor_id = motor_id_;
  record.slave_alias = slave_config_.alias;
  record.slave_position = slave_config_.position;
  record.zero_offset_units = feedback_position_units_.load(std::memory_order_acquire);
  std::string error;
  if (!lift::ZeroOffsetStore::save_atomic(zero_offset_file_, record, error)) {
    error_reason_code_.store(4, std::memory_order_release);
    motion_blocked_.store(true, std::memory_order_release);
    latched_motion_reason_code_.store(4, std::memory_order_release);
    zero_request_pending_.store(false, std::memory_order_release);
    zero_disable_observed_.store(false, std::memory_order_release);
    return false;
  }
  zero_offset_units_atomic_.store(record.zero_offset_units, std::memory_order_release);
  zero_offset_apply_pending_.store(true, std::memory_order_release);
  zero_request_pending_.store(false, std::memory_order_release);
  zero_disable_observed_.store(false, std::memory_order_release);
  error_reason_code_.store(0, std::memory_order_release);
  RCLCPP_INFO(
    logger_,
    "Lift zero offset persisted from current 6064h; the real-time cycle will apply it without a motor absolute-position write");
  return true;
}

void LiftHardware::apply_pending_zero_offset()
{
  if (!zero_offset_apply_pending_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  zero_offset_units_ = zero_offset_units_atomic_.load(std::memory_order_acquire);
  zero_offset_loaded_ = true;
  const double position = lift::position_units_to_m(
    feedback_position_units_.load(std::memory_order_acquire), zero_offset_units_, unit_config_);
  if (idx_joint_motor_ >= 0) {
    hw_positions_[idx_joint_motor_] = position;
    hw_commands_[idx_joint_motor_] = 0.0;
    hw_velocity_commands_[idx_joint_motor_] = 0.0;
    hw_acceleration_commands_[idx_joint_motor_] = 0.0;
    hw_velocities_[idx_joint_motor_] = 0.0;
  }
  feedback_position_m_.store(position, std::memory_order_release);
  feedback_velocity_mps_.store(0.0, std::memory_order_release);
  reset_feedback_continuity();
  observe_feedback_continuity(
    position,
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count(),
    true);
  filtered_velocity_mps_ = 0.0;
  last_target_wire_rpm_ = 0.0;
  last_target_position_m_ = position;
}

bool LiftHardware::target_is_at_limit(double position, double target) const noexcept
{
  return (position <= position_min_m_ && target < position) ||
         (position >= position_max_m_ && target > position);
}

void LiftHardware::update_state_interfaces()
{
  if (idx_joint_motor_ < 0) {
    return;
  }
  hw_status_[idx_joint_motor_] = static_cast<double>(rx_pdo_.status_word);
  hw_error_codes_[idx_joint_motor_] = static_cast<double>(rx_pdo_.error_code);
  hw_modes_[idx_joint_motor_] = static_cast<double>(rx_pdo_.mode_display);
  hw_brake_unlocked_[idx_joint_motor_] =
    brake_unlocked_.load(std::memory_order_acquire) ? 1.0 : 0.0;
  hw_digital_inputs_[idx_joint_motor_] = static_cast<double>(rx_pdo_.digital_inputs);
  const bool actual_power_enabled = feedback_is_fresh() &&
    link_state_from_code(link_state_atomic_.load(std::memory_order_acquire)) ==
    lift::EthercatLinkState::operational &&
    working_counter_atomic_.load(std::memory_order_acquire) >= expected_working_counter_ &&
    rx_pdo_.error_code == 0U &&
    rx_pdo_.mode_display == (homing_active_.load(std::memory_order_acquire) ? 6 : 9) &&
    lift::parse_cia402_status(rx_pdo_.status_word) == lift::Cia402State::operation_enabled;
  hw_power_enabled_[idx_joint_motor_] = actual_power_enabled ? 1.0 : 0.0;
  power_enabled_.store(actual_power_enabled, std::memory_order_release);
}

void LiftHardware::start_service_thread()
{
  if (service_node_) {
    return;
  }
  std::ostringstream node_name;
  node_name << "joint_lift_services_" << std::hex << reinterpret_cast<std::uintptr_t>(this);
  service_node_ = std::make_shared<rclcpp::Node>(node_name.str());
  status_publisher_ = service_node_->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  brake_service_ = service_node_->create_service<std_srvs::srv::SetBool>(
    "/lift_brake_command",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      handle_brake_command(request, response);
    });
  reset_velocity_service_ = service_node_->create_service<std_srvs::srv::SetBool>(
    "/lift_reset_velocity",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      handle_reset_velocity(request, response);
    });
  reset_hold_service_ = service_node_->create_service<std_srvs::srv::SetBool>(
    "/lift_reset_hold",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      handle_reset_hold(request, response);
    });
  reset_zero_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/lift_reset_zero",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_reset_zero(request, response);
    });
  home_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/lift_home",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_home(request, response);
    });
  drive_zero_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/lift_set_drive_zero",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_set_drive_zero(request, response);
    });
  drive_zero_alias_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/lift_set_zero_position",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_reset_zero(request, response);
    });
  estop_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/joint/safety/estop",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_estop(request, response);
    });
  safety_reset_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/joint/safety/reset",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_safety_reset(request, response);
    });
  status_timer_ = service_node_->create_wall_timer(
    std::chrono::milliseconds(100), [this]() {publish_driver_status();});
  service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(service_node_);
  service_thread_ = std::thread(
    [this]() {
      if (service_executor_) {
        service_executor_->spin();
      }
    });
}

void LiftHardware::stop_service_thread()
{
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_thread_.joinable()) {
    service_thread_.join();
  }
  if (service_executor_ && service_node_) {
    try {
      service_executor_->remove_node(service_node_);
    } catch (...) {
    }
  }
  status_timer_.reset();
  brake_service_.reset();
  reset_velocity_service_.reset();
  reset_hold_service_.reset();
  reset_zero_service_.reset();
  home_service_.reset();
  drive_zero_service_.reset();
  drive_zero_alias_service_.reset();
  estop_service_.reset();
  safety_reset_service_.reset();
  status_publisher_.reset();
  service_executor_.reset();
  service_node_.reset();
}

void LiftHardware::publish_driver_status()
{
  if (!status_publisher_) {
    return;
  }
  // Persistence is deliberately handled by this non-real-time service thread.
  // The 100 Hz read/write path only sets zero_disable_observed_ and applies the
  // completed offset on its next cycle.
  (void)complete_pending_zero();
  const auto now = std::chrono::steady_clock::now();
  if (target_clamp_pending_.load(std::memory_order_acquire) &&
    (last_clamp_log_time_.time_since_epoch().count() == 0 ||
    now - last_clamp_log_time_ >= std::chrono::milliseconds(500)))
  {
    if (target_clamp_pending_.exchange(false, std::memory_order_acq_rel)) {
      RCLCPP_WARN(
        logger_, "Lift target clamped from %.6f m to %.6f m within [%.6f, %.6f]",
        last_unclamped_target_m_.load(std::memory_order_acquire),
        last_clamped_target_m_.load(std::memory_order_acquire), position_min_m_, position_max_m_);
      last_clamp_log_time_ = now;
    }
  }
  const auto state = lift::parse_cia402_status(status_word_.load(std::memory_order_acquire));
  const int64_t feedback_stamp = last_feedback_ns_.load(std::memory_order_acquire);
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
  const double feedback_age_ms = feedback_stamp > 0 && now_ns >= feedback_stamp ?
    static_cast<double>(now_ns - feedback_stamp) * 1.0e-6 : -1.0;
  const bool transport_recovering = transport_fault_active_.load(std::memory_order_acquire);
  const bool feedback_fresh = feedback_is_fresh() && !transport_recovering;
  const bool ethercat_operational =
    link_state_from_code(link_state_atomic_.load()) == lift::EthercatLinkState::operational;
  const bool working_counter_ok = working_counter_atomic_.load() >= expected_working_counter_;
  const bool quick_stop_active =
    brake_stop_phase_atomic_.load() == static_cast<uint8_t>(BrakeStopPhase::quick_stop) ||
    state == lift::Cia402State::quick_stop_active;
  std::ostringstream status;
  const bool effective_enable_command =
    power_command_requested_.load() || brake_request_enable_.load();
  status << "{\"component\":\"lift\",\"link_state\":\""
         << link_state_string(link_state_from_code(link_state_atomic_.load()))
         << "\",\"cia402_state\":\""
         << cia_state_string(state) << "\",\"status_word\":" << status_word_.load()
         << ",\"error_code\":" << error_code_.load()
         << ",\"mode\":" << static_cast<int>(mode_display_.load())
         << ",\"mode_display\":" << static_cast<int>(mode_display_.load())
         << ",\"feedback_age_ms\":" << feedback_age_ms
         << ",\"feedback_fresh\":" << (feedback_fresh ? "true" : "false")
         << ",\"pdo_fresh\":" << (feedback_fresh ? "true" : "false")
         << ",\"ethercat_operational\":" <<
    (ethercat_operational ? "true" : "false")
         << ",\"working_counter\":" << working_counter_atomic_.load()
         << ",\"working_counter_ok\":" << (working_counter_ok ? "true" : "false")
         << ",\"transport_recovering\":" << (transport_recovering ? "true" : "false")
         << ",\"transport_recovery_healthy_cycles\":" <<
    transport_recovery_healthy_cycles_.load()
         << ",\"initialized\":" << (initialized_.load() ? "true" : "false")
         << ",\"power_enable_command\":" <<
    (effective_enable_command ? "true" : "false")
         << ",\"power_enabled\":" << (power_enabled_.load() ? "true" : "false")
         << ",\"servo_online\":" <<
    (ethercat_operational && feedback_fresh ? "true" : "false")
         << ",\"brake_unlocked\":" << (brake_unlocked_.load() ? "true" : "false")
         << ",\"brake_unlocked_inferred\":" <<
    (brake_unlocked_.load() ? "true" : "false")
         << ",\"estop_latched\":" << (estop_latched_.load() ? "true" : "false")
         << ",\"motion_blocked\":" << (motion_blocked_.load() ? "true" : "false")
         << ",\"position_limit_violation\":" <<
    (position_limit_violation_.load() ? "true" : "false")
         << ",\"limit_recovery_active\":" <<
    (limit_recovery_active_.load() ? "true" : "false")
         << ",\"position_limit_recovery_max_rpm\":" << position_limit_recovery_max_rpm_
         << ",\"quick_stop_active\":" << (quick_stop_active ? "true" : "false")
         << ",\"position\":" << feedback_position_m_.load()
         << ",\"velocity\":" << hw_velocities_[idx_joint_motor_]
         << ",\"command_position\":" << command_position_atomic_.load()
         << ",\"command_velocity\":" << command_velocity_atomic_.load()
         << ",\"command_acceleration\":" << command_acceleration_atomic_.load()
         << ",\"filtered_velocity\":" << feedback_velocity_mps_.load()
         << ",\"target_rpm\":" << target_wire_rpm_atomic_.load()
         << ",\"target_velocity_units\":" << target_velocity_units_atomic_.load()
         << ",\"velocity_mode_active\":" <<
    (std::abs(target_wire_rpm_atomic_.load()) > command_epsilon_rpm_ ? "true" : "false")
         << ",\"reset_velocity_active\":" <<
    (reset_velocity_request_.load() ? "true" : "false")
         << ",\"reset_search_start_position\":" <<
    reset_search_start_position_m_.load()
         << ",\"reset_max_search_travel\":" << reset_max_search_travel_m_
         << ",\"reset_hold_active\":" <<
    (reset_hold_request_.load() ? "true" : "false")
         << ",\"homing_active\":" <<
    (homing_active_.load() ? "true" : "false")
         << ",\"homing_complete\":" <<
    (homing_complete_.load() ? "true" : "false")
         << ",\"homing_failed\":" <<
    (homing_failed_.load() ? "true" : "false")
         << ",\"homing_method\":" << homing_method_
         << ",\"brake_stop_phase\":\""
         << brake_stop_phase_string(brake_stop_phase_atomic_.load()) << "\""
         << ",\"brake_stop_timeout\":"
         << (brake_stop_timeout_.load() ? "true" : "false")
         << ",\"brake_feedback\":\"inferred_from_cia402_and_timing\""
         << ",\"limit_switch_enabled\":" << (limit_switch_enabled_ ? "true" : "false")
         << ",\"actual_position_units\":" << feedback_position_units_.load()
         << ",\"zero_offset_units\":" << zero_offset_units_atomic_.load()
         << ",\"zero_offset_valid\":" << (zero_offset_loaded_ ? "true" : "false")
         << ",\"effective_command_units_per_rev\":" <<
    lift::effective_command_units_per_rev(unit_config_)
         << ",\"effective_lead_mm_per_rev\":" << unit_config_.lead_mm_per_rev
         << ",\"position_m_per_unit\":" <<
    unit_config_.lead_mm_per_rev /
    (1000.0 * lift::effective_command_units_per_rev(unit_config_))
         << ",\"feedback_baseline_valid\":" <<
    (feedback_baseline_valid_.load() ? "true" : "false")
         << ",\"feedback_epoch\":" << feedback_epoch_.load()
         << ",\"feedback_source\":\"" << backend_name_ << "\""
         << ",\"max_feedback_jump_m\":" << max_feedback_jump_m_
         << ",\"fault_reset_attempts\":" << fault_reset_attempts_.load()
         << ",\"mode_mismatch_cycles\":" << mode_mismatch_cycles_.load()
         << ",\"mode_mismatch_debounce_cycles\":" << mode_mismatch_debounce_cycles_
         << ",\"recovery_exhausted\":" << (recovery_exhausted_.load() ? "true" : "false")
         << ",\"fault_reason\":\"" << detailed_error_reason(
    motion_blocked_.load() ? latched_motion_reason_code_.load() : error_reason_code_.load()) << "\""
         << ",\"last_error\":\"" << detailed_error_reason(error_reason_code_.load()) << "\"}";
  std_msgs::msg::String message;
  message.data = status.str();
  status_publisher_->publish(message);
}

void LiftHardware::handle_brake_command(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // Lock is always accepted as a fail-safe action.  Only an explicit unlock
  // requires the operator to authorize brake control in the launch parameters.
  if (request->data && !brake_control_enabled_) {
    response->success = false;
    response->message = "brake_control_enabled is false; unlock refused";
    return;
  }
  if (request->data && estop_latched_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "brake release refused while emergency stop is latched";
    return;
  }
  if (request->data && motion_blocked_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message =
      "brake release refused while a latched lift motion fault is active";
    return;
  }
  brake_request_enable_.store(request->data, std::memory_order_release);
  if (!request->data) {
    service_disable_latched_.store(true, std::memory_order_release);
    reset_velocity_request_.store(false, std::memory_order_release);
    reset_velocity_stop_pending_.store(false, std::memory_order_release);
    brake_stop_requested_.store(true, std::memory_order_release);
    brake_unlocked_.store(false, std::memory_order_release);
    brake_stop_timeout_.store(false, std::memory_order_release);
    response->success = true;
    response->message =
      "controlled stop and brake lock requested; Quick Stop is sent on the next 10 ms cycle and LD3M-EC drives BR+/BR- automatically";
  } else {
    service_disable_latched_.store(false, std::memory_order_release);
    brake_stop_requested_.store(false, std::memory_order_release);
    brake_stop_timeout_.store(false, std::memory_order_release);
    response->success = true;
    response->message =
      "Operation enabled requested; brake unlock and P04.38 release timing are checked before motion";
  }
}

void LiftHardware::handle_reset_velocity(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (estop_latched_.load(std::memory_order_acquire)) {
      response->success = false;
      response->message = "reset velocity refused while emergency stop is latched";
      return;
    }
    if (!reset_velocity_debug_enabled_) {
      response->success = false;
      response->message = "reset velocity debug takeover is disabled by configuration";
      return;
    }
    if (!brake_control_enabled_) {
      response->success = false;
      response->message =
        "reset velocity refused because brake_control_enabled is false";
      return;
    }
    if (!feedback_is_fresh()) {
      response->success = false;
      response->message = "reset velocity refused because PDO feedback is stale";
      return;
    }
    if (position_limit_violation_.load(std::memory_order_acquire)) {
      response->success = false;
      response->message =
        "reset velocity refused outside software limits; use normal controlled motion toward the valid range";
      return;
    }
    if (motion_blocked_.load(std::memory_order_acquire)) {
      response->success = false;
      response->message = "reset velocity refused while a latched motion fault is active";
      return;
    }
    brake_request_enable_.store(true, std::memory_order_release);
    const bool was_active =
      reset_velocity_request_.exchange(true, std::memory_order_acq_rel);
    if (!was_active) {
      reset_search_start_position_m_.store(
        feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
    }
    reset_velocity_stop_pending_.store(false, std::memory_order_release);
    reset_hold_request_.store(false, std::memory_order_release);
    reset_velocity_start_time_ = std::chrono::steady_clock::now();
    response->success = true;
    response->message = was_active ?
      "reset velocity watchdog refreshed" :
      "bounded reset velocity takeover requested through CSV/PDO";
  } else {
    reset_velocity_request_.store(false, std::memory_order_release);
    reset_velocity_stop_pending_.store(true, std::memory_order_release);
    reset_hold_target_m_.store(
      feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
    reset_hold_request_.store(true, std::memory_order_release);
    response->success = true;
    response->message = "reset velocity takeover stopped; controlled zero-speed hold requested";
  }
}

void LiftHardware::handle_reset_hold(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (estop_latched_.load(std::memory_order_acquire)) {
      response->success = false;
      response->message = "reset hold refused while emergency stop is latched";
      return;
    }
    if (!feedback_is_fresh()) {
      response->success = false;
      response->message = "reset hold refused because PDO feedback is stale";
      return;
    }
    reset_hold_target_m_.store(
      feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
    reset_hold_request_.store(true, std::memory_order_release);
    response->success = true;
    response->message =
      "reset hold captured the current host position and uses CSV/PDO velocity control";
  } else {
    reset_hold_request_.store(false, std::memory_order_release);
    response->success = true;
    response->message = "reset hold released";
  }
}

void LiftHardware::handle_reset_zero(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (estop_latched_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "zero refused while emergency stop is latched";
    return;
  }
  if (!brake_control_enabled_) {
    response->success = false;
    response->message = "zero refused because brake_control_enabled is false";
    return;
  }
  if (!feedback_is_fresh() ||
    std::abs(feedback_velocity_mps_.load(std::memory_order_acquire)) >
    stationary_velocity_threshold_mps_)
  {
    response->success = false;
    response->message = "zero refused until PDO feedback is fresh and velocity is near zero";
    return;
  }
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(true, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  zero_request_pending_.store(true, std::memory_order_release);
  response->success = true;
  response->message =
    "zero request accepted; drive will stop, disable and persist the current 6064h as a host zero offset (no HM or P00.15 write)";
}

bool LiftHardware::wait_for_drive_stop(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (active_.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    if (!power_enabled_.load(std::memory_order_acquire) &&
      feedback_is_fresh() &&
      std::abs(feedback_velocity_mps_.load(std::memory_order_acquire)) <=
      stationary_velocity_threshold_mps_ &&
      brake_stop_phase_atomic_.load(std::memory_order_acquire) ==
      static_cast<uint8_t>(BrakeStopPhase::disabled))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool LiftHardware::write_homing_sdo(
  uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value)
{
  // Do not hold backend_mutex_ here. EtherLab's synchronous SDO API waits for
  // the master cycle to process the transfer, while read()/write() need that
  // mutex to keep the PDO cycle alive. Holding both causes a deadlock.
  std::lock_guard<std::mutex> lock(sdo_mutex_);
  if (backend_->write_sdo(index, subindex, value)) {
    return true;
  }
  RCLCPP_ERROR(
    logger_, "LD3M SDO write 0x%04X:%02X failed: %s", index, subindex,
    backend_->error_message().c_str());
  return false;
}

bool LiftHardware::wait_for_homing_result(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (active_.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    if (homing_failed_.load(std::memory_order_acquire)) {
      return false;
    }
    if (homing_complete_.load(std::memory_order_acquire)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool LiftHardware::wait_for_host_zero(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (active_.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    // The service callback occupies the single-threaded service executor, so
    // the status timer cannot complete this maintenance transaction for us.
    (void)complete_pending_zero();
    if (!zero_request_pending_.load(std::memory_order_acquire)) {
      return error_reason_code_.load(std::memory_order_acquire) == 0U;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

void LiftHardware::handle_home(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (estop_latched_.load(std::memory_order_acquire) ||
    motion_blocked_.load(std::memory_order_acquire))
  {
    response->success = false;
    response->message = "homing refused while the lift safety state is latched";
    return;
  }
  if (!brake_control_enabled_) {
    response->success = false;
    response->message = "homing refused because brake_control_enabled is false";
    return;
  }
  if (homing_active_.load(std::memory_order_acquire) ||
    drive_zero_pending_.load(std::memory_order_acquire))
  {
    response->success = false;
    response->message = "another drive maintenance operation is already active";
    return;
  }
  if (!feedback_is_fresh() ||
    std::abs(feedback_velocity_mps_.load(std::memory_order_acquire)) >
    stationary_velocity_threshold_mps_)
  {
    response->success = false;
    response->message = "homing requires fresh, stationary PDO feedback";
    return;
  }

  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  if (!wait_for_drive_stop(std::chrono::milliseconds(drive_zero_timeout_ms_))) {
    response->success = false;
    response->message = "homing refused because the controlled drive stop timed out";
    return;
  }

  if (!write_homing_sdo(0x6098, 0, {static_cast<uint8_t>(homing_method_ & 0xff)}) ||
    !write_homing_sdo(0x6099, 1, encode_u32_le(static_cast<uint32_t>(homing_speed_high_units_s_))) ||
    !write_homing_sdo(0x6099, 2, encode_u32_le(static_cast<uint32_t>(homing_speed_low_units_s_))) ||
    !write_homing_sdo(
      0x609a, 0, encode_u32_le(static_cast<uint32_t>(homing_acceleration_units_s2_))) ||
    !write_homing_sdo(0x607c, 0, encode_i32_le(static_cast<int32_t>(homing_offset_units_))) ||
    !write_homing_sdo(0x6060, 0, {6}))
  {
    response->success = false;
    response->message = "LD3M HM parameters or 6060h=6 could not be written";
    return;
  }

  homing_complete_.store(false, std::memory_order_release);
  homing_failed_.store(false, std::memory_order_release);
  homing_active_.store(true, std::memory_order_release);
  homing_start_requested_.store(true, std::memory_order_release);
  service_disable_latched_.store(false, std::memory_order_release);
  brake_stop_requested_.store(false, std::memory_order_release);
  brake_request_enable_.store(true, std::memory_order_release);

  const bool homed = wait_for_homing_result(std::chrono::milliseconds(homing_timeout_ms_));
  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  if (!wait_for_drive_stop(std::chrono::milliseconds(drive_zero_timeout_ms_))) {
    homing_active_.store(false, std::memory_order_release);
    response->success = false;
    response->message = "homing stopped, but the controlled disable timed out";
    return;
  }

  const bool csv_restored = write_homing_sdo(0x6060, 0, {9});
  homing_active_.store(false, std::memory_order_release);
  homing_start_requested_.store(false, std::memory_order_release);
  homing_failed_.store(!homed || !csv_restored, std::memory_order_release);
  if (!homed || !csv_restored) {
    response->success = false;
    response->message = homing_failed_.load() ?
      "LD3M homing failed or timed out; the drive remains disabled" :
      "LD3M homing completed but CSV mode could not be restored";
    return;
  }

  // 607Ch defines the drive-side home offset. Persist the resulting 6064h as
  // the host frame too, so CSV feedback and the ROS joint coordinate agree.
  zero_request_pending_.store(true, std::memory_order_release);
  zero_disable_observed_.store(false, std::memory_order_release);
  if (!wait_for_host_zero(std::chrono::milliseconds(drive_zero_timeout_ms_))) {
    response->success = false;
    response->message = "homing completed, but the host zero offset could not be saved";
    return;
  }
  response->success = true;
  response->message = "LD3M HM homing completed, CSV mode restored and zero saved";
}

void LiftHardware::handle_set_drive_zero(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (estop_latched_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "drive zero refused while emergency stop is latched";
    return;
  }
  if (drive_zero_pending_.exchange(true, std::memory_order_acq_rel) ||
    homing_active_.load(std::memory_order_acquire))
  {
    drive_zero_pending_.store(true, std::memory_order_release);
    response->success = false;
    response->message = "another drive maintenance operation is already active";
    return;
  }
  if (!feedback_is_fresh() ||
    std::abs(feedback_velocity_mps_.load(std::memory_order_acquire)) >
    stationary_velocity_threshold_mps_)
  {
    drive_zero_pending_.store(false, std::memory_order_release);
    response->success = false;
    response->message = "drive zero requires fresh, stationary PDO feedback";
    return;
  }

  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  if (!wait_for_drive_stop(std::chrono::milliseconds(drive_zero_timeout_ms_))) {
    drive_zero_pending_.store(false, std::memory_order_release);
    response->success = false;
    response->message = "drive zero refused because the controlled drive stop timed out";
    return;
  }
  if (!write_homing_sdo(0x2015, 0, {9, 0})) {
    drive_zero_pending_.store(false, std::memory_order_release);
    response->success = false;
    response->message = "LD3M P00.15=9 (2015h) was refused by the drive";
    return;
  }
  drive_zero_pending_.store(false, std::memory_order_release);
  // P00.15 is an absolute-encoder maintenance setting. The LD3M manual marks
  // it effective only after a power restart, so 6064h during this process is
  // not a valid completion signal. Keep the axis disabled and require a fresh
  // post-restart verification before treating the drive coordinate as known.
  response->success = true;
  response->message =
    "LD3M P00.15=9 accepted while disabled; power-cycle/restart the drive and verify its "
    "absolute position before motion. This does not set the ROS coordinate zero.";
}

void LiftHardware::handle_estop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  estop_latched_.store(true, std::memory_order_release);
  error_reason_code_.store(8, std::memory_order_release);
  latched_motion_reason_code_.store(8, std::memory_order_release);
  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  brake_unlocked_.store(false, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(true, std::memory_order_release);
  reset_hold_request_.store(false, std::memory_order_release);
  zero_request_pending_.store(false, std::memory_order_release);
  reset_hold_target_m_.store(
    feedback_position_m_.load(std::memory_order_acquire), std::memory_order_release);
  response->success = true;
  response->message =
    "lift emergency stop latched; next 100 Hz cycle forces zero 60FFh and CiA 402 Quick Stop";
}

void LiftHardware::handle_safety_reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!estop_latched_.load(std::memory_order_acquire) &&
    !motion_blocked_.load(std::memory_order_acquire))
  {
    response->success = true;
    response->message = "lift safety state is not latched";
    return;
  }
  const bool operational =
    link_state_from_code(link_state_atomic_.load()) == lift::EthercatLinkState::operational;
  const bool command_cleared = idx_joint_motor_ >= 0 &&
    std::abs(command_position_atomic_.load() - feedback_position_m_.load()) <=
    command_epsilon_m_ &&
    std::abs(command_velocity_atomic_.load()) <= stationary_velocity_threshold_mps_ &&
    std::abs(command_acceleration_atomic_.load()) <= 1.0e-6;
  if (!active_.load() || !operational || !feedback_is_fresh() ||
    transport_fault_active_.load(std::memory_order_acquire) ||
    error_code_.load() != 0U || mode_display_.load() != 9 ||
    std::abs(feedback_velocity_mps_.load()) > stationary_velocity_threshold_mps_ ||
    !command_cleared)
  {
    response->success = false;
    response->message =
      "safety reset refused: require Operational/fresh PDO, CSV mode, no fault, stationary feedback and cleared upper command";
    return;
  }
  const double position = feedback_position_m_.load(std::memory_order_acquire);
  reset_hold_target_m_.store(position, std::memory_order_release);
  reset_hold_request_.store(true, std::memory_order_release);
  reset_velocity_request_.store(false, std::memory_order_release);
  reset_velocity_stop_pending_.store(false, std::memory_order_release);
  brake_request_enable_.store(false, std::memory_order_release);
  brake_stop_requested_.store(true, std::memory_order_release);
  motion_blocked_.store(false, std::memory_order_release);
  latched_motion_reason_code_.store(0, std::memory_order_release);
  position_limit_violation_.store(false, std::memory_order_release);
  limit_recovery_active_.store(false, std::memory_order_release);
  mode_mismatch_cycles_.store(0, std::memory_order_release);
  cia402_.reset();
  estop_latched_.store(false, std::memory_order_release);
  error_reason_code_.store(0, std::memory_order_release);
  feedback_continuity_reset_pending_.store(true, std::memory_order_release);
  response->success = true;
  response->message = "safety reset accepted; HOLD current position, old motion is not resumed";
}

}  // namespace joint_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(joint_hardware::LiftHardware, hardware_interface::SystemInterface)

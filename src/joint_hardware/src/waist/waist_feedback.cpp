#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{

bool WaistHardware::manufacturer_error_known() const
{
  return waist_feedback_valid_;
}

bool WaistHardware::manufacturer_error_active() const
{
  return waist_feedback_valid_ &&
         (waist_feedback_state_ & waist_can::kFeedbackStateError) != 0U;
}

uint16_t WaistHardware::effective_error_code() const
{
  return waist_feedback_valid_.load(std::memory_order_acquire) ?
         waist_feedback_err_code_.load(std::memory_order_relaxed) : 0U;
}

bool WaistHardware::try_update_waist_feedback(
  uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  waist_can::Feedback feedback;
  if (!waist_can::parse_feedback(can_id, data, dlc, waist_node_id_, feedback)) {
    return false;
  }

  waist_feedback_pos_raw_ = feedback.position_raw;
  waist_feedback_vel_rpm_ = feedback.velocity_rpm;
  waist_feedback_current_ma_ = feedback.current_ma;
  waist_feedback_err_code_ = feedback.error_code;
  waist_feedback_temp_tenth_c_ = feedback.temperature_tenth_c;
  waist_feedback_mode_ = feedback.mode;
  waist_feedback_state_ = feedback.state;
  waist_feedback_valid_ = true;
  waist_feedback_stamp_ = std::chrono::steady_clock::now();

  if (
    !waist_feedback_logged_ ||
    waist_feedback_last_log_err_code_ != waist_feedback_err_code_ ||
    waist_feedback_last_log_state_ != waist_feedback_state_)
  {
    const std::string error_text = waist_can::describe_error_code(waist_feedback_err_code_);
    RCLCPP_INFO(
      logger_,
      "Waist feedback: pos_raw=%d vel=%d err=0x%04X(%s) mode=0x%02X state=0x%02X",
      static_cast<int>(waist_feedback_pos_raw_),
      static_cast<int>(waist_feedback_vel_rpm_),
      static_cast<unsigned int>(waist_feedback_err_code_),
      error_text.c_str(),
      static_cast<unsigned int>(waist_feedback_mode_),
      static_cast<unsigned int>(waist_feedback_state_));
    waist_feedback_logged_ = true;
    waist_feedback_last_log_err_code_.store(
      waist_feedback_err_code_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    waist_feedback_last_log_state_.store(
      waist_feedback_state_.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
  return true;
}

bool WaistHardware::poll_waist_feedback(int timeout_ms)
{
  if (!usb_bridge_) {
    return false;
  }
  if (timeout_ms <= 0) {
    return waist_feedback_valid_;
  }

  const auto before = waist_feedback_stamp_.load(std::memory_order_acquire);
  std::unique_lock<std::mutex> lock(bridge_rx_mutex_);
  return bridge_rx_cv_.wait_for(
    lock,
    std::chrono::milliseconds(timeout_ms),
    [this, before]() {
      return waist_feedback_stamp_.load(std::memory_order_acquire) != before || !usb_bridge_;
    });
}

bool WaistHardware::poll_waist_position(int timeout_ms)
{
  if (timeout_ms > 0 && !waist_feedback_valid_) {
    (void)poll_waist_feedback(timeout_ms);
  }
  if (!waist_feedback_valid_) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto feedback_stamp = waist_feedback_stamp_.load(std::memory_order_acquire);
  if (
    feedback_stamp.time_since_epoch().count() == 0 ||
    now - feedback_stamp > std::chrono::milliseconds(500))
  {
    return false;
  }

  const bool recovering = !waist_position_valid_;
  hw_positions_[idx_joint_qugan_] = clamp(
    qugan_sign_ * waist_can::raw_to_rad(waist_feedback_pos_raw_),
    joint_qugan_min_rad_,
    joint_qugan_max_rad_);
  waist_position_valid_ = true;
  waist_position_feedback_stamp_.store(feedback_stamp, std::memory_order_release);

  if (!waist_position_log_valid_ || waist_position_last_raw_ != waist_feedback_pos_raw_) {
    RCLCPP_DEBUG(
      logger_, "Waist position from 0x300: raw=%d position=%.4f rad",
      static_cast<int>(waist_feedback_pos_raw_), hw_positions_[idx_joint_qugan_]);
    waist_position_log_valid_ = true;
    waist_position_last_raw_ = waist_feedback_pos_raw_;
  }

  if (recovering) {
    hw_commands_[idx_joint_qugan_] = hw_positions_[idx_joint_qugan_];
    hw_velocity_commands_[idx_joint_qugan_] = 0.0;
    prev_positions_[idx_joint_qugan_] = hw_positions_[idx_joint_qugan_];
    RCLCPP_INFO(
      logger_, "Waist command synchronized to measured position %.4f rad.",
      hw_positions_[idx_joint_qugan_]);
  }
  return true;
}

}  // namespace joint_hardware

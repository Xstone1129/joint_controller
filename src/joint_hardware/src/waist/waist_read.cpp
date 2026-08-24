#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{

void WaistHardware::publish_feedback_status(std::chrono::steady_clock::time_point now)
{
  if (!feedback_status_pub_) {return;}
  if (last_feedback_status_pub_time_.time_since_epoch().count() != 0 &&
    now - last_feedback_status_pub_time_ < std::chrono::milliseconds(50)) {return;}
  last_feedback_status_pub_time_ = now;
  const auto position_feedback_stamp =
    waist_position_feedback_stamp_.load(std::memory_order_acquire);
  const long long age_ms = position_feedback_stamp.time_since_epoch().count() == 0 ? -1 :
    std::chrono::duration_cast<std::chrono::milliseconds>(
    now - position_feedback_stamp).count();
  const bool manufacturer_error = manufacturer_error_active();
  std::ostringstream out;
  out << "{\"component\":\"waist\",\"feedback_age_ms\":" << age_ms
      << ",\"feedback_fresh\":"
      << (waist_position_valid_ && age_ms >= 0 && age_ms <= 500 ? "true" : "false")
      << ",\"initialized\":" << (waist_initialized_ ? "true" : "false")
      << ",\"fault_active\":" << (manufacturer_error ? "true" : "false")
      << ",\"error_known\":" << (manufacturer_error_known() ? "true" : "false")
      << ",\"error_code\":" << effective_error_code()
      << ",\"state\":" << static_cast<unsigned int>(waist_feedback_state_)
      << ",\"position\":" << hw_positions_[idx_joint_qugan_]
      << ",\"velocity_rpm\":" << waist_feedback_vel_rpm_
      << ",\"current_ma\":" << waist_feedback_current_ma_
      << ",\"temperature_c\":" << (static_cast<double>(waist_feedback_temp_tenth_c_) / 10.0) << "}";
  std_msgs::msg::String msg;
  msg.data = out.str();
  feedback_status_pub_->publish(msg);
}

hardware_interface::return_type WaistHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!can_transport_ready()) {
    publish_feedback_status(std::chrono::steady_clock::now());
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto feedback_sync_period =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / waist_feedback_sync_hz_));
  if (
    last_waist_sync_time_.time_since_epoch().count() == 0 ||
    now - last_waist_sync_time_ >= feedback_sync_period)
  {
    runtime_sync_requested_.store(true, std::memory_order_release);
    last_waist_sync_time_ = now;
  }
  const bool position_ok = poll_waist_position(0);
  if (!position_ok) {
    waist_position_valid_ = false;
    waist_initialized_ = false;
  } else if (manufacturer_error_active()) {
    waist_initialized_ = false;
  } else if ((waist_feedback_state_ & waist_can::kFeedbackStateEnabled) != 0U) {
    waist_initialized_ = true;
  }

  const double dt = period.seconds() > 1e-6 ? period.seconds() : 0.01;
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt;
    prev_positions_[i] = hw_positions_[i];
  }

  publish_feedback_status(now);

  if (hardware_state_bus_ && idx_joint_qugan_ >= 0) {
    HardwareStateBus::JointState state;
    state.position = hw_positions_[idx_joint_qugan_];
    state.velocity = hw_velocities_[idx_joint_qugan_];
    state.effort = 0.0;
    state.seq = ++state_bus_state_seq_;
    hardware_state_bus_->write_state(kWaistSharedIndex, state);

    const auto position_feedback_stamp =
      waist_position_feedback_stamp_.load(std::memory_order_acquire);
    const bool have_position_feedback = position_feedback_stamp.time_since_epoch().count() != 0;
    const bool feedback_fresh =
      waist_position_valid_ && have_position_feedback &&
      now - position_feedback_stamp <= std::chrono::milliseconds(500);
    HardwareStateBus::WaistStatus status;
    status.initialized = waist_initialized_;
    status.feedback_valid = waist_position_valid_;
    status.feedback_fresh = feedback_fresh;
    status.fault_active = manufacturer_error_active();
    status.feedback_error_code = effective_error_code();
    status.seq = ++state_bus_status_seq_;
    hardware_state_bus_->write_waist_status(status);
  }

  return hardware_interface::return_type::OK;
}


}  // namespace joint_hardware

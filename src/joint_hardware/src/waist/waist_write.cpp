#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

hardware_interface::return_type WaistHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!can_transport_ready() || !waist_initialized_ || !waist_position_valid_) {
    return hardware_interface::return_type::OK;
  }
  if (!manufacturer_error_known() || manufacturer_error_active()) {
    waist_initialized_ = false;
    return hardware_interface::return_type::OK;
  }

  const double requested = hw_commands_[idx_joint_qugan_];
  if (!std::isfinite(requested) ||
    !std::isfinite(hw_velocity_commands_[idx_joint_qugan_]) ||
    !std::isfinite(hw_acceleration_commands_[idx_joint_qugan_]))
  {
    return hardware_interface::return_type::OK;
  }

  const double target = clamp(requested, joint_qugan_min_rad_, joint_qugan_max_rad_);
  const double current = hw_positions_[idx_joint_qugan_];
  if (std::fabs(target - current) <= command_epsilon_qugan_) {
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto resend_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / waist_command_resend_hz_));
  if (
    last_waist_command_enqueue_time_.time_since_epoch().count() != 0 &&
    now - last_waist_command_enqueue_time_ < resend_period)
  {
    return hardware_interface::return_type::OK;
  }

  runtime_target_rad_.store(target, std::memory_order_relaxed);
  runtime_target_seq_.fetch_add(1, std::memory_order_release);
  last_waist_command_enqueue_time_ = now;
  return hardware_interface::return_type::OK;
}

void WaistHardware::start_runtime_io_worker()
{
  if (runtime_io_running_.exchange(true)) {
    return;
  }
  runtime_io_thread_ = std::thread([this]() {runtime_io_worker_loop();});
}

void WaistHardware::stop_runtime_io_worker()
{
  runtime_io_running_.store(false, std::memory_order_release);
  if (runtime_io_thread_.joinable()) {
    runtime_io_thread_.join();
  }
}

void WaistHardware::runtime_io_worker_loop()
{
  uint64_t consumed_target_seq = runtime_target_seq_.load(std::memory_order_acquire);
  while (runtime_io_running_.load(std::memory_order_acquire)) {
    bool did_work = false;
    if (runtime_sync_requested_.exchange(false, std::memory_order_acq_rel)) {
      const std::array<uint8_t, 8> sync_payload{};
      (void)send_frame(waist_can::kSyncId, sync_payload, 0);
      did_work = true;
    }
    const uint64_t target_seq = runtime_target_seq_.load(std::memory_order_acquire);
    if (target_seq != consumed_target_seq) {
      const double target = runtime_target_rad_.load(std::memory_order_relaxed);
      if (move_waist_abs_rad_frame(target)) {
        last_waist_command_time_ = std::chrono::steady_clock::now();
      }
      consumed_target_seq = target_seq;
      did_work = true;
    }
    if (!did_work) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

bool WaistHardware::move_waist_abs_rad_frame(double target_rad)
{
  if (!waist_initialized_ || !waist_position_valid_ || manufacturer_error_active()) {
    return false;
  }

  const double clamped = clamp(target_rad, joint_qugan_min_rad_, joint_qugan_max_rad_);
  const double motor_deg = qugan_sign_ * clamped * 180.0 / kPi;
  const int32_t target_raw = waist_can::deg_to_raw(motor_deg);
  const auto command = waist_can::build_position_frame_command(
    waist_node_id_, target_raw, waist_profile_acc_, waist_profile_vel_);
  if (!send_frame(command.can_id, command.data, command.dlc)) {
    return false;
  }

  static auto last_log = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (now - last_log > std::chrono::milliseconds(500)) {
    RCLCPP_INFO(
      logger_, "Waist target: position=%.4f rad raw=%d vel=%d acc=%d",
      clamped, static_cast<int>(target_raw), waist_profile_vel_, waist_profile_acc_);
    last_log = now;
  }
  return true;
}

}  // namespace joint_hardware

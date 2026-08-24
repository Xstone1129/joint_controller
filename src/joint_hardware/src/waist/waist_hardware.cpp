#include "joint_hardware/waist_hardware.hpp"

#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{

hardware_interface::CallbackReturn WaistHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!open_can()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  waist_feedback_valid_ = false;
  waist_position_valid_ = false;
  waist_feedback_pos_raw_ = 0;
  waist_feedback_vel_rpm_ = 0;
  waist_feedback_current_ma_ = 0;
  waist_feedback_err_code_ = 0;
  waist_feedback_temp_tenth_c_ = 0;
  waist_feedback_mode_ = 0;
  waist_feedback_state_ = 0;
  waist_feedback_logged_ = false;
  waist_feedback_last_log_err_code_ = 0;
  waist_feedback_last_log_state_ = 0;
  waist_position_log_valid_ = false;
  waist_position_last_raw_ = 0;
  waist_feedback_stamp_ = std::chrono::steady_clock::time_point{};
  waist_position_feedback_stamp_ = std::chrono::steady_clock::time_point{};
  last_feedback_status_pub_time_ = std::chrono::steady_clock::time_point{};

  waist_initialized_ = init_waist();
  if (!waist_initialized_) {
    close_can();
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_[idx_joint_qugan_] = hw_positions_[idx_joint_qugan_];
  hw_velocity_commands_[idx_joint_qugan_] = 0.0;
  hw_acceleration_commands_[idx_joint_qugan_] = 0.0;
  prev_positions_ = hw_positions_;

  last_waist_command_time_ = std::chrono::steady_clock::time_point{};
  last_waist_command_enqueue_time_ = std::chrono::steady_clock::time_point{};
  last_waist_sync_time_ = std::chrono::steady_clock::now();
  runtime_target_rad_.store(hw_positions_[idx_joint_qugan_], std::memory_order_relaxed);
  runtime_target_seq_.store(0, std::memory_order_relaxed);
  runtime_sync_requested_.store(false, std::memory_order_relaxed);
  start_runtime_io_worker();

  if (service_node_) {
    service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    service_executor_->add_node(service_node_);
    service_spin_thread_ = std::thread(
      [this]() {
        try {
          if (service_executor_) {
            service_executor_->spin();
          }
        } catch (const std::exception & exc) {
          RCLCPP_DEBUG(logger_, "Waist hardware service executor stopped: %s", exc.what());
        }
      });
  }

  RCLCPP_INFO(logger_, "joint_hardware waist activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaistHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_runtime_io_worker();
  if (can_transport_ready()) {
    const auto disable_frame = waist_can::build_clear_error_frame_command(waist_node_id_, false);
    if (!send_frame(disable_frame.can_id, disable_frame.data, disable_frame.dlc)) {
      RCLCPP_ERROR(logger_, "Failed to send waist disable/brake frame during deactivation.");
    }
    if (!sdo_write_u16(
        waist_can::kIndexControlword, 0x00,
        waist_can::kControlwordDisableVoltage, waist_timeout_ms_))
    {
      RCLCPP_ERROR(logger_, "Waist 6040h disable was not acknowledged during deactivation.");
    }
  }
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_spin_thread_.joinable()) {
    service_spin_thread_.join();
  }
  if (service_executor_ && service_node_) {
    try {
      service_executor_->remove_node(service_node_);
    } catch (const std::exception & exc) {
      RCLCPP_DEBUG(logger_, "Waist hardware service executor remove_node skipped: %s", exc.what());
    }
  }
  service_executor_.reset();
  close_can(!waist_defer_bridge_release_on_deactivate_);
  waist_initialized_ = false;
  waist_feedback_valid_ = false;
  waist_position_valid_ = false;
  waist_feedback_logged_ = false;
  waist_position_log_valid_ = false;
  waist_feedback_stamp_ = std::chrono::steady_clock::time_point{};
  waist_position_feedback_stamp_ = std::chrono::steady_clock::time_point{};
  RCLCPP_INFO(logger_, "joint_hardware waist deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaistHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaistHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 3);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
      &hw_acceleration_commands_[i]);
  }

  return command_interfaces;
}


}  // namespace joint_hardware

PLUGINLIB_EXPORT_CLASS(
  joint_hardware::WaistHardware,
  hardware_interface::SystemInterface)

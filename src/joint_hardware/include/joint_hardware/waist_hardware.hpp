#ifndef JOINT_HARDWARE__WAIST_HARDWARE_HPP_
#define JOINT_HARDWARE__WAIST_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "joint_hardware/hardware_state_bus.hpp"
#include "joint_hardware/protocol/waist_can.hpp"
#include "joint_hardware/transport/usb_bridge_hub.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace joint_hardware
{

class WaistHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WaistHardware)

  ~WaistHardware() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_can();
  void close_can(bool allow_blocking_bridge_release = true);
  bool can_transport_ready() const;
  bool open_serial_bridge();
  void close_serial_bridge(bool allow_blocking_release = true);
  void handle_bridge_frame(const CanValue & value);

  bool init_waist();
  bool init_waist_once(int timeout_ms, bool verbose);
  bool clear_waist_error(std::string & detail);
  bool set_waist_zero_position(std::string & detail);
  bool poll_waist_feedback(int timeout_ms);
  bool poll_waist_position(int timeout_ms = -1);
  void publish_feedback_status(std::chrono::steady_clock::time_point now);

  bool move_waist_abs_rad_frame(double target_rad);
  void start_runtime_io_worker();
  void stop_runtime_io_worker();
  void runtime_io_worker_loop();
  bool try_update_waist_feedback(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool manufacturer_error_known() const;
  bool manufacturer_error_active() const;
  uint16_t effective_error_code() const;

  bool send_frame(uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc = 8);
  bool recv_frame(struct can_frame & frame, int timeout_ms);
  bool sdo_write_i8(uint16_t idx, uint8_t sub, int8_t value, int timeout_ms = -1);
  bool sdo_write_u16(uint16_t idx, uint8_t sub, uint16_t value, int timeout_ms = -1);
  bool sdo_write_i32(uint16_t idx, uint8_t sub, int32_t value, int timeout_ms = -1);
  bool sdo_write_u32(uint16_t idx, uint8_t sub, uint32_t value, int timeout_ms = -1);
  bool sdo_read_i32(uint16_t idx, uint8_t sub, int32_t & value, int timeout_ms = -1);
  bool wait_sdo_response(
    uint16_t idx, uint8_t sub, std::array<uint8_t, 8> & out_data, int timeout_ms,
    int expected_cmd = -1);

  static double clamp(double value, double lower, double upper);

  rclcpp::Logger logger_{rclcpp::get_logger("joint_hardware.WaistHardware")};
  std::shared_ptr<HardwareStateBus> hardware_state_bus_{HardwareStateBus::instance()};
  uint64_t state_bus_state_seq_{0};
  uint64_t state_bus_status_seq_{0};

  std::string waist_port_;
  int serial_baudrate_{2000000};
  int bridge_channel_{2};
  int bridge_bitrate_{5000000};
  uint32_t can_nom_baud_{1000000U};
  uint32_t can_dat_baud_{5000000U};
  UsbBridgeRouteConfig bridge_route_config_{};
  std::shared_ptr<UsbBridgeHub> usb_bridge_;
  UsbBridgeHub::ListenerHandle usb_bridge_listener_handle_{0};
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr waist_clear_error_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr waist_set_zero_position_srv_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  std::thread service_spin_thread_;
  mutable std::mutex bridge_rx_mutex_;
  std::condition_variable bridge_rx_cv_;
  std::deque<struct can_frame> bridge_rx_queue_;

  int waist_node_id_{0x0F};
  int waist_timeout_ms_{600};
  int waist_init_timeout_ms_{300};
  int waist_init_retry_count_{3};
  int waist_profile_vel_{10};
  int waist_profile_acc_{2000};
  double joint_qugan_min_rad_{0.0};
  double joint_qugan_max_rad_{1.57};
  double waist_command_resend_hz_{100.0};
  double waist_feedback_sync_hz_{100.0};
  double command_epsilon_qugan_{1e-4};
  double qugan_sign_{1.0};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_acceleration_commands_;
  std::vector<double> prev_positions_;
  int idx_joint_qugan_{-1};

  std::atomic<bool> waist_initialized_{false};
  std::atomic<bool> waist_position_valid_{false};
  bool waist_defer_bridge_release_on_deactivate_{true};
  std::atomic<bool> waist_feedback_valid_{false};
  std::atomic<int32_t> waist_feedback_pos_raw_{0};
  std::atomic<int16_t> waist_feedback_vel_rpm_{0};
  std::atomic<int16_t> waist_feedback_current_ma_{0};
  std::atomic<uint16_t> waist_feedback_err_code_{0};
  std::atomic<int16_t> waist_feedback_temp_tenth_c_{0};
  std::atomic<uint8_t> waist_feedback_mode_{0};
  std::atomic<uint8_t> waist_feedback_state_{0};
  std::atomic<bool> waist_feedback_logged_{false};
  std::atomic<uint16_t> waist_feedback_last_log_err_code_{0};
  std::atomic<uint8_t> waist_feedback_last_log_state_{0};
  bool waist_position_log_valid_{false};
  int32_t waist_position_last_raw_{0};

  std::chrono::steady_clock::time_point last_waist_command_time_{};
  std::chrono::steady_clock::time_point last_waist_command_enqueue_time_{};
  std::chrono::steady_clock::time_point last_waist_sync_time_{};
  std::atomic<std::chrono::steady_clock::time_point> waist_feedback_stamp_{};
  std::atomic<std::chrono::steady_clock::time_point> waist_position_feedback_stamp_{};
  std::chrono::steady_clock::time_point last_feedback_status_pub_time_{};
  std::atomic<bool> runtime_io_running_{false};
  std::atomic<bool> runtime_sync_requested_{false};
  std::atomic<double> runtime_target_rad_{0.0};
  std::atomic<uint64_t> runtime_target_seq_{0};
  std::thread runtime_io_thread_;
};

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__WAIST_HARDWARE_HPP_

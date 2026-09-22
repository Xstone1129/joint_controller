#ifndef JOINT_HARDWARE__LIFT_HARDWARE_HPP_
#define JOINT_HARDWARE__LIFT_HARDWARE_HPP_

#include <atomic>
#include <future>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "joint_hardware/lift/cia402.hpp"
#include "joint_hardware/lift/ethercat_backend.hpp"
#include "joint_hardware/lift/feedback_continuity.hpp"
#include "joint_hardware/lift/lift_units.hpp"
#include "joint_hardware/lift/zero_offset_store.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace joint_hardware
{

class LiftHardware final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LiftHardware)

  LiftHardware();
  explicit LiftHardware(std::unique_ptr<lift::LiftEthercatBackend> backend);
  ~LiftHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
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
  enum class BrakeStopPhase : uint8_t
  {
    disabled,
    idle,
    quick_stop
  };

  bool parse_parameters();
  bool configure_backend();
  bool bootstrap_first_feedback();
  bool validate_brake_parameters();
  bool load_zero_offset();
  bool read_validated_sdo(
    uint16_t index, uint8_t subindex, uint32_t & value, bool allow_zero = false);
  void initialize_interfaces();
  void start_service_thread();
  void stop_service_thread();
  void publish_driver_status();
  void handle_transport_fault(const char * reason);
  void update_transport_health(bool healthy) noexcept;
  uint32_t feedback_source_code() const noexcept;
  void reset_feedback_continuity() noexcept;
  void invalidate_feedback_continuity() noexcept;
  void observe_feedback_continuity(double position_m, int64_t sample_time_ns, bool fresh) noexcept;
  std::string detailed_error_reason(uint8_t code) const;
  bool feedback_is_fresh() const noexcept;
  bool compute_motion_command(double & target_wire_rpm, double period_s);
  bool complete_pending_zero();
  void apply_pending_zero_offset();
  bool target_is_at_limit(double position, double target) const noexcept;
  void update_state_interfaces();
  void latch_motion_fault(uint8_t reason_code) noexcept;

  void handle_brake_command(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handle_reset_velocity(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handle_reset_hold(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handle_reset_zero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_home(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_set_drive_zero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // Read-only object-dictionary diagnostics (see handle_read_drive_objects).
  //
  // The IGH backend uploads SDOs with ecrt_master_sdo_upload(), which blocks
  // until the master services the request.  While the realtime cycle owns the
  // master that can take arbitrarily long, so the read runs on a worker and the
  // service answers with a bounded timeout instead of hanging its caller.  At
  // most one read is in flight; a stuck one is never retried concurrently.
  std::shared_ptr<std::future<std::string>> sdo_read_future_;
  std::string read_drive_objects_text();
  void handle_read_drive_objects(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool wait_for_drive_stop(std::chrono::milliseconds timeout);
  bool write_homing_sdo(uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value);
  bool wait_for_homing_result(std::chrono::milliseconds timeout);
  bool wait_for_host_zero(std::chrono::milliseconds timeout);
  void handle_estop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_safety_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Logger logger_{rclcpp::get_logger("joint_lift_hardware")};

  std::unique_ptr<lift::LiftEthercatBackend> backend_;
  // SDO transfers are synchronous in EtherLab and require the PDO cycle to
  // continue servicing the master. Keep them serialized without blocking the
  // backend mutex used by read()/write().
  std::mutex sdo_mutex_;
  bool backend_injected_{false};
  lift::EthercatMasterConfig master_config_{};
  lift::EthercatSlaveConfig slave_config_{};
  lift::LiftUnitConfig unit_config_{};
  lift::Cia402Controller cia402_{};
  lift::FeedbackContinuityGuard feedback_continuity_{};

  std::string backend_name_{"unavailable"};
  std::string motor_id_{"LVM08008H3G3-M17"};
  std::string zero_offset_file_{"/tmp/joint_hardware_lift_zero_offset.cfg"};
  // The application coordinate frame is host-managed. This keeps a requested
  // zero independent of the LD3M absolute-encoder maintenance setting.
  bool use_persistent_zero_offset_{true};
  uint32_t configured_command_units_per_rev_{10000};
  bool command_units_parameter_set_{false};
  uint32_t expected_working_counter_{1};
  int feedback_timeout_ms_{100};
  int brake_release_wait_ms_{-1};
  int brake_release_wait_observed_ms_{-1};
  int max_fault_reset_attempts_{3};
  int fault_reset_backoff_ms_{100};
  // 6061h may briefly expose the previous mode during CiA 402 transitions.
  // A persistent mismatch while enable is explicitly requested still latches
  // a mode fault after this bounded number of 10 ms cycles.
  int mode_mismatch_debounce_cycles_{5};
  // Require consecutive healthy PDO samples before a transient transport
  // failure is considered recovered.
  uint32_t transport_recovery_required_cycles_{5};
  int motion_command_period_ms_{10};
  int reset_velocity_rpm_{96};
  int reset_velocity_period_ms_{50};
  int reset_velocity_timeout_ms_{1000};
  int homing_method_{19};
  int homing_speed_high_units_s_{10000};
  int homing_speed_low_units_s_{5000};
  int homing_acceleration_units_s2_{500000};
  int homing_offset_units_{0};
  int homing_timeout_ms_{60000};
  int drive_zero_timeout_ms_{5000};
  int startup_motion_guard_ms_{3000};
  int max_rpm_{0};
  int position_limit_recovery_max_rpm_{300};
  int stop_window_max_rpm_{10};
  int overshoot_recovery_max_rpm_{10};
  int stop_target_stable_ms_{50};
  int brake_p04_37_ms_{150};
  int brake_p04_39_rpm_{30};
  int brake_p06_14_ms_{500};
  int limit_switch_positive_bit_{-1};
  int limit_switch_negative_bit_{-1};
  std::string brake_p05_06_mode_{"drive_default"};
  std::string brake_p05_10_mode_{"drive_default"};
  double kp_rpm_per_m_{3000.0};
  double kd_rpm_per_mps_{300.0};
  double deadband_m_{0.001};
  double slowdown_distance_m_{0.006};
  double stop_window_m_{0.002};
  double feedback_filter_alpha_{0.20};
  double velocity_slew_rpm_per_s_{600.0};
  double stop_slew_rpm_per_s_{0.0};
  double brake_accel_rpm_per_s_{1200.0};
  double overshoot_guard_window_m_{0.006};
  double stationary_velocity_threshold_mps_{0.001};
  double command_epsilon_m_{0.0005};
  double command_epsilon_rpm_{1.0};
  double max_feedback_jump_m_{0.2};
  double max_feedback_velocity_mps_{std::numeric_limits<double>::quiet_NaN()};
  double feedback_velocity_tolerance_mps_{0.003};
  double position_min_m_{std::numeric_limits<double>::quiet_NaN()};
  double position_max_m_{std::numeric_limits<double>::quiet_NaN()};
  double reset_max_search_travel_m_{0.85};
  // Releasing a vertical-axis brake must always be an explicit opt-in.
  bool brake_control_enabled_{false};
  bool limit_switch_enabled_{false};
  bool limit_switch_active_high_{true};
  bool direct_stop_in_stop_window_{false};
  bool auto_fault_reset_{true};
  bool startup_motion_guard_enabled_{true};
  bool reset_velocity_debug_enabled_{false};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_status_;
  std::vector<double> hw_error_codes_;
  std::vector<double> hw_modes_;
  std::vector<double> hw_brake_unlocked_;
  std::vector<double> hw_digital_inputs_;
  // power_enable follows the arm interface contract: the command is only a
  // request, while the state is derived from fresh CiA 402/PDO feedback.
  std::vector<double> hw_power_enabled_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_acceleration_commands_;
  std::vector<double> hw_power_commands_;
  int idx_joint_motor_{-1};
  bool has_velocity_command_{false};
  bool has_acceleration_command_{false};

  lift::LiftRxPdo tx_pdo_{};
  lift::LiftTxPdo rx_pdo_{};
  int32_t zero_offset_units_{0};
  bool zero_offset_loaded_{false};
  bool configured_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> brake_request_enable_{false};
  std::atomic<bool> service_disable_latched_{false};
  // Set by the compatibility service and consumed by the 100 Hz cycle. The
  // service never writes PDO memory directly, avoiding a data race with
  // read()/write().
  std::atomic<bool> brake_stop_requested_{false};
  std::atomic<bool> brake_stop_timeout_{false};
  std::atomic<uint8_t> brake_stop_phase_atomic_{
    static_cast<uint8_t>(BrakeStopPhase::disabled)};
  std::atomic<bool> reset_velocity_request_{false};
  std::atomic<bool> reset_velocity_stop_pending_{false};
  std::atomic<double> reset_search_start_position_m_{0.0};
  std::atomic<bool> reset_hold_request_{false};
  std::atomic<bool> zero_request_pending_{false};
  std::atomic<double> reset_hold_target_m_{0.0};
  std::atomic<double> feedback_position_m_{0.0};
  std::atomic<bool> motion_blocked_{false};
  std::atomic<bool> position_limit_violation_{false};
  std::atomic<bool> limit_recovery_active_{false};
  std::atomic<bool> feedback_fresh_{false};
  std::atomic<bool> power_command_requested_{false};
  std::atomic<bool> power_enabled_{false};
  std::atomic<double> feedback_velocity_mps_{0.0};
  std::atomic<int32_t> feedback_position_units_{0};
  std::atomic<bool> brake_unlocked_{false};
  std::atomic<uint16_t> status_word_{0};
  std::atomic<uint16_t> error_code_{0};
  std::atomic<int8_t> mode_display_{0};
  std::atomic<uint32_t> digital_inputs_{0};
  std::atomic<uint32_t> fault_reset_attempts_{0};
  std::atomic<bool> recovery_exhausted_{false};
  std::atomic<uint32_t> mode_mismatch_cycles_{0};
  std::atomic<int32_t> zero_offset_units_atomic_{0};
  // Bumped every time the lift coordinate frame moves underneath the
  // command path (a host re-zero rewrites the drive origin).  Published in
  // the driver status so LiftController can re-seat its own setpoint
  // instead of chasing a target that belongs to the previous frame.
  std::atomic<uint64_t> zero_generation_{0};
  std::atomic<int64_t> last_feedback_ns_{0};
  std::atomic<uint8_t> link_state_atomic_{
    static_cast<uint8_t>(lift::EthercatLinkState::offline)};
  std::atomic<uint32_t> working_counter_atomic_{0};
  std::atomic<uint8_t> error_reason_code_{0};
  std::atomic<uint8_t> latched_motion_reason_code_{0};
  std::atomic<bool> transport_fault_active_{false};
  std::atomic<uint32_t> transport_recovery_healthy_cycles_{0};
  std::atomic<bool> zero_disable_observed_{false};
  std::atomic<bool> zero_offset_apply_pending_{false};
  std::atomic<bool> homing_active_{false};
  std::atomic<bool> homing_start_requested_{false};
  std::atomic<bool> homing_complete_{false};
  std::atomic<bool> homing_failed_{false};
  std::atomic<bool> drive_zero_pending_{false};
  std::atomic<bool> estop_latched_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> first_write_sync_pending_{false};
  std::atomic<double> command_position_atomic_{0.0};
  std::atomic<double> command_velocity_atomic_{0.0};
  std::atomic<double> command_acceleration_atomic_{0.0};
  std::atomic<double> target_wire_rpm_atomic_{0.0};
  std::atomic<int32_t> target_velocity_units_atomic_{0};

  // EtherLab mailbox access and cyclic PDO access share the same master. The
  // maintenance services use this lock while the axis is stopped.
  mutable std::mutex backend_mutex_;

  std::atomic<bool> feedback_baseline_valid_{false};
  std::atomic<bool> feedback_continuity_reset_pending_{false};
  std::atomic<uint64_t> feedback_epoch_{0};
  std::atomic<uint32_t> feedback_source_atomic_{0};
  std::atomic<double> jump_previous_position_m_{0.0};
  std::atomic<double> jump_current_position_m_{0.0};
  std::atomic<double> jump_delta_m_{0.0};
  std::atomic<double> jump_interval_ms_{0.0};
  std::atomic<uint64_t> jump_epoch_{0};
  std::atomic<uint32_t> jump_source_{0};

  double filtered_velocity_mps_{0.0};
  double last_target_wire_rpm_{0.0};
  double last_target_position_m_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point brake_enable_time_{};
  BrakeStopPhase brake_stop_phase_{BrakeStopPhase::disabled};
  std::chrono::steady_clock::time_point brake_stop_start_time_{};
  bool previous_operation_enabled_{false};
  bool previous_power_command_{false};
  std::chrono::steady_clock::time_point target_change_time_{};
  std::chrono::steady_clock::time_point last_clamp_log_time_{};
  std::chrono::steady_clock::time_point startup_guard_until_{};
  std::chrono::steady_clock::time_point reset_velocity_start_time_{};

  std::atomic<bool> target_clamp_pending_{false};
  std::atomic<double> last_unclamped_target_m_{0.0};
  std::atomic<double> last_clamped_target_m_{0.0};

  rclcpp::Node::SharedPtr service_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr service_executor_;
  std::thread service_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_velocity_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_hold_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_zero_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drive_zero_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_drive_objects_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drive_zero_alias_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safety_reset_service_;
};

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__LIFT_HARDWARE_HPP_

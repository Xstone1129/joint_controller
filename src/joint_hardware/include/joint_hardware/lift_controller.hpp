#ifndef JOINT_HARDWARE__LIFT_CONTROLLER_HPP_
#define JOINT_HARDWARE__LIFT_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "joint_hardware/lift/lift_motion_profile.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_command_v1.hpp"
#include "robot_control_msg/srv/selected_joint_control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_hardware
{

class LiftController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class CommandType : uint8_t
  {
    none,
    position,
    trajectory,
    jog,
    stream,
    soft_stop,
    hold,
  };

  enum class Mode : uint8_t
  {
    hold,
    brake_gate,
    position,
    trajectory,
    jog,
    streaming,
    soft_stop,
    estop,
    fault,
  };

  enum class SoftStopCompletion : uint8_t
  {
    disable,
    hold_powered,
  };

  enum class HoldFailureStage : uint8_t
  {
    none,
    driver_gate,
    motion_profile,
    velocity_confirmation,
    mode_confirmation,
  };

  struct MotionCommand
  {
    CommandType type{CommandType::none};
    double position_m{0.0};
    double velocity_mps{0.0};
    double acceleration_mps2{0.0};
    double acceleration_limit_mps2{0.0};
    double velocity_scale{0.0};
    uint64_t generation{0};
    uint64_t heavy_sequence{0};
  };

  uint64_t enqueue_command(MotionCommand command);
  uint64_t enqueue_safety_command(MotionCommand command);
  uint64_t next_motion_generation() noexcept;
  void cancel_pending_trajectory(uint64_t cancel_generation);
  bool hold_snapshot_is_safe() const noexcept;
  void begin_brake_gate(const MotionCommand & command);
  void start_gated_motion();
  void begin_soft_stop(
    const char * reason,
    SoftStopCompletion completion = SoftStopCompletion::disable,
    uint64_t hold_generation = 0);
  void apply_zero_frame_resync();
  void enter_hold(double measured_position, const char * reason);
  void confirm_pending_hold();
  void fail_pending_hold(HoldFailureStage stage);
  static const char * hold_failure_stage_name(HoldFailureStage stage) noexcept;
  bool driver_gate_ready() const noexcept;
  bool driver_status_fresh() const noexcept;
  // Categorized diagnostics.  Every line carries a stable category tag so one
  // session log can be filtered afterwards without re-running the motion:
  //   [LIFT_FAULT] latched fault / latch cleared   ERROR (1 Hz heartbeat), WARN
  //   [LIFT_GATE]  driver gate degraded / recovered DEBUG (1 Hz), promoted WARN
  //   [LIFT_CMD]   rejected or ignored command     DEBUG (10 Hz max)
  //   [LIFT_CFG]   configuration and lifecycle     INFO
  // A latched fault re-enters the same branch on every 100 Hz cycle, so the
  // detail string is built only when a line is really emitted; repeats inside
  // one throttle window are counted instead of formatted.
  std::string driver_gate_failure_reason() const;
  bool driver_gate_expected() const noexcept;
  bool lift_fault_line_due(const char * code) const noexcept;
  void note_lift_fault_repeat(const char * code) noexcept;
  void write_lift_fault(const char * code, const std::string & detail);

  // Newest latched fault as "code: detail", empty when nothing is latched.
  // Published as the top-level fault_reason of the controller status so a
  // controller-side fault is never reported to the host without a reason.
  std::string last_fault_text() const;
  template<typename DetailBuilder>
  void record_lift_fault(const char * code, DetailBuilder && build_detail)
  {
    if (!lift_fault_line_due(code)) {
      note_lift_fault_repeat(code);
      return;
    }
    write_lift_fault(code, build_detail());
  }
  void log_gate_diagnostics(const char * context);
  void log_command_rejection(const char * category, const std::string & detail);
  void request_brake(bool enable);
  void on_driver_status(const std_msgs::msg::String & message);
  void publish_status();
  void write_command_interfaces(const lift::LiftMotionSample & sample);
  bool start_trajectory_point(std::size_t index);
  static const char * mode_name(Mode mode) noexcept;

  std::string joint_name_{"joint_motor"};
  std::string command_service_name_{"/joint/lift/command"};
  std::string trajectory_topic_{"/joint/lift/trajectory"};
  std::string stream_topic_{"/joint/lift/stream"};
  std::string jog_topic_{"/joint/lift/jog_velocity"};
  std::string status_topic_{"/joint/lift/control_status"};
  std::string driver_status_topic_{"/joint/lift/driver_status"};
  double jog_timeout_sec_{0.45};
  double brake_gate_stable_sec_{0.1};
  double driver_status_timeout_sec_{2.0};
  double target_stable_sec_{1.0};
  double goal_tolerance_m_{0.001};
  double stationary_velocity_mps_{0.001};
  double status_rate_hz_{20.0};

  lift::LiftMotionLimits limits_{};
  lift::LiftMotionProfile profile_{0.01};
  lift::LiftMotionSample command_sample_{};
  realtime_tools::RealtimeBuffer<MotionCommand> command_buffer_;
  realtime_tools::RealtimeBuffer<MotionCommand> safety_command_buffer_;
  realtime_tools::RealtimeBuffer<MotionCommand> heavy_command_buffer_;
  std::atomic<uint64_t> command_generation_{0};
  std::atomic<uint64_t> safety_command_generation_{0};
  std::atomic<uint64_t> heavy_command_generation_{0};
  std::atomic<uint64_t> admitted_heavy_sequence_{0};
  std::atomic<uint64_t> applied_heavy_sequence_{0};
  std::atomic<uint64_t> motion_generation_{0};
  std::atomic<uint64_t> cancellation_generation_{0};
  std::atomic<uint64_t> applied_cancellation_generation_{0};
  std::atomic<bool> heavy_lease_active_{false};
  bool heavy_lease_active_rt_{false};
  uint64_t consumed_generation_{0};
  uint64_t consumed_safety_command_generation_{0};
  uint64_t consumed_heavy_command_generation_{0};
  // FOLLOW is not applied until the CiA402/brake gate has entered streaming.
  // Keep its acknowledgement pending so the gateway cannot report execution
  // while the command interfaces still carry a stationary HOLD sample.
  uint64_t pending_heavy_follow_ack_sequence_{0};
  MotionCommand gated_command_{};
  bool gated_command_valid_{false};

  std::mutex trajectory_mutex_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr pending_trajectory_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr active_trajectory_;
  uint64_t pending_trajectory_generation_{0};
  uint64_t consumed_trajectory_generation_{0};
  uint64_t active_trajectory_generation_{0};
  std::size_t trajectory_point_index_{0};
  double trajectory_elapsed_sec_{0.0};
  bool trajectory_waiting_at_point_{false};

  Mode mode_{Mode::hold};
  double measured_position_m_{0.0};
  double measured_velocity_mps_{0.0};
  std::atomic<bool> active_{false};
  bool goal_stable_active_{false};
  SoftStopCompletion soft_stop_completion_{SoftStopCompletion::disable};
  uint64_t pending_hold_generation_{0};
  std::atomic<uint64_t> completed_hold_generation_{0};
  std::atomic<uint64_t> failed_hold_generation_{0};
  std::atomic<uint8_t> hold_failure_stage_{static_cast<uint8_t>(HoldFailureStage::none)};
  std::chrono::steady_clock::time_point goal_stable_since_{};
  std::chrono::steady_clock::time_point gate_stable_since_{};
  std::chrono::steady_clock::time_point last_jog_command_time_{};
  const char * status_message_{"HOLD"};

  std::atomic<double> measured_position_atomic_{0.0};
  std::atomic<double> measured_velocity_atomic_{0.0};
  std::atomic<double> command_position_atomic_{0.0};
  std::atomic<double> command_velocity_atomic_{0.0};
  std::atomic<double> command_acceleration_atomic_{0.0};
  std::atomic<uint8_t> mode_atomic_{static_cast<uint8_t>(Mode::hold)};
  std::atomic<bool> trajectory_active_atomic_{false};
  std::atomic<bool> gated_command_active_atomic_{false};

  std::atomic<bool> driver_feedback_fresh_{false};
  std::atomic<bool> driver_ethercat_operational_{false};
  std::atomic<bool> driver_working_counter_ok_{false};
  std::atomic<bool> driver_operation_enabled_{false};
  std::atomic<bool> driver_brake_unlocked_{false};
  std::atomic<bool> driver_estop_latched_{false};
  std::atomic<bool> driver_motion_blocked_{false};
  std::atomic<bool> driver_quick_stop_{false};
  std::atomic<uint16_t> driver_error_code_{0};
  std::atomic<int8_t> driver_mode_display_{0};
  std::atomic<int64_t> driver_status_ns_{0};
  // Coordinate-frame generation reported by LiftHardware.  A change means a
  // re-zero moved the drive origin, so every setpoint captured earlier is
  // now expressed in the previous frame.
  std::atomic<uint64_t> driver_zero_generation_{0};
  std::atomic<bool> driver_zero_generation_valid_{false};
  uint64_t applied_zero_generation_{0};
  uint64_t zero_resync_total_{0};
  double last_zero_resync_from_m_{0.0};
  double last_zero_resync_to_m_{0.0};
  std::atomic<bool> power_enable_requested_{false};
  std::atomic<bool> power_enabled_state_{false};

  // Bounded fault history so a fault that already recovered is still visible
  // after the fact. Fixed-size char buffers keep a record allocation-free; only
  // an emitted fault line (at most once per second) formats strings. A latch
  // that keeps re-entering the same branch inside one burst updates the newest
  // record instead of evicting the rest of the history.
  static constexpr std::size_t kFaultHistoryCapacity = 8;
  static constexpr int64_t kFaultBurstWindowNs = 60000000000LL;    // 60 s
  static constexpr int64_t kFaultLineThrottleNs = 1000000000LL;    // 1 s
  static constexpr int64_t kGateDebugThrottleNs = 1000000000LL;    // 1 s
  static constexpr int64_t kGateWarnWindowNs = 5000000000LL;       // 5 s
  static constexpr int64_t kCommandDebugThrottleNs = 100000000LL;  // 100 ms
  static constexpr double kGateWarnAfterSec = 2.0;
  struct LiftFaultRecord
  {
    int64_t first_ns{0};
    int64_t last_ns{0};
    uint32_t repeats{0};
    char code[40]{};
    char detail[256]{};
  };
  std::array<LiftFaultRecord, kFaultHistoryCapacity> fault_history_{};
  std::size_t fault_history_count_{0};
  std::size_t fault_history_next_{0};
  uint64_t fault_total_{0};
  int64_t last_fault_record_ns_{0};
  int64_t last_fault_line_ns_{0};
  uint64_t fault_line_suppressed_{0};
  char last_fault_code_[40]{};
  int64_t last_gate_warn_ns_{0};
  int64_t last_gate_debug_ns_{0};
  int64_t gate_unready_since_ns_{0};
  bool gate_warn_emitted_{false};
  int64_t last_command_debug_ns_{0};

  std::size_t position_command_index_{0};
  std::size_t velocity_command_index_{1};
  std::size_t acceleration_command_index_{2};
  std::size_t power_enable_command_index_{3};
  std::size_t position_state_index_{0};
  std::size_t velocity_state_index_{1};
  std::size_t power_enable_state_index_{2};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr driver_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr jog_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr stream_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heavy_lease_gate_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1>::SharedPtr
    heavy_command_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr heavy_ack_publisher_;
  rclcpp::Service<robot_control_msg::srv::SelectedJointControl>::SharedPtr command_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_service_;
};

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__LIFT_CONTROLLER_HPP_

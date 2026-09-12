#ifndef ROBOT_LOWER_GATEWAY__HEAVY_GATEWAY_V1_HPP_
#define ROBOT_LOWER_GATEWAY__HEAVY_GATEWAY_V1_HPP_

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/arm_control_mode_status.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_command_v1.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_feedback_v1.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_status_v1.hpp"
#include "robot_control_msg/msg/lift_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "robot_control_msg/srv/acquire_heavy_execution_lease_v1.hpp"
#include "robot_control_msg/srv/get_heavy_gateway_capabilities_v1.hpp"
#include "robot_control_msg/srv/release_heavy_execution_lease_v1.hpp"
#include "robot_control_msg/srv/set_heavy_lift_brake_v1.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace robot_lower_gateway
{

class HeavyGatewayV1
{
public:
  explicit HeavyGatewayV1(rclcpp::Node & node);

private:
  using Command = robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1;
  using Feedback = robot_control_msg::msg::HeavyUpperBodyGatewayFeedbackV1;
  using Status = robot_control_msg::msg::HeavyUpperBodyGatewayStatusV1;
  using Capabilities = robot_control_msg::srv::GetHeavyGatewayCapabilitiesV1;
  using AcquireLease = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  using ReleaseLease = robot_control_msg::srv::ReleaseHeavyExecutionLeaseV1;
  using LiftBrake = robot_control_msg::srv::SetHeavyLiftBrakeV1;

  struct JointSnapshot
  {
    std::array<double, 15> position{};
    std::array<double, 15> velocity{};
    std::uint16_t position_valid_mask{0};
    std::uint16_t velocity_valid_mask{0};
    bool arm_received{false};
    bool lift_received{false};
    std::chrono::steady_clock::time_point arm_time{};
    std::chrono::steady_clock::time_point lift_time{};
  };

  // These fields always move together under mutex_.  References below retain
  // the established local names while preventing an owner/lease split across
  // independently stored state.
  struct LeaseState
  {
    bool active{false};
    std::string owner_id;
    std::string lease_id;
    std::uint64_t last_received_sequence{0};
    std::uint64_t last_accepted_sequence{0};
    std::uint64_t applied_command_sequence{0};
    std::chrono::steady_clock::time_point last_received_command_time{};
    std::chrono::steady_clock::time_point last_accepted_command_time{};
    std::uint8_t state{Status::STATE_OFFLINE};
    std::string detail{"waiting for complete lower-controller feedback"};
    std::array<double, 15> watchdog_hold_position{};
    bool watchdog_hold_captured{false};
  };

  static constexpr double kNoCommandAgeMs = -1.0;

  void configureParameters();
  void configureRosInterfaces();
  void handleCommand(const Command::SharedPtr message);
  void handleCapabilities(
    const Capabilities::Request::SharedPtr request,
    Capabilities::Response::SharedPtr response);
  void handleAcquire(
    const AcquireLease::Request::SharedPtr request,
    AcquireLease::Response::SharedPtr response);
  void handleRelease(
    const ReleaseLease::Request::SharedPtr request,
    ReleaseLease::Response::SharedPtr response);
  void handleLiftBrake(
    const LiftBrake::Request::SharedPtr request,
    LiftBrake::Response::SharedPtr response);
  void handleArmJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void handleLiftJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void handleArmAck(const std_msgs::msg::UInt64::SharedPtr message);
  void handleLiftAck(const std_msgs::msg::UInt64::SharedPtr message);
  void handleAck(bool arm, std::uint64_t sequence);
  void watchdogAndFeedbackTick();
  void publishStatus(bool force = false);
  void publishFeedback();
  void publishInternalCommand(const Command & command, bool retry = false);
  void publishLeaseGate(bool active);
  void beginSafetyHoldLocked(const std::string & reason, bool revoke_lease);
  Command dispatchUserCommandLocked(
    const Command & command, const std::chrono::steady_clock::time_point & now);
  std::uint64_t newestAdmittedCommandSequenceLocked() const;
  struct BrakeAction
  {
    bool release{false};
    std::string reason;
  };
  void queueBrakeLockLocked(const std::string & reason, bool force);
  std::optional<BrakeAction> takePendingBrakeActionLocked(
    const std::chrono::steady_clock::time_point & now);
  bool executeBrakeAction(const BrakeAction & action);
  bool liftEnableReadyLocked(
    const std::chrono::steady_clock::time_point & now, std::string & reason) const;
  bool liftPowerEnabledLocked(const std::chrono::steady_clock::time_point & now) const;
  bool controllerEndpointsReadyLocked(std::string & reason) const;
  std::string brakeDiagnosticLocked(const std::chrono::steady_clock::time_point & now) const;
  bool feedbackReadyLocked(
    const std::chrono::steady_clock::time_point & now, std::string & reason) const;
  bool jointFeedbackCompleteLocked(
    const std::chrono::steady_clock::time_point & now) const;
  bool workspaceReadyLocked(
    const std::chrono::steady_clock::time_point & now, std::string & reason) const;
  bool armHoldCommandLocked(const Command & command, std::string & reason) const;
  bool validateCommandLocked(
    const Command & command, const std::chrono::steady_clock::time_point & now,
    std::string & reason, bool allow_lift_power_disabled = false);
  void recordCommandDiagnosticLocked(
    const Command & command, const std::string & outcome, const std::string & reason,
    const std::chrono::steady_clock::time_point & now);
  void recordLeaseEventLocked(
    const std::string & event, const std::string & reason,
    const std::chrono::steady_clock::time_point & now) const;
  void clearHeavyLeaseLocked(
    const std::string & reason, const std::chrono::steady_clock::time_point & now);
  double clampLiftSafetyTargetLocked(double target) const;
  Command makeHoldCommandLocked(std::uint64_t sequence) const;
  std::string generateLeaseId(const std::string & owner);
  double commandAgeMsLocked(const std::chrono::steady_clock::time_point & now) const;
  double receivedCommandAgeMsLocked(const std::chrono::steady_clock::time_point & now) const;
  double leaseRemainingMsLocked(const std::chrono::steady_clock::time_point & now) const;
  void updateStateWithoutLeaseLocked(const std::chrono::steady_clock::time_point & now);

  rclcpp::Node & node_;
  rclcpp::Logger logger_;
  std::vector<std::string> arm_native_names_;
  std::string lift_native_name_{"joint_motor"};
  std::string heavy_lift_brake_native_service_;
  double command_rate_hz_{100.0};
  double controller_rate_hz_{100.0};
  double feedback_rate_hz_{100.0};
  double status_rate_hz_{10.0};
  double arm_max_velocity_rad_s_{0.3};
  double arm_max_acceleration_rad_s2_{1.0};
  double lift_min_position_m_{-1.0};
  double lift_max_position_m_{0.0};
  double lift_position_tolerance_m_{0.002};
  double lift_max_velocity_mps_{0.060};
  double lift_max_acceleration_mps2_{0.10};
  bool allow_lift_only_follow_{false};
  double lift_only_arm_hold_position_tolerance_rad_{0.001};
  std::chrono::milliseconds command_hold_timeout_{100};
  std::chrono::milliseconds feedback_fault_timeout_{100};
  std::chrono::milliseconds workspace_status_timeout_{3000};
  std::chrono::milliseconds execution_lease_timeout_{500};
  std::chrono::milliseconds startup_feedback_grace_{2000};
  std::chrono::milliseconds controller_ack_timeout_{200};
  std::chrono::milliseconds controller_ack_retry_interval_{50};
  std::chrono::milliseconds service_timeout_{5000};
  std::chrono::milliseconds lift_enable_timeout_{1000};
  double brake_lock_velocity_threshold_mps_{0.001};
  std::chrono::steady_clock::time_point started_at_;

  mutable std::mutex mutex_;
  std::condition_variable state_condition_;
  JointSnapshot joints_;
  robot_control_msg::msg::ArmPowerStatus arm_power_;
  robot_control_msg::msg::LiftStatus lift_status_;
  robot_control_msg::msg::WorkspaceStatus workspace_status_;
  robot_control_msg::msg::ArmControlModeStatus control_mode_status_;
  bool arm_power_received_{false};
  bool lift_status_received_{false};
  bool workspace_received_{false};
  bool control_mode_received_{false};
  std::chrono::steady_clock::time_point arm_power_time_{};
  std::chrono::steady_clock::time_point lift_status_time_{};
  std::chrono::steady_clock::time_point workspace_time_{};
  std::chrono::steady_clock::time_point control_mode_time_{};

  LeaseState lease_;
  std::uint8_t & state_{lease_.state};
  std::string & detail_{lease_.detail};
  std::string & owner_id_{lease_.owner_id};
  std::string & lease_id_{lease_.lease_id};
  std::uint64_t lease_generation_{0};
  std::uint64_t initial_sequence_{0};
  std::uint64_t & last_command_sequence_{lease_.last_accepted_sequence};
  std::uint64_t & applied_command_sequence_{lease_.applied_command_sequence};
  std::uint64_t controller_command_generation_{0};
  std::uint64_t pending_command_sequence_{0};
  std::uint64_t pending_external_command_sequence_{0};
  double pending_lift_target_m_{0.0};
  Command pending_internal_command_{};
  std::chrono::steady_clock::time_point last_pending_command_publish_time_{};
  std::uint64_t controller_ack_retry_count_{0};
  std::uint64_t arm_ack_sequence_{0};
  std::uint64_t lift_ack_sequence_{0};
  std::uint64_t last_arm_ack_received_sequence_{0};
  std::uint64_t last_lift_ack_received_sequence_{0};
  std::uint64_t ignored_arm_ack_count_{0};
  std::uint64_t ignored_lift_ack_count_{0};
  std::uint64_t feedback_sequence_{0};
  bool & lease_active_{lease_.active};
  bool aligned_hold_{false};
  bool pending_command_valid_{false};
  bool pending_command_is_user_{false};
  // At most one command may wait for lower-controller acknowledgement.  New
  // valid frames replace this slot, but cannot replace the in-flight command
  // or extend the execution lease before they are actually dispatched.
  bool staged_command_valid_{false};
  Command staged_command_{};
  bool revoke_after_ack_{false};
  std::uint8_t last_mode_{Command::MODE_STOP};
  bool first_user_command_for_lease_{true};
  bool lease_acquisition_in_progress_{false};
  bool lift_brake_locked_{true};
  bool brake_lock_commanded_{false};
  bool brake_lock_pending_{false};
  bool brake_lock_force_{false};
  bool brake_transition_in_progress_{false};
  std::uint64_t brake_transition_count_{0};
  std::string pending_brake_reason_;
  std::string last_brake_reason_{"startup: brake state awaiting feedback"};
  std::chrono::steady_clock::time_point last_physical_brake_time_{};
  std::string pending_lease_clear_reason_;
  std::string last_lease_clear_reason_{"none"};
  std::chrono::steady_clock::time_point lease_deadline_{};
  std::chrono::steady_clock::time_point & last_command_time_{
    lease_.last_accepted_command_time};
  std::chrono::steady_clock::time_point pending_command_time_{};
  std::chrono::steady_clock::time_point last_feedback_publish_time_{};
  std::chrono::steady_clock::time_point last_status_publish_time_{};
  double measured_feedback_rate_hz_{0.0};
  std::uint64_t accepted_commands_{0};
  std::uint64_t rejected_commands_{0};
  std::uint64_t duplicate_commands_{0};
  std::uint64_t out_of_order_commands_{0};
  std::uint64_t command_watchdog_hold_count_{0};
  std::uint64_t lease_expiry_count_{0};
  double last_command_header_delay_ms_{kNoCommandAgeMs};
  double last_controller_ack_delay_ms_{kNoCommandAgeMs};
  double last_controller_publish_latency_ms_{kNoCommandAgeMs};
  std::string last_controller_submission_result_{"no command submitted"};
  std::uint64_t received_commands_{0};
  std::unordered_map<std::string, std::uint64_t> command_rejection_reasons_;
  std::string last_command_rejection_reason_;
  std::chrono::steady_clock::time_point last_command_diagnostic_time_{};

  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_publication_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_publication_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Publisher<Command>::SharedPtr internal_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lease_gate_publisher_;
  rclcpp::Subscription<Command>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lift_joint_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmPowerStatus>::SharedPtr arm_power_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::LiftStatus>::SharedPtr lift_status_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::WorkspaceStatus>::SharedPtr workspace_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmControlModeStatus>::SharedPtr
    control_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr arm_ack_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr lift_ack_subscription_;
  rclcpp::Service<Capabilities>::SharedPtr capabilities_service_;
  rclcpp::Service<AcquireLease>::SharedPtr acquire_service_;
  rclcpp::Service<ReleaseLease>::SharedPtr release_service_;
  rclcpp::Service<LiftBrake>::SharedPtr lift_brake_service_;
  rclcpp::Client<robot_control_msg::srv::SetRobotPower>::SharedPtr arm_power_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heavy_lift_brake_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__HEAVY_GATEWAY_V1_HPP_

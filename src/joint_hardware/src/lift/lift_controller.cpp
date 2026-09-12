#include "joint_hardware/lift_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace joint_hardware
{
namespace
{

bool json_bool(const std::string & text, const char * key)
{
  return text.find(std::string("\"") + key + "\":true") != std::string::npos;
}

long json_integer(const std::string & text, const char * key, long fallback = 0)
{
  const std::string prefix = std::string("\"") + key + "\":";
  const auto begin = text.find(prefix);
  if (begin == std::string::npos) {
    return fallback;
  }
  try {
    std::size_t used = 0;
    return std::stol(text.substr(begin + prefix.size()), &used, 0);
  } catch (...) {
    return fallback;
  }
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

// Diagnostics helpers. Fault details can contain commas and brackets, so they
// must be quoted before they are embedded in the status JSON.
std::string json_quote(const std::string & text)
{
  std::ostringstream quoted;
  quoted << '"';
  for (const char character : text) {
    switch (character) {
      case '"': quoted << "\\\""; break;
      case '\\': quoted << "\\\\"; break;
      case '\n': quoted << "\\n"; break;
      case '\r': quoted << "\\r"; break;
      case '\t': quoted << "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          quoted << ' ';
        } else {
          quoted << character;
        }
        break;
    }
  }
  quoted << '"';
  return quoted.str();
}

int64_t now_steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

controller_interface::CallbackReturn LiftController::on_init()
{
  try {
    auto_declare<std::string>("joint", "joint_motor");
    auto_declare<double>("position_min_m", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("position_max_m", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("max_velocity_mps", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("max_acceleration_mps2", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("max_jerk_mps3", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("default_velocity_scale", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("jog_timeout_sec", 0.45);
    auto_declare<double>("brake_gate_stable_sec", 0.1);
    auto_declare<double>("driver_status_timeout_sec", 2.0);
    auto_declare<double>("target_stable_sec", 1.0);
    auto_declare<double>("goal_tolerance_m", 0.001);
    auto_declare<double>("stationary_velocity_mps", 0.001);
    auto_declare<double>("status_rate_hz", 20.0);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Lift controller parameter declaration failed: %s",
      exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LiftController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {
    joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
    joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    joint_name_ + "/" + hardware_interface::HW_IF_ACCELERATION,
    joint_name_ + "/power_enable",
  };
  return configuration;
}

controller_interface::InterfaceConfiguration LiftController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {
    joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
    joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    joint_name_ + "/power_enable",
  };
  return configuration;
}

controller_interface::CallbackReturn LiftController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_name_ = get_node()->get_parameter("joint").as_string();
  limits_.min_position_m = get_node()->get_parameter("position_min_m").as_double();
  limits_.max_position_m = get_node()->get_parameter("position_max_m").as_double();
  limits_.max_velocity_mps = get_node()->get_parameter("max_velocity_mps").as_double();
  limits_.max_acceleration_mps2 =
    get_node()->get_parameter("max_acceleration_mps2").as_double();
  limits_.max_jerk_mps3 = get_node()->get_parameter("max_jerk_mps3").as_double();
  limits_.default_velocity_scale =
    get_node()->get_parameter("default_velocity_scale").as_double();
  jog_timeout_sec_ = get_node()->get_parameter("jog_timeout_sec").as_double();
  brake_gate_stable_sec_ = get_node()->get_parameter("brake_gate_stable_sec").as_double();
  driver_status_timeout_sec_ =
    get_node()->get_parameter("driver_status_timeout_sec").as_double();
  target_stable_sec_ = get_node()->get_parameter("target_stable_sec").as_double();
  goal_tolerance_m_ = get_node()->get_parameter("goal_tolerance_m").as_double();
  stationary_velocity_mps_ =
    get_node()->get_parameter("stationary_velocity_mps").as_double();
  status_rate_hz_ = get_node()->get_parameter("status_rate_hz").as_double();

  std::string error;
  if (joint_name_ != "joint_motor" || !profile_.configure(limits_, error) ||
    !std::isfinite(jog_timeout_sec_) || jog_timeout_sec_ <= 0.0 ||
    !std::isfinite(brake_gate_stable_sec_) || brake_gate_stable_sec_ < 0.1 ||
    !std::isfinite(driver_status_timeout_sec_) || driver_status_timeout_sec_ <= 0.0 ||
    !std::isfinite(target_stable_sec_) || target_stable_sec_ < 0.0 ||
    !std::isfinite(goal_tolerance_m_) || goal_tolerance_m_ <= 0.0 ||
    !std::isfinite(stationary_velocity_mps_) || stationary_velocity_mps_ <= 0.0 ||
    !std::isfinite(status_rate_hz_) || status_rate_hz_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid lift controller configuration: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  status_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
    status_topic_, rclcpp::QoS(1).best_effort());
  status_timer_ = get_node()->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / status_rate_hz_)),
    [this]() {publish_status();});
  driver_status_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
    driver_status_topic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr message) {
      if (message) {
        on_driver_status(*message);
      }
    });
  jog_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    jog_topic_, rclcpp::QoS(1).best_effort(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      if (!message || !std::isfinite(message->data) || driver_estop_latched_.load() ||
      heavy_lease_active_.load(std::memory_order_acquire))
      {
        return;
      }
      MotionCommand command;
      command.type = std::abs(message->data) <= 1.0e-9 ?
      CommandType::soft_stop : CommandType::jog;
      command.velocity_mps = std::clamp(
        message->data, -limits_.max_velocity_mps, limits_.max_velocity_mps);
      command.velocity_scale = 1.0;
      enqueue_command(command);
    });
  stream_subscription_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    stream_topic_, rclcpp::QoS(1).best_effort(),
    [this](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr message) {
      if (!message || message->positions.empty() ||
      !std::isfinite(message->positions[0]) || driver_estop_latched_.load() ||
      heavy_lease_active_.load(std::memory_order_acquire))
      {
        return;
      }
      MotionCommand command;
      command.type = CommandType::stream;
      command.position_m = std::clamp(
        message->positions[0], limits_.min_position_m, limits_.max_position_m);
      command.velocity_mps = !message->velocities.empty() &&
      std::isfinite(message->velocities[0]) ? message->velocities[0] : 0.0;
      command.acceleration_mps2 = !message->accelerations.empty() &&
      std::isfinite(message->accelerations[0]) ? message->accelerations[0] : 0.0;
      command.velocity_scale = limits_.default_velocity_scale;
      enqueue_command(command);
    });
  trajectory_subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    trajectory_topic_, rclcpp::QoS(1).reliable(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
      if (!message || message->joint_names.size() != 1 ||
      message->joint_names[0] != joint_name_ || message->points.empty() ||
      driver_estop_latched_.load() || heavy_lease_active_.load(std::memory_order_acquire))
      {
        return;
      }
      for (const auto & point : message->points) {
        if (point.positions.size() != 1 || !std::isfinite(point.positions[0]) ||
        point.positions[0] < limits_.min_position_m ||
        point.positions[0] > limits_.max_position_m ||
        (!point.velocities.empty() &&
        (point.velocities.size() != 1 || !std::isfinite(point.velocities[0]))) ||
        (!point.accelerations.empty() &&
        (point.accelerations.size() != 1 || !std::isfinite(point.accelerations[0]))))
        {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected invalid lift JointTrajectory");
          return;
        }
      }
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      pending_trajectory_ = message;
      pending_trajectory_generation_ = next_motion_generation();
    });

  command_service_ = get_node()->create_service<robot_control_msg::srv::SelectedJointControl>(
    command_service_name_,
    [this](
      const std::shared_ptr<robot_control_msg::srv::SelectedJointControl::Request> request,
      std::shared_ptr<robot_control_msg::srv::SelectedJointControl::Response> response)
    {
      if (heavy_lease_active_.load(std::memory_order_acquire)) {
        response->accepted = false;
        response->message = "lift command rejected: Heavy V1 execution lease is active";
        return;
      }
      if (driver_estop_latched_.load()) {
        response->accepted = false;
        response->message = "lift command rejected: emergency stop is latched";
        return;
      }
      if (request->joint_names.size() != 1 || request->joint_names[0] != joint_name_ ||
      request->values.size() != 1 || !std::isfinite(request->values[0]) ||
      !std::isfinite(request->vel) || !std::isfinite(request->acc))
      {
        response->accepted = false;
        response->message = "expected one finite joint_motor target";
        return;
      }
      MotionCommand command;
      command.type = CommandType::position;
      const double requested_target = request->relative ?
      measured_position_atomic_.load(std::memory_order_acquire) + request->values[0] :
      request->values[0];
      command.position_m = std::clamp(
        requested_target, limits_.min_position_m, limits_.max_position_m);
      command.velocity_scale = request->vel > 0.0 && std::isfinite(request->vel) ?
      std::clamp(request->vel / limits_.max_velocity_mps, 0.01, 1.0) :
      limits_.default_velocity_scale;
      command.acceleration_limit_mps2 = request->acc > 0.0 ?
      std::min(request->acc, limits_.max_acceleration_mps2) :
      limits_.max_acceleration_mps2;
      enqueue_command(command);
      response->accepted = true;
      response->message = command.position_m == requested_target ?
      "lift command accepted" : "lift command clamped to software travel";
    });

  rclcpp::QoS heavy_command_qos(rclcpp::KeepLast(1));
  heavy_command_qos.reliable().durability_volatile();
  heavy_command_subscription_ = get_node()->create_subscription<
    robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1>(
    "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command", heavy_command_qos,
    [this](const robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::SharedPtr message) {
      if (!message) {
        return;
      }
      uint64_t admitted = admitted_heavy_sequence_.load(std::memory_order_acquire);
      while (message->sequence > admitted &&
        !admitted_heavy_sequence_.compare_exchange_weak(
          admitted, message->sequence, std::memory_order_acq_rel, std::memory_order_acquire))
      {
      }
      if (message->sequence <= admitted) {
        if (message->sequence == applied_heavy_sequence_.load(std::memory_order_acquire) &&
          heavy_ack_publisher_)
        {
          // The gateway may retransmit the current generation after an ACK
          // delivery gap. Re-ACK only work RT has already applied.
          std_msgs::msg::UInt64 ack;
          ack.data = message->sequence;
          heavy_ack_publisher_->publish(ack);
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Ignored duplicate or out-of-order Heavy V1 lift sequence");
        {
          std::ostringstream detail;
          detail << "received_sequence=" << message->sequence
                 << " admitted_sequence=" << admitted_heavy_sequence_.load(
            std::memory_order_acquire)
                 << " applied_sequence=" << applied_heavy_sequence_.load(
            std::memory_order_acquire);
          log_command_rejection("heavy_sequence_duplicate_or_out_of_order", detail.str());
        }
        return;
      }
      MotionCommand command;
      command.heavy_sequence = message->sequence;
      if (message->mode ==
      robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_FOLLOW_POSITION)
      {
        const bool position_provided = (message->field_mask &
        robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_POSITION) != 0U;
        const bool velocity_provided = (message->field_mask &
        robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_VELOCITY) != 0U;
        const bool acceleration_provided = (message->field_mask &
        robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_ACCELERATION) != 0U;
        const double velocity = velocity_provided ? message->velocity[0] : 0.0;
        const double acceleration = acceleration_provided ? message->acceleration[0] : 0.0;
        if (!position_provided || !std::isfinite(message->position[0]) ||
        message->position[0] < limits_.min_position_m ||
        message->position[0] > limits_.max_position_m ||
        !std::isfinite(velocity) || std::abs(velocity) > limits_.max_velocity_mps ||
        !std::isfinite(acceleration) || std::abs(acceleration) > limits_.max_acceleration_mps2)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected invalid Heavy V1 lift target");
          std::ostringstream detail;
          detail << "sequence=" << message->sequence
                 << " field_mask=" << static_cast<unsigned int>(message->field_mask)
                 << " position=" << message->position[0]
                 << " velocity=" << velocity
                 << " acceleration=" << acceleration
                 << " limits[min_position_m=" << limits_.min_position_m
                 << ",max_position_m=" << limits_.max_position_m
                 << ",max_velocity_mps=" << limits_.max_velocity_mps
                 << ",max_acceleration_mps2=" << limits_.max_acceleration_mps2 << ']';
          log_command_rejection("heavy_target_out_of_limits", detail.str());
          return;
        }
        command.type = CommandType::stream;
        command.position_m = message->position[0];
        command.velocity_mps = velocity;
        command.acceleration_mps2 = acceleration;
        command.velocity_scale = limits_.default_velocity_scale;
      } else if (message->mode ==
      robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_STOP)
      {
        command.type = CommandType::soft_stop;
      } else if (message->mode ==
        robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_HOLD) {
        command.type = CommandType::hold;
        if ((message->field_mask &
          robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_POSITION) == 0U ||
          !std::isfinite(message->position[0]))
        {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected invalid Heavy V1 lift HOLD target");
          return;
        }
        // HOLD targets are snapshots of measured feedback.  Quantization and
        // calibration can put that snapshot a fraction outside the software
        // travel range (for example +0.00047 m at the 0.0 m upper limit).
        // Clamp it like the native lift command path instead of dropping the
        // command: a dropped HOLD cannot produce an apply acknowledgement and
        // makes the upper controller report controller_apply_ack_timeout.
        command.position_m = std::clamp(
          message->position[0], limits_.min_position_m, limits_.max_position_m);
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Rejected unknown Heavy V1 lift command mode");
        return;
      }
      command.generation = next_motion_generation();
      heavy_command_generation_.store(command.generation, std::memory_order_release);
      heavy_command_buffer_.writeFromNonRT(command);
    });
  rclcpp::QoS heavy_gate_qos(rclcpp::KeepLast(1));
  heavy_gate_qos.reliable().transient_local();
  heavy_lease_gate_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/ubuntu_lower_gateway/internal/heavy/v1/lease_active", heavy_gate_qos,
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      if (message) {
        heavy_lease_active_.store(message->data, std::memory_order_release);
        if (!message->data) {
          // Internal controller generations are scoped to the gateway
          // process. A gateway restart publishes a retained false gate before
          // granting a new lease, so an old high-water mark must not reject
          // the restarted gateway's generation 1 as out-of-order.
          admitted_heavy_sequence_.store(0, std::memory_order_release);
          applied_heavy_sequence_.store(0, std::memory_order_release);
        }
      }
    });
  heavy_ack_publisher_ = get_node()->create_publisher<std_msgs::msg::UInt64>(
    "/ubuntu_lower_gateway/internal/heavy/v1/lift_applied_sequence",
    rclcpp::QoS(10).reliable());
  stop_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/joint/lift/stop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      MotionCommand command;
      command.type = CommandType::soft_stop;
      const uint64_t generation = enqueue_safety_command(command);
      if (!active_.load(std::memory_order_acquire)) {
        response->success = true;
        response->message = "stage=inactive: lift controller is stopped; STOP is already fail-safe";
        return;
      }
      if (hold_snapshot_is_safe() &&
      !power_enable_requested_.load(std::memory_order_acquire))
      {
        applied_cancellation_generation_.store(generation, std::memory_order_release);
        response->success = true;
        response->message = "stage=idempotent: lift already stopped; old trajectory invalidated";
        return;
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (std::chrono::steady_clock::now() < deadline) {
        if (applied_cancellation_generation_.load(std::memory_order_acquire) >= generation) {
          response->success = true;
          response->message = "stage=cancellation: lift STOP committed; old trajectory invalidated";
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      response->success = false;
      response->message = "stage=cancellation: timed out waiting for control-loop acknowledgement";
    });
  hold_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/joint/lift/hold",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load(std::memory_order_acquire)) {
        response->success = true;
        response->message = "stage=inactive: lift controller is stopped; HOLD is already fail-safe";
        return;
      }
      MotionCommand command;
      command.type = CommandType::hold;
      command.position_m = measured_position_atomic_.load(std::memory_order_acquire);
      const uint64_t generation = enqueue_safety_command(command);
      if (hold_snapshot_is_safe()) {
        applied_cancellation_generation_.store(
          generation,
          std::memory_order_release);
        response->success = true;
        response->message = "stage=idempotent: lift already in safe HOLD; old trajectory invalidated";
        return;
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (std::chrono::steady_clock::now() < deadline) {
        if (completed_hold_generation_.load(std::memory_order_acquire) >= generation) {
          response->success = true;
          response->message = "stage=mode_confirmation: lift HOLD confirmed";
          return;
        }
        if (failed_hold_generation_.load(std::memory_order_acquire) >= generation) {
          const auto stage = static_cast<HoldFailureStage>(
            hold_failure_stage_.load(std::memory_order_acquire));
          response->success = false;
          response->message = std::string("stage=") + hold_failure_stage_name(stage) +
          ": lift HOLD failed";
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      response->success = false;
      response->message = "stage=mode_confirmation: timed out waiting for confirmed HOLD";
    });
  MotionCommand empty;
  command_buffer_.writeFromNonRT(empty);
  safety_command_buffer_.writeFromNonRT(empty);
  heavy_command_buffer_.writeFromNonRT(empty);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[LIFT_CFG] configured unified lift controller joint=%s "
    "v=%.3f m/s a=%.3f m/s^2 j=%.3f m/s^3 scale=%.2f "
    "travel=[%.3f,%.3f] m jog_timeout=%.3f s brake_gate_stable=%.3f s "
    "driver_status_timeout=%.3f s goal_tolerance=%.4f m stationary_velocity=%.4f m/s",
    joint_name_.c_str(),
    limits_.max_velocity_mps, limits_.max_acceleration_mps2,
    limits_.max_jerk_mps3, limits_.default_velocity_scale,
    limits_.min_position_m, limits_.max_position_m,
    jog_timeout_sec_, brake_gate_stable_sec_, driver_status_timeout_sec_,
    goal_tolerance_m_, stationary_velocity_mps_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 4 || state_interfaces_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Lift controller interface count mismatch");
    return controller_interface::CallbackReturn::ERROR;
  }
  measured_position_m_ = state_interfaces_[position_state_index_].get_value();
  measured_velocity_mps_ = state_interfaces_[velocity_state_index_].get_value();
  const double power_enabled = state_interfaces_[power_enable_state_index_].get_value();
  if (!std::isfinite(measured_position_m_) || !std::isfinite(measured_velocity_mps_) ||
    !std::isfinite(power_enabled))
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  power_enabled_state_.store(power_enabled >= 0.5, std::memory_order_release);
  power_enable_requested_.store(false, std::memory_order_release);
  profile_.reset(measured_position_m_, 0.0, 0.0);
  measured_position_atomic_.store(measured_position_m_, std::memory_order_release);
  measured_velocity_atomic_.store(measured_velocity_mps_, std::memory_order_release);
  command_sample_ = profile_.state();
  write_command_interfaces(command_sample_);
  consumed_generation_ = command_generation_.load();
  consumed_safety_command_generation_ = safety_command_generation_.load();
  consumed_heavy_command_generation_ = heavy_command_generation_.load();
  heavy_lease_active_rt_ = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    pending_trajectory_.reset();
    active_trajectory_.reset();
    consumed_trajectory_generation_ = pending_trajectory_generation_;
    active_trajectory_generation_ = 0;
  }
  gated_command_valid_ = false;
  const auto previous_mode = static_cast<Mode>(mode_atomic_.load(std::memory_order_acquire));
  mode_ = Mode::hold;
  // Re-activation is the only path out of a latched Mode::fault, so report the
  // latch that this activation just cleared and re-arm the diagnostics window.
  if (fault_total_ > 0U || previous_mode == Mode::fault) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[LIFT_FAULT] state=cleared_by_activate previous_mode=%s total_faults=%llu "
      "last_code=%s last_fault_age_ms=%.1f",
      mode_name(previous_mode), static_cast<unsigned long long>(fault_total_),
      last_fault_code_[0] != '\0' ? last_fault_code_ : "none",
      last_fault_line_ns_ > 0 ? static_cast<double>(now_steady_ns() - last_fault_line_ns_) * 1e-6 :
      -1.0);
  }
  last_fault_line_ns_ = 0;
  last_fault_record_ns_ = 0;
  fault_line_suppressed_ = 0;
  last_fault_code_[0] = '\0';
  last_gate_debug_ns_ = 0;
  last_gate_warn_ns_ = 0;
  gate_unready_since_ns_ = 0;
  gate_warn_emitted_ = false;
  soft_stop_completion_ = SoftStopCompletion::disable;
  pending_hold_generation_ = 0;
  cancellation_generation_.store(motion_generation_.load(), std::memory_order_release);
  applied_cancellation_generation_.store(motion_generation_.load(), std::memory_order_release);
  status_message_ = "HOLD current measured position";
  active_.store(true, std::memory_order_release);
  goal_stable_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  heavy_lease_active_.store(false, std::memory_order_release);
  consumed_heavy_command_generation_ = heavy_command_generation_.load();
  heavy_lease_active_rt_ = false;
  request_brake(false);
  if (state_interfaces_.size() >= 3 && command_interfaces_.size() >= 4) {
    measured_position_m_ = state_interfaces_[position_state_index_].get_value();
    profile_.hold(std::isfinite(measured_position_m_) ? measured_position_m_ : 0.0);
    write_command_interfaces(profile_.state());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LiftController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  measured_position_m_ = state_interfaces_[position_state_index_].get_value();
  measured_velocity_mps_ = state_interfaces_[velocity_state_index_].get_value();
  const double power_enabled = state_interfaces_[power_enable_state_index_].get_value();
  measured_position_atomic_.store(measured_position_m_, std::memory_order_release);
  measured_velocity_atomic_.store(measured_velocity_mps_, std::memory_order_release);
  power_enabled_state_.store(
    std::isfinite(power_enabled) && power_enabled >= 0.5, std::memory_order_release);
  if (!std::isfinite(measured_position_m_) || !std::isfinite(measured_velocity_mps_) ||
    !std::isfinite(power_enabled))
  {
    record_lift_fault("non_finite_state_interfaces", [this, power_enabled]() {
      std::ostringstream detail;
      detail << "entry_mode=" << mode_name(mode_)
             << " position=" << measured_position_m_
             << " velocity=" << measured_velocity_mps_
             << " power_enable=" << power_enabled;
      return detail.str();
    });
    fail_pending_hold(HoldFailureStage::motion_profile);
    mode_ = Mode::fault;
    request_brake(false);
    command_sample_ = {0.0, 0.0, 0.0, 0.0, true, false};
    write_command_interfaces(command_sample_);
    return controller_interface::return_type::ERROR;
  }
  const double dt = period.seconds() > 0.0 ? period.seconds() : 0.01;
  const auto now = std::chrono::steady_clock::now();

  if (driver_estop_latched_.load()) {
    mode_ = Mode::estop;
    gated_command_valid_ = false;
    active_trajectory_.reset();
    {
      std::unique_lock<std::mutex> lock(trajectory_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        pending_trajectory_.reset();
        consumed_trajectory_generation_ = pending_trajectory_generation_;
      }
    }
    consumed_generation_ = command_generation_.load();
    profile_.hold(measured_position_m_);
    command_sample_ = profile_.state();
    request_brake(false);
    write_command_interfaces(command_sample_);
    status_message_ = "emergency stop latched";
    return controller_interface::return_type::OK;
  }
  if (mode_ == Mode::estop) {
    // A successful safety reset never resumes the pre-estop command. The
    // hardware remains disabled and the controller waits in HOLD for a new
    // command, which will run through the brake gate again.
    consumed_generation_ = command_generation_.load();
    enter_hold(measured_position_m_, "safety reset complete; waiting for a new command");
  }

  const bool heavy_lease_active = heavy_lease_active_.load(std::memory_order_acquire);
  if (!heavy_lease_active && heavy_lease_active_rt_) {
    consumed_heavy_command_generation_ = heavy_command_generation_.load(
      std::memory_order_acquire);
  }
  heavy_lease_active_rt_ = heavy_lease_active;
  trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory;
  if (!heavy_lease_active) {
    std::unique_lock<std::mutex> lock(trajectory_mutex_, std::try_to_lock);
    if (lock.owns_lock() && pending_trajectory_ &&
      consumed_trajectory_generation_ != pending_trajectory_generation_ &&
      pending_trajectory_generation_ > cancellation_generation_.load(std::memory_order_acquire))
    {
      trajectory = pending_trajectory_;
      consumed_trajectory_generation_ = pending_trajectory_generation_;
      active_trajectory_generation_ = pending_trajectory_generation_;
    }
  } else {
    consumed_generation_ = command_generation_.load(std::memory_order_acquire);
    active_trajectory_.reset();
    active_trajectory_generation_ = 0;
    std::unique_lock<std::mutex> lock(trajectory_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      pending_trajectory_.reset();
      consumed_trajectory_generation_ = pending_trajectory_generation_;
    }
  }
  if (trajectory) {
    active_trajectory_ = trajectory;
    trajectory_point_index_ = 0;
    trajectory_elapsed_sec_ = 0.0;
    trajectory_waiting_at_point_ = false;
    MotionCommand command;
    command.type = CommandType::trajectory;
    begin_brake_gate(command);
  }

  const MotionCommand * buffered = nullptr;
  MotionCommand safety_command;
  bool safety_command_consumed = false;
  MotionCommand heavy_command;
  bool heavy_command_consumed = false;
  bool heavy_stream_sample_selected = false;
  const auto safety_generation = safety_command_generation_.load(std::memory_order_acquire);
  if (safety_generation != consumed_safety_command_generation_) {
    const MotionCommand * pending_safety = safety_command_buffer_.readFromRT();
    if (pending_safety) {
      safety_command = *pending_safety;
      buffered = &safety_command;
      consumed_safety_command_generation_ = safety_generation;
      safety_command_consumed = true;
    }
  } else if (heavy_lease_active) {
    const auto heavy_generation = heavy_command_generation_.load(std::memory_order_acquire);
    if (heavy_generation != consumed_heavy_command_generation_) {
      const MotionCommand * pending_heavy = heavy_command_buffer_.readFromRT();
      if (pending_heavy) {
        heavy_command = *pending_heavy;
        buffered = &heavy_command;
        consumed_heavy_command_generation_ = heavy_generation;
        heavy_command_consumed = true;
      }
    }
  } else {
    buffered = command_buffer_.readFromRT();
  }
  const bool buffered_is_new = buffered && buffered->generation != 0 &&
    (safety_command_consumed || buffered->generation >
    cancellation_generation_.load(std::memory_order_acquire)) &&
    (safety_command_consumed || heavy_command_consumed ||
    buffered->generation != consumed_generation_);
  if (buffered_is_new) {
    const MotionCommand command = *buffered;
    if (!heavy_command_consumed && !safety_command_consumed) {
      consumed_generation_ = command.generation;
    }
    if (command.type == CommandType::soft_stop || command.type == CommandType::hold) {
      if (safety_command_consumed) {
        cancel_pending_trajectory(command.generation);
        active_trajectory_.reset();
        active_trajectory_generation_ = 0;
        applied_cancellation_generation_.store(command.generation, std::memory_order_release);
      }
      if ((command.type == CommandType::hold || command.type == CommandType::soft_stop) &&
        mode_ == Mode::hold &&
        std::abs(measured_velocity_mps_) <= stationary_velocity_mps_ &&
        std::abs(command_sample_.velocity_mps) <= stationary_velocity_mps_)
      {
        if (command.type == CommandType::hold) {
          pending_hold_generation_ = safety_command_consumed ? command.generation : 0;
          const double hold_position = safety_command_consumed ? measured_position_m_ :
            command.position_m;
          enter_hold(hold_position, "HOLD confirmed at current measured position");
        } else {
          enter_hold(measured_position_m_, "STOP confirmed while already stationary");
          request_brake(false);
        }
      } else {
        begin_soft_stop(
          command.type == CommandType::hold ? "HOLD requested" : "STOP requested",
          command.type == CommandType::hold ? SoftStopCompletion::hold_powered :
          SoftStopCompletion::disable,
          command.type == CommandType::hold && safety_command_consumed ? command.generation : 0);
      }
    } else if (mode_ == Mode::brake_gate && command.type != CommandType::none) {
      // Jog and stream publishers send keepalive samples faster than the
      // brake gate's 100 ms stability window.  Updating the pending command
      // must not restart that window, otherwise a 50 Hz Jog can never reach
      // Operation enabled and no non-zero velocity is emitted.
      gated_command_ = command;
      gated_command_valid_ = true;
      if (command.type == CommandType::jog) {
        last_jog_command_time_ = now;
      }
    } else if (command.type == CommandType::jog && mode_ == Mode::jog && driver_gate_ready()) {
      if (!profile_.set_velocity_target(command.velocity_mps, command.velocity_scale)) {
        record_lift_fault("invalid_jog_velocity", [this, &command]() {
          std::ostringstream detail;
          detail << "entry_mode=" << mode_name(mode_)
                 << " jog_velocity_mps=" << command.velocity_mps
                 << " requested_velocity_scale=" << command.velocity_scale
                 << " limits[max_velocity_mps=" << limits_.max_velocity_mps
                 << ",max_acceleration_mps2=" << limits_.max_acceleration_mps2
                 << ",max_jerk_mps3=" << limits_.max_jerk_mps3 << ']';
          return detail.str();
        });
        mode_ = Mode::fault;
        profile_.hold(measured_position_m_);
        request_brake(false);
        status_message_ = "invalid Jog velocity command";
      } else {
        last_jog_command_time_ = now;
      }
    } else if (command.type == CommandType::stream && mode_ == Mode::streaming &&
      driver_gate_ready())
    {
      // Heavy FOLLOW is an upstream time-parameterized servo sample, not a
      // new point-to-point target. Do not reset or advance the OTG here.
      command_sample_ = {
        command.position_m, command.velocity_mps, command.acceleration_mps2,
        0.0, false, true};
      heavy_stream_sample_selected = true;
      status_message_ = "Heavy external streaming setpoint active";
    } else if (command.type != CommandType::none) {
      active_trajectory_.reset();
      active_trajectory_generation_ = 0;
      begin_brake_gate(command);
    }
  }

  if (mode_ != Mode::hold && mode_ != Mode::brake_gate && mode_ != Mode::estop &&
    !driver_gate_ready())
  {
    record_lift_fault("gate_lost_during_motion", [this]() {
      std::ostringstream detail;
      detail << "entry_mode=" << mode_name(mode_)
             << " position=" << measured_position_m_
             << " velocity=" << measured_velocity_mps_
             << " command_position=" << command_sample_.position_m
             << " command_velocity=" << command_sample_.velocity_mps
             << " gate[" << driver_gate_failure_reason() << ']';
      return detail.str();
    });
    fail_pending_hold(HoldFailureStage::driver_gate);
    mode_ = Mode::fault;
    gated_command_valid_ = false;
    active_trajectory_.reset();
    profile_.hold(measured_position_m_);
    request_brake(false);
    status_message_ = "driver feedback/state invalid during motion";
  }

  if (mode_ == Mode::brake_gate) {
    const bool jog_gate_timed_out = gated_command_valid_ &&
      gated_command_.type == CommandType::jog &&
      last_jog_command_time_.time_since_epoch().count() != 0 &&
      now - last_jog_command_time_ > std::chrono::duration<double>(jog_timeout_sec_);
    if (jog_gate_timed_out) {
      begin_soft_stop("Jog keepalive timeout during brake gate");
      command_sample_ = profile_.update(dt);
    } else if (driver_gate_ready()) {
      if (gate_stable_since_.time_since_epoch().count() == 0) {
        gate_stable_since_ = now;
      } else if (now - gate_stable_since_ >=
        std::chrono::duration<double>(brake_gate_stable_sec_))
      {
        start_gated_motion();
        heavy_stream_sample_selected = mode_ == Mode::streaming;
      }
    } else {
      gate_stable_since_ = std::chrono::steady_clock::time_point{};
      request_brake(true);
    }
    if (mode_ == Mode::brake_gate) {
      command_sample_ = profile_.state();
    }
  } else if (mode_ == Mode::jog &&
    now - last_jog_command_time_ > std::chrono::duration<double>(jog_timeout_sec_))
  {
    begin_soft_stop("Jog keepalive timeout");
    command_sample_ = profile_.update(dt);
  } else if (mode_ == Mode::trajectory && active_trajectory_) {
    trajectory_elapsed_sec_ += dt;
    command_sample_ = profile_.update(dt);
    if (command_sample_.finished) {
      const double point_time = duration_seconds(
        active_trajectory_->points[trajectory_point_index_].time_from_start);
      if (trajectory_elapsed_sec_ + 1.0e-9 >= point_time) {
        if (trajectory_point_index_ + 1 < active_trajectory_->points.size()) {
          ++trajectory_point_index_;
          if (!start_trajectory_point(trajectory_point_index_)) {
            begin_soft_stop("invalid trajectory point");
          }
        } else {
          trajectory_waiting_at_point_ = true;
        }
      }
    }
  } else if (mode_ == Mode::position || mode_ == Mode::jog ||
    mode_ == Mode::soft_stop)
  {
    command_sample_ = profile_.update(dt);
  } else if (mode_ == Mode::streaming) {
    // command_sample_ is the latest upstream time-parameterized Heavy sample.
    // Do not replace it with profile_.state(), which still contains the HOLD
    // state captured before streaming began.
  } else {
    command_sample_ = profile_.state();
  }

  if (!command_sample_.valid) {
    record_lift_fault("ruckig_rejected_state", [this]() {
      const auto last_sample = profile_.state();
      std::ostringstream detail;
      detail << "entry_mode=" << mode_name(mode_)
             << " position=" << measured_position_m_
             << " velocity=" << measured_velocity_mps_
             << " profile_position=" << last_sample.position_m
             << " profile_velocity=" << last_sample.velocity_mps
             << " profile_acceleration=" << last_sample.acceleration_mps2
             << " profile_jerk=" << last_sample.jerk_mps3
             << " limits[max_velocity_mps=" << limits_.max_velocity_mps
             << ",max_acceleration_mps2=" << limits_.max_acceleration_mps2
             << ",max_jerk_mps3=" << limits_.max_jerk_mps3 << ']';
      return detail.str();
    });
    fail_pending_hold(HoldFailureStage::motion_profile);
    mode_ = Mode::fault;
    profile_.hold(measured_position_m_);
    command_sample_ = profile_.state();
    request_brake(false);
    status_message_ = "Ruckig rejected the current motion state";
  }

  if (mode_ == Mode::soft_stop &&
    command_sample_.finished &&
    std::abs(command_sample_.velocity_mps) <= stationary_velocity_mps_ &&
    std::abs(command_sample_.acceleration_mps2) <= 1.0e-9)
  {
    enter_hold(measured_position_m_, "soft stop complete");
    if (soft_stop_completion_ == SoftStopCompletion::disable) {
      request_brake(false);
    }
  }

  const bool terminal_mode = mode_ == Mode::position ||
    (mode_ == Mode::trajectory && trajectory_waiting_at_point_);
  // command_sample_ is an intermediate Ruckig sample.  At low commanded
  // velocities it can remain within the goal tolerance of a stationary axis
  // for long enough to look "stable", even though the final target is still
  // several millimetres away.  Only the profile's actual target may complete
  // a position move or the final trajectory point.
  const double terminal_target = profile_.target_position_m();
  const bool stable = terminal_mode &&
    std::abs(terminal_target - measured_position_m_) <= goal_tolerance_m_ &&
    std::abs(measured_velocity_mps_) <= stationary_velocity_mps_ &&
    std::abs(command_sample_.velocity_mps) <= stationary_velocity_mps_;
  if (stable) {
    if (!goal_stable_active_) {
      goal_stable_active_ = true;
      goal_stable_since_ = now;
    } else if (now - goal_stable_since_ >= std::chrono::duration<double>(target_stable_sec_)) {
      active_trajectory_.reset();
      enter_hold(terminal_target, "target stable; servo HOLD active");
    }
  } else {
    goal_stable_active_ = false;
  }

  // Categorized gate diagnostics: DEBUG while the gate is degraded, promoted to
  // one WARN per window once the failure is sustained, and one INFO on
  // recovery. A latched fault is skipped here because the [LIFT_FAULT] record
  // already carries the same breakdown.
  log_gate_diagnostics("update");
  write_command_interfaces(command_sample_);
  confirm_pending_hold();
  if (heavy_command_consumed && heavy_command.heavy_sequence != 0) {
    if (heavy_command.type == CommandType::stream) {
      pending_heavy_follow_ack_sequence_ = heavy_command.heavy_sequence;
    } else if (heavy_ack_publisher_) {
      std_msgs::msg::UInt64 ack;
      ack.data = heavy_command.heavy_sequence;
      applied_heavy_sequence_.store(ack.data, std::memory_order_release);
      heavy_ack_publisher_->publish(ack);
    }
  }
  if (pending_heavy_follow_ack_sequence_ != 0 && mode_ == Mode::streaming &&
    driver_gate_ready() && heavy_stream_sample_selected && heavy_ack_publisher_)
  {
    std_msgs::msg::UInt64 ack;
    ack.data = pending_heavy_follow_ack_sequence_;
    applied_heavy_sequence_.store(ack.data, std::memory_order_release);
    heavy_ack_publisher_->publish(ack);
    pending_heavy_follow_ack_sequence_ = 0;
  }
  return controller_interface::return_type::OK;
}

uint64_t LiftController::enqueue_command(MotionCommand command)
{
  command.generation = next_motion_generation();
  command_generation_.store(command.generation, std::memory_order_release);
  command_buffer_.writeFromNonRT(command);
  return command.generation;
}

uint64_t LiftController::enqueue_safety_command(MotionCommand command)
{
  {
    // Serialize trajectory publication and cancellation generation assignment.
    // Whichever callback owns this boundary first gets the older generation;
    // a safety command can therefore invalidate every trajectory that was
    // actually queued before it without relying on executor callback order.
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    command.generation = next_motion_generation();
    cancellation_generation_.store(command.generation, std::memory_order_release);
    if (pending_trajectory_ && pending_trajectory_generation_ <= command.generation) {
      pending_trajectory_.reset();
    }
    if (pending_trajectory_generation_ <= command.generation) {
      consumed_trajectory_generation_ = pending_trajectory_generation_;
    }
  }
  safety_command_generation_.store(command.generation, std::memory_order_release);
  safety_command_buffer_.writeFromNonRT(command);
  return command.generation;
}

uint64_t LiftController::next_motion_generation() noexcept
{
  return motion_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
}

void LiftController::cancel_pending_trajectory(uint64_t cancel_generation)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (pending_trajectory_ && pending_trajectory_generation_ <= cancel_generation) {
    pending_trajectory_.reset();
  }
  if (pending_trajectory_generation_ <= cancel_generation) {
    consumed_trajectory_generation_ = pending_trajectory_generation_;
  }
}

bool LiftController::hold_snapshot_is_safe() const noexcept
{
  return mode_atomic_.load(std::memory_order_acquire) == static_cast<uint8_t>(Mode::hold) &&
         !trajectory_active_atomic_.load(std::memory_order_acquire) &&
         !gated_command_active_atomic_.load(std::memory_order_acquire) &&
         std::abs(measured_velocity_atomic_.load(std::memory_order_acquire)) <=
         stationary_velocity_mps_ &&
         std::abs(command_velocity_atomic_.load(std::memory_order_acquire)) <=
         stationary_velocity_mps_ &&
         std::abs(command_acceleration_atomic_.load(std::memory_order_acquire)) <= 1.0e-9;
}

void LiftController::begin_brake_gate(const MotionCommand & command)
{
  gated_command_ = command;
  gated_command_valid_ = true;
  mode_ = Mode::brake_gate;
  profile_.reset(measured_position_m_, 0.0, 0.0);
  command_sample_ = profile_.state();
  gate_stable_since_ = std::chrono::steady_clock::time_point{};
  goal_stable_active_ = false;
  if (command.type == CommandType::jog) {
    last_jog_command_time_ = std::chrono::steady_clock::now();
  }
  request_brake(true);
  status_message_ = "HOLD while waiting for CiA 402/brake gate";
}

void LiftController::start_gated_motion()
{
  if (!gated_command_valid_ || driver_estop_latched_.load()) {
    return;
  }
  const MotionCommand command = gated_command_;
  gated_command_valid_ = false;
  switch (command.type) {
    case CommandType::position:
      profile_.set_position_target(
        command.position_m, 0.0, 0.0, command.velocity_scale,
        command.acceleration_limit_mps2);
      mode_ = Mode::position;
      status_message_ = "absolute/relative Ruckig motion";
      break;
    case CommandType::trajectory:
      trajectory_elapsed_sec_ = 0.0;
      trajectory_point_index_ = 0;
      trajectory_waiting_at_point_ = false;
      if (active_trajectory_ && start_trajectory_point(0)) {
        mode_ = Mode::trajectory;
        status_message_ = "JointTrajectory active";
      } else {
        begin_soft_stop("trajectory unavailable after brake gate");
      }
      break;
    case CommandType::jog:
      profile_.set_velocity_target(command.velocity_mps, command.velocity_scale);
      last_jog_command_time_ = std::chrono::steady_clock::now();
      mode_ = Mode::jog;
      status_message_ = "Jog velocity active";
      break;
    case CommandType::stream:
      command_sample_ = {
        command.position_m, command.velocity_mps, command.acceleration_mps2,
        0.0, false, true};
      mode_ = Mode::streaming;
      status_message_ = "Heavy external streaming setpoint active";
      break;
    default:
      enter_hold(measured_position_m_, "empty brake-gated command");
      break;
  }
}

void LiftController::begin_soft_stop(
  const char * reason, SoftStopCompletion completion, uint64_t hold_generation)
{
  gated_command_valid_ = false;
  active_trajectory_.reset();
  active_trajectory_generation_ = 0;
  trajectory_waiting_at_point_ = false;
  profile_.request_soft_stop();
  mode_ = Mode::soft_stop;
  soft_stop_completion_ = completion;
  pending_hold_generation_ = hold_generation;
  status_message_ = reason ? reason : "soft stop";
  goal_stable_active_ = false;
}

void LiftController::enter_hold(double measured_position, const char * reason)
{
  profile_.hold(std::clamp(measured_position, limits_.min_position_m, limits_.max_position_m));
  command_sample_ = profile_.state();
  mode_ = Mode::hold;
  status_message_ = reason ? reason : "HOLD";
  goal_stable_active_ = false;
}

void LiftController::confirm_pending_hold()
{
  if (pending_hold_generation_ == 0 || mode_ != Mode::hold) {
    return;
  }
  if (driver_estop_latched_.load(std::memory_order_acquire) ||
    driver_motion_blocked_.load(std::memory_order_acquire) ||
    driver_quick_stop_.load(std::memory_order_acquire) || driver_error_code_.load() != 0)
  {
    fail_pending_hold(HoldFailureStage::driver_gate);
    return;
  }
  if (std::abs(measured_velocity_mps_) > stationary_velocity_mps_ ||
    std::abs(command_sample_.velocity_mps) > stationary_velocity_mps_ ||
    std::abs(command_sample_.acceleration_mps2) > 1.0e-9)
  {
    return;
  }
  const bool powered = power_enabled_state_.load(std::memory_order_acquire);
  if (powered && (!driver_operation_enabled_.load(std::memory_order_acquire) ||
    !driver_brake_unlocked_.load(std::memory_order_acquire)))
  {
    return;
  }
  completed_hold_generation_.store(pending_hold_generation_, std::memory_order_release);
  hold_failure_stage_.store(
    static_cast<uint8_t>(HoldFailureStage::none), std::memory_order_release);
  pending_hold_generation_ = 0;
}

void LiftController::fail_pending_hold(HoldFailureStage stage)
{
  if (pending_hold_generation_ == 0) {
    return;
  }
  hold_failure_stage_.store(static_cast<uint8_t>(stage), std::memory_order_release);
  failed_hold_generation_.store(pending_hold_generation_, std::memory_order_release);
  pending_hold_generation_ = 0;
}

const char * LiftController::hold_failure_stage_name(HoldFailureStage stage) noexcept
{
  switch (stage) {
    case HoldFailureStage::driver_gate: return "driver_gate";
    case HoldFailureStage::motion_profile: return "soft_stop";
    case HoldFailureStage::velocity_confirmation: return "velocity_confirmation";
    case HoldFailureStage::mode_confirmation: return "mode_confirmation";
    default: return "unknown";
  }
}

bool LiftController::driver_status_fresh() const noexcept
{
  const int64_t stamp = driver_status_ns_.load(std::memory_order_relaxed);
  if (stamp <= 0) {
    return false;
  }
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return now >= stamp && static_cast<double>(now - stamp) * 1e-9 <= driver_status_timeout_sec_;
}

bool LiftController::driver_gate_ready() const noexcept
{
  // BR+/BR- is released automatically by the LD3M after CiA402 reaches
  // Operation Enabled. `brake_unlocked` is only our local inference of that
  // transition, not an independent hardware acknowledgement. Requiring it
  // here turns the stream admission into a circular wait: no streamed output
  // until the inferred release, while the drive owns release as it enables.
  return driver_status_fresh() && driver_feedback_fresh_.load() &&
         driver_ethercat_operational_.load() && driver_working_counter_ok_.load() &&
         power_enabled_state_.load() && driver_operation_enabled_.load() &&
         !driver_estop_latched_.load() && driver_error_code_.load() == 0 &&
         driver_mode_display_.load() == 9;
}

std::string LiftController::driver_gate_failure_reason() const
{
  // Report every failing sub-condition together with its measured value so a
  // single log line identifies the cause without having to re-run the motion.
  std::ostringstream reason;
  bool first = true;
  const auto note = [&reason, &first](const char * name, const std::string & value) {
      if (!first) {
        reason << ',';
      }
      first = false;
      reason << name << '=' << value;
    };
  const int64_t stamp = driver_status_ns_.load(std::memory_order_relaxed);
  const int64_t now = now_steady_ns();
  if (stamp <= 0) {
    note("driver_status", "never_received");
  } else if (now >= stamp &&
    static_cast<double>(now - stamp) * 1e-9 > driver_status_timeout_sec_)
  {
    note(
      "driver_status_age_sec",
      std::to_string(static_cast<double>(now - stamp) * 1e-9));
  }
  if (!driver_feedback_fresh_.load()) {
    note("feedback_fresh", "false");
  }
  if (!driver_ethercat_operational_.load()) {
    note("ethercat_operational", "false");
  }
  if (!driver_working_counter_ok_.load()) {
    note("working_counter_ok", "false");
  }
  if (!power_enabled_state_.load()) {
    note("power_enabled_state", "false");
  }
  if (!driver_operation_enabled_.load()) {
    note("cia402_operation_enabled", "false");
  }
  if (driver_estop_latched_.load()) {
    note("estop_latched", "true");
  }
  if (driver_error_code_.load() != 0) {
    note("driver_error_code", std::to_string(driver_error_code_.load()));
  }
  if (driver_mode_display_.load() != 9) {
    note("mode_display", std::to_string(static_cast<int>(driver_mode_display_.load())));
  }
  if (first) {
    reason << "none";
  }
  return reason.str();
}

bool LiftController::lift_fault_line_due(const char * code) const noexcept
{
  const char * fault_code = code != nullptr ? code : "unknown";
  if (last_fault_line_ns_ == 0) {
    return true;
  }
  // A different fault is always reported, even inside the throttle window: the
  // suppressed counter only covers repeats of the code already on the wire.
  if (std::strcmp(last_fault_code_, fault_code) != 0) {
    return true;
  }
  return now_steady_ns() - last_fault_line_ns_ >= kFaultLineThrottleNs;
}

void LiftController::note_lift_fault_repeat(const char * code) noexcept
{
  // Control-cycle bookkeeping for a fault that is already latched and already
  // on the wire. It must stay allocation-free: this runs at 100 Hz for as long
  // as the latch survives.
  ++fault_total_;
  ++fault_line_suppressed_;
  if (fault_history_count_ == 0U) {
    return;
  }
  LiftFaultRecord & newest =
    fault_history_[(fault_history_next_ + kFaultHistoryCapacity - 1U) % kFaultHistoryCapacity];
  if (code == nullptr || std::strcmp(newest.code, code) != 0) {
    return;
  }
  newest.last_ns = now_steady_ns();
  ++newest.repeats;
}

void LiftController::write_lift_fault(const char * code, const std::string & detail)
{
  const char * fault_code = code != nullptr ? code : "unknown";
  const int64_t now_ns = now_steady_ns();
  ++fault_total_;

  LiftFaultRecord * newest = fault_history_count_ > 0U ?
    &fault_history_[(fault_history_next_ + kFaultHistoryCapacity - 1U) % kFaultHistoryCapacity] :
    nullptr;
  const bool new_burst = newest == nullptr || last_fault_record_ns_ == 0 ||
    std::strcmp(newest->code, fault_code) != 0 ||
    now_ns - last_fault_record_ns_ >= kFaultBurstWindowNs;
  if (new_burst) {
    LiftFaultRecord & record = fault_history_[fault_history_next_];
    record = LiftFaultRecord{};
    record.first_ns = now_ns;
    record.last_ns = now_ns;
    record.repeats = 1U;
    std::strncpy(record.code, fault_code, sizeof(record.code) - 1U);
    record.code[sizeof(record.code) - 1U] = '\0';
    std::strncpy(record.detail, detail.c_str(), sizeof(record.detail) - 1U);
    record.detail[sizeof(record.detail) - 1U] = '\0';
    fault_history_next_ = (fault_history_next_ + 1U) % kFaultHistoryCapacity;
    if (fault_history_count_ < kFaultHistoryCapacity) {
      ++fault_history_count_;
    }
  } else {
    newest->last_ns = now_ns;
    ++newest->repeats;
    std::strncpy(newest->detail, detail.c_str(), sizeof(newest->detail) - 1U);
    newest->detail[sizeof(newest->detail) - 1U] = '\0';
  }
  last_fault_record_ns_ = now_ns;

  const LiftFaultRecord & current =
    fault_history_[(fault_history_next_ + kFaultHistoryCapacity - 1U) % kFaultHistoryCapacity];
  RCLCPP_ERROR(
    get_node()->get_logger(),
    "[LIFT_FAULT] state=latched code=%s mode=%s total=%llu repeats_burst=%u "
    "suppressed_since_last=%llu detail=%s",
    fault_code, mode_name(mode_), static_cast<unsigned long long>(fault_total_),
    current.repeats, static_cast<unsigned long long>(fault_line_suppressed_), detail.c_str());
  fault_line_suppressed_ = 0;
  last_fault_line_ns_ = now_ns;
  std::strncpy(last_fault_code_, fault_code, sizeof(last_fault_code_) - 1U);
  last_fault_code_[sizeof(last_fault_code_) - 1U] = '\0';
}

bool LiftController::driver_gate_expected() const noexcept
{
  // The CiA 402 gate is only supposed to be ready while the drive is expected
  // to be enabled. An idle, unpowered lift parked in HOLD reports an unready
  // gate by design; warning about that would be pure noise.
  return power_enable_requested_.load(std::memory_order_acquire) ||
         power_enabled_state_.load(std::memory_order_acquire) ||
         gated_command_valid_ ||
         pending_hold_generation_ != 0 ||
         (mode_ != Mode::hold && mode_ != Mode::estop);
}

void LiftController::log_gate_diagnostics(const char * context)
{
  const char * site = context != nullptr ? context : "update";
  if (mode_ == Mode::fault) {
    // The latched [LIFT_FAULT] record already owns the ERROR line for this
    // condition; do not restate it once per control cycle.
    gate_unready_since_ns_ = 0;
    last_gate_debug_ns_ = 0;
    last_gate_warn_ns_ = 0;
    gate_warn_emitted_ = false;
    return;
  }
  const int64_t now_ns = now_steady_ns();
  if (driver_gate_ready()) {
    if (gate_unready_since_ns_ == 0) {
      return;
    }
    const double unready_sec = static_cast<double>(now_ns - gate_unready_since_ns_) * 1e-9;
    if (gate_warn_emitted_) {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[LIFT_GATE] context=%s state=recovered mode=%s unready_sec=%.3f",
        site, mode_name(mode_), unready_sec);
    } else {
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "[LIFT_GATE] context=%s state=recovered mode=%s unready_sec=%.3f",
        site, mode_name(mode_), unready_sec);
    }
    gate_unready_since_ns_ = 0;
    last_gate_debug_ns_ = 0;
    last_gate_warn_ns_ = 0;
    gate_warn_emitted_ = false;
    return;
  }
  if (gate_unready_since_ns_ == 0) {
    gate_unready_since_ns_ = now_ns;
  }
  const double unready_sec = static_cast<double>(now_ns - gate_unready_since_ns_) * 1e-9;
  const bool expected = driver_gate_expected();
  if (last_gate_debug_ns_ == 0 || now_ns - last_gate_debug_ns_ >= kGateDebugThrottleNs) {
    last_gate_debug_ns_ = now_ns;
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "[LIFT_GATE] context=%s state=unready mode=%s expected=%s unready_sec=%.3f "
      "power_requested=%s power_enabled=%s gate[%s]",
      site, mode_name(mode_), expected ? "true" : "false", unready_sec,
      power_enable_requested_.load() ? "true" : "false",
      power_enabled_state_.load() ? "true" : "false",
      driver_gate_failure_reason().c_str());
  }
  if (!expected || unready_sec < kGateWarnAfterSec) {
    return;
  }
  if (last_gate_warn_ns_ != 0 && now_ns - last_gate_warn_ns_ < kGateWarnWindowNs) {
    return;
  }
  last_gate_warn_ns_ = now_ns;
  gate_warn_emitted_ = true;
  RCLCPP_WARN(
    get_node()->get_logger(),
    "[LIFT_GATE] context=%s state=unready_sustained mode=%s unready_sec=%.3f "
    "power_requested=%s power_enabled=%s gate[%s]",
    site, mode_name(mode_), unready_sec,
    power_enable_requested_.load() ? "true" : "false",
    power_enabled_state_.load() ? "true" : "false",
    driver_gate_failure_reason().c_str());
}

void LiftController::log_command_rejection(const char * category, const std::string & detail)
{
  const int64_t now_ns = now_steady_ns();
  if (last_command_debug_ns_ != 0 &&
    now_ns - last_command_debug_ns_ < kCommandDebugThrottleNs)
  {
    return;
  }
  last_command_debug_ns_ = now_ns;
  RCLCPP_DEBUG(
    get_node()->get_logger(),
    "[LIFT_CMD] category=%s mode=%s %s",
    category != nullptr ? category : "-", mode_name(mode_), detail.c_str());
}

void LiftController::request_brake(bool enable)
{
  power_enable_requested_.store(enable, std::memory_order_release);
}

void LiftController::on_driver_status(const std_msgs::msg::String & message)
{
  driver_feedback_fresh_.store(
    json_bool(message.data, "feedback_fresh") || json_bool(message.data, "pdo_fresh"));
  driver_ethercat_operational_.store(
    json_bool(message.data, "ethercat_operational") ||
    message.data.find("\"link_state\":\"operational\"") != std::string::npos);
  driver_working_counter_ok_.store(json_bool(message.data, "working_counter_ok"));
  driver_operation_enabled_.store(
    message.data.find("\"cia402_state\":\"operation_enabled\"") != std::string::npos);
  driver_brake_unlocked_.store(
    json_bool(message.data, "brake_unlocked_inferred") ||
    json_bool(message.data, "brake_unlocked"));
  driver_estop_latched_.store(json_bool(message.data, "estop_latched"));
  driver_motion_blocked_.store(json_bool(message.data, "motion_blocked"));
  driver_quick_stop_.store(json_bool(message.data, "quick_stop_active"));
  driver_error_code_.store(static_cast<uint16_t>(json_integer(message.data, "error_code")));
  driver_mode_display_.store(
    static_cast<int8_t>(
      json_integer(message.data, "mode_display", json_integer(message.data, "mode"))));
  driver_status_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void LiftController::publish_status()
{
  if (!status_publisher_ || !active_.load(std::memory_order_acquire)) {
    return;
  }
  const auto mode = static_cast<Mode>(mode_atomic_.load(std::memory_order_acquire));
  std_msgs::msg::String message;
  std::ostringstream status;
  status << "{\"component\":\"lift_controller\",\"mode\":\"" << mode_name(mode)
         << "\",\"position\":" << measured_position_atomic_.load()
         << ",\"velocity\":" << measured_velocity_atomic_.load()
         << ",\"command_position\":" << command_position_atomic_.load()
         << ",\"command_velocity\":" << command_velocity_atomic_.load()
         << ",\"command_acceleration\":" << command_acceleration_atomic_.load()
         << ",\"estop_latched\":" << (driver_estop_latched_.load() ? "true" : "false")
         << ",\"brake_gate_ready\":" << (driver_gate_ready() ? "true" : "false")
         << ",\"power_enable_command\":" <<
    (power_enable_requested_.load() ? "true" : "false")
         << ",\"power_enabled\":" << (power_enabled_state_.load() ? "true" : "false")
         << ",\"trajectory_active\":" << (mode == Mode::trajectory ? "true" : "false")
         << ",\"jog_active\":" << (mode == Mode::jog ? "true" : "false")
         << ",\"motion_generation\":" << motion_generation_.load()
         << ",\"cancellation_generation\":" << cancellation_generation_.load()
         << ",\"applied_cancellation_generation\":" <<
    applied_cancellation_generation_.load()
         << ",\"gate_ready\":" << (driver_gate_ready() ? "true" : "false")
         << ",\"gate_failure\":" << json_quote(
    driver_gate_ready() ? std::string("none") : driver_gate_failure_reason())
         << ",\"fault_total\":" << fault_total_
         << ",\"fault_history\":[";
  // Diagnostics only. The RT writer appends without a lock so a reader can
  // briefly observe a half-written record; a garbled diagnostic string is
  // preferable to blocking the 100 Hz control loop on a mutex.
  const int64_t status_now_ns = now_steady_ns();
  for (std::size_t offset = 0; offset < fault_history_count_; ++offset) {
    const std::size_t index =
      (fault_history_next_ + kFaultHistoryCapacity - 1U - offset) % kFaultHistoryCapacity;
    const LiftFaultRecord & record = fault_history_[index];
    if (offset != 0U) {
      status << ',';
    }
    const double age_ms = record.first_ns > 0 && status_now_ns >= record.first_ns ?
      static_cast<double>(status_now_ns - record.first_ns) * 1e-6 : -1.0;
    const double last_age_ms = record.last_ns > 0 && status_now_ns >= record.last_ns ?
      static_cast<double>(status_now_ns - record.last_ns) * 1e-6 : -1.0;
    status << "{\"age_ms\":" << age_ms
           << ",\"last_age_ms\":" << last_age_ms
           << ",\"repeats\":" << record.repeats
           << ",\"code\":" << json_quote(record.code)
           << ",\"detail\":" << json_quote(record.detail) << '}';
  }
  status << "]}";
  message.data = status.str();
  status_publisher_->publish(message);
}

void LiftController::write_command_interfaces(const lift::LiftMotionSample & sample)
{
  const bool finite = std::isfinite(sample.position_m) && std::isfinite(sample.velocity_mps) &&
    std::isfinite(sample.acceleration_mps2);
  command_interfaces_[position_command_index_].set_value(
    finite ? std::clamp(sample.position_m, limits_.min_position_m, limits_.max_position_m) :
    measured_position_m_);
  command_interfaces_[velocity_command_index_].set_value(finite ? sample.velocity_mps : 0.0);
  command_interfaces_[acceleration_command_index_].set_value(
    finite ? sample.acceleration_mps2 : 0.0);
  command_interfaces_[power_enable_command_index_].set_value(
    power_enable_requested_.load(std::memory_order_acquire) ? 1.0 : 0.0);
  command_position_atomic_.store(
    finite ? std::clamp(sample.position_m, limits_.min_position_m, limits_.max_position_m) :
    measured_position_m_, std::memory_order_release);
  command_velocity_atomic_.store(finite ? sample.velocity_mps : 0.0, std::memory_order_release);
  command_acceleration_atomic_.store(
    finite ? sample.acceleration_mps2 : 0.0, std::memory_order_release);
  mode_atomic_.store(static_cast<uint8_t>(mode_), std::memory_order_release);
  trajectory_active_atomic_.store(
    mode_ == Mode::trajectory && active_trajectory_ != nullptr, std::memory_order_release);
  gated_command_active_atomic_.store(gated_command_valid_, std::memory_order_release);
}

bool LiftController::start_trajectory_point(std::size_t index)
{
  if (!active_trajectory_ || index >= active_trajectory_->points.size()) {
    return false;
  }
  const auto & point = active_trajectory_->points[index];
  const double velocity = point.velocities.empty() ? 0.0 : point.velocities[0];
  const double acceleration = point.accelerations.empty() ? 0.0 : point.accelerations[0];
  trajectory_waiting_at_point_ = false;
  return profile_.set_position_target(
    point.positions[0], velocity, acceleration, limits_.default_velocity_scale);
}

const char * LiftController::mode_name(Mode mode) noexcept
{
  switch (mode) {
    case Mode::hold: return "hold";
    case Mode::brake_gate: return "brake_gate";
    case Mode::position: return "position";
    case Mode::trajectory: return "trajectory";
    case Mode::jog: return "jog";
    case Mode::streaming: return "streaming";
    case Mode::soft_stop: return "soft_stop";
    case Mode::estop: return "estop";
    case Mode::fault: return "fault";
    default: return "unknown";
  }
}

}  // namespace joint_hardware

PLUGINLIB_EXPORT_CLASS(
  joint_hardware::LiftController, controller_interface::ControllerInterface)

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
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
#include "robot_lower_gateway/heavy_gateway_v1.hpp"
#include "robot_lower_gateway/junior_gateway_contract.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{

using namespace std::chrono_literals;
using Command = robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1;
using Feedback = robot_control_msg::msg::HeavyUpperBodyGatewayFeedbackV1;
using Status = robot_control_msg::msg::HeavyUpperBodyGatewayStatusV1;
namespace contract = junior_gateway::junior_heavy::v1;

class MockLowerControllers final : public rclcpp::Node
{
public:
  MockLowerControllers()
  : Node("mock_heavy_lower_controllers")
  {
    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable().transient_local();
    arm_joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/arm/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    lift_joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/lift/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    arm_power_publisher_ = create_publisher<robot_control_msg::msg::ArmPowerStatus>(
      "/arm/power_status", status_qos);
    lift_status_publisher_ = create_publisher<robot_control_msg::msg::LiftStatus>(
      "/ubuntu_lower_gateway/lift/status", status_qos);
    workspace_publisher_ = create_publisher<robot_control_msg::msg::WorkspaceStatus>(
      "/workspace/status", status_qos);
    control_mode_publisher_ = create_publisher<robot_control_msg::msg::ArmControlModeStatus>(
      "/arm/control_mode_status", status_qos);
    arm_ack_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "/ubuntu_lower_gateway/internal/heavy/v1/arm_applied_sequence",
      rclcpp::QoS(10).reliable());
    lift_ack_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "/ubuntu_lower_gateway/internal/heavy/v1/lift_applied_sequence",
      rclcpp::QoS(10).reliable());
    lift_brake_service_ = create_service<std_srvs::srv::SetBool>(
      "/mock/lift_brake",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request->data) {
          ++brake_release_calls_;
        } else {
          ++brake_lock_calls_;
        }
        brake_unlocked_.store(request->data, std::memory_order_release);
        response->success = true;
        response->message = request->data ? "mock brake released" : "mock brake locked";
        condition_.notify_all();
      });

    rclcpp::QoS command_qos(rclcpp::KeepLast(1));
    command_qos.reliable().durability_volatile();
    internal_command_subscription_ = create_subscription<Command>(
      "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command", command_qos,
      [this](const Command::SharedPtr command) {
        if (!command) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          internal_sequences_.push_back(command->sequence);
          internal_commands_.push_back(*command);
        }
        const auto acknowledgement_delay = std::chrono::milliseconds(
          acknowledgements_delay_ms_.load(std::memory_order_acquire));
        if (acknowledgements_enabled_.load(std::memory_order_acquire) &&
          acknowledgement_delay.count() == 0)
        {
          std_msgs::msg::UInt64 ack;
          ack.data = command->sequence;
          arm_ack_publisher_->publish(ack);
          if (!drop_next_lift_acknowledgement_.exchange(false, std::memory_order_acq_rel)) {
            lift_ack_publisher_->publish(ack);
          }
        } else if (acknowledgements_enabled_.load(std::memory_order_acquire)) {
          std::lock_guard<std::mutex> lock(mutex_);
          delayed_acknowledgements_.push_back(
            DelayedAcknowledgement{command->sequence, std::chrono::steady_clock::now() +
            acknowledgement_delay});
        }
        condition_.notify_all();
      });
    second_internal_command_subscription_ = create_subscription<Command>(
      "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command", command_qos,
      [](const Command::SharedPtr) {});
    publish_timer_ = create_wall_timer(10ms, [this]() {publishFeedback();});
  }

  bool waitForInternalCommands(std::size_t count, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, timeout, [this, count]() {
        return internal_sequences_.size() >= count;
      });
  }

  void setWorkspacePublishing(bool enabled)
  {
    publish_workspace_.store(enabled, std::memory_order_release);
  }

  void setWorkspaceRunning(bool running)
  {
    workspace_running_.store(running, std::memory_order_release);
  }

  void setWorkspaceMode(uint8_t mode)
  {
    workspace_mode_.store(mode, std::memory_order_release);
  }

  void setBrakeUnlocked(bool unlocked)
  {
    brake_unlocked_.store(unlocked, std::memory_order_release);
  }

  void setArmJointPublishing(bool enabled)
  {
    publish_arm_joint_.store(enabled, std::memory_order_release);
  }

  void setSimulatedActuatorsEnabled(bool enabled)
  {
    simulated_actuators_enabled_.store(enabled, std::memory_order_release);
    simulated_lift_enabled_.store(enabled, std::memory_order_release);
  }

  void setArmActuatorsEnabled(bool enabled)
  {
    simulated_actuators_enabled_.store(enabled, std::memory_order_release);
  }

  void setLiftEnabled(bool enabled)
  {
    simulated_lift_enabled_.store(enabled, std::memory_order_release);
  }

  void setAcknowledgementsEnabled(bool enabled)
  {
    acknowledgements_enabled_.store(enabled, std::memory_order_release);
    if (!enabled) {
      std::lock_guard<std::mutex> lock(mutex_);
      delayed_acknowledgements_.clear();
    }
  }

  void setAcknowledgementDelay(std::chrono::milliseconds delay)
  {
    acknowledgements_delay_ms_.store(delay.count(), std::memory_order_release);
  }

  void dropNextLiftAcknowledgement()
  {
    drop_next_lift_acknowledgement_.store(true, std::memory_order_release);
  }

  void disconnectHeavyEndpoints()
  {
    internal_command_subscription_.reset();
    second_internal_command_subscription_.reset();
    arm_ack_publisher_.reset();
    lift_ack_publisher_.reset();
  }

  Command lastInternalCommand()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return internal_commands_.back();
  }

  std::uint64_t brakeLockCalls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return brake_lock_calls_;
  }

  std::uint64_t brakeReleaseCalls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return brake_release_calls_;
  }

private:
  void publishFeedback()
  {
    std::vector<std::uint64_t> due_acknowledgements;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      while (!delayed_acknowledgements_.empty() &&
        delayed_acknowledgements_.front().due <= now)
      {
        due_acknowledgements.push_back(delayed_acknowledgements_.front().sequence);
        delayed_acknowledgements_.pop_front();
      }
    }
    for (const auto sequence : due_acknowledgements) {
      std_msgs::msg::UInt64 ack;
      ack.data = sequence;
      arm_ack_publisher_->publish(ack);
      lift_ack_publisher_->publish(ack);
    }
    const bool actuators_enabled = simulated_actuators_enabled_.load(std::memory_order_acquire);
    const bool lift_enabled = simulated_lift_enabled_.load(std::memory_order_acquire);
    static const std::array<std::string, 14> names = {
      "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
      "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
    sensor_msgs::msg::JointState arm;
    arm.header.stamp = now();
    for (std::size_t reverse = 0; reverse < names.size(); ++reverse) {
      const auto index = names.size() - 1 - reverse;
      arm.name.push_back(names[index]);
      arm.position.push_back(0.1 * static_cast<double>(index + 1));
      arm.velocity.push_back(0.01 * static_cast<double>(index + 1));
    }
    if (publish_arm_joint_.load(std::memory_order_acquire)) {
      arm_joint_publisher_->publish(arm);
    }

    sensor_msgs::msg::JointState lift;
    lift.header.stamp = now();
    lift.name = {"joint_motor"};
    lift.position = {-0.25};
    lift.velocity = {0.0};
    lift_joint_publisher_->publish(lift);

    robot_control_msg::msg::ArmPowerStatus power;
    power.stamp = now();
    power.joint_names.assign(names.begin(), names.end());
    power.status_codes.assign(
      14, actuators_enabled ? power.ENABLED_STATUS : power.DISABLED_STATUS);
    power.enabled.assign(14, actuators_enabled);
    power.command_enabled = actuators_enabled;
    power.all_enabled = actuators_enabled;
    arm_power_publisher_->publish(power);

    robot_control_msg::msg::LiftStatus lift_status;
    lift_status.stamp = now();
    lift_status.joint_name = "joint_motor";
    lift_status.valid = true;
    lift_status.feedback_fresh = true;
    lift_status.ethercat_operational = true;
    lift_status.working_counter_ok = true;
    lift_status.initialized = true;
    lift_status.cia402_state = "operation_enabled";
    lift_status.status_word = 0x27;
    lift_status.command_enabled = lift_enabled;
    lift_status.enabled = lift_enabled;
    // Brake state is independent of the SIM torque gate.  This lets the test
    // verify that FOLLOW releases a previously locked brake even while SIM
    // intentionally reports all drives disabled.
    lift_status.brake_unlocked = brake_unlocked_.load(std::memory_order_acquire);
    lift_status_publisher_->publish(lift_status);

    if (publish_workspace_.load(std::memory_order_acquire)) {
      robot_control_msg::msg::WorkspaceStatus workspace;
      workspace.stamp = now();
      const bool workspace_running = workspace_running_.load(std::memory_order_acquire);
      workspace.state = workspace_running ? workspace.RUNNING : workspace.STOPPED;
      workspace.mode = workspace_mode_.load(std::memory_order_acquire);
      workspace.accepted = true;
      workspace.message = workspace_running ? "mock simulation running" : "mock simulation stopped";
      workspace_publisher_->publish(workspace);
    }

    robot_control_msg::msg::ArmControlModeStatus control;
    control.stamp = now();
    control.requested_mode = control.POSITION;
    control.active_mode = control.POSITION;
    control.position_command_ready = true;
    control_mode_publisher_->publish(control);
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  struct DelayedAcknowledgement
  {
    std::uint64_t sequence;
    std::chrono::steady_clock::time_point due;
  };
  std::vector<std::uint64_t> internal_sequences_;
  std::vector<Command> internal_commands_;
  std::deque<DelayedAcknowledgement> delayed_acknowledgements_;
  std::uint64_t brake_lock_calls_{0};
  std::uint64_t brake_release_calls_{0};
  std::atomic<bool> publish_workspace_{true};
  std::atomic<bool> workspace_running_{true};
  std::atomic<uint8_t> workspace_mode_{robot_control_msg::msg::WorkspaceStatus::SIMULATION};
  std::atomic<bool> publish_arm_joint_{true};
  std::atomic<bool> simulated_actuators_enabled_{true};
  std::atomic<bool> simulated_lift_enabled_{true};
  std::atomic<bool> acknowledgements_enabled_{true};
  std::atomic<std::int64_t> acknowledgements_delay_ms_{0};
  std::atomic<bool> drop_next_lift_acknowledgement_{false};
  std::atomic<bool> brake_unlocked_{true};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lift_joint_publisher_;
  rclcpp::Publisher<robot_control_msg::msg::ArmPowerStatus>::SharedPtr arm_power_publisher_;
  rclcpp::Publisher<robot_control_msg::msg::LiftStatus>::SharedPtr lift_status_publisher_;
  rclcpp::Publisher<robot_control_msg::msg::WorkspaceStatus>::SharedPtr workspace_publisher_;
  rclcpp::Publisher<robot_control_msg::msg::ArmControlModeStatus>::SharedPtr
    control_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr arm_ack_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr lift_ack_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lift_brake_service_;
  rclcpp::Subscription<Command>::SharedPtr internal_command_subscription_;
  rclcpp::Subscription<Command>::SharedPtr second_internal_command_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

class ObservationNode final : public rclcpp::Node
{
public:
  ObservationNode()
  : Node("heavy_gateway_v1_observer")
  {
    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable().transient_local();
    status_subscription_ = create_subscription<Status>(
      contract::kStatusTopic.data(), status_qos,
      [this](const Status::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = *message;
        status_received_ = true;
        condition_.notify_all();
      });
    feedback_subscription_ = create_subscription<Feedback>(
      contract::kFeedbackTopic.data(), rclcpp::QoS(1).best_effort(),
      [this](const Feedback::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback_ = *message;
        feedback_received_ = true;
        ++feedback_statistics_.samples;
        if (message->position_valid_mask != 0x7fffU ||
          message->velocity_valid_mask != 0x7fffU)
        {
          ++feedback_statistics_.invalid_masks;
        }
        if (message->resource_fault_mask != 0U) {
          ++feedback_statistics_.resource_faults;
        }
        condition_.notify_all();
      });
    rclcpp::QoS command_qos(rclcpp::KeepLast(1));
    command_qos.best_effort().durability_volatile();
    command_qos.deadline(rclcpp::Duration(0, 20000000));
    command_qos.lifespan(rclcpp::Duration(0, 50000000));
    command_publisher_ = create_publisher<Command>(contract::kCommandTopic.data(), command_qos);
  }

  template<typename Predicate>
  bool waitFor(Predicate predicate, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [&]() {return predicate(status_, feedback_);});
  }

  Status status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  Feedback feedback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
  }

  struct FeedbackStatistics
  {
    std::uint64_t samples{0};
    std::uint64_t invalid_masks{0};
    std::uint64_t resource_faults{0};
  };

  FeedbackStatistics feedbackStatistics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_statistics_;
  }

  rclcpp::Publisher<Command>::SharedPtr command_publisher_;

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  Status status_;
  Feedback feedback_;
  FeedbackStatistics feedback_statistics_;
  bool status_received_{false};
  bool feedback_received_{false};
  rclcpp::Subscription<Status>::SharedPtr status_subscription_;
  rclcpp::Subscription<Feedback>::SharedPtr feedback_subscription_;
};

template<typename ServiceT>
typename ServiceT::Response::SharedPtr callService(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request)
{
  if (!client->wait_for_service(2s)) {
    return nullptr;
  }
  auto future = client->async_send_request(request);
  return future.wait_for(2s) == std::future_status::ready ? future.get() : nullptr;
}

class ExecutorGuard
{
public:
  explicit ExecutorGuard(rclcpp::executors::MultiThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();}) {}

  ~ExecutorGuard()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  rclcpp::executors::MultiThreadedExecutor & executor_;
  std::thread thread_;
};

TEST(HeavyGatewayV1, RetransmitsLostLiftAcknowledgementWithoutRevokingLease)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides({
    rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake"),
    rclcpp::Parameter("heavy_v1.controller_ack_retry_interval_ms", 50)});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_ack_retry", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_ack_retry_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "ack-retry-test";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(lease, nullptr);
  ASSERT_TRUE(lease->granted) << lease->message;
  ASSERT_TRUE(mock->waitForInternalCommands(1, 1s));
  const auto aligned_hold = mock->lastInternalCommand();
  ASSERT_TRUE(observer->waitFor(
      [&lease](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "ack-retry-test" &&
               status.active_lease_id == lease->lease_id;
      }, 1s));

  Command hold;
  hold.header.stamp = observer->now();
  hold.protocol_major = 1;
  hold.protocol_minor = 0;
  hold.layout_crc32 = contract::kLayoutCrc32;
  hold.owner_id = acquire_request->owner_id;
  hold.lease_id = lease->lease_id;
  hold.sequence = lease->initial_sequence;
  hold.mode = Command::MODE_HOLD;
  hold.field_mask = Command::FIELD_POSITION;
  hold.position[0] = -0.25;
  for (std::size_t index = 1; index < hold.position.size(); ++index) {
    hold.position[index] = 0.1 * static_cast<double>(index);
  }
  hold.velocity.fill(0.0);
  hold.acceleration.fill(0.0);

  mock->dropNextLiftAcknowledgement();
  observer->command_publisher_->publish(hold);
  ASSERT_TRUE(mock->waitForInternalCommands(3, 1s));
  ASSERT_TRUE(observer->waitFor(
      [&hold, &lease](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "ack-retry-test" &&
               status.active_lease_id == lease->lease_id &&
               feedback.applied_command_sequence == hold.sequence;
      }, 1s));
  EXPECT_EQ(mock->lastInternalCommand().sequence, aligned_hold.sequence + 1);
}

TEST(HeavyGatewayV1, ContractLeaseValidationWatchdogsAndFeedbackMapping)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides(
    {rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake")});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_under_test", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  // The real workspace supervisor publishes every 2 seconds. A gap longer
  // than the old 500 ms threshold must not make an otherwise healthy gateway
  // alternate between MONITOR and FAULT.
  mock->setWorkspacePublishing(false);
  std::this_thread::sleep_for(750ms);
  EXPECT_EQ(observer->status().state, Status::STATE_MONITOR);
  EXPECT_EQ(observer->feedback().workspace_state, Feedback::WORKSPACE_SIMULATION);
  mock->setWorkspacePublishing(true);

  using Capabilities = robot_control_msg::srv::GetHeavyGatewayCapabilitiesV1;
  auto capabilities_client = clients->create_client<Capabilities>(
    contract::kCapabilitiesService.data());
  auto capabilities_request = std::make_shared<Capabilities::Request>();
  capabilities_request->requested_protocol_major = 1;
  const auto capabilities = callService<Capabilities>(capabilities_client, capabilities_request);
  ASSERT_NE(capabilities, nullptr);
  EXPECT_TRUE(capabilities->compatible);
  EXPECT_EQ(capabilities->layout_crc32, contract::kLayoutCrc32);
  EXPECT_EQ(capabilities->resource_count, 15);
  EXPECT_TRUE(capabilities->supports_velocity_feedforward);
  EXPECT_TRUE(capabilities->supports_acceleration_feedforward);
  EXPECT_EQ(capabilities->interface_schema_sha256, contract::kInterfaceSchemaSha256);
  capabilities_request->requested_protocol_major = 2;
  const auto incompatible = callService<Capabilities>(capabilities_client, capabilities_request);
  ASSERT_NE(incompatible, nullptr);
  EXPECT_FALSE(incompatible->compatible);

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  using LiftBrake = robot_control_msg::srv::SetHeavyLiftBrakeV1;
  auto brake_client = clients->create_client<LiftBrake>(contract::kLiftBrakeService.data());
  auto brake_request = std::make_shared<LiftBrake::Request>();
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "integration-test";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto acquired = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(acquired, nullptr);
  ASSERT_TRUE(acquired->granted) << acquired->message;
  ASSERT_FALSE(acquired->lease_id.empty());
  EXPECT_DOUBLE_EQ(acquired->lease_timeout_ms, 500.0);
  ASSERT_TRUE(mock->waitForInternalCommands(1, 1s));
  const auto aligned_hold = mock->lastInternalCommand();
  EXPECT_EQ(aligned_hold.mode, Command::MODE_HOLD);
  EXPECT_EQ(aligned_hold.field_mask, Command::FIELD_POSITION);
  EXPECT_DOUBLE_EQ(aligned_hold.position[0], -0.25);
  for (std::size_t index = 1; index < aligned_hold.position.size(); ++index) {
    EXPECT_DOUBLE_EQ(aligned_hold.position[index], 0.1 * static_cast<double>(index));
  }
  // The aligned HOLD is position-only by construction: makeHoldCommandLocked()
  // sets field_mask = FIELD_POSITION and fills velocity/acceleration with zero,
  // so a hold can never command motion.  The test used to expect the feedback
  // velocities (0.01 * index) to be echoed here, which only held while the
  // command also carried FIELD_VELOCITY; the position mapping above still proves
  // the per-joint feedback mapping.
  EXPECT_EQ(aligned_hold.field_mask, Command::FIELD_POSITION);
  for (std::size_t index = 0; index < aligned_hold.velocity.size(); ++index) {
    EXPECT_DOUBLE_EQ(aligned_hold.velocity[index], 0.0);
    EXPECT_DOUBLE_EQ(aligned_hold.acceleration[index], 0.0);
  }
  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD;
      }, 1s));

  Command hold;
  hold.protocol_major = 1;
  hold.protocol_minor = 0;
  hold.layout_crc32 = contract::kLayoutCrc32;
  hold.owner_id = acquire_request->owner_id;
  hold.lease_id = acquired->lease_id;
  hold.sequence = acquired->initial_sequence;
  hold.mode = Command::MODE_HOLD;
  hold.field_mask = Command::FIELD_POSITION;
  hold.position[0] = -0.25;
  for (std::size_t index = 1; index < hold.position.size(); ++index) {
    hold.position[index] = 0.1 * static_cast<double>(index);
  }
  hold.velocity.fill(0.0);
  hold.acceleration.fill(0.0);
  observer->command_publisher_->publish(hold);
  ASSERT_TRUE(observer->waitFor(
      [&hold, &acquired](const Status & status, const Feedback & feedback) {
        return feedback.applied_command_sequence == hold.sequence &&
        status.state == Status::STATE_LEASED_HOLD &&
        status.active_owner_id == "integration-test" &&
        status.active_lease_id == acquired->lease_id;
      }, 1s));

  auto next_hold = hold;
  next_hold.sequence++;
  observer->command_publisher_->publish(next_hold);
  ASSERT_TRUE(observer->waitFor(
      [&next_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == next_hold.sequence;
      }, 1s));

  // This exercises the real ROS 2 BEST_EFFORT/VOLATILE command endpoint at
  // 100 Hz.  A HOLD is a lease keepalive and must remain LEASED_HOLD well
  // beyond the 100 ms FOLLOW watchdog; only a 500 ms absence may revoke it.
  const auto keepalive_baseline = observer->status();
  const auto feedback_baseline = observer->feedbackStatistics();
  auto keepalive = next_hold;
  // 100 Hz HOLD is the normal remote watchdog.  Ten seconds exercises a
  // realistic keepalive window without allowing it to retrigger a lock.
  const auto keepalive_until = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < keepalive_until) {
    ++keepalive.sequence;
    keepalive.header.stamp = observer->now();
    observer->command_publisher_->publish(keepalive);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(observer->waitFor(
      [&keepalive, &acquired, &keepalive_baseline](const Status & status, const Feedback & feedback) {
        return feedback.applied_command_sequence >= keepalive.sequence - 2 &&
        status.state == Status::STATE_LEASED_HOLD &&
        status.active_owner_id == "integration-test" &&
        status.active_lease_id == acquired->lease_id &&
        status.accepted_commands >= keepalive_baseline.accepted_commands + 900 &&
        status.rejected_commands == keepalive_baseline.rejected_commands &&
        status.duplicate_commands == keepalive_baseline.duplicate_commands &&
        status.out_of_order_commands == keepalive_baseline.out_of_order_commands;
      }, 1s));
  const auto feedback_after_hold = observer->feedbackStatistics();
  EXPECT_GE(feedback_after_hold.samples - feedback_baseline.samples, 900U);
  EXPECT_EQ(feedback_after_hold.invalid_masks, feedback_baseline.invalid_masks);
  EXPECT_EQ(feedback_after_hold.resource_faults, feedback_baseline.resource_faults);
  EXPECT_LE(mock->brakeLockCalls(), 1U);
  EXPECT_NE(observer->status().detail.find("brake_transition_count=1"), std::string::npos);

  const auto conflict = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(conflict, nullptr);
  EXPECT_FALSE(conflict->granted);

  auto rejected = Command();
  rejected.protocol_major = 1;
  rejected.protocol_minor = 0;
  rejected.layout_crc32 = 0;
  rejected.owner_id = acquire_request->owner_id;
  rejected.lease_id = acquired->lease_id;
  rejected.sequence = keepalive.sequence + 1;
  rejected.mode = Command::MODE_FOLLOW_POSITION;
  rejected.field_mask = Command::FIELD_POSITION;
  rejected.position.fill(0.0);
  const auto releases_before_invalid_follow = mock->brakeReleaseCalls();
  observer->command_publisher_->publish(rejected);
  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {return status.rejected_commands >= 1;}, 1s));
  EXPECT_EQ(mock->brakeReleaseCalls(), releases_before_invalid_follow)
    << "an invalid FOLLOW must not release the brake";

  auto command = rejected;
  command.layout_crc32 = contract::kLayoutCrc32;
  command.field_mask = Command::FIELD_POSITION | Command::FIELD_VELOCITY |
    Command::FIELD_ACCELERATION;
  command.position[0] = -0.24;
  for (std::size_t index = 1; index < command.position.size(); ++index) {
    command.position[index] = 0.1 * static_cast<double>(index);
    command.velocity[index] = 0.01 * static_cast<double>(index);
    command.acceleration[index] = 0.02 * static_cast<double>(index);
  }
  command.velocity[0] = -0.03;
  command.acceleration[0] = -0.04;
  // The SIM state provider deliberately reports all drives disabled.  A
  // valid Heavy FOLLOW must still reach the mock controller while REAL keeps
  // its physical torque/brake requirement.
  mock->setSimulatedActuatorsEnabled(false);
  observer->command_publisher_->publish(command);
  ASSERT_TRUE(
    observer->waitFor(
      [&command](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == command.sequence &&
        feedback.arm_torque_enabled_mask == 0U && !feedback.lift_drive_enabled;
      }, 1s));
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {
        return status.state == Status::STATE_FOLLOWING;
      }, 1s));
  EXPECT_EQ(mock->brakeReleaseCalls(), releases_before_invalid_follow + 1)
    << "the first valid FOLLOW releases the locked brake exactly once";

  // The 100 ms command watchdog must move FOLLOW to aligned HOLD, but only
  // the 500 ms lease watchdog may revoke the complete owner/lease identity.
  std::this_thread::sleep_for(250ms);
  ASSERT_TRUE(observer->waitFor(
      [&acquired](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "integration-test" &&
               status.active_lease_id == acquired->lease_id &&
               feedback.active_owner_id == status.active_owner_id &&
               feedback.active_lease_id == status.active_lease_id &&
               status.detail.find("command_timeout: FOLLOWING to HOLD") != std::string::npos;
      }, 1s));

  // The command watchdog already performed the FOLLOW -> HOLD brake
  // transition.  The following explicit HOLD and later keepalives must not
  // replay it.
  const auto brake_before_follow_hold = mock->brakeLockCalls();
  auto hold_after_follow = command;
  hold_after_follow.sequence++;
  hold_after_follow.mode = Command::MODE_HOLD;
  hold_after_follow.field_mask = Command::FIELD_POSITION;
  hold_after_follow.velocity.fill(0.0);
  hold_after_follow.acceleration.fill(0.0);
  observer->command_publisher_->publish(hold_after_follow);
  ASSERT_TRUE(observer->waitFor(
      [&hold_after_follow](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == hold_after_follow.sequence;
      }, 1s));
  ASSERT_TRUE(observer->waitFor(
      [&mock, brake_before_follow_hold](const Status &, const Feedback &) {
        return mock->brakeLockCalls() == brake_before_follow_hold;
      }, 1s));
  for (int frame = 0; frame < 20; ++frame) {
    ++hold_after_follow.sequence;
    observer->command_publisher_->publish(hold_after_follow);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(mock->brakeLockCalls(), brake_before_follow_hold);

  const auto forwarded = mock->lastInternalCommand();
  EXPECT_EQ(forwarded.field_mask, hold_after_follow.field_mask);
  EXPECT_EQ(forwarded.position, hold_after_follow.position);
  EXPECT_EQ(forwarded.velocity, hold_after_follow.velocity);
  EXPECT_EQ(forwarded.acceleration, hold_after_follow.acceleration);

  const auto mapped = observer->feedback();
  EXPECT_EQ(mapped.position_valid_mask, 0x7fffU);
  EXPECT_EQ(mapped.velocity_valid_mask, 0x7fffU);
  EXPECT_DOUBLE_EQ(mapped.position[0], -0.25);
  for (std::size_t index = 1; index < mapped.position.size(); ++index) {
    EXPECT_DOUBLE_EQ(mapped.position[index], 0.1 * static_cast<double>(index));
  }
  EXPECT_EQ(mapped.arm_torque_enabled_mask, 0U);
  EXPECT_FALSE(mapped.lift_drive_enabled);

  EXPECT_EQ(observer->status().state, Status::STATE_LEASED_HOLD);

  auto invalid_velocity = command;
  invalid_velocity.sequence++;
  invalid_velocity.velocity[0] = std::numeric_limits<double>::infinity();
  observer->command_publisher_->publish(invalid_velocity);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.rejected_commands >= 2;}, 1s));

  auto excessive_acceleration = command;
  excessive_acceleration.sequence += 2;
  excessive_acceleration.acceleration[1] = 1.01;
  observer->command_publisher_->publish(excessive_acceleration);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.rejected_commands >= 3;}, 1s));

  auto unknown_field = command;
  unknown_field.sequence += 3;
  unknown_field.field_mask |= 0x80U;
  observer->command_publisher_->publish(unknown_field);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.rejected_commands >= 4;}, 1s));

  auto duplicate = hold_after_follow;
  ++duplicate.sequence;
  observer->command_publisher_->publish(duplicate);
  ASSERT_TRUE(observer->waitFor(
      [&duplicate](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == duplicate.sequence;
      }, 1s));
  observer->command_publisher_->publish(duplicate);
  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {return status.duplicate_commands >= 1;},
      1s));
  auto out_of_order = command;
  out_of_order.sequence = acquired->initial_sequence - 1;
  observer->command_publisher_->publish(out_of_order);
  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {
        return status.out_of_order_commands >= 1;
      }, 1s));

  // Lease expiry is independently safe: after a deliberate release, the
  // expiry watchdog must issue exactly one native lock even though it runs at
  // the feedback rate.
  brake_request->release = true;
  const auto released_before_expiry = callService<LiftBrake>(brake_client, brake_request);
  ASSERT_NE(released_before_expiry, nullptr);
  ASSERT_TRUE(released_before_expiry->accepted) << released_before_expiry->message;
  const auto locks_before_expiry = mock->brakeLockCalls();
  ASSERT_TRUE(
    observer->waitFor(
      [&mock, locks_before_expiry](const Status & status, const Feedback &) {
        return status.active_lease_id.empty() && status.active_owner_id.empty() &&
        mock->brakeLockCalls() == locks_before_expiry + 1 &&
        status.detail.find("execution lease expired after 500 ms") != std::string::npos;
      }, 1s));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(mock->brakeLockCalls(), locks_before_expiry + 1);
  ASSERT_TRUE(
    observer->waitFor(
      [](const Status & status, const Feedback &) {
        return status.active_lease_id.empty() && status.active_owner_id.empty();
      }, 2s));

  const auto reacquired = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(reacquired, nullptr);
  ASSERT_TRUE(reacquired->granted) << reacquired->message;
  ASSERT_TRUE(mock->waitForInternalCommands(4, 1s));

  // MODE_STOP is also an edge-triggered safety operation.  Repeated STOP
  // keepalives must not replay the physical lock once it is already latched.
  brake_request->release = true;
  const auto released_before_stop = callService<LiftBrake>(brake_client, brake_request);
  ASSERT_NE(released_before_stop, nullptr);
  ASSERT_TRUE(released_before_stop->accepted) << released_before_stop->message;
  Command stop = hold;
  stop.owner_id = acquire_request->owner_id;
  stop.lease_id = reacquired->lease_id;
  stop.sequence = reacquired->initial_sequence;
  stop.mode = Command::MODE_STOP;
  stop.field_mask = 0U;
  const auto locks_before_stop = mock->brakeLockCalls();
  observer->command_publisher_->publish(stop);
  ASSERT_TRUE(observer->waitFor(
      [&stop](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == stop.sequence;
      }, 1s));
  ASSERT_TRUE(observer->waitFor(
      [&mock, locks_before_stop](const Status &, const Feedback &) {
        return mock->brakeLockCalls() == locks_before_stop + 1;
      }, 1s));
  ++stop.sequence;
  observer->command_publisher_->publish(stop);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(mock->brakeLockCalls(), locks_before_stop + 1);

  using Release = robot_control_msg::srv::ReleaseHeavyExecutionLeaseV1;
  auto release_client = clients->create_client<Release>(contract::kReleaseLeaseService.data());
  auto release_request = std::make_shared<Release::Request>();
  release_request->owner_id = acquire_request->owner_id;
  release_request->lease_id = "wrong-lease";
  const auto wrong_release = callService<Release>(release_client, release_request);
  ASSERT_NE(wrong_release, nullptr);
  EXPECT_FALSE(wrong_release->released);
  release_request->lease_id = reacquired->lease_id;
  const auto released = callService<Release>(release_client, release_request);
  ASSERT_NE(released, nullptr);
  EXPECT_TRUE(released->released) << released->message;

  // A workspace STOP is a distinct safety path: it must HOLD and clear the
  // complete lease immediately rather than waiting for the 500 ms watchdog.
  const auto workspace_lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(workspace_lease, nullptr);
  ASSERT_TRUE(workspace_lease->granted) << workspace_lease->message;
  Command workspace_hold = hold;
  workspace_hold.owner_id = acquire_request->owner_id;
  workspace_hold.lease_id = workspace_lease->lease_id;
  workspace_hold.sequence = workspace_lease->initial_sequence;
  observer->command_publisher_->publish(workspace_hold);
  ASSERT_TRUE(observer->waitFor(
      [&workspace_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == workspace_hold.sequence;
      }, 1s));
  mock->setWorkspaceRunning(false);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback & feedback) {
        return status.active_owner_id.empty() && status.active_lease_id.empty() &&
               feedback.active_owner_id.empty() && feedback.active_lease_id.empty() &&
               status.detail.find("workspace_stop:") != std::string::npos;
      }, 1s));
  mock->setWorkspaceRunning(true);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      1s));

  // A lower-controller feedback outage must revoke the execution lease
  // before exposing incomplete masks to the remote hardware interface.
  const auto fault_lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(fault_lease, nullptr);
  ASSERT_TRUE(fault_lease->granted) << fault_lease->message;
  brake_request->release = true;
  const auto released_before_fault = callService<LiftBrake>(brake_client, brake_request);
  ASSERT_NE(released_before_fault, nullptr);
  ASSERT_TRUE(released_before_fault->accepted) << released_before_fault->message;
  const auto locks_before_fault = mock->brakeLockCalls();
  mock->setArmJointPublishing(false);
  ASSERT_TRUE(observer->waitFor(
      [&mock, locks_before_fault](const Status & status, const Feedback & feedback) {
        return status.active_owner_id.empty() && status.active_lease_id.empty() &&
        feedback.active_owner_id.empty() && feedback.active_lease_id.empty() &&
        feedback.position_valid_mask == 0x0001U && feedback.velocity_valid_mask == 0x0001U &&
        (feedback.resource_fault_mask & 0x7ffeU) == 0x7ffeU &&
        mock->brakeLockCalls() == locks_before_fault + 1;
      }, 2s));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(mock->brakeLockCalls(), locks_before_fault + 1);
  mock->setArmJointPublishing(true);
  ASSERT_TRUE(observer->waitFor(
      [](const Status &, const Feedback & feedback) {
        return feedback.position_valid_mask == 0x7fffU &&
        feedback.velocity_valid_mask == 0x7fffU && feedback.resource_fault_mask == 0U;
      }, 2s));

  // A new owner/lease gets one lock on its first HOLD.  The later keepalives
  // must not replay that physical transition.
  brake_request->release = true;
  const auto released_brake = callService<LiftBrake>(brake_client, brake_request);
  ASSERT_NE(released_brake, nullptr);
  ASSERT_TRUE(released_brake->accepted) << released_brake->message;
  acquire_request->owner_id = "owner-switch";
  const auto switched_lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(switched_lease, nullptr);
  ASSERT_TRUE(switched_lease->granted) << switched_lease->message;
  Command switched_hold = hold;
  switched_hold.owner_id = acquire_request->owner_id;
  switched_hold.lease_id = switched_lease->lease_id;
  switched_hold.sequence = switched_lease->initial_sequence;
  const auto locks_before_switch_hold = mock->brakeLockCalls();
  observer->command_publisher_->publish(switched_hold);
  ASSERT_TRUE(observer->waitFor(
      [&switched_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == switched_hold.sequence;
      }, 1s));
  ASSERT_TRUE(observer->waitFor(
      [&mock, locks_before_switch_hold](const Status &, const Feedback &) {
        return mock->brakeLockCalls() == locks_before_switch_hold + 1;
      }, 1s));
  for (int frame = 0; frame < 5; ++frame) {
    ++switched_hold.sequence;
    observer->command_publisher_->publish(switched_hold);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(mock->brakeLockCalls(), locks_before_switch_hold + 1);

  // A controller can be briefly busy without invalidating a healthy lease.
  // While arm/lift ACK takes 120 ms, the gateway keeps a latest-wins staged
  // slot and dispatches every confirmed command serially.  This must remain
  // one owner/lease, rather than alternating between false expiry and reacquire.
  ASSERT_TRUE(observer->waitFor(
      [&switched_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == switched_hold.sequence;
      }, 1s));
  mock->setAcknowledgementDelay(120ms);
  const auto delayed_ack_baseline = observer->status();
  auto delayed_ack_hold = switched_hold;
  for (int frame = 0; frame < 70; ++frame) {
    ++delayed_ack_hold.sequence;
    delayed_ack_hold.header.stamp = observer->now();
    observer->command_publisher_->publish(delayed_ack_hold);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(observer->waitFor(
      [&switched_lease, &delayed_ack_baseline](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "owner-switch" &&
               status.active_lease_id == switched_lease->lease_id &&
               status.accepted_commands >= delayed_ack_baseline.accepted_commands + 4;
      }, 1s));
  mock->setAcknowledgementDelay(0ms);
  ASSERT_TRUE(observer->waitFor(
      [&delayed_ack_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == delayed_ack_hold.sequence;
      }, 1s));
  switched_hold = delayed_ack_hold;

  // A single lost lift ACK must be recovered by retransmitting the same
  // internal generation. The retry is not a new external command and must
  // preserve the active owner/lease.
  mock->dropNextLiftAcknowledgement();
  auto retried_ack_hold = switched_hold;
  ++retried_ack_hold.sequence;
  retried_ack_hold.header.stamp = observer->now();
  observer->command_publisher_->publish(retried_ack_hold);
  ASSERT_TRUE(observer->waitFor(
      [&retried_ack_hold, &switched_lease](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "owner-switch" &&
               status.active_lease_id == switched_lease->lease_id &&
               feedback.applied_command_sequence == retried_ack_hold.sequence;
      }, 1s));
  switched_hold = retried_ack_hold;

  // Intermittent FOLLOW gaps are a motion watchdog case, not a lease-loss
  // case.  Two 250 ms gaps must each enter aligned HOLD with the same lease,
  // and the same owner must be able to resume FOLLOW afterwards.
  auto intermittent_follow = switched_hold;
  ++intermittent_follow.sequence;
  intermittent_follow.mode = Command::MODE_FOLLOW_POSITION;
  observer->command_publisher_->publish(intermittent_follow);
  ASSERT_TRUE(observer->waitFor(
      [&intermittent_follow](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_FOLLOWING &&
               feedback.applied_command_sequence == intermittent_follow.sequence;
      }, 1s));
  std::this_thread::sleep_for(250ms);
  ASSERT_TRUE(observer->waitFor(
      [&switched_lease](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "owner-switch" &&
               status.active_lease_id == switched_lease->lease_id &&
               feedback.active_owner_id == status.active_owner_id &&
               feedback.active_lease_id == status.active_lease_id;
      }, 1s));
  ++intermittent_follow.sequence;
  observer->command_publisher_->publish(intermittent_follow);
  ASSERT_TRUE(observer->waitFor(
      [&intermittent_follow](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_FOLLOWING &&
               feedback.applied_command_sequence == intermittent_follow.sequence;
      }, 1s));
  std::this_thread::sleep_for(250ms);
  ASSERT_TRUE(observer->waitFor(
      [&switched_lease](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_owner_id == "owner-switch" &&
               status.active_lease_id == switched_lease->lease_id &&
               feedback.active_owner_id == status.active_owner_id &&
               feedback.active_lease_id == status.active_lease_id;
      }, 1s));
  auto recovered_hold = intermittent_follow;
  ++recovered_hold.sequence;
  recovered_hold.mode = Command::MODE_HOLD;
  observer->command_publisher_->publish(recovered_hold);
  ASSERT_TRUE(observer->waitFor(
      [&recovered_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == recovered_hold.sequence;
      }, 1s));
  switched_hold = recovered_hold;

  // A sustained stream of malformed frames is still DDS activity, but it is
  // not execution activity.  It must neither advance applied/accepted state
  // nor keep this lease alive past the normal 500 ms accepted-command limit.
  ASSERT_TRUE(observer->waitFor(
      [&switched_hold](const Status &, const Feedback & feedback) {
        return feedback.applied_command_sequence == switched_hold.sequence;
      }, 1s));
  const auto invalid_flood_baseline = observer->status();
  const auto applied_before_invalid_flood = observer->feedback().applied_command_sequence;
  auto invalid_flood = switched_hold;
  invalid_flood.layout_crc32 = 0;
  for (int frame = 0; frame < 40; ++frame) {
    ++invalid_flood.sequence;
    invalid_flood.header.stamp = observer->now();
    observer->command_publisher_->publish(invalid_flood);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(observer->waitFor(
      [&invalid_flood_baseline](const Status & status, const Feedback &) {
        return status.rejected_commands > invalid_flood_baseline.rejected_commands;
      }, 1s));
  EXPECT_EQ(observer->status().accepted_commands, invalid_flood_baseline.accepted_commands);
  EXPECT_EQ(observer->feedback().applied_command_sequence, applied_before_invalid_flood);
  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback & feedback) {
        return status.active_owner_id.empty() && status.active_lease_id.empty() &&
               feedback.active_owner_id.empty() && feedback.active_lease_id.empty() &&
               status.detail.find("execution lease expired after 500 ms") != std::string::npos;
      }, 1s));

  // ACK loss must be visible as a controller fault even while fresh valid
  // commands continue to arrive.  Only the first frame is dispatched; later
  // frames replace the staged slot and cannot continually postpone the ACK
  // timeout or create a false accepted-command lease refresh.
  acquire_request->owner_id = "ack-stall";
  const auto stalled_lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(stalled_lease, nullptr);
  ASSERT_TRUE(stalled_lease->granted) << stalled_lease->message;
  ASSERT_TRUE(observer->waitFor(
      [&stalled_lease](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_lease_id == stalled_lease->lease_id;
      }, 1s));
  mock->setAcknowledgementsEnabled(false);
  Command stalled_hold = hold;
  stalled_hold.owner_id = acquire_request->owner_id;
  stalled_hold.lease_id = stalled_lease->lease_id;
  stalled_hold.sequence = stalled_lease->initial_sequence;
  const auto stalled_baseline = observer->status();
  observer->command_publisher_->publish(stalled_hold);
  for (int frame = 0; frame < 35; ++frame) {
    ++stalled_hold.sequence;
    stalled_hold.header.stamp = observer->now();
    observer->command_publisher_->publish(stalled_hold);
    std::this_thread::sleep_for(10ms);
  }
  const bool stalled_released = observer->waitFor(
      [&stalled_lease, &stalled_baseline](const Status & status, const Feedback &) {
      return status.active_owner_id.empty() &&
               status.active_lease_id.empty() &&
               status.accepted_commands == stalled_baseline.accepted_commands + 1 &&
               status.detail.find("automatic release") != std::string::npos &&
               // The automatic-release path composes its own reason
               // ("automatic release: controller HOLD ack timeout; lease revoked
               // after STOP/HOLD submission"), so it names the ack timeout as
               // "controller HOLD ack timeout".  The test used to look for the
               // FAULT-branch wording ("lower controller apply acknowledgement
               // timeout"), which never appears on this path even though the
               // ack-timeout release happens exactly as intended.
               status.detail.find("controller HOLD ack timeout") != std::string::npos;
      }, 1s);
  // Report what the gateway actually did instead of only "condition not met":
  // a lease that stays held, a different accepted-command count and a different
  // detail are three different bugs and the bare waitFor hides which one it is.
  ASSERT_TRUE(stalled_released)
    << "state=" << observer->status().state
    << " owner='" << observer->status().active_owner_id << "'"
    << " lease='" << observer->status().active_lease_id << "'"
    << " accepted=" << observer->status().accepted_commands
    << " baseline=" << stalled_baseline.accepted_commands
    << " detail='" << observer->status().detail << "'";
  EXPECT_NE(observer->status().detail.find("received_commands="), std::string::npos);
  EXPECT_NE(
    observer->status().detail.find("last_received_sequence=" +
    std::to_string(stalled_hold.sequence)), std::string::npos);

}

TEST(HeavyGatewayV1, AllowsOptInLiftOnlyFollowWithDisabledArmsHoldingPosition)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides({
    rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake"),
    rclcpp::Parameter("heavy_v1.allow_lift_only_follow", true),
    rclcpp::Parameter("heavy_v1.lift_only_arm_hold_position_tolerance_rad", 0.001)});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_lift_only_follow", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  mock->setWorkspaceMode(robot_control_msg::msg::WorkspaceStatus::REAL);
  mock->setArmActuatorsEnabled(false);
  mock->setLiftEnabled(true);
  mock->setBrakeUnlocked(true);
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_lift_only_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "lift-only-follow";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(lease, nullptr);
  ASSERT_TRUE(lease->granted) << lease->message;
  ASSERT_TRUE(observer->waitFor(
      [&lease](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_lease_id == lease->lease_id;
      }, 1s));

  Command follow;
  follow.header.stamp = observer->now();
  follow.protocol_major = 1;
  follow.protocol_minor = 0;
  follow.layout_crc32 = contract::kLayoutCrc32;
  follow.owner_id = acquire_request->owner_id;
  follow.lease_id = lease->lease_id;
  follow.sequence = lease->initial_sequence;
  follow.mode = Command::MODE_FOLLOW_POSITION;
  follow.field_mask = Command::FIELD_POSITION | Command::FIELD_VELOCITY |
    Command::FIELD_ACCELERATION;
  follow.position[0] = -0.20;
  follow.velocity[0] = 0.05;
  follow.acceleration[0] = 0.10;
  for (std::size_t index = 1; index < follow.position.size(); ++index) {
    follow.position[index] = 0.1 * static_cast<double>(index);
  }

  observer->command_publisher_->publish(follow);
  ASSERT_TRUE(observer->waitFor(
      [&follow](const Status & status, const Feedback & feedback) {
        return status.state == Status::STATE_FOLLOWING &&
               feedback.applied_command_sequence == follow.sequence;
      }, 1s));
  const auto forwarded = mock->lastInternalCommand();
  EXPECT_DOUBLE_EQ(forwarded.position[0], follow.position[0]);
  EXPECT_DOUBLE_EQ(forwarded.velocity[0], follow.velocity[0]);
  EXPECT_DOUBLE_EQ(forwarded.acceleration[0], follow.acceleration[0]);

  Command unsafe_arm_target = follow;
  ++unsafe_arm_target.sequence;
  unsafe_arm_target.header.stamp = observer->now();
  unsafe_arm_target.position[1] += 0.002;
  const auto baseline = observer->status();
  observer->command_publisher_->publish(unsafe_arm_target);
  ASSERT_TRUE(observer->waitFor(
      [&baseline](const Status & status, const Feedback &) {
        return status.rejected_commands == baseline.rejected_commands + 1 &&
               status.detail.find("lift-only FOLLOW_POSITION requires safe arm hold") !=
               std::string::npos;
      }, 1s));
  EXPECT_EQ(observer->status().accepted_commands, baseline.accepted_commands);
}

TEST(HeavyGatewayV1, RejectsRealFollowWhenALockedBrakeHasNoNativeReleaseRoute)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides(
    {rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "")});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_missing_brake_route", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  mock->setWorkspaceMode(robot_control_msg::msg::WorkspaceStatus::REAL);
  mock->setSimulatedActuatorsEnabled(true);
  mock->setBrakeUnlocked(true);
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_missing_brake_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "missing-brake-route";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto lease = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(lease, nullptr);
  ASSERT_TRUE(lease->granted) << lease->message;
  ASSERT_TRUE(observer->waitFor(
      [&lease](const Status & status, const Feedback &) {
        return status.state == Status::STATE_LEASED_HOLD &&
               status.active_lease_id == lease->lease_id;
      }, 1s));

  // A real feedback transition reports that the brake is locked after the
  // lease alignment.  With no native service configured, FOLLOW must fail in
  // the gateway callback and never become an unacknowledged controller job.
  mock->setBrakeUnlocked(false);
  std::this_thread::sleep_for(30ms);
  Command follow;
  follow.header.stamp = observer->now();
  follow.protocol_major = 1;
  follow.protocol_minor = 0;
  follow.layout_crc32 = contract::kLayoutCrc32;
  follow.owner_id = acquire_request->owner_id;
  follow.lease_id = lease->lease_id;
  follow.sequence = lease->initial_sequence;
  follow.mode = Command::MODE_FOLLOW_POSITION;
  follow.field_mask = Command::FIELD_POSITION | Command::FIELD_VELOCITY |
    Command::FIELD_ACCELERATION;
  follow.position[0] = -0.25;
  for (std::size_t index = 1; index < follow.position.size(); ++index) {
    follow.position[index] = 0.1 * static_cast<double>(index);
  }
  follow.velocity.fill(0.0);
  follow.acceleration.fill(0.0);
  const auto baseline = observer->status();
  observer->command_publisher_->publish(follow);
  ASSERT_TRUE(observer->waitFor(
      [&baseline](const Status & status, const Feedback &) {
        return status.rejected_commands == baseline.rejected_commands + 1 &&
               status.detail.find("native lift brake service is not configured") !=
               std::string::npos;
      }, 1s));
  EXPECT_EQ(observer->status().accepted_commands, baseline.accepted_commands);
  EXPECT_NE(observer->status().detail.find("not submitted"), std::string::npos);
  EXPECT_NE(observer->status().detail.find("applied_sequence=0"), std::string::npos);
  std::this_thread::sleep_for(300ms);
  EXPECT_NE(observer->status().state, Status::STATE_FAULT);
  EXPECT_EQ(mock->brakeReleaseCalls(), 0U);
}

TEST(HeavyGatewayV1, RejectsLeaseBeforeBothLowerControllerEndpointsAreReady)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides(
    {rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake")});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_endpoint_readiness", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_endpoint_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));
  mock->disconnectHeavyEndpoints();
  std::this_thread::sleep_for(200ms);

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto request = std::make_shared<Acquire::Request>();
  request->owner_id = "controllers-not-ready";
  request->protocol_major = 1;
  request->protocol_minor = 0;
  request->layout_crc32 = contract::kLayoutCrc32;
  const auto response = callService<Acquire>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->granted);
  EXPECT_NE(response->message.find("lower controller Heavy endpoints are not ready"),
    std::string::npos);
  EXPECT_NE(response->message.find("command_subscriptions=0/2"), std::string::npos);
  EXPECT_EQ(observer->status().accepted_commands, 0U);
}

TEST(HeavyGatewayV1, DropsLeaseWhenTheWatchdogExpiresWithTheAckStillMissing)
{
  // Fault injection: the release HOLD is submitted but the controllers never
  // acknowledge it.  Exercising that state used to leave the gateway holding a
  // lease the lower side had already dropped ("owner or lease mismatch" forever,
  // recoverable only by restarting the stack).  Whatever path clears it -- the
  // bounded release fallback or the execution watchdog -- the lease must be gone
  // within a few seconds and a different owner must be able to take over.
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides(
    {rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake")});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_watchdog_without_ack", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_watchdog_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "watchdog-no-ack-owner";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto first = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->granted) << first->message;
  const auto held_lease = first->lease_id;

  // The controllers stop acknowledging, so nothing but the watchdogs can end the
  // lease from here on.
  mock->setAcknowledgementsEnabled(false);

  using Release = robot_control_msg::srv::ReleaseHeavyExecutionLeaseV1;
  auto release_client = clients->create_client<Release>(contract::kReleaseLeaseService.data());
  auto release_request = std::make_shared<Release::Request>();
  release_request->owner_id = acquire_request->owner_id;
  release_request->lease_id = held_lease;
  const auto released = callService<Release>(release_client, release_request);
  ASSERT_NE(released, nullptr);

  // Cleanup must converge without a restart, whichever fallback fires first.
  const auto deadline = std::chrono::steady_clock::now() + 6s;
  bool lease_cleared = false;
  while (std::chrono::steady_clock::now() < deadline) {
    if (observer->status().active_owner_id.empty() &&
      observer->status().active_lease_id.empty())
    {
      lease_cleared = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(lease_cleared)
    << "lease survived a dropped release acknowledgement: owner='"
    << observer->status().active_owner_id << "' lease='"
    << observer->status().active_lease_id << "' detail=" << observer->status().detail;

  auto next_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto next_request = std::make_shared<Acquire::Request>();
  next_request->owner_id = "owner-after-watchdog-expiry";
  next_request->protocol_major = 1;
  next_request->protocol_minor = 0;
  next_request->layout_crc32 = contract::kLayoutCrc32;
  const auto next = callService<Acquire>(next_client, next_request);
  ASSERT_NE(next, nullptr);
  EXPECT_TRUE(next->granted) << next->message;
}

TEST(HeavyGatewayV1, ClearsLeaseWhenReleaseHoldIsNotAcknowledged)
{
  // Regression: when the HOLD submitted for an explicit release was not
  // acknowledged within controller_ack_timeout_ms, the gateway only switched to
  // FAULT and left the revocation pending.  The lease then stayed active with a
  // stale owner and every new acquire was refused, so `lift power on` kept
  // failing until the stack was restarted.
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions gateway_options;
  gateway_options.parameter_overrides(
    {rclcpp::Parameter("heavy_v1.heavy_lift_brake_native_service", "/mock/lift_brake")});
  auto gateway_node = std::make_shared<rclcpp::Node>(
    "heavy_gateway_v1_release_ack_timeout", gateway_options);
  auto gateway = std::make_unique<robot_lower_gateway::HeavyGatewayV1>(*gateway_node);
  auto mock = std::make_shared<MockLowerControllers>();
  auto observer = std::make_shared<ObservationNode>();
  auto clients = std::make_shared<rclcpp::Node>("heavy_gateway_v1_release_clients");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(gateway_node);
  executor.add_node(mock);
  executor.add_node(observer);
  executor.add_node(clients);
  ExecutorGuard executor_guard(executor);

  ASSERT_TRUE(observer->waitFor(
      [](const Status & status, const Feedback &) {return status.state == Status::STATE_MONITOR;},
      2s));

  using Acquire = robot_control_msg::srv::AcquireHeavyExecutionLeaseV1;
  auto acquire_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto acquire_request = std::make_shared<Acquire::Request>();
  acquire_request->owner_id = "release-timeout-owner";
  acquire_request->protocol_major = 1;
  acquire_request->protocol_minor = 0;
  acquire_request->layout_crc32 = contract::kLayoutCrc32;
  const auto acquired = callService<Acquire>(acquire_client, acquire_request);
  ASSERT_NE(acquired, nullptr);
  ASSERT_TRUE(acquired->granted) << acquired->message;

  // The release HOLD will never be acknowledged, so the bounded ACK timeout path
  // is what must clear the lease.
  mock->setAcknowledgementsEnabled(false);

  using Release = robot_control_msg::srv::ReleaseHeavyExecutionLeaseV1;
  auto release_client = clients->create_client<Release>(contract::kReleaseLeaseService.data());
  auto release_request = std::make_shared<Release::Request>();
  release_request->owner_id = acquire_request->owner_id;
  release_request->lease_id = acquired->lease_id;
  const auto released = callService<Release>(release_client, release_request);
  ASSERT_NE(released, nullptr);
  // The release must finish deterministically instead of only flipping to FAULT.
  EXPECT_TRUE(released->released) << released->message;
  EXPECT_NE(released->message.find("ack timeout"), std::string::npos) << released->message;
  // The stuck revocation must also be visible in diagnostics, not only in the
  // service reply.
  EXPECT_NE(
    observer->status().detail.find("explicit_release_ack_timeout"), std::string::npos)
    << observer->status().detail;

  auto next_client = clients->create_client<Acquire>(contract::kAcquireLeaseService.data());
  auto next_request = std::make_shared<Acquire::Request>();
  next_request->owner_id = "owner-after-release-timeout";
  next_request->protocol_major = 1;
  next_request->protocol_minor = 0;
  next_request->layout_crc32 = contract::kLayoutCrc32;
  const auto next_acquired = callService<Acquire>(next_client, next_request);
  ASSERT_NE(next_acquired, nullptr);
  EXPECT_TRUE(next_acquired->granted) << next_acquired->message;
}

}  // namespace

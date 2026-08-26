#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "joint_hardware/lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_command_v1.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{

void spin_delivery(rclcpp::executors::SingleThreadedExecutor & executor)
{
  for (int attempt = 0; attempt < 10; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

class ExecutorThreadGuard
{
public:
  explicit ExecutorThreadGuard(rclcpp::executors::SingleThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();}) {}

  ~ExecutorThreadGuard()
  {
    stop();
  }

  void stop()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::thread thread_;
};

std_msgs::msg::String driver_status(
  bool estop, bool brake_unlocked = true, int error_code = 0,
  bool motion_blocked = false, bool quick_stop = false)
{
  std_msgs::msg::String status;
  status.data =
    std::string("{\"feedback_fresh\":true,\"ethercat_operational\":true,") +
    "\"working_counter_ok\":true,\"cia402_state\":\"operation_enabled\"," +
    "\"brake_unlocked_inferred\":" + (brake_unlocked ? "true" : "false") +
    ",\"error_code\":" + std::to_string(error_code) + ",\"mode_display\":9," +
    "\"motion_blocked\":" + (motion_blocked ? "true" : "false") +
    ",\"quick_stop_active\":" + (quick_stop ? "true" : "false") + "," +
    "\"estop_latched\":" + (estop ? "true}" : "false}");
  return status;
}

trajectory_msgs::msg::JointTrajectory trajectory_to(double target)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_motor"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target};
  point.velocities = {0.0};
  point.accelerations = {0.0};
  point.time_from_start.sec = 1;
  trajectory.points.push_back(point);
  return trajectory;
}

TEST(LiftController, SlowPositionCommandDoesNotTreatIntermediateSampleAsFinalTarget)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<joint_hardware::LiftController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("brake_gate_stable_sec", 0.1),
      rclcpp::Parameter("driver_status_timeout_sec", 2.0),
      rclcpp::Parameter("target_stable_sec", 0.0),
      rclcpp::Parameter("goal_tolerance_m", 0.001),
      rclcpp::Parameter("stationary_velocity_mps", 0.001),
    });
  ASSERT_EQ(
    controller->init("lift_slow_position_test", "", options),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  double command_position = -0.10;
  double command_velocity = 0.0;
  double command_acceleration = 0.0;
  double command_power_enable = 0.0;
  double state_position = -0.10;
  double state_velocity = 0.0;
  double state_power_enable = 1.0;
  hardware_interface::CommandInterface position_command(
    "joint_motor", "position", &command_position);
  hardware_interface::CommandInterface velocity_command(
    "joint_motor", "velocity", &command_velocity);
  hardware_interface::CommandInterface acceleration_command(
    "joint_motor", "acceleration", &command_acceleration);
  hardware_interface::CommandInterface power_enable_command(
    "joint_motor", "power_enable", &command_power_enable);
  hardware_interface::StateInterface position_state(
    "joint_motor", "position", &state_position);
  hardware_interface::StateInterface velocity_state(
    "joint_motor", "velocity", &state_velocity);
  hardware_interface::StateInterface power_enable_state(
    "joint_motor", "power_enable", &state_power_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(position_command);
  commands.emplace_back(velocity_command);
  commands.emplace_back(acceleration_command);
  commands.emplace_back(power_enable_command);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(power_enable_state);
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  auto io_node = std::make_shared<rclcpp::Node>("lift_slow_position_test_io");
  auto status_publisher = io_node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  auto command_client = io_node->create_client<robot_control_msg::srv::SelectedJointControl>(
    "/joint/lift/command");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  executor.add_node(io_node);
  status_publisher->publish(driver_status(false));
  spin_delivery(executor);

  auto request = std::make_shared<robot_control_msg::srv::SelectedJointControl::Request>();
  request->joint_names = {"joint_motor"};
  request->values = {-0.005};
  request->relative = true;
  request->vel = 0.001;
  request->acc = 0.01;
  auto response = command_client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(response, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(response.get()->accepted);

  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(
    controller->update(rclcpp::Time(0, 0, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  ASSERT_EQ(
    controller->update(rclcpp::Time(10'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);

  bool motion_sample_seen = false;
  for (int cycle = 0; cycle < 30; ++cycle) {
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((20 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
    motion_sample_seen = motion_sample_seen || command_position < state_position - 1.0e-8;
    EXPECT_GT(command_position, -0.105 + 0.001);
  }
  spin_delivery(executor);

  EXPECT_TRUE(motion_sample_seen);
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);
  EXPECT_LT(command_position, state_position);

  EXPECT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  executor.remove_node(io_node);
  executor.remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  rclcpp::shutdown();
}

TEST(LiftController, BrakeGateAndEstopResetDoNotAdvanceOldTrajectory)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<joint_hardware::LiftController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("brake_gate_stable_sec", 0.1),
      rclcpp::Parameter("driver_status_timeout_sec", 2.0),
      rclcpp::Parameter("jog_timeout_sec", 0.05),
      rclcpp::Parameter("target_stable_sec", 0.0),
    });
  ASSERT_EQ(
    controller->init("lift_controller_test", "", options),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  double command_position = 0.25;
  double command_velocity = 0.25;
  double command_acceleration = 0.25;
  double command_power_enable = 1.0;
  double state_position = -0.50;
  double state_velocity = 0.0;
  double state_power_enable = 1.0;
  hardware_interface::CommandInterface position_command(
    "joint_motor", "position", &command_position);
  hardware_interface::CommandInterface velocity_command(
    "joint_motor", "velocity", &command_velocity);
  hardware_interface::CommandInterface acceleration_command(
    "joint_motor", "acceleration", &command_acceleration);
  hardware_interface::CommandInterface power_enable_command(
    "joint_motor", "power_enable", &command_power_enable);
  hardware_interface::StateInterface position_state(
    "joint_motor", "position", &state_position);
  hardware_interface::StateInterface velocity_state(
    "joint_motor", "velocity", &state_velocity);
  hardware_interface::StateInterface power_enable_state(
    "joint_motor", "power_enable", &state_power_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(position_command);
  commands.emplace_back(velocity_command);
  commands.emplace_back(acceleration_command);
  commands.emplace_back(power_enable_command);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(power_enable_state);
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Activation aligns motion commands and starts with power disabled.
  // any values left in the interfaces by an earlier controller instance.
  EXPECT_DOUBLE_EQ(command_position, state_position);
  EXPECT_DOUBLE_EQ(command_velocity, 0.0);
  EXPECT_DOUBLE_EQ(command_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(command_power_enable, 0.0);

  auto io_node = std::make_shared<rclcpp::Node>("lift_controller_test_io");
  auto status_publisher = io_node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  auto trajectory_publisher =
    io_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint/lift/trajectory", rclcpp::QoS(1).reliable());
  auto jog_publisher = io_node->create_publisher<std_msgs::msg::Float64>(
    "/joint/lift/jog_velocity", rclcpp::QoS(1).best_effort());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  executor.add_node(io_node);

  trajectory_publisher->publish(trajectory_to(-0.40));
  spin_delivery(executor);
  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 20; ++cycle) {
    const rclcpp::Time time(cycle * 10'000'000LL, RCL_ROS_TIME);
    ASSERT_EQ(controller->update(time, period), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(command_position, state_position);
    EXPECT_DOUBLE_EQ(command_velocity, 0.0);
    EXPECT_DOUBLE_EQ(command_acceleration, 0.0);
  }

  status_publisher->publish(driver_status(false));
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(rclcpp::Time(210'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  ASSERT_EQ(
    controller->update(rclcpp::Time(220'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  bool ruckig_motion_seen = false;
  for (int cycle = 0; cycle < 30; ++cycle) {
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((230 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
    ruckig_motion_seen = ruckig_motion_seen || std::abs(command_velocity) > 1.0e-8;
    EXPECT_LE(std::abs(command_velocity), 0.020 + 1.0e-9);
    EXPECT_LE(std::abs(command_acceleration), 0.033333333 + 1.0e-9);
  }
  EXPECT_TRUE(ruckig_motion_seen);
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);

  // Let the mock state follow the synchronized controller outputs. Once the
  // final target and measured velocity are stable, normal completion must keep
  // the servo enabled so it can hold the vertical load until an explicit stop
  // or power-off request.
  for (int cycle = 0; cycle < 1000; ++cycle) {
    state_position = command_position;
    state_velocity = command_velocity;
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((530 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
    if (cycle % 10 == 0) {
      spin_delivery(executor);
    }
    if (cycle > 300 && std::abs(command_velocity) <= 1.0e-9) {
      break;
    }
  }
  spin_delivery(executor);
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);

  status_publisher->publish(driver_status(true));
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(rclcpp::Time(600'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_position, state_position);
  EXPECT_DOUBLE_EQ(command_velocity, 0.0);
  EXPECT_DOUBLE_EQ(command_acceleration, 0.0);

  // A trajectory received while latched is rejected. Clearing the reported
  // latch represents a successful hardware safety reset and must only enter
  // HOLD; the pre-estop target must not resume.
  trajectory_publisher->publish(trajectory_to(-0.30));
  spin_delivery(executor);
  status_publisher->publish(driver_status(false));
  spin_delivery(executor);
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((610 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(command_position, state_position);
    EXPECT_DOUBLE_EQ(command_velocity, 0.0);
    EXPECT_DOUBLE_EQ(command_acceleration, 0.0);
  }

  std_msgs::msg::Float64 jog;
  jog.data = 0.04;
  // Jog is a 50 Hz keepalive.  Re-publish while the brake gate is settling;
  // these updates must not restart the gate stability timer.
  for (int cycle = 0; cycle < 12; ++cycle) {
    jog_publisher->publish(jog);
    spin_delivery(executor);
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((820 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
  }
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);
  for (int cycle = 0; cycle < 20; ++cycle) {
    jog_publisher->publish(jog);
    spin_delivery(executor);
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((940 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
  }
  ASSERT_GT(std::abs(command_velocity), 1.0e-6);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  ASSERT_EQ(
    controller->update(rclcpp::Time(1'050'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  EXPECT_GT(std::abs(command_velocity), 0.0);
  EXPECT_LE(std::abs(command_acceleration), 0.033333333 + 1.0e-9);
  double previous_acceleration = command_acceleration;
  for (int cycle = 0; cycle < 500; ++cycle) {
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((1060 + cycle * 10) * 1'000'000LL, RCL_ROS_TIME), period),
      controller_interface::return_type::OK);
    EXPECT_LE(std::abs(command_velocity), 0.020 + 1.0e-9);
    EXPECT_LE(std::abs(command_acceleration), 0.033333333 + 1.0e-9);
    EXPECT_LE(std::abs(command_acceleration - previous_acceleration), 0.004 + 1.0e-6);
    previous_acceleration = command_acceleration;
  }
  EXPECT_NEAR(command_velocity, 0.0, 1.0e-8);
  EXPECT_NEAR(command_acceleration, 0.0, 1.0e-8);

  EXPECT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  executor.remove_node(io_node);
  executor.remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  rclcpp::shutdown();
}

TEST(LiftController, HeavyLeaseArbitratesLegacyWriterAndKeepsSafetyStopAvailable)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<joint_hardware::LiftController>();
  ASSERT_EQ(
    controller->init("lift_heavy_arbiter_test", "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  double command_position = -0.2;
  double command_velocity = 0.0;
  double command_acceleration = 0.0;
  double command_power_enable = 0.0;
  double state_position = -0.2;
  double state_velocity = 0.0;
  double state_power_enable = 0.0;
  hardware_interface::CommandInterface position_command(
    "joint_motor", "position", &command_position);
  hardware_interface::CommandInterface velocity_command(
    "joint_motor", "velocity", &command_velocity);
  hardware_interface::CommandInterface acceleration_command(
    "joint_motor", "acceleration", &command_acceleration);
  hardware_interface::CommandInterface power_enable_command(
    "joint_motor", "power_enable", &command_power_enable);
  hardware_interface::StateInterface position_state(
    "joint_motor", "position", &state_position);
  hardware_interface::StateInterface velocity_state(
    "joint_motor", "velocity", &state_velocity);
  hardware_interface::StateInterface power_enable_state(
    "joint_motor", "power_enable", &state_power_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(position_command);
  commands.emplace_back(velocity_command);
  commands.emplace_back(acceleration_command);
  commands.emplace_back(power_enable_command);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(power_enable_state);
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  auto io_node = std::make_shared<rclcpp::Node>("lift_heavy_arbiter_test_io");
  rclcpp::QoS gate_qos(rclcpp::KeepLast(1));
  gate_qos.reliable().transient_local();
  auto gate_publisher = io_node->create_publisher<std_msgs::msg::Bool>(
    "/ubuntu_lower_gateway/internal/heavy/v1/lease_active", gate_qos);
  auto heavy_publisher = io_node->create_publisher<
    robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1>(
    "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command",
    rclcpp::QoS(1).reliable());
  auto status_publisher = io_node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  std::atomic<uint64_t> acknowledged_sequence{0};
  auto ack_subscription = io_node->create_subscription<std_msgs::msg::UInt64>(
    "/ubuntu_lower_gateway/internal/heavy/v1/lift_applied_sequence",
    rclcpp::QoS(10).reliable(),
    [&acknowledged_sequence](const std_msgs::msg::UInt64::SharedPtr message) {
      if (message) {
        acknowledged_sequence.store(message->data);
      }
    });
  auto legacy_client = io_node->create_client<robot_control_msg::srv::SelectedJointControl>(
    "/joint/lift/command");
  auto stop_client = io_node->create_client<std_srvs::srv::Trigger>("/joint/lift/stop");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  executor.add_node(io_node);

  std_msgs::msg::Bool gate;
  gate.data = true;
  gate_publisher->publish(gate);
  spin_delivery(executor);

  auto legacy_request = std::make_shared<robot_control_msg::srv::SelectedJointControl::Request>();
  legacy_request->joint_names = {"joint_motor"};
  legacy_request->values = {-0.1};
  legacy_request->vel = 0.01;
  legacy_request->acc = 0.01;
  auto blocked = legacy_client->async_send_request(legacy_request);
  ASSERT_EQ(
    executor.spin_until_future_complete(blocked, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto blocked_response = blocked.get();
  EXPECT_FALSE(blocked_response->accepted);
  EXPECT_NE(blocked_response->message.find("Heavy V1"), std::string::npos);

  robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1 heavy;
  heavy.sequence = 42;
  heavy.mode = heavy.MODE_HOLD;
  heavy.field_mask = heavy.FIELD_POSITION;
  heavy.position.fill(0.0);
  heavy.position[0] = state_position;
  heavy_publisher->publish(heavy);
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_nanoseconds(10'000'000)),
    controller_interface::return_type::OK);
  spin_delivery(executor);
  EXPECT_EQ(acknowledged_sequence.load(), 42U);
  EXPECT_DOUBLE_EQ(command_position, state_position);

  auto stop = stop_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int cycle = 0; cycle < 20 &&
    stop.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++cycle)
  {
    ASSERT_EQ(
      controller->update(
        rclcpp::Time((10 + cycle) * 10'000'000LL, RCL_ROS_TIME),
        rclcpp::Duration::from_nanoseconds(10'000'000)),
      controller_interface::return_type::OK);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(stop.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(stop.get()->success);

  // A Heavy FOLLOW sample is already time-parameterized by the upper
  // controller. Once the initial brake gate has completed, it must reach the
  // command interfaces unchanged rather than becoming a Ruckig move with a
  // zero terminal velocity.
  state_power_enable = 1.0;
  // Brake release is owned by the drive after Operation Enabled. The
  // controller must not wait for its inferred brake_unlocked status before
  // admitting the first Heavy FOLLOW into streaming.
  status_publisher->publish(driver_status(false, false));
  spin_delivery(executor);
  heavy.sequence = 43;
  heavy.mode = heavy.MODE_FOLLOW_POSITION;
  heavy.field_mask = heavy.FIELD_POSITION | heavy.FIELD_VELOCITY | heavy.FIELD_ACCELERATION;
  heavy.position[0] = -0.19;
  heavy.velocity[0] = 0.02;
  heavy.acceleration[0] = 0.01;
  heavy_publisher->publish(heavy);
  spin_delivery(executor);
  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(
    controller->update(rclcpp::Time(250'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  spin_delivery(executor);
  // Reading a FOLLOW sample is not sufficient: before the CiA402/brake gate
  // completes, the controller is still outputting HOLD and must not claim it
  // has applied the sample.
  EXPECT_EQ(acknowledged_sequence.load(), 42U);
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  status_publisher->publish(driver_status(false, false));
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(rclcpp::Time(260'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_position, -0.19);
  EXPECT_DOUBLE_EQ(command_velocity, 0.02);
  EXPECT_DOUBLE_EQ(command_acceleration, 0.01);
  spin_delivery(executor);
  EXPECT_EQ(acknowledged_sequence.load(), 43U);

  // Once streaming is active, every subsequent Heavy sample must replace the
  // previous command and survive the mode-update phase in the same cycle.
  heavy.sequence = 44;
  heavy.position[0] = -0.18;
  heavy.velocity[0] = 0.015;
  heavy.acceleration[0] = 0.02;
  heavy_publisher->publish(heavy);
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(rclcpp::Time(270'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_position, -0.18);
  EXPECT_DOUBLE_EQ(command_velocity, 0.015);
  EXPECT_DOUBLE_EQ(command_acceleration, 0.02);
  spin_delivery(executor);
  EXPECT_EQ(acknowledged_sequence.load(), 44U);

  gate.data = false;
  gate_publisher->publish(gate);
  spin_delivery(executor);

  // A restarted gateway begins its private controller generation at 1. The
  // retained false lease gate must clear the prior generation watermark so
  // this new-session HOLD is admitted instead of being discarded as old.
  gate.data = true;
  gate_publisher->publish(gate);
  heavy.sequence = 1;
  heavy.mode = heavy.MODE_HOLD;
  heavy.field_mask = heavy.FIELD_POSITION;
  heavy.position[0] = state_position;
  heavy.velocity.fill(0.0);
  heavy.acceleration.fill(0.0);
  heavy_publisher->publish(heavy);
  spin_delivery(executor);
  ASSERT_EQ(
    controller->update(rclcpp::Time(270'000'000LL, RCL_ROS_TIME), period),
    controller_interface::return_type::OK);
  spin_delivery(executor);
  EXPECT_EQ(acknowledged_sequence.load(), 1U);

  gate.data = false;
  gate_publisher->publish(gate);
  spin_delivery(executor);
  auto restored = legacy_client->async_send_request(legacy_request);
  ASSERT_EQ(
    executor.spin_until_future_complete(restored, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(restored.get()->accepted);

  EXPECT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  executor.remove_node(io_node);
  executor.remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  rclcpp::shutdown();
}

TEST(LiftController, HoldIsConfirmedAndKeepsServoEnabledUntilExplicitStop)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<joint_hardware::LiftController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("brake_gate_stable_sec", 0.1),
      rclcpp::Parameter("driver_status_timeout_sec", 5.0),
      rclcpp::Parameter("target_stable_sec", 0.0),
      rclcpp::Parameter("goal_tolerance_m", 0.0001),
      rclcpp::Parameter("stationary_velocity_mps", 0.001),
    });
  ASSERT_EQ(
    controller->init("lift_hold_confirmation_test", "", options),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  double command_position = -0.10;
  double command_velocity = 0.0;
  double command_acceleration = 0.0;
  double command_power_enable = 0.0;
  double state_position = -0.10;
  double state_velocity = 0.0;
  double state_power_enable = 1.0;
  hardware_interface::CommandInterface position_command(
    "joint_motor", "position", &command_position);
  hardware_interface::CommandInterface velocity_command(
    "joint_motor", "velocity", &command_velocity);
  hardware_interface::CommandInterface acceleration_command(
    "joint_motor", "acceleration", &command_acceleration);
  hardware_interface::CommandInterface power_enable_command(
    "joint_motor", "power_enable", &command_power_enable);
  hardware_interface::StateInterface position_state(
    "joint_motor", "position", &state_position);
  hardware_interface::StateInterface velocity_state(
    "joint_motor", "velocity", &state_velocity);
  hardware_interface::StateInterface power_enable_state(
    "joint_motor", "power_enable", &state_power_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(position_command);
  commands.emplace_back(velocity_command);
  commands.emplace_back(acceleration_command);
  commands.emplace_back(power_enable_command);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(power_enable_state);
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  auto io_node = std::make_shared<rclcpp::Node>("lift_hold_confirmation_test_io");
  auto status_publisher = io_node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  auto command_client = io_node->create_client<robot_control_msg::srv::SelectedJointControl>(
    "/joint/lift/command");
  auto trajectory_publisher =
    io_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint/lift/trajectory", rclcpp::QoS(1).reliable());
  auto hold_client = io_node->create_client<std_srvs::srv::Trigger>("/joint/lift/hold");
  auto stop_client = io_node->create_client<std_srvs::srv::Trigger>("/joint/lift/stop");
  std::atomic<bool> mode_hold{false};
  std::atomic<bool> mode_fault{false};
  auto control_subscription = io_node->create_subscription<std_msgs::msg::String>(
    "/joint/lift/control_status", rclcpp::QoS(10).best_effort(),
    [&mode_hold, &mode_fault](const std_msgs::msg::String::SharedPtr message) {
      if (message) {
        mode_hold.store(message->data.find("\"mode\":\"hold\"") != std::string::npos);
        mode_fault.store(message->data.find("\"mode\":\"fault\"") != std::string::npos);
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  executor.add_node(io_node);
  ExecutorThreadGuard executor_thread(executor);
  ASSERT_TRUE(command_client->wait_for_service(std::chrono::seconds(1)));
  ASSERT_TRUE(hold_client->wait_for_service(std::chrono::seconds(1)));
  ASSERT_TRUE(stop_client->wait_for_service(std::chrono::seconds(1)));
  status_publisher->publish(driver_status(false));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  auto run_cycle = [&](int64_t cycle) {
      EXPECT_EQ(
        controller->update(rclcpp::Time(cycle * 10'000'000LL, RCL_ROS_TIME), period),
        controller_interface::return_type::OK);
      state_position = command_position;
      state_velocity = command_velocity;
      status_publisher->publish(driver_status(false));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

  // Establish an enabled HOLD through the normal brake gate.
  auto command_request = std::make_shared<robot_control_msg::srv::SelectedJointControl::Request>();
  command_request->joint_names = {"joint_motor"};
  command_request->values = {state_position};
  command_request->relative = false;
  command_request->vel = 0.01;
  command_request->acc = 0.02;
  auto initial_command_response = command_client->async_send_request(command_request);
  ASSERT_EQ(initial_command_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  ASSERT_TRUE(initial_command_response.get()->accepted);
  for (int cycle = 0; cycle < 30; ++cycle) {
    run_cycle(cycle);
  }
  ASSERT_DOUBLE_EQ(command_power_enable, 1.0);

  // HOLD while already stationary is idempotent and confirms the resulting mode.
  auto stationary_hold_response = hold_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int cycle = 30; cycle < 80 &&
    stationary_hold_response.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready; ++cycle)
  {
    run_cycle(cycle);
  }
  ASSERT_EQ(
    stationary_hold_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(stationary_hold_response.get()->success);
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);
  EXPECT_TRUE(mode_hold.load());
  EXPECT_FALSE(mode_fault.load());

  // HOLD during motion performs a bounded soft stop and captures measured position.
  command_request = std::make_shared<robot_control_msg::srv::SelectedJointControl::Request>();
  command_request->joint_names = {"joint_motor"};
  command_request->values = {-0.02};
  command_request->relative = true;
  command_request->vel = 0.03;
  command_request->acc = 0.05;
  auto moving_command_response = command_client->async_send_request(command_request);
  ASSERT_EQ(moving_command_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  ASSERT_TRUE(moving_command_response.get()->accepted);
  bool moving = false;
  for (int cycle = 80; cycle < 130; ++cycle) {
    run_cycle(cycle);
    moving = moving || std::abs(command_velocity) > 1.0e-5;
  }
  ASSERT_TRUE(moving);
  auto moving_hold_response = hold_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int cycle = 130; cycle < 400 &&
    moving_hold_response.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready; ++cycle)
  {
    run_cycle(cycle);
  }
  ASSERT_EQ(moving_hold_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(moving_hold_response.get()->success);
  EXPECT_NEAR(command_velocity, 0.0, 1.0e-8);
  EXPECT_DOUBLE_EQ(command_power_enable, 1.0);
  EXPECT_TRUE(mode_hold.load());
  EXPECT_FALSE(mode_fault.load());

  // Explicit STOP remains the safety path that requests controlled disable.
  auto stop_response = stop_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int cycle = 400; cycle < 500 &&
    stop_response.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready; ++cycle)
  {
    run_cycle(cycle);
  }
  ASSERT_EQ(stop_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(stop_response.get()->success);
  for (int cycle = 500; cycle < 650 && command_power_enable > 0.5; ++cycle) {
    run_cycle(cycle);
  }
  EXPECT_DOUBLE_EQ(command_power_enable, 0.0);

  // Repeating STOP after hardware disable/brake lock is idempotent. It must
  // not reinterpret the expected closed brake as a driver-gate motion fault.
  state_power_enable = 0.0;
  status_publisher->publish(driver_status(false, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  auto disabled_stop_response = stop_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    disabled_stop_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(disabled_stop_response.get()->success);
  run_cycle(650);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_TRUE(mode_hold.load());
  EXPECT_FALSE(mode_fault.load());

  // A second HOLD while already stationary is a synchronous idempotent
  // success and must capture the current measured position, not the default
  // numeric value of a freshly constructed command. Queue an old trajectory
  // immediately before HOLD to prove that the same cancellation boundary
  // removes pending trajectory work before the service reports success.
  const double idempotent_hold_position = state_position;
  trajectory_publisher->publish(trajectory_to(-0.40));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto second_hold = hold_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(second_hold.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  const auto second_hold_result = second_hold.get();
  EXPECT_TRUE(second_hold_result->success);
  EXPECT_NE(second_hold_result->message.find("stage=idempotent"), std::string::npos);
  for (int cycle = 651; cycle < 951; ++cycle) {
    run_cycle(cycle);
    EXPECT_NEAR(command_position, idempotent_hold_position, 1.0e-9);
    EXPECT_NEAR(command_velocity, 0.0, 1.0e-9);
    EXPECT_NEAR(command_acceleration, 0.0, 1.0e-9);
  }

  EXPECT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  auto inactive_hold = hold_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(inactive_hold.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  const auto inactive_hold_result = inactive_hold.get();
  EXPECT_TRUE(inactive_hold_result->success);
  EXPECT_NE(inactive_hold_result->message.find("stage=inactive"), std::string::npos);

  executor_thread.stop();
  executor.remove_node(io_node);
  executor.remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  rclcpp::shutdown();
}

TEST(LiftController, HoldReportsDriverGateFailureInsteadOfFalseSuccess)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<joint_hardware::LiftController>();
  ASSERT_EQ(
    controller->init("lift_hold_failure_test", "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  double command_position = -0.1;
  double command_velocity = 0.0;
  double command_acceleration = 0.0;
  double command_power_enable = 0.0;
  double state_position = -0.1;
  double state_velocity = 0.01;
  double state_power_enable = 1.0;
  hardware_interface::CommandInterface position_command(
    "joint_motor", "position", &command_position);
  hardware_interface::CommandInterface velocity_command(
    "joint_motor", "velocity", &command_velocity);
  hardware_interface::CommandInterface acceleration_command(
    "joint_motor", "acceleration", &command_acceleration);
  hardware_interface::CommandInterface power_enable_command(
    "joint_motor", "power_enable", &command_power_enable);
  hardware_interface::StateInterface position_state(
    "joint_motor", "position", &state_position);
  hardware_interface::StateInterface velocity_state(
    "joint_motor", "velocity", &state_velocity);
  hardware_interface::StateInterface power_enable_state(
    "joint_motor", "power_enable", &state_power_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(position_command);
  commands.emplace_back(velocity_command);
  commands.emplace_back(acceleration_command);
  commands.emplace_back(power_enable_command);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(power_enable_state);
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  auto io_node = std::make_shared<rclcpp::Node>("lift_hold_failure_test_io");
  auto status_publisher = io_node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10));
  auto hold_client = io_node->create_client<std_srvs::srv::Trigger>("/joint/lift/hold");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  executor.add_node(io_node);
  ExecutorThreadGuard executor_thread(executor);
  ASSERT_TRUE(hold_client->wait_for_service(std::chrono::seconds(1)));
  status_publisher->publish(driver_status(false, false, 7, true, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto hold_response = hold_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int cycle = 0; cycle < 20 &&
    hold_response.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++cycle)
  {
    EXPECT_EQ(
      controller->update(
        rclcpp::Time(cycle * 10'000'000LL, RCL_ROS_TIME),
        rclcpp::Duration::from_nanoseconds(10'000'000)),
      controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(hold_response.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  const auto hold_result = hold_response.get();
  EXPECT_FALSE(hold_result->success);
  EXPECT_NE(hold_result->message.find("stage=driver_gate"), std::string::npos);
  EXPECT_DOUBLE_EQ(command_power_enable, 0.0);

  EXPECT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  executor_thread.stop();
  executor.remove_node(io_node);
  executor.remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  rclcpp::shutdown();
}

}  // namespace

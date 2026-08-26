#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "joint_hardware/lift_hardware.hpp"
#include "joint_hardware/lift/ethercat_backend.hpp"
#include "joint_hardware/lift/zero_offset_store.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

hardware_interface::HardwareInfo make_lift_hardware_info(
  const std::vector<std::pair<std::string, std::string>> & overrides = {})
{
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters = {
    {"ethercat_backend", "mock"},
    {"ethercat_cycle_ms", "10"},
    {"expected_working_counter", "1"},
    {"slave_alias", "0"},
    {"slave_position", "0"},
    {"motor_id", "LVM08008H3G3-M17"},
    {"lead_mm_per_rev", "3.333333333"},
    {"lift_sign", "-1.0"},
    {"command_units_per_rev", "10000"},
    {"brake_control_enabled", "true"},
    {"brake_release_wait_ms", "0"},
    {"zero_offset_file", "/tmp/joint_hardware_mock_safety_zero.cfg"},
  };
  for (const auto & [key, value] : overrides) {
    info.hardware_parameters[key] = value;
  }

  hardware_interface::ComponentInfo joint;
  joint.name = "joint_motor";
  for (const char * name :
    {"position", "velocity", "status", "error_code", "mode", "brake_unlocked",
      "digital_inputs", "power_enable"})
  {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.state_interfaces.push_back(interface);
  }
  for (const char * name : {"position", "velocity", "acceleration", "power_enable"}) {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.command_interfaces.push_back(interface);
  }
  info.joints.push_back(joint);
  return info;
}

TEST(MockLiftEthercat, PdoCycleAndSafetyInputs)
{
  joint_hardware::lift::MockLiftEthercatBackend backend;
  joint_hardware::lift::EthercatMasterConfig master;
  master.cycle_period = std::chrono::milliseconds(10);
  ASSERT_TRUE(backend.initialize(master));
  ASSERT_TRUE(backend.configure_slave({0, 0, 0, 0}));
  ASSERT_TRUE(backend.start());

  std::vector<uint8_t> mode;
  ASSERT_TRUE(backend.read_sdo(0x6061, 0, mode));
  ASSERT_EQ(mode.size(), 1U);
  EXPECT_EQ(mode[0], 9);

  joint_hardware::lift::LiftRxPdo output;
  joint_hardware::lift::LiftTxPdo input;
  output.control_word = 0x0006;
  ASSERT_TRUE(backend.exchange_pdo(output, input));
  EXPECT_EQ(input.status_word, 0x0021);
  output.control_word = 0x0007;
  ASSERT_TRUE(backend.exchange_pdo(output, input));
  EXPECT_EQ(input.status_word, 0x0023);
  output.control_word = 0x000f;
  output.target_velocity_units_per_s = 1000;
  for (int cycle = 0; cycle < 10; ++cycle) {
    ASSERT_TRUE(backend.exchange_pdo(output, input));
  }
  EXPECT_EQ(input.status_word, 0x0027);
  EXPECT_EQ(input.actual_position_units, 100);
  EXPECT_EQ(backend.last_output().target_velocity_units_per_s, 1000);

  output.control_word = 0x000b;
  output.target_velocity_units_per_s = 0;
  ASSERT_TRUE(backend.exchange_pdo(output, input));
  EXPECT_EQ(input.status_word, 0x0007);
  EXPECT_EQ(input.actual_velocity_units_per_s, 0);

  backend.set_pdo_fresh(false);
  EXPECT_TRUE(backend.exchange_pdo(output, input));
  EXPECT_FALSE(backend.pdo_fresh());
  backend.set_online(false);
  EXPECT_FALSE(backend.exchange_pdo(output, input));
  EXPECT_EQ(backend.link_state(), joint_hardware::lift::EthercatLinkState::offline);
}

TEST(MockLiftEthercat, PdoWireRoundTripIsLittleEndianAndSigned)
{
  joint_hardware::lift::LiftRxPdo output;
  output.control_word = 0x0080;
  output.target_velocity_units_per_s = -123456;
  output.torque_feedforward = -321;
  joint_hardware::lift::LiftRxPdoWire output_wire{};
  ASSERT_TRUE(joint_hardware::lift::encode_lift_rx_pdo_le(output, output_wire));
  EXPECT_EQ(output_wire[0], 0x80);
  EXPECT_EQ(output_wire[1], 0x00);
  EXPECT_EQ(output_wire[2], 0xc0);
  EXPECT_EQ(output_wire[3], 0x1d);
  EXPECT_EQ(output_wire[4], 0xfe);
  EXPECT_EQ(output_wire[5], 0xff);
  joint_hardware::lift::LiftRxPdo decoded_output;
  ASSERT_TRUE(joint_hardware::lift::decode_lift_rx_pdo_le(output_wire, decoded_output));
  EXPECT_EQ(decoded_output.control_word, output.control_word);
  EXPECT_EQ(decoded_output.target_velocity_units_per_s, output.target_velocity_units_per_s);
  EXPECT_EQ(decoded_output.torque_feedforward, output.torque_feedforward);

  joint_hardware::lift::LiftTxPdo input;
  input.status_word = 0x0027;
  input.error_code = 0x2310;
  input.mode_display = 9;
  input.actual_position_units = -100000;
  input.actual_velocity_units_per_s = -2000;
  input.digital_inputs = 0x12345678;
  joint_hardware::lift::LiftTxPdoWire input_wire{};
  ASSERT_TRUE(joint_hardware::lift::encode_lift_tx_pdo_le(input, input_wire));
  EXPECT_EQ(input_wire[0], 0x27);
  EXPECT_EQ(input_wire[1], 0x00);
  EXPECT_EQ(input_wire[13], 0x78);
  EXPECT_EQ(input_wire[14], 0x56);
  EXPECT_EQ(input_wire[15], 0x34);
  EXPECT_EQ(input_wire[16], 0x12);
  joint_hardware::lift::LiftTxPdo decoded_input;
  ASSERT_TRUE(joint_hardware::lift::decode_lift_tx_pdo_le(input_wire, decoded_input));
  EXPECT_EQ(decoded_input.status_word, input.status_word);
  EXPECT_EQ(decoded_input.error_code, input.error_code);
  EXPECT_EQ(decoded_input.mode_display, input.mode_display);
  EXPECT_EQ(decoded_input.actual_position_units, input.actual_position_units);
  EXPECT_EQ(decoded_input.actual_velocity_units_per_s, input.actual_velocity_units_per_s);
  EXPECT_EQ(decoded_input.digital_inputs, input.digital_inputs);
}

TEST(MockLiftEthercat, ImplementsLd3mHomingAndDriveZeroObjects)
{
  joint_hardware::lift::MockLiftEthercatBackend backend;
  joint_hardware::lift::EthercatMasterConfig master;
  ASSERT_TRUE(backend.initialize(master));
  ASSERT_TRUE(backend.configure_slave({0, 0, 0, 0}));
  ASSERT_TRUE(backend.write_sdo(0x6060, 0, {6}));
  ASSERT_TRUE(backend.write_sdo(0x6098, 0, {19}));
  ASSERT_TRUE(backend.write_sdo(0x6099, 1, {0x10, 0x27, 0, 0}));
  ASSERT_TRUE(backend.write_sdo(0x6099, 2, {0x88, 0x13, 0, 0}));
  ASSERT_TRUE(backend.write_sdo(0x609a, 0, {0x20, 0xa1, 0x07, 0}));
  ASSERT_TRUE(backend.write_sdo(0x607c, 0, {0xe8, 0x03, 0, 0}));
  ASSERT_TRUE(backend.start());

  joint_hardware::lift::LiftRxPdo output;
  joint_hardware::lift::LiftTxPdo input;
  output.control_word = 0x001f;
  ASSERT_TRUE(backend.exchange_pdo(output, input));
  EXPECT_EQ(input.mode_display, 6);
  EXPECT_EQ(input.status_word & 0x1400, 0x1400);
  EXPECT_EQ(input.actual_position_units, 1000);

  output.control_word = 0x0006;
  ASSERT_TRUE(backend.exchange_pdo(output, input));
  ASSERT_TRUE(backend.write_sdo(0x6060, 0, {9}));
  ASSERT_TRUE(backend.write_sdo(0x2015, 0, {9, 0})) << backend.error_message();
  std::vector<uint8_t> encoder_setting;
  ASSERT_TRUE(backend.read_sdo(0x2015, 0, encoder_setting));
  ASSERT_EQ(encoder_setting, (std::vector<uint8_t>{9, 0}));
}

TEST(EthercatSdkAdapter, RefusesUntilImplemented)
{
  auto backend = joint_hardware::lift::make_lift_ethercat_backend("real_sdk");
  joint_hardware::lift::EthercatMasterConfig master;
  EXPECT_FALSE(backend->initialize(master));
  EXPECT_EQ(backend->link_state(), joint_hardware::lift::EthercatLinkState::offline);
  EXPECT_NE(backend->error_message().find("unavailable"), std::string::npos);
}

TEST(EtherLabBackend, FactoryStartsOfflineUntilExplicitlyInitialized)
{
  auto backend = joint_hardware::lift::make_lift_ethercat_backend("etherlab");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(
    backend->link_state(), joint_hardware::lift::EthercatLinkState::offline);
  EXPECT_FALSE(backend->pdo_fresh());
  EXPECT_EQ(backend->working_counter(), 0U);
}

TEST(MockLiftHardware, IgnoresPersistentZeroOffsetAndStartsAtSimulationOrigin)
{
  const std::string offset_path = "/tmp/joint_hardware_mock_persisted_real_zero.cfg";
  std::filesystem::remove(offset_path);
  joint_hardware::lift::ZeroOffsetRecord record;
  record.motor_id = "LVM08008H3G3-M17";
  record.zero_offset_units = -618455;
  std::string error;
  ASSERT_TRUE(joint_hardware::lift::ZeroOffsetStore::save_atomic(offset_path, record, error));

  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", offset_path},
      {"use_persistent_zero_offset", "false"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);

  bool position_seen = false;
  for (const auto & state : hardware.export_state_interfaces()) {
    if (state.get_interface_name() == "position") {
      position_seen = true;
      EXPECT_DOUBLE_EQ(state.get_value(), 0.0);
    }
  }
  EXPECT_TRUE(position_seen);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
  std::filesystem::remove(offset_path);
}

TEST(MockLiftHardware, HomeServiceRunsHmAndRestoresCsv)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"use_persistent_zero_offset", "false"},
      {"homing_timeout_ms", "2000"},
      {"drive_zero_timeout_ms", "4000"},
      {"lift_startup_motion_guard_enabled", "false"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::thread control_thread([&]() {
      const rclcpp::Time time(0, 0, RCL_ROS_TIME);
      const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
      while (running.load(std::memory_order_acquire)) {
        (void)hardware.read(time, period);
        (void)hardware.write(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_home_test");
  auto client = client_node->create_client<std_srvs::srv::Trigger>("/lift_home");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  const bool service_available = client->wait_for_service(std::chrono::seconds(2));
  EXPECT_TRUE(service_available);
  if (service_available) {
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto result = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      const auto response = future.get();
      ASSERT_NE(response, nullptr);
      EXPECT_TRUE(response->success) << response->message;
      EXPECT_EQ(backend_view->current_input().mode_display, 9);
      EXPECT_FALSE(backend_view->current_input().actual_velocity_units_per_s);
    }
  }

  running.store(false, std::memory_order_release);
  control_thread.join();
  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, DriveZeroServiceStagesAbsoluteEncoderMaintenance)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  backend_view->set_actual_position_units(50'000);
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"use_persistent_zero_offset", "false"},
      {"drive_zero_timeout_ms", "4000"},
      {"lift_startup_motion_guard_enabled", "false"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::thread control_thread([&]() {
      const rclcpp::Time time(0, 0, RCL_ROS_TIME);
      const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
      while (running.load(std::memory_order_acquire)) {
        (void)hardware.read(time, period);
        (void)hardware.write(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_drive_zero_test");
  auto client = client_node->create_client<std_srvs::srv::Trigger>("/lift_set_drive_zero");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_NE(response->message.find("power-cycle/restart"), std::string::npos);
  EXPECT_EQ(backend_view->current_input().actual_position_units, 50'000);

  running.store(false, std::memory_order_release);
  control_thread.join();
  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, BootstrapRejectsAStaleZeroSampleBeforeEstablishingPositionBaseline)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  backend->set_startup_position_sequence(
    {0, -618455, -618455, -618455, -618455, -618455, -618455});
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"max_feedback_jump_m", "0.2"},
      {"zero_offset_file", "/tmp/joint_hardware_mock_bootstrap_zero.cfg"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  EXPECT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);

  auto node = std::make_shared<rclcpp::Node>("lift_bootstrap_stability_test");
  auto client = node->create_client<std_srvs::srv::SetBool>("/lift_brake_command");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = client->async_send_request(request);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);

  hardware.on_deactivate(rclcpp_lifecycle::State{});
  rclcpp::shutdown();
}

TEST(MockLiftHardware, CoordinateJumpStaysLatchedUntilExplicitSafeReset)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"max_feedback_jump_m", "0.05"},
      {"lift_startup_motion_guard_enabled", "false"},
      {"zero_offset_file", "/tmp/joint_hardware_mock_jump_latch_zero.cfg"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_jump_latch_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  const auto call_bool = [&](bool value) {
      auto client = client_node->create_client<std_srvs::srv::SetBool>("/lift_brake_command");
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = value;
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };
  const auto call_reset = [&]() {
      auto client = client_node->create_client<std_srvs::srv::Trigger>("/joint/safety/reset");
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      EXPECT_EQ(
        executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);

  // 200000 units is -0.06666666666 m with the geared conversion, exceeding the
  // 0.05 m threshold in one fresh sample of the same epoch.
  backend_view->set_actual_position_units(200'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  EXPECT_FALSE(call_bool(true)->success);

  // Stable feedback alone does not silently clear a real jump latch.
  for (int cycle = 0; cycle < 10; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_FALSE(call_bool(true)->success);

  // Clear the upper command to the current measured position. The explicit
  // safety reset is then allowed to establish a new baseline without resuming
  // the old motion command.
  for (auto & command : hardware.export_command_interfaces()) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.06666666666);
    } else if (command.get_interface_name() == "velocity" ||
      command.get_interface_name() == "acceleration")
    {
      command.set_value(0.0);
    }
  }
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  const auto reset = call_reset();
  ASSERT_TRUE(reset->success) << reset->message;
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(call_bool(true)->success);

  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_jump_latch_zero.cfg");
  executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, RunsRos2ControlReadWriteAt100Hz)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters = {
    {"ethercat_backend", "mock"},
    {"ethercat_cycle_ms", "10"},
    {"expected_working_counter", "1"},
    {"slave_alias", "0"},
    {"slave_position", "0"},
    {"motor_id", "LVM08008H3G3-M17"},
    {"lead_mm_per_rev", "3.333333333"},
    {"lift_sign", "-1.0"},
    {"command_units_per_rev", "10000"},
    {"brake_control_enabled", "true"},
    {"brake_release_wait_ms", "0"},
    {"lift_startup_motion_guard_enabled", "false"},
    {"reset_velocity_debug_enabled", "true"},
    {"zero_offset_file", "/tmp/joint_hardware_mock_integration_zero.cfg"},
  };
  hardware_interface::ComponentInfo joint;
  joint.name = "joint_motor";
  for (const char * name :
    {"position", "velocity", "status", "error_code", "mode", "brake_unlocked",
      "digital_inputs", "power_enable"})
  {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.state_interfaces.push_back(interface);
  }
  for (const char * name : {"position", "velocity", "acceleration", "power_enable"}) {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.command_interfaces.push_back(interface);
  }
  info.joints.push_back(joint);

  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_service_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  const auto call_bool_service = [&](const std::string & name, bool value) {
      auto client = client_node->create_client<std_srvs::srv::SetBool>(name);
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = value;
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return std::shared_ptr<std_srvs::srv::SetBool::Response>{};
      }
      return future.get();
    };

  const auto disable_response = call_bool_service("/lift_brake_command", false);
  ASSERT_NE(disable_response, nullptr);
  EXPECT_TRUE(disable_response->success);
  EXPECT_NE(disable_response->message.find("controlled stop"), std::string::npos);
  const auto enable_response = call_bool_service("/lift_brake_command", true);
  ASSERT_NE(enable_response, nullptr);
  EXPECT_TRUE(enable_response->success);
  EXPECT_NE(enable_response->message.find("Operation enabled"), std::string::npos);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  // The first write is intentionally command-free and aligns all command
  // interfaces with feedback. Set the motion target only after that barrier.
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);

  auto commands = hardware.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.10);
    }
  }

  bool command_sent_in_same_cycle = false;
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    command_sent_in_same_cycle = command_sent_in_same_cycle ||
      backend_view->last_output().target_velocity_units_per_s != 0;
  }
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    command_sent_in_same_cycle = command_sent_in_same_cycle ||
      backend_view->last_output().target_velocity_units_per_s != 0;
  }
  EXPECT_TRUE(command_sent_in_same_cycle);
  auto states = hardware.export_state_interfaces();
  bool position_seen = false;
  for (const auto & state : states) {
    if (state.get_interface_name() == "position") {
      position_seen = true;
      EXPECT_LT(state.get_value(), 0.0);
    }
  }
  EXPECT_TRUE(position_seen);

  backend_view->set_working_counter(0);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  backend_view->set_working_counter(1);
  backend_view->set_pdo_fresh(false);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  backend_view->set_pdo_fresh(true);
  bool recovered_motion_seen = false;
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    if (cycle < 4) {
      EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
    }
    recovered_motion_seen = recovered_motion_seen ||
      backend_view->last_output().target_velocity_units_per_s != 0;
  }
  EXPECT_TRUE(recovered_motion_seen);

  double current_position = 0.0;
  for (const auto & state : states) {
    if (state.get_interface_name() == "position") {
      current_position = state.get_value();
    }
  }
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(current_position);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(-0.02);
    }
  }
  bool feedforward_seen_at_zero_position_error = false;
  for (int cycle = 0; cycle < 6; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    feedforward_seen_at_zero_position_error = feedforward_seen_at_zero_position_error ||
      backend_view->last_output().target_velocity_units_per_s != 0;
  }
  EXPECT_TRUE(feedforward_seen_at_zero_position_error);
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.10);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(0.0);
    }
  }

  // This is the hardware half of Heavy direct streaming: controller tests
  // verify that each accepted Heavy FOLLOW reaches these p/v interfaces;
  // here a 100 Hz monotonic sample sequence must continue through the CSV
  // conversion into 60FFh without an opposite-sign PDO target.
  double stream_origin = current_position;
  bool streamed_pdo_seen = false;
  int32_t streamed_pdo_sign = 0;
  for (int sample = 1; sample <= 100; ++sample) {
    const double streamed_position = stream_origin - 0.0001 * static_cast<double>(sample);
    for (auto & command : commands) {
      if (command.get_interface_name() == "position") {
        command.set_value(streamed_position);
      } else if (command.get_interface_name() == "velocity") {
        command.set_value(-0.01);
      } else if (command.get_interface_name() == "acceleration") {
        command.set_value(0.0);
      }
    }
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    const auto pdo_velocity = backend_view->last_output().target_velocity_units_per_s;
    if (pdo_velocity != 0) {
      if (!streamed_pdo_seen) {
        streamed_pdo_seen = true;
        streamed_pdo_sign = pdo_velocity > 0 ? 1 : -1;
      } else {
        EXPECT_EQ(pdo_velocity > 0 ? 1 : -1, streamed_pdo_sign);
      }
    }
  }
  EXPECT_TRUE(streamed_pdo_seen);

  const auto reset_velocity_response = call_bool_service("/lift_reset_velocity", true);
  ASSERT_NE(reset_velocity_response, nullptr);
  EXPECT_TRUE(reset_velocity_response->success);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  const auto reset_velocity_stop_response = call_bool_service("/lift_reset_velocity", false);
  ASSERT_NE(reset_velocity_stop_response, nullptr);
  EXPECT_TRUE(reset_velocity_stop_response->success);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);

  // A lock request must zero 60FFh in this cycle. If the sampled speed is
  // already below P04.39, disabling immediately is safe; otherwise the first
  // frame is the bounded Quick Stop before Operation-disabled. Both paths
  // mirror the drive's automatic BR+/BR- brake sequence without a text command.
  const auto lock_response = call_bool_service("/lift_brake_command", false);
  ASSERT_NE(lock_response, nullptr);
  EXPECT_TRUE(lock_response->success);
  EXPECT_NE(lock_response->message.find("Quick Stop"), std::string::npos);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  EXPECT_TRUE(
    backend_view->last_output().control_word == 0x000b ||
    backend_view->last_output().control_word == 0x0006);
  if (backend_view->last_output().control_word == 0x000b) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);

  const auto hold_response = call_bool_service("/lift_reset_hold", true);
  ASSERT_NE(hold_response, nullptr);
  EXPECT_TRUE(hold_response->success);
  EXPECT_NE(hold_response->message.find("captured"), std::string::npos);
  const auto release_hold_response = call_bool_service("/lift_reset_hold", false);
  ASSERT_NE(release_hold_response, nullptr);
  EXPECT_TRUE(release_hold_response->success);
  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_integration_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, PowerCommandIsDistinctFromConfirmedEnabledState)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"lift_startup_motion_guard_enabled", "false"},
      {"zero_offset_file", "/tmp/joint_hardware_mock_power_interface_zero.cfg"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = hardware.export_command_interfaces();
  auto set_power_command = [&](double value) {
      bool found = false;
      for (auto & command : commands) {
        if (command.get_interface_name() == "power_enable") {
          command.set_value(value);
          found = true;
        }
      }
      ASSERT_TRUE(found);
    };
  const auto power_state = [&]() {
      for (const auto & state : hardware.export_state_interfaces()) {
        if (state.get_interface_name() == "power_enable") {
          return state.get_value();
        }
      }
      return -1.0;
    };

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const auto period = rclcpp::Duration::from_nanoseconds(10'000'000);
  EXPECT_DOUBLE_EQ(power_state(), 0.0);
  set_power_command(1.0);
  // Writing the request cannot fabricate Operation Enabled feedback.
  EXPECT_DOUBLE_EQ(power_state(), 0.0);

  bool enabled_from_feedback = false;
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    enabled_from_feedback = enabled_from_feedback || power_state() >= 0.5;
  }
  EXPECT_TRUE(enabled_from_feedback);
  EXPECT_EQ(backend_view->last_output().control_word, 0x000f);

  backend_view->set_working_counter(0);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(power_state(), 0.0);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, StartupGuardDiscardsResidualCommand)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", "/tmp/joint_hardware_mock_startup_guard_zero.cfg"},
      {"lift_startup_motion_guard_ms", "3000"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = hardware.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.50);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(-0.04);
    } else if (command.get_interface_name() == "acceleration") {
      command.set_value(-0.10);
    }
  }

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_startup_guard_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  auto brake_client = client_node->create_client<std_srvs::srv::SetBool>("/lift_brake_command");
  ASSERT_TRUE(brake_client->wait_for_service(std::chrono::seconds(2)));
  auto unlock = std::make_shared<std_srvs::srv::SetBool::Request>();
  unlock->data = true;
  auto unlock_future = brake_client->async_send_request(unlock);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(unlock_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(unlock_future.get()->success);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  }
  for (const auto & command : commands) {
    if (command.get_interface_name() == "position") {
      EXPECT_NEAR(command.get_value(), 0.0, 1.0e-12);
    } else {
      EXPECT_NEAR(command.get_value(), 0.0, 1.0e-12);
    }
  }

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_startup_guard_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, EmergencyStopIsLatchedAndResetDoesNotResumeMotion)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", "/tmp/joint_hardware_mock_estop_zero.cfg"},
      {"lift_startup_motion_guard_enabled", "false"},
      {"reset_velocity_debug_enabled", "true"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_estop_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  const auto call_bool = [&](const std::string & name, bool value) {
      auto client = client_node->create_client<std_srvs::srv::SetBool>(name);
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = value;
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };
  const auto call_trigger = [&](const std::string & name) {
      auto client = client_node->create_client<std_srvs::srv::Trigger>(name);
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };

  ASSERT_TRUE(call_bool("/lift_brake_command", true)->success);
  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  auto commands = hardware.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.20);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(-0.02);
    }
  }
  bool moving_command_seen = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    moving_command_seen = moving_command_seen ||
      backend_view->last_output().target_velocity_units_per_s != 0;
  }
  ASSERT_TRUE(moving_command_seen);

  const auto estop = call_trigger("/joint/safety/estop");
  ASSERT_TRUE(estop->success);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  EXPECT_TRUE(
    backend_view->last_output().control_word == 0x000b ||
    backend_view->last_output().control_word == 0x0006);

  EXPECT_FALSE(call_bool("/lift_brake_command", true)->success);
  EXPECT_FALSE(call_bool("/lift_reset_velocity", true)->success);
  EXPECT_FALSE(call_trigger("/lift_reset_zero")->success);

  // Let the filtered velocity converge below the stationary threshold. Safety
  // reset intentionally uses the filtered feedback and must reject earlier.
  for (int cycle = 0; cycle < 50; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  }
  const auto reset = call_trigger("/joint/safety/reset");
  ASSERT_TRUE(reset->success) << reset->message;
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  }
  EXPECT_NE(backend_view->last_output().control_word, 0x000f);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_estop_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, QuickStopDeadlineForcesDisable)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  backend_view->set_quick_stop_velocity_units(20000);
  backend_view->set_brake_p06_14_ms(1);
  joint_hardware::LiftHardware hardware(std::move(backend));
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters = {
    {"ethercat_backend", "mock"},
    {"ethercat_cycle_ms", "10"},
    {"expected_working_counter", "1"},
    {"slave_alias", "0"},
    {"slave_position", "0"},
    {"motor_id", "LVM08008H3G3-M17"},
    {"lead_mm_per_rev", "3.333333333"},
    {"lift_sign", "-1.0"},
    {"command_units_per_rev", "10000"},
    {"brake_control_enabled", "true"},
    {"brake_release_wait_ms", "0"},
    {"brake_p06_14_ms", "1"},
    {"zero_offset_file", "/tmp/joint_hardware_mock_quick_stop_zero.cfg"},
  };
  hardware_interface::ComponentInfo joint;
  joint.name = "joint_motor";
  for (const char * name : {"position", "velocity", "status", "error_code", "mode",
      "brake_unlocked", "digital_inputs"})
  {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.state_interfaces.push_back(interface);
  }
  hardware_interface::InterfaceInfo position_command;
  position_command.name = "position";
  joint.command_interfaces.push_back(position_command);
  info.joints.push_back(joint);

  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_quick_stop_deadline_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  auto client = client_node->create_client<std_srvs::srv::SetBool>("/lift_brake_command");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto enable_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  enable_request->data = true;
  auto enable_future = client->async_send_request(enable_request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(enable_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(enable_future.get()->success);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }

  auto lock_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  lock_request->data = false;
  auto lock_future = client->async_send_request(lock_request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(lock_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(lock_future.get()->success);

  bool disable_seen = false;
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    if (backend_view->last_output().control_word == 0x0006) {
      disable_seen = true;
      EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(disable_seen);
  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_quick_stop_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, BrakeControlDisabledNeverRequestsOperationEnabled)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters = {
    {"ethercat_backend", "mock"},
    {"ethercat_cycle_ms", "10"},
    {"expected_working_counter", "1"},
    {"slave_alias", "0"},
    {"slave_position", "0"},
    {"motor_id", "LVM08008H3G3-M17"},
    {"lead_mm_per_rev", "3.333333333"},
    {"lift_sign", "-1.0"},
    {"command_units_per_rev", "10000"},
    {"brake_control_enabled", "false"},
    {"brake_release_wait_ms", "0"},
    {"zero_offset_file", "/tmp/joint_hardware_mock_brake_disabled_zero.cfg"},
  };
  hardware_interface::ComponentInfo joint;
  joint.name = "joint_motor";
  for (const char * name : {"position", "velocity", "status", "error_code", "mode",
      "brake_unlocked", "digital_inputs"})
  {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.state_interfaces.push_back(interface);
  }
  hardware_interface::InterfaceInfo position_command;
  position_command.name = "position";
  joint.command_interfaces.push_back(position_command);
  info.joints.push_back(joint);

  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_brake_disabled_service_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  auto brake_client = client_node->create_client<std_srvs::srv::SetBool>(
    "/lift_brake_command");
  ASSERT_TRUE(brake_client->wait_for_service(std::chrono::seconds(2)));
  auto brake_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  brake_request->data = true;
  auto brake_future = brake_client->async_send_request(brake_request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(brake_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(brake_future.get()->success);

  auto lock_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  lock_request->data = false;
  auto lock_future = brake_client->async_send_request(lock_request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(lock_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(lock_future.get()->success);

  auto reset_client = client_node->create_client<std_srvs::srv::SetBool>(
    "/lift_reset_velocity");
  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(2)));
  auto reset_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  reset_request->data = true;
  auto reset_future = reset_client->async_send_request(reset_request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(reset_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(reset_future.get()->success);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_brake_disabled_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, ConfiguredNegativeLimitStopsOnlyNegativeMotion)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  backend_view->set_digital_inputs(1U << 0U);
  joint_hardware::LiftHardware hardware(std::move(backend));
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters = {
    {"ethercat_backend", "mock"},
    {"ethercat_cycle_ms", "10"},
    {"expected_working_counter", "1"},
    {"slave_alias", "0"},
    {"slave_position", "0"},
    {"motor_id", "LVM08008H3G3-M17"},
    {"lead_mm_per_rev", "3.333333333"},
    {"lift_sign", "-1.0"},
    {"command_units_per_rev", "10000"},
    {"brake_control_enabled", "true"},
    {"brake_release_wait_ms", "0"},
    {"limit_switch_enabled", "true"},
    {"limit_switch_negative_bit", "0"},
    {"zero_offset_file", "/tmp/joint_hardware_mock_limit_zero.cfg"},
  };
  hardware_interface::ComponentInfo joint;
  joint.name = "joint_motor";
  for (const char * name : {"position", "velocity", "status", "error_code", "mode",
      "brake_unlocked", "digital_inputs"})
  {
    hardware_interface::InterfaceInfo interface;
    interface.name = name;
    joint.state_interfaces.push_back(interface);
  }
  hardware_interface::InterfaceInfo position_command;
  position_command.name = "position";
  joint.command_interfaces.push_back(position_command);
  info.joints.push_back(joint);

  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_limit_service_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  auto client = client_node->create_client<std_srvs::srv::SetBool>("/lift_brake_command");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);

  for (auto & command : hardware.export_command_interfaces()) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.10);
    }
  }
  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(backend_view->last_output().control_word, 0x000f);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_limit_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, PositionLimitViolationAllowsOnlyBoundedInwardRecovery)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  // With lift_sign=-1, 10000 units/rev and 3.333333333 mm/rev,
  // 3,030,000 units is -1.01 m.
  backend_view->set_actual_position_units(3'030'000);
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", "/tmp/joint_hardware_mock_limit_recovery_zero.cfg"},
      {"lift_startup_motion_guard_enabled", "false"},
      {"reset_velocity_debug_enabled", "true"},
      {"position_limit_recovery_max_rpm", "10"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_limit_recovery_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  const auto call_bool = [&](const std::string & name, bool value) {
      auto client = client_node->create_client<std_srvs::srv::SetBool>(name);
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = value;
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  EXPECT_TRUE(
    backend_view->last_output().control_word == 0x000b ||
    backend_view->last_output().control_word == 0x0006);

  const auto reset_velocity = call_bool("/lift_reset_velocity", true);
  ASSERT_NE(reset_velocity, nullptr);
  EXPECT_FALSE(reset_velocity->success);
  EXPECT_NE(reset_velocity->message.find("outside software limits"), std::string::npos);

  auto commands = hardware.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-1.0);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(-0.01);
    } else if (command.get_interface_name() == "acceleration") {
      command.set_value(0.0);
    }
  }
  ASSERT_TRUE(call_bool("/lift_brake_command", true)->success);
  // A command farther below the lower limit remains blocked even after the
  // drive reaches Operation Enabled.
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  }

  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.99);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(0.01);
    }
  }
  bool inward_motion_seen = false;
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    const int32_t target_units = backend_view->last_output().target_velocity_units_per_s;
    inward_motion_seen = inward_motion_seen || target_units < 0;
    EXPECT_LE(std::abs(target_units), 1667);  // 10 rpm at 10000 units/rev.
  }
  EXPECT_TRUE(inward_motion_seen);

  // Simulate re-entry to the valid range. A debug takeover is no longer
  // rejected for the position violation, proving the recovery latch cleared.
  backend_view->set_actual_position_units(2'970'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  const auto reset_after_recovery = call_bool("/lift_reset_velocity", true);
  ASSERT_NE(reset_after_recovery, nullptr);
  EXPECT_TRUE(reset_after_recovery->success) << reset_after_recovery->message;
  ASSERT_TRUE(call_bool("/lift_reset_velocity", false)->success);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_limit_recovery_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, ResetMayCrossUpperLimitOnlyWithinBoundedSearchWindow)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", "/tmp/joint_hardware_mock_reset_search_zero.cfg"},
      {"lift_startup_motion_guard_enabled", "false"},
      {"reset_velocity_debug_enabled", "true"},
      {"reset_max_search_travel_m", "0.05"},
      {"reset_velocity_timeout_ms", "1000"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_reset_search_bound_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  const auto call_reset_velocity = [&](bool enabled) {
      auto client = client_node->create_client<std_srvs::srv::SetBool>("/lift_reset_velocity");
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enabled;
      auto future = client->async_send_request(request);
      EXPECT_EQ(
        client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  ASSERT_TRUE(call_reset_velocity(true)->success);

  // lift_sign=-1 and 3.333333333 mm/rev: -60,000 units is about +0.02 m.
  backend_view->set_actual_position_units(-60'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  const auto heartbeat_inside_bound = call_reset_velocity(true);
  ASSERT_NE(heartbeat_inside_bound, nullptr);
  EXPECT_TRUE(heartbeat_inside_bound->success) << heartbeat_inside_bound->message;
  EXPECT_NE(heartbeat_inside_bound->message.find("watchdog refreshed"), std::string::npos);

  // Crossing the 0.05 m reset-search bound revokes the special upper-limit
  // allowance. A later heartbeat is rejected as outside software limits.
  backend_view->set_actual_position_units(-180'000);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  const auto heartbeat_outside_bound = call_reset_velocity(true);
  ASSERT_NE(heartbeat_outside_bound, nullptr);
  EXPECT_FALSE(heartbeat_outside_bound->success);
  EXPECT_NE(
    heartbeat_outside_bound->message.find("outside software limits"), std::string::npos);

  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_reset_search_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(MockLiftHardware, DebouncesTransientModeMismatchButLatchesPersistentMismatch)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto backend = std::make_unique<joint_hardware::lift::MockLiftEthercatBackend>();
  auto * backend_view = backend.get();
  joint_hardware::LiftHardware hardware(std::move(backend));
  auto info = make_lift_hardware_info({
      {"zero_offset_file", "/tmp/joint_hardware_mock_mode_zero.cfg"},
      {"lift_startup_motion_guard_enabled", "false"},
      {"mode_mismatch_debounce_cycles", "3"},
    });
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("joint_lift_mode_debounce_test");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  auto brake_client = client_node->create_client<std_srvs::srv::SetBool>(
    "/lift_brake_command");
  ASSERT_TRUE(brake_client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = brake_client->async_send_request(request);
  ASSERT_EQ(
    client_executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(10'000'000);
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }

  // Two bad samples are below the configured threshold and must not latch.
  backend_view->set_mode(8);
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  }
  backend_view->set_mode(9);
  auto commands = hardware.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_interface_name() == "position") {
      command.set_value(-0.10);
    } else if (command.get_interface_name() == "velocity") {
      command.set_value(-0.02);
    }
  }
  bool motion_seen = false;
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    motion_seen = motion_seen || backend_view->last_output().target_velocity_units_per_s != 0;
  }
  EXPECT_TRUE(motion_seen);

  // A persistent mismatch reaches the threshold and remains safely latched.
  backend_view->set_mode(8);
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(backend_view->last_output().control_word, 0x0006);
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);
  backend_view->set_mode(9);
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(backend_view->last_output().target_velocity_units_per_s, 0);

  ASSERT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  std::filesystem::remove("/tmp/joint_hardware_mock_mode_zero.cfg");
  client_executor.remove_node(client_node);
  rclcpp::shutdown();
}

}  // namespace

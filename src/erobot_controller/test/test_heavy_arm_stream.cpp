#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "erobot_controller/CommandProcessor_arm.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr std::size_t kArmJointCount = 14;

TEST(HeavyArmStream, MonotonicFollowWritesRawPositionAndVelocitySetpoints)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  const std::vector<std::string> names = {
    "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
    "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
  CommandProcessor_arm processor(kArmJointCount, names);

  std::array<double, kArmJointCount> state_position{};
  std::array<double, kArmJointCount> state_velocity{};
  std::array<double, kArmJointCount> state_status{};
  std::array<double, kArmJointCount> state_error{};
  std::array<double, kArmJointCount> command_position{};
  std::array<double, kArmJointCount> command_velocity{};
  std::array<double, kArmJointCount> command_effort{};
  std::array<double, kArmJointCount> command_power{};
  std::array<double, kArmJointCount> command_mode{};
  state_status.fill(39.0);

  std::vector<hardware_interface::StateInterface> position_state_interfaces;
  std::vector<hardware_interface::StateInterface> velocity_state_interfaces;
  std::vector<hardware_interface::StateInterface> status_state_interfaces;
  std::vector<hardware_interface::StateInterface> error_state_interfaces;
  std::vector<hardware_interface::CommandInterface> position_command_interfaces;
  std::vector<hardware_interface::CommandInterface> velocity_command_interfaces;
  std::vector<hardware_interface::CommandInterface> effort_command_interfaces;
  std::vector<hardware_interface::CommandInterface> power_command_interfaces;
  std::vector<hardware_interface::CommandInterface> mode_command_interfaces;
  position_state_interfaces.reserve(kArmJointCount);
  velocity_state_interfaces.reserve(kArmJointCount);
  status_state_interfaces.reserve(kArmJointCount);
  error_state_interfaces.reserve(kArmJointCount);
  position_command_interfaces.reserve(kArmJointCount);
  velocity_command_interfaces.reserve(kArmJointCount);
  effort_command_interfaces.reserve(kArmJointCount);
  power_command_interfaces.reserve(kArmJointCount);
  mode_command_interfaces.reserve(kArmJointCount);

  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    position_state_interfaces.emplace_back(names[index], "position", &state_position[index]);
    velocity_state_interfaces.emplace_back(names[index], "velocity", &state_velocity[index]);
    status_state_interfaces.emplace_back(names[index], "status", &state_status[index]);
    error_state_interfaces.emplace_back(names[index], "error_code", &state_error[index]);
    position_command_interfaces.emplace_back(names[index], "position", &command_position[index]);
    velocity_command_interfaces.emplace_back(names[index], "velocity", &command_velocity[index]);
    effort_command_interfaces.emplace_back(names[index], "effort", &command_effort[index]);
    power_command_interfaces.emplace_back(names[index], "power_enable", &command_power[index]);
    mode_command_interfaces.emplace_back(names[index], "mode", &command_mode[index]);
  }

  std::vector<hardware_interface::LoanedStateInterface> position_states;
  std::vector<hardware_interface::LoanedStateInterface> velocity_states;
  std::vector<hardware_interface::LoanedStateInterface> status_states;
  std::vector<hardware_interface::LoanedStateInterface> error_states;
  std::vector<hardware_interface::LoanedCommandInterface> position_commands;
  std::vector<hardware_interface::LoanedCommandInterface> velocity_commands;
  std::vector<hardware_interface::LoanedCommandInterface> effort_commands;
  std::vector<hardware_interface::LoanedCommandInterface> power_commands;
  std::vector<hardware_interface::LoanedCommandInterface> mode_commands;
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    position_states.emplace_back(position_state_interfaces[index]);
    velocity_states.emplace_back(velocity_state_interfaces[index]);
    status_states.emplace_back(status_state_interfaces[index]);
    error_states.emplace_back(error_state_interfaces[index]);
    position_commands.emplace_back(position_command_interfaces[index]);
    velocity_commands.emplace_back(velocity_command_interfaces[index]);
    effort_commands.emplace_back(effort_command_interfaces[index]);
    power_commands.emplace_back(power_command_interfaces[index]);
    mode_commands.emplace_back(mode_command_interfaces[index]);
  }

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> status_state_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> error_state_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_command_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_command_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> power_command_refs;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> mode_command_refs;
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    position_state_refs.emplace_back(position_states[index]);
    velocity_state_refs.emplace_back(velocity_states[index]);
    status_state_refs.emplace_back(status_states[index]);
    error_state_refs.emplace_back(error_states[index]);
    position_command_refs.emplace_back(position_commands[index]);
    velocity_command_refs.emplace_back(velocity_commands[index]);
    effort_command_refs.emplace_back(effort_commands[index]);
    power_command_refs.emplace_back(power_commands[index]);
    mode_command_refs.emplace_back(mode_commands[index]);
  }

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> no_compensation_states;
  processor.updateJointState(
    position_state_refs, velocity_state_refs, status_state_refs, error_state_refs,
    no_compensation_states);

  double previous_position = 0.0;
  for (std::size_t sample = 0; sample <= 100; ++sample) {
    std::array<double, kArmJointCount> position{};
    std::array<double, kArmJointCount> velocity{};
    std::array<double, kArmJointCount> acceleration{};
    position[13] = 0.5 * static_cast<double>(sample) / 100.0;
    velocity[13] = sample == 100 ? 0.0 : 0.05;
    ASSERT_TRUE(processor.processHeavyPositionCommand(
      position, velocity, acceleration, true, true, 0.3, 1.0));
    processor.updateJointCommands(
      position_state_refs, position_command_refs, velocity_command_refs, effort_command_refs,
      power_command_refs, mode_command_refs, false);

    EXPECT_GE(command_position[13] + 1.0e-12, previous_position);
    EXPECT_GE(command_velocity[13], -1.0e-12);
    EXPECT_DOUBLE_EQ(command_position[13], position[13]);
    EXPECT_DOUBLE_EQ(command_velocity[13], velocity[13]);
    EXPECT_DOUBLE_EQ(command_mode[13], 5.0);
    previous_position = command_position[13];
    state_position[13] = command_position[13];
    state_velocity[13] = command_velocity[13];
    processor.updateJointState(
      position_state_refs, velocity_state_refs, status_state_refs, error_state_refs,
      no_compensation_states);
  }

  rclcpp::shutdown();
}

}  // namespace

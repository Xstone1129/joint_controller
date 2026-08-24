#include "joint_hardware/joint_hardware.hpp"

#include <exception>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{
namespace
{

bool parse_bool_parameter(const std::string & value, bool & parsed_value)
{
  if (value == "1" || value == "true" || value == "True" || value == "TRUE") {
    parsed_value = true;
    return true;
  }
  if (value == "0" || value == "false" || value == "False" || value == "FALSE") {
    parsed_value = false;
    return true;
  }
  return false;
}

}  // namespace

hardware_interface::CallbackReturn JointHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("joint_hardware"),
      "The hardware component must define at least one joint");
    return hardware_interface::CallbackReturn::ERROR;
  }

  simulate_ = false;
  const auto simulate_parameter = info_.hardware_parameters.find("simulate");
  if (simulate_parameter != info_.hardware_parameters.end() &&
    !parse_bool_parameter(simulate_parameter->second, simulate_))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("joint_hardware"),
      "Hardware parameter 'simulate' must be one of: 0, 1, false, true");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    if (joint.name.empty() || joint.state_interfaces.empty() ||
      joint.command_interfaces.empty())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("joint_hardware"),
        "Each joint must have a name and at least one state and command interface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    JointData data;
    data.state_values.assign(joint.state_interfaces.size(), 0.0);
    data.command_values.assign(joint.command_interfaces.size(), 0.0);

    for (std::size_t index = 0; index < joint.state_interfaces.size(); ++index) {
      const auto & interface = joint.state_interfaces[index];
      if (!data.state_offsets.emplace(interface.name, index).second) {
        RCLCPP_ERROR(
          rclcpp::get_logger("joint_hardware"),
          "Joint '%s' declares duplicate state interface '%s'",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!interface.initial_value.empty()) {
        try {
          data.state_values[index] = std::stod(interface.initial_value);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            rclcpp::get_logger("joint_hardware"),
            "Invalid initial value '%s' for joint '%s' state interface '%s': %s",
            interface.initial_value.c_str(), joint.name.c_str(), interface.name.c_str(),
            error.what());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }

    for (std::size_t index = 0; index < joint.command_interfaces.size(); ++index) {
      const auto & interface = joint.command_interfaces[index];
      if (!data.command_offsets.emplace(interface.name, index).second) {
        RCLCPP_ERROR(
          rclcpp::get_logger("joint_hardware"),
          "Joint '%s' declares duplicate command interface '%s'",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    joints_.push_back(std::move(data));
  }

  initialized_ = true;
  configured_ = false;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JointHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!initialized_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    for (const auto & command : joint.command_offsets) {
      const auto state = joint.state_offsets.find(command.first);
      if (state != joint.state_offsets.end()) {
        joint.command_values[command.second] = joint.state_values[state->second];
      }
    }
  }

  configured_ = true;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JointHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!configured_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!simulate_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("joint_hardware"),
      "No peripheral-motor transport is configured yet; set hardware parameter "
      "'simulate' to true or implement the transport in read()/write() before activation");
    return hardware_interface::CallbackReturn::ERROR;
  }

  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JointHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JointHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  if (!initialized_) {
    return interfaces;
  }

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint = info_.joints[joint_index];
    for (std::size_t interface_index = 0;
      interface_index < joint.state_interfaces.size(); ++interface_index)
    {
      interfaces.emplace_back(
        joint.name,
        joint.state_interfaces[interface_index].name,
        &joints_[joint_index].state_values[interface_index]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> JointHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (!initialized_) {
    return interfaces;
  }

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint = info_.joints[joint_index];
    for (std::size_t interface_index = 0;
      interface_index < joint.command_interfaces.size(); ++interface_index)
    {
      interfaces.emplace_back(
        joint.name,
        joint.command_interfaces[interface_index].name,
        &joints_[joint_index].command_values[interface_index]);
    }
  }
  return interfaces;
}

hardware_interface::return_type JointHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_ || !simulate_) {
    return hardware_interface::return_type::ERROR;
  }

  // Simulation mirrors command values into state values. Replace this block with the
  // peripheral-motor read operation when the transport protocol is available.
  for (auto & joint : joints_) {
    for (const auto & state : joint.state_offsets) {
      const auto command = joint.command_offsets.find(state.first);
      if (command != joint.command_offsets.end()) {
        joint.state_values[state.second] = joint.command_values[command->second];
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JointHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_ || !simulate_) {
    return hardware_interface::return_type::ERROR;
  }

  // The command buffer is already exported to ros2_control. A real implementation
  // should serialize these values and send them to the motor transport here.
  return hardware_interface::return_type::OK;
}

}  // namespace joint_hardware

PLUGINLIB_EXPORT_CLASS(joint_hardware::JointHardware, hardware_interface::SystemInterface)

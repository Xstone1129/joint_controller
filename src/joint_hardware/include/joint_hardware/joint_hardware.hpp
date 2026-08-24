#ifndef JOINT_HARDWARE__JOINT_HARDWARE_HPP_
#define JOINT_HARDWARE__JOINT_HARDWARE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace joint_hardware
{

class JointHardware final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JointHardware)

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
  struct JointData
  {
    std::unordered_map<std::string, std::size_t> state_offsets;
    std::unordered_map<std::string, std::size_t> command_offsets;
    std::vector<double> state_values;
    std::vector<double> command_values;
  };

  std::vector<JointData> joints_;
  bool initialized_{false};
  bool configured_{false};
  bool active_{false};
  bool simulate_{false};
};

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__JOINT_HARDWARE_HPP_

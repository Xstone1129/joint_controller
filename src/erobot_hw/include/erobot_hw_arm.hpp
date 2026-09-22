// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_6__RRBOT_ACTUATOR_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_6__RRBOT_ACTUATOR_HPP_

#include <memory>
#include <chrono>
#include <string>
#include <vector>

//#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "arm_axis_mapping.hpp"
#include "ethercat_arm_data_struct.hpp"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/permissions.hpp>


using namespace boost::interprocess;

namespace Robot_arm_hardware_interface
{
class Robot_arm_Ethercat_Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot_arm_Ethercat_Hardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  //shm
  void init_shm_desire();
  void init_shm_real();

  uint8_t bais_;

private:
  bool is_igh_driver_alive();

  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  double arm_class_;
  double update_rate_;
  double sim_;

  std::vector<MotorState> hw_commands_;
  std::vector<MotorState> hw_states_;
  // ros2_control joint index -> original IGH shared-memory slot.  Never use
  // the ROS configuration order as a proxy for the physical EtherCAT order.
  std::vector<std::size_t> joint_shared_slots_;

  // shm_desire
  ec_app_desire_reg_t * joint_shm_desire_ptr;
  std::shared_ptr<mapped_region> mapped_region_desire_ptr;
  std::shared_ptr<shared_memory_object> shm_desire_ptr;

  // shm_real
  ec_app_real_reg_t * joint_shm_real_ptr;
  std::shared_ptr<mapped_region> mapped_region_real_ptr;
  std::shared_ptr<shared_memory_object> shm_real_ptr;

  std::vector<uint16_t> hw_commands_uint; //rad to degree to uint16
  std::vector<uint16_t> hw_states_uint; //uint16

  std::chrono::steady_clock::time_point last_igh_driver_check_{};
  int igh_driver_pid_{-1};
  bool igh_driver_alive_{false};

};

}  // namespace eRobot5_hardware_interface

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_6__RRBOT_ACTUATOR_HPP_
// Robot_arm_hardware_interface/Robot_arm_Ethercat_Hardware

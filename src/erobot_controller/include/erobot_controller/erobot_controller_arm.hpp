// Copyright 2023 ros2_control Development Team
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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_

#include <chrono>
#include <atomic>
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "erobot_controller/CommandProcessor_arm.hpp"
#include "erobot_controller/CubicPlanner.hpp"
#include "robot_control_msg/msg/robotarmservomsg.hpp"
#include "robot_control_msg/msg/robotarmmovel.hpp"
#include "robot_control_msg/msg/robotarmjoint.hpp"
#include "robot_control_msg/msg/heavy_upper_body_gateway_command_v1.hpp"
#include "robot_control_msg/srv/set_arm_control_mode.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace erobot_controller_arm
{

struct HeavyArmSetpoint
{
  uint64_t sequence{0};
  uint8_t mode{0};
  uint8_t field_mask{0};
  std::array<double, 14> position{};
  std::array<double, 14> velocity{};
  std::array<double, 14> acceleration{};
};

class RobotController_arm : public controller_interface::ControllerInterface
{
public:
  RobotController_arm();
  // ~RobotController_leg();
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  void handlePowerService(
    const std::shared_ptr<robot_control_msg::srv::SetRobotPower::Request> request,
    std::shared_ptr<robot_control_msg::srv::SetRobotPower::Response> response);
  bool waitForPowerFeedback(
    uint64_t generation,
    bool enable,
    std::chrono::milliseconds timeout,
    std::string &description);
  uint64_t submitPowerRequest(bool enable);
  void updatePowerFeedbackSnapshot();

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> gravity_compensation_joint_names_;

  // 类实例
  std::unique_ptr<CommandProcessor_arm> command_processor_;

  rclcpp::Subscription<robot_control_msg::msg::Robotarmservomsg>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<robot_control_msg::msg::Robotarmmovel>::SharedPtr cmd_cartesian_subscriber_;
  rclcpp::Subscription<robot_control_msg::msg::Robotarmjoint>::SharedPtr arm_joint_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr poweron_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hardware_mode_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heavy_lease_gate_subscriber_;
  rclcpp::Subscription<robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1>::SharedPtr
    heavy_command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr heavy_ack_publisher_;
  rclcpp::Service<robot_control_msg::srv::SetArmControlMode>::SharedPtr control_mode_service_;
  rclcpp::Service<robot_control_msg::srv::SetRobotPower>::SharedPtr power_service_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<robot_control_msg::msg::Robotarmservomsg>> cmd_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<robot_control_msg::msg::Robotarmmovel>> cmd_cartesian_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<robot_control_msg::msg::Robotarmjoint>> arm_joint_buffer_;
  realtime_tools::RealtimeBuffer<HeavyArmSetpoint> heavy_command_buffer_;

  std::shared_ptr<robot_control_msg::msg::Robotarmservomsg> cmd_buffer_msg_;
  std::shared_ptr<robot_control_msg::msg::Robotarmmovel> cmd_cartesian_buffer_msg_;
  std::shared_ptr<robot_control_msg::msg::Robotarmjoint> arm_joint_buffer_msg_;

  rclcpp::Time start_time_;
  bool new_msg_ = 0;
  bool new_cartesian_msg_ = 0;
  bool new_arm_joint_msg_ = 0;
  std::atomic<int> pending_control_mode_request_{-1};
  std::atomic<int> pending_power_request_{-1};
  std::atomic<int> pending_hardware_mode_request_{-1};
  std::atomic<int> active_control_mode_cache_{static_cast<int>(ControlMode::POSITION)};
  std::atomic<bool> heavy_lease_active_{false};
  std::atomic<uint64_t> heavy_command_generation_{0};
  uint64_t consumed_heavy_command_generation_{0};
  uint64_t last_heavy_sequence_{0};
  bool heavy_lease_active_last_update_{false};
  double heavy_max_velocity_{0.3};
  double heavy_max_acceleration_{1.0};
  // This is set only by the managed SIM controller parameter file.  It never
  // changes arm power status and requires the lower gateway's active lease.
  bool allow_disabled_simulation_execution_{false};

  // Requests are consumed only by update(), which is also the sole caller of
  // CommandProcessor_arm::requestPowerEnable(). The service waits on this
  // snapshot without holding any lock used by the controller loop.
  std::atomic<uint64_t> power_request_generation_{0};
  std::atomic<uint64_t> power_applied_generation_{0};
  std::mutex power_service_mutex_;
  std::mutex power_feedback_mutex_;
  std::condition_variable power_feedback_cv_;
  std::array<int, 14> power_status_codes_{};
  std::array<bool, 14> power_enabled_states_{};
  bool power_command_enabled_{false};
  bool power_all_enabled_{false};
  std::atomic<bool> controller_active_{false};


  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_,joint_velocity_command_interface_,joint_effort_command_interface_,joint_power_command_interface_,joint_mode_command_interface_;


  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_,joint_velocity_state_interface_,joint_effort_state_interface_,joint_motor_encoder_0_state_interface_,joint_motor_encoder_1_state_interface_,
    joint_mode_state_interface_,joint_error_code_state_interface_,joint_power_state_interface_,joint_compensation_position_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"velocity", &joint_velocity_command_interface_},
      {"effort", &joint_effort_command_interface_},
      {"mode", &joint_mode_command_interface_},
      {"power_enable", &joint_power_command_interface_}
    };

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_},
      {"effort", &joint_effort_state_interface_},
      {"motor_encoder_0", &joint_motor_encoder_0_state_interface_},
      {"motor_encoder_1", &joint_motor_encoder_1_state_interface_},
      {"status", &joint_mode_state_interface_},
      {"error_code", &joint_error_code_state_interface_},
      {"power_enable", &joint_power_state_interface_}
    };
    double get_pos[14] = {0.0};
};

}  

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_

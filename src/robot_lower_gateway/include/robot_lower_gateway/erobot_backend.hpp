#ifndef ROBOT_LOWER_GATEWAY__EROBOT_BACKEND_HPP_
#define ROBOT_LOWER_GATEWAY__EROBOT_BACKEND_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/arm_control_mode_status.hpp"
#include "robot_control_msg/msg/arm_motion_status.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/cartesian_execution_status.hpp"
#include "robot_control_msg/msg/end_effector_pose.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "robot_lower_gateway/backend.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_lower_gateway
{

class ErobotBackend final : public Backend
{
public:
  std::string name() const override;
  BackendCapabilities capabilities() const override;
  void configure(
    rclcpp::Node & node,
    const BackendConfiguration & configuration) override;
  BackendSnapshot snapshot() const override;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static double ageSeconds(const SteadyTime & then, bool received);
  bool containsCode(const std::vector<std::int32_t> & values, std::int32_t code) const;
  DriveState normalizeDriveState(
    std::int32_t status_code, bool enabled) const;

  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void handlePowerStatus(const robot_control_msg::msg::ArmPowerStatus::SharedPtr message);
  void handleControlMode(
    const robot_control_msg::msg::ArmControlModeStatus::SharedPtr message);
  void handleMotionStatus(const robot_control_msg::msg::ArmMotionStatus::SharedPtr message);
  void handleTcpPose(const robot_control_msg::msg::EndEffectorPose::SharedPtr message);
  void handleExecutionStatus(
    const robot_control_msg::msg::CartesianExecutionStatus::SharedPtr message);
  void handleWorkspaceStatus(const robot_control_msg::msg::WorkspaceStatus::SharedPtr message);

  mutable std::mutex mutex_;
  BackendConfiguration configuration_;
  BackendSnapshot snapshot_;
  std::unordered_map<std::string, std::size_t> native_joint_index_;

  SteadyTime joint_state_time_{};
  SteadyTime power_status_time_{};
  SteadyTime control_mode_time_{};
  SteadyTime motion_status_time_{};
  SteadyTime tcp_pose_time_{};
  SteadyTime execution_status_time_{};
  bool starting_command_seen_{false};
  std::uint32_t last_starting_command_seq_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmPowerStatus>::SharedPtr
    power_status_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmControlModeStatus>::SharedPtr
    control_mode_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr
    motion_status_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::EndEffectorPose>::SharedPtr
    tcp_pose_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::CartesianExecutionStatus>::SharedPtr
    execution_status_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::WorkspaceStatus>::SharedPtr
    workspace_status_subscription_;
};

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__EROBOT_BACKEND_HPP_

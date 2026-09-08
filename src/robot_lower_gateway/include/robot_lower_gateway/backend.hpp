#ifndef ROBOT_LOWER_GATEWAY__BACKEND_HPP_
#define ROBOT_LOWER_GATEWAY__BACKEND_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/node.hpp"

namespace robot_lower_gateway
{

enum class DriveState : std::uint8_t
{
  kUnavailable = 0,
  kNotEnabled = 1,
  kEnabled = 2,
  kUnknown = 3,
};

struct BackendConfiguration
{
  std::vector<std::string> canonical_joint_names;
  std::vector<std::string> native_joint_names;

  std::string joint_state_topic;
  std::string power_status_topic;
  std::string control_mode_status_topic;
  std::string motion_status_topic;
  std::string tcp_pose_topic;
  std::string execution_status_topic;
  std::string workspace_status_topic;

  std::vector<std::int32_t> enabled_status_codes;
  std::vector<std::int32_t> unavailable_status_codes;
};

struct BackendCapabilities
{
  bool observes_joint_state{false};
  bool observes_power{false};
  bool observes_control_mode{false};
  bool observes_motion{false};
  bool observes_tcp_pose{false};
  bool observes_execution{false};

  // Command ownership stays disabled until an adapter explicitly implements it.
  bool commands_power{false};
  bool commands_control_mode{false};
  bool commands_joint_position{false};
  bool commands_cartesian{false};
};

struct BackendSnapshot
{
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  std::vector<double> joint_efforts;
  bool joint_state_received{false};
  bool joint_state_complete{false};
  bool joint_state_valid{false};
  std::string joint_state_error;
  double joint_state_age_sec{std::numeric_limits<double>::infinity()};

  std::vector<std::int32_t> raw_drive_status_codes;
  std::vector<DriveState> drive_states;
  bool power_status_received{false};
  bool power_command_enabled{false};
  bool all_drives_enabled{false};
  double power_status_age_sec{std::numeric_limits<double>::infinity()};

  bool control_mode_received{false};
  std::uint8_t requested_control_mode{0};
  std::uint8_t active_control_mode{0};
  bool position_command_ready{false};
  double control_mode_age_sec{std::numeric_limits<double>::infinity()};

  bool motion_status_received{false};
  bool is_moving{false};
  bool goal_reached{false};
  double motion_status_age_sec{std::numeric_limits<double>::infinity()};

  bool tcp_pose_received{false};
  geometry_msgs::msg::Pose left_tcp_pose;
  geometry_msgs::msg::Pose right_tcp_pose;
  double tcp_pose_age_sec{std::numeric_limits<double>::infinity()};

  bool execution_status_received{false};
  std::uint8_t execution_state{0};
  std::uint32_t planned_points{0};
  double planned_duration_sec{0.0};
  std::string execution_message;
  double execution_status_age_sec{std::numeric_limits<double>::infinity()};

  bool workspace_status_received{false};
  std::uint32_t workspace_command_seq{0};
  std::uint32_t workspace_generation{0};
  std::uint8_t workspace_state{0};
  std::uint8_t workspace_mode{0};
};

class Backend
{
public:
  virtual ~Backend() = default;

  virtual std::string name() const = 0;
  virtual BackendCapabilities capabilities() const = 0;
  virtual void configure(rclcpp::Node & node, const BackendConfiguration & configuration) = 0;
  virtual BackendSnapshot snapshot() const = 0;
};

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__BACKEND_HPP_

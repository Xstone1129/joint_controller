#ifndef ROBOT_LOWER_GATEWAY__LIFT_STATUS_PARSER_HPP_
#define ROBOT_LOWER_GATEWAY__LIFT_STATUS_PARSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace robot_lower_gateway
{

struct LiftDriverStatus
{
  bool feedback_fresh{false};
  bool ethercat_operational{false};
  std::int32_t working_counter{0};
  bool working_counter_ok{false};
  bool initialized{false};
  std::string cia402_state;
  std::uint16_t status_word{0};
  std::uint16_t error_code{0};
  std::int8_t mode_display{0};
  bool command_enabled{false};
  bool enabled{false};
  bool brake_unlocked{false};
  bool estop_latched{false};
  bool motion_blocked{false};
  bool quick_stop_active{false};
  double position{0.0};
  double velocity{0.0};
  double command_position{0.0};
  double command_velocity{0.0};
  double command_acceleration{0.0};
  bool pdo_target_available{false};
  double target_rpm{0.0};
  std::int32_t target_velocity_units{0};
  bool velocity_mode_active{false};
  std::string fault_reason;
};

struct LiftControlStatus
{
  std::string mode;
  bool trajectory_active{false};
  bool jog_active{false};
};

struct LiftSafetyStatus
{
  bool fault{false};
  bool motion_blocked{false};
  std::string fault_reason;
};

constexpr std::int8_t kExpectedLiftModeDisplay = 9;
constexpr std::uint16_t kCia402StateMask = 0x006f;
constexpr std::uint16_t kCia402ReadyToSwitchOn = 0x0021;
constexpr std::uint16_t kCia402SwitchedOn = 0x0023;
constexpr std::uint16_t kCia402OperationEnabled = 0x0027;

bool parseLiftDriverStatus(
  const std::string & json, LiftDriverStatus & status, std::string & error);
bool parseLiftControlStatus(
  const std::string & json, LiftControlStatus & status, std::string & error);
bool liftPowerStateMatches(const LiftDriverStatus & status, bool enabled);
LiftSafetyStatus evaluateLiftSafety(const LiftDriverStatus & status);
bool liftEnablePreconditionsMet(
  const LiftDriverStatus & status, bool workspace_running, std::string & reason);
bool liftMotionPreconditionsMet(
  const LiftDriverStatus & status, bool workspace_running, std::string & reason);
bool liftCommandRequestValid(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & values,
  double velocity_mps,
  double acceleration_mps2,
  const std::string & expected_joint_name,
  std::string & reason);

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__LIFT_STATUS_PARSER_HPP_

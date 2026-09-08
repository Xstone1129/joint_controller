#include "robot_lower_gateway/feedback_policy.hpp"

#include <cmath>

#include "robot_control_msg/msg/cartesian_execution_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"

namespace robot_lower_gateway
{

bool executionStatusRequiresFreshness(std::uint8_t state)
{
  using Status = robot_control_msg::msg::CartesianExecutionStatus;
  return state == Status::PLANNING || state == Status::EXECUTING;
}

bool executionStatusIsStale(
  bool received, std::uint8_t state, double age_sec, double stale_sec)
{
  return received && executionStatusRequiresFreshness(state) &&
         (!std::isfinite(age_sec) || age_sec > stale_sec);
}

bool isNewWorkspaceStarting(
  std::uint8_t state, std::uint32_t command_seq,
  bool previous_start_seen, std::uint32_t previous_start_command_seq)
{
  return state == robot_control_msg::msg::WorkspaceStatus::STARTING &&
         (!previous_start_seen || command_seq != previous_start_command_seq);
}

}  // namespace robot_lower_gateway

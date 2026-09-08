#ifndef ROBOT_LOWER_GATEWAY__FEEDBACK_POLICY_HPP_
#define ROBOT_LOWER_GATEWAY__FEEDBACK_POLICY_HPP_

#include <cstdint>

namespace robot_lower_gateway
{

bool executionStatusRequiresFreshness(std::uint8_t state);

bool executionStatusIsStale(
  bool received, std::uint8_t state, double age_sec, double stale_sec);

bool isNewWorkspaceStarting(
  std::uint8_t state, std::uint32_t command_seq,
  bool previous_start_seen, std::uint32_t previous_start_command_seq);

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__FEEDBACK_POLICY_HPP_

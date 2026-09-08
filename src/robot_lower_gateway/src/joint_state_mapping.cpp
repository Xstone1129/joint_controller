#include "robot_lower_gateway/joint_state_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace robot_lower_gateway
{
namespace
{

void appendError(std::string & error, const std::string & issue)
{
  if (!error.empty()) {
    error += "; ";
  }
  error += issue;
}

}  // namespace

JointStateMappingResult mapJointState(
  const sensor_msgs::msg::JointState & message,
  const std::vector<std::string> & canonical_joint_names,
  const std::vector<std::string> & native_joint_names)
{
  JointStateMappingResult result;
  const auto target_count = canonical_joint_names.size();
  result.positions.assign(target_count, std::numeric_limits<double>::quiet_NaN());

  if (target_count == 0 || target_count != native_joint_names.size()) {
    result.error = "invalid canonical/native joint configuration";
    return result;
  }
  if (message.position.size() != message.name.size()) {
    result.error = "position length " + std::to_string(message.position.size()) +
      " does not match name length " + std::to_string(message.name.size());
    return result;
  }
  if (!message.velocity.empty() && message.velocity.size() != message.name.size()) {
    result.error = "nonempty velocity length " + std::to_string(message.velocity.size()) +
      " does not match name length " + std::to_string(message.name.size());
    return result;
  }
  if (!message.effort.empty() && message.effort.size() != message.name.size()) {
    result.error = "nonempty effort length " + std::to_string(message.effort.size()) +
      " does not match name length " + std::to_string(message.name.size());
    return result;
  }

  if (!message.velocity.empty()) {
    result.velocities.assign(target_count, std::numeric_limits<double>::quiet_NaN());
  }
  if (!message.effort.empty()) {
    result.efforts.assign(target_count, std::numeric_limits<double>::quiet_NaN());
  }

  std::unordered_map<std::string, std::size_t> target_indices;
  target_indices.reserve(target_count);
  for (std::size_t index = 0; index < target_count; ++index) {
    target_indices.emplace(native_joint_names[index], index);
  }

  std::vector<bool> seen(target_count, false);
  for (std::size_t source_index = 0; source_index < message.name.size(); ++source_index) {
    const auto target = target_indices.find(message.name[source_index]);
    if (target == target_indices.end()) {
      appendError(result.error, "unknown joint '" + message.name[source_index] + "'");
      continue;
    }
    const auto target_index = target->second;
    if (seen[target_index]) {
      appendError(result.error, "duplicate joint '" + message.name[source_index] + "'");
      continue;
    }
    seen[target_index] = true;

    if (!std::isfinite(message.position[source_index])) {
      appendError(result.error, "non-finite position for '" + message.name[source_index] + "'");
    } else {
      result.positions[target_index] = message.position[source_index];
    }
    if (!message.velocity.empty()) {
      if (!std::isfinite(message.velocity[source_index])) {
        appendError(result.error, "non-finite velocity for '" + message.name[source_index] + "'");
      } else {
        result.velocities[target_index] = message.velocity[source_index];
      }
    }
    if (!message.effort.empty()) {
      if (!std::isfinite(message.effort[source_index])) {
        appendError(result.error, "non-finite effort for '" + message.name[source_index] + "'");
      } else {
        result.efforts[target_index] = message.effort[source_index];
      }
    }
  }

  for (std::size_t index = 0; index < seen.size(); ++index) {
    if (!seen[index]) {
      appendError(result.error, "missing joint '" + native_joint_names[index] + "'");
    }
  }
  result.complete = std::all_of(seen.begin(), seen.end(), [](bool value) {return value;});
  result.valid = result.complete && result.error.empty();
  return result;
}

}  // namespace robot_lower_gateway

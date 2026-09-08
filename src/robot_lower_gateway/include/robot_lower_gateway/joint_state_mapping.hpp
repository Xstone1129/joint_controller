#ifndef ROBOT_LOWER_GATEWAY__JOINT_STATE_MAPPING_HPP_
#define ROBOT_LOWER_GATEWAY__JOINT_STATE_MAPPING_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_lower_gateway
{

struct JointStateMappingResult
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  bool complete{false};
  bool valid{false};
  std::string error;
};

JointStateMappingResult mapJointState(
  const sensor_msgs::msg::JointState & message,
  const std::vector<std::string> & canonical_joint_names,
  const std::vector<std::string> & native_joint_names);

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__JOINT_STATE_MAPPING_HPP_

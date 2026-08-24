#ifndef ROBOT_CONTROL__JOINT_COMPLETION_POLICY_HPP_
#define ROBOT_CONTROL__JOINT_COMPLETION_POLICY_HPP_

namespace robot_control
{

inline bool jointFeedbackSettlingCandidate(
  bool positions_reached,
  bool has_post_command_status,
  bool is_moving,
  bool motion_detected)
{
  return positions_reached && has_post_command_status && !is_moving && motion_detected;
}

inline bool jointMotionCompleted(
  bool positions_reached,
  bool has_post_command_status,
  bool is_moving,
  bool controller_goal_reached,
  bool motion_detected,
  bool feedback_settled)
{
  return positions_reached && has_post_command_status && !is_moving &&
         (controller_goal_reached || (motion_detected && feedback_settled));
}

}  // namespace robot_control

#endif  // ROBOT_CONTROL__JOINT_COMPLETION_POLICY_HPP_

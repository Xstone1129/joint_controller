#include <gtest/gtest.h>

#include "robot_control/joint_completion_policy.hpp"

TEST(JointCompletionPolicy, StartsSettlingOnlyAfterFreshStoppedTargetFeedback)
{
  EXPECT_TRUE(robot_control::jointFeedbackSettlingCandidate(true, true, false, true));
  EXPECT_FALSE(robot_control::jointFeedbackSettlingCandidate(false, true, false, true));
  EXPECT_FALSE(robot_control::jointFeedbackSettlingCandidate(true, false, false, true));
  EXPECT_FALSE(robot_control::jointFeedbackSettlingCandidate(true, true, true, true));
  EXPECT_FALSE(robot_control::jointFeedbackSettlingCandidate(true, true, false, false));
}

TEST(JointCompletionPolicy, AcceptsControllerConfirmation)
{
  EXPECT_TRUE(robot_control::jointMotionCompleted(true, true, false, true, false, false));
}

TEST(JointCompletionPolicy, AcceptsFreshSettledFeedbackAfterObservedMotion)
{
  EXPECT_TRUE(robot_control::jointMotionCompleted(true, true, false, false, true, true));
}

TEST(JointCompletionPolicy, RejectsNoopMovingStaleAndOutOfToleranceStates)
{
  EXPECT_FALSE(robot_control::jointMotionCompleted(true, true, false, false, false, true));
  EXPECT_FALSE(robot_control::jointMotionCompleted(true, true, true, false, true, true));
  EXPECT_FALSE(robot_control::jointMotionCompleted(true, false, false, false, true, true));
  EXPECT_FALSE(robot_control::jointMotionCompleted(false, true, false, true, true, true));
  EXPECT_FALSE(robot_control::jointMotionCompleted(true, true, false, false, true, false));
}

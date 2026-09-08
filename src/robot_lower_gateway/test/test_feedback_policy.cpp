#include <gtest/gtest.h>

#include <limits>

#include "robot_control_msg/msg/cartesian_execution_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "robot_lower_gateway/feedback_policy.hpp"

namespace
{

TEST(FeedbackPolicy, OnlyActiveExecutionStatesExpire)
{
  using Status = robot_control_msg::msg::CartesianExecutionStatus;
  EXPECT_FALSE(robot_lower_gateway::executionStatusIsStale(true, Status::IDLE, 30.0, 3.0));
  EXPECT_FALSE(
    robot_lower_gateway::executionStatusIsStale(true, Status::STREAM_FINISHED, 30.0, 3.0));
  EXPECT_FALSE(robot_lower_gateway::executionStatusIsStale(true, Status::FAILED, 30.0, 3.0));
  EXPECT_FALSE(
    robot_lower_gateway::executionStatusIsStale(true, Status::REJECTED_BUSY, 30.0, 3.0));
  EXPECT_TRUE(robot_lower_gateway::executionStatusIsStale(true, Status::PLANNING, 3.1, 3.0));
  EXPECT_TRUE(robot_lower_gateway::executionStatusIsStale(true, Status::EXECUTING, 3.1, 3.0));
  EXPECT_TRUE(
    robot_lower_gateway::executionStatusIsStale(
      true, Status::EXECUTING, std::numeric_limits<double>::infinity(), 3.0));
  EXPECT_FALSE(robot_lower_gateway::executionStatusIsStale(false, Status::EXECUTING, 30.0, 3.0));
}

TEST(FeedbackPolicy, NewStartingCommandDefinesNewGeneration)
{
  using Workspace = robot_control_msg::msg::WorkspaceStatus;
  EXPECT_TRUE(
    robot_lower_gateway::isNewWorkspaceStarting(
      Workspace::STARTING, 42, false, 0));
  EXPECT_FALSE(
    robot_lower_gateway::isNewWorkspaceStarting(
      Workspace::STARTING, 42, true, 42));
  EXPECT_TRUE(
    robot_lower_gateway::isNewWorkspaceStarting(
      Workspace::STARTING, 43, true, 42));
  EXPECT_FALSE(
    robot_lower_gateway::isNewWorkspaceStarting(
      Workspace::RUNNING, 43, true, 42));
}

}  // namespace

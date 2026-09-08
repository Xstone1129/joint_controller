#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "robot_lower_gateway/joint_state_mapping.hpp"

namespace
{

std::vector<std::string> jointNames()
{
  std::vector<std::string> names;
  for (int index = 0; index < 14; ++index) {
    names.push_back("joint" + std::to_string(index));
  }
  return names;
}

sensor_msgs::msg::JointState fullMessage(const std::vector<std::string> & names)
{
  sensor_msgs::msg::JointState message;
  for (auto iterator = names.rbegin(); iterator != names.rend(); ++iterator) {
    const auto target = static_cast<double>(std::distance(
        names.begin(), std::find(
          names.begin(), names.end(), *iterator)));
    message.name.push_back(*iterator);
    message.position.push_back(target + 0.1);
    message.velocity.push_back(target + 0.2);
    message.effort.push_back(target + 0.3);
  }
  return message;
}

TEST(JointStateMapping, RemapsAllArraysByNameToCanonicalOrder)
{
  const auto names = jointNames();
  const auto result = robot_lower_gateway::mapJointState(fullMessage(names), names, names);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_TRUE(result.complete);
  ASSERT_EQ(result.positions.size(), 14U);
  ASSERT_EQ(result.velocities.size(), 14U);
  ASSERT_EQ(result.efforts.size(), 14U);
  for (std::size_t index = 0; index < names.size(); ++index) {
    EXPECT_DOUBLE_EQ(result.positions[index], static_cast<double>(index) + 0.1);
    EXPECT_DOUBLE_EQ(result.velocities[index], static_cast<double>(index) + 0.2);
    EXPECT_DOUBLE_EQ(result.efforts[index], static_cast<double>(index) + 0.3);
  }
}

TEST(JointStateMapping, PreservesTrulyEmptyOptionalArrays)
{
  const auto names = jointNames();
  auto message = fullMessage(names);
  message.velocity.clear();
  message.effort.clear();

  const auto result = robot_lower_gateway::mapJointState(message, names, names);
  EXPECT_TRUE(result.valid) << result.error;
  EXPECT_TRUE(result.velocities.empty());
  EXPECT_TRUE(result.efforts.empty());
}

TEST(JointStateMapping, RejectsMalformedNonemptyOptionalArrays)
{
  const auto names = jointNames();
  auto velocity_short = fullMessage(names);
  velocity_short.velocity.pop_back();
  auto result = robot_lower_gateway::mapJointState(velocity_short, names, names);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("nonempty velocity length"), std::string::npos);

  auto effort_short = fullMessage(names);
  effort_short.effort.pop_back();
  result = robot_lower_gateway::mapJointState(effort_short, names, names);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("nonempty effort length"), std::string::npos);
}

TEST(JointStateMapping, RejectsDuplicateUnknownMissingAndNonfiniteData)
{
  const auto names = jointNames();

  auto duplicate = fullMessage(names);
  duplicate.name[0] = duplicate.name[1];
  auto result = robot_lower_gateway::mapJointState(duplicate, names, names);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("duplicate joint"), std::string::npos);
  EXPECT_NE(result.error.find("missing joint"), std::string::npos);

  auto unknown = fullMessage(names);
  unknown.name[0] = "not_configured";
  result = robot_lower_gateway::mapJointState(unknown, names, names);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("unknown joint"), std::string::npos);
  EXPECT_NE(result.error.find("missing joint"), std::string::npos);

  auto nonfinite = fullMessage(names);
  nonfinite.position[0] = std::numeric_limits<double>::quiet_NaN();
  result = robot_lower_gateway::mapJointState(nonfinite, names, names);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("non-finite position"), std::string::npos);
}

}  // namespace

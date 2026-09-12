#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace junior_gateway::junior_heavy::v1
{

inline constexpr std::size_t kResourceCount = 15;
inline constexpr std::uint32_t kLayoutCrc32 = 0x8c172bd8U;
inline constexpr std::uint16_t kAllResourceMask = 0x7fffU;
inline constexpr std::string_view kInterfaceSchemaSha256 =
  "c27babf536dfd746b49c66fdefff987fefbf55fbe6201925406eac5f2d3c448a";

inline constexpr std::array<std::string_view, kResourceCount> kResources = {
  "joint_motor",
  "joint_left_arm_1", "joint_left_arm_2", "joint_left_arm_3", "joint_left_arm_4",
  "joint_left_arm_5", "joint_left_arm_6", "joint_left_arm_7",
  "joint_right_arm_1", "joint_right_arm_2", "joint_right_arm_3", "joint_right_arm_4",
  "joint_right_arm_5", "joint_right_arm_6", "joint_right_arm_7",
};

inline constexpr std::string_view kCommandTopic =
  "/ubuntu_lower_gateway/heavy/v1/upper_body/command";
inline constexpr std::string_view kFeedbackTopic =
  "/ubuntu_lower_gateway/heavy/v1/upper_body/feedback";
inline constexpr std::string_view kStatusTopic =
  "/ubuntu_lower_gateway/heavy/v1/upper_body/status";
inline constexpr std::string_view kCapabilitiesService =
  "/ubuntu_lower_gateway/heavy/v1/get_capabilities";
inline constexpr std::string_view kAcquireLeaseService =
  "/ubuntu_lower_gateway/heavy/v1/acquire_execution_lease";
inline constexpr std::string_view kReleaseLeaseService =
  "/ubuntu_lower_gateway/heavy/v1/release_execution_lease";
inline constexpr std::string_view kTorqueService =
  "/ubuntu_lower_gateway/set_robot_power";
inline constexpr std::string_view kLiftBrakeService =
  "/ubuntu_lower_gateway/heavy/v1/lift/set_brake";

}  // namespace junior_gateway::junior_heavy::v1

#include <gtest/gtest.h>

#include <array>
#include <cstddef>

#include "arm_axis_mapping.hpp"

namespace
{

TEST(ArmAxisMapping, MapsEveryNamedJointToTheOriginalIghSlot)
{
  constexpr std::array<std::size_t, 14> expected_slots{{
    12, 10, 13, 1, 11, 9, 6, 8, 4, 7, 0, 3, 5, 2,
  }};
  constexpr std::array<const char *, 14> joint_names{{
    "rjoint6", "rjoint4", "rjoint7", "ljoint2", "rjoint5", "rjoint3", "ljoint7",
    "rjoint2", "ljoint5", "rjoint1", "ljoint1", "ljoint4", "ljoint6", "ljoint3",
  }};

  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    std::size_t shared_slot = Robot_arm_hardware_interface::kArmAxisCount;
    EXPECT_TRUE(
      Robot_arm_hardware_interface::shared_axis_slot_for_joint(
        joint_names[index], shared_slot));
    EXPECT_EQ(shared_slot, expected_slots[index]);
  }
}

TEST(ArmAxisMapping, RejectsUnknownJointNames)
{
  std::size_t shared_slot = 0;
  EXPECT_FALSE(
    Robot_arm_hardware_interface::shared_axis_slot_for_joint(
      "waist_joint", shared_slot));
}

}  // namespace

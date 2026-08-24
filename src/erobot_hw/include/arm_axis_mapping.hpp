#ifndef EROBOT_HW__ARM_AXIS_MAPPING_HPP_
#define EROBOT_HW__ARM_AXIS_MAPPING_HPP_

#include <array>
#include <cstddef>
#include <string_view>

namespace Robot_arm_hardware_interface
{

constexpr std::size_t kArmAxisCount = 14;

// The IGH driver routes physical slaves by EtherCAT alias: 0x1000 + slot.
// These slots are stable even when the slaves are discovered in a different
// physical order on either EtherCAT master.
constexpr std::array<std::string_view, kArmAxisCount> kArmJointNamesBySharedSlot{{
  "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
  "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7",
}};

inline bool shared_axis_slot_for_joint(
  std::string_view joint_name, std::size_t & shared_slot)
{
  for (std::size_t slot = 0; slot < kArmJointNamesBySharedSlot.size(); ++slot) {
    if (kArmJointNamesBySharedSlot[slot] == joint_name) {
      shared_slot = slot;
      return true;
    }
  }
  return false;
}

}  // namespace Robot_arm_hardware_interface

#endif  // EROBOT_HW__ARM_AXIS_MAPPING_HPP_

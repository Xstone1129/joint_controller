#ifndef JOINT_HARDWARE__LIFT__ZERO_OFFSET_STORE_HPP_
#define JOINT_HARDWARE__LIFT__ZERO_OFFSET_STORE_HPP_

#include <cstdint>
#include <string>

namespace joint_hardware::lift
{

struct ZeroOffsetRecord
{
  uint32_t schema{1};
  std::string motor_id;
  uint16_t slave_alias{0};
  uint16_t slave_position{0};
  int32_t zero_offset_units{0};
};

enum class ZeroOffsetLoadResult : uint8_t
{
  missing,
  loaded,
  invalid
};

class ZeroOffsetStore
{
public:
  static ZeroOffsetLoadResult load(
    const std::string & path,
    const std::string & expected_motor_id,
    uint16_t expected_alias,
    uint16_t expected_position,
    ZeroOffsetRecord & record,
    std::string & error);

  static bool save_atomic(
    const std::string & path, const ZeroOffsetRecord & record, std::string & error);
};

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ZERO_OFFSET_STORE_HPP_

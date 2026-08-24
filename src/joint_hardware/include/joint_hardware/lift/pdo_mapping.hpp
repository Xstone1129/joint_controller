#ifndef JOINT_HARDWARE__LIFT__PDO_MAPPING_HPP_
#define JOINT_HARDWARE__LIFT__PDO_MAPPING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "joint_hardware/lift/ethercat_backend.hpp"

namespace joint_hardware::lift
{

struct PdoMappingEntry
{
  uint16_t index{0};
  uint8_t subindex{0};
  uint8_t bit_length{0};
};

constexpr uint32_t pdo_mapping_value(const PdoMappingEntry & entry) noexcept
{
  return (static_cast<uint32_t>(entry.index) << 16U) |
         (static_cast<uint32_t>(entry.subindex) << 8U) |
         static_cast<uint32_t>(entry.bit_length);
}

constexpr std::array<PdoMappingEntry, 3> kLiftRxPdoMapping{{
  {0x6040, 0x00, 16},
  {0x60FF, 0x00, 32},
  {0x60B2, 0x00, 16},
}};

// This is the 17-byte CSV feedback profile used by LiftTxPdo. The manual's
// factory TXPDO table also exposes probe objects 60B9/60BA; they are not needed
// for CSV velocity control, so 606C is selected explicitly and 60FD is kept as
// a reserved digital-input extension.
constexpr std::array<PdoMappingEntry, 6> kLiftTxPdoMapping{{
  {0x6041, 0x00, 16},
  {0x603F, 0x00, 16},
  {0x6061, 0x00, 8},
  {0x6064, 0x00, 32},
  {0x606C, 0x00, 32},
  {0x60FD, 0x00, 32},
}};

static_assert(
  (16U + 32U + 16U) == 64U, "lift RxPDO must be exactly 8 bytes");
static_assert(
  (16U + 16U + 8U + 32U + 32U + 32U) == 136U,
  "lift TxPDO must be exactly 17 bytes");

struct PdoSdoWrite
{
  uint16_t index{0};
  uint8_t subindex{0};
  uint8_t width_bytes{0};
  uint32_t value{0};
};

struct PdoMappingPlan
{
  static constexpr std::size_t kMaxWrites = 32;
  std::array<PdoSdoWrite, kMaxWrites> writes{};
  std::size_t count{0};
  bool requires_pre_operational{true};
};

bool validate_pdo_mapping(
  const PdoMappingEntry * entries, std::size_t count, std::size_t expected_bytes,
  std::string & error) noexcept;

bool build_lift_csv_pdo_mapping_plan(PdoMappingPlan & plan, std::string & error) noexcept;

// Applies the plan through the backend's SDO boundary. This is a configure/
// activate operation and must never be called from the 100 Hz read/write path.
bool configure_lift_csv_pdos(LiftEthercatBackend & backend, std::string & error);

bool encode_pdo_sdo_value(
  const PdoSdoWrite & write, std::vector<uint8_t> & value) noexcept;

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__PDO_MAPPING_HPP_

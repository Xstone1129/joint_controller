#include "joint_hardware/lift/pdo_mapping.hpp"

#include <limits>

namespace joint_hardware::lift
{

bool validate_pdo_mapping(
  const PdoMappingEntry * entries, std::size_t count, std::size_t expected_bytes,
  std::string & error) noexcept
{
  if (entries == nullptr || count == 0 || expected_bytes == 0) {
    error = "PDO mapping must contain at least one entry";
    return false;
  }
  std::size_t total_bits = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto & entry = entries[i];
    if (entry.index == 0 || entry.bit_length == 0 || entry.bit_length % 8U != 0U) {
      error = "PDO mapping entries must use non-zero byte-aligned objects";
      return false;
    }
    if (total_bits > std::numeric_limits<std::size_t>::max() - entry.bit_length) {
      error = "PDO mapping bit length overflow";
      return false;
    }
    total_bits += entry.bit_length;
  }
  if (total_bits != expected_bytes * 8U) {
    error = "PDO mapping length does not match the configured process-data size";
    return false;
  }
  return true;
}

bool build_lift_csv_pdo_mapping_plan(PdoMappingPlan & plan, std::string & error) noexcept
{
  if (!validate_pdo_mapping(kLiftRxPdoMapping.data(), kLiftRxPdoMapping.size(), 8, error) ||
    !validate_pdo_mapping(kLiftTxPdoMapping.data(), kLiftTxPdoMapping.size(), 17, error))
  {
    return false;
  }
  plan = PdoMappingPlan{};
  const auto append = [&plan](uint16_t index, uint8_t subindex, uint8_t width, uint32_t value) {
      if (plan.count >= plan.writes.size()) {
        return false;
      }
      plan.writes[plan.count++] = PdoSdoWrite{index, subindex, width, value};
      return true;
    };

  // The sequence follows LD3M manual section 8.4.4: unlink assignments,
  // invalidate mapping counts, write entries, then assign and enable PDOs.
  if (!append(0x1C12, 0x00, 1, 0) || !append(0x1C13, 0x00, 1, 0) ||
    !append(0x1600, 0x00, 1, 0) || !append(0x1601, 0x00, 1, 0) ||
    !append(0x1602, 0x00, 1, 0) || !append(0x1603, 0x00, 1, 0) ||
    !append(0x1A00, 0x00, 1, 0) || !append(0x1A01, 0x00, 1, 0))
  {
    error = "lift PDO mapping plan exceeds its fixed write capacity";
    return false;
  }
  for (std::size_t i = 0; i < kLiftRxPdoMapping.size(); ++i) {
    if (!append(
        0x1601, static_cast<uint8_t>(i + 1U), 4,
        pdo_mapping_value(kLiftRxPdoMapping[i])))
    {
      error = "lift PDO mapping plan exceeds its fixed write capacity";
      return false;
    }
  }
  if (!append(0x1601, 0x00, 1, static_cast<uint32_t>(kLiftRxPdoMapping.size()))) {
    error = "lift PDO mapping plan exceeds its fixed write capacity";
    return false;
  }
  for (std::size_t i = 0; i < kLiftTxPdoMapping.size(); ++i) {
    if (!append(
        0x1A00, static_cast<uint8_t>(i + 1U), 4,
        pdo_mapping_value(kLiftTxPdoMapping[i])))
    {
      error = "lift PDO mapping plan exceeds its fixed write capacity";
      return false;
    }
  }
  if (!append(0x1A00, 0x00, 1, static_cast<uint32_t>(kLiftTxPdoMapping.size())) ||
    !append(0x1C12, 0x01, 2, 0x1601) || !append(0x1C13, 0x01, 2, 0x1A00) ||
    !append(0x1C12, 0x00, 1, 1) || !append(0x1C13, 0x00, 1, 1))
  {
    error = "lift PDO mapping plan exceeds its fixed write capacity";
    return false;
  }
  return true;
}

bool encode_pdo_sdo_value(
  const PdoSdoWrite & write, std::vector<uint8_t> & value) noexcept
{
  if (write.width_bytes != 1 && write.width_bytes != 2 && write.width_bytes != 4) {
    return false;
  }
  if (write.width_bytes < 4 && (write.value >> (write.width_bytes * 8U)) != 0U) {
    return false;
  }
  value.resize(write.width_bytes);
  for (std::size_t i = 0; i < write.width_bytes; ++i) {
    value[i] = static_cast<uint8_t>((write.value >> (8U * i)) & 0xFFU);
  }
  return true;
}

bool configure_lift_csv_pdos(LiftEthercatBackend & backend, std::string & error)
{
  PdoMappingPlan plan;
  if (!build_lift_csv_pdo_mapping_plan(plan, error)) {
    return false;
  }
  for (std::size_t i = 0; i < plan.count; ++i) {
    std::vector<uint8_t> value;
    if (!encode_pdo_sdo_value(plan.writes[i], value) ||
      !backend.write_sdo(plan.writes[i].index, plan.writes[i].subindex, value))
    {
      error = "failed to write PDO mapping SDO 0x";
      static constexpr char hex[] = "0123456789ABCDEF";
      for (int shift = 12; shift >= 0; shift -= 4) {
        error.push_back(hex[(plan.writes[i].index >> shift) & 0x0FU]);
      }
      error += ":";
      error.push_back(hex[(plan.writes[i].subindex >> 4U) & 0x0FU]);
      error.push_back(hex[plan.writes[i].subindex & 0x0FU]);
      error += ": ";
      error += backend.error_message();
      return false;
    }
  }
  return true;
}

}  // namespace joint_hardware::lift

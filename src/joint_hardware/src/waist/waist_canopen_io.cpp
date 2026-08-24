#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace joint_hardware
{

bool WaistHardware::wait_sdo_response(
  uint16_t idx,
  uint8_t sub,
  std::array<uint8_t, 8> & out_data,
  int timeout_ms,
  int expected_cmd)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame frame;
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    const int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    const int wait_slice_ms = std::max(1, std::min(remaining_ms, 2));
    if (!recv_frame(frame, wait_slice_ms)) {
      continue;
    }
    (void)try_update_waist_feedback(frame.can_id, frame.data, frame.can_dlc);
    if (!waist_can::is_sdo_response(frame, waist_node_id_, idx, sub, expected_cmd)) {
      continue;
    }

    std::memcpy(out_data.data(), frame.data, 8);
    return true;
  }

  return false;
}

bool WaistHardware::sdo_write_i8(uint16_t idx, uint8_t sub, int8_t value, int timeout_ms)
{
  const int timeout = timeout_ms > 0 ? timeout_ms : waist_timeout_ms_;
  const auto req = waist_can::build_sdo_write_i8(idx, sub, value);
  if (!send_frame(waist_can::sdo_tx_id(waist_node_id_), req)) {
    return false;
  }
  std::array<uint8_t, 8> resp{};
  return wait_sdo_response(idx, sub, resp, timeout, waist_can::kSdoWriteAck) &&
         resp[0] == waist_can::kSdoWriteAck;
}

bool WaistHardware::sdo_write_u16(uint16_t idx, uint8_t sub, uint16_t value, int timeout_ms)
{
  const int timeout = timeout_ms > 0 ? timeout_ms : waist_timeout_ms_;
  const auto req = waist_can::build_sdo_write_u16(idx, sub, value);

  if (!send_frame(waist_can::sdo_tx_id(waist_node_id_), req)) {
    return false;
  }

  std::array<uint8_t, 8> resp{};
  if (!wait_sdo_response(idx, sub, resp, timeout, waist_can::kSdoWriteAck)) {
    return false;
  }

  return resp[0] == waist_can::kSdoWriteAck;
}

bool WaistHardware::sdo_write_i32(uint16_t idx, uint8_t sub, int32_t value, int timeout_ms)
{
  const int timeout = timeout_ms > 0 ? timeout_ms : waist_timeout_ms_;
  const auto req = waist_can::build_sdo_write_i32(idx, sub, value);
  if (!send_frame(waist_can::sdo_tx_id(waist_node_id_), req)) {
    return false;
  }
  std::array<uint8_t, 8> resp{};
  return wait_sdo_response(idx, sub, resp, timeout, waist_can::kSdoWriteAck) &&
         resp[0] == waist_can::kSdoWriteAck;
}

bool WaistHardware::sdo_write_u32(uint16_t idx, uint8_t sub, uint32_t value, int timeout_ms)
{
  const int timeout = timeout_ms > 0 ? timeout_ms : waist_timeout_ms_;
  const auto req = waist_can::build_sdo_write_u32(idx, sub, value);

  if (!send_frame(waist_can::sdo_tx_id(waist_node_id_), req)) {
    return false;
  }

  std::array<uint8_t, 8> resp{};
  if (!wait_sdo_response(idx, sub, resp, timeout, waist_can::kSdoWriteAck)) {
    return false;
  }

  return resp[0] == waist_can::kSdoWriteAck;
}

bool WaistHardware::sdo_read_i32(uint16_t idx, uint8_t sub, int32_t & value, int timeout_ms)
{
  const int timeout = timeout_ms > 0 ? timeout_ms : waist_timeout_ms_;
  const auto req = waist_can::build_sdo_read(idx, sub);

  if (!send_frame(waist_can::sdo_tx_id(waist_node_id_), req)) {
    return false;
  }

  std::array<uint8_t, 8> resp{};
  if (!wait_sdo_response(idx, sub, resp, timeout, waist_can::kSdoReadI32Response)) {
    return false;
  }

  value = waist_can::sdo_i32_value(resp);
  return true;
}

}  // namespace joint_hardware

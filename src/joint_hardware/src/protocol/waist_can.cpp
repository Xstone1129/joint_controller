#include "joint_hardware/protocol/waist_can.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace joint_hardware::waist_can
{
namespace
{

std::array<uint8_t, 2> u16_to_le(uint16_t value)
{
  return {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8U)};
}

std::array<uint8_t, 4> u32_to_le(uint32_t value)
{
  return {
    static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8U),
    static_cast<uint8_t>(value >> 16U), static_cast<uint8_t>(value >> 24U)};
}

int16_t be_to_i16(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}

}  // namespace

uint32_t sdo_tx_id(int node_id) {return kSdoTxBaseId | static_cast<uint32_t>(node_id & 0x7F);}
uint32_t sdo_rx_id(int node_id) {return kSdoRxBaseId | static_cast<uint32_t>(node_id & 0x7F);}
uint32_t frame_command_id(int node_id)
{
  return kFrameCommandBaseId | static_cast<uint32_t>(node_id & 0x7F);
}
uint32_t feedback_id(int node_id)
{
  return kFeedbackBaseId | static_cast<uint32_t>(node_id & 0x7F);
}

Payload build_sdo_write_i8(uint16_t index, uint8_t subindex, int8_t value)
{
  return {
    kSdoWriteI8Command, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U),
    subindex, static_cast<uint8_t>(value), 0, 0, 0};
}

Payload build_sdo_write_u16(uint16_t index, uint8_t subindex, uint16_t value)
{
  const auto bytes = u16_to_le(value);
  return {
    kSdoWriteU16Command, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U),
    subindex, bytes[0], bytes[1], 0, 0};
}

Payload build_sdo_write_i32(uint16_t index, uint8_t subindex, int32_t value)
{
  return build_sdo_write_u32(index, subindex, static_cast<uint32_t>(value));
}

Payload build_sdo_write_u32(uint16_t index, uint8_t subindex, uint32_t value)
{
  const auto bytes = u32_to_le(value);
  return {
    kSdoWriteU32Command, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U),
    subindex, bytes[0], bytes[1], bytes[2], bytes[3]};
}

Payload build_sdo_read(uint16_t index, uint8_t subindex)
{
  return {
    kSdoReadCommand, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U),
    subindex, 0, 0, 0, 0};
}

bool is_sdo_response(
  const can_frame & frame, int node_id, uint16_t index, uint8_t subindex,
  int expected_command)
{
  if ((frame.can_id & CAN_EFF_FLAG) != 0U ||
    (frame.can_id & CAN_SFF_MASK) != sdo_rx_id(node_id) || frame.can_dlc < 8U)
  {
    return false;
  }
  const uint16_t received_index = static_cast<uint16_t>(frame.data[1]) |
    static_cast<uint16_t>(static_cast<uint16_t>(frame.data[2]) << 8U);
  return received_index == index && frame.data[3] == subindex &&
         (expected_command < 0 || frame.data[0] == static_cast<uint8_t>(expected_command));
}

int32_t sdo_i32_value(const Payload & response)
{
  const uint32_t value = static_cast<uint32_t>(response[4]) |
    (static_cast<uint32_t>(response[5]) << 8U) |
    (static_cast<uint32_t>(response[6]) << 16U) |
    (static_cast<uint32_t>(response[7]) << 24U);
  return static_cast<int32_t>(value);
}

uint16_t sdo_u16_value(const Payload & response)
{
  return static_cast<uint16_t>(response[4]) |
         static_cast<uint16_t>(static_cast<uint16_t>(response[5]) << 8U);
}

bool is_operation_enabled(uint16_t statusword)
{
  return (statusword & kStatusMaskState) == kStatusOperationEnabled;
}

bool is_status_error(uint16_t statusword)
{
  return (statusword & kStatusMaskState) == kStatusFault;
}

std::string describe_error_code(uint16_t error_code)
{
  if (error_code == 0U) {
    return "no error";
  }
  struct ErrorBit {uint16_t mask; const char * text;};
  constexpr ErrorBit bits[] = {
    {0x0001U, "overvoltage"}, {0x0002U, "undervoltage"},
    {0x0004U, "overtemperature"}, {0x0008U, "stall"},
    {0x0010U, "overload"}, {0x0020U, "overspeed/current-sampling"},
    {0x0040U, "positive-limit"}, {0x0080U, "negative-limit"},
    {0x0100U, "encoder-read/timeout"}, {0x0200U, "maximum-speed"},
    {0x0400U, "electrical-angle-init"}, {0x1000U, "position-error"},
    {0x2000U, "encoder-fault"}};
  constexpr uint16_t known_mask = 0x37FFU;
  std::string result;
  for (const auto & bit : bits) {
    if ((error_code & bit.mask) != 0U) {
      if (!result.empty()) {result += '|';}
      result += bit.text;
    }
  }
  const uint16_t unknown = static_cast<uint16_t>(error_code & ~known_mask);
  if (unknown != 0U) {
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "unknown-0x%04X", unknown);
    if (!result.empty()) {result += '|';}
    result += buffer;
  }
  return result;
}

bool parse_feedback(
  uint32_t can_id, const uint8_t * data, uint8_t dlc, int node_id, Feedback & feedback)
{
  if ((can_id & CAN_EFF_FLAG) != 0U || (can_id & CAN_SFF_MASK) != feedback_id(node_id) ||
    data == nullptr || dlc < 12U)
  {
    return false;
  }
  feedback.position_raw = be_to_i16(data[0], data[1]);
  feedback.velocity_rpm = be_to_i16(data[2], data[3]);
  feedback.current_ma = be_to_i16(data[4], data[5]);
  feedback.error_code = static_cast<uint16_t>(
    (static_cast<uint16_t>(data[6]) << 8U) | static_cast<uint16_t>(data[7]));
  feedback.temperature_tenth_c = be_to_i16(data[8], data[9]);
  feedback.mode = data[10];
  feedback.state = data[11];
  return true;
}

FrameCommand build_position_frame_command(
  int node_id, int32_t target_raw, int32_t acceleration, int32_t velocity)
{
  const auto target = static_cast<uint16_t>(
    static_cast<int16_t>(std::clamp(target_raw, -32768, 32767)));
  const auto accel = static_cast<uint16_t>(
    static_cast<int16_t>(std::clamp(acceleration, -32768, 32767)));
  const auto speed = static_cast<uint16_t>(
    static_cast<int16_t>(std::clamp(velocity, -32768, 32767)));
  FrameCommand command;
  command.can_id = frame_command_id(node_id);
  command.data = {
    static_cast<uint8_t>(kFrameControlEnable | kFrameControlBrakeRelease |
    (kFrameModeProfilePosition << 1U)),
    static_cast<uint8_t>(target >> 8U), static_cast<uint8_t>(target),
    static_cast<uint8_t>(accel >> 8U), static_cast<uint8_t>(accel),
    static_cast<uint8_t>(speed >> 8U), static_cast<uint8_t>(speed), 0};
  command.dlc = 7;
  return command;
}

FrameCommand build_clear_error_frame_command(int node_id, bool clear_error)
{
  FrameCommand command;
  command.can_id = frame_command_id(node_id);
  command.data = {
    static_cast<uint8_t>(clear_error ? kFrameControlClearError : 0U), 0, 0, 0, 0, 0, 0, 0};
  command.dlc = 7;
  return command;
}

int deg_to_raw(double deg)
{
  const double bounded = std::clamp(deg, -kPositionDegMax, kPositionDegMax);
  return static_cast<int>(std::llround(bounded / kPositionDegMax * kPositionRawMax));
}

double raw_to_rad(int32_t raw)
{
  constexpr double pi = 3.14159265358979323846;
  return static_cast<double>(raw) / kPositionRawMax * pi;
}

}  // namespace joint_hardware::waist_can

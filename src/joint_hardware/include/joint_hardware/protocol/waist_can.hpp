#ifndef JOINT_HARDWARE__PROTOCOL__WAIST_CAN_HPP_
#define JOINT_HARDWARE__PROTOCOL__WAIST_CAN_HPP_

#include <array>
#include <cstdint>
#include <string>

#include <linux/can.h>

namespace joint_hardware::waist_can
{

using Payload = std::array<uint8_t, 8>;

constexpr uint32_t kSdoTxBaseId = 0x600;
constexpr uint32_t kSdoRxBaseId = 0x580;
constexpr uint32_t kFrameCommandBaseId = 0x100;
constexpr uint32_t kFeedbackBaseId = 0x300;
constexpr uint32_t kSyncId = 0x080;

constexpr uint16_t kIndexControlword = 0x6040;
constexpr uint16_t kIndexStatusword = 0x6041;
constexpr uint16_t kIndexErrorCode = 0x603F;
constexpr uint16_t kIndexMode = 0x6060;
constexpr uint16_t kIndexTargetPosition = 0x607A;
constexpr uint16_t kIndexActualPosition = 0x6064;
constexpr uint16_t kIndexSetZeroPosition = 0x2531;
constexpr uint16_t kIndexTargetVelocity = 0x60FF;
constexpr uint16_t kIndexProfileAcceleration = 0x6083;
constexpr uint16_t kIndexProfileDeceleration = 0x6084;
constexpr uint16_t kIndexProfileVelocity = 0x6081;

constexpr int8_t kModeProfilePosition = 1;
constexpr int8_t kModeProfileVelocity = 3;

constexpr uint16_t kControlwordDisableVoltage = 0x0000;
constexpr uint16_t kControlwordShutdown = 0x0006;
constexpr uint16_t kControlwordSwitchOn = 0x0007;
constexpr uint16_t kControlwordEnableOperation = 0x000F;
constexpr uint16_t kControlwordStartProfilePosition = 0x004F;

constexpr uint16_t kStatusMaskState = 0x006F;
constexpr uint16_t kStatusReadyToSwitchOn = 0x0021;
constexpr uint16_t kStatusSwitchedOn = 0x0023;
constexpr uint16_t kStatusOperationEnabled = 0x0027;
constexpr uint16_t kStatusFault = 0x0008;

constexpr uint8_t kFeedbackStateEnabled = 0x80;
constexpr uint8_t kFeedbackStateBrakeReleased = 0x40;
constexpr uint8_t kFeedbackStateError = 0x20;
constexpr uint8_t kFeedbackStateTargetReached = 0x10;

constexpr uint8_t kFrameControlEnable = 0x80;
constexpr uint8_t kFrameControlBrakeRelease = 0x40;
constexpr uint8_t kFrameControlClearError = 0x20;
constexpr uint8_t kFrameModeProfilePosition = 0x01;

constexpr uint8_t kSdoReadCommand = 0x40;
constexpr uint8_t kSdoWriteI8Command = 0x2F;
constexpr uint8_t kSdoWriteU16Command = 0x2B;
constexpr uint8_t kSdoWriteU32Command = 0x23;
constexpr uint8_t kSdoWriteAck = 0x60;
constexpr uint8_t kSdoReadI32Response = 0x43;
constexpr uint8_t kSdoReadU16Response = 0x4B;

constexpr int kPositionRawMax = 32768;
constexpr double kPositionDegMax = 180.0;

struct Feedback
{
  int32_t position_raw{0};
  int16_t velocity_rpm{0};
  int16_t current_ma{0};
  uint16_t error_code{0};
  int16_t temperature_tenth_c{0};
  uint8_t mode{0};
  uint8_t state{0};
};

struct FrameCommand
{
  uint32_t can_id{0};
  Payload data{};
  uint8_t dlc{0};
};

uint32_t sdo_tx_id(int node_id);
uint32_t sdo_rx_id(int node_id);
uint32_t frame_command_id(int node_id);
uint32_t feedback_id(int node_id);

Payload build_sdo_write_i8(uint16_t index, uint8_t subindex, int8_t value);
Payload build_sdo_write_u16(uint16_t index, uint8_t subindex, uint16_t value);
Payload build_sdo_write_i32(uint16_t index, uint8_t subindex, int32_t value);
Payload build_sdo_write_u32(uint16_t index, uint8_t subindex, uint32_t value);
Payload build_sdo_read(uint16_t index, uint8_t subindex);

bool is_sdo_response(
  const can_frame & frame, int node_id, uint16_t index, uint8_t subindex,
  int expected_command = -1);
int32_t sdo_i32_value(const Payload & response);
uint16_t sdo_u16_value(const Payload & response);
bool is_operation_enabled(uint16_t statusword);
bool is_status_error(uint16_t statusword);
std::string describe_error_code(uint16_t error_code);

bool parse_feedback(
  uint32_t can_id, const uint8_t * data, uint8_t dlc, int node_id, Feedback & feedback);
FrameCommand build_position_frame_command(
  int node_id, int32_t target_raw, int32_t acceleration, int32_t velocity);
FrameCommand build_clear_error_frame_command(int node_id, bool clear_error);

int deg_to_raw(double deg);
double raw_to_rad(int32_t raw);

}  // namespace joint_hardware::waist_can

#endif  // JOINT_HARDWARE__PROTOCOL__WAIST_CAN_HPP_

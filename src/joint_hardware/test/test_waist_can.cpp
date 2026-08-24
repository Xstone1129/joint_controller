#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "joint_hardware/protocol/waist_can.hpp"

namespace wc = joint_hardware::waist_can;

TEST(WaistCan, BuildsLittleEndianSdoFrames)
{
  EXPECT_EQ(
    wc::build_sdo_write_u16(0x6040, 0, 0x1234),
    (wc::Payload{0x2B, 0x40, 0x60, 0x00, 0x34, 0x12, 0x00, 0x00}));
  EXPECT_EQ(
    wc::build_sdo_write_i32(0x60FF, 0, -2),
    (wc::Payload{0x23, 0xFF, 0x60, 0x00, 0xFE, 0xFF, 0xFF, 0xFF}));
  EXPECT_EQ(
    wc::build_sdo_read(0x6064, 0),
    (wc::Payload{0x40, 0x64, 0x60, 0x00, 0, 0, 0, 0}));
}

TEST(WaistCan, ValidatesSdoResponseIdentity)
{
  can_frame frame{};
  frame.can_id = wc::sdo_rx_id(15);
  frame.can_dlc = 8;
  const std::array<uint8_t, 8> response{0x60, 0x40, 0x60, 0x00, 0, 0, 0, 0};
  std::memcpy(frame.data, response.data(), response.size());
  EXPECT_TRUE(wc::is_sdo_response(frame, 15, 0x6040, 0, wc::kSdoWriteAck));
  EXPECT_FALSE(wc::is_sdo_response(frame, 14, 0x6040, 0, wc::kSdoWriteAck));
  EXPECT_FALSE(wc::is_sdo_response(frame, 15, 0x6060, 0, wc::kSdoWriteAck));
}

TEST(WaistCan, ParsesVendorFeedbackAsBigEndian)
{
  const std::array<uint8_t, 12> data{
    0x40, 0x00, 0xFF, 0x9C, 0x01, 0xF4, 0x10, 0x00, 0x01, 0x2C, 0x01, 0xD0};
  wc::Feedback feedback;
  ASSERT_TRUE(wc::parse_feedback(wc::feedback_id(15), data.data(), data.size(), 15, feedback));
  EXPECT_EQ(feedback.position_raw, 16384);
  EXPECT_EQ(feedback.velocity_rpm, -100);
  EXPECT_EQ(feedback.current_ma, 500);
  EXPECT_EQ(feedback.error_code, 0x1000);
  EXPECT_EQ(feedback.temperature_tenth_c, 300);
  EXPECT_EQ(feedback.mode, 1);
  EXPECT_EQ(feedback.state, 0xD0);
  EXPECT_FALSE(wc::parse_feedback(wc::feedback_id(14), data.data(), data.size(), 15, feedback));
}

TEST(WaistCan, ConvertsPositionAndClampsCommand)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_EQ(wc::deg_to_raw(90.0), 16384);
  EXPECT_EQ(wc::deg_to_raw(200.0), 32768);
  EXPECT_NEAR(wc::raw_to_rad(16384), pi / 2.0, 1e-12);

  const auto command = wc::build_position_frame_command(15, 50000, 2000, 10);
  EXPECT_EQ(command.can_id, 0x10FU);
  EXPECT_EQ(command.dlc, 7);
  EXPECT_EQ(command.data[0], 0xC2);
  EXPECT_EQ(command.data[1], 0x7F);
  EXPECT_EQ(command.data[2], 0xFF);
}

TEST(WaistCan, DecodesCia402StateAndManufacturerErrors)
{
  EXPECT_TRUE(wc::is_operation_enabled(0x0027));
  EXPECT_TRUE(wc::is_operation_enabled(0x1227));
  EXPECT_TRUE(wc::is_status_error(0x0008));
  EXPECT_EQ(wc::describe_error_code(0), "no error");
  EXPECT_NE(wc::describe_error_code(0x1001).find("overvoltage"), std::string::npos);
  EXPECT_NE(wc::describe_error_code(0x1001).find("position-error"), std::string::npos);
}

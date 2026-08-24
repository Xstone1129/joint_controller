#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "joint_hardware/lift/ethercat_coe.hpp"
#include "joint_hardware/lift/ethercat_frame.hpp"
#include "joint_hardware/lift/pdo_mapping.hpp"

namespace
{

TEST(EthercatFrame, EncodesAndDecodesLittleEndianDatagram)
{
  joint_hardware::lift::EthercatEthernetHeader ethernet_header;
  ethernet_header.destination = {0, 1, 2, 3, 4, 5};
  ethernet_header.source = {6, 7, 8, 9, 10, 11};
  std::array<uint8_t, joint_hardware::lift::kEthernetHeaderSize> ethernet_bytes{};
  ASSERT_TRUE(
    joint_hardware::lift::encode_ethercat_ethernet_header(
      ethernet_header, ethernet_bytes));
  EXPECT_EQ(ethernet_bytes[12], 0x88U);
  EXPECT_EQ(ethernet_bytes[13], 0xA4U);

  joint_hardware::lift::LiftRxPdo pdo;
  pdo.control_word = 0x000F;
  pdo.target_velocity_units_per_s = -123456;
  pdo.torque_feedforward = -321;
  std::vector<uint8_t> bytes;
  ASSERT_TRUE(
    joint_hardware::lift::encode_lift_rx_pdo_frame(
      pdo, 0x2A, 0x11223344U, 0x0042, bytes));
  ASSERT_EQ(bytes.size(), 22U);
  EXPECT_EQ(bytes[0], 0x14);
  EXPECT_EQ(bytes[1], 0x10);  // 11-bit payload length + EtherCAT frame type 1.
  EXPECT_EQ(bytes[2], static_cast<uint8_t>(joint_hardware::lift::EthercatCommand::lwr));
  EXPECT_EQ(bytes[3], 0x2A);
  EXPECT_EQ(bytes[4], 0x44);
  EXPECT_EQ(bytes[5], 0x33);
  EXPECT_EQ(bytes[6], 0x22);
  EXPECT_EQ(bytes[7], 0x11);
  EXPECT_EQ(bytes[8], 8U);
  EXPECT_EQ(bytes[9], 0U);
  EXPECT_EQ(bytes[20], 0x42U);
  EXPECT_EQ(bytes[21], 0U);

  joint_hardware::lift::EthercatFrame decoded;
  ASSERT_TRUE(joint_hardware::lift::decode_ethercat_frame(bytes.data(), bytes.size(), decoded));
  ASSERT_EQ(decoded.datagrams.size(), 1U);
  EXPECT_FALSE(decoded.datagrams.front().header.more);
  EXPECT_EQ(decoded.datagrams.front().header.address, 0x11223344U);
  EXPECT_EQ(decoded.datagrams.front().working_counter, 0x0042U);
  ASSERT_EQ(decoded.datagrams.front().data.size(), sizeof(joint_hardware::lift::LiftRxPdo));
  joint_hardware::lift::LiftRxPdo decoded_pdo;
  joint_hardware::lift::LiftRxPdoWire wire{};
  std::copy(
    decoded.datagrams.front().data.begin(), decoded.datagrams.front().data.end(),
    wire.begin());
  ASSERT_TRUE(joint_hardware::lift::decode_lift_rx_pdo_le(wire, decoded_pdo));
  EXPECT_EQ(decoded_pdo.control_word, pdo.control_word);
  EXPECT_EQ(decoded_pdo.target_velocity_units_per_s, pdo.target_velocity_units_per_s);
  EXPECT_EQ(decoded_pdo.torque_feedforward, pdo.torque_feedforward);
}

TEST(EthercatFrame, DecodesLiftFeedbackAndRejectsWrongMapping)
{
  joint_hardware::lift::LiftTxPdo pdo;
  pdo.status_word = 0x0027;
  pdo.error_code = 0x2310;
  pdo.mode_display = 9;
  pdo.actual_position_units = -1000;
  pdo.actual_velocity_units_per_s = 2000;
  pdo.digital_inputs = 0x80000001U;
  joint_hardware::lift::LiftTxPdoWire wire{};
  ASSERT_TRUE(joint_hardware::lift::encode_lift_tx_pdo_le(pdo, wire));

  joint_hardware::lift::EthercatDatagram datagram;
  datagram.header.command = joint_hardware::lift::EthercatCommand::lrd;
  datagram.header.index = 3;
  datagram.header.address = 0x100U;
  datagram.header.data_length = static_cast<uint16_t>(wire.size());
  datagram.data.assign(wire.begin(), wire.end());
  datagram.working_counter = 1;
  joint_hardware::lift::EthercatFrame frame;
  frame.datagrams.push_back(datagram);
  std::vector<uint8_t> bytes;
  ASSERT_TRUE(joint_hardware::lift::encode_ethercat_frame(frame, bytes));

  joint_hardware::lift::LiftTxPdo decoded;
  uint16_t working_counter = 0;
  std::string error;
  ASSERT_TRUE(
    joint_hardware::lift::decode_lift_tx_pdo_frame(
      bytes.data(), bytes.size(), decoded, working_counter, error)) << error;
  EXPECT_EQ(decoded.actual_position_units, pdo.actual_position_units);
  EXPECT_EQ(decoded.digital_inputs, pdo.digital_inputs);
  EXPECT_EQ(working_counter, 1U);

  datagram.header.data_length = 16;
  datagram.data.resize(16);
  frame.datagrams.front() = datagram;
  ASSERT_TRUE(joint_hardware::lift::encode_ethercat_frame(frame, bytes));
  EXPECT_FALSE(
    joint_hardware::lift::decode_lift_tx_pdo_frame(
      bytes.data(), bytes.size(), decoded, working_counter, error));
}

TEST(EthercatCoe, ExpeditedSdoRoundTrip)
{
  std::vector<uint8_t> request;
  ASSERT_TRUE(joint_hardware::lift::encode_coe_sdo_upload_request(0x6060, 0, request));
  ASSERT_EQ(request.size(), 12U);
  EXPECT_EQ(request[0], 6U);  // CoE header + SDO upload request.
  EXPECT_EQ(request[5], joint_hardware::lift::kEthercatMailboxTypeCoe);
  EXPECT_EQ(request[6], 0U);
  EXPECT_EQ(request[7], 0x20U);
  EXPECT_EQ(request[8], 0x40U);
  EXPECT_EQ(request[9], 0x60U);
  EXPECT_EQ(request[10], 0x60U);

  std::vector<uint8_t> response_payload{
    0x00, static_cast<uint8_t>(joint_hardware::lift::kCoeServiceSdoResponse << 4U),
    0x43, 0x60, 0x60, 0x00, 0x09, 0x00, 0x00, 0x00};
  joint_hardware::lift::EthercatMailbox mailbox;
  mailbox.payload = response_payload;
  std::vector<uint8_t> response_bytes;
  ASSERT_TRUE(joint_hardware::lift::encode_ethercat_mailbox(mailbox, response_bytes));
  joint_hardware::lift::CoeSdoResponse response;
  std::string error;
  ASSERT_TRUE(
    joint_hardware::lift::decode_coe_sdo_response(
      response_bytes.data(), response_bytes.size(), response, error)) << error;
  EXPECT_TRUE(response.upload);
  EXPECT_EQ(response.index, 0x6060U);
  ASSERT_EQ(response.data.size(), 4U);
  EXPECT_EQ(response.data[0], 9U);

  ASSERT_TRUE(
    joint_hardware::lift::encode_coe_sdo_download_request(
      0x2008, 0, {0x10, 0x27, 0x00, 0x00}, request));
  EXPECT_EQ(request[8], 0x23U);
  EXPECT_EQ(request[12], 0x10U);
  EXPECT_EQ(request[13], 0x27U);
}

TEST(LiftPdoMapping, BuildsManualDynamicMappingSequence)
{
  joint_hardware::lift::PdoMappingPlan plan;
  std::string error;
  ASSERT_TRUE(joint_hardware::lift::build_lift_csv_pdo_mapping_plan(plan, error)) << error;
  EXPECT_EQ(plan.count, 23U);
  EXPECT_EQ(plan.writes[0].index, 0x1C12U);
  EXPECT_EQ(plan.writes[0].subindex, 0U);
  EXPECT_EQ(plan.writes[0].value, 0U);
  EXPECT_EQ(plan.writes[8].index, 0x1601U);
  EXPECT_EQ(plan.writes[8].value, 0x60400010U);
  EXPECT_EQ(plan.writes[10].value, 0x60B20010U);
  EXPECT_EQ(plan.writes[12].index, 0x1A00U);
  EXPECT_EQ(plan.writes[12].value, 0x60410010U);
  EXPECT_EQ(plan.writes[15].value, 0x60640020U);
  EXPECT_EQ(plan.writes[16].value, 0x606C0020U);
  EXPECT_EQ(plan.writes[22].index, 0x1C13U);
  EXPECT_EQ(plan.writes[22].value, 1U);
}

}  // namespace

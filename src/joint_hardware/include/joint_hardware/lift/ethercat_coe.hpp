#ifndef JOINT_HARDWARE__LIFT__ETHERCAT_COE_HPP_
#define JOINT_HARDWARE__LIFT__ETHERCAT_COE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace joint_hardware::lift
{

constexpr uint8_t kEthercatMailboxTypeCoe = 0x03;
constexpr uint8_t kCoeServiceSdoRequest = 0x02;
constexpr uint8_t kCoeServiceSdoResponse = 0x03;

struct EthercatMailbox
{
  uint16_t address{0};
  uint8_t channel{0};
  uint8_t priority{0};
  uint8_t type{kEthercatMailboxTypeCoe};
  std::vector<uint8_t> payload;
};

struct CoeSdoResponse
{
  uint16_t index{0};
  uint8_t subindex{0};
  bool upload{false};
  bool abort{false};
  uint32_t abort_code{0};
  std::vector<uint8_t> data;
};

bool encode_ethercat_mailbox(const EthercatMailbox & mailbox, std::vector<uint8_t> & bytes);
bool decode_ethercat_mailbox(
  const uint8_t * bytes, std::size_t size, EthercatMailbox & mailbox, std::string & error);

bool encode_coe_sdo_upload_request(
  uint16_t index, uint8_t subindex, std::vector<uint8_t> & mailbox_bytes);
bool encode_coe_sdo_download_request(
  uint16_t index, uint8_t subindex, const std::vector<uint8_t> & data,
  std::vector<uint8_t> & mailbox_bytes);
bool decode_coe_sdo_response(
  const uint8_t * mailbox_bytes, std::size_t mailbox_size,
  CoeSdoResponse & response, std::string & error);
bool decode_coe_sdo_download_ack(
  const uint8_t * mailbox_bytes, std::size_t mailbox_size,
  uint16_t expected_index, uint8_t expected_subindex, std::string & error);

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ETHERCAT_COE_HPP_

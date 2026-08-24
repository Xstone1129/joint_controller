#include "joint_hardware/lift/ethercat_coe.hpp"

#include <algorithm>

namespace joint_hardware::lift
{

namespace
{

void store_u16_le(uint8_t * destination, uint16_t value) noexcept
{
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

uint16_t load_u16_le(const uint8_t * source) noexcept
{
  return static_cast<uint16_t>(source[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(source[1]) << 8U);
}

uint32_t load_u32_le(const uint8_t * source) noexcept
{
  uint32_t value = 0;
  for (std::size_t i = 0; i < sizeof(uint32_t); ++i) {
    value |= static_cast<uint32_t>(source[i]) << (8U * i);
  }
  return value;
}

uint8_t expedited_download_command(std::size_t size) noexcept
{
  switch (size) {
    case 1:
      return 0x2F;
    case 2:
      return 0x2B;
    case 3:
      return 0x27;
    case 4:
      return 0x23;
    default:
      return 0;
  }
}

bool is_coe_mailbox(const EthercatMailbox & mailbox) noexcept
{
  return (mailbox.type & 0x0FU) == kEthercatMailboxTypeCoe;
}

bool decode_sdo_service(
  const EthercatMailbox & mailbox, uint8_t expected_service, std::string & error) noexcept
{
  if (!is_coe_mailbox(mailbox) || mailbox.payload.size() < 2) {
    error = "mailbox is not a complete CoE payload";
    return false;
  }
  const uint16_t coe_header = load_u16_le(mailbox.payload.data());
  const uint8_t service = static_cast<uint8_t>((coe_header >> 12U) & 0x0FU);
  if (service != expected_service) {
    error = "unexpected CoE service type";
    return false;
  }
  return true;
}

}  // namespace

bool encode_ethercat_mailbox(const EthercatMailbox & mailbox, std::vector<uint8_t> & bytes)
{
  if (mailbox.payload.size() > 0xFFFFU || mailbox.channel > 0x3FU || mailbox.priority > 0x03U ||
    (mailbox.type & 0xF0U) != 0)
  {
    return false;
  }
  bytes.resize(6U + mailbox.payload.size());
  store_u16_le(bytes.data(), static_cast<uint16_t>(mailbox.payload.size()));
  store_u16_le(bytes.data() + 2, mailbox.address);
  bytes[4] = static_cast<uint8_t>(mailbox.channel | (mailbox.priority << 6U));
  bytes[5] = mailbox.type;
  std::copy(mailbox.payload.begin(), mailbox.payload.end(), bytes.begin() + 6);
  return true;
}

bool decode_ethercat_mailbox(
  const uint8_t * bytes, std::size_t size, EthercatMailbox & mailbox, std::string & error)
{
  if (bytes == nullptr || size < 6) {
    error = "EtherCAT mailbox is shorter than its six-byte header";
    return false;
  }
  const std::size_t payload_size = load_u16_le(bytes);
  if (size != 6U + payload_size) {
    error = "EtherCAT mailbox length field does not match the received bytes";
    return false;
  }
  mailbox.address = load_u16_le(bytes + 2);
  mailbox.channel = bytes[4] & 0x3FU;
  mailbox.priority = static_cast<uint8_t>((bytes[4] >> 6U) & 0x03U);
  mailbox.type = bytes[5] & 0x0FU;
  mailbox.payload.assign(bytes + 6, bytes + size);
  return true;
}

bool encode_coe_sdo_upload_request(
  uint16_t index, uint8_t subindex, std::vector<uint8_t> & mailbox_bytes)
{
  EthercatMailbox mailbox;
  mailbox.payload = {0x00, static_cast<uint8_t>(kCoeServiceSdoRequest << 4U), 0x40,
    static_cast<uint8_t>(index & 0xFFU), static_cast<uint8_t>((index >> 8U) & 0xFFU), subindex};
  return encode_ethercat_mailbox(mailbox, mailbox_bytes);
}

bool encode_coe_sdo_download_request(
  uint16_t index, uint8_t subindex, const std::vector<uint8_t> & data,
  std::vector<uint8_t> & mailbox_bytes)
{
  const uint8_t command = expedited_download_command(data.size());
  if (command == 0) {
    return false;
  }
  EthercatMailbox mailbox;
  mailbox.payload = {0x00, static_cast<uint8_t>(kCoeServiceSdoRequest << 4U), command,
    static_cast<uint8_t>(index & 0xFFU), static_cast<uint8_t>((index >> 8U) & 0xFFU), subindex};
  mailbox.payload.insert(mailbox.payload.end(), data.begin(), data.end());
  mailbox.payload.resize(2U + 1U + 2U + 1U + 4U, 0);
  return encode_ethercat_mailbox(mailbox, mailbox_bytes);
}

bool decode_coe_sdo_response(
  const uint8_t * mailbox_bytes, std::size_t mailbox_size,
  CoeSdoResponse & response, std::string & error)
{
  EthercatMailbox mailbox;
  if (!decode_ethercat_mailbox(mailbox_bytes, mailbox_size, mailbox, error) ||
    !decode_sdo_service(mailbox, kCoeServiceSdoResponse, error) || mailbox.payload.size() < 6)
  {
    if (error.empty()) {
      error = "CoE SDO response is too short";
    }
    return false;
  }
  const uint8_t command = mailbox.payload[2];
  response = CoeSdoResponse{};
  response.index = load_u16_le(mailbox.payload.data() + 3);
  response.subindex = mailbox.payload[5];
  if (command == 0x80) {
    if (mailbox.payload.size() < 10) {
      error = "SDO abort response is missing its abort code";
      return false;
    }
    response.abort = true;
    response.abort_code = load_u32_le(mailbox.payload.data() + 6);
    return true;
  }
  if ((command & 0xE0U) != 0x40U || (command & 0x02U) == 0 ||
    (command & 0x01U) == 0)
  {
    error = "segmented or unsupported SDO upload response";
    return false;
  }
  const std::size_t unused = static_cast<std::size_t>((command >> 2U) & 0x03U);
  const std::size_t data_size = 4U - unused;
  if (mailbox.payload.size() != 6U + 4U) {
    error = "expedited SDO upload response must contain four data bytes";
    return false;
  }
  response.upload = true;
  response.data.assign(mailbox.payload.begin() + 6, mailbox.payload.begin() + 6 + data_size);
  return true;
}

bool decode_coe_sdo_download_ack(
  const uint8_t * mailbox_bytes, std::size_t mailbox_size,
  uint16_t expected_index, uint8_t expected_subindex, std::string & error)
{
  EthercatMailbox mailbox;
  if (!decode_ethercat_mailbox(mailbox_bytes, mailbox_size, mailbox, error) ||
    !decode_sdo_service(mailbox, kCoeServiceSdoResponse, error) || mailbox.payload.size() < 6)
  {
    return false;
  }
  if (mailbox.payload[2] == 0x80) {
    if (mailbox.payload.size() < 10) {
      error = "SDO abort response is missing its abort code";
    } else {
      error = "SDO download was aborted with code 0x";
      const uint32_t code = load_u32_le(mailbox.payload.data() + 6);
      static constexpr char hex[] = "0123456789ABCDEF";
      for (int shift = 28; shift >= 0; shift -= 4) {
        error.push_back(hex[(code >> shift) & 0x0FU]);
      }
    }
    return false;
  }
  if (mailbox.payload[2] != 0x60 || load_u16_le(mailbox.payload.data() + 3) != expected_index ||
    mailbox.payload[5] != expected_subindex)
  {
    error = "unexpected SDO download acknowledgement";
    return false;
  }
  return true;
}

}  // namespace joint_hardware::lift

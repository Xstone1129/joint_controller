#include "joint_hardware/lift/ethercat_frame.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace joint_hardware::lift
{

namespace
{

void store_u16_le(uint8_t * destination, uint16_t value) noexcept
{
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void store_u32_le(uint8_t * destination, uint32_t value) noexcept
{
  for (std::size_t i = 0; i < sizeof(uint32_t); ++i) {
    destination[i] = static_cast<uint8_t>((value >> (8U * i)) & 0xFFU);
  }
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

bool valid_command(uint8_t value) noexcept
{
  return value <= static_cast<uint8_t>(EthercatCommand::frmw);
}

}  // namespace

bool encode_ethercat_ethernet_header(
  const EthercatEthernetHeader & header,
  std::array<uint8_t, kEthernetHeaderSize> & bytes) noexcept
{
  if (header.ether_type != kEthercatEtherType) {
    return false;
  }
  std::copy(header.destination.begin(), header.destination.end(), bytes.begin());
  std::copy(header.source.begin(), header.source.end(), bytes.begin() + 6);
  // Ethernet EtherType is network byte order; EtherCAT payload fields below
  // use little-endian encoding.
  bytes[12] = static_cast<uint8_t>((header.ether_type >> 8U) & 0xFFU);
  bytes[13] = static_cast<uint8_t>(header.ether_type & 0xFFU);
  return true;
}

bool decode_ethercat_ethernet_header(
  const uint8_t * bytes, std::size_t size, EthercatEthernetHeader & header) noexcept
{
  if (bytes == nullptr || size < kEthernetHeaderSize) {
    return false;
  }
  std::copy(bytes, bytes + 6, header.destination.begin());
  std::copy(bytes + 6, bytes + 12, header.source.begin());
  header.ether_type = static_cast<uint16_t>(static_cast<uint16_t>(bytes[12]) << 8U) |
    static_cast<uint16_t>(bytes[13]);
  return header.ether_type == kEthercatEtherType;
}

bool encode_ethercat_frame_header(
  const EthercatFrameHeader & header,
  std::array<uint8_t, kEthercatFrameHeaderSize> & bytes) noexcept
{
  if (header.payload_length > kEthercatLengthMask || header.type > 0x0FU) {
    return false;
  }
  const uint16_t wire = static_cast<uint16_t>(header.payload_length) |
    static_cast<uint16_t>(static_cast<uint16_t>(header.type) << 12U);
  store_u16_le(bytes.data(), wire);
  return true;
}

bool decode_ethercat_frame_header(
  const uint8_t * bytes, std::size_t size, EthercatFrameHeader & header) noexcept
{
  if (bytes == nullptr || size < kEthercatFrameHeaderSize) {
    return false;
  }
  const uint16_t wire = load_u16_le(bytes);
  header.payload_length = wire & kEthercatLengthMask;
  header.type = static_cast<uint8_t>((wire >> 12U) & 0x0FU);
  return header.type == kEthercatFrameType;
}

bool encode_ethercat_datagram_header(
  const EthercatDatagramHeader & header,
  std::array<uint8_t, kEthercatDatagramHeaderSize> & bytes) noexcept
{
  if (header.data_length > kEthercatLengthMask) {
    return false;
  }
  bytes[0] = static_cast<uint8_t>(header.command);
  bytes[1] = header.index;
  store_u32_le(bytes.data() + 2, header.address);
  uint16_t length = header.data_length;
  if (header.more) {
    length = static_cast<uint16_t>(length | kEthercatDatagramMoreFlag);
  }
  store_u16_le(bytes.data() + 6, length);
  store_u16_le(bytes.data() + 8, header.interrupt);
  return true;
}

bool decode_ethercat_datagram_header(
  const uint8_t * bytes, std::size_t size, EthercatDatagramHeader & header) noexcept
{
  if (bytes == nullptr || size < kEthercatDatagramHeaderSize || !valid_command(bytes[0])) {
    return false;
  }
  header.command = static_cast<EthercatCommand>(bytes[0]);
  header.index = bytes[1];
  header.address = load_u32_le(bytes + 2);
  const uint16_t length = load_u16_le(bytes + 6);
  header.data_length = length & kEthercatLengthMask;
  header.more = (length & kEthercatDatagramMoreFlag) != 0;
  header.interrupt = load_u16_le(bytes + 8);
  return true;
}

bool encode_ethercat_datagram(const EthercatDatagram & datagram, std::vector<uint8_t> & bytes)
{
  if (datagram.data.size() > kEthercatLengthMask ||
    datagram.data.size() != datagram.header.data_length)
  {
    return false;
  }
  std::array<uint8_t, kEthercatDatagramHeaderSize> header_bytes{};
  if (!encode_ethercat_datagram_header(datagram.header, header_bytes)) {
    return false;
  }
  bytes.resize(kEthercatDatagramHeaderSize + datagram.data.size() + kEthercatWorkingCounterSize);
  std::copy(header_bytes.begin(), header_bytes.end(), bytes.begin());
  std::copy(
    datagram.data.begin(), datagram.data.end(),
    bytes.begin() + kEthercatDatagramHeaderSize);
  store_u16_le(
    bytes.data() + kEthercatDatagramHeaderSize + datagram.data.size(), datagram.working_counter);
  return true;
}

bool decode_ethercat_datagram(
  const uint8_t * bytes, std::size_t size, EthercatDatagram & datagram)
{
  if (bytes == nullptr || size < kEthercatDatagramHeaderSize + kEthercatWorkingCounterSize ||
    !decode_ethercat_datagram_header(bytes, size, datagram.header))
  {
    return false;
  }
  const std::size_t expected = kEthercatDatagramHeaderSize + datagram.header.data_length +
    kEthercatWorkingCounterSize;
  if (size != expected) {
    return false;
  }
  datagram.data.assign(
    bytes + kEthercatDatagramHeaderSize,
    bytes + kEthercatDatagramHeaderSize + datagram.header.data_length);
  datagram.working_counter = load_u16_le(
    bytes + kEthercatDatagramHeaderSize + datagram.header.data_length);
  return true;
}

bool encode_ethercat_frame(const EthercatFrame & frame, std::vector<uint8_t> & bytes)
{
  if (frame.datagrams.empty()) {
    return false;
  }
  std::size_t payload_size = 0;
  for (const auto & datagram : frame.datagrams) {
    if (datagram.data.size() > kEthercatLengthMask ||
      datagram.data.size() != datagram.header.data_length)
    {
      return false;
    }
    if (payload_size > std::numeric_limits<std::size_t>::max() -
      kEthercatDatagramHeaderSize - datagram.data.size() - kEthercatWorkingCounterSize)
    {
      return false;
    }
    payload_size += kEthercatDatagramHeaderSize + datagram.data.size() +
      kEthercatWorkingCounterSize;
  }
  if (payload_size > kEthercatLengthMask) {
    return false;
  }
  EthercatFrameHeader header;
  header.payload_length = static_cast<uint16_t>(payload_size);
  header.type = kEthercatFrameType;
  std::array<uint8_t, kEthercatFrameHeaderSize> header_bytes{};
  if (!encode_ethercat_frame_header(header, header_bytes)) {
    return false;
  }
  bytes.clear();
  bytes.reserve(kEthercatFrameHeaderSize + payload_size);
  bytes.insert(bytes.end(), header_bytes.begin(), header_bytes.end());
  for (std::size_t i = 0; i < frame.datagrams.size(); ++i) {
    EthercatDatagram datagram = frame.datagrams[i];
    datagram.header.more = i + 1U < frame.datagrams.size();
    std::vector<uint8_t> datagram_bytes;
    if (!encode_ethercat_datagram(datagram, datagram_bytes)) {
      return false;
    }
    bytes.insert(bytes.end(), datagram_bytes.begin(), datagram_bytes.end());
  }
  return true;
}

bool decode_ethercat_frame(const uint8_t * bytes, std::size_t size, EthercatFrame & frame)
{
  if (bytes == nullptr || size < kEthercatFrameHeaderSize) {
    return false;
  }
  EthercatFrameHeader header;
  if (!decode_ethercat_frame_header(bytes, size, header) ||
    size != kEthercatFrameHeaderSize + header.payload_length)
  {
    return false;
  }
  frame.datagrams.clear();
  std::size_t offset = kEthercatFrameHeaderSize;
  while (offset < size) {
    EthercatDatagramHeader datagram_header;
    if (!decode_ethercat_datagram_header(bytes + offset, size - offset, datagram_header)) {
      return false;
    }
    const std::size_t datagram_size = kEthercatDatagramHeaderSize + datagram_header.data_length +
      kEthercatWorkingCounterSize;
    if (datagram_size > size - offset) {
      return false;
    }
    EthercatDatagram datagram;
    if (!decode_ethercat_datagram(bytes + offset, datagram_size, datagram)) {
      return false;
    }
    const bool is_last = offset + datagram_size == size;
    if (datagram.header.more == is_last) {
      return false;
    }
    frame.datagrams.push_back(std::move(datagram));
    offset += datagram_size;
  }
  return !frame.datagrams.empty();
}

bool encode_lift_rx_pdo_frame(
  const LiftRxPdo & pdo, uint8_t datagram_index, uint32_t logical_address,
  uint16_t working_counter, std::vector<uint8_t> & frame_bytes)
{
  LiftRxPdoWire wire{};
  if (!encode_lift_rx_pdo_le(pdo, wire)) {
    return false;
  }
  EthercatDatagram datagram;
  datagram.header.command = EthercatCommand::lwr;
  datagram.header.index = datagram_index;
  datagram.header.address = ethercat_logical_address(logical_address);
  datagram.header.data_length = static_cast<uint16_t>(wire.size());
  datagram.data.assign(wire.begin(), wire.end());
  datagram.working_counter = working_counter;
  EthercatFrame frame;
  frame.datagrams.push_back(std::move(datagram));
  return encode_ethercat_frame(frame, frame_bytes);
}

bool decode_lift_tx_pdo_frame(
  const uint8_t * frame_bytes, std::size_t frame_size, LiftTxPdo & pdo,
  uint16_t & working_counter, std::string & error)
{
  EthercatFrame frame;
  if (!decode_ethercat_frame(frame_bytes, frame_size, frame)) {
    error = "invalid EtherCAT frame";
    return false;
  }
  if (frame.datagrams.size() != 1 ||
    frame.datagrams.front().header.command != EthercatCommand::lrd)
  {
    error = "expected one logical-read datagram for the lift TxPDO";
    return false;
  }
  const auto & datagram = frame.datagrams.front();
  if (datagram.data.size() != sizeof(LiftTxPdo)) {
    error = "lift TxPDO length does not match the configured 17-byte mapping";
    return false;
  }
  LiftTxPdoWire wire{};
  std::copy(datagram.data.begin(), datagram.data.end(), wire.begin());
  if (!decode_lift_tx_pdo_le(wire, pdo)) {
    error = "failed to decode lift TxPDO little-endian fields";
    return false;
  }
  working_counter = datagram.working_counter;
  return true;
}

}  // namespace joint_hardware::lift

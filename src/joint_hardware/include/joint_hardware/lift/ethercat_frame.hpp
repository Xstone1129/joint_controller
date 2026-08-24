#ifndef JOINT_HARDWARE__LIFT__ETHERCAT_FRAME_HPP_
#define JOINT_HARDWARE__LIFT__ETHERCAT_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "joint_hardware/lift/ethercat_backend.hpp"

namespace joint_hardware::lift
{

// EtherCAT is carried directly in Ethernet frames with EtherType 0x88A4. The
// functions below encode the EtherCAT payload that follows the Ethernet header;
// a future raw-socket adapter is responsible for the six-byte MAC addresses.
constexpr uint16_t kEthercatEtherType = 0x88A4;
constexpr uint8_t kEthercatFrameType = 0x1;
constexpr std::size_t kEthernetHeaderSize = 14;
constexpr std::size_t kEthercatFrameHeaderSize = 2;
constexpr std::size_t kEthercatDatagramHeaderSize = 10;
constexpr std::size_t kEthercatWorkingCounterSize = 2;
constexpr uint16_t kEthercatLengthMask = 0x07FF;
constexpr uint16_t kEthercatDatagramMoreFlag = 0x8000;

enum class EthercatCommand : uint8_t
{
  nop = 0x00,
  aprd = 0x01,
  apwr = 0x02,
  aprw = 0x03,
  fprd = 0x04,
  fpwr = 0x05,
  fprw = 0x06,
  brd = 0x07,
  bwr = 0x08,
  brw = 0x09,
  lrd = 0x0A,
  lwr = 0x0B,
  lrw = 0x0C,
  armw = 0x0D,
  frmw = 0x0E
};

using EthercatMacAddress = std::array<uint8_t, 6>;

struct EthercatEthernetHeader
{
  EthercatMacAddress destination{};
  EthercatMacAddress source{};
  uint16_t ether_type{kEthercatEtherType};
};

struct EthercatFrameHeader
{
  uint16_t payload_length{0};
  uint8_t type{kEthercatFrameType};
};

struct EthercatDatagramHeader
{
  EthercatCommand command{EthercatCommand::nop};
  uint8_t index{0};
  uint32_t address{0};
  uint16_t data_length{0};
  uint16_t interrupt{0};
  bool more{false};
};

// Data excludes the two-byte working counter. This owning representation is
// used by offline tests and configuration tooling; the 100 Hz path should use
// fixed PDO buffers and the explicit encode/decode helpers in ethercat_backend.
struct EthercatDatagram
{
  EthercatDatagramHeader header{};
  std::vector<uint8_t> data;
  uint16_t working_counter{0};
};

struct EthercatFrame
{
  std::vector<EthercatDatagram> datagrams;
};

constexpr uint32_t ethercat_physical_address(uint16_t auto_increment, uint16_t offset) noexcept
{
  return static_cast<uint32_t>(auto_increment) |
         (static_cast<uint32_t>(offset) << 16U);
}

constexpr uint32_t ethercat_logical_address(uint32_t address) noexcept
{
  return address;
}

bool encode_ethercat_ethernet_header(
  const EthercatEthernetHeader & header,
  std::array<uint8_t, kEthernetHeaderSize> & bytes) noexcept;
bool decode_ethercat_ethernet_header(
  const uint8_t * bytes, std::size_t size, EthercatEthernetHeader & header) noexcept;

bool encode_ethercat_frame_header(
  const EthercatFrameHeader & header,
  std::array<uint8_t, kEthercatFrameHeaderSize> & bytes) noexcept;
bool decode_ethercat_frame_header(
  const uint8_t * bytes, std::size_t size, EthercatFrameHeader & header) noexcept;

bool encode_ethercat_datagram_header(
  const EthercatDatagramHeader & header,
  std::array<uint8_t, kEthercatDatagramHeaderSize> & bytes) noexcept;
bool decode_ethercat_datagram_header(
  const uint8_t * bytes, std::size_t size, EthercatDatagramHeader & header) noexcept;

bool encode_ethercat_datagram(const EthercatDatagram & datagram, std::vector<uint8_t> & bytes);
bool decode_ethercat_datagram(
  const uint8_t * bytes, std::size_t size, EthercatDatagram & datagram);
bool encode_ethercat_frame(const EthercatFrame & frame, std::vector<uint8_t> & bytes);
bool decode_ethercat_frame(const uint8_t * bytes, std::size_t size, EthercatFrame & frame);

// Build the single logical-write PDO datagram used for the lift output and
// parse a single logical-read response containing the lift input PDO.
bool encode_lift_rx_pdo_frame(
  const LiftRxPdo & pdo, uint8_t datagram_index, uint32_t logical_address,
  uint16_t working_counter, std::vector<uint8_t> & frame_bytes);
bool decode_lift_tx_pdo_frame(
  const uint8_t * frame_bytes, std::size_t frame_size, LiftTxPdo & pdo,
  uint16_t & working_counter, std::string & error);

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ETHERCAT_FRAME_HPP_

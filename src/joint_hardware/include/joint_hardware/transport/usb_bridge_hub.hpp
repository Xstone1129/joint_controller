#ifndef JOINT_HARDWARE__TRANSPORT__USB_BRIDGE_HUB_HPP_
#define JOINT_HARDWARE__TRANSPORT__USB_BRIDGE_HUB_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace joint_hardware
{

struct CanValue
{
  uint32_t id{0};
  uint8_t data_length{0};
  bool is_fd{false};
  bool bitrate_switch{false};
  uint8_t data[64]{};
};

struct UsbBridgeRouteConfig
{
  uint8_t right_tool_channel{0};
  uint8_t left_tool_channel{1};
  uint8_t waist_channel{2};
  uint32_t right_tool_bitrate{2000000U};
  uint32_t left_tool_bitrate{2000000U};
  uint32_t waist_bitrate{5000000U};
};

class UsbBridgeHub
{
public:
  using FrameCallback = std::function<void (const CanValue &)>;
  using ListenerHandle = uint64_t;

  static std::shared_ptr<UsbBridgeHub> acquire(
    uint32_t serial_baudrate, const std::string & waist_port,
    const UsbBridgeRouteConfig & route_config);
  ~UsbBridgeHub();

  UsbBridgeHub(const UsbBridgeHub &) = delete;
  UsbBridgeHub & operator=(const UsbBridgeHub &) = delete;

  ListenerHandle add_frame_listener(FrameCallback callback);
  void remove_frame_listener(ListenerHandle handle);
  void send_frame(
    uint8_t channel, const std::vector<uint8_t> & data, uint32_t can_id,
    bool fd, bool bitrate_switch);
  bool online() const;
  std::string last_error() const;

private:
  struct Impl;
  explicit UsbBridgeHub(
    uint32_t serial_baudrate, const std::string & waist_port,
    const UsbBridgeRouteConfig & route_config);

  std::unique_ptr<Impl> impl_;
};

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__TRANSPORT__USB_BRIDGE_HUB_HPP_

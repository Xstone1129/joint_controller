#include "joint_hardware/waist_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace joint_hardware
{

bool WaistHardware::open_can()
{
  close_can();
  return open_serial_bridge();
}

void WaistHardware::close_can(bool allow_blocking_bridge_release)
{
  close_serial_bridge(allow_blocking_bridge_release);
}

bool WaistHardware::can_transport_ready() const
{
  return usb_bridge_ && usb_bridge_->online();
}

bool WaistHardware::open_serial_bridge()
{
  try {
    usb_bridge_ = UsbBridgeHub::acquire(
      static_cast<uint32_t>(serial_baudrate_),
      waist_port_,
      bridge_route_config_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to acquire waist USB-FDCAN bridge: %s", e.what());
    usb_bridge_.reset();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(bridge_rx_mutex_);
    bridge_rx_queue_.clear();
  }
  usb_bridge_listener_handle_ = usb_bridge_->add_frame_listener(
    [this](const CanValue & value) {
      handle_bridge_frame(value);
    });
  RCLCPP_INFO(
    logger_,
    "Waist USB-FDCAN bridge ready: port=%s channel=%d bitrate=%d",
    waist_port_.c_str(), bridge_channel_ + 1, bridge_bitrate_);
  return true;
}

void WaistHardware::close_serial_bridge(bool allow_blocking_release)
{
  if (usb_bridge_ && usb_bridge_listener_handle_ != 0) {
    usb_bridge_->remove_frame_listener(usb_bridge_listener_handle_);
    usb_bridge_listener_handle_ = 0;
  }
  (void)allow_blocking_release;
  usb_bridge_.reset();
  {
    std::lock_guard<std::mutex> lock(bridge_rx_mutex_);
    bridge_rx_queue_.clear();
  }
  bridge_rx_cv_.notify_all();
}

void WaistHardware::handle_bridge_frame(const CanValue & value)
{
  const uint32_t can_id = value.id & CAN_SFF_MASK;
  if (try_update_waist_feedback(can_id, value.data, value.data_length)) {
    bridge_rx_cv_.notify_all();
    return;
  }
  if (can_id != waist_can::sdo_rx_id(waist_node_id_)) {
    return;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(std::min<int>(value.data_length, 8));
  std::memcpy(frame.data, value.data, frame.can_dlc);

  {
    std::lock_guard<std::mutex> lock(bridge_rx_mutex_);
    bridge_rx_queue_.push_back(frame);
    while (bridge_rx_queue_.size() > 32) {
      bridge_rx_queue_.pop_front();
    }
  }
  bridge_rx_cv_.notify_all();
}

bool WaistHardware::send_frame(
  uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc)
{
  if (!usb_bridge_) {
    return false;
  }

  const uint32_t std_id = can_id & CAN_SFF_MASK;
  const bool is_sdo =
    std_id >= waist_can::kSdoTxBaseId && std_id < waist_can::kSdoTxBaseId + 0x80U;
  try {
    const uint8_t payload_size = std::min<uint8_t>(dlc, 8U);
    std::vector<uint8_t> payload(data.begin(), data.begin() + payload_size);
    usb_bridge_->send_frame(
      static_cast<uint8_t>(bridge_channel_),
      payload,
      std_id,
      !is_sdo,
      !is_sdo);
    return true;
  } catch (const std::exception & e) {
    static auto last_log = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log > std::chrono::seconds(1)) {
      RCLCPP_WARN(
        logger_, "Waist USB-FDCAN send failed: id=0x%03X error=%s",
        static_cast<unsigned int>(std_id), e.what());
      last_log = now;
    }
    return false;
  }
}

bool WaistHardware::recv_frame(struct can_frame & frame, int timeout_ms)
{
  if (!usb_bridge_) {
    return false;
  }

  std::unique_lock<std::mutex> lock(bridge_rx_mutex_);
  const auto ready = [this]() {return !bridge_rx_queue_.empty() || !usb_bridge_;};
  if (bridge_rx_queue_.empty()) {
    if (!bridge_rx_cv_.wait_for(
        lock, std::chrono::milliseconds(std::max(0, timeout_ms)), ready))
    {
      return false;
    }
  }
  if (bridge_rx_queue_.empty()) {
    return false;
  }
  frame = bridge_rx_queue_.front();
  bridge_rx_queue_.pop_front();
  return true;
}

}  // namespace joint_hardware

#include "joint_hardware/transport/usb_bridge_hub.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>

namespace joint_hardware
{
namespace
{

constexpr uint8_t kSync0 = 0x55;
constexpr uint8_t kSync1 = 0xAA;
constexpr uint8_t kMsgCanTx = 0x01;
constexpr uint8_t kMsgCanSetBitrate = 0x02;
constexpr uint8_t kMsgCanRx = 0x81;
constexpr uint8_t kMsgCanSetBitrateAck = 0x82;
constexpr uint8_t kFlagFd = 1U << 2U;
constexpr uint8_t kFlagBrs = 1U << 3U;
constexpr std::size_t kCanHeaderSize = 7;

uint8_t crc8(const uint8_t * data, std::size_t size)
{
  uint8_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ?
        static_cast<uint8_t>((crc << 1U) ^ 0x07U) : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

void encode_u32_le(uint8_t * output, uint32_t value)
{
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8U);
  output[2] = static_cast<uint8_t>(value >> 16U);
  output[3] = static_cast<uint8_t>(value >> 24U);
}

uint32_t decode_u32_le(const uint8_t * input)
{
  return static_cast<uint32_t>(input[0]) |
         (static_cast<uint32_t>(input[1]) << 8U) |
         (static_cast<uint32_t>(input[2]) << 16U) |
         (static_cast<uint32_t>(input[3]) << 24U);
}

speed_t baud_constant(uint32_t requested)
{
  switch (requested) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
    default: throw std::runtime_error(
              "unsupported host serial baudrate " + std::to_string(
                requested));
  }
}

void write_all(int fd, const uint8_t * data, std::size_t size)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  std::size_t offset = 0;
  while (offset < size) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      throw std::runtime_error("USB-FDCAN serial write timeout");
    }
    pollfd descriptor{fd, POLLOUT, 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (result < 0 && errno == EINTR) {continue;}
    if (result <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("USB-FDCAN serial endpoint is not writable");
    }
    const ssize_t count = ::write(fd, data + offset, size - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
    } else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      throw std::runtime_error(
              std::string("USB-FDCAN serial write failed: ") +
              std::strerror(errno));
    }
  }
}

}  // namespace

struct UsbBridgeHub::Impl
{
  int fd{-1};
  uint8_t waist_channel{2};
  uint32_t waist_bitrate{5000000U};
  std::atomic<bool> stop{false};
  std::atomic<bool> link_online{false};
  std::thread rx_thread;
  std::mutex write_mutex;
  std::mutex callback_mutex;
  std::unordered_map<ListenerHandle, FrameCallback> callbacks;
  ListenerHandle next_handle{1};
  std::mutex ack_mutex;
  std::condition_variable ack_cv;
  bool ack_received{false};
  uint8_t ack_channel{0};
  uint8_t ack_status{0xFF};
  uint32_t ack_bitrate{0};
  mutable std::mutex error_mutex;
  std::string error;
  std::vector<uint8_t> rx_buffer;

  void set_error(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(error_mutex);
    error = message;
  }

  void send_packet(uint8_t type, const uint8_t * payload, uint8_t payload_size)
  {
    if (!link_online.load(std::memory_order_acquire) || fd < 0) {
      throw std::runtime_error("USB-FDCAN bridge is offline");
    }
    std::vector<uint8_t> frame;
    frame.reserve(static_cast<std::size_t>(payload_size) + 5U);
    frame.push_back(kSync0);
    frame.push_back(kSync1);
    frame.push_back(type);
    frame.push_back(payload_size);
    frame.insert(frame.end(), payload, payload + payload_size);
    frame.push_back(crc8(frame.data() + 2, static_cast<std::size_t>(payload_size) + 2U));
    std::lock_guard<std::mutex> lock(write_mutex);
    write_all(fd, frame.data(), frame.size());
  }

  void handle_packet(uint8_t type, const uint8_t * payload, uint8_t payload_size)
  {
    if (type == kMsgCanSetBitrateAck && payload_size == 6U) {
      {
        std::lock_guard<std::mutex> lock(ack_mutex);
        ack_channel = payload[0];
        ack_status = payload[1];
        ack_bitrate = decode_u32_le(payload + 2);
        ack_received = true;
      }
      ack_cv.notify_all();
      return;
    }
    if (type != kMsgCanRx || payload_size < kCanHeaderSize) {
      return;
    }
    const uint8_t data_size = payload[6];
    if (data_size > 64U || payload_size != kCanHeaderSize + data_size) {
      return;
    }
    CanValue value;
    value.id = decode_u32_le(payload + 2);
    value.data_length = data_size;
    value.is_fd = (payload[1] & kFlagFd) != 0U;
    value.bitrate_switch = (payload[1] & kFlagBrs) != 0U;
    std::copy(payload + kCanHeaderSize, payload + kCanHeaderSize + data_size, value.data);
    std::vector<FrameCallback> listeners;
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      listeners.reserve(callbacks.size());
      for (const auto & entry : callbacks) {listeners.push_back(entry.second);}
    }
    for (const auto & callback : listeners) {
      if (callback) {callback(value);}
    }
  }

  void process_bytes(const uint8_t * data, std::size_t size)
  {
    rx_buffer.insert(rx_buffer.end(), data, data + size);
    while (rx_buffer.size() >= 5U) {
      if (rx_buffer[0] != kSync0 || rx_buffer[1] != kSync1) {
        rx_buffer.erase(rx_buffer.begin());
        continue;
      }
      const uint8_t payload_size = rx_buffer[3];
      const std::size_t frame_size = static_cast<std::size_t>(payload_size) + 5U;
      if (rx_buffer.size() < frame_size) {return;}
      if (rx_buffer[frame_size - 1U] == crc8(rx_buffer.data() + 2, payload_size + 2U)) {
        handle_packet(rx_buffer[2], rx_buffer.data() + 4, payload_size);
      }
      rx_buffer.erase(
        rx_buffer.begin(),
        rx_buffer.begin() + static_cast<std::ptrdiff_t>(frame_size));
    }
  }

  void receive_loop()
  {
    std::array<uint8_t, 512> data{};
    while (!stop.load(std::memory_order_acquire)) {
      pollfd descriptor{fd, POLLIN, 0};
      const int result = ::poll(&descriptor, 1, 100);
      if (result < 0 && errno == EINTR) {continue;}
      if (result < 0 ||
        (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0))
      {
        if (!stop.load()) {set_error("USB-FDCAN serial link lost");}
        link_online.store(false, std::memory_order_release);
        return;
      }
      if (result == 0 || (descriptor.revents & POLLIN) == 0) {continue;}
      const ssize_t count = ::read(fd, data.data(), data.size());
      if (count > 0) {
        process_bytes(data.data(), static_cast<std::size_t>(count));
      } else if (count == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
        if (!stop.load()) {set_error("USB-FDCAN serial read failed");}
        link_online.store(false, std::memory_order_release);
        return;
      }
    }
  }

  void configure_bitrate()
  {
    uint8_t payload[5] = {waist_channel, 0, 0, 0, 0};
    encode_u32_le(payload + 1, waist_bitrate);
    for (int attempt = 0; attempt < 3; ++attempt) {
      {
        std::lock_guard<std::mutex> lock(ack_mutex);
        ack_received = false;
      }
      send_packet(kMsgCanSetBitrate, payload, sizeof(payload));
      std::unique_lock<std::mutex> lock(ack_mutex);
      const bool received = ack_cv.wait_for(
        lock, std::chrono::seconds(1), [this]() {return ack_received || !link_online.load();});
      // Older bridge firmware reports success as an errno-style 0, while the
      // STM32 CDC firmware used on the robot reports it as a boolean 1.  The
      // motor-specific initialization that follows this transport setup still
      // requires fresh feedback and SDO acknowledgements before enabling.
      const bool status_success = ack_status == 0U || ack_status == 1U;
      if (received && ack_received && ack_channel == waist_channel &&
        ack_bitrate == waist_bitrate && status_success)
      {
        return;
      }
    }
    throw std::runtime_error("USB-FDCAN bridge did not confirm waist channel bitrate");
  }
};

std::shared_ptr<UsbBridgeHub> UsbBridgeHub::acquire(
  uint32_t serial_baudrate, const std::string & waist_port,
  const UsbBridgeRouteConfig & route_config)
{
  return std::shared_ptr<UsbBridgeHub>(
    new UsbBridgeHub(serial_baudrate, waist_port, route_config));
}

UsbBridgeHub::UsbBridgeHub(
  uint32_t serial_baudrate, const std::string & waist_port,
  const UsbBridgeRouteConfig & route_config)
: impl_(std::make_unique<Impl>())
{
  if (waist_port.empty()) {throw std::invalid_argument("waist serial port is empty");}
  impl_->waist_channel = route_config.waist_channel;
  impl_->waist_bitrate = route_config.waist_bitrate;
  impl_->fd = ::open(waist_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (impl_->fd < 0) {
    throw std::runtime_error(
            "failed to open waist port '" + waist_port + "': " + std::strerror(errno));
  }
  termios options{};
  if (tcgetattr(impl_->fd, &options) != 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
    throw std::runtime_error("failed to read waist serial settings");
  }
  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  const uint32_t host_baud = waist_port.find("ttyACM") !=
    std::string::npos ? 115200U : serial_baudrate;
  const speed_t speed = baud_constant(host_baud);
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (tcsetattr(impl_->fd, TCSANOW, &options) != 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
    throw std::runtime_error("failed to configure waist serial port");
  }
  tcflush(impl_->fd, TCIOFLUSH);
  impl_->link_online.store(true, std::memory_order_release);
  impl_->rx_thread = std::thread([this]() {impl_->receive_loop();});
  try {
    impl_->configure_bitrate();
  } catch (...) {
    impl_->stop.store(true);
    impl_->link_online.store(false);
    if (impl_->rx_thread.joinable()) {impl_->rx_thread.join();}
    ::close(impl_->fd);
    impl_->fd = -1;
    throw;
  }
}

UsbBridgeHub::~UsbBridgeHub()
{
  if (!impl_) {return;}
  impl_->stop.store(true, std::memory_order_release);
  impl_->link_online.store(false, std::memory_order_release);
  if (impl_->rx_thread.joinable()) {impl_->rx_thread.join();}
  if (impl_->fd >= 0) {::close(impl_->fd);}
}

UsbBridgeHub::ListenerHandle UsbBridgeHub::add_frame_listener(FrameCallback callback)
{
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  const ListenerHandle handle = impl_->next_handle++;
  impl_->callbacks.emplace(handle, std::move(callback));
  return handle;
}

void UsbBridgeHub::remove_frame_listener(ListenerHandle handle)
{
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->callbacks.erase(handle);
}

void UsbBridgeHub::send_frame(
  uint8_t channel, const std::vector<uint8_t> & data, uint32_t can_id,
  bool fd, bool bitrate_switch)
{
  if (data.size() > 64U || (!fd && data.size() > 8U) || (bitrate_switch && !fd)) {
    throw std::invalid_argument("invalid CAN/CAN-FD frame layout");
  }
  std::vector<uint8_t> payload(kCanHeaderSize + data.size());
  payload[0] = channel;
  payload[1] = static_cast<uint8_t>((fd ? kFlagFd : 0U) | (bitrate_switch ? kFlagBrs : 0U));
  encode_u32_le(payload.data() + 2, can_id);
  payload[6] = static_cast<uint8_t>(data.size());
  std::copy(data.begin(), data.end(), payload.begin() + kCanHeaderSize);
  impl_->send_packet(kMsgCanTx, payload.data(), static_cast<uint8_t>(payload.size()));
}

bool UsbBridgeHub::online() const
{
  return impl_ && impl_->link_online.load(std::memory_order_acquire);
}

std::string UsbBridgeHub::last_error() const
{
  if (!impl_) {return "bridge is not initialized";}
  std::lock_guard<std::mutex> lock(impl_->error_mutex);
  return impl_->error;
}

}  // namespace joint_hardware

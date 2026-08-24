#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "joint_hardware/transport/usb_bridge_hub.hpp"

namespace
{

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

void append_u32(std::vector<uint8_t> & data, uint32_t value)
{
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
  data.push_back(static_cast<uint8_t>(value >> 16U));
  data.push_back(static_cast<uint8_t>(value >> 24U));
}

std::vector<uint8_t> packet(uint8_t type, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> result{0x55, 0xAA, type, static_cast<uint8_t>(payload.size())};
  result.insert(result.end(), payload.begin(), payload.end());
  result.push_back(crc8(result.data() + 2, payload.size() + 2));
  return result;
}

bool read_packet(int fd, std::vector<uint8_t> & output, int timeout_ms)
{
  output.clear();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{fd, POLLIN, 0};
    const int remaining = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count());
    if (::poll(&descriptor, 1, std::max(1, remaining)) <= 0) {continue;}
    std::array<uint8_t, 128> buffer{};
    const ssize_t count = ::read(fd, buffer.data(), buffer.size());
    if (count > 0) {output.insert(output.end(), buffer.begin(), buffer.begin() + count);}
    while (output.size() >= 5U && (output[0] != 0x55 || output[1] != 0xAA)) {
      output.erase(output.begin());
    }
    if (output.size() >= 5U && output.size() >= static_cast<std::size_t>(output[3]) + 5U) {
      output.resize(static_cast<std::size_t>(output[3]) + 5U);
      return output.back() == crc8(output.data() + 2, output[3] + 2U);
    }
  }
  return false;
}

}  // namespace

TEST(UsbBridgeHub, ConfiguresBitrateAndExchangesCanFdFrames)
{
  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master_fd, 0);
  ASSERT_EQ(grantpt(master_fd), 0);
  ASSERT_EQ(unlockpt(master_fd), 0);
  const char * slave_name = ptsname(master_fd);
  ASSERT_NE(slave_name, nullptr);

  std::vector<uint8_t> transmitted;
  std::thread firmware([&]() {
      std::vector<uint8_t> request;
      ASSERT_TRUE(read_packet(master_fd, request, 1500));
      ASSERT_EQ(request[2], 0x02);
      ASSERT_EQ(request[4], 2);
      const std::vector<uint8_t> ack_payload{2, 0, 0x40, 0x4B, 0x4C, 0x00};
      const auto ack = packet(0x82, ack_payload);
      ASSERT_EQ(::write(master_fd, ack.data(), ack.size()), static_cast<ssize_t>(ack.size()));

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::vector<uint8_t> rx_payload{2, 0x0C};
      append_u32(rx_payload, 0x30F);
      rx_payload.push_back(12);
      for (uint8_t value = 0; value < 12; ++value) {
        rx_payload.push_back(value);
      }
      const auto rx = packet(0x81, rx_payload);
      ASSERT_EQ(::write(master_fd, rx.data(), rx.size()), static_cast<ssize_t>(rx.size()));
      ASSERT_TRUE(read_packet(master_fd, transmitted, 1500));
    });

  joint_hardware::UsbBridgeRouteConfig route;
  route.waist_channel = 2;
  route.waist_bitrate = 5000000U;
  auto bridge = joint_hardware::UsbBridgeHub::acquire(115200, slave_name, route);
  ASSERT_TRUE(bridge->online());

  std::mutex mutex;
  std::condition_variable condition;
  bool received = false;
  joint_hardware::CanValue received_frame;
  const auto listener = bridge->add_frame_listener(
    [&](const joint_hardware::CanValue & frame) {
      std::lock_guard<std::mutex> lock(mutex);
      received_frame = frame;
      received = true;
      condition.notify_all();
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {return received;}));
  }
  EXPECT_EQ(received_frame.id, 0x30FU);
  EXPECT_EQ(received_frame.data_length, 12);
  EXPECT_TRUE(received_frame.is_fd);
  EXPECT_TRUE(received_frame.bitrate_switch);
  EXPECT_EQ(received_frame.data[11], 11);

  bridge->send_frame(2, {0xC2, 0x01, 0x02}, 0x10F, true, true);
  firmware.join();
  ASSERT_GE(transmitted.size(), 15U);
  EXPECT_EQ(transmitted[2], 0x01);
  EXPECT_EQ(transmitted[4], 2);
  EXPECT_EQ(transmitted[5], 0x0C);
  EXPECT_EQ(transmitted[6], 0x0F);
  EXPECT_EQ(transmitted[7], 0x01);
  EXPECT_EQ(transmitted[10], 3);
  EXPECT_EQ(transmitted[11], 0xC2);
  bridge->remove_frame_listener(listener);
  bridge.reset();
  ::close(master_fd);
}

TEST(UsbBridgeHub, AcceptsBooleanSuccessBitrateAck)
{
  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master_fd, 0);
  ASSERT_EQ(grantpt(master_fd), 0);
  ASSERT_EQ(unlockpt(master_fd), 0);
  const char * slave_name = ptsname(master_fd);
  ASSERT_NE(slave_name, nullptr);

  std::thread firmware([&]() {
      std::vector<uint8_t> request;
      ASSERT_TRUE(read_packet(master_fd, request, 1500));
      ASSERT_EQ(request[2], 0x02);
      const std::vector<uint8_t> ack_payload{2, 1, 0x40, 0x4B, 0x4C, 0x00};
      const auto ack = packet(0x82, ack_payload);
      ASSERT_EQ(::write(master_fd, ack.data(), ack.size()), static_cast<ssize_t>(ack.size()));
    });

  joint_hardware::UsbBridgeRouteConfig route;
  route.waist_channel = 2;
  route.waist_bitrate = 5000000U;
  auto bridge = joint_hardware::UsbBridgeHub::acquire(115200, slave_name, route);
  EXPECT_TRUE(bridge->online());

  firmware.join();
  bridge.reset();
  ::close(master_fd);
}

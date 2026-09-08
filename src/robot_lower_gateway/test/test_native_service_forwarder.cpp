#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"
#include "robot_lower_gateway/native_service_forwarder.hpp"

namespace
{

using namespace std::chrono_literals;
using Service = robot_control_msg::srv::SetRobotPower;

class NativeServiceForwarderTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(NativeServiceForwarderTest, UnavailableServiceReturnsWithinDeadline)
{
  auto client_node = std::make_shared<rclcpp::Node>("forwarder_unavailable_client");
  auto client = client_node->create_client<Service>("/test/native_unavailable");
  auto request = std::make_shared<Service::Request>();
  std::string error;
  const auto started = std::chrono::steady_clock::now();
  const auto response = robot_lower_gateway::callNativeServiceBounded<Service>(
    request, client, "/test/native_unavailable", 40ms, error);

  EXPECT_EQ(response, nullptr);
  EXPECT_NE(error.find("native service unavailable"), std::string::npos);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
}

TEST_F(NativeServiceForwarderTest, ResponseIsDeliveredByAnotherExecutorThread)
{
  auto server_node = std::make_shared<rclcpp::Node>("forwarder_response_server");
  auto client_node = std::make_shared<rclcpp::Node>("forwarder_response_client");
  auto service = server_node->create_service<Service>(
    "/test/native_response",
    [](const Service::Request::SharedPtr, Service::Response::SharedPtr response) {
      response->success = false;
      response->message = "downstream refusal preserved";
    });
  auto client = client_node->create_client<Service>("/test/native_response");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto request = std::make_shared<Service::Request>();
  std::string error;
  const auto response = robot_lower_gateway::callNativeServiceBounded<Service>(
    request, client, "/test/native_response", 2s, error);

  ASSERT_NE(response, nullptr) << error;
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "downstream refusal preserved");
  executor.cancel();
  spin_thread.join();
  (void)service;
}

TEST_F(NativeServiceForwarderTest, NonrespondingServiceTimesOut)
{
  auto server_node = std::make_shared<rclcpp::Node>("forwarder_timeout_server");
  auto client_node = std::make_shared<rclcpp::Node>("forwarder_timeout_client");
  std::atomic<bool> callback_started{false};
  auto service = server_node->create_service<Service>(
    "/test/native_timeout",
    [&callback_started](const Service::Request::SharedPtr, Service::Response::SharedPtr response) {
      callback_started.store(true);
      std::this_thread::sleep_for(200ms);
      response->success = true;
    });
  auto client = client_node->create_client<Service>("/test/native_timeout");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto request = std::make_shared<Service::Request>();
  std::string error;
  const auto started = std::chrono::steady_clock::now();
  const auto response = robot_lower_gateway::callNativeServiceBounded<Service>(
    request, client, "/test/native_timeout", 60ms, error);

  EXPECT_EQ(response, nullptr);
  EXPECT_TRUE(callback_started.load());
  EXPECT_NE(error.find("timeout waiting for native service"), std::string::npos);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
  executor.cancel();
  spin_thread.join();
  (void)service;
}

}  // namespace

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

class GatewayChild
{
public:
  GatewayChild(std::string native_service, std::chrono::milliseconds timeout)
  {
    const std::string native_parameter =
      "native_services.lift_set_drive_zero:=" + std::move(native_service);
    const std::string timeout_parameter =
      "command_timeout_ms.lift_set_drive_zero:=" + std::to_string(timeout.count());
    pid_ = fork();
    if (pid_ == 0) {
      execl(
        GATEWAY_NODE_EXECUTABLE, GATEWAY_NODE_EXECUTABLE,
        "--ros-args", "--params-file", GATEWAY_TEST_CONFIG,
        "-p", "enable_command_proxy:=true",
        "-p", "command_proxy_prefix:=~/",
        "-p", native_parameter.c_str(),
        "-p", timeout_parameter.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
  }

  ~GatewayChild()
  {
    stop();
  }

  GatewayChild(const GatewayChild &) = delete;
  GatewayChild & operator=(const GatewayChild &) = delete;

  bool running() const
  {
    if (pid_ <= 0) {
      return false;
    }
    int status = 0;
    return waitpid(pid_, &status, WNOHANG) == 0;
  }

  void stop()
  {
    if (pid_ <= 0) {
      return;
    }
    kill(pid_, SIGINT);
    for (int attempt = 0; attempt < 20; ++attempt) {
      int status = 0;
      if (waitpid(pid_, &status, WNOHANG) == pid_) {
        pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }

private:
  pid_t pid_{-1};
};

class NodeExecutor
{
public:
  explicit NodeExecutor(const rclcpp::Node::SharedPtr & node)
  {
    executor_.add_node(node);
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~NodeExecutor()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  NodeExecutor(const NodeExecutor &) = delete;
  NodeExecutor & operator=(const NodeExecutor &) = delete;

private:
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::thread thread_;
};

Trigger::Response::SharedPtr callTrigger(
  const rclcpp::Client<Trigger>::SharedPtr & client, std::chrono::milliseconds timeout)
{
  if (!client->wait_for_service(timeout)) {
    return nullptr;
  }
  auto request = std::make_shared<Trigger::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    return nullptr;
  }
  return future.get();
}

class LiftZeroProxyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp/robot_lower_gateway_lift_zero_proxy", 1);
    setenv("ROS_DOMAIN_ID", "225", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    setenv("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4", 1);
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

  static rclcpp::Client<Trigger>::SharedPtr proxyClient(
    const rclcpp::Node::SharedPtr & node)
  {
    return node->create_client<Trigger>("/ubuntu_lower_gateway/lift/set_drive_zero");
  }
};

TEST_F(LiftZeroProxyTest, ReturnsNativeSuccessAndMessageUnchanged)
{
  auto node = std::make_shared<rclcpp::Node>("lift_zero_proxy_success_test");
  auto native_service = node->create_service<Trigger>(
    "/mock/lift_zero_success",
    [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      response->success = true;
      response->message = "drive encoder zero stored";
    });
  NodeExecutor executor(node);
  GatewayChild gateway("/mock/lift_zero_success", 1000ms);
  ASSERT_TRUE(gateway.running());

  const auto response = callTrigger(proxyClient(node), 3s);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "drive encoder zero stored");
  EXPECT_TRUE(gateway.running());
  (void)native_service;
}

TEST_F(LiftZeroProxyTest, ReturnsNativeRejectionAndMessageUnchanged)
{
  auto node = std::make_shared<rclcpp::Node>("lift_zero_proxy_rejection_test");
  auto native_service = node->create_service<Trigger>(
    "/mock/lift_zero_rejection",
    [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      response->success = false;
      response->message = "lift is moving; zero rejected";
    });
  NodeExecutor executor(node);
  GatewayChild gateway("/mock/lift_zero_rejection", 1000ms);
  ASSERT_TRUE(gateway.running());

  const auto response = callTrigger(proxyClient(node), 3s);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "lift is moving; zero rejected");
  (void)native_service;
}

TEST_F(LiftZeroProxyTest, ReturnsUnavailableWhenNativeServiceDoesNotExist)
{
  auto node = std::make_shared<rclcpp::Node>("lift_zero_proxy_unavailable_test");
  NodeExecutor executor(node);
  GatewayChild gateway("/mock/lift_zero_unavailable", 80ms);
  ASSERT_TRUE(gateway.running());

  const auto response = callTrigger(proxyClient(node), 3s);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("native service unavailable"), std::string::npos);
  EXPECT_NE(response->message.find("/mock/lift_zero_unavailable"), std::string::npos);
}

TEST_F(LiftZeroProxyTest, ReturnsTimeoutWhenNativeServiceDoesNotRespondInTime)
{
  auto node = std::make_shared<rclcpp::Node>("lift_zero_proxy_timeout_test");
  std::atomic<bool> callback_started{false};
  auto native_service = node->create_service<Trigger>(
    "/mock/lift_zero_timeout",
    [&callback_started](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      callback_started.store(true);
      std::this_thread::sleep_for(300ms);
      response->success = true;
      response->message = "late response";
    });
  NodeExecutor executor(node);
  GatewayChild gateway("/mock/lift_zero_timeout", 80ms);
  ASSERT_TRUE(gateway.running());

  const auto response = callTrigger(proxyClient(node), 3s);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(callback_started.load());
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("timeout waiting for native service"), std::string::npos);
  (void)native_service;
}

TEST_F(LiftZeroProxyTest, LeavesHoldAndStatusPublishingAvailableDuringZeroCall)
{
  auto node = std::make_shared<rclcpp::Node>("lift_zero_proxy_concurrency_test");
  std::mutex mutex;
  std::condition_variable condition;
  bool zero_started = false;
  std::size_t diagnostic_count = 0;
  auto zero_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto hold_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto native_zero_service = node->create_service<Trigger>(
    "/mock/lift_zero_slow",
    [&mutex, &condition, &zero_started](
      const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        zero_started = true;
      }
      condition.notify_all();
      std::this_thread::sleep_for(1500ms);
      response->success = true;
      response->message = "zero complete";
    }, rmw_qos_profile_services_default, zero_callback_group);
  auto native_hold_service = node->create_service<Trigger>(
    "/joint/lift/hold",
    [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      response->success = true;
      response->message = "hold confirmed";
    }, rmw_qos_profile_services_default, hold_callback_group);
  auto diagnostics_subscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/ubuntu_lower_gateway/diagnostics", rclcpp::QoS(10).reliable(),
    [&mutex, &condition, &diagnostic_count](
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        ++diagnostic_count;
      }
      condition.notify_all();
    });
  NodeExecutor executor(node);
  GatewayChild gateway("/mock/lift_zero_slow", 3000ms);
  ASSERT_TRUE(gateway.running());

  auto zero_client = proxyClient(node);
  auto hold_client = node->create_client<Trigger>("/ubuntu_lower_gateway/lift/hold");
  ASSERT_TRUE(zero_client->wait_for_service(3s));
  ASSERT_TRUE(hold_client->wait_for_service(3s));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 3s, [&diagnostic_count]() {
      return diagnostic_count > 0;
    }));
  }
  const auto diagnostics_before = [&mutex, &diagnostic_count]() {
      std::lock_guard<std::mutex> lock(mutex);
      return diagnostic_count;
    }();

  auto zero_future = zero_client->async_send_request(std::make_shared<Trigger::Request>());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 2s, [&zero_started]() {return zero_started;}));
  }
  const auto hold_response = callTrigger(hold_client, 1000ms);
  ASSERT_NE(hold_response, nullptr);
  EXPECT_TRUE(hold_response->success);
  EXPECT_EQ(hold_response->message, "hold confirmed");
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, 1000ms, [&diagnostic_count, diagnostics_before]() {
      return diagnostic_count > diagnostics_before;
    }));
  }
  ASSERT_EQ(zero_future.wait_for(3s), std::future_status::ready);
  const auto zero_response = zero_future.get();
  ASSERT_NE(zero_response, nullptr);
  EXPECT_TRUE(zero_response->success);
  EXPECT_EQ(zero_response->message, "zero complete");
  (void)native_zero_service;
  (void)native_hold_service;
  (void)diagnostics_subscription;
  (void)zero_callback_group;
  (void)hold_callback_group;
}

}  // namespace

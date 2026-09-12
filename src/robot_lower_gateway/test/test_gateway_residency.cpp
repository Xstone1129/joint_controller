#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "robot_control_msg/srv/set_robot_power.hpp"

namespace
{
using namespace std::chrono_literals;
using WorkspaceStatus = robot_control_msg::msg::WorkspaceStatus;
using SetRobotPower = robot_control_msg::srv::SetRobotPower;

class ChildProcess
{
public:
  ChildProcess()
  {
    pid_ = fork();
    if (pid_ == 0) {
      execl(
        GATEWAY_NODE_EXECUTABLE, GATEWAY_NODE_EXECUTABLE,
        "--ros-args", "--params-file", GATEWAY_TEST_CONFIG,
        "-p", "enable_command_proxy:=true",
        "-p", "command_proxy_prefix:=~/", static_cast<char *>(nullptr));
      _exit(127);
    }
  }

  ~ChildProcess()
  {
    stop();
  }

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

class GatewayResidencyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp/robot_lower_gateway_residency", 1);
    setenv("ROS_DOMAIN_ID", "223", 1);
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
};

TEST_F(GatewayResidencyTest, NodeAndPowerProxySurviveEveryWorkspacePhase)
{
  ChildProcess gateway;
  ASSERT_TRUE(gateway.running());

  auto test_node = std::make_shared<rclcpp::Node>("gateway_residency_test");
  auto native_service = test_node->create_service<SetRobotPower>(
    "/set_robot_power",
    [](const SetRobotPower::Request::SharedPtr request,
    SetRobotPower::Response::SharedPtr response) {
      EXPECT_FALSE(request->enable);
      response->success = true;
      response->message = "mock power off confirmed";
    });
  auto proxy_client = test_node->create_client<SetRobotPower>(
    "/ubuntu_lower_gateway/set_robot_power");
  auto workspace_publisher = test_node->create_publisher<WorkspaceStatus>(
    "/workspace/status", rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(test_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  ASSERT_TRUE(proxy_client->wait_for_service(5s));
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> phases = {
    {WorkspaceStatus::STOPPED, WorkspaceStatus::UNKNOWN},
    {WorkspaceStatus::STARTING, WorkspaceStatus::REAL},
    {WorkspaceStatus::RUNNING, WorkspaceStatus::REAL},
    {WorkspaceStatus::STOPPING, WorkspaceStatus::REAL},
    {WorkspaceStatus::STOPPED, WorkspaceStatus::UNKNOWN},
  };
  for (const auto & phase : phases) {
    WorkspaceStatus status;
    status.state = phase.first;
    status.mode = phase.second;
    status.accepted = true;
    workspace_publisher->publish(status);
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(gateway.running());
    EXPECT_TRUE(proxy_client->service_is_ready());
    const auto nodes = test_node->get_node_names();
    EXPECT_NE(
      std::find(nodes.begin(), nodes.end(), "/ubuntu_lower_gateway"), nodes.end());
  }

  for (int request_index = 0; request_index < 2; ++request_index) {
    auto request = std::make_shared<SetRobotPower::Request>();
    request->enable = false;
    auto future = proxy_client->async_send_request(request);
    ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
    const auto response = future.get();
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(gateway.running());
    EXPECT_TRUE(proxy_client->service_is_ready());
  }

  executor.cancel();
  spin_thread.join();
  (void)native_service;
}
}  // namespace

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/lift_status.hpp"
#include "robot_control_msg/msg/workspace_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using namespace std::chrono_literals;
using LiftStatus = robot_control_msg::msg::LiftStatus;
using WorkspaceStatus = robot_control_msg::msg::WorkspaceStatus;

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
        "-p", "enable_command_proxy:=false", static_cast<char *>(nullptr));
      _exit(127);
    }
  }

  ~ChildProcess()
  {
    if (pid_ <= 0) {
      return;
    }
    kill(pid_, SIGINT);
    for (int attempt = 0; attempt < 20; ++attempt) {
      int status = 0;
      if (waitpid(pid_, &status, WNOHANG) == pid_) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
  }

  bool running() const
  {
    int status = 0;
    return pid_ > 0 && waitpid(pid_, &status, WNOHANG) == 0;
  }

private:
  pid_t pid_{-1};
};

std_msgs::msg::String driverStatus(
  bool operation_enabled, bool motion_blocked = false,
  const std::string & fault_reason = "")
{
  std_msgs::msg::String message;
  const char * state = operation_enabled ? "operation_enabled" : "ready_to_switch_on";
  const int status_word = operation_enabled ? 39 : 1585;
  message.data =
    std::string("{\"component\":\"lift\",\"link_state\":\"operational\",") +
    "\"cia402_state\":\"" + state + "\",\"status_word\":" +
    std::to_string(status_word) +
    ",\"error_code\":0,\"mode_display\":9,\"feedback_fresh\":true," +
    "\"ethercat_operational\":true,\"working_counter\":3," +
    "\"working_counter_ok\":true,\"initialized\":true," +
    "\"power_enable_command\":" + (operation_enabled ? "true" : "false") +
    ",\"power_enabled\":" + (operation_enabled ? "true" : "false") +
    ",\"brake_unlocked\":" + (operation_enabled ? "true" : "false") +
    ",\"estop_latched\":false,\"motion_blocked\":" +
    (motion_blocked ? "true" : "false") + "," +
    "\"quick_stop_active\":false,\"position\":-0.125,\"velocity\":0.0," +
    "\"command_position\":-0.125,\"command_velocity\":0.0," +
    "\"command_acceleration\":0.0,\"fault_reason\":\"" + fault_reason + "\"}";
  return message;
}

std_msgs::msg::String controlStatus(
  const std::string & mode = "hold", const std::string & fault_reason = "")
{
  std_msgs::msg::String message;
  message.data =
    "{\"component\":\"lift_controller\",\"mode\":\"" + mode + "\"," 
    "\"position\":-0.125,\"velocity\":0.0,\"trajectory_active\":false," 
    "\"jog_active\":false,\"fault_reason\":\"" + fault_reason + "\"}";
  return message;
}

sensor_msgs::msg::JointState jointState()
{
  sensor_msgs::msg::JointState message;
  message.name = {"joint_motor"};
  message.position = {-0.125};
  message.velocity = {0.0};
  return message;
}

class LiftStatusPublicationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp/robot_lower_gateway_lift_status", 1);
    setenv("ROS_DOMAIN_ID", "224", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    setenv("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4", 1);
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(LiftStatusPublicationTest, NormalizesFeedbackFailClosedAndSurvivesLifecycle)
{
  ChildProcess gateway;
  ASSERT_TRUE(gateway.running());

  auto node = std::make_shared<rclcpp::Node>("lift_status_publication_test");
  auto driver_publisher = node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/driver_status", rclcpp::QoS(10).reliable());
  // Match the real lift_controller writer and prove that the gateway reader is compatible.
  auto control_publisher = node->create_publisher<std_msgs::msg::String>(
    "/joint/lift/control_status", rclcpp::QoS(1).best_effort());
  auto joint_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
    "/lift/joint_states", rclcpp::SensorDataQoS());
  auto workspace_publisher = node->create_publisher<WorkspaceStatus>(
    "/workspace/status", rclcpp::QoS(1).reliable().transient_local());

  std::mutex mutex;
  std::condition_variable condition;
  LiftStatus latest;
  diagnostic_msgs::msg::DiagnosticStatus latest_lift_diagnostic;
  std::size_t received = 0;
  bool lift_diagnostic_received = false;
  auto status_subscription = node->create_subscription<LiftStatus>(
    "/ubuntu_lower_gateway/lift/status",
    rclcpp::QoS(1).reliable().transient_local(),
    [&](const LiftStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      latest = *message;
      ++received;
      condition.notify_all();
    });
  auto diagnostics_subscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/ubuntu_lower_gateway/diagnostics", rclcpp::QoS(1).reliable(),
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      const auto found = std::find_if(
        message->status.begin(), message->status.end(),
        [](const diagnostic_msgs::msg::DiagnosticStatus & status) {
          return status.name == "ubuntu_lower_gateway/lift";
        });
      if (found == message->status.end()) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex);
      latest_lift_diagnostic = *found;
      lift_diagnostic_received = true;
      condition.notify_all();
    });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto wait_until = [&](const std::function<bool(const LiftStatus &)> & predicate,
    std::chrono::milliseconds timeout = 3s) {
      std::unique_lock<std::mutex> lock(mutex);
      return condition.wait_for(lock, timeout, [&]() {return predicate(latest);});
    };
  const auto publish_workspace = [&](std::uint8_t state, std::uint8_t mode) {
      WorkspaceStatus status;
      status.state = state;
      status.mode = mode;
      status.accepted = true;
      workspace_publisher->publish(status);
    };
  const auto publish_inputs = [&](const std_msgs::msg::String & driver) {
      for (int attempt = 0; attempt < 12; ++attempt) {
        driver_publisher->publish(driver);
        control_publisher->publish(controlStatus());
        joint_publisher->publish(jointState());
        std::this_thread::sleep_for(25ms);
      }
    };

  // Same as publish_inputs but with an explicit controller status, so a test can
  // drive the controller side (mode / fault_reason) independently of the drive.
  const auto publish_inputs_with_control =
    [&](const std_msgs::msg::String & driver, const std_msgs::msg::String & control) {
      for (int attempt = 0; attempt < 12; ++attempt) {
        driver_publisher->publish(driver);
        control_publisher->publish(control);
        joint_publisher->publish(jointState());
        std::this_thread::sleep_for(25ms);
      }
    };

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((driver_publisher->get_subscription_count() == 0 ||
    control_publisher->get_subscription_count() == 0 ||
    joint_publisher->get_subscription_count() == 0 ||
    status_subscription->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(25ms);
  }
  ASSERT_GT(control_publisher->get_subscription_count(), 0U);
  ASSERT_GT(status_subscription->get_publisher_count(), 0U);

  publish_workspace(WorkspaceStatus::STARTING, WorkspaceStatus::REAL);
  publish_inputs(driverStatus(false));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && status.cia402_state == "ready_to_switch_on";
    }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_TRUE(latest.feedback_fresh);
    EXPECT_TRUE(latest.ethercat_operational);
    EXPECT_TRUE(latest.working_counter_ok);
    EXPECT_TRUE(latest.initialized);
    EXPECT_FALSE(latest.command_enabled);
    EXPECT_FALSE(latest.enabled);
    EXPECT_FALSE(latest.brake_unlocked);
    EXPECT_FALSE(latest.motion_blocked);
    EXPECT_EQ(latest.controller_mode, "hold");
    EXPECT_EQ(latest.status_word, 1585U);
  }

  publish_inputs(driverStatus(false, true, "CiA 402 fault or mode mismatch"));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && !status.fault && !status.motion_blocked &&
             status.fault_reason.empty();
    }));

  // A controller-latched fault with no drive fault code must still reach the host
  // with an explanation.  The drive-side safety status carries no reason here, so
  // the published fault_reason has to fall back to the controller's own text
  // instead of staying empty ("silent fault").
  publish_inputs_with_control(
      driverStatus(false),
      controlStatus("fault", "gate_lost_during_motion: entry_mode=streaming"));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.controller_mode == "fault" &&
             status.fault_reason == "gate_lost_during_motion: entry_mode=streaming";
    }));

  publish_inputs(driverStatus(false, true, "PDO exchange failed"));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && !status.fault && !status.motion_blocked &&
             status.fault_reason.empty() &&
               status.message.find("lift feedback ready") == 0;
    }));

  publish_inputs(driverStatus(
      false, true, "feedback coordinate jump exceeded max_feedback_jump_m"));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && status.motion_blocked &&
             status.fault_reason == "feedback coordinate jump exceeded max_feedback_jump_m" &&
             status.message.find("lift motion blocked") != std::string::npos;
    }));
  publish_inputs(driverStatus(false));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && !status.motion_blocked;
    }));

  const auto verify_fail_closed_field = [&](const std::string & field,
    const std::function<bool(const LiftStatus &)> & predicate) {
      auto unhealthy = driverStatus(false);
      const std::string present = "\"" + field + "\":true";
      const auto offset = unhealthy.data.find(present);
      ASSERT_NE(offset, std::string::npos);
      unhealthy.data.replace(offset, present.size(), "\"" + field + "\":false");
      publish_inputs(unhealthy);
      ASSERT_TRUE(wait_until([&](const LiftStatus & status) {
          return !status.valid && status.motion_blocked && predicate(status);
        }));
      publish_inputs(driverStatus(false));
      ASSERT_TRUE(wait_until([](const LiftStatus & status) {return status.valid;}));
    };
  verify_fail_closed_field(
    "feedback_fresh", [](const LiftStatus & status) {return !status.feedback_fresh;});
  verify_fail_closed_field(
    "ethercat_operational",
    [](const LiftStatus & status) {return !status.ethercat_operational;});
  verify_fail_closed_field(
    "working_counter_ok", [](const LiftStatus & status) {return !status.working_counter_ok;});
  verify_fail_closed_field(
    "initialized", [](const LiftStatus & status) {return !status.initialized;});
  ASSERT_TRUE(wait_until([&](const LiftStatus &) {return lift_diagnostic_received;}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto has_key = [&](const std::string & key) {
        return std::any_of(
          latest_lift_diagnostic.values.begin(), latest_lift_diagnostic.values.end(),
          [&](const diagnostic_msgs::msg::KeyValue & value) {return value.key == key;});
      };
    EXPECT_EQ(latest_lift_diagnostic.name, "ubuntu_lower_gateway/lift");
    EXPECT_TRUE(has_key("driver_feedback_age"));
    EXPECT_TRUE(has_key("control_feedback_age"));
    EXPECT_TRUE(has_key("joint_feedback_age"));
    EXPECT_TRUE(has_key("driver_parse_valid"));
    EXPECT_TRUE(has_key("control_parse_valid"));
    EXPECT_TRUE(has_key("status_generation"));
    EXPECT_TRUE(has_key("published_count"));
    EXPECT_TRUE(has_key("last_publish_time"));
    EXPECT_TRUE(has_key("publish_rate_hz"));
    EXPECT_TRUE(has_key("degrade_reason"));
  }

  publish_workspace(WorkspaceStatus::RUNNING, WorkspaceStatus::REAL);
  publish_inputs(driverStatus(true));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return status.valid && status.cia402_state == "operation_enabled" && status.enabled;
    }));

  std_msgs::msg::String malformed;
  malformed.data = "{not-json";
  publish_inputs(malformed);
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return !status.valid && status.message.find("invalid JSON") != std::string::npos;
    }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_FALSE(latest.feedback_fresh);
    EXPECT_FALSE(latest.command_enabled);
    EXPECT_FALSE(latest.enabled);
    EXPECT_FALSE(latest.brake_unlocked);
    EXPECT_TRUE(latest.motion_blocked);
    EXPECT_NE(latest.fault_reason.find("invalid JSON"), std::string::npos);
  }

  publish_inputs(driverStatus(false));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {return status.valid;}));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return !status.valid && !status.feedback_fresh && status.motion_blocked &&
             status.message.find("stale") != std::string::npos;
    }, 2s));

  publish_inputs(driverStatus(false));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {return status.valid;}));
  publish_workspace(WorkspaceStatus::STOPPED, WorkspaceStatus::UNKNOWN);
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {
      return !status.valid && status.message.find("waiting") != std::string::npos;
    }));

  publish_workspace(WorkspaceStatus::STARTING, WorkspaceStatus::REAL);
  publish_inputs(driverStatus(false));
  ASSERT_TRUE(wait_until([](const LiftStatus & status) {return status.valid;}));

  std::atomic<std::size_t> late_received{0};
  auto late_subscription = node->create_subscription<LiftStatus>(
    "/ubuntu_lower_gateway/lift/status",
    rclcpp::QoS(1).reliable().transient_local(),
    [&](const LiftStatus::SharedPtr) {late_received.fetch_add(1);});
  const auto late_deadline = std::chrono::steady_clock::now() + 500ms;
  while (late_received.load() == 0 && std::chrono::steady_clock::now() < late_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(late_received.load(), 0U);

  std::size_t before = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    before = received;
  }
  std::this_thread::sleep_for(600ms);
  std::size_t published_in_window = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    published_in_window = received - before;
  }
  EXPECT_GE(published_in_window, 5U);
  EXPECT_LE(published_in_window, 20U);
  EXPECT_TRUE(gateway.running());

  executor.cancel();
  spin_thread.join();
  (void)diagnostics_subscription;
  (void)late_subscription;
}
}  // namespace

#include <chrono>
#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_control_msg/msg/arm_power_status.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
#include <robot_control_msg/srv/set_robot_power.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr int kUnknownStatusCode = 0;
}

class RobotPowerServiceNode : public rclcpp::Node
{
public:
  using ArmPowerStatus = robot_control_msg::msg::ArmPowerStatus;
  using SetRobotPower = robot_control_msg::srv::SetRobotPower;

  RobotPowerServiceNode()
  : Node("robot_power_service")
  {
    power_status_topic_ = declare_parameter<std::string>(
      "power_status_topic", "/arm/power_status");
    power_timeout_ms_ = declare_parameter<int>("power_timeout_ms", 5000);
    rollback_timeout_ms_ = declare_parameter<int>("rollback_timeout_ms", 1500);
    status_stale_timeout_ms_ = declare_parameter<int>("status_stale_timeout_ms", 1000);

    service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    status_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    poweron_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/robot_poweron",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = status_callback_group_;
    power_status_subscription_ = create_subscription<ArmPowerStatus>(
      power_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&RobotPowerServiceNode::handle_power_status, this, std::placeholders::_1),
      subscription_options);

    service_ = create_service<SetRobotPower>(
      "/set_robot_power",
      std::bind(
        &RobotPowerServiceNode::handle_service_request, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Robot power service ready; status_topic='%s', timeout=%d ms, rollback_timeout=%d ms, "
      "stale_timeout=%d ms",
      power_status_topic_.c_str(), power_timeout_ms_, rollback_timeout_ms_,
      status_stale_timeout_ms_);
  }

private:
  void handle_power_status(const ArmPowerStatus::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_power_status_ = *message;
      has_power_status_ = true;
      ++status_generation_;
      last_status_receive_time_ = std::chrono::steady_clock::now();
    }
    status_condition_.notify_all();
  }

  bool status_shape_valid(const ArmPowerStatus & status, std::string & error) const
  {
    constexpr std::size_t kArmJointCount = 14;
    if (status.joint_names.size() != kArmJointCount ||
      status.status_codes.size() != kArmJointCount ||
      status.enabled.size() != kArmJointCount)
    {
      error = "invalid /arm/power_status array sizes: names=" +
        std::to_string(status.joint_names.size()) + ", status_codes=" +
        std::to_string(status.status_codes.size()) + ", enabled=" +
        std::to_string(status.enabled.size());
      return false;
    }
    return true;
  }

  bool target_reached(const ArmPowerStatus & status, bool enable) const
  {
    if (enable) {
      if (!status.command_enabled || !status.all_enabled) {
        return false;
      }
      for (bool joint_enabled : status.enabled) {
        if (!joint_enabled) {
          return false;
        }
      }
      return true;
    }

    if (status.command_enabled) {
      return false;
    }
    for (bool joint_enabled : status.enabled) {
      if (joint_enabled) {
        return false;
      }
    }
    return true;
  }

  std::string unavailable_drive_message(const ArmPowerStatus & status) const
  {
    std::size_t unavailable_count = 0;
    std::ostringstream unavailable;
    for (std::size_t i = 0; i < status.status_codes.size(); ++i) {
      if (status.status_codes[i] == kUnknownStatusCode) {
        ++unavailable_count;
        if (unavailable.tellp() > 0) {
          unavailable << ", ";
        }
        const auto name = i < status.joint_names.size() ? status.joint_names[i] :
          ("joint_" + std::to_string(i));
        unavailable << name << "=0";
      }
    }
    if (unavailable_count == 0) {
      return {};
    }

    std::ostringstream message;
    if (unavailable_count == status.status_codes.size() && unavailable_count == 14) {
      message << "no EtherCAT drives detected; power enable rejected";
    } else {
      message << "not all 14 drives are available; power enable rejected";
    }
    message << "; unavailable_drives=[" << unavailable.str() << "]";
    return message.str();
  }

  std::string describe_status(const ArmPowerStatus & status, bool enable) const
  {
    std::size_t enabled_count = 0;
    std::ostringstream mismatches;
    for (std::size_t i = 0; i < status.enabled.size(); ++i) {
      if (status.enabled[i]) {
        ++enabled_count;
      }
      if (enable && i < status.joint_names.size() && i < status.status_codes.size() &&
        !status.enabled[i])
      {
        if (mismatches.tellp() > 0) {
          mismatches << ", ";
        }
        mismatches << status.joint_names[i] << "=" << status.status_codes[i];
      }
    }

    std::ostringstream description;
    description << "command_enabled=" << (status.command_enabled ? "true" : "false")
                << ", all_enabled=" << (status.all_enabled ? "true" : "false")
                << ", enabled=" << enabled_count << "/" << status.enabled.size();
    if (mismatches.tellp() > 0) {
      description << ", mismatched_status=[" << mismatches.str() << "]";
    }
    if (!status.message.empty()) {
      description << ", hardware_message='" << status.message << "'";
    }
    return description.str();
  }

  void publish_power_command(bool enable)
  {
    std_msgs::msg::Bool command;
    command.data = enable;
    poweron_publisher_->publish(command);
    RCLCPP_INFO(
      get_logger(), "Published %s command to /robot_poweron",
      enable ? "ENABLE" : "DISABLE");
  }

  std::string rollback_failed_enable()
  {
    std::uint64_t generation_before = 0;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      generation_before = status_generation_;
    }

    publish_power_command(false);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(rollback_timeout_ms_);
    std::unique_lock<std::mutex> lock(status_mutex_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      status_condition_.wait_until(lock, deadline, [this, generation_before]() {
        return status_generation_ > generation_before;
      });

      if (status_generation_ <= generation_before) {
        continue;
      }
      generation_before = status_generation_;
      std::string shape_error;
      if (status_shape_valid(last_power_status_, shape_error) &&
        target_reached(last_power_status_, false))
      {
        RCLCPP_WARN(
          get_logger(), "Enable failure rollback confirmed: %s",
          describe_status(last_power_status_, false).c_str());
        return "automatic disable rollback confirmed";
      }
    }

    RCLCPP_ERROR(
      get_logger(),
      "Enable failure rollback was published but not confirmed within %d ms",
      rollback_timeout_ms_);
    return "automatic disable rollback published but confirmation timed out";
  }

  void handle_service_request(
    const std::shared_ptr<SetRobotPower::Request> request,
    std::shared_ptr<SetRobotPower::Response> response)
  {
    RCLCPP_INFO(
      get_logger(), "Received robot power request: %s",
      request->enable ? "ENABLE" : "DISABLE");

    std::unique_ptr<robot_control_msg::safety::RobotCommandGuard> command_guard;
    if (request->enable) {
      command_guard = std::make_unique<robot_control_msg::safety::RobotCommandGuard>(
        "robot_power_enable");
      if (!command_guard->acquired()) {
        response->success = false;
        response->message = "Enable rejected: " + command_guard->error();
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
    }

    ArmPowerStatus status_before;
    std::uint64_t generation_before = 0;
    bool has_status = false;
    bool status_is_fresh = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (has_power_status_) {
        has_status = true;
        status_before = last_power_status_;
        generation_before = status_generation_;
        status_is_fresh =
          std::chrono::steady_clock::now() - last_status_receive_time_ <=
          std::chrono::milliseconds(status_stale_timeout_ms_);
      }
    }

    std::string shape_error;
    if (request->enable && (!has_status || !status_is_fresh)) {
      response->success = false;
      response->message = "Enable rejected: /arm/power_status is unavailable or stale";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (request->enable && !status_shape_valid(status_before, shape_error)) {
      response->success = false;
      response->message = "Enable rejected: " + shape_error;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (request->enable) {
      const auto unavailable_message = unavailable_drive_message(status_before);
      if (!unavailable_message.empty()) {
        response->success = false;
        response->message = unavailable_message + "; enabled=0/14";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
    }
    if (status_is_fresh && status_shape_valid(status_before, shape_error) &&
      target_reached(status_before, request->enable))
    {
      response->success = true;
      response->message = request->enable ?
        "Robot power already enabled: 14/14 motors enabled" :
        "Robot power already disabled: 0/14 motors enabled";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      return;
    }

    publish_power_command(request->enable);

    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(power_timeout_ms_);
    ArmPowerStatus latest_status = status_before;
    bool received_post_command_status = false;

    std::unique_lock<std::mutex> lock(status_mutex_);
    while (rclcpp::ok()) {
      status_condition_.wait_until(lock, deadline, [this, generation_before]() {
        return status_generation_ > generation_before;
      });

      if (status_generation_ > generation_before) {
        received_post_command_status = true;
        latest_status = last_power_status_;
        generation_before = status_generation_;

        std::string current_shape_error;
        if (request->enable) {
          const auto unavailable_message = unavailable_drive_message(latest_status);
          if (!unavailable_message.empty()) {
            lock.unlock();
            response->success = false;
            response->message = unavailable_message + "; enabled=" +
              std::to_string(std::count(latest_status.enabled.begin(), latest_status.enabled.end(), true)) +
              "/14";
            response->message += "; " + rollback_failed_enable();
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
          }
        }
        if (status_shape_valid(latest_status, current_shape_error) &&
          target_reached(latest_status, request->enable))
        {
          lock.unlock();
          response->success = true;
          response->message = request->enable ?
            "Robot power enabled: 14/14 motors report status 39" :
            "Robot power disabled: 0/14 motors enabled";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
          return;
        }
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
    lock.unlock();

    response->success = false;
    if (!received_post_command_status) {
      response->message = "Power command published but no post-command /arm/power_status was received";
    } else {
      response->message = "Power confirmation timed out: " + describe_status(latest_status, request->enable);
    }
    if (request->enable) {
      response->message += "; " + rollback_failed_enable();
    }
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  std::string power_status_topic_;
  int power_timeout_ms_{5000};
  int rollback_timeout_ms_{1500};
  int status_stale_timeout_ms_{1000};

  std::mutex status_mutex_;
  std::condition_variable status_condition_;
  ArmPowerStatus last_power_status_;
  bool has_power_status_{false};
  std::uint64_t status_generation_{0};
  std::chrono::steady_clock::time_point last_status_receive_time_{};

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr poweron_publisher_;
  rclcpp::Subscription<ArmPowerStatus>::SharedPtr power_status_subscription_;
  rclcpp::Service<SetRobotPower>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPowerServiceNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

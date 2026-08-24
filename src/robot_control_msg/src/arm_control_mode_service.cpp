#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_control_msg/msg/arm_control_mode_status.hpp>
#include <robot_control_msg/msg/arm_power_status.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
#include <robot_control_msg/srv/set_arm_control_mode.hpp>

using namespace std::chrono_literals;

class ArmControlModeServiceNode : public rclcpp::Node
{
public:
  using ModeStatus = robot_control_msg::msg::ArmControlModeStatus;
  using PowerStatus = robot_control_msg::msg::ArmPowerStatus;
  using SetMode = robot_control_msg::srv::SetArmControlMode;

  ArmControlModeServiceNode()
  : Node(
      "arm_control_mode_service",
      rclcpp::NodeOptions().start_parameter_services(false))
  {
    internal_service_name_ = declare_parameter<std::string>(
      "internal_service_name", "/arm/set_control_mode");
    mode_status_topic_ = declare_parameter<std::string>(
      "mode_status_topic", "/arm/control_mode_status");
    power_status_topic_ = declare_parameter<std::string>(
      "power_status_topic", "/arm/power_status");
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 5000);
    status_stale_timeout_ms_ = declare_parameter<int>("status_stale_timeout_ms", 1000);
    enable_effort_mode_switch_ = declare_parameter<bool>(
      "enable_effort_mode_switch", false);
    require_power_disabled_for_effort_switch_ = declare_parameter<bool>(
      "require_power_disabled_for_effort_switch", true);

    service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    status_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = status_callback_group_;
    mode_status_subscription_ = create_subscription<ModeStatus>(
      mode_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&ArmControlModeServiceNode::handle_mode_status, this, std::placeholders::_1),
      subscription_options);
    power_status_subscription_ = create_subscription<PowerStatus>(
      power_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&ArmControlModeServiceNode::handle_power_status, this, std::placeholders::_1),
      subscription_options);

    internal_client_ = create_client<SetMode>(
      internal_service_name_, rmw_qos_profile_services_default, client_callback_group_);
    external_service_ = create_service<SetMode>(
      "/set_arm_control_mode",
      std::bind(
        &ArmControlModeServiceNode::handle_request, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Arm control mode service ready; internal_service='%s', effort_switch=%s, "
      "require_power_disabled=%s",
      internal_service_name_.c_str(), enable_effort_mode_switch_ ? "true" : "false",
      require_power_disabled_for_effort_switch_ ? "true" : "false");
  }

private:
  void handle_mode_status(const ModeStatus::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_mode_status_ = *message;
      has_mode_status_ = true;
      ++mode_status_generation_;
      last_mode_status_receive_time_ = std::chrono::steady_clock::now();
    }
    status_condition_.notify_all();
  }

  void handle_power_status(const PowerStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_power_status_ = *message;
    has_power_status_ = true;
    last_power_status_receive_time_ = std::chrono::steady_clock::now();
  }

  bool is_fresh(
    const std::chrono::steady_clock::time_point & receive_time,
    const std::chrono::steady_clock::time_point & now) const
  {
    return now - receive_time <= std::chrono::milliseconds(status_stale_timeout_ms_);
  }

  bool power_is_safely_disabled(const PowerStatus & status, std::string & error) const
  {
    constexpr std::size_t kJointCount = 14;
    if (status.joint_names.size() != kJointCount ||
      status.status_codes.size() != kJointCount || status.enabled.size() != kJointCount)
    {
      error = "invalid /arm/power_status array sizes";
      return false;
    }

    if (status.command_enabled || status.all_enabled) {
      error = "EFFORT switch requires command_enabled=false and all_enabled=false";
      return false;
    }

    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (status.enabled[index] ||
        status.status_codes[index] != PowerStatus::DISABLED_STATUS)
      {
        error = "EFFORT switch requires all 14 motors to report disabled status 64";
        return false;
      }
    }
    return true;
  }

  static const char * mode_name(std::uint8_t mode)
  {
    return mode == SetMode::Request::EFFORT ? "EFFORT" : "POSITION";
  }

  void handle_request(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response)
  {
    std::unique_lock<std::mutex> command_lock(command_mutex_, std::try_to_lock);
    if (!command_lock.owns_lock()) {
      response->success = false;
      response->message = "another control mode command is already in progress";
      return;
    }

    if (request->mode != SetMode::Request::POSITION &&
      request->mode != SetMode::Request::EFFORT)
    {
      response->success = false;
      response->message = "invalid control mode; expected POSITION(0) or EFFORT(1)";
      return;
    }

    robot_control_msg::safety::RobotCommandGuard robot_command_guard(
      std::string("arm_control_mode_") + mode_name(request->mode));
    if (!robot_command_guard.acquired()) {
      response->success = false;
      response->message = "Control mode request rejected: " + robot_command_guard.error();
      return;
    }

    ModeStatus mode_before;
    std::uint64_t generation_before = 0;
    bool has_fresh_mode_status = false;
    PowerStatus power_before;
    bool has_fresh_power_status = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (has_mode_status_) {
        mode_before = last_mode_status_;
        generation_before = mode_status_generation_;
        has_fresh_mode_status = is_fresh(last_mode_status_receive_time_, now);
      }
      if (has_power_status_) {
        power_before = last_power_status_;
        has_fresh_power_status = is_fresh(last_power_status_receive_time_, now);
      }
    }

    response->active_mode = has_fresh_mode_status ?
      mode_before.active_mode : SetMode::Request::POSITION;

    if (request->mode == SetMode::Request::EFFORT) {
      const bool enable_effort_mode_switch =
        get_parameter("enable_effort_mode_switch").as_bool();
      const bool require_power_disabled_for_effort_switch =
        get_parameter("require_power_disabled_for_effort_switch").as_bool();

      if (!enable_effort_mode_switch) {
        response->success = false;
        response->message =
          "EFFORT mode switch is disabled; set enable_effort_mode_switch:=true explicitly";
        return;
      }

      if (!has_fresh_mode_status) {
        response->success = false;
        response->message = "EFFORT mode rejected: /arm/control_mode_status is unavailable or stale";
        return;
      }

      if (require_power_disabled_for_effort_switch) {
        std::string power_error;
        if (!has_fresh_power_status || !power_is_safely_disabled(power_before, power_error)) {
          response->success = false;
          response->message = !has_fresh_power_status ?
            "EFFORT mode rejected: /arm/power_status is unavailable or stale" :
            "EFFORT mode rejected: " + power_error;
          return;
        }
      }
    }

    if (has_fresh_mode_status && mode_before.active_mode == request->mode &&
      mode_before.requested_mode == request->mode)
    {
      response->success = true;
      response->active_mode = request->mode;
      response->message = std::string("Arm control mode already confirmed as ") +
        mode_name(request->mode);
      return;
    }

    if (!internal_client_->wait_for_service(500ms)) {
      response->success = false;
      response->message = "internal arm control mode service is unavailable";
      return;
    }

    auto internal_request = std::make_shared<SetMode::Request>();
    internal_request->mode = request->mode;
    auto future = internal_client_->async_send_request(internal_request);
    const auto timeout = std::chrono::milliseconds(command_timeout_ms_);
    if (future.wait_for(timeout) != std::future_status::ready) {
      response->success = false;
      response->message = "internal arm control mode service response timed out";
      return;
    }

    const auto internal_response = future.get();
    if (!internal_response->success) {
      response->success = false;
      response->active_mode = internal_response->active_mode;
      response->message = "internal arm control mode request rejected: " +
        internal_response->message;
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    ModeStatus latest_status = mode_before;
    std::unique_lock<std::mutex> status_lock(status_mutex_);
    while (rclcpp::ok()) {
      status_condition_.wait_until(status_lock, deadline, [this, generation_before]() {
        return mode_status_generation_ > generation_before;
      });

      if (mode_status_generation_ > generation_before) {
        latest_status = last_mode_status_;
        generation_before = mode_status_generation_;
        if (latest_status.active_mode == request->mode &&
          latest_status.requested_mode == request->mode)
        {
          status_lock.unlock();
          response->success = true;
          response->active_mode = latest_status.active_mode;
          response->message = std::string("Arm control mode confirmed as ") +
            mode_name(request->mode) + "; " + latest_status.message;
          return;
        }
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }

    response->success = false;
    response->active_mode = latest_status.active_mode;
    response->message = std::string("Control mode confirmation timed out; requested=") +
      mode_name(request->mode) + ", active=" + mode_name(latest_status.active_mode);
  }

  std::string internal_service_name_;
  std::string mode_status_topic_;
  std::string power_status_topic_;
  int command_timeout_ms_{5000};
  int status_stale_timeout_ms_{1000};
  bool enable_effort_mode_switch_{false};
  bool require_power_disabled_for_effort_switch_{true};

  std::mutex status_mutex_;
  std::condition_variable status_condition_;
  ModeStatus last_mode_status_;
  PowerStatus last_power_status_;
  bool has_mode_status_{false};
  bool has_power_status_{false};
  std::uint64_t mode_status_generation_{0};
  std::chrono::steady_clock::time_point last_mode_status_receive_time_{};
  std::chrono::steady_clock::time_point last_power_status_receive_time_{};
  std::mutex command_mutex_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Subscription<ModeStatus>::SharedPtr mode_status_subscription_;
  rclcpp::Subscription<PowerStatus>::SharedPtr power_status_subscription_;
  rclcpp::Client<SetMode>::SharedPtr internal_client_;
  rclcpp::Service<SetMode>::SharedPtr external_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControlModeServiceNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

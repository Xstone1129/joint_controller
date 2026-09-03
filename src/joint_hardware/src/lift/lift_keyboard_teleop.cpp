#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    if (!isatty(STDIN_FILENO)) {
      error_ = "stdin is not a terminal";
      return;
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      error_ = std::string("tcgetattr failed: ") + std::strerror(errno);
      return;
    }
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0) {
      error_ = std::string("F_GETFL failed: ") + std::strerror(errno);
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0 ||
      fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0)
    {
      error_ = std::string("failed to configure terminal: ") + std::strerror(errno);
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      return;
    }
    active_ = true;
  }

  ~TerminalRawMode()
  {
    if (active_) {
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      (void)fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    }
  }

  bool active() const noexcept {return active_;}
  const std::string & error() const noexcept {return error_;}

private:
  termios original_termios_{};
  int original_flags_{-1};
  bool active_{false};
  std::string error_;
};

enum class Direction : int
{
  down = -1,
  stopped = 0,
  up = 1,
};

class LiftKeyboardTeleop : public rclcpp::Node
{
public:
  LiftKeyboardTeleop()
  : Node("lift_keyboard_teleop")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "joint_motor");
    speed_rpm_ = declare_parameter<double>("speed_rpm", 30.0);
    lead_mm_per_rev_ = declare_parameter<double>("lead_mm_per_rev", 10.0 / 3.0);
    position_min_m_ = declare_parameter<double>("position_min_m", -1.0);
    position_max_m_ = declare_parameter<double>("position_max_m", 0.0);
    rate_hz_ = declare_parameter<double>("command_rate_hz", 50.0);
    // A terminal commonly waits several hundred milliseconds before it starts
    // repeating a held arrow key.  A shorter timeout turns a valid jog request
    // into a STOP before the brake gate can complete.
    release_timeout_ms_ = declare_parameter<int>("release_timeout_ms", 1000);
    arm_timeout_ms_ = declare_parameter<int>("arm_timeout_ms", 1000);
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/lift/joint_states");

    if (!std::isfinite(speed_rpm_) || speed_rpm_ <= 0.0 || speed_rpm_ > 1440.0) {
      throw std::invalid_argument("speed_rpm must be in (0, 1440]");
    }
    if (!std::isfinite(lead_mm_per_rev_) || lead_mm_per_rev_ <= 0.0) {
      throw std::invalid_argument("lead_mm_per_rev must be positive");
    }
    if (!std::isfinite(position_min_m_) || !std::isfinite(position_max_m_) ||
      position_min_m_ >= position_max_m_)
    {
      throw std::invalid_argument("position_min_m must be less than position_max_m");
    }
    if (!std::isfinite(rate_hz_) || rate_hz_ < 10.0 || rate_hz_ > 200.0) {
      throw std::invalid_argument("command_rate_hz must be in [10, 200]");
    }
    if (release_timeout_ms_ < 600 || arm_timeout_ms_ < release_timeout_ms_) {
      throw std::invalid_argument(
              "release_timeout_ms must be at least 600 and arm_timeout_ms must be no smaller");
    }

    speed_mps_ = speed_rpm_ * lead_mm_per_rev_ / 60000.0;
    jog_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/joint/lift/jog_velocity", rclcpp::QoS(1).best_effort());
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        for (std::size_t index = 0; index < message->name.size(); ++index) {
          if (message->name[index] == joint_name_ && index < message->position.size() &&
            std::isfinite(message->position[index]))
          {
            feedback_position_m_ = message->position[index];
            feedback_received_ = true;
            return;
          }
        }
      });
    stop_client_ = create_client<std_srvs::srv::Trigger>("/joint/lift/stop");
    brake_client_ = create_client<std_srvs::srv::SetBool>("/lift_brake_command");
    driver_status_subscription_ = create_subscription<std_msgs::msg::String>(
      "/joint/lift/driver_status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        if (message) { driver_status_ = message->data; }
      });

    RCLCPP_INFO(
      get_logger(),
      "Jog speed %.1f rpm (%.4f m/s), travel [%.3f, %.3f] m",
      speed_rpm_, speed_mps_, position_min_m_, position_max_m_);
    RCLCPP_INFO(get_logger(), "Joint feedback topic: %s", joint_state_topic_.c_str());
  }

  void process_input()
  {
    char bytes[64];
    while (true) {
      const ssize_t count = read(STDIN_FILENO, bytes, sizeof(bytes));
      if (count > 0) {
        for (ssize_t index = 0; index < count; ++index) {
          process_byte(bytes[index]);
        }
        continue;
      }
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "keyboard read failed: %s", std::strerror(errno));
        quit_requested_ = true;
      }
      break;
    }
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();
    if (jog_publisher_->get_subscription_count() == 0) {
      if (active_direction_ != Direction::stopped) {
        stop_and_hold("lift_controller subscriber unavailable");
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no /joint/lift/jog_velocity subscriber; keep the lift controller running");
      return;
    }
    if (!feedback_received_) {
      return;
    }

    if (!initial_hold_sent_) {
      publish_jog(0.0);
      initial_hold_sent_ = true;
      command_target_m_ = feedback_position_m_;
      last_update_ = now;
    }

    if (pending_direction_ != Direction::stopped &&
      now - pending_since_ > std::chrono::milliseconds(arm_timeout_ms_))
    {
      pending_direction_ = Direction::stopped;
    }

    if (active_direction_ != Direction::stopped &&
      now - last_key_event_ > std::chrono::milliseconds(release_timeout_ms_))
    {
      stop_and_hold("key released");
    }

    if (active_direction_ == Direction::stopped) {
      last_update_ = now;
      render_status();
      return;
    }

    const double elapsed_s = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;
    (void)elapsed_s;
    const double jog_velocity = static_cast<int>(active_direction_) * speed_mps_;
    publish_jog(jog_velocity);
    render_status();
    if ((active_direction_ == Direction::up && feedback_position_m_ >= position_max_m_) ||
      (active_direction_ == Direction::down && feedback_position_m_ <= position_min_m_))
    {
      stop_and_hold("software travel limit reached");
    }
  }

  void render_status()
  {
    std::cout << "\033[2J\033[H"
              << "Lift keyboard control  [e] enable  [d] disable  [UP/DOWN] jog  "
              << "[SPACE] stop  [q] quit\n"
              << "position: " << (feedback_received_ ? std::to_string(feedback_position_m_) : "--")
              << " m   command: " << (active_direction_ == Direction::up ? "UP" :
                active_direction_ == Direction::down ? "DOWN" : "STOP")
              << "   brake request: " << (brake_requested_ ? "ON" : "OFF") << "\n"
              << "driver: " << (driver_status_.empty() ? "waiting" : driver_status_)
              << std::endl;
  }

  bool quit_requested() const noexcept {return quit_requested_;}

  void shutdown_safely()
  {
    if (feedback_received_) {
      stop_and_hold("teleop exit");
    }
    request_soft_stop();
    if (brake_requested_) {
      set_brake(false);
    }
  }

private:
  void process_byte(char byte)
  {
    if (escape_state_ == 0) {
      if (byte == '\x1b') {
        escape_state_ = 1;
      } else if (byte == 'q' || byte == 'Q') {
        quit_requested_ = true;
      } else if (byte == ' ') {
        stop_and_hold("stop key");
      } else if (byte == 'e' || byte == 'E') {
        set_brake(true);
      } else if (byte == 'd' || byte == 'D') {
        set_brake(false);
      }
      return;
    }
    if (escape_state_ == 1) {
      escape_state_ = byte == '[' ? 2 : 0;
      return;
    }
    escape_state_ = 0;
    if (byte == 'A') {
      direction_event(Direction::up);
    } else if (byte == 'B') {
      direction_event(Direction::down);
    }
  }

  void direction_event(Direction direction)
  {
    if (!feedback_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for joint_motor feedback");
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (active_direction_ == direction) {
      last_key_event_ = now;
      return;
    }
    if (active_direction_ != Direction::stopped) {
      stop_and_hold("direction change");
      pending_direction_ = direction;
      pending_since_ = now;
      return;
    }
    if (pending_direction_ != Direction::stopped) {
      if (pending_direction_ != direction ||
        now - pending_since_ > std::chrono::milliseconds(arm_timeout_ms_))
      {
        pending_direction_ = direction;
        pending_since_ = now;
        return;
      }
    }
    active_direction_ = direction;
    pending_direction_ = Direction::stopped;
    last_key_event_ = now;
    last_update_ = now;
    command_target_m_ = feedback_position_m_;
    RCLCPP_INFO(
      get_logger(), "%s at %.1f rpm",
      direction == Direction::up ? "UP" : "DOWN", speed_rpm_);
  }

  void stop_and_hold(const char * reason)
  {
    const bool was_moving = active_direction_ != Direction::stopped;
    active_direction_ = Direction::stopped;
    pending_direction_ = Direction::stopped;
    if (feedback_received_) {
      command_target_m_ = std::clamp(feedback_position_m_, position_min_m_, position_max_m_);
      publish_jog(0.0);
    }
    if (was_moving) {
      RCLCPP_INFO(get_logger(), "STOP: %s, hold %.6f m", reason, command_target_m_);
    }
  }

  void publish_jog(double velocity_mps)
  {
    std_msgs::msg::Float64 command;
    command.data = velocity_mps;
    jog_publisher_->publish(command);
  }

  void request_soft_stop()
  {
    if (!stop_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "/joint/lift/stop is not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)stop_client_->async_send_request(request);
  }

  void set_brake(bool enable)
  {
    if (!brake_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "/lift_brake_command is not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    (void)brake_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (response && response->success) {
          brake_requested_ = enable;
        }
        RCLCPP_INFO(get_logger(), "%s: %s", enable ? "enable" : "disable",
          response ? response->message.c_str() : "no response");
      });
  }

  std::string joint_name_;
  double speed_rpm_{30.0};
  double lead_mm_per_rev_{10.0 / 3.0};
  double speed_mps_{0.005};
  double position_min_m_{-1.0};
  double position_max_m_{0.0};
  double rate_hz_{50.0};
  int release_timeout_ms_{150};
  int arm_timeout_ms_{1000};
  std::string joint_state_topic_{"/lift/joint_states"};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jog_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr brake_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr driver_status_subscription_;

  Direction active_direction_{Direction::stopped};
  Direction pending_direction_{Direction::stopped};
  std::chrono::steady_clock::time_point pending_since_{};
  std::chrono::steady_clock::time_point last_key_event_{};
  std::chrono::steady_clock::time_point last_update_{};
  double feedback_position_m_{0.0};
  double command_target_m_{0.0};
  bool feedback_received_{false};
  bool initial_hold_sent_{false};
  bool quit_requested_{false};
  bool brake_requested_{false};
  std::string driver_status_;
  int escape_state_{0};

public:
  std::chrono::duration<double> loop_period() const
  {
    return std::chrono::duration<double>(1.0 / rate_hz_);
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TerminalRawMode terminal;
  if (!terminal.active()) {
    std::cerr << "lift_keyboard_teleop: " << terminal.error() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  try {
    auto node = std::make_shared<LiftKeyboardTeleop>();
    std::cout << "Hold UP to move up; hold DOWN to move down; SPACE stops; Q stops and exits."
              << std::endl;
    const auto loop_period = node->loop_period();
    auto next_update = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !node->quit_requested()) {
      rclcpp::spin_some(node);
      node->process_input();
      node->update();
      next_update += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
      std::this_thread::sleep_until(next_update);
    }
    if (rclcpp::ok()) {
      node->shutdown_safely();
      for (int index = 0; index < 10; ++index) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  } catch (const std::exception & exception) {
    std::cerr << "lift_keyboard_teleop: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_control_msg/msg/planner_input.hpp"

using namespace std::chrono_literals;

class PlannerInputPublisherUbuntu : public rclcpp::Node
{
public:
  PlannerInputPublisherUbuntu()
  : Node("planner_input_publisher_ubuntu")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<robot_control_msg::msg::PlannerInput>(
      "/planner/input", qos);
    timer_ = create_wall_timer(10ms, [this]() {publish_planner_input();});
  }

private:
  void publish_planner_input()
  {
    robot_control_msg::msg::PlannerInput msg;
    msg.stamp = this->now();
    msg.seq = seq_++;

    msg.param1 = std::sin(msg.seq * 0.01);
    msg.param2 = std::cos(msg.seq * 0.01);
    msg.param3 = msg.seq * 0.001;
    msg.extra_params = {0.1, 0.2, 0.3, static_cast<double>(msg.seq)};
    msg.frame_id = "base_link";
    msg.source = "ubuntu_192_168_2_20";

    publisher_->publish(msg);

    ++published_count_;
    if (published_count_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(), "Published seq=%u, param1=%.6f, param2=%.6f, param3=%.6f",
        msg.seq, msg.param1, msg.param2, msg.param3);
    }
  }

  std::uint32_t seq_{0};
  std::uint32_t published_count_{0};
  rclcpp::Publisher<robot_control_msg::msg::PlannerInput>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerInputPublisherUbuntu>());
  rclcpp::shutdown();
  return 0;
}

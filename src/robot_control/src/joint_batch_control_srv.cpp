#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_control_msg/msg/arm_motion_status.hpp>
#include <robot_control_msg/msg/joint_batch_execution_status.hpp>
#include <robot_control_msg/msg/workspace_status.hpp>
#include <robot_control_msg/srv/joint_absolute_control.hpp>
#include <robot_control_msg/srv/joint_batch_control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace robot_control
{

class JointBatchControlSrv : public rclcpp::Node
{
public:
  using Batch = robot_control_msg::srv::JointBatchControl;
  using Absolute = robot_control_msg::srv::JointAbsoluteControl;
  using Status = robot_control_msg::msg::JointBatchExecutionStatus;

  JointBatchControlSrv()
  : Node("joint_batch_control_srv", rclcpp::NodeOptions().start_parameter_services(false))
  {
    urdf_path_ = declare_parameter<std::string>("urdf_path", "");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/arm/joint_states");
    motion_status_topic_ = declare_parameter<std::string>(
      "motion_status_topic", "/arm/arm_controller/motion_status");
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 500);
    workspace_timeout_ms_ = declare_parameter<int>("workspace_timeout_ms", 3000);
    forward_timeout_ms_ = declare_parameter<int>("forward_timeout_ms", 20000);
    max_velocity_rad_s_ = declare_parameter<double>("max_velocity_rad_s", 0.50);
    max_acceleration_rad_s2_ = declare_parameter<double>("max_acceleration_rad_s2", 1.0);
    max_single_joint_delta_rad_ = declare_parameter<double>("max_single_joint_delta_rad", 0.50);
    noop_joint_tolerance_rad_ = declare_parameter<double>("noop_joint_tolerance_rad", 1e-4);

    if (!loadModel()) {
      throw std::runtime_error("Failed to initialize joint batch control model");
    }

    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    feedback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions feedback_options;
    feedback_options.callback_group = feedback_group_;

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&JointBatchControlSrv::handleJointState, this, std::placeholders::_1),
      feedback_options);
    motion_status_sub_ = create_subscription<robot_control_msg::msg::ArmMotionStatus>(
      motion_status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&JointBatchControlSrv::handleMotionStatus, this, std::placeholders::_1),
      feedback_options);

    rclcpp::QoS workspace_qos(rclcpp::KeepLast(1));
    workspace_qos.reliable();
    workspace_qos.transient_local();
    workspace_status_sub_ = create_subscription<robot_control_msg::msg::WorkspaceStatus>(
      "/workspace/status", workspace_qos,
      std::bind(&JointBatchControlSrv::handleWorkspaceStatus, this, std::placeholders::_1),
      feedback_options);

    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = create_publisher<Status>("/arm/joint_batch_execution_status", status_qos);

    absolute_client_ = create_client<Absolute>("/arm_absolute_control");
    batch_service_ = create_service<Batch>(
      "/arm/joint_batch_control",
      std::bind(&JointBatchControlSrv::handleBatchRequest, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

    RCLCPP_INFO(
      get_logger(),
      "Joint batch control ready: max_vel=%.3f rad/s max_acc=%.3f rad/s^2 max_delta=%.3f rad",
      max_velocity_rad_s_, max_acceleration_rad_s2_, max_single_joint_delta_rad_);
  }

private:
  class ActiveCommandReset
  {
  public:
    explicit ActiveCommandReset(std::atomic_bool& active) : active_(active) {}
    ~ActiveCommandReset() { active_.store(false); }

  private:
    std::atomic_bool& active_;
  };

  static const std::array<std::string, 14>& canonicalJointNames()
  {
    static const std::array<std::string, 14> names {
      "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
      "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
    return names;
  }

  bool loadModel()
  {
    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
      q_current_.setZero(model_.nq);
      for (std::size_t i = 0; i < canonicalJointNames().size(); ++i) {
        const auto joint_id = model_.getJointId(canonicalJointNames()[i]);
        if (joint_id == 0 || model_.joints[joint_id].nq() != 1) {
          RCLCPP_ERROR(get_logger(), "Required one-DoF joint '%s' is missing", canonicalJointNames()[i].c_str());
          return false;
        }
        const int q_index = model_.joints[joint_id].idx_q();
        canonical_to_model_index_[canonicalJointNames()[i]] = q_index;
        canonical_model_indices_[i] = q_index;
      }
      return true;
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "URDF load failed: %s", error.what());
      return false;
    }
  }

  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::array<double, 14> snapshot {};
    std::array<bool, 14> seen {};
    std::size_t received = 0;
    for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
      const auto canonical_it = std::find(
        canonicalJointNames().begin(), canonicalJointNames().end(), message->name[i]);
      if (canonical_it == canonicalJointNames().end() || !std::isfinite(message->position[i])) {
        continue;
      }
      const auto canonical_index = static_cast<std::size_t>(
        std::distance(canonicalJointNames().begin(), canonical_it));
      if (seen[canonical_index]) {
        continue;
      }
      snapshot[canonical_index] = message->position[i];
      seen[canonical_index] = true;
      ++received;
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    latest_joint_state_complete_ = received == canonicalJointNames().size();
    if (!latest_joint_state_complete_) {
      return;
    }
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
      q_current_[canonical_model_indices_[i]] = snapshot[i];
    }
    last_joint_state_time_ = std::chrono::steady_clock::now();
  }

  void handleMotionStatus(const robot_control_msg::msg::ArmMotionStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_motion_status_ = *message;
    has_motion_status_ = true;
    last_motion_status_time_ = std::chrono::steady_clock::now();
  }

  void handleWorkspaceStatus(const robot_control_msg::msg::WorkspaceStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_workspace_status_ = *message;
    has_workspace_status_ = true;
    last_workspace_status_time_ = std::chrono::steady_clock::now();
  }

  bool snapshotCurrent(
    std::array<double, 14>& current,
    std::string& error) const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!has_workspace_status_ ||
      now - last_workspace_status_time_ > std::chrono::milliseconds(workspace_timeout_ms_))
    {
      error = "workspace status is unavailable or stale";
      return false;
    }
    if (!last_workspace_status_.accepted ||
      last_workspace_status_.state != robot_control_msg::msg::WorkspaceStatus::RUNNING)
    {
      error = "workspace must be RUNNING";
      return false;
    }
    if (!latest_joint_state_complete_ ||
      now - last_joint_state_time_ > std::chrono::milliseconds(feedback_timeout_ms_))
    {
      error = "/arm/joint_states is incomplete or stale";
      return false;
    }
    if (!has_motion_status_ ||
      now - last_motion_status_time_ > std::chrono::milliseconds(feedback_timeout_ms_))
    {
      error = "motion status is unavailable or stale";
      return false;
    }
    if (last_motion_status_.is_moving) {
      error = "arm is already moving";
      return false;
    }
    for (std::size_t i = 0; i < current.size(); ++i) {
      current[i] = q_current_[canonical_model_indices_[i]];
    }
    return true;
  }

  bool buildTarget(
    const Batch::Request& request,
    const std::array<double, 14>& current,
    std::array<double, 14>& target,
    std::string& error) const
  {
    if (request.joint_names.empty() || request.joint_names.size() > canonicalJointNames().size() ||
      request.joint_names.size() != request.values.size())
    {
      error = "joint_names and values must have the same size between 1 and 14";
      return false;
    }
    if (request.source_mode != Batch::Request::MANUAL &&
      request.source_mode != Batch::Request::TRAJECTORY)
    {
      error = "source_mode must be MANUAL or TRAJECTORY";
      return false;
    }
    if (!std::isfinite(request.vel) || request.vel <= 0.0 || request.vel > max_velocity_rad_s_) {
      error = "vel must be finite, > 0, and <= " + std::to_string(max_velocity_rad_s_);
      return false;
    }
    if (!std::isfinite(request.acc) || request.acc <= 0.0 || request.acc > max_acceleration_rad_s2_) {
      error = "acc must be finite, > 0, and <= " + std::to_string(max_acceleration_rad_s2_);
      return false;
    }

    target = current;
    std::unordered_set<std::string> seen;
    for (std::size_t i = 0; i < request.joint_names.size(); ++i) {
      const auto& name = request.joint_names[i];
      const auto index_it = canonical_to_model_index_.find(name);
      if (index_it == canonical_to_model_index_.end()) {
        error = "unknown joint '" + name + "'";
        return false;
      }
      if (!seen.insert(name).second) {
        error = "duplicate joint '" + name + "'";
        return false;
      }
      if (!std::isfinite(request.values[i])) {
        error = "joint value must be finite";
        return false;
      }
      const auto canonical_it = std::find(
        canonicalJointNames().begin(), canonicalJointNames().end(), name);
      const std::size_t canonical_index = static_cast<std::size_t>(
        std::distance(canonicalJointNames().begin(), canonical_it));
      target[canonical_index] = request.relative ?
        current[canonical_index] + request.values[i] : request.values[i];
    }

    for (std::size_t i = 0; i < target.size(); ++i) {
      const int model_index = canonical_model_indices_[i];
      const double lower = model_.lowerPositionLimit[model_index];
      const double upper = model_.upperPositionLimit[model_index];
      if (!std::isfinite(target[i]) || target[i] < lower || target[i] > upper) {
        error = "target for " + canonicalJointNames()[i] + " is outside URDF position limits";
        return false;
      }
      if (std::abs(target[i] - current[i]) > max_single_joint_delta_rad_) {
        error = "target delta for " + canonicalJointNames()[i] + " exceeds " +
          std::to_string(max_single_joint_delta_rad_) + " rad";
        return false;
      }
    }
    return true;
  }

  static Absolute::Request makeAbsoluteRequest(
    const std::array<double, 14>& target,
    double velocity,
    double acceleration)
  {
    Absolute::Request request;
    request.ljoint1 = target[0]; request.ljoint2 = target[1]; request.ljoint3 = target[2];
    request.ljoint4 = target[3]; request.ljoint5 = target[4]; request.ljoint6 = target[5];
    request.ljoint7 = target[6]; request.rjoint1 = target[7]; request.rjoint2 = target[8];
    request.rjoint3 = target[9]; request.rjoint4 = target[10]; request.rjoint5 = target[11];
    request.rjoint6 = target[12]; request.rjoint7 = target[13];
    request.vel = velocity;
    request.acc = acceleration;
    return request;
  }

  void publishStatus(
    uint8_t state,
    bool accepted,
    uint32_t command_seq,
    uint8_t source_mode,
    const std::array<double, 14>& target,
    const std::string& message)
  {
    Status status;
    status.stamp = now();
    status.command_seq = command_seq;
    status.state = state;
    status.source_mode = source_mode;
    status.accepted = accepted;
    status.target_joints = target;
    status.message = message;
    status_pub_->publish(status);
  }

  void reject(
    const Batch::Request& request,
    const std::array<double, 14>& target,
    const std::string& message,
    Batch::Response& response)
  {
    response.accepted = false;
    response.command_seq = request.command_seq;
    response.target_joints = target;
    response.message = message;
    publishStatus(Status::REJECTED, false, request.command_seq, request.source_mode, target, message);
  }

  void handleBatchRequest(
    const std::shared_ptr<Batch::Request> request,
    std::shared_ptr<Batch::Response> response)
  {
    std::array<double, 14> current {};
    std::array<double, 14> target {};
    bool expected = false;
    if (!command_in_progress_.compare_exchange_strong(expected, true)) {
      reject(*request, target, "joint batch command rejected: another batch command is in progress", *response);
      return;
    }
    ActiveCommandReset reset(command_in_progress_);

    std::string error;
    if (!snapshotCurrent(current, error)) {
      reject(*request, current, "joint batch command rejected: " + error, *response);
      return;
    }
    if (!buildTarget(*request, current, target, error)) {
      reject(*request, target, "joint batch command rejected: " + error, *response);
      return;
    }

    const bool already_at_target = std::all_of(
      target.begin(), target.end(), [this, &current, index = std::size_t {0}](double value) mutable {
        return std::abs(value - current[index++]) <= noop_joint_tolerance_rad_;
      });
    if (already_at_target) {
      response->accepted = true;
      response->command_seq = request->command_seq;
      response->target_joints = target;
      response->message = "joint batch command already at target";
      publishStatus(
        Status::SUCCEEDED, true, request->command_seq, request->source_mode, target, response->message);
      return;
    }

    if (!absolute_client_->wait_for_service(2s)) {
      reject(*request, target, "joint batch command rejected: /arm_absolute_control is unavailable", *response);
      return;
    }

    publishStatus(
      Status::EXECUTING, true, request->command_seq, request->source_mode, target,
      "joint batch command forwarded to /arm_absolute_control");
    auto future = absolute_client_->async_send_request(
      std::make_shared<Absolute::Request>(makeAbsoluteRequest(target, request->vel, request->acc)));
    if (future.wait_for(std::chrono::milliseconds(forward_timeout_ms_)) != std::future_status::ready) {
      reject(*request, target, "joint batch command failed: timeout waiting for /arm_absolute_control", *response);
      return;
    }

    const auto lower_response = future.get();
    response->accepted = lower_response->success;
    response->command_seq = request->command_seq;
    response->target_joints = target;
    response->message = lower_response->message;
    publishStatus(
      lower_response->success ? Status::SUCCEEDED : Status::FAILED,
      lower_response->success, request->command_seq, request->source_mode, target, response->message);
  }

  std::string urdf_path_;
  std::string joint_state_topic_;
  std::string motion_status_topic_;
  int feedback_timeout_ms_ {500};
  int workspace_timeout_ms_ {3000};
  int forward_timeout_ms_ {20000};
  double max_velocity_rad_s_ {0.50};
  double max_acceleration_rad_s2_ {1.0};
  double max_single_joint_delta_rad_ {0.50};
  double noop_joint_tolerance_rad_ {1e-4};

  pinocchio::Model model_;
  Eigen::VectorXd q_current_;
  std::array<int, 14> canonical_model_indices_ {};
  std::unordered_map<std::string, int> canonical_to_model_index_;
  bool latest_joint_state_complete_ {false};

  mutable std::mutex feedback_mutex_;
  robot_control_msg::msg::ArmMotionStatus last_motion_status_;
  robot_control_msg::msg::WorkspaceStatus last_workspace_status_;
  bool has_motion_status_ {false};
  bool has_workspace_status_ {false};
  std::chrono::steady_clock::time_point last_joint_state_time_ {};
  std::chrono::steady_clock::time_point last_motion_status_time_ {};
  std::chrono::steady_clock::time_point last_workspace_status_time_ {};
  std::atomic_bool command_in_progress_ {false};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<robot_control_msg::msg::WorkspaceStatus>::SharedPtr workspace_status_sub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Client<Absolute>::SharedPtr absolute_client_;
  rclcpp::Service<Batch>::SharedPtr batch_service_;
};

}  // namespace robot_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_control::JointBatchControlSrv>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_control_msg/msg/arm_control_mode_status.hpp>
#include <robot_control_msg/msg/arm_motion_status.hpp>
#include <robot_control_msg/msg/arm_power_status.hpp>
#include <robot_control_msg/msg/cartesian_execution_status.hpp>
#include <robot_control_msg/msg/robotarmmovel.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
#include <robot_control_msg/srv/cartesian_absolute_control.hpp>
#include <robot_control_msg/srv/cartesian_increment_control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace robot_control
{

class CartesianSingleControlSrv : public rclcpp::Node
{
public:
  using Absolute = robot_control_msg::srv::CartesianAbsoluteControl;
  using Increment = robot_control_msg::srv::CartesianIncrementControl;
  using ExecutionStatus = robot_control_msg::msg::CartesianExecutionStatus;
  using ModeStatus = robot_control_msg::msg::ArmControlModeStatus;
  using PowerStatus = robot_control_msg::msg::ArmPowerStatus;

  CartesianSingleControlSrv()
  : Node(
      "cartesian_single_control_srv",
      rclcpp::NodeOptions().start_parameter_services(false))
  {
    urdf_path_ = declare_parameter<std::string>("urdf_path", "");
    ee_frame_left_ = declare_parameter<std::string>("ee_frame_left", "lee_link");
    ee_frame_right_ = declare_parameter<std::string>("ee_frame_right", "ree_link");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/arm/joint_states");
    motion_status_topic_ = declare_parameter<std::string>(
      "motion_status_topic", "/arm/arm_controller/motion_status");

    feedback_stale_timeout_ms_ = declare_parameter<int>("feedback_stale_timeout_ms", 1000);
    feedback_wait_timeout_ms_ = declare_parameter<int>("feedback_wait_timeout_ms", 5000);
    acceptance_timeout_ms_ = declare_parameter<int>("acceptance_timeout_ms", 6000);
    minimum_completion_timeout_sec_ = declare_parameter<double>(
      "minimum_completion_timeout_sec", 6.0);
    maximum_completion_timeout_sec_ = declare_parameter<double>(
      "maximum_completion_timeout_sec", 60.0);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.002);
    orientation_tolerance_rad_ = declare_parameter<double>(
      "orientation_tolerance_rad", 0.03);
    command_position_deadband_m_ = declare_parameter<double>(
      "command_position_deadband_m", 0.0001);
    command_orientation_deadband_rad_ = declare_parameter<double>(
      "command_orientation_deadband_rad", 0.001);
    max_absolute_translation_delta_m_ = declare_parameter<double>(
      "max_absolute_translation_delta_m", 0.20);
    max_absolute_rotation_delta_rad_ = declare_parameter<double>(
      "max_absolute_rotation_delta_rad", 0.75);
    max_increment_translation_m_ = declare_parameter<double>(
      "max_increment_translation_m", 0.10);
    max_increment_rotation_rad_ = declare_parameter<double>(
      "max_increment_rotation_rad", 0.35);
    max_velocity_mps_ = declare_parameter<double>("max_velocity_mps", 0.50);
    max_acceleration_mps2_ = declare_parameter<double>("max_acceleration_mps2", 1.0);
    quaternion_norm_tolerance_ = declare_parameter<double>(
      "quaternion_norm_tolerance", 0.05);

    tcp_transform_left_ = readTcpTransform("tcp_left");
    tcp_transform_right_ = readTcpTransform("tcp_right");
    if (!loadModel()) {
      throw std::runtime_error("Failed to initialize Cartesian single control model");
    }

    service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    feedback_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = feedback_callback_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CartesianSingleControlSrv::handleJointState, this, std::placeholders::_1),
      options);
    motion_status_subscription_ = create_subscription<robot_control_msg::msg::ArmMotionStatus>(
      motion_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&CartesianSingleControlSrv::handleMotionStatus, this, std::placeholders::_1),
      options);
    execution_status_subscription_ = create_subscription<ExecutionStatus>(
      "/arm_cartesian_path_execution_status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&CartesianSingleControlSrv::handleExecutionStatus, this, std::placeholders::_1),
      options);
    power_status_subscription_ = create_subscription<PowerStatus>(
      "/arm/power_status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&CartesianSingleControlSrv::handlePowerStatus, this, std::placeholders::_1),
      options);
    mode_status_subscription_ = create_subscription<ModeStatus>(
      "/arm/control_mode_status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&CartesianSingleControlSrv::handleModeStatus, this, std::placeholders::_1),
      options);

    command_publisher_ = create_publisher<robot_control_msg::msg::Robotarmmovel>(
      "/arm_cartrsian_position_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    rclcpp::QoS execution_status_qos(rclcpp::KeepLast(1));
    execution_status_qos.reliable();
    execution_status_qos.transient_local();
    execution_status_publisher_ = create_publisher<ExecutionStatus>(
      "/arm_cartesian_path_execution_status", execution_status_qos);

    absolute_service_ = create_service<Absolute>(
      "/cartesian_absolute_control",
      std::bind(
        &CartesianSingleControlSrv::handleAbsoluteRequest, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);
    increment_service_ = create_service<Increment>(
      "/cartesian_increment_control",
      std::bind(
        &CartesianSingleControlSrv::handleIncrementRequest, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Cartesian single control ready; max_increment=%.3fm/%.3frad, "
      "max_absolute_delta=%.3fm/%.3frad, max_vel=%.3fm/s, max_acc=%.3fm/s^2",
      max_increment_translation_m_, max_increment_rotation_rad_,
      max_absolute_translation_delta_m_, max_absolute_rotation_delta_rad_,
      max_velocity_mps_, max_acceleration_mps2_);
  }

private:
  struct Pose
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  };

  struct FeedbackSnapshot
  {
    Eigen::VectorXd q;
    robot_control_msg::msg::ArmMotionStatus motion;
    ExecutionStatus execution;
    PowerStatus power;
    ModeStatus mode;
    bool joint_complete{false};
    bool has_motion{false};
    bool has_execution{false};
    bool has_power{false};
    bool has_mode{false};
    std::uint64_t execution_generation{0};
    std::chrono::steady_clock::time_point joint_time{};
    std::chrono::steady_clock::time_point motion_time{};
    std::chrono::steady_clock::time_point power_time{};
    std::chrono::steady_clock::time_point mode_time{};
  };

  pinocchio::SE3 readTcpTransform(const std::string & prefix)
  {
    const double x = declare_parameter<double>(prefix + "_x", 0.0);
    const double y = declare_parameter<double>(prefix + "_y", 0.0);
    const double z = declare_parameter<double>(prefix + "_z", 0.0);
    const double roll = declare_parameter<double>(prefix + "_roll", 0.0);
    const double pitch = declare_parameter<double>(prefix + "_pitch", 0.0);
    const double yaw = declare_parameter<double>(prefix + "_yaw", 0.0);
    return pinocchio::SE3(rpyToRotation(roll, pitch, yaw), Eigen::Vector3d(x, y, z));
  }

  bool loadModel()
  {
    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
      data_ = pinocchio::Data(model_);
      q_current_ = pinocchio::neutral(model_);
      ee_id_left_ = model_.getFrameId(ee_frame_left_);
      ee_id_right_ = model_.getFrameId(ee_frame_right_);
      if (ee_id_left_ >= model_.frames.size() || ee_id_right_ >= model_.frames.size()) {
        RCLCPP_ERROR(get_logger(), "End-effector frame is missing from URDF");
        return false;
      }

      const std::array<std::string, 14> names = {
        "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7",
        "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
      for (const auto & name : names) {
        const auto joint_id = model_.getJointId(name);
        if (joint_id == 0 || model_.joints[joint_id].nq() != 1) {
          RCLCPP_ERROR(get_logger(), "Required one-DoF joint '%s' is missing", name.c_str());
          return false;
        }
        joint_q_index_[name] = model_.joints[joint_id].idx_q();
        required_joint_names_.insert(name);
      }
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "URDF load failed: %s", error.what());
      return false;
    }
  }

  static Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw)
  {
    return (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  static double rotationDistance(
    const Eigen::Matrix3d & current, const Eigen::Matrix3d & target)
  {
    return std::abs(Eigen::AngleAxisd(current.transpose() * target).angle());
  }

  bool parseOrientation(
    double roll, double pitch, double yaw,
    double qx, double qy, double qz, double qw,
    const std::string & label,
    Eigen::Matrix3d & rotation,
    std::string & error) const
  {
    const double quaternion_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (quaternion_norm > 1e-8) {
      if (std::abs(quaternion_norm - 1.0) > quaternion_norm_tolerance_) {
        std::ostringstream message;
        message << label << " quaternion norm " << quaternion_norm
                << " is outside 1+/-" << quaternion_norm_tolerance_;
        error = message.str();
        return false;
      }
      Eigen::Quaterniond quaternion(qw, qx, qy, qz);
      quaternion.normalize();
      rotation = quaternion.toRotationMatrix();
      return true;
    }

    rotation = rpyToRotation(roll, pitch, yaw);
    return true;
  }

  template<typename RequestT>
  bool validateNumerics(const RequestT & request, std::string & error) const
  {
    const std::array<double, 22> values = {
      request.lx, request.ly, request.lz,
      request.lroll, request.lpitch, request.lyaw,
      request.lqx, request.lqy, request.lqz, request.lqw,
      request.rx, request.ry, request.rz,
      request.rroll, request.rpitch, request.ryaw,
      request.rqx, request.rqy, request.rqz, request.rqw,
      request.vel, request.acc};
    if (!std::all_of(
        values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      error = "request contains NaN or Inf";
      return false;
    }
    if (request.vel <= 0.0 || request.vel > max_velocity_mps_) {
      error = "vel must be > 0 and <= " + std::to_string(max_velocity_mps_) + " m/s";
      return false;
    }
    if (request.acc <= 0.0 || request.acc > max_acceleration_mps2_) {
      error = "acc must be > 0 and <= " + std::to_string(max_acceleration_mps2_) + " m/s^2";
      return false;
    }
    return true;
  }

  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (std::size_t index = 0;
      index < message->name.size() && index < message->position.size(); ++index)
    {
      const auto mapping = joint_q_index_.find(message->name[index]);
      if (mapping != joint_q_index_.end() && std::isfinite(message->position[index])) {
        q_current_[mapping->second] = message->position[index];
        received_joint_names_.insert(message->name[index]);
      }
    }
    joint_state_time_ = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
  }

  void handleMotionStatus(
    const robot_control_msg::msg::ArmMotionStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_motion_status_ = *message;
    has_motion_status_ = true;
    motion_status_time_ = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
  }

  void handleExecutionStatus(const ExecutionStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_execution_status_ = *message;
    has_execution_status_ = true;
    ++execution_status_generation_;
    feedback_condition_.notify_all();
  }

  void handlePowerStatus(const PowerStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_power_status_ = *message;
    has_power_status_ = true;
    power_status_time_ = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
  }

  void handleModeStatus(const ModeStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_mode_status_ = *message;
    has_mode_status_ = true;
    mode_status_time_ = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
  }

  FeedbackSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    FeedbackSnapshot result;
    result.q = q_current_;
    result.motion = last_motion_status_;
    result.execution = last_execution_status_;
    result.power = last_power_status_;
    result.mode = last_mode_status_;
    result.joint_complete = received_joint_names_.size() == required_joint_names_.size();
    result.has_motion = has_motion_status_;
    result.has_execution = has_execution_status_;
    result.has_power = has_power_status_;
    result.has_mode = has_mode_status_;
    result.execution_generation = execution_status_generation_;
    result.joint_time = joint_state_time_;
    result.motion_time = motion_status_time_;
    result.power_time = power_status_time_;
    result.mode_time = mode_status_time_;
    return result;
  }

  bool isFresh(
    const std::chrono::steady_clock::time_point & timestamp,
    const std::chrono::steady_clock::time_point & now) const
  {
    return now - timestamp <= std::chrono::milliseconds(feedback_stale_timeout_ms_);
  }

  bool feedbackAvailableAndFresh(const FeedbackSnapshot & feedback, std::string & error) const
  {
    const auto now = std::chrono::steady_clock::now();
    if (!feedback.joint_complete || !isFresh(feedback.joint_time, now)) {
      error = "/arm/joint_states is incomplete or stale";
      return false;
    }
    if (!feedback.has_motion || !isFresh(feedback.motion_time, now)) {
      error = "motion_status is unavailable or stale";
      return false;
    }
    if (!feedback.has_execution) {
      error = "cartesian execution status is unavailable";
      return false;
    }
    if (!feedback.has_power || !isFresh(feedback.power_time, now)) {
      error = "/arm/power_status is unavailable or stale";
      return false;
    }
    if (!feedback.has_mode || !isFresh(feedback.mode_time, now)) {
      error = "/arm/control_mode_status is unavailable or stale";
      return false;
    }
    return true;
  }

  bool validateRobotState(const FeedbackSnapshot & feedback, std::string & error) const
  {
    if (feedback.power.joint_names.size() != 14 ||
      feedback.power.status_codes.size() != 14 || feedback.power.enabled.size() != 14)
    {
      error = "invalid /arm/power_status array sizes";
      return false;
    }
    if (!feedback.power.command_enabled || !feedback.power.all_enabled) {
      error = "Cartesian motion requires all 14 motors enabled";
      return false;
    }
    for (std::size_t index = 0; index < 14; ++index) {
      if (!feedback.power.enabled[index] ||
        feedback.power.status_codes[index] != PowerStatus::ENABLED_STATUS)
      {
        error = "Cartesian motion requires every motor to report enabled status 39";
        return false;
      }
    }
    if (feedback.mode.active_mode != ModeStatus::POSITION) {
      error = "Cartesian motion requires POSITION control mode";
      return false;
    }
    if (feedback.execution.state == ExecutionStatus::PLANNING ||
      feedback.execution.state == ExecutionStatus::EXECUTING)
    {
      error = "Cartesian planner is busy";
      return false;
    }
    return true;
  }

  bool waitForReadyFeedback(FeedbackSnapshot & feedback, std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(feedback_wait_timeout_ms_);
    while (rclcpp::ok()) {
      feedback = snapshot();
      if (feedbackAvailableAndFresh(feedback, error)) {
        return validateRobotState(feedback, error);
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::unique_lock<std::mutex> lock(feedback_mutex_);
      feedback_condition_.wait_for(lock, 20ms);
    }
    error = "interrupted while waiting for Cartesian feedback";
    return false;
  }

  std::pair<Pose, Pose> currentPoses(const Eigen::VectorXd & q)
  {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3 left = data_.oMf[ee_id_left_] * tcp_transform_left_;
    const pinocchio::SE3 right = data_.oMf[ee_id_right_] * tcp_transform_right_;
    return {
      Pose{left.translation(), left.rotation()},
      Pose{right.translation(), right.rotation()}};
  }

  bool targetReached(
    const FeedbackSnapshot & feedback,
    const Pose & left_target,
    const Pose & right_target,
    double position_tolerance,
    double orientation_tolerance,
    std::string * detail = nullptr)
  {
    const auto current = currentPoses(feedback.q);
    const double left_position_error =
      (current.first.position - left_target.position).norm();
    const double right_position_error =
      (current.second.position - right_target.position).norm();
    const double left_rotation_error =
      rotationDistance(current.first.rotation, left_target.rotation);
    const double right_rotation_error =
      rotationDistance(current.second.rotation, right_target.rotation);
    if (detail != nullptr) {
      std::ostringstream message;
      message << "left_pos_error=" << left_position_error
              << "m left_rot_error=" << left_rotation_error
              << "rad right_pos_error=" << right_position_error
              << "m right_rot_error=" << right_rotation_error << "rad";
      *detail = message.str();
    }
    return left_position_error <= position_tolerance &&
           right_position_error <= position_tolerance &&
           left_rotation_error <= orientation_tolerance &&
           right_rotation_error <= orientation_tolerance;
  }

  bool waitForAcceptance(
    std::uint64_t generation_before,
    FeedbackSnapshot & accepted_feedback,
    std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(acceptance_timeout_ms_);
    while (rclcpp::ok()) {
      accepted_feedback = snapshot();
      if (accepted_feedback.execution_generation > generation_before) {
        switch (accepted_feedback.execution.state) {
          case ExecutionStatus::EXECUTING:
          case ExecutionStatus::STREAM_FINISHED:
            return true;
          case ExecutionStatus::FAILED:
          case ExecutionStatus::REJECTED_BUSY:
            error = accepted_feedback.execution.message.empty() ?
              "Cartesian planner rejected the command" : accepted_feedback.execution.message;
            return false;
          default:
            break;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        error = "Timed out waiting for Cartesian planner acceptance";
        return false;
      }
      std::unique_lock<std::mutex> lock(feedback_mutex_);
      feedback_condition_.wait_for(lock, 20ms);
    }
    error = "Interrupted while waiting for Cartesian planner acceptance";
    return false;
  }

  bool waitForCompletion(
    const Pose & left_target,
    const Pose & right_target,
    const FeedbackSnapshot & accepted_feedback,
    std::string & detail)
  {
    double timeout_seconds = minimum_completion_timeout_sec_;
    if (accepted_feedback.execution.planned_duration_sec > 0.0) {
      timeout_seconds = std::max(
        timeout_seconds, accepted_feedback.execution.planned_duration_sec * 1.5 + 2.0);
    }
    timeout_seconds = std::min(timeout_seconds, maximum_completion_timeout_sec_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int64_t>(timeout_seconds * 1000.0));

    while (rclcpp::ok()) {
      const FeedbackSnapshot feedback = snapshot();
      const auto now = std::chrono::steady_clock::now();
      if (feedback.execution_generation >= accepted_feedback.execution_generation &&
        (feedback.execution.state == ExecutionStatus::FAILED ||
        feedback.execution.state == ExecutionStatus::REJECTED_BUSY))
      {
        detail = feedback.execution.message.empty() ?
          "Cartesian execution failed" : feedback.execution.message;
        return false;
      }

      std::string pose_detail;
      const bool stream_finished =
        feedback.execution_generation >= accepted_feedback.execution_generation &&
        feedback.execution.state == ExecutionStatus::STREAM_FINISHED;
      const bool feedback_fresh =
        feedback.joint_complete && feedback.has_motion &&
        isFresh(feedback.joint_time, now) && isFresh(feedback.motion_time, now);
      const bool tcp_reached = targetReached(
        feedback, left_target, right_target,
        position_tolerance_m_, orientation_tolerance_rad_, &pose_detail);

      // STREAM_FINISHED means the planner has published its final point. For
      // Cartesian completion, the authoritative result is fresh TCP feedback
      // with zero motion. ArmMotionStatus::goal_reached uses the controller's
      // much tighter all-joint tolerance (10 urad), which can remain false
      // after a physically settled TCP target and caused false timeouts.
      if (stream_finished && feedback_fresh && tcp_reached &&
        !feedback.motion.is_moving)
      {
        if (!feedback.motion.goal_reached) {
          RCLCPP_WARN(
            get_logger(),
            "Cartesian target reached with controller goal_reached=false; "
            "accepting fresh settled TCP feedback: %s",
            pose_detail.c_str());
        }
        detail = pose_detail;
        return true;
      }
      detail = pose_detail;

      if (std::chrono::steady_clock::now() >= deadline) {
        detail = "Completion timed out after " + std::to_string(timeout_seconds) +
          "s; " + detail + "; execution_state=" +
          std::to_string(feedback.execution.state) + " message='" +
          feedback.execution.message + "'";
        return false;
      }
      std::unique_lock<std::mutex> lock(feedback_mutex_);
      feedback_condition_.wait_for(lock, 20ms);
    }
    detail = "Interrupted while waiting for Cartesian completion";
    return false;
  }

  robot_control_msg::msg::Robotarmmovel makeCommand(
    const Pose & left_target,
    const Pose & right_target,
    double velocity,
    double acceleration) const
  {
    robot_control_msg::msg::Robotarmmovel command;
    command.lx = left_target.position.x();
    command.ly = left_target.position.y();
    command.lz = left_target.position.z();
    const Eigen::Quaterniond left_quaternion(left_target.rotation);
    command.lqx = left_quaternion.x();
    command.lqy = left_quaternion.y();
    command.lqz = left_quaternion.z();
    command.lqw = left_quaternion.w();
    command.rx = right_target.position.x();
    command.ry = right_target.position.y();
    command.rz = right_target.position.z();
    const Eigen::Quaterniond right_quaternion(right_target.rotation);
    command.rqx = right_quaternion.x();
    command.rqy = right_quaternion.y();
    command.rqz = right_quaternion.z();
    command.rqw = right_quaternion.w();
    command.vel = velocity;
    command.acc = acceleration;
    return command;
  }

  template<typename ResponseT>
  void executeCommand(
    const Pose & left_target,
    const Pose & right_target,
    double velocity,
    double acceleration,
    const FeedbackSnapshot & feedback_before,
    const std::string & command_label,
    ResponseT & response)
  {
    if (targetReached(
        feedback_before, left_target, right_target,
        command_position_deadband_m_, command_orientation_deadband_rad_))
    {
      publishNoOpExecutionStatus(command_label + " already at target");
      response.success = true;
      response.message = command_label + " already at target";
      return;
    }

    command_publisher_->publish(
      makeCommand(left_target, right_target, velocity, acceleration));

    FeedbackSnapshot accepted_feedback;
    std::string error;
    if (!waitForAcceptance(
        feedback_before.execution_generation, accepted_feedback, error))
    {
      response.success = false;
      response.message = command_label + " rejected: " + error;
      return;
    }

    std::string completion_detail;
    if (!waitForCompletion(
        left_target, right_target, accepted_feedback, completion_detail))
    {
      response.success = false;
      response.message = command_label + " failed: " + completion_detail;
      return;
    }

    response.success = true;
    response.message = command_label + " completed and TCP pose confirmed; " +
      completion_detail;
  }

  void publishNoOpExecutionStatus(const std::string & message)
  {
    ExecutionStatus status;
    status.stamp = now();
    status.state = ExecutionStatus::STREAM_FINISHED;
    status.planned_points = 0;
    status.planned_duration_sec = 0.0;
    status.message = message;
    execution_status_publisher_->publish(status);
  }

  void handleAbsoluteRequest(
    const std::shared_ptr<Absolute::Request> request,
    std::shared_ptr<Absolute::Response> response)
  {
    std::string error;
    if (!validateNumerics(*request, error)) {
      response->success = false;
      response->message = "Cartesian absolute request rejected: " + error;
      return;
    }

    robot_control_msg::safety::RobotCommandGuard command_guard(
      "cartesian_absolute_control");
    if (!command_guard.acquired()) {
      response->success = false;
      response->message = "Cartesian absolute request rejected: " + command_guard.error();
      return;
    }

    FeedbackSnapshot feedback;
    if (!waitForReadyFeedback(feedback, error)) {
      response->success = false;
      response->message = "Cartesian absolute request rejected: " + error;
      return;
    }

    Pose left_target;
    Pose right_target;
    left_target.position = Eigen::Vector3d(request->lx, request->ly, request->lz);
    right_target.position = Eigen::Vector3d(request->rx, request->ry, request->rz);
    if (!parseOrientation(
        request->lroll, request->lpitch, request->lyaw,
        request->lqx, request->lqy, request->lqz, request->lqw,
        "left", left_target.rotation, error) ||
      !parseOrientation(
        request->rroll, request->rpitch, request->ryaw,
        request->rqx, request->rqy, request->rqz, request->rqw,
        "right", right_target.rotation, error))
    {
      response->success = false;
      response->message = "Cartesian absolute request rejected: " + error;
      return;
    }

    const auto current = currentPoses(feedback.q);
    const double left_translation = (left_target.position - current.first.position).norm();
    const double right_translation = (right_target.position - current.second.position).norm();
    const double left_rotation = rotationDistance(current.first.rotation, left_target.rotation);
    const double right_rotation = rotationDistance(current.second.rotation, right_target.rotation);
    if (left_translation > max_absolute_translation_delta_m_ ||
      right_translation > max_absolute_translation_delta_m_)
    {
      response->success = false;
      response->message =
        "Cartesian absolute request exceeds maximum per-command translation delta " +
        std::to_string(max_absolute_translation_delta_m_) + " m";
      return;
    }
    if (left_rotation > max_absolute_rotation_delta_rad_ ||
      right_rotation > max_absolute_rotation_delta_rad_)
    {
      response->success = false;
      response->message =
        "Cartesian absolute request exceeds maximum per-command rotation delta " +
        std::to_string(max_absolute_rotation_delta_rad_) + " rad";
      return;
    }

    executeCommand(
      left_target, right_target, request->vel, request->acc,
      feedback, "Cartesian absolute control", *response);
  }

  void handleIncrementRequest(
    const std::shared_ptr<Increment::Request> request,
    std::shared_ptr<Increment::Response> response)
  {
    std::string error;
    if (!validateNumerics(*request, error)) {
      response->success = false;
      response->message = "Cartesian increment request rejected: " + error;
      return;
    }

    Eigen::Matrix3d left_increment_rotation;
    Eigen::Matrix3d right_increment_rotation;
    if (!parseOrientation(
        request->lroll, request->lpitch, request->lyaw,
        request->lqx, request->lqy, request->lqz, request->lqw,
        "left increment", left_increment_rotation, error) ||
      !parseOrientation(
        request->rroll, request->rpitch, request->ryaw,
        request->rqx, request->rqy, request->rqz, request->rqw,
        "right increment", right_increment_rotation, error))
    {
      response->success = false;
      response->message = "Cartesian increment request rejected: " + error;
      return;
    }

    const Eigen::Vector3d left_translation(request->lx, request->ly, request->lz);
    const Eigen::Vector3d right_translation(request->rx, request->ry, request->rz);
    const double left_rotation = rotationDistance(
      Eigen::Matrix3d::Identity(), left_increment_rotation);
    const double right_rotation = rotationDistance(
      Eigen::Matrix3d::Identity(), right_increment_rotation);
    if (left_translation.norm() > max_increment_translation_m_ ||
      right_translation.norm() > max_increment_translation_m_)
    {
      response->success = false;
      response->message =
        "Cartesian increment request exceeds maximum translation " +
        std::to_string(max_increment_translation_m_) + " m";
      return;
    }
    if (left_rotation > max_increment_rotation_rad_ ||
      right_rotation > max_increment_rotation_rad_)
    {
      response->success = false;
      response->message =
        "Cartesian increment request exceeds maximum rotation " +
        std::to_string(max_increment_rotation_rad_) + " rad";
      return;
    }

    robot_control_msg::safety::RobotCommandGuard command_guard(
      "cartesian_increment_control");
    if (!command_guard.acquired()) {
      response->success = false;
      response->message = "Cartesian increment request rejected: " + command_guard.error();
      return;
    }

    FeedbackSnapshot feedback;
    if (!waitForReadyFeedback(feedback, error)) {
      response->success = false;
      response->message = "Cartesian increment request rejected: " + error;
      return;
    }

    const auto current = currentPoses(feedback.q);
    Pose left_target;
    Pose right_target;
    left_target.position = current.first.position + left_translation;
    right_target.position = current.second.position + right_translation;
    left_target.rotation = current.first.rotation * left_increment_rotation;
    right_target.rotation = current.second.rotation * right_increment_rotation;

    executeCommand(
      left_target, right_target, request->vel, request->acc,
      feedback, "Cartesian increment control", *response);
  }

  std::string urdf_path_;
  std::string ee_frame_left_;
  std::string ee_frame_right_;
  std::string joint_state_topic_;
  std::string motion_status_topic_;

  int feedback_stale_timeout_ms_{1000};
  int feedback_wait_timeout_ms_{5000};
  int acceptance_timeout_ms_{6000};
  double minimum_completion_timeout_sec_{6.0};
  double maximum_completion_timeout_sec_{60.0};
  double position_tolerance_m_{0.002};
  double orientation_tolerance_rad_{0.03};
  double command_position_deadband_m_{0.0001};
  double command_orientation_deadband_rad_{0.001};
  double max_absolute_translation_delta_m_{0.20};
  double max_absolute_rotation_delta_rad_{0.75};
  double max_increment_translation_m_{0.10};
  double max_increment_rotation_rad_{0.35};
  double max_velocity_mps_{0.50};
  double max_acceleration_mps2_{1.0};
  double quaternion_norm_tolerance_{0.05};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_id_left_{0};
  pinocchio::FrameIndex ee_id_right_{0};
  pinocchio::SE3 tcp_transform_left_;
  pinocchio::SE3 tcp_transform_right_;
  std::unordered_map<std::string, int> joint_q_index_;
  std::unordered_set<std::string> required_joint_names_;

  mutable std::mutex feedback_mutex_;
  std::condition_variable feedback_condition_;
  Eigen::VectorXd q_current_;
  std::unordered_set<std::string> received_joint_names_;
  robot_control_msg::msg::ArmMotionStatus last_motion_status_;
  ExecutionStatus last_execution_status_;
  PowerStatus last_power_status_;
  ModeStatus last_mode_status_;
  bool has_motion_status_{false};
  bool has_execution_status_{false};
  bool has_power_status_{false};
  bool has_mode_status_{false};
  std::uint64_t execution_status_generation_{0};
  std::chrono::steady_clock::time_point joint_state_time_{};
  std::chrono::steady_clock::time_point motion_status_time_{};
  std::chrono::steady_clock::time_point power_status_time_{};
  std::chrono::steady_clock::time_point mode_status_time_{};

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr
    motion_status_subscription_;
  rclcpp::Subscription<ExecutionStatus>::SharedPtr execution_status_subscription_;
  rclcpp::Subscription<PowerStatus>::SharedPtr power_status_subscription_;
  rclcpp::Subscription<ModeStatus>::SharedPtr mode_status_subscription_;
  rclcpp::Publisher<robot_control_msg::msg::Robotarmmovel>::SharedPtr command_publisher_;
  rclcpp::Publisher<ExecutionStatus>::SharedPtr execution_status_publisher_;
  rclcpp::Service<Absolute>::SharedPtr absolute_service_;
  rclcpp::Service<Increment>::SharedPtr increment_service_;
};

}  // namespace robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_control::CartesianSingleControlSrv>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cartesian_single_control_srv"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}

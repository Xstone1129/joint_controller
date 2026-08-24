#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <robot_control_msg/msg/arm_motion_status.hpp>
#include <robot_control_msg/msg/cartesian_execution_status.hpp>
#include <robot_control_msg/msg/end_effector_pose.hpp>
#include <robot_control_msg/msg/robotarmmovelpath.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
#include <robot_control_msg/srv/cartesian_path_absolute_control.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace teleop_control
{

class CartesianPathAbsoluteControlSrv : public rclcpp::Node
{
public:
  CartesianPathAbsoluteControlSrv() : Node("cartesian_path_absolute_control_srv")
  {
    // Keep these parameters for launch compatibility.
    urdf_path_ = this->declare_parameter<std::string>(
      "urdf_path", "");
    ee_frame_left_ = this->declare_parameter<std::string>("ee_frame_left", "lee_link");
    ee_frame_right_ = this->declare_parameter<std::string>("ee_frame_right", "ree_link");
    motion_status_topic_ = this->declare_parameter<std::string>(
      "motion_status_topic", "/arm/arm_controller/motion_status");

    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.005);
    rotation_tolerance_ = this->declare_parameter<double>("rotation_tolerance", 0.01);
    accept_timeout_sec_ = this->declare_parameter<double>("accept_timeout_sec", 5.0);
    settle_timeout_sec_ = this->declare_parameter<double>("settle_timeout_sec", 5.0);
    no_progress_timeout_sec_ = this->declare_parameter<double>("no_progress_timeout_sec", 10.0);
    progress_position_epsilon_ = this->declare_parameter<double>("progress_position_epsilon", 0.001);
    progress_rotation_epsilon_ = this->declare_parameter<double>("progress_rotation_epsilon", 0.002);
    post_stream_feedback_grace_ms_ =
      this->declare_parameter<int>("post_stream_feedback_grace_ms", 50);

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions topic_sub_options;
    topic_sub_options.callback_group = topic_cb_group_;

    motion_status_sub_ = this->create_subscription<robot_control_msg::msg::ArmMotionStatus>(
      motion_status_topic_, 10,
      std::bind(&CartesianPathAbsoluteControlSrv::motionStatusCallback, this, std::placeholders::_1),
      topic_sub_options);

    tcp_pose_sub_ = this->create_subscription<robot_control_msg::msg::EndEffectorPose>(
      "/arm_tcp_pose", 10,
      std::bind(&CartesianPathAbsoluteControlSrv::tcpPoseCallback, this, std::placeholders::_1),
      topic_sub_options);

    rclcpp::QoS execution_status_qos(1);
    execution_status_qos.reliable();
    execution_status_qos.transient_local();
    execution_status_sub_ =
      this->create_subscription<robot_control_msg::msg::CartesianExecutionStatus>(
        "/arm_cartesian_path_execution_status", execution_status_qos,
        std::bind(
          &CartesianPathAbsoluteControlSrv::executionStatusCallback, this, std::placeholders::_1),
        topic_sub_options);

    path_cmd_pub_ = this->create_publisher<robot_control_msg::msg::Robotarmmovelpath>(
      "/arm_cartesian_path_cmd", 10);

    absolute_control_srv_ = this->create_service<robot_control_msg::srv::CartesianPathAbsoluteControl>(
      "cartesian_path_absolute_control",
      std::bind(
        &CartesianPathAbsoluteControlSrv::handleAbsolutePathRequest, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_cb_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "Cartesian Path Absolute Control Service is ready on cartesian_path_absolute_control");
  }

private:
  struct PoseState
  {
    Eigen::Vector3d position {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation {Eigen::Quaterniond::Identity()};
  };

  struct GoalErrorState
  {
    double max_position_error {std::numeric_limits<double>::infinity()};
    double max_rotation_error {std::numeric_limits<double>::infinity()};
    bool pose_reached {false};
  };

  static double clampUnit(double value)
  {
    return std::clamp(value, -1.0, 1.0);
  }

  static Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond& q)
  {
    Eigen::Quaterniond normalized = q;
    normalized.normalize();
    if (normalized.w() < 0.0) {
      normalized.coeffs() *= -1.0;
    }
    return normalized;
  }

  static double quaternionAngularDistance(
    const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
  {
    const double dot = std::abs(a.normalized().dot(b.normalized()));
    return 2.0 * std::acos(clampUnit(dot));
  }

  bool poseFromMsg(
    const geometry_msgs::msg::Pose& pose_msg, PoseState& pose, std::string& error,
    const std::string& field_name) const
  {
    const Eigen::Quaterniond quat(
      pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
    if (quat.norm() < 1e-8) {
      error = field_name + " quaternion norm is zero";
      return false;
    }

    pose.position = Eigen::Vector3d(
      pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    pose.orientation = normalizeQuaternion(quat);
    return true;
  }

  void motionStatusCallback(const robot_control_msg::msg::ArmMotionStatus::SharedPtr msg)
  {
    last_motion_status_ = *msg;
    has_motion_status_ = true;
    last_motion_status_time_ = this->now();
  }

  void tcpPoseCallback(const robot_control_msg::msg::EndEffectorPose::SharedPtr msg)
  {
    latest_tcp_pose_ = *msg;
    has_tcp_pose_ = true;
    last_tcp_pose_time_ = this->now();
  }

  void executionStatusCallback(
    const robot_control_msg::msg::CartesianExecutionStatus::SharedPtr msg)
  {
    latest_execution_status_ = *msg;
    has_execution_status_ = true;
    last_execution_status_time_ = this->now();
  }

  bool validateBlendRadii(
    const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::vector<double>& radii,
    const std::string& arm_name, std::string& error) const
  {
    if (waypoints.empty()) {
      if (!radii.empty()) {
        error = arm_name + " blend radii must be empty when waypoints are empty";
        return false;
      }
      return true;
    }

    if (radii.empty()) {
      return true;
    }

    if (radii.size() != waypoints.size() - 1) {
      error = arm_name + " blend radii size must equal waypoint_count - 1";
      return false;
    }

    for (double radius : radii) {
      if (radius < 0.0) {
        error = arm_name + " blend radius must be non-negative";
        return false;
      }
    }
    return true;
  }

  bool validateAbsoluteRequest(
    const robot_control_msg::srv::CartesianPathAbsoluteControl::Request& request,
    std::string& error) const
  {
    if (request.left_waypoints.empty() && request.right_waypoints.empty()) {
      error = "left_waypoints and right_waypoints cannot both be empty";
      return false;
    }

    if (request.vel <= 0.0 || request.acc <= 0.0) {
      error = "vel and acc must be positive";
      return false;
    }

    if (!validateBlendRadii(request.left_waypoints, request.left_blend_radii, "left", error)) {
      return false;
    }
    if (!validateBlendRadii(request.right_waypoints, request.right_blend_radii, "right", error)) {
      return false;
    }

    PoseState pose;
    for (size_t i = 0; i < request.left_waypoints.size(); ++i) {
      if (!poseFromMsg(
            request.left_waypoints[i], pose, error,
            "left_waypoints[" + std::to_string(i) + "]")) {
        return false;
      }
    }

    for (size_t i = 0; i < request.right_waypoints.size(); ++i) {
      if (!poseFromMsg(
            request.right_waypoints[i], pose, error,
            "right_waypoints[" + std::to_string(i) + "]")) {
        return false;
      }
    }
    return true;
  }

  bool currentTCPPosePair(PoseState& left_pose, PoseState& right_pose, std::string& error) const
  {
    if (!has_tcp_pose_) {
      error = "TCP pose feedback is not available yet";
      return false;
    }

    if (!poseFromMsg(latest_tcp_pose_.left_ee_pose, left_pose, error, "left_tcp_pose")) {
      return false;
    }
    if (!poseFromMsg(latest_tcp_pose_.right_ee_pose, right_pose, error, "right_tcp_pose")) {
      return false;
    }
    return true;
  }

  bool waitForFeedbackReady(std::string& error)
  {
    const auto deadline =
      this->now() + rclcpp::Duration::from_seconds(accept_timeout_sec_);
    while (rclcpp::ok()) {
      if (has_motion_status_ && has_tcp_pose_ && has_execution_status_) {
        return true;
      }
      if (this->now() > deadline) {
        error = "Timed out waiting for path feedback topics";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    error = "Interrupted while waiting for path feedback topics";
    return false;
  }

  bool waitForArmIdle(std::string& error)
  {
    const auto deadline =
      this->now() + rclcpp::Duration::from_seconds(accept_timeout_sec_);
    while (rclcpp::ok()) {
      if (
        has_motion_status_ &&
        latest_execution_status_.state != robot_control_msg::msg::CartesianExecutionStatus::PLANNING &&
        latest_execution_status_.state != robot_control_msg::msg::CartesianExecutionStatus::EXECUTING &&
        !last_motion_status_.is_moving) {
        return true;
      }

      if (this->now() > deadline) {
        error = "Timed out waiting for arm path pipeline to become idle";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    error = "Interrupted while waiting for arm path pipeline to become idle";
    return false;
  }

  bool analyzeAbsoluteArmPath(
    const PoseState& start_pose, const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double& path_length, bool& has_motion, PoseState& target_pose, std::string& error,
    const std::string& arm_name) const
  {
    path_length = 0.0;
    has_motion = false;
    target_pose = start_pose;
    PoseState previous_pose = start_pose;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      PoseState current_pose;
      if (!poseFromMsg(
            waypoints[i], current_pose, error,
            arm_name + "_waypoints[" + std::to_string(i) + "]")) {
        return false;
      }

      path_length += (current_pose.position - previous_pose.position).norm();
      if (
        !has_motion &&
        ((current_pose.position - previous_pose.position).norm() > 1e-6 ||
        quaternionAngularDistance(current_pose.orientation, previous_pose.orientation) > 1e-4)) {
        has_motion = true;
      }

      previous_pose = current_pose;
      target_pose = current_pose;
    }

    return true;
  }

  bool isTCPPoseReached(
    const PoseState& target_left, const PoseState& target_right) const
  {
    GoalErrorState error_state;
    std::string error;
    return computeGoalErrorState(target_left, target_right, error_state, error) && error_state.pose_reached;
  }

  bool computeGoalErrorState(
    const PoseState& target_left, const PoseState& target_right, GoalErrorState& error_state,
    std::string& error) const
  {
    PoseState current_left;
    PoseState current_right;
    if (!currentTCPPosePair(current_left, current_right, error)) {
      return false;
    }

    const double left_pos_error = (current_left.position - target_left.position).norm();
    const double right_pos_error = (current_right.position - target_right.position).norm();
    const double left_rot_error =
      quaternionAngularDistance(current_left.orientation, target_left.orientation);
    const double right_rot_error =
      quaternionAngularDistance(current_right.orientation, target_right.orientation);

    error_state.max_position_error = std::max(left_pos_error, right_pos_error);
    error_state.max_rotation_error = std::max(left_rot_error, right_rot_error);
    error_state.pose_reached =
      left_pos_error <= position_tolerance_ &&
      right_pos_error <= position_tolerance_ &&
      left_rot_error <= rotation_tolerance_ &&
      right_rot_error <= rotation_tolerance_;
    return true;
  }

  bool waitForCommandAcceptance(
    const rclcpp::Time& command_time, double& planned_duration_sec, std::string& error)
  {
    const auto deadline =
      this->now() + rclcpp::Duration::from_seconds(accept_timeout_sec_);
    planned_duration_sec = 0.0;

    while (rclcpp::ok()) {
      if (has_execution_status_) {
        const rclcpp::Time status_stamp(latest_execution_status_.stamp);
        if (status_stamp >= command_time) {
          switch (latest_execution_status_.state) {
            case robot_control_msg::msg::CartesianExecutionStatus::FAILED:
              error = "Path execution rejected: " + latest_execution_status_.message;
              return false;
            case robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY:
              error = "Path execution busy: " + latest_execution_status_.message;
              return false;
            case robot_control_msg::msg::CartesianExecutionStatus::PLANNING:
            case robot_control_msg::msg::CartesianExecutionStatus::EXECUTING:
            case robot_control_msg::msg::CartesianExecutionStatus::STREAM_FINISHED:
              planned_duration_sec = latest_execution_status_.planned_duration_sec;
              return true;
            default:
              break;
          }
        }
      }

      if (this->now() > deadline) {
        error = "Timed out waiting for path command acceptance";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    error = "Interrupted while waiting for path command acceptance";
    return false;
  }

  bool waitForStreamFinished(
    const rclcpp::Time& command_time, double planned_duration_sec, rclcpp::Time& stream_finished_time,
    std::string& error)
  {
    // 使用planned_duration_sec作为超时时间，如果可用的话
    double stream_timeout_sec = accept_timeout_sec_ + settle_timeout_sec_;
    if (planned_duration_sec > 0) {
      stream_timeout_sec = std::max(stream_timeout_sec, planned_duration_sec * 1.5); // 增加50%的安全余量
      RCLCPP_INFO(this->get_logger(), "Using planned duration %.2f sec as timeout (with 50%% margin): %.2f sec", 
                 planned_duration_sec, stream_timeout_sec);
    }
    
    const auto deadline =
      command_time + rclcpp::Duration::from_seconds(stream_timeout_sec);

    while (rclcpp::ok()) {
      if (has_execution_status_) {
        const rclcpp::Time status_stamp(latest_execution_status_.stamp);
        if (status_stamp >= command_time) {
          switch (latest_execution_status_.state) {
            case robot_control_msg::msg::CartesianExecutionStatus::FAILED:
              error = "Path execution failed: " + latest_execution_status_.message;
              return false;
            case robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY:
              error = "Path execution rejected: " + latest_execution_status_.message;
              return false;
            case robot_control_msg::msg::CartesianExecutionStatus::STREAM_FINISHED:
              stream_finished_time = status_stamp;
              return true;
            default:
              break;
          }
        }
      }

      if (this->now() > deadline) {
        error = "Timed out waiting for path stream completion";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    error = "Interrupted while waiting for path stream completion";
    return false;
  }

  bool waitForGoalReached(
    const PoseState& target_left, const PoseState& target_right,
    const rclcpp::Time& stream_finished_time, std::string& error)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(post_stream_feedback_grace_ms_));
    const rclcpp::Time feedback_ready_time = this->now();
    const auto stall_timeout =
      rclcpp::Duration::from_seconds(no_progress_timeout_sec_);
    rclcpp::Time fresh_feedback_wait_start = feedback_ready_time;
    rclcpp::Time stall_start_time = feedback_ready_time;
    bool stall_timer_active = false;
    double best_position_error = std::numeric_limits<double>::infinity();
    double best_rotation_error = std::numeric_limits<double>::infinity();
    rclcpp::Time last_motion_feedback_time = feedback_ready_time;
    rclcpp::Time last_tcp_feedback_time = feedback_ready_time;

    while (rclcpp::ok()) {
      const rclcpp::Time now = this->now();

      if (has_execution_status_) {
        const rclcpp::Time status_stamp(latest_execution_status_.stamp);
        if (status_stamp >= stream_finished_time) {
          if (latest_execution_status_.state == robot_control_msg::msg::CartesianExecutionStatus::FAILED) {
            error = "Path execution failed after stream completion: " + latest_execution_status_.message;
            return false;
          }
          if (latest_execution_status_.state == robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY) {
            error = "Path execution was rejected after stream completion";
            return false;
          }
        }
      }

      if (last_motion_status_time_ < feedback_ready_time || last_tcp_pose_time_ < feedback_ready_time) {
        if ((now - fresh_feedback_wait_start) > stall_timeout) {
          error = "Timed out waiting for fresh feedback after path stream completion";
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      GoalErrorState error_state;
      std::string error_state_message;
      const bool has_goal_error =
        computeGoalErrorState(target_left, target_right, error_state, error_state_message);

      bool progress = false;
      if (has_goal_error) {
        if (!std::isfinite(best_position_error) ||
          best_position_error - error_state.max_position_error > progress_position_epsilon_) {
          best_position_error = error_state.max_position_error;
          progress = true;
        }

        if (!std::isfinite(best_rotation_error) ||
          best_rotation_error - error_state.max_rotation_error > progress_rotation_epsilon_) {
          best_rotation_error = error_state.max_rotation_error;
          progress = true;
        }
      }

      if (last_motion_status_time_ > last_motion_feedback_time) {
        last_motion_feedback_time = last_motion_status_time_;
        progress = true;
      }

      if (last_tcp_pose_time_ > last_tcp_feedback_time) {
        last_tcp_feedback_time = last_tcp_pose_time_;
        progress = true;
      }

      if (
        has_motion_status_ &&
        has_tcp_pose_ &&
        last_motion_status_time_ >= feedback_ready_time &&
        last_tcp_pose_time_ >= feedback_ready_time &&
        last_motion_status_time_ >= stream_finished_time &&
        last_tcp_pose_time_ >= stream_finished_time &&
        !last_motion_status_.is_moving &&
        last_motion_status_.goal_reached &&
        has_goal_error &&
        error_state.pose_reached) {
        return true;
      }

      if ((now - last_motion_status_time_) > stall_timeout) {
        error = "Motion status feedback stalled while waiting for final path goal";
        return false;
      }

      if ((now - last_tcp_pose_time_) > stall_timeout) {
        error = "TCP pose feedback stalled while waiting for final path goal";
        return false;
      }

      const bool motion_stopped = has_motion_status_ && !last_motion_status_.is_moving;
      const bool goal_not_reached = !has_motion_status_ || !last_motion_status_.goal_reached;
      const bool tcp_not_reached = !has_goal_error || !error_state.pose_reached;
      const bool stall_condition = motion_stopped && (goal_not_reached || tcp_not_reached);

      if (progress) {
        if (stall_condition) {
          stall_start_time = now;
          stall_timer_active = true;
        } else {
          stall_timer_active = false;
        }
      } else if (stall_condition) {
        if (!stall_timer_active) {
          stall_start_time = now;
          stall_timer_active = true;
        } else if ((now - stall_start_time) > stall_timeout) {
          if (has_goal_error) {
            error =
              "Arm stopped before reaching final path goal: max_position_error=" +
              std::to_string(error_state.max_position_error) +
              ", max_rotation_error=" + std::to_string(error_state.max_rotation_error);
          } else {
            error = "Arm stopped before reaching final path goal: " + error_state_message;
          }
          return false;
        }
      } else {
        stall_timer_active = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    error = "Interrupted while waiting for final path goal";
    return false;
  }

  void handleAbsolutePathRequest(
    const std::shared_ptr<robot_control_msg::srv::CartesianPathAbsoluteControl::Request> request,
    std::shared_ptr<robot_control_msg::srv::CartesianPathAbsoluteControl::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received cartesian path absolute control request");

    std::string error;
    robot_control_msg::safety::RobotCommandGuard command_guard(
      "cartesian_path_absolute_control");
    if (!command_guard.acquired()) {
      response->success = false;
      response->message = command_guard.error();
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!waitForFeedbackReady(error)) {
      response->success = false;
      // 在错误消息中包含当前状态信息
      std::string detailed_error = error;
      if (has_execution_status_) {
        detailed_error += ". Current execution status: state=" + 
          std::to_string(latest_execution_status_.state) + 
          ", planned_duration=" + 
          std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
          ", planned_points=" + 
          std::to_string(latest_execution_status_.planned_points) +
          ", message='" + latest_execution_status_.message + "'";
      }
      response->message = detailed_error;
      RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
      return;
    }

    if (!waitForArmIdle(error)) {
      response->success = false;
      // 在错误消息中包含当前状态信息
      std::string detailed_error = error;
      if (has_execution_status_) {
        detailed_error += ". Current execution status: state=" + 
          std::to_string(latest_execution_status_.state) + 
          ", planned_duration=" + 
          std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
          ", planned_points=" + 
          std::to_string(latest_execution_status_.planned_points) +
          ", message='" + latest_execution_status_.message + "'";
      }
      response->message = detailed_error;
      RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
      return;
    }

    if (!validateAbsoluteRequest(*request, error)) {
      response->success = false;
      response->message = error;
      RCLCPP_ERROR(this->get_logger(), "Invalid absolute path request: %s", error.c_str());
      return;
    }

    PoseState current_left;
    PoseState current_right;
    if (!currentTCPPosePair(current_left, current_right, error)) {
      response->success = false;
      // 在错误消息中包含当前状态信息
      std::string detailed_error = error;
      if (has_execution_status_) {
        detailed_error += ". Current execution status: state=" + 
          std::to_string(latest_execution_status_.state) + 
          ", planned_duration=" + 
          std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
          ", planned_points=" + 
          std::to_string(latest_execution_status_.planned_points) +
          ", message='" + latest_execution_status_.message + "'";
      }
      response->message = detailed_error;
      RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
      return;
    }

    PoseState target_left = current_left;
    PoseState target_right = current_right;
    double left_path_length = 0.0;
    double right_path_length = 0.0;
    bool left_has_motion = false;
    bool right_has_motion = false;

    if (!analyzeAbsoluteArmPath(
          current_left, request->left_waypoints, left_path_length, left_has_motion, target_left, error, "left")) {
      response->success = false;
      // 在错误消息中包含当前状态信息
      std::string detailed_error = error;
      if (has_execution_status_) {
        detailed_error += ". Current execution status: state=" + 
          std::to_string(latest_execution_status_.state) + 
          ", planned_duration=" + 
          std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
          ", planned_points=" + 
          std::to_string(latest_execution_status_.planned_points) +
          ", message='" + latest_execution_status_.message + "'";
      }
      response->message = detailed_error;
      RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
      return;
    }

    if (!analyzeAbsoluteArmPath(
          current_right, request->right_waypoints, right_path_length, right_has_motion, target_right, error, "right")) {
      response->success = false;
      // 在错误消息中包含当前状态信息
      std::string detailed_error = error;
      if (has_execution_status_) {
        detailed_error += ". Current execution status: state=" + 
          std::to_string(latest_execution_status_.state) + 
          ", planned_duration=" + 
          std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
          ", planned_points=" + 
          std::to_string(latest_execution_status_.planned_points) +
          ", message='" + latest_execution_status_.message + "'";
      }
      response->message = detailed_error;
      RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
      return;
    }

    robot_control_msg::msg::Robotarmmovelpath cmd_msg;
    cmd_msg.use_left = !request->left_waypoints.empty();
    cmd_msg.use_right = !request->right_waypoints.empty();
    cmd_msg.left_waypoints = request->left_waypoints;
    cmd_msg.left_blend_radii = request->left_blend_radii;
    cmd_msg.right_waypoints = request->right_waypoints;
    cmd_msg.right_blend_radii = request->right_blend_radii;
    cmd_msg.vel = request->vel;
    cmd_msg.acc = request->acc;

    const auto command_time = this->now();
    path_cmd_pub_->publish(cmd_msg);

    double planned_duration_sec = 0.0;
    if (!waitForCommandAcceptance(command_time, planned_duration_sec, error)) {
      response->success = false;
      response->message = error;
      return;
    }

    rclcpp::Time stream_finished_time(0, 0, this->get_clock()->get_clock_type());
    if (!waitForStreamFinished(command_time, planned_duration_sec, stream_finished_time, error)) {
      response->success = false;
      response->message = error;
      return;
    }

    if (!waitForGoalReached(target_left, target_right, stream_finished_time, error)) {
      response->success = false;
      response->message = error;
      return;
    }

    response->success = true;
    response->message = "Cartesian path absolute control completed successfully";
    RCLCPP_INFO(
      this->get_logger(),
      "Cartesian path absolute motion completed successfully (left_length=%.4f m, right_length=%.4f m, left_motion=%d, right_motion=%d)",
      left_path_length, right_path_length, left_has_motion, right_has_motion);
  }

  std::string urdf_path_;
  std::string ee_frame_left_;
  std::string ee_frame_right_;
  std::string motion_status_topic_;
  double position_tolerance_ {0.005};
  double rotation_tolerance_ {0.01};
  double accept_timeout_sec_ {5.0};
  double settle_timeout_sec_ {5.0};
  double no_progress_timeout_sec_ {10.0};
  double progress_position_epsilon_ {0.001};
  double progress_rotation_epsilon_ {0.002};
  int post_stream_feedback_grace_ms_ {50};

  bool has_motion_status_ {false};
  bool has_tcp_pose_ {false};
  bool has_execution_status_ {false};
  robot_control_msg::msg::ArmMotionStatus last_motion_status_;
  robot_control_msg::msg::EndEffectorPose latest_tcp_pose_;
  robot_control_msg::msg::CartesianExecutionStatus latest_execution_status_;
  rclcpp::Time last_motion_status_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tcp_pose_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_execution_status_time_ {0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<robot_control_msg::msg::EndEffectorPose>::SharedPtr tcp_pose_sub_;
  rclcpp::Subscription<robot_control_msg::msg::CartesianExecutionStatus>::SharedPtr
    execution_status_sub_;
  rclcpp::Publisher<robot_control_msg::msg::Robotarmmovelpath>::SharedPtr path_cmd_pub_;
  rclcpp::Service<robot_control_msg::srv::CartesianPathAbsoluteControl>::SharedPtr
    absolute_control_srv_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
};

}  // namespace teleop_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop_control::CartesianPathAbsoluteControlSrv>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

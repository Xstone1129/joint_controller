#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <robot_control_msg/srv/joint_absolute_control.hpp>
#include <robot_control_msg/msg/arm_control_mode_status.hpp>
#include <robot_control_msg/msg/robotarmjoint.hpp>
#include <robot_control_msg/msg/arm_motion_status.hpp>
#include <robot_control_msg/msg/arm_power_status.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
#include <robot_control/joint_completion_policy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace teleop_control
{

class JointAbsoluteControlSrv : public rclcpp::Node
{
public:
    JointAbsoluteControlSrv() : Node("joint_absolute_control_srv")
    {
        /* ------------------- 参数 ------------------- */
        urdf_path_ = this->declare_parameter<std::string>(
            "urdf_path", "");
        joint_state_topic_ = this->declare_parameter<std::string>(
            "joint_state_topic", "/arm/joint_states");
        motion_status_topic_ = this->declare_parameter<std::string>(
            "motion_status_topic", "/arm/arm_controller/motion_status");
        position_tolerance_ = this->declare_parameter<double>(
            "position_tolerance", 0.01);
        motion_start_timeout_ms_ = this->declare_parameter<int>(
            "motion_start_timeout_ms", 3000);
        motion_timeout_ms_ = this->declare_parameter<int>(
            "motion_timeout_ms", 15000);
        feedback_timeout_ms_ = this->declare_parameter<int>(
            "feedback_timeout_ms", 1000);
        settled_feedback_ms_ = this->declare_parameter<int>(
            "settled_feedback_ms", 200);
        if (!loadModel())
        {
            rclcpp::shutdown();
            return;
        }

        /* ------------------- 创建回调组 ------------------- */
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        /* ------------------- ROS ------------------- */
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = topic_cb_group_;
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, 50,
            std::bind(&JointAbsoluteControlSrv::jointStateCallback, this, std::placeholders::_1),
            sub_options);

        arm_cmd_pub_ = this->create_publisher<robot_control_msg::msg::Robotarmjoint>(
            "/arm_joint_absolute_cmd", 10);

        rclcpp::SubscriptionOptions motion_sub_options;
        motion_sub_options.callback_group = topic_cb_group_;
        motion_status_sub_ = this->create_subscription<robot_control_msg::msg::ArmMotionStatus>(
            motion_status_topic_, 10,
            std::bind(&JointAbsoluteControlSrv::motionStatusCallback, this, std::placeholders::_1),
            motion_sub_options);

        const auto state_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        power_status_sub_ = this->create_subscription<robot_control_msg::msg::ArmPowerStatus>(
            "/arm/power_status", state_qos,
            std::bind(&JointAbsoluteControlSrv::powerStatusCallback, this, std::placeholders::_1),
            motion_sub_options);
        control_mode_status_sub_ =
            this->create_subscription<robot_control_msg::msg::ArmControlModeStatus>(
            "/arm/control_mode_status", state_qos,
            std::bind(
                &JointAbsoluteControlSrv::controlModeStatusCallback, this,
                std::placeholders::_1),
            motion_sub_options);

        absolute_control_srv_ = this->create_service<robot_control_msg::srv::JointAbsoluteControl>(
            "arm_absolute_control",
            std::bind(&JointAbsoluteControlSrv::handleAbsoluteControlRequest, this, 
                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);

        joint_position_received_.assign(model_.nq, false);

        RCLCPP_INFO(
            this->get_logger(),
            "Joint Absolute Control Service is ready (position_tolerance=%.4f rad, "
            "motion_start_timeout=%d ms, motion_timeout=%d ms, feedback_timeout=%d ms, "
            "settled_feedback=%d ms).",
            position_tolerance_, motion_start_timeout_ms_, motion_timeout_ms_, feedback_timeout_ms_,
            settled_feedback_ms_);
    }

private:
    bool loadModel()
    {
        try
        {
            pinocchio::urdf::buildModel(urdf_path_, model_);
            data_ = pinocchio::Data(model_);
            
            for (int i = 0; i < model_.nq; ++i)
            {
                std::string jname = model_.names[i + 1];
                joint_index_map_[jname] = i;
            }
            
            std::vector<std::string> left_arm_joints = {"ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7"};
            std::vector<std::string> right_arm_joints = {"rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"};
            
            for (const auto& name : left_arm_joints) {
                auto it = joint_index_map_.find(name);
                if (it != joint_index_map_.end()) {
                    left_arm_indices_.push_back(it->second);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Could not find left arm joint: %s", name.c_str());
                }
            }
            for (const auto& name : right_arm_joints) {
                auto it = joint_index_map_.find(name);
                if (it != joint_index_map_.end()) {
                    right_arm_indices_.push_back(it->second);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Could not find right arm joint: %s", name.c_str());
                }
            }
            
            std::vector<int> left_arm_indices_filtered;
            for (int idx : left_arm_indices_) {
                if (idx < model_.nv) {
                    left_arm_indices_filtered.push_back(idx);
                }
            }
            left_arm_indices_ = left_arm_indices_filtered;
            
            std::vector<int> right_arm_indices_filtered;
            for (int idx : right_arm_indices_) {
                if (idx < model_.nv) {
                    right_arm_indices_filtered.push_back(idx);
                }
            }
            right_arm_indices_ = right_arm_indices_filtered;
            
            if (left_arm_indices_.size() != 7 || right_arm_indices_.size() != 7) {
                RCLCPP_ERROR(this->get_logger(), "Failed to find all arm joints. Left: %zu, Right: %zu", 
                            left_arm_indices_.size(), right_arm_indices_.size());
                return false;
            }
            
            q_current_.setZero(model_.nq);
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "URDF load error: %s", e.what());
            return false;
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                auto it = joint_index_map_.find(msg->name[i]);
                if (it != joint_index_map_.end() && i < msg->position.size())
                {
                    q_current_[it->second] = msg->position[i];
                    joint_position_received_[it->second] = true;
                }
            }
            has_joint_state_ = true;
            ++joint_state_generation_;
            last_joint_state_time_ = std::chrono::steady_clock::now();
        }
        feedback_cv_.notify_all();
    }

    void motionStatusCallback(const robot_control_msg::msg::ArmMotionStatus::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_motion_status_ = *msg;
            has_motion_status_ = true;
            ++motion_status_generation_;
            last_motion_status_time_ = std::chrono::steady_clock::now();
        }
        feedback_cv_.notify_all();
    }

    void powerStatusCallback(const robot_control_msg::msg::ArmPowerStatus::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_power_status_ = *msg;
            has_power_status_ = true;
            last_power_status_time_ = std::chrono::steady_clock::now();
        }
        feedback_cv_.notify_all();
    }

    void controlModeStatusCallback(
        const robot_control_msg::msg::ArmControlModeStatus::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_control_mode_status_ = *msg;
            has_control_mode_status_ = true;
            last_control_mode_status_time_ = std::chrono::steady_clock::now();
        }
        feedback_cv_.notify_all();
    }

    Eigen::VectorXd targetPositionsFromRequest(
        const robot_control_msg::srv::JointAbsoluteControl::Request& request) const
    {
        Eigen::VectorXd targets = Eigen::VectorXd::Zero(model_.nq);
        std::vector<std::string> left_joint_names = {"ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7"};
        std::vector<double> left_targets = {request.ljoint1, request.ljoint2, request.ljoint3, request.ljoint4, 
                                           request.ljoint5, request.ljoint6, request.ljoint7};
        
        for (size_t i = 0; i < left_joint_names.size(); ++i) {
            auto it = joint_index_map_.find(left_joint_names[i]);
            if (it != joint_index_map_.end()) {
                targets[it->second] = left_targets[i];
            }
        }

        std::vector<std::string> right_joint_names = {"rjoint1", "rjoint2", "rjoint3", "rjoint4", 
                                                     "rjoint5", "rjoint6", "rjoint7"};
        std::vector<double> right_targets = {request.rjoint1, request.rjoint2, request.rjoint3, request.rjoint4,
                                            request.rjoint5, request.rjoint6, request.rjoint7};
        
        for (size_t i = 0; i < right_joint_names.size(); ++i) {
            auto it = joint_index_map_.find(right_joint_names[i]);
            if (it != joint_index_map_.end()) {
                targets[it->second] = right_targets[i];
            }
        }

        return targets;
    }

    bool validateAbsoluteControlRequest(
        const robot_control_msg::srv::JointAbsoluteControl::Request& request,
        std::string& error) const
    {
        const std::vector<std::pair<std::string, double>> targets = {
            {"ljoint1", request.ljoint1}, {"ljoint2", request.ljoint2},
            {"ljoint3", request.ljoint3}, {"ljoint4", request.ljoint4},
            {"ljoint5", request.ljoint5}, {"ljoint6", request.ljoint6},
            {"ljoint7", request.ljoint7}, {"rjoint1", request.rjoint1},
            {"rjoint2", request.rjoint2}, {"rjoint3", request.rjoint3},
            {"rjoint4", request.rjoint4}, {"rjoint5", request.rjoint5},
            {"rjoint6", request.rjoint6}, {"rjoint7", request.rjoint7},
        };

        if (!std::isfinite(request.vel) || request.vel <= 0.0) {
            error = "vel must be finite and greater than zero";
            return false;
        }
        if (!std::isfinite(request.acc) || request.acc <= 0.0) {
            error = "acc must be finite and greater than zero";
            return false;
        }

        for (const auto& named_target : targets) {
            const std::string& name = named_target.first;
            const double target = named_target.second;
            if (!std::isfinite(target)) {
                error = "target for " + name + " must be finite";
                return false;
            }

            const auto index_it = joint_index_map_.find(name);
            if (index_it == joint_index_map_.end()) {
                error = "joint is missing from the loaded URDF: " + name;
                return false;
            }

            const int index = index_it->second;
            const double lower_limit = model_.lowerPositionLimit[index];
            const double upper_limit = model_.upperPositionLimit[index];
            if ((std::isfinite(lower_limit) && target < lower_limit) ||
                (std::isfinite(upper_limit) && target > upper_limit)) {
                error = "target for " + name + " is outside its URDF position limits";
                return false;
            }
        }

        return true;
    }

    bool feedbackReady(std::string& error) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (!has_joint_state_) {
            error = "no /arm/joint_states feedback received";
            return false;
        }
        if (!has_motion_status_) {
            error = "no /arm/arm_controller/motion_status feedback received";
            return false;
        }
        if (!has_power_status_) {
            error = "no /arm/power_status feedback received";
            return false;
        }
        if (!has_control_mode_status_) {
            error = "no /arm/control_mode_status feedback received";
            return false;
        }
        for (int idx : left_arm_indices_) {
            if (!joint_position_received_[idx]) {
                error = "joint state is missing " + model_.names[idx + 1];
                return false;
            }
        }
        for (int idx : right_arm_indices_) {
            if (!joint_position_received_[idx]) {
                error = "joint state is missing " + model_.names[idx + 1];
                return false;
            }
        }
        if (now - last_joint_state_time_ > std::chrono::milliseconds(feedback_timeout_ms_)) {
            error = "/arm/joint_states feedback is stale";
            return false;
        }
        if (now - last_motion_status_time_ > std::chrono::milliseconds(feedback_timeout_ms_)) {
            error = "/arm/arm_controller/motion_status feedback is stale";
            return false;
        }
        if (now - last_power_status_time_ > std::chrono::milliseconds(feedback_timeout_ms_)) {
            error = "/arm/power_status feedback is stale";
            return false;
        }
        if (now - last_control_mode_status_time_ >
            std::chrono::milliseconds(feedback_timeout_ms_)) {
            error = "/arm/control_mode_status feedback is stale";
            return false;
        }
        if (last_power_status_.joint_names.size() != 14 ||
            last_power_status_.status_codes.size() != 14 ||
            last_power_status_.enabled.size() != 14) {
            error = "invalid /arm/power_status array sizes";
            return false;
        }
        if (!last_power_status_.command_enabled || !last_power_status_.all_enabled) {
            error = "joint motion requires all 14 motors enabled";
            return false;
        }
        for (std::size_t index = 0; index < 14; ++index) {
            if (!last_power_status_.enabled[index] ||
                last_power_status_.status_codes[index] !=
                robot_control_msg::msg::ArmPowerStatus::ENABLED_STATUS) {
                error = "joint motion requires every motor to report enabled status 39";
                return false;
            }
        }
        if (last_control_mode_status_.active_mode !=
            robot_control_msg::msg::ArmControlModeStatus::POSITION) {
            error = "joint motion requires POSITION control mode";
            return false;
        }
        return true;
    }

    bool waitForArmToStop(std::string& error)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        std::unique_lock<std::mutex> lock(feedback_mutex_);
        while (rclcpp::ok()) {
            if (!last_motion_status_.is_moving) {
                return true;
            }
            if (feedback_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
                error = "timeout waiting for arm to stop before sending command";
                return false;
            }
        }
        error = "ROS shutdown while waiting for arm to stop";
        return false;
    }

    double maxArmPositionError(
        const Eigen::VectorXd& current, const Eigen::VectorXd& target,
        std::string& worst_joint) const
    {
        double max_error = 0.0;
        auto inspect = [&](int idx) {
            const double error = std::abs(std::remainder(current[idx] - target[idx], 2.0 * M_PI));
            if (error > max_error) {
                max_error = error;
                worst_joint = model_.names[idx + 1];
            }
        };
        for (int idx : left_arm_indices_) {
            inspect(idx);
        }
        for (int idx : right_arm_indices_) {
            inspect(idx);
        }
        return max_error;
    }

    bool waitForArmToReachPosition(
        const Eigen::VectorXd& target_positions,
        std::uint64_t motion_generation_before_command,
        std::string& error)
    {
        const auto start_time = std::chrono::steady_clock::now();
        const auto start_deadline = start_time + std::chrono::milliseconds(motion_start_timeout_ms_);
        const auto motion_deadline = start_time + std::chrono::milliseconds(motion_timeout_ms_);
        bool motion_detected = false;
        std::optional<std::chrono::steady_clock::time_point> settling_since;
        std::optional<std::chrono::steady_clock::time_point> stopped_outside_target_since;

        Eigen::VectorXd initial_q;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            initial_q = q_current_;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for final arm execution result...");

        while (rclcpp::ok()) {
            Eigen::VectorXd current_q;
            robot_control_msg::msg::ArmMotionStatus motion_status;
            std::uint64_t motion_generation = 0;
            std::chrono::steady_clock::time_point joint_state_time;
            std::chrono::steady_clock::time_point motion_status_time;
            {
                std::unique_lock<std::mutex> lock(feedback_mutex_);
                feedback_cv_.wait_for(lock, std::chrono::milliseconds(20));
                current_q = q_current_;
                motion_status = last_motion_status_;
                motion_generation = motion_status_generation_;
                joint_state_time = last_joint_state_time_;
                motion_status_time = last_motion_status_time_;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - joint_state_time > std::chrono::milliseconds(feedback_timeout_ms_)) {
                error = "/arm/joint_states feedback stopped during motion";
                return false;
            }
            if (now - motion_status_time > std::chrono::milliseconds(feedback_timeout_ms_)) {
                error = "/arm/arm_controller/motion_status feedback stopped during motion";
                return false;
            }

            const double total_change = (current_q - initial_q).cwiseAbs().maxCoeff();
            const bool has_post_command_status =
                motion_generation > motion_generation_before_command;
            if (!motion_detected && (motion_status.is_moving || total_change > 1e-4)) {
                motion_detected = true;
                RCLCPP_INFO(
                    this->get_logger(), "Arm motion detected via %s",
                    motion_status.is_moving ? "motion status" : "joint position change");
            }

            std::string worst_joint;
            const double max_error = maxArmPositionError(current_q, target_positions, worst_joint);
            const bool positions_reached = max_error <= position_tolerance_;
            const bool settling_candidate = robot_control::jointFeedbackSettlingCandidate(
                positions_reached, has_post_command_status, motion_status.is_moving,
                motion_detected);
            if (settling_candidate) {
                if (!settling_since.has_value()) {
                    settling_since = now;
                }
            } else {
                settling_since.reset();
            }
            const bool feedback_settled = settling_since.has_value() &&
                now - *settling_since >= std::chrono::milliseconds(settled_feedback_ms_);
            if (robot_control::jointMotionCompleted(
                    positions_reached, has_post_command_status, motion_status.is_moving,
                    motion_status.goal_reached, motion_detected, feedback_settled)) {
                if (!motion_status.goal_reached) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Arm target reached with controller goal_reached=false; "
                        "accepting fresh settled joint feedback");
                }
                RCLCPP_INFO(
                    this->get_logger(),
                    "Arm execution completed: max_position_error=%.6f rad",
                    max_error);
                return true;
            }

            if (!motion_detected && now >= start_deadline) {
                error = "timeout waiting for arm to start moving; max_position_error=" +
                    std::to_string(max_error) + " rad at " + worst_joint;
                return false;
            }

            const bool stopped_outside_target = motion_detected && has_post_command_status &&
                !motion_status.is_moving && !motion_status.goal_reached && !positions_reached;
            if (stopped_outside_target) {
                if (!stopped_outside_target_since.has_value()) {
                    stopped_outside_target_since = now;
                } else if (
                    now - *stopped_outside_target_since > std::chrono::seconds(2))
                {
                    error = "arm stopped before reaching target; max_position_error=" +
                        std::to_string(max_error) + " rad at " + worst_joint;
                    return false;
                }
            } else {
                stopped_outside_target_since.reset();
            }

            if (now >= motion_deadline) {
                error = "motion timeout; moving=" + std::string(motion_status.is_moving ? "true" : "false") +
                    ", goal_reached=" + std::string(motion_status.goal_reached ? "true" : "false") +
                    ", max_position_error=" + std::to_string(max_error) + " rad at " + worst_joint;
                return false;
            }
        }

        error = "ROS shutdown while waiting for arm execution result";
        return false;
    }

    void handleAbsoluteControlRequest(
        const std::shared_ptr<robot_control_msg::srv::JointAbsoluteControl::Request> request,
        std::shared_ptr<robot_control_msg::srv::JointAbsoluteControl::Response> response)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received joint absolute control request: "
            "left=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f], "
            "right=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f], vel=%.6f, acc=%.6f",
            request->ljoint1, request->ljoint2, request->ljoint3, request->ljoint4,
            request->ljoint5, request->ljoint6, request->ljoint7,
            request->rjoint1, request->rjoint2, request->rjoint3, request->rjoint4,
            request->rjoint5, request->rjoint6, request->rjoint7,
            request->vel, request->acc);

        std::string validation_error;
        if (!validateAbsoluteControlRequest(*request, validation_error)) {
            response->success = false;
            response->message = "Rejected joint absolute control request: " + validation_error;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        robot_control_msg::safety::RobotCommandGuard command_guard(
            "joint_absolute_control");
        if (!command_guard.acquired()) {
            response->success = false;
            response->message = "Rejected joint absolute control request: " +
                command_guard.error();
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::string execution_error;
        if (!feedbackReady(execution_error)) {
            response->success = false;
            response->message = "Cannot execute joint command: " + execution_error;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (!waitForArmToStop(execution_error)) {
            response->success = false;
            response->message = "Cannot execute joint command: " + execution_error;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        const Eigen::VectorXd target_positions = targetPositionsFromRequest(*request);

        auto cmd_msg = robot_control_msg::msg::Robotarmjoint();
        
        cmd_msg.ljoint1 = request->ljoint1;
        cmd_msg.ljoint2 = request->ljoint2;
        cmd_msg.ljoint3 = request->ljoint3;
        cmd_msg.ljoint4 = request->ljoint4;
        cmd_msg.ljoint5 = request->ljoint5;
        cmd_msg.ljoint6 = request->ljoint6;
        cmd_msg.ljoint7 = request->ljoint7;
        
        cmd_msg.rjoint1 = request->rjoint1;
        cmd_msg.rjoint2 = request->rjoint2;
        cmd_msg.rjoint3 = request->rjoint3;
        cmd_msg.rjoint4 = request->rjoint4;
        cmd_msg.rjoint5 = request->rjoint5;
        cmd_msg.rjoint6 = request->rjoint6;
        cmd_msg.rjoint7 = request->rjoint7;
        
        cmd_msg.vel = request->vel;
        cmd_msg.acc = request->acc;

        std::uint64_t motion_generation_before_command = 0;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            motion_generation_before_command = motion_status_generation_;
        }

        arm_cmd_pub_->publish(cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Published complete 14-joint target to /arm_joint_absolute_cmd");

        if (waitForArmToReachPosition(
                target_positions, motion_generation_before_command, execution_error)) {
            response->success = true;
            response->message = "Joint absolute control completed successfully";
            RCLCPP_INFO(
                this->get_logger(),
                "Motion completed successfully - controller stopped at target within %.4f rad",
                position_tolerance_);
        } else {
            response->success = false;
            response->message = "Motion failed: " + execution_error;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::string urdf_path_;
    std::string joint_state_topic_;
    std::string motion_status_topic_;
    std::unordered_map<std::string, int> joint_index_map_;
    Eigen::VectorXd q_current_;
    std::vector<int> left_arm_indices_;
    std::vector<int> right_arm_indices_;

    mutable std::mutex feedback_mutex_;
    std::condition_variable feedback_cv_;
    std::vector<bool> joint_position_received_;
    robot_control_msg::msg::ArmMotionStatus last_motion_status_;
    robot_control_msg::msg::ArmPowerStatus last_power_status_;
    robot_control_msg::msg::ArmControlModeStatus last_control_mode_status_;
    bool has_joint_state_{false};
    bool has_motion_status_{false};
    bool has_power_status_{false};
    bool has_control_mode_status_{false};
    std::uint64_t joint_state_generation_{0};
    std::uint64_t motion_status_generation_{0};
    std::chrono::steady_clock::time_point last_joint_state_time_{};
    std::chrono::steady_clock::time_point last_motion_status_time_{};
    std::chrono::steady_clock::time_point last_power_status_time_{};
    std::chrono::steady_clock::time_point last_control_mode_status_time_{};
    double position_tolerance_{0.01};
    int motion_start_timeout_ms_{3000};
    int motion_timeout_ms_{15000};
    int feedback_timeout_ms_{1000};
    int settled_feedback_ms_{200};
    
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr motion_status_sub_;
    rclcpp::Subscription<robot_control_msg::msg::ArmPowerStatus>::SharedPtr power_status_sub_;
    rclcpp::Subscription<robot_control_msg::msg::ArmControlModeStatus>::SharedPtr
        control_mode_status_sub_;
    rclcpp::Publisher<robot_control_msg::msg::Robotarmjoint>::SharedPtr arm_cmd_pub_;
    rclcpp::Service<robot_control_msg::srv::JointAbsoluteControl>::SharedPtr absolute_control_srv_;
};

} // namespace teleop_control

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<teleop_control::JointAbsoluteControlSrv>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

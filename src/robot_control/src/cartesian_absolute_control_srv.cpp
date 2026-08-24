#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <robot_control_msg/srv/cartesian_absolute_control.hpp>
#include <robot_control_msg/msg/cartesian_execution_status.hpp>
#include <robot_control_msg/msg/robotarmmovel.hpp>
#include <robot_control_msg/msg/robotarmservomsg.hpp>
#include <robot_control_msg/msg/arm_motion_status.hpp>
#include <robot_control_msg/robot_command_guard.hpp>
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
#include <thread>

namespace teleop_control
{

class CartesianAbsoluteControlSrv : public rclcpp::Node
{
public:
    CartesianAbsoluteControlSrv() : Node("cartesian_absolute_control_srv")
    {
        /* ------------------- 参数 ------------------- */
        urdf_path_ = this->declare_parameter<std::string>(
            "urdf_path", "");
        ee_frame_left_ = this->declare_parameter<std::string>("ee_frame_left", "lee_link");
        ee_frame_right_ = this->declare_parameter<std::string>("ee_frame_right", "ree_link");
        joint_state_topic_ = this->declare_parameter<std::string>(
            "joint_state_topic", "/arm/joint_states");
        motion_status_topic_ = this->declare_parameter<std::string>(
            "motion_status_topic", "/arm/arm_controller/motion_status");

        if (!loadModel())
        {
            rclcpp::shutdown();
            return;
        }

        /* ------------------- TCP变换矩阵初始化 ------------------- */
        tcp_transform_left_.setIdentity();
        tcp_transform_right_.setIdentity();

        /* ------------------- TCP参数声明 ------------------- */
        this->declare_parameter<double>("tcp_left_x", 0.0);
        this->declare_parameter<double>("tcp_left_y", 0.0);
        this->declare_parameter<double>("tcp_left_z", 0.0);
        this->declare_parameter<double>("tcp_left_roll", 0.0);
        this->declare_parameter<double>("tcp_left_pitch", 0.0);
        this->declare_parameter<double>("tcp_left_yaw", 0.0);
        
        this->declare_parameter<double>("tcp_right_x", 0.0);
        this->declare_parameter<double>("tcp_right_y", 0.0);
        this->declare_parameter<double>("tcp_right_z", 0.0);
        this->declare_parameter<double>("tcp_right_roll", 0.0);
        this->declare_parameter<double>("tcp_right_pitch", 0.0);
        this->declare_parameter<double>("tcp_right_yaw", 0.0);

        // 初始化参数事件处理器
        param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        
        // 注册参数回调
        auto on_tcp_param_change = [this]([[maybe_unused]] const rclcpp::Parameter &param) {
            updateTCPTransforms();
        };
        
        // 监控所有TCP相关参数
        tcp_param_cb_ = param_handler_->add_parameter_callback(
            "tcp_left_x", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_left_y", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_left_z", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_left_roll", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_left_pitch", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_left_yaw", on_tcp_param_change);
        
        param_handler_->add_parameter_callback("tcp_right_x", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_right_y", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_right_z", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_right_roll", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_right_pitch", on_tcp_param_change);
        param_handler_->add_parameter_callback("tcp_right_yaw", on_tcp_param_change);
        
        // 初始更新TCP变换
        updateTCPTransforms();

        /* ------------------- 创建回调组 ------------------- */
        // 为服务创建一个独立的互斥回调组，使其与话题回调不互相阻塞
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // 为话题创建一个独立的互斥回调组，也可使用可重入组，但互斥组已足够
        topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        /* ------------------- ROS ------------------- */
        // 使用选项将话题订阅绑定到 topic_cb_group_
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = topic_cb_group_;
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, 50,
            std::bind(&CartesianAbsoluteControlSrv::jointStateCallback, this, std::placeholders::_1),
            sub_options);

        arm_cmd_pub_ = this->create_publisher<robot_control_msg::msg::Robotarmmovel>(
            "/arm_cartrsian_position_cmd", 10);

        // 运动状态订阅也绑定到话题回调组
        rclcpp::SubscriptionOptions motion_sub_options;
        motion_sub_options.callback_group = topic_cb_group_;
        motion_status_sub_ = this->create_subscription<robot_control_msg::msg::ArmMotionStatus>(
            motion_status_topic_, 10,
            std::bind(&CartesianAbsoluteControlSrv::motionStatusCallback, this, std::placeholders::_1),
            motion_sub_options);

        // 添加笛卡尔执行状态订阅
        rclcpp::QoS execution_status_qos(1);
        execution_status_qos.reliable();
        execution_status_qos.transient_local();
        execution_status_sub_ = this->create_subscription<robot_control_msg::msg::CartesianExecutionStatus>(
            "/arm_cartesian_path_execution_status", execution_status_qos,
            std::bind(&CartesianAbsoluteControlSrv::executionStatusCallback, this, std::placeholders::_1),
            motion_sub_options);

        // 创建服务服务器，绑定到 service_cb_group_
        // 使用 rmw_qos_profile_services_default 作为 QoS 参数
        absolute_control_srv_ = this->create_service<robot_control_msg::srv::CartesianAbsoluteControl>(
            "cartesian_absolute_control",
            std::bind(&CartesianAbsoluteControlSrv::handleAbsoluteControlRequest, this, 
                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Cartesian Absolute Control Service is ready.");
    }

private:
    /* === URDF & 模型 === */
    bool loadModel()
    {
        try
        {
            pinocchio::urdf::buildModel(urdf_path_, model_);
            data_ = pinocchio::Data(model_);
            ee_id_left_ = model_.getFrameId(ee_frame_left_);
            ee_id_right_ = model_.getFrameId(ee_frame_right_);
            if (ee_id_left_ == (size_t)-1 || ee_id_right_ == (size_t)-1)
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot find end-effector frames in URDF");
                return false;
            }
            // map joint name -> index
            for (int i = 0; i < model_.nq; ++i)
            {
                std::string jname = model_.names[i + 1]; // +1 skip universe
                joint_index_map_[jname] = i;
            }
            
            // 定义双臂关节索引
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
            
            // 过滤掉连续关节的索引
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
            
            // 验证是否找到所有双臂关节
            if (left_arm_indices_.size() != 7 || right_arm_indices_.size() != 7) {
                RCLCPP_ERROR(this->get_logger(), "Failed to find all arm joints. Left: %zu, Right: %zu", 
                            left_arm_indices_.size(), right_arm_indices_.size());
                return false;
            }
            
            q_current_.setZero(model_.nq);
            
            // 初始化TCP变换矩阵
            tcp_transform_left_.setIdentity();
            tcp_transform_right_.setIdentity();
            
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "URDF load error: %s", e.what());
            return false;
        }
    }

    /* === 订阅回调（多线程环境，无锁，存在数据竞争风险） === */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = joint_index_map_.find(msg->name[i]);
            if (it != joint_index_map_.end() && i < msg->position.size())
            {
                q_current_[it->second] = msg->position[i];
                //RCLCPP_INFO(this->get_logger(), "Joint %s: pos=%.4f", msg->name[i].c_str(), q_current_[it->second]);
            }
        }
        has_joint_state_ = true;
    }

    void motionStatusCallback(const robot_control_msg::msg::ArmMotionStatus::SharedPtr msg)
    {
        last_motion_status_ = *msg;
        has_motion_status_ = true;
    }

    void executionStatusCallback(const robot_control_msg::msg::CartesianExecutionStatus::SharedPtr msg)
    {
        latest_execution_status_ = *msg;
        has_execution_status_ = true;
    }

    static bool isPlannerBusyState(uint8_t state)
    {
        return state == robot_control_msg::msg::CartesianExecutionStatus::PLANNING ||
               state == robot_control_msg::msg::CartesianExecutionStatus::EXECUTING;
    }

    bool waitForFeedbackReady(const std::chrono::seconds& timeout, std::string& error)
    {
        auto start_time = this->get_clock()->now();

        while (rclcpp::ok()) {
            if (has_joint_state_ && has_motion_status_ && has_execution_status_) {
                return true;
            }

            if ((this->get_clock()->now() - start_time) > rclcpp::Duration::from_seconds(timeout.count())) {
                error = "Timed out waiting for single MoveL feedback topics";
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        error = "Interrupted while waiting for single MoveL feedback topics";
        return false;
    }

    bool waitForPlannerIdle(const std::chrono::seconds& timeout, std::string& error)
    {
        auto start_time = this->get_clock()->now();

        while (rclcpp::ok()) {
            if (has_execution_status_ && !isPlannerBusyState(latest_execution_status_.state)) {
                return true;
            }

            if ((this->get_clock()->now() - start_time) > rclcpp::Duration::from_seconds(timeout.count())) {
                error = "Timed out waiting for single MoveL planner to become idle";
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        error = "Interrupted while waiting for single MoveL planner to become idle";
        return false;
    }

    bool waitForCommandAcceptance(
        const rclcpp::Time& command_time,
        const rclcpp::Duration& timeout,
        std::string& error)
    {
        auto start_time = this->get_clock()->now();

        while (rclcpp::ok()) {
            if (has_execution_status_) {
                const rclcpp::Time status_stamp(latest_execution_status_.stamp);
                if (status_stamp >= command_time) {
                    switch (latest_execution_status_.state) {
                        case robot_control_msg::msg::CartesianExecutionStatus::FAILED:
                            error = latest_execution_status_.message.empty() ?
                                "Single MoveL planning failed" :
                                latest_execution_status_.message;
                            return false;
                        case robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY:
                            error = latest_execution_status_.message.empty() ?
                                "Single MoveL planner is busy" :
                                latest_execution_status_.message;
                            return false;
                        case robot_control_msg::msg::CartesianExecutionStatus::EXECUTING:
                        case robot_control_msg::msg::CartesianExecutionStatus::STREAM_FINISHED:
                            return true;
                        case robot_control_msg::msg::CartesianExecutionStatus::PLANNING:
                        case robot_control_msg::msg::CartesianExecutionStatus::IDLE:
                        default:
                            break;
                    }
                }
            }

            if ((this->get_clock()->now() - start_time) > timeout) {
                error = "Timed out waiting for single MoveL command acceptance";
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        error = "Interrupted while waiting for single MoveL command acceptance";
        return false;
    }

    /* === TCP变换工具函数 === */
    static Eigen::Matrix3d rpyToRot(double r, double p, double y)
    {
        // 使用ZYX旋转顺序（先绕Z轴，再绕Y轴，最后绕X轴）
        Eigen::AngleAxisd ax(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ay(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd az(y, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = az * ay * ax;
        return q.toRotationMatrix();
    }

    /* === 更新TCP变换 === */
    void updateTCPTransforms()
    {
        // 读取左手TCP变换参数
        double tcp_left_x = this->get_parameter("tcp_left_x").as_double();
        double tcp_left_y = this->get_parameter("tcp_left_y").as_double();
        double tcp_left_z = this->get_parameter("tcp_left_z").as_double();
        double tcp_left_roll = this->get_parameter("tcp_left_roll").as_double();
        double tcp_left_pitch = this->get_parameter("tcp_left_pitch").as_double();
        double tcp_left_yaw = this->get_parameter("tcp_left_yaw").as_double();
        
        // 读取右手TCP变换参数
        double tcp_right_x = this->get_parameter("tcp_right_x").as_double();
        double tcp_right_y = this->get_parameter("tcp_right_y").as_double();
        double tcp_right_z = this->get_parameter("tcp_right_z").as_double();
        double tcp_right_roll = this->get_parameter("tcp_right_roll").as_double();
        double tcp_right_pitch = this->get_parameter("tcp_right_pitch").as_double();
        double tcp_right_yaw = this->get_parameter("tcp_right_yaw").as_double();
        
        // 计算旋转矩阵（ZYX顺序）
        Eigen::Matrix3d R_left = rpyToRot(tcp_left_roll, tcp_left_pitch, tcp_left_yaw);
        Eigen::Matrix3d R_right = rpyToRot(tcp_right_roll, tcp_right_pitch, tcp_right_yaw);
        
        // 更新TCP变换矩阵
        tcp_transform_left_.rotation() = R_left;
        tcp_transform_left_.translation() = Eigen::Vector3d(tcp_left_x, tcp_left_y, tcp_left_z);
        
        tcp_transform_right_.rotation() = R_right;
        tcp_transform_right_.translation() = Eigen::Vector3d(tcp_right_x, tcp_right_y, tcp_right_z);
        
        RCLCPP_INFO(this->get_logger(), "Updated TCP transforms:");
        RCLCPP_INFO(this->get_logger(), "Left TCP: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", 
                    tcp_left_x, tcp_left_y, tcp_left_z, tcp_left_roll, tcp_left_pitch, tcp_left_yaw);
        RCLCPP_INFO(this->get_logger(), "Right TCP: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", 
                    tcp_right_x, tcp_right_y, tcp_right_z, tcp_right_roll, tcp_right_pitch, tcp_right_yaw);
    }

    /* === 检查TCP是否到达目标位置（多线程读取，无锁） === */
    bool isTCPPoseReached(const Eigen::Vector3d& target_pos_L, const Eigen::Matrix3d& target_rot_L,
                         const Eigen::Vector3d& target_pos_R, const Eigen::Matrix3d& target_rot_R,
                         double pos_tolerance = 0.01, double rot_tolerance = 0.1)
    {
        pinocchio::forwardKinematics(model_, data_, q_current_);
        pinocchio::updateFramePlacements(model_, data_);

        auto M_ee_L = data_.oMf[ee_id_left_];
        auto M_tcp_L = M_ee_L * tcp_transform_left_;
        Eigen::Vector3d current_pos_L = M_tcp_L.translation();
        Eigen::Matrix3d current_rot_L = M_tcp_L.rotation();
        
        auto M_ee_R = data_.oMf[ee_id_right_];
        auto M_tcp_R = M_ee_R * tcp_transform_right_;
        Eigen::Vector3d current_pos_R = M_tcp_R.translation();
        Eigen::Matrix3d current_rot_R = M_tcp_R.rotation();
        
        // 位置误差
        double pos_error_L = (current_pos_L - target_pos_L).norm();
        double pos_error_R = (current_pos_R - target_pos_R).norm();
        
        // 姿态误差：计算从当前姿态到目标姿态所需的增量，然后检查增量是否接近0
        Eigen::Matrix3d rot_error_L = target_rot_L.transpose() * current_rot_L;
        Eigen::AngleAxisd angle_axis_L(rot_error_L);
        double rot_error_L_rad = std::abs(angle_axis_L.angle());
        
        Eigen::Matrix3d rot_error_R = target_rot_R.transpose() * current_rot_R;
        Eigen::AngleAxisd angle_axis_R(rot_error_R);
        double rot_error_R_rad = std::abs(angle_axis_R.angle());
        
        bool left_ok = (pos_error_L <= pos_tolerance) && (rot_error_L_rad <= rot_tolerance);
        bool right_ok = (pos_error_R <= pos_tolerance) && (rot_error_R_rad <= rot_tolerance);

        RCLCPP_DEBUG(this->get_logger(),
            "TCP Pose Check - L pos_err: %.4f (tol: %.4f), rot_err: %.4f rad (tol: %.4f rad) -> %s, "
            "R pos_err: %.4f (tol: %.4f), rot_err: %.4f rad (tol: %.4f rad) -> %s",
            pos_error_L, pos_tolerance, rot_error_L_rad, rot_tolerance, left_ok ? "OK" : "NOT OK",
            pos_error_R, pos_tolerance, rot_error_R_rad, rot_tolerance, right_ok ? "OK" : "NOT OK");
        
        return left_ok && right_ok;
    }

    /* === 等待机械臂静止（服务回调线程阻塞，但其他线程仍可处理话题回调） === */
    bool waitForArmToStop(const std::chrono::seconds& timeout = std::chrono::seconds(6))
    {
        auto start_time = this->get_clock()->now();
        
        while (rclcpp::ok()) {
            if (!last_motion_status_.is_moving) {
                return true; // 机械臂已停止
            }
            
            if ((this->get_clock()->now() - start_time) > timeout) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm to stop");
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }

    /* === 等待机械臂到达目标（服务回调线程阻塞，但其他线程仍可处理话题回调） === */
    bool waitForArmToReachGoal(const std::chrono::seconds& timeout = std::chrono::seconds(6))
    {
        auto start_time = this->get_clock()->now();
        
        while (rclcpp::ok()) {
            if (!last_motion_status_.is_moving) {
                if (last_motion_status_.goal_reached) {
                    return true; // 机械臂已到达目标
                } else {
                    return false; // 机械臂停止但未到达目标
                }
            }
            
            if ((this->get_clock()->now() - start_time) > timeout) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm to reach goal");
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }

    /* === 等待机械臂到达目标（基于TCP位姿检查） === */
    bool waitForArmToReachGoalByPose(const Eigen::Vector3d& target_pos_L, const Eigen::Matrix3d& target_rot_L,
                                    const Eigen::Vector3d& target_pos_R, const Eigen::Matrix3d& target_rot_R,
                                    const std::chrono::seconds& timeout = std::chrono::seconds(6))
    {
        auto start_time = this->get_clock()->now();
        
        while (rclcpp::ok()) {
            if (isTCPPoseReached(target_pos_L, target_rot_L, target_pos_R, target_rot_R)) {
                return true;
            }
            
            if ((this->get_clock()->now() - start_time) > timeout) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm to reach goal by pose");
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }

    /* === 等待机械臂到达目标（服务回调线程阻塞，但其他线程仍可处理话题回调） === */
    bool waitForArmToReachGoal(const Eigen::Vector3d& target_pos_L, const Eigen::Matrix3d& target_rot_L,
                              const Eigen::Vector3d& target_pos_R, const Eigen::Matrix3d& target_rot_R,
                              const std::chrono::seconds& timeout = std::chrono::seconds(6))
    {
        auto start_time = this->get_clock()->now();
        
        // 使用planned_duration_sec作为超时时间，如果可用的话
        std::chrono::seconds actual_timeout = timeout;
        if (has_execution_status_ && latest_execution_status_.planned_duration_sec > 0) {
            actual_timeout = std::chrono::seconds(static_cast<int64_t>(
                std::ceil(latest_execution_status_.planned_duration_sec * 1.5))); // 增加50%的安全余量
            RCLCPP_INFO(this->get_logger(), "Using planned duration %.2f sec as timeout (with 50%% margin): %ld sec", 
                       latest_execution_status_.planned_duration_sec, actual_timeout.count());
        }
        
        while (rclcpp::ok()) {
            if (isTCPPoseReached(target_pos_L, target_rot_L, target_pos_R, target_rot_R)) {
                return true;
            }
            
            if ((this->get_clock()->now() - start_time) > actual_timeout) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm to reach goal (timeout: %ld sec)", actual_timeout.count());
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }

    /* === 服务回调 === */
    void handleAbsoluteControlRequest(
        const std::shared_ptr<robot_control_msg::srv::CartesianAbsoluteControl::Request> request,
        std::shared_ptr<robot_control_msg::srv::CartesianAbsoluteControl::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received cartesian absolute control request");

        const bool all_zero = (std::abs(request->lx) + std::abs(request->ly) + std::abs(request->lz) < 1e-6) &&
                              (std::abs(request->rx) + std::abs(request->ry) + std::abs(request->rz) < 1e-6) &&
                              (std::abs(request->lqx) + std::abs(request->lqy) + std::abs(request->lqz) + std::abs(request->lqw) < 1e-6) &&
                              (std::abs(request->rqx) + std::abs(request->rqy) + std::abs(request->rqz) + std::abs(request->rqw) < 1e-6) &&
                              (std::abs(request->lroll) + std::abs(request->lpitch) + std::abs(request->lyaw) < 1e-6) &&
                              (std::abs(request->rroll) + std::abs(request->rpitch) + std::abs(request->ryaw) < 1e-6);

        if (all_zero) {
            response->success = true;
            response->message = "No movement requested, already at target";
            RCLCPP_INFO(this->get_logger(), "No movement requested, returning success immediately");
            return;
        }

        robot_control_msg::safety::RobotCommandGuard command_guard(
            "legacy_cartesian_absolute_control");
        if (!command_guard.acquired()) {
            response->success = false;
            response->message = command_guard.error();
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::string status_error;
        if (!waitForFeedbackReady(std::chrono::seconds(5), status_error)) {
            response->success = false;
            // 在错误消息中包含当前状态信息
            std::string detailed_error = status_error;
            if (has_execution_status_) {
                detailed_error += ". Current execution status: state=" + 
                    std::to_string(latest_execution_status_.state) + 
                    ", planned_duration=" + 
                    std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
                    ", message='" + latest_execution_status_.message + "'";
            }
            response->message = detailed_error;
            RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
            return;
        }

        if (!waitForPlannerIdle(std::chrono::seconds(6), status_error)) {
            response->success = false;
            // 在错误消息中包含当前状态信息
            std::string detailed_error = status_error;
            if (has_execution_status_) {
                detailed_error += ". Current execution status: state=" + 
                    std::to_string(latest_execution_status_.state) + 
                    ", planned_duration=" + 
                    std::to_string(latest_execution_status_.planned_duration_sec) + "s" +
                    ", message='" + latest_execution_status_.message + "'";
            }
            response->message = detailed_error;
            RCLCPP_ERROR(this->get_logger(), "%s", detailed_error.c_str());
            return;
        }

        // 计算目标姿态
        Eigen::Matrix3d target_rot_L;
        if (std::abs(request->lqx) + std::abs(request->lqy) + std::abs(request->lqz) + std::abs(request->lqw) > 1e-6) {
            Eigen::Quaterniond q(request->lqw, request->lqx, request->lqy, request->lqz);
            target_rot_L = q.toRotationMatrix();
        } else {
            target_rot_L = rpyToRot(request->lroll, request->lpitch, request->lyaw);
        }
        
        Eigen::Matrix3d target_rot_R;
        if (std::abs(request->rqx) + std::abs(request->rqy) + std::abs(request->rqz) + std::abs(request->rqw) > 1e-6) {
            Eigen::Quaterniond q(request->rqw, request->rqx, request->rqy, request->rqz);
            target_rot_R = q.toRotationMatrix();
        } else {
            target_rot_R = rpyToRot(request->rroll, request->rpitch, request->ryaw);
        }

        // 2. 直接发布绝对位置控制命令
        auto cmd_msg = robot_control_msg::msg::Robotarmmovel();
        
        // 左臂目标位姿（直接使用请求中的值）
        cmd_msg.lx = request->lx;
        cmd_msg.ly = request->ly;
        cmd_msg.lz = request->lz;
        
        cmd_msg.lroll = request->lroll;
        cmd_msg.lpitch = request->lpitch;
        cmd_msg.lyaw = request->lyaw;
        
        cmd_msg.lqx = request->lqx;
        cmd_msg.lqy = request->lqy;
        cmd_msg.lqz = request->lqz;
        cmd_msg.lqw = request->lqw;
        
        // 右臂目标位姿（直接使用请求中的值）
        cmd_msg.rx = request->rx;
        cmd_msg.ry = request->ry;
        cmd_msg.rz = request->rz;
        
        cmd_msg.rroll = request->rroll;
        cmd_msg.rpitch = request->rpitch;
        cmd_msg.ryaw = request->ryaw;
        
        cmd_msg.rqx = request->rqx;
        cmd_msg.rqy = request->rqy;
        cmd_msg.rqz = request->rqz;
        cmd_msg.rqw = request->rqw;
        
        // 速度和加速度参数
        cmd_msg.vel = request->vel;
        cmd_msg.acc = request->acc;

        const rclcpp::Time command_time = this->now();
        arm_cmd_pub_->publish(cmd_msg);

        if (!waitForCommandAcceptance(
                command_time,
                rclcpp::Duration::from_seconds(6),
                status_error)) {
            response->success = false;
            // 在错误消息中包含当前状态信息
            std::string detailed_error = status_error;
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
            RCLCPP_ERROR(this->get_logger(), "Motion failed during absolute control: %s", detailed_error.c_str());
            return;
        }

        // 6. 等待运动完成（基于TCP位姿检查）
        if (waitForArmToReachGoal(
                Eigen::Vector3d(request->lx, request->ly, request->lz), target_rot_L,
                Eigen::Vector3d(request->rx, request->ry, request->rz), target_rot_R)) {
            response->success = true;
            response->message = "Cartesian absolute control completed successfully";
            RCLCPP_INFO(this->get_logger(), "Motion completed successfully");
        } else {
            response->success = false;
            // 在错误消息中包含当前状态信息
            std::string detailed_error = "Motion failed: Robot did not reach goal or timed out";
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
            RCLCPP_ERROR(this->get_logger(), "Motion failed during absolute control: %s", detailed_error.c_str());
        }
    }

    /* === 成员变量 === */
    // 模型相关
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::string urdf_path_;
    std::string ee_frame_left_, ee_frame_right_;
    std::string joint_state_topic_;
    std::string motion_status_topic_;
    pinocchio::FrameIndex ee_id_left_, ee_id_right_;
    std::unordered_map<std::string, int> joint_index_map_;
    Eigen::VectorXd q_current_;  // 多线程读写，无锁保护
    
    // TCP变换
    pinocchio::SE3 tcp_transform_left_;
    pinocchio::SE3 tcp_transform_right_;
    
    // 关节索引
    std::vector<int> left_arm_indices_;
    std::vector<int> right_arm_indices_;
    
    // 状态管理
    robot_control_msg::msg::ArmMotionStatus last_motion_status_;
    robot_control_msg::msg::CartesianExecutionStatus latest_execution_status_;
    bool has_joint_state_ {false};
    bool has_motion_status_ {false};
    bool has_execution_status_ {false};
    
    // 参数处理
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> tcp_param_cb_;
    
    // ROS相关
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<robot_control_msg::msg::ArmMotionStatus>::SharedPtr motion_status_sub_;
    rclcpp::Subscription<robot_control_msg::msg::CartesianExecutionStatus>::SharedPtr execution_status_sub_;
    rclcpp::Publisher<robot_control_msg::msg::Robotarmmovel>::SharedPtr arm_cmd_pub_;
    rclcpp::Service<robot_control_msg::srv::CartesianAbsoluteControl>::SharedPtr absolute_control_srv_;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
};

} // namespace teleop_control

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<teleop_control::CartesianAbsoluteControlSrv>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

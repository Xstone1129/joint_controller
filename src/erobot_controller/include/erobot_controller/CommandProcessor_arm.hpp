#include <array>
#include <chrono>
#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "erobot_controller/CubicPlanner.hpp"
#include "robot_control_msg/msg/robotarmservomsg.hpp"
#include "robot_control_msg/msg/robotarmmovel.hpp"
#include "robot_control_msg/msg/robotarmjoint.hpp"
#include "robot_control_msg/msg/arm_motion_status.hpp"
#include "robot_control_msg/msg/arm_power_status.hpp"
#include "robot_control_msg/msg/arm_control_mode_status.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include "erobot_controller/traj.h"
// 添加Pinocchio相关头文件
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <atomic>
#ifndef EROBOT_CONTROLLER_COMMAND_PROCESSOR_ARM_HPP
#define EROBOT_CONTROLLER_COMMAND_PROCESSOR_ARM_HPP
using namespace ruckig;

#define MOTOR_STATUS_DISABLED 64 // 下使能
#define MOTOR_STATUS_ENABLED 39 // 上使能
inline bool is_motor_enabled(double v) {
  return static_cast<int>(std::llround(v)) == MOTOR_STATUS_ENABLED;
}
inline bool is_motor_disabled(double v) {
  return static_cast<int>(std::llround(v)) == MOTOR_STATUS_DISABLED;
}


struct MotorCoefficients {
  double kp;       // 比例增益
  double kd;       // 微分增益
  double max_effort; // 最大力矩
  double Torqueconstant; // 转矩常数
};

struct MotorState {
    double position = 0;        // rad
    double velocity = 0;        // rad/s
    double acceleration = 0;    // rad/s^2
    double effort = 0;          // Nm
    double power_enable = 0;    // 电机上下使能（如硬件提供）
    double motor_status = 0;    // 电机状态码（来自status接口），例如 39=上使能，64=下使能
    double error_code = 0;      // CiA402 0x603F error code
};

// 机械臂控制模式：决定底层是位置控制还是力矩控制
enum class ControlMode {
    POSITION = 0,
    EFFORT = 1
};

// 位置控制下的插补模式
enum class InterpolationMode {
    CUBIC_POLYNOMIAL = 0,
    RUCKIG = 1
};

// 下发到底层硬件接口的模式值
enum class HardwareCommandMode {
    CUBIC_POLYNOMIAL = 0,
    RUCKIG = 1,
    VELOCITY = 2,
    EFFORT = 3,
    // The setpoint was time-parameterized upstream. It maps to CSP but must
    // never enter either of the local point-to-point planners.
    EXTERNAL_POSITION_STREAM = 5
};

// 机器人状态机（运行时状态）
enum class RobotState {
    Idle = 0,
    ExecutingJoint,
    Stopped,
    Error
};

class CommandProcessor_arm  : public rclcpp::Node{
private:
    std::mutex get_pos_mutex_; // 保护 get_pos_ 数组的互斥锁

public:
    struct JointLimitGuardConfig {
        bool enabled = false;
        double soft_zone_deg = 6.0;
        double stiffness_nm_per_rad = 120.0;
        double damping_nm_s_per_rad = 6.0;
        double overrun_stiffness_scale = 3.0;
        double overrun_damping_scale = 2.0;
        double max_torque_ratio = 0.35;
    };

    // 原有构造：自行从参数服务器读取（保持兼容）
    CommandProcessor_arm(size_t num_joints, const std::vector<std::string>& joint_names);
    // 新增构造：由控制器把已解析好的参数传入，避免在此节点上声明/获取参数
    CommandProcessor_arm(size_t num_joints,
                         const std::vector<std::string>& joint_names,
                         const std::vector<std::string>& compensation_joint_names,
                         const JointLimitGuardConfig& joint_limit_guard_config,
                         const std::string& urdf_filename,
                         const std::string& left_ee_link,
                         const std::string& right_ee_link,
                         double offset_x = 0.0,
                         double offset_y = 0.0,
                         double offset_z = 0.0);
    
    // 核心方法：处理新消息并更新命令
    bool processNewCommand(robot_control_msg::msg::Robotarmservomsg& new_command);
    bool processNewArmJointCommand(robot_control_msg::msg::Robotarmjoint& new_command);
    bool processHeavyPositionCommand(
        const std::array<double, 14>& position,
        const std::array<double, 14>& target_velocity,
        const std::array<double, 14>& target_acceleration,
        bool velocity_provided,
        bool acceleration_provided,
        double max_velocity,
        double max_acceleration,
        std::string* error = nullptr);
    bool processHeavyHoldPositionCommand(
        const std::array<double, 14>& position, std::string* error = nullptr);
    bool requestPositionHold(const std::string& reason);
    void clearHeavyPositionStream();
    bool requestControlMode(ControlMode mode, std::string* message = nullptr);
    bool requestPowerEnable(bool enable, std::string* message = nullptr);
    bool isPowerEnabledRequested() const { return power_enabled_; }
    bool isHardwareAvailable() const { return hardware_available_; }
    bool areAllMotorsEnabled() const { return all_enabled_; }
    ControlMode getActiveControlMode() const;
    uint8_t getActiveControlModeValue() const;
    uint8_t getRequestedControlModeValue() const;
    bool isPositionCommandReady() const;

    // 更新关节位置命令
    void updateJointCommands(
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& position_states,
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& position_commands,
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& velocity_commands,
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& effort_commands,
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& power_commands,
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& run_mode_commands,
        bool allow_disabled_simulation_execution
    );
    void updateJointState(
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& position_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& velocity_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& motor_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& error_code_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& compensation_position_states) ;
    
    // 重置方法
    void reset();
    void command_arm_plan(const rclcpp::Time & time);
    std::vector<MotorCoefficients> motor_coeffs_;

    // 状态查询（单个函数返回运动/模式/状态）
    struct ArmStatus {
        bool is_moving;
        ControlMode control_mode;
        InterpolationMode interpolation_mode;
        RobotState state;
        std::string control_mode_name;
        std::string interpolation_mode_name;
        std::string state_name;
    };
    ArmStatus getArmStatus() const;
    static const char* controlModeName(ControlMode m);
    static const char* interpolationModeName(InterpolationMode m);

private:
    struct ArmJointModelIndex {
        std::string name;
        pinocchio::JointIndex joint_id = 0;
        int idx_q = -1;
        int nq = 0;
        int idx_v = -1;
        int nv = 0;
    };

    // 公共初始化逻辑（在参数就绪后调用）
    void initializeCore_();
    void initializeArmJointMappings_();
    void initializeCompensationJointMappings_();
    void initializeJointMappings_(
        const std::vector<std::string>& joint_names,
        std::vector<ArmJointModelIndex>& joint_mappings,
        const char* joint_group_name);
    void logGravityCompensationDebug_(
        const std::vector<double>& raw_torques,
        const std::vector<double>& scaled_torques);
    void resetPinocchioStateToNeutral_();
    void computeAndLogArmTorques_();
    double computeJointLimitGuardTorqueNm_(size_t joint_index) const;
    double clampTorqueToMotorLimitNm_(double torque_nm, size_t joint_index) const;
    double convertTorqueToEffortCommand_(double torque_nm, size_t joint_index) const;
    double lowerPositionLimitForArmJoint_(size_t joint_index) const;
    double upperPositionLimitForArmJoint_(size_t joint_index) const;
    double clampPositionToArmJointLimit_(size_t joint_index, double position) const;
    void clampArmTargetsToJointLimits_(std::array<double, 14>& positions) const;
    static double wrappedAbsPositionError_(double current, double target);
    bool shouldDisableRuckigAxis_(size_t joint_index, double target_position) const;
    bool updateInterpolationModeFromMsg_(const robot_control_msg::msg::Robotarmservomsg& new_command);
    bool isFreshPositionStamp_(const builtin_interfaces::msg::Time& stamp) const;
    void syncPositionTargetsToCurrent_();
    void resetCubicPlannerToCurrent_();
    void resetRuckigPlanToCurrent_();
    void transitionPositionHoldToWait_();
    HardwareCommandMode currentHardwareCommandMode_() const;

    enum class PositionControlState {
        POSITION_ACTIVE = 0,
        EFFORT_ACTIVE,
        EFFORT_TO_POSITION_HOLD,
        WAIT_FRESH_POSITION_TARGET
    };

    const size_t num_joints_;
    std::array<double, 14> get_pos_{0}; // 所有元素初始化为0.0
    // Target derivatives belong to the current Ruckig position plan.  They
    // are explicitly reset for every command so a missing Heavy field cannot
    // reuse a derivative from an older sequence.
    std::array<double, 14> planned_target_velocity_{};
    std::array<double, 14> planned_target_acceleration_{};
    std::array<MotorState, 14> external_stream_target_{};
    bool external_position_stream_active_{false};
    double run_time_ = 0.0;
    bool power_enabled_ = false;
    bool all_enabled_ = false; // 全部轴上使能聚合标志all_enabled
    bool hardware_available_ = false; // 每个驱动都提供了非零、可确认的状态码
    bool position_targets_initialized_ = false; // 位置控制目标是否已和当前姿态同步
    std::vector<MotorState> MotorState_desired_,MotorState_current_;


    std::vector<erobot_controller::TrajectoryPoint> trajectory_points;
    // stamp;
    // 轨迹规划器相关成员
    std::unique_ptr<erobot_controller::MultiAxisCubicPlanner> planner_;
    std::vector<erobot_controller::MotionConstraints> constraints_list_;
    bool CubicPlanner_planner_initialized_ = false;
    double start_time_ = 0.0;  // 轨迹开始时间戳（s），预分配成员，避免临时变量

    std::vector<bool> need_plan;
    InterpolationMode interpolation_mode_ = InterpolationMode::RUCKIG;
    ControlMode active_control_mode_ = ControlMode::POSITION;
    ControlMode requested_control_mode_ = ControlMode::POSITION;
    PositionControlState position_control_state_ = PositionControlState::POSITION_ACTIVE;
    rclcpp::Time position_reentry_time_{0, 0, RCL_ROS_TIME};
    RobotState robot_state_ = RobotState::Idle;   // 运行状态

    // 持位控制
    std::vector<double> hold_pos_;
    std::vector<bool> hold_active_;

    // 状态机与工具函数封装
    void setRobotState(RobotState s, const std::string& reason = "");
    static const char* robotStateName(RobotState s);

    // 角度差与限位检查
    bool checkLimitMargin(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double margin_deg);
    bool checkAngleDiffExceeded(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double max_angle_diff_deg);

    // 应用/保持关节命令
    void applySolutionToMotor(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double dt_control);
    void holdCurrentForArm(int start, int end_exclusive);
    void activateHoldForArm(int start, int end_exclusive);
    void deactivateHoldForArm(int start, int end_exclusive);
    void deactivateHoldAll();

    Ruckig<14> otg_joint_plan; // 创建轨迹生成器
    InputParameter<14> input_joint_plan; // 输入参数
    OutputParameter<14> output_joint_plan; // 输出参数

    //pinocchio
    pinocchio::Model model_;                   // 机器人模型
    pinocchio::Data data_;                     // 模型数据

    // 配置参数
    std::string urdf_filename_;                // URDF文件路径
    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> compensation_joint_names_;
    JointLimitGuardConfig joint_limit_guard_config_;
    std::vector<ArmJointModelIndex> arm_joint_mappings_;
    std::vector<ArmJointModelIndex> compensation_joint_mappings_;
    std::vector<double> compensation_joint_positions_;

    // 预分配的状态与缓存
    Eigen::VectorXd q_;                        // 关节位置向量
    Eigen::VectorXd q_neutral_;                // Pinocchio合法中性位姿
    Eigen::VectorXd v_;                        // 关节速度向量
    Eigen::VectorXd a_;                        // 关节加速度向量
    Eigen::VectorXd tau_;                      // 计算得到的关节力矩
    Eigen::VectorXd q_sol_;                    // 本周期求解用配置向量（复用）

    rclcpp::Time last_update_time_;
    std::chrono::steady_clock::time_point last_gravity_debug_log_time_{};

    // 参数回调句柄（用于运行时参数更新）
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 运动状态发布者
    rclcpp::Publisher<robot_control_msg::msg::ArmMotionStatus>::SharedPtr motion_status_publisher_;
    rclcpp::Publisher<robot_control_msg::msg::ArmPowerStatus>::SharedPtr power_status_publisher_;
    rclcpp::Publisher<robot_control_msg::msg::ArmControlModeStatus>::SharedPtr control_mode_status_publisher_;
    std::chrono::steady_clock::time_point last_power_status_publish_time_{};
    std::chrono::steady_clock::time_point last_control_mode_status_publish_time_{};
    
    // 状态判断相关
    static constexpr double GOAL_TOLERANCE = 0.00001;  // 目标到达容差（弧度）
    static constexpr double RUCKIG_DISABLE_VEL_TOLERANCE = 0.001;
    static constexpr double RUCKIG_DISABLE_ACC_TOLERANCE = 0.5;
    static constexpr double REPEATED_TARGET_DEADBAND = 1e-3;
    static constexpr double MIN_TORQUE_CONSTANT = 1e-9;
    static constexpr std::chrono::milliseconds GRAVITY_DEBUG_LOG_PERIOD{10000};
}; 
#endif  // EROBOT_CONTROLLER_COMMAND_PROCESSOR_ARM_HPP

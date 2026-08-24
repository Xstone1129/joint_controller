#include "erobot_controller/CommandProcessor_arm.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <unordered_set>
using namespace ruckig;

namespace
{
constexpr int kUnknownStatusCode = 0;

std::string defaultArmUrdfPath()
{
    return ament_index_cpp::get_package_share_directory("robot_arm_description") + "/urdf/right_left_arm.urdf";
}
}  // namespace

/**
 * @brief 机械臂命令处理器构造函数（旧：自取参数）
 * @param num_joints 关节数量
 */
CommandProcessor_arm::CommandProcessor_arm(size_t num_joints, const std::vector<std::string>& joint_names)
    : rclcpp::Node("command_processor"), num_joints_(num_joints), otg_joint_plan(0.002)
{
    const JointLimitGuardConfig joint_limit_guard_defaults{};

    // 声明并获取参数（保持兼容：允许直接从参数服务器读取）
    this->declare_parameter<std::string>("urdf_filename", defaultArmUrdfPath());
    this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>(
        "gravity_compensation_joints",
        std::vector<std::string>{});
    this->declare_parameter<bool>("joint_limit_guard.enabled", joint_limit_guard_defaults.enabled);
    this->declare_parameter<double>("joint_limit_guard.soft_zone_deg", joint_limit_guard_defaults.soft_zone_deg);
    this->declare_parameter<double>(
        "joint_limit_guard.stiffness_nm_per_rad",
        joint_limit_guard_defaults.stiffness_nm_per_rad);
    this->declare_parameter<double>(
        "joint_limit_guard.damping_nm_s_per_rad",
        joint_limit_guard_defaults.damping_nm_s_per_rad);
    this->declare_parameter<double>(
        "joint_limit_guard.overrun_stiffness_scale",
        joint_limit_guard_defaults.overrun_stiffness_scale);
    this->declare_parameter<double>(
        "joint_limit_guard.overrun_damping_scale",
        joint_limit_guard_defaults.overrun_damping_scale);
    this->declare_parameter<double>(
        "joint_limit_guard.max_torque_ratio",
        joint_limit_guard_defaults.max_torque_ratio);
    this->get_parameter("urdf_filename", urdf_filename_);
    this->get_parameter("joints", arm_joint_names_);
    this->get_parameter("gravity_compensation_joints", compensation_joint_names_);
    this->get_parameter("joint_limit_guard.enabled", joint_limit_guard_config_.enabled);
    this->get_parameter("joint_limit_guard.soft_zone_deg", joint_limit_guard_config_.soft_zone_deg);
    this->get_parameter(
        "joint_limit_guard.stiffness_nm_per_rad",
        joint_limit_guard_config_.stiffness_nm_per_rad);
    this->get_parameter(
        "joint_limit_guard.damping_nm_s_per_rad",
        joint_limit_guard_config_.damping_nm_s_per_rad);
    this->get_parameter(
        "joint_limit_guard.overrun_stiffness_scale",
        joint_limit_guard_config_.overrun_stiffness_scale);
    this->get_parameter(
        "joint_limit_guard.overrun_damping_scale",
        joint_limit_guard_config_.overrun_damping_scale);
    this->get_parameter("joint_limit_guard.max_torque_ratio", joint_limit_guard_config_.max_torque_ratio);
    if (!joint_names.empty()) {
        arm_joint_names_ = joint_names;
    }

    initializeCore_();
}

/**
 * @brief 机械臂命令处理器构造函数（新：外部传参）
 */
CommandProcessor_arm::CommandProcessor_arm(size_t num_joints,
                         const std::vector<std::string>& joint_names,
                         const std::vector<std::string>& compensation_joint_names,
                         const JointLimitGuardConfig& joint_limit_guard_config,
                         const std::string& urdf_filename,
                         const std::string& left_ee_link,
                         const std::string& right_ee_link,
                         double offset_x,
                         double offset_y,
                         double offset_z)
    : rclcpp::Node("command_processor"), num_joints_(num_joints), otg_joint_plan(0.002)
{
    (void)left_ee_link;
    (void)right_ee_link;
    (void)offset_x;
    (void)offset_y;
    (void)offset_z;
    arm_joint_names_ = joint_names;
    compensation_joint_names_ = compensation_joint_names;
    joint_limit_guard_config_ = joint_limit_guard_config;
    urdf_filename_ = urdf_filename;
    initializeCore_();
}

// 公共初始化逻辑（参数准备好之后调用）
void CommandProcessor_arm::initializeCore_() {
    // 检查必要的参数是否存在
    if (urdf_filename_.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'urdf_filename' is not set. Please provide a URDF file path.");
        throw std::runtime_error("Missing required parameter: urdf_filename");
    }

    RCLCPP_INFO(this->get_logger(), "Loading URDF from: %s", urdf_filename_.c_str());
    try {
        pinocchio::urdf::buildModel(urdf_filename_, model_);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Pinocchio buildModel failed: ") + e.what());
    }

    try {
        data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Pinocchio Data(model) failed: ") + e.what());
    }

    initializeArmJointMappings_();
    initializeCompensationJointMappings_();
    
    // 初始化关节状态向量
    try {
        q_neutral_ = pinocchio::neutral(model_);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Pinocchio neutral(model) failed: ") + e.what());
    }
    q_.resize(model_.nq);
    v_.resize(model_.nv);
    a_.resize(model_.nv);
    tau_.resize(model_.nv);
    
    q_ = q_neutral_;
    v_.setZero();
    a_.setZero();
    tau_.setZero();

    compensation_joint_positions_.resize(compensation_joint_mappings_.size(), 0.0);
    for (size_t i = 0; i < compensation_joint_mappings_.size(); ++i) {
        compensation_joint_positions_[i] = q_neutral_(compensation_joint_mappings_[i].idx_q);
    }



    // 初始化存储关节状态和轨迹点的向量
    MotorState_desired_.resize(num_joints_);
    MotorState_current_.resize(num_joints_);
    trajectory_points.resize(num_joints_);
    reset();
    
    // 初始化持位数组
    hold_pos_.resize(num_joints_);
    hold_active_.resize(num_joints_, false);
    for (size_t i = 0; i < num_joints_; ++i) hold_pos_[i] = 0.0;
    
    // 初始化运动约束列表
    constraints_list_.resize(num_joints_);
    motor_coeffs_.resize(num_joints_);
    
    // 设置默认速度和加速度约束
    for (size_t i = 0; i < num_joints_; i++) {
        constraints_list_[i].max_velocity = 1.0;     // 最大速度 4 rad/s
        constraints_list_[i].max_acceleration = 5.0; // 最大加速度 10 rad/s²

        constraints_list_[i].min_position = lowerPositionLimitForArmJoint_(i);
        constraints_list_[i].max_position = upperPositionLimitForArmJoint_(i);
    }

    RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "num_joints_: %zu", constraints_list_.size());
    
    // 创建轨迹规划器实例
    planner_ = std::make_unique<erobot_controller::MultiAxisCubicPlanner>(num_joints_, constraints_list_);
    CubicPlanner_planner_initialized_ = false;

    // 初始化关节规划标志向量
    need_plan.resize(num_joints_, false);
    
    // 设置Ruckig轨迹生成器的运动约束
    std::fill(input_joint_plan.max_velocity.begin(), input_joint_plan.max_velocity.end(), 3.14f);    // 最大速度 (rad/s)
    std::fill(input_joint_plan.max_acceleration.begin(), input_joint_plan.max_acceleration.end(), 60.0f); // 最大加速度 (rad/s²)
    std::fill(input_joint_plan.max_jerk.begin(), input_joint_plan.max_jerk.end(), 60.0f);           // 最大加加速度 (rad/s³)
    //速度和加速度在60最高，但是会出现抖动。
    // 设置为相位同步模式，让每个轴独立运行
    std::fill(input_joint_plan.enabled.begin(), input_joint_plan.enabled.end(), true);
    input_joint_plan.synchronization = Synchronization::Phase;
    
    RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), 
                "Ruckig initialized with non-synchronized mode");

    last_update_time_ = rclcpp::Time(0);
    
    // 初始化运动状态发布者
    motion_status_publisher_ = this->create_publisher<robot_control_msg::msg::ArmMotionStatus>("arm_controller/motion_status", 10);
    power_status_publisher_ = this->create_publisher<robot_control_msg::msg::ArmPowerStatus>(
        "power_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    control_mode_status_publisher_ =
        this->create_publisher<robot_control_msg::msg::ArmControlModeStatus>(
        "control_mode_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
}

void CommandProcessor_arm::initializeArmJointMappings_() {
    if (arm_joint_names_.size() != num_joints_) {
        std::ostringstream oss;
        oss << "Expected " << num_joints_ << " arm joint names, got " << arm_joint_names_.size();
        throw std::runtime_error(oss.str());
    }

    initializeJointMappings_(arm_joint_names_, arm_joint_mappings_, "Arm");
}

void CommandProcessor_arm::initializeCompensationJointMappings_() {
    initializeJointMappings_(compensation_joint_names_, compensation_joint_mappings_, "Gravity compensation");
}

void CommandProcessor_arm::initializeJointMappings_(
    const std::vector<std::string>& joint_names,
    std::vector<ArmJointModelIndex>& joint_mappings,
    const char* joint_group_name) {
    joint_mappings.clear();
    joint_mappings.reserve(joint_names.size());

    std::unordered_set<std::string> seen_joint_names;
    for (const auto& joint_name : joint_names) {
        if (joint_name.empty()) {
            throw std::runtime_error(std::string(joint_group_name) + " joint name is empty");
        }
        if (!seen_joint_names.insert(joint_name).second) {
            throw std::runtime_error(std::string(joint_group_name) + " joint name is duplicated: " + joint_name);
        }

        const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
        if (joint_id == 0 || joint_id >= static_cast<pinocchio::JointIndex>(model_.joints.size()) ||
            model_.names[joint_id] != joint_name) {
            throw std::runtime_error(std::string(joint_group_name) + " joint not found in URDF model: " + joint_name);
        }

        const auto& joint = model_.joints[joint_id];
        if (joint.nq() != 1 || joint.nv() != 1) {
            std::ostringstream oss;
            oss << joint_group_name << " joint '" << joint_name << "' must be single-DoF, got nq=" << joint.nq()
                << ", nv=" << joint.nv();
            throw std::runtime_error(oss.str());
        }

        ArmJointModelIndex mapping;
        mapping.name = joint_name;
        mapping.joint_id = joint_id;
        mapping.idx_q = static_cast<int>(joint.idx_q());
        mapping.nq = static_cast<int>(joint.nq());
        mapping.idx_v = static_cast<int>(joint.idx_v());
        mapping.nv = static_cast<int>(joint.nv());
        joint_mappings.push_back(mapping);

        RCLCPP_INFO(
            this->get_logger(),
            "%s joint map %s -> joint_id=%u idx_q=%d nq=%d idx_v=%d nv=%d",
            joint_group_name,
            joint_name.c_str(),
            static_cast<unsigned>(mapping.joint_id),
            mapping.idx_q,
            mapping.nq,
            mapping.idx_v,
            mapping.nv);
    }
}

void CommandProcessor_arm::resetPinocchioStateToNeutral_() {
    q_ = q_neutral_;
    v_.setZero();
    a_.setZero();
}

void CommandProcessor_arm::logGravityCompensationDebug_(
    const std::vector<double>& raw_torques,
    const std::vector<double>& scaled_torques) {
    (void)raw_torques;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_gravity_debug_log_time_ < GRAVITY_DEBUG_LOG_PERIOD) {
        return;
    }
    last_gravity_debug_log_time_ = now;

    // std::ostringstream leg_stream;
    // std::ostringstream torque_raw_stream;
    std::ostringstream torque_cmd_stream;
    // leg_stream << std::fixed << std::setprecision(3) << "gravity_comp_legs:";
    // torque_raw_stream << std::fixed << std::setprecision(3) << "gravity_comp_tau_raw_nm:";
    // torque_cmd_stream << std::fixed << std::setprecision(3) << "gravity_comp_tau_cmd:";

    // for (size_t i = 0; i < compensation_joint_mappings_.size(); ++i) {
    //     leg_stream << ' ' << compensation_joint_mappings_[i].name << '=' << compensation_joint_positions_[i];
    // }

    for (size_t i = 0; i < num_joints_; ++i) {
        // torque_raw_stream << ' ' << arm_joint_mappings_[i].name << '=' << raw_torques[i];
        torque_cmd_stream << ' ' << arm_joint_mappings_[i].name << '=' << scaled_torques[i];
    }

    // RCLCPP_INFO(this->get_logger(), "%s", leg_stream.str().c_str());
    // RCLCPP_INFO(this->get_logger(), "%s", torque_raw_stream.str().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", torque_cmd_stream.str().c_str());
}

void CommandProcessor_arm::computeAndLogArmTorques_() {
    std::vector<double> raw_torques(num_joints_, 0.0);
    std::vector<double> scaled_torques(num_joints_, 0.0);

    // Framework stage: CST mode is enabled by topic switching, but torque output stays zero.
    for (size_t i = 0; i < num_joints_; ++i) {
        MotorState_desired_[i].effort = 0.0;
    }

    logGravityCompensationDebug_(raw_torques, scaled_torques);
}

double CommandProcessor_arm::computeJointLimitGuardTorqueNm_(size_t joint_index) const {
    if (joint_index >= MotorState_current_.size()) {
        return 0.0;
    }

    rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    const double lower = lowerPositionLimitForArmJoint_(joint_index);
    const double upper = upperPositionLimitForArmJoint_(joint_index);
    if (!std::isfinite(lower) || !std::isfinite(upper) || upper <= lower) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock,
            5000,
            "Invalid joint limits for %s: lower=%.6f upper=%.6f",
            arm_joint_mappings_[joint_index].name.c_str(),
            lower,
            upper);
        return 0.0;
    }

    const double stiffness_nm_per_rad = std::max(0.0, joint_limit_guard_config_.stiffness_nm_per_rad);
    const double damping_nm_s_per_rad = std::max(0.0, joint_limit_guard_config_.damping_nm_s_per_rad);
    if (stiffness_nm_per_rad <= 0.0 && damping_nm_s_per_rad <= 0.0) {
        return 0.0;
    }

    const double range = upper - lower;
    double soft_zone_rad = std::max(0.0, joint_limit_guard_config_.soft_zone_deg) * M_PI / 180.0;
    soft_zone_rad = std::min(soft_zone_rad, 0.25 * range);

    const double soft_lower = lower + soft_zone_rad;
    const double soft_upper = upper - soft_zone_rad;
    const double position = MotorState_current_[joint_index].position;
    const double velocity = MotorState_current_[joint_index].velocity;
    const double overrun_stiffness_scale = std::max(0.0, joint_limit_guard_config_.overrun_stiffness_scale);
    const double overrun_damping_scale = std::max(0.0, joint_limit_guard_config_.overrun_damping_scale);

    double tau_limit_nm = 0.0;

    if (position < soft_lower) {
        const double penetration = soft_lower - position;
        const double inward_velocity = std::max(0.0, -velocity);
        tau_limit_nm += stiffness_nm_per_rad * penetration + damping_nm_s_per_rad * inward_velocity;

        if (position < lower) {
            tau_limit_nm += stiffness_nm_per_rad * overrun_stiffness_scale * (lower - position);
            tau_limit_nm += damping_nm_s_per_rad * overrun_damping_scale * inward_velocity;
        }
    }

    if (position > soft_upper) {
        const double penetration = position - soft_upper;
        const double inward_velocity = std::max(0.0, velocity);
        tau_limit_nm -= stiffness_nm_per_rad * penetration + damping_nm_s_per_rad * inward_velocity;

        if (position > upper) {
            tau_limit_nm -= stiffness_nm_per_rad * overrun_stiffness_scale * (position - upper);
            tau_limit_nm -= damping_nm_s_per_rad * overrun_damping_scale * inward_velocity;
        }
    }

    if (joint_index < motor_coeffs_.size()) {
        const double max_effort = motor_coeffs_[joint_index].max_effort;
        if (std::isfinite(max_effort) && max_effort > 0.0) {
            const double guard_cap_nm = max_effort * std::max(0.0, joint_limit_guard_config_.max_torque_ratio);
            tau_limit_nm = std::clamp(tau_limit_nm, -guard_cap_nm, guard_cap_nm);
        }
    }

    return tau_limit_nm;
}

double CommandProcessor_arm::clampTorqueToMotorLimitNm_(double torque_nm, size_t joint_index) const {
    if (joint_index >= motor_coeffs_.size()) {
        return torque_nm;
    }

    rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    const double max_effort = motor_coeffs_[joint_index].max_effort;
    if (!std::isfinite(max_effort) || max_effort <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock,
            5000,
            "Joint %s max_effort is invalid (%.6f), skipping torque clamp.",
            arm_joint_mappings_[joint_index].name.c_str(),
            max_effort);
        return torque_nm;
    }

    return std::clamp(torque_nm, -max_effort, max_effort);
}

double CommandProcessor_arm::convertTorqueToEffortCommand_(double torque_nm, size_t joint_index) const {
    if (joint_index >= motor_coeffs_.size()) {
        return 0.0;
    }

    rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    const double torque_constant = motor_coeffs_[joint_index].Torqueconstant;
    if (!std::isfinite(torque_constant) || std::abs(torque_constant) <= MIN_TORQUE_CONSTANT) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock,
            5000,
            "Joint %s Torqueconstant is invalid (%.9f), forcing effort command to zero.",
            arm_joint_mappings_[joint_index].name.c_str(),
            torque_constant);
        return 0.0;
    }

    return torque_nm / torque_constant;
}

double CommandProcessor_arm::lowerPositionLimitForArmJoint_(size_t joint_index) const {
    if (joint_index >= arm_joint_mappings_.size()) {
        return -6.28;
    }

    const int idx_q = arm_joint_mappings_[joint_index].idx_q;
    if (idx_q >= 0 && idx_q < model_.lowerPositionLimit.size()) {
        return model_.lowerPositionLimit[idx_q];
    }
    return -6.28;
}

double CommandProcessor_arm::upperPositionLimitForArmJoint_(size_t joint_index) const {
    if (joint_index >= arm_joint_mappings_.size()) {
        return 6.28;
    }

    const int idx_q = arm_joint_mappings_[joint_index].idx_q;
    if (idx_q >= 0 && idx_q < model_.upperPositionLimit.size()) {
        return model_.upperPositionLimit[idx_q];
    }
    return 6.28;
}

double CommandProcessor_arm::clampPositionToArmJointLimit_(size_t joint_index, double position) const {
    const double lower = lowerPositionLimitForArmJoint_(joint_index);
    const double upper = upperPositionLimitForArmJoint_(joint_index);
    if (!std::isfinite(lower) || !std::isfinite(upper) || upper < lower) {
        return position;
    }

    return std::clamp(position, lower, upper);
}

void CommandProcessor_arm::clampArmTargetsToJointLimits_(std::array<double, 14>& positions) const {
    const size_t clamp_count = std::min(num_joints_, positions.size());
    for (size_t i = 0; i < clamp_count; ++i) {
        positions[i] = clampPositionToArmJointLimit_(i, positions[i]);
    }
}

double CommandProcessor_arm::wrappedAbsPositionError_(double current, double target) {
    double error = std::abs(current - target);
    if (error > M_PI) {
        error = 2 * M_PI - error;
    }
    return error;
}

bool CommandProcessor_arm::shouldDisableRuckigAxis_(size_t joint_index, double target_position) const {
    if (joint_index >= MotorState_current_.size()) {
        return false;
    }

    const double position_error = wrappedAbsPositionError_(MotorState_current_[joint_index].position, target_position);
    const double velocity = std::abs(MotorState_current_[joint_index].velocity);
    const double acceleration = std::abs(MotorState_current_[joint_index].acceleration);

    return position_error <= GOAL_TOLERANCE
        && velocity <= RUCKIG_DISABLE_VEL_TOLERANCE
        && acceleration <= RUCKIG_DISABLE_ACC_TOLERANCE;
}

bool CommandProcessor_arm::updateInterpolationModeFromMsg_(const robot_control_msg::msg::Robotarmservomsg& new_command) {
    const InterpolationMode previous_mode = interpolation_mode_;

    switch (static_cast<int>(new_command.run_mode)) {
        case 0:
            interpolation_mode_ = InterpolationMode::CUBIC_POLYNOMIAL;
            break;
        case 1:
            interpolation_mode_ = InterpolationMode::RUCKIG;
            std::fill(input_joint_plan.max_velocity.begin(), input_joint_plan.max_velocity.end(), 3.14f);
            std::fill(input_joint_plan.max_acceleration.begin(), input_joint_plan.max_acceleration.end(), 30.0f);
            std::fill(input_joint_plan.max_jerk.begin(), input_joint_plan.max_jerk.end(), 40.0f);
            input_joint_plan.synchronization = Synchronization::None;
            break;
        case 2:
            interpolation_mode_ = InterpolationMode::RUCKIG;
            std::fill(input_joint_plan.max_velocity.begin(), input_joint_plan.max_velocity.end(), 3.14f);
            std::fill(input_joint_plan.max_acceleration.begin(), input_joint_plan.max_acceleration.end(), 30.0f);
            std::fill(input_joint_plan.max_jerk.begin(), input_joint_plan.max_jerk.end(), 40.0f);
            input_joint_plan.synchronization = Synchronization::Phase;
            break;
        case 3:
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Robotarmservomsg.run_mode=%d is reserved for legacy control mode and is ignored. Use 0=CUBIC or 1=RUCKIG.",
                static_cast<int>(new_command.run_mode));
            return false;
        default:
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Invalid interpolation run_mode=%d received, keeping previous interpolation mode.",
                static_cast<int>(new_command.run_mode));
            return false;
    }

    return interpolation_mode_ != previous_mode;
}

bool CommandProcessor_arm::isFreshPositionStamp_(const builtin_interfaces::msg::Time& stamp) const {
    const rclcpp::Time command_stamp(stamp);
    return command_stamp.nanoseconds() > 0 &&
        command_stamp > position_reentry_time_;
}

void CommandProcessor_arm::resetCubicPlannerToCurrent_() {
    if (!planner_) {
        return;
    }

    planner_->start_pos.clear();
    planner_->start_vel.clear();
    planner_->end_pos.clear();
    planner_->end_vel.clear();

    planner_->start_pos.reserve(num_joints_);
    planner_->start_vel.reserve(num_joints_);
    planner_->end_pos.reserve(num_joints_);
    planner_->end_vel.reserve(num_joints_);

    for (size_t i = 0; i < num_joints_; ++i) {
        planner_->start_pos.push_back(MotorState_current_[i].position);
        planner_->start_vel.push_back(0.0);
        planner_->end_pos.push_back(MotorState_current_[i].position);
        planner_->end_vel.push_back(0.0);
    }

    CubicPlanner_planner_initialized_ = false;
}

void CommandProcessor_arm::resetRuckigPlanToCurrent_() {
    planned_target_velocity_.fill(0.0);
    planned_target_acceleration_.fill(0.0);
    for (size_t i = 0; i < num_joints_; ++i) {
        const double current_position = MotorState_current_[i].position;
        input_joint_plan.current_position[i] = current_position;
        input_joint_plan.current_velocity[i] = 0.0;
        input_joint_plan.current_acceleration[i] = 0.0;
        input_joint_plan.target_position[i] = current_position;
        input_joint_plan.target_velocity[i] = 0.0;
        input_joint_plan.target_acceleration[i] = 0.0;
        input_joint_plan.enabled[i] = true;

        output_joint_plan.new_position[i] = current_position;
        output_joint_plan.new_velocity[i] = 0.0;
        output_joint_plan.new_acceleration[i] = 0.0;
    }
}

void CommandProcessor_arm::syncPositionTargetsToCurrent_() {
    std::array<double, 14> current_positions = get_pos_;

    for (size_t i = 0; i < num_joints_; ++i) {
        const double current_position = MotorState_current_[i].position;
        current_positions[i] = current_position;
        MotorState_desired_[i].position = current_position;
        MotorState_desired_[i].velocity = 0.0;
        MotorState_desired_[i].acceleration = 0.0;
    }

    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        get_pos_ = current_positions;
    }

    position_targets_initialized_ = true;

    resetRuckigPlanToCurrent_();
    resetCubicPlannerToCurrent_();
}

void CommandProcessor_arm::transitionPositionHoldToWait_() {
    if (position_control_state_ == PositionControlState::EFFORT_TO_POSITION_HOLD) {
        position_control_state_ = PositionControlState::WAIT_FRESH_POSITION_TARGET;
    }
}

HardwareCommandMode CommandProcessor_arm::currentHardwareCommandMode_() const {
    if (active_control_mode_ == ControlMode::EFFORT) {
        return HardwareCommandMode::EFFORT;
    }

    if (external_position_stream_active_) {
        return HardwareCommandMode::EXTERNAL_POSITION_STREAM;
    }
    return interpolation_mode_ == InterpolationMode::RUCKIG
        ? HardwareCommandMode::RUCKIG
        : HardwareCommandMode::CUBIC_POLYNOMIAL;
}

/**
 * @brief 重置命令处理器状态
 */
void CommandProcessor_arm::reset() {
    get_pos_.fill(0.0);
    power_enabled_ = false;
    hardware_available_ = false;
    position_targets_initialized_ = false;
    external_position_stream_active_ = false;
    run_time_ = 0.0;
    interpolation_mode_ = InterpolationMode::RUCKIG;
    active_control_mode_ = ControlMode::POSITION;
    requested_control_mode_ = ControlMode::POSITION;
    position_control_state_ = PositionControlState::POSITION_ACTIVE;
    position_reentry_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::fill(input_joint_plan.enabled.begin(), input_joint_plan.enabled.end(), true);
    for (size_t i = 0; i < compensation_joint_mappings_.size() && i < compensation_joint_positions_.size(); ++i) {
        compensation_joint_positions_[i] = q_neutral_(compensation_joint_mappings_[i].idx_q);
    }
    resetCubicPlannerToCurrent_();
    resetRuckigPlanToCurrent_();
}

bool CommandProcessor_arm::requestControlMode(ControlMode mode, std::string* message) {
    requested_control_mode_ = mode;

    if (mode == ControlMode::EFFORT) {
        if (active_control_mode_ == ControlMode::EFFORT &&
            position_control_state_ == PositionControlState::EFFORT_ACTIVE) {
            if (message != nullptr) {
                *message = "Arm control mode already in EFFORT";
            }
            return true;
        }

        active_control_mode_ = ControlMode::EFFORT;
        position_control_state_ = PositionControlState::EFFORT_ACTIVE;
        deactivateHoldAll();
        setRobotState(RobotState::Stopped, "EFFORT requested");

        if (message != nullptr) {
            *message = "Arm control mode switched to EFFORT";
        }
        return true;
    }

    if (active_control_mode_ == ControlMode::POSITION &&
        position_control_state_ == PositionControlState::POSITION_ACTIVE) {
        if (message != nullptr) {
            *message = "Arm control mode already in POSITION";
        }
        return true;
    }

    active_control_mode_ = ControlMode::POSITION;
    syncPositionTargetsToCurrent_();
    position_reentry_time_ = this->now();
    position_control_state_ = PositionControlState::EFFORT_TO_POSITION_HOLD;
    setRobotState(RobotState::Stopped, "Holding current pose before accepting a fresh position target");

    if (message != nullptr) {
        *message = "Arm control mode switched to POSITION hold; waiting for a fresh stamped target";
    }
    return true;
}

bool CommandProcessor_arm::requestPowerEnable(bool enable, std::string* message) {
    if (power_enabled_ == enable) {
        if (message != nullptr) {
            *message = enable ? "Arm power already enabled" : "Arm power already disabled";
        }
        return true;
    }

    power_enabled_ = enable;

    if (!power_enabled_) {
        position_targets_initialized_ = false;
        deactivateHoldAll();
        for (size_t i = 0; i < num_joints_; ++i) {
            MotorState_desired_[i].position = MotorState_current_[i].position;
            MotorState_desired_[i].velocity = 0.0;
            MotorState_desired_[i].acceleration = 0.0;
            MotorState_desired_[i].effort = 0.0;
        }
        setRobotState(RobotState::Stopped, "Power disabled");
    }

    if (message != nullptr) {
        *message = enable ? "Arm power enabled" : "Arm power disabled";
    }
    return true;
}

ControlMode CommandProcessor_arm::getActiveControlMode() const {
    return active_control_mode_;
}

uint8_t CommandProcessor_arm::getActiveControlModeValue() const {
    return static_cast<uint8_t>(active_control_mode_);
}

uint8_t CommandProcessor_arm::getRequestedControlModeValue() const {
    return static_cast<uint8_t>(requested_control_mode_);
}

bool CommandProcessor_arm::isPositionCommandReady() const {
    return active_control_mode_ == ControlMode::POSITION &&
        position_control_state_ == PositionControlState::POSITION_ACTIVE;
}

/**
 * @brief 处理新的机械臂控制命令
 * @param new_command 新的机械臂控制消息
 */
bool CommandProcessor_arm::processNewCommand(robot_control_msg::msg::Robotarmservomsg& new_command) {
    external_position_stream_active_ = false;
    if (active_control_mode_ == ControlMode::EFFORT ||
        position_control_state_ == PositionControlState::EFFORT_TO_POSITION_HOLD) {
        return false;
    }

    if (position_control_state_ == PositionControlState::WAIT_FRESH_POSITION_TARGET &&
        !isFreshPositionStamp_(new_command.stamp)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Ignoring stale arm position command while waiting for a fresh target after EFFORT->POSITION.");
        return false;
    }

    // 使用当前缓存目标作为去抖基准
    std::array<double, 14> cached_get_pos;
    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        cached_get_pos = get_pos_;
    }
    std::array<double, 14> temp_get_pos = cached_get_pos;
    temp_get_pos[0] = clampPositionToArmJointLimit_(0, new_command.ljoint1_position);
    temp_get_pos[1] = clampPositionToArmJointLimit_(1, new_command.ljoint2_position);
    temp_get_pos[2] = clampPositionToArmJointLimit_(2, new_command.ljoint3_position);
    temp_get_pos[3] = clampPositionToArmJointLimit_(3, new_command.ljoint4_position);
    temp_get_pos[4] = clampPositionToArmJointLimit_(4, new_command.ljoint5_position);
    temp_get_pos[5] = clampPositionToArmJointLimit_(5, new_command.ljoint6_position);
    temp_get_pos[6] = clampPositionToArmJointLimit_(6, new_command.ljoint7_position);
    temp_get_pos[7] = clampPositionToArmJointLimit_(7, new_command.rjoint1_position);
    temp_get_pos[8] = clampPositionToArmJointLimit_(8, new_command.rjoint2_position);
    temp_get_pos[9] = clampPositionToArmJointLimit_(9, new_command.rjoint3_position);
    temp_get_pos[10] = clampPositionToArmJointLimit_(10, new_command.rjoint4_position);
    temp_get_pos[11] = clampPositionToArmJointLimit_(11, new_command.rjoint5_position);
    temp_get_pos[12] = clampPositionToArmJointLimit_(12, new_command.rjoint6_position);
    temp_get_pos[13] = clampPositionToArmJointLimit_(13, new_command.rjoint7_position);

    bool target_changed = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (std::abs(temp_get_pos[i] - cached_get_pos[i]) >= REPEATED_TARGET_DEADBAND) {
            target_changed = true;
            break;
        }
    }

    // 设置运行参数
    const double previous_run_time = run_time_;
    run_time_ = new_command.run_time;

    const bool mode_changed = updateInterpolationModeFromMsg_(new_command);
    const bool run_time_changed = std::abs(run_time_ - previous_run_time) > 1e-9;
    const bool should_replan = target_changed || mode_changed || run_time_changed;

    // 进入新的规划前，解除所有持位
    deactivateHoldAll();

    if (position_control_state_ == PositionControlState::WAIT_FRESH_POSITION_TARGET) {
        position_control_state_ = PositionControlState::POSITION_ACTIVE;
    }

    if (!should_replan) {
        return false;
    }

    std::fill(input_joint_plan.enabled.begin(), input_joint_plan.enabled.end(), true);
    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        get_pos_ = temp_get_pos;
    }
    position_targets_initialized_ = true;

    return true;
}

/**
 * @brief 规划机械臂运动轨迹
 * @param time 当前时间戳
 */
void CommandProcessor_arm::command_arm_plan(const rclcpp::Time & time) {
    if (active_control_mode_ != ControlMode::POSITION ||
        position_control_state_ != PositionControlState::POSITION_ACTIVE) {
        return;
    }

    // 复制get_pos_到局部变量，避免在规划过程中被修改
    std::array<double, 14> current_get_pos;
    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        current_get_pos = get_pos_;
    }
    
    // 完整性检查：确保get_pos_数组大小正确
    if (current_get_pos.size() != num_joints_) {
        RCLCPP_ERROR(rclcpp::get_logger("CommandProcessor_arm"), 
                    "get_pos_ array size mismatch: expected %zu, got %zu", 
                    num_joints_, current_get_pos.size());
        return;
    }
    
    // 完整性检查：确保所有值都是有效的（非NaN）
    for (size_t i = 0; i < current_get_pos.size(); i++) {
        if (std::isnan(current_get_pos[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("CommandProcessor_arm"), 
                        "get_pos_[%zu] is NaN, skipping planning", i);
            return;
        }
    }
    
    // 根据插补模式执行不同的轨迹规划逻辑
    switch (interpolation_mode_) {
        case InterpolationMode::RUCKIG:
            // Ruckig轨迹生成器模式
            // 设置当前状态作为Ruckig输入
            for (size_t i = 0; i < num_joints_; i++) {
                const double target_position = current_get_pos[i];
                const bool disable_axis = shouldDisableRuckigAxis_(i, target_position);

                if (disable_axis) {
                    input_joint_plan.enabled[i] = false;
                    input_joint_plan.current_position[i] = MotorState_current_[i].position;
                    input_joint_plan.current_velocity[i] = 0.0;
                    input_joint_plan.current_acceleration[i] = 0.0;
                    input_joint_plan.target_position[i] = MotorState_current_[i].position;
                    input_joint_plan.target_velocity[i] = 0.0;
                    input_joint_plan.target_acceleration[i] = 0.0;
                } else {
                    input_joint_plan.enabled[i] = true;
                    input_joint_plan.current_position[i] = MotorState_desired_[i].position;
                    input_joint_plan.current_velocity[i] = MotorState_desired_[i].velocity;
                    input_joint_plan.current_acceleration[i] = MotorState_desired_[i].acceleration;
                    input_joint_plan.target_position[i] = target_position;
                    input_joint_plan.target_velocity[i] = planned_target_velocity_[i];
                    input_joint_plan.target_acceleration[i] = planned_target_acceleration_[i];
                }
            }
            
            // 设置目标状态
            break;
        case InterpolationMode::CUBIC_POLYNOMIAL:
        default:
            {
                bool positions_equal = true;
                if (current_get_pos.size() != planner_->end_pos.size()) {
                    positions_equal = false;
                } else {
                    for (size_t i = 0; i < current_get_pos.size(); i++) {
                        if (std::abs(current_get_pos[i] - planner_->end_pos[i]) > 0.001) {
                            positions_equal = false;
                            break;
                        }
                    }
                }

                if (!positions_equal) {
                    planner_->start_pos.clear();
                    planner_->start_vel.clear();
                    planner_->end_pos.clear();
                    planner_->end_vel.clear();

                    for (size_t i = 0; i < num_joints_; i++) {
                        planner_->start_pos.push_back(MotorState_current_[i].position);
                        planner_->start_vel.push_back(MotorState_current_[i].velocity);
                        planner_->end_pos.push_back(current_get_pos[i]);
                        planner_->end_vel.push_back(0.0);
                    }

                    planner_->plan(
                        planner_->start_pos,
                        planner_->start_vel,
                        planner_->end_pos,
                        planner_->end_vel,
                        run_time_);

                    start_time_ = time.seconds();
                    CubicPlanner_planner_initialized_ = true;

                    RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "plan_end");
                }
            }
            break;
    }
}

/**
 * @brief 更新关节状态信息
 * @param position_states 位置状态接口
 * @param velocity_states 速度状态接口
 * @param motor_states 电机状态接口
 */
void CommandProcessor_arm::updateJointState(std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& position_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& velocity_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& motor_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& error_code_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& compensation_position_states) {

    if (position_states.size() < num_joints_ ||
        velocity_states.size() < num_joints_ ||
        motor_states.size() < num_joints_ ||
        error_code_states.size() < num_joints_) {
        all_enabled_ = false;
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Arm state interface count mismatch: position=%zu velocity=%zu status=%zu error_code=%zu expected=%zu",
            position_states.size(),
            velocity_states.size(),
            motor_states.size(),
            error_code_states.size(),
            num_joints_);
        return;
    }

    if (compensation_position_states.size() < compensation_joint_positions_.size()) {
        all_enabled_ = false;
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Gravity compensation joint state count mismatch: position=%zu expected=%zu",
            compensation_position_states.size(),
            compensation_joint_positions_.size());
        return;
    }
    
    bool all_enabled_local = true;
    bool hardware_available_local = true;
    for (uint8_t i = 0; i < num_joints_; i++) {
        // 更新当前关节位置
        MotorState_current_[i].position = position_states[i].get().get_value();
        
        // 计算关节加速度 (速度变化率)
        MotorState_current_[i].acceleration = ((velocity_states[i].get().get_value() - MotorState_current_[i].velocity) / 0.002f);
        
        // 更新当前关节速度和电源状态
        MotorState_current_[i].velocity = velocity_states[i].get().get_value();
        MotorState_current_[i].motor_status = motor_states[i].get().get_value();
        MotorState_current_[i].error_code = error_code_states[i].get().get_value();
        if (!std::isfinite(MotorState_current_[i].motor_status) ||
            static_cast<int>(std::llround(MotorState_current_[i].motor_status)) ==
              kUnknownStatusCode) {
            hardware_available_local = false;
        }
        if (!is_motor_enabled(MotorState_current_[i].motor_status)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Joint %u motor status is %.0f, expected %d for enabled",
                i,
                MotorState_current_[i].motor_status,
                MOTOR_STATUS_ENABLED);
        }

        // 累计全使能聚合（仅当全部为 39 时为 true）
        if (!is_motor_enabled(MotorState_current_[i].motor_status)) {
            all_enabled_local = false;
        }
    }

    for (size_t i = 0; i < compensation_joint_positions_.size(); ++i) {
        compensation_joint_positions_[i] = compensation_position_states[i].get().get_value();
    }

    // 循环结束后一次性更新全局 all_enabled_
    if (all_enabled_ != all_enabled_local) {
        RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"),
                    "all_enabled_ changed: %s -> %s",
                    all_enabled_ ? "true" : "false",
                    all_enabled_local ? "true" : "false");
    }
    all_enabled_ = all_enabled_local;
    hardware_available_ = hardware_available_local;
    if (!hardware_available_) {
        // Do not leave a previous enable request latched when the hardware
        // disappears or the EtherCAT shared-memory state is unavailable.
        power_enabled_ = false;
    }
    // RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "Status current: %.0f", MotorState_current_[0].motor_status);
    // RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "Status motor: %.0f", motor_states[0].get().get_value());
}

/**
 * @brief 更新关节控制命令
 * @param position_states 位置状态接口
 * @param position_commands 位置命令接口
 * @param velocity_commands 速度命令接口
 * @param run_mode_commands 电源命令接口
 * @param effort_commands 力矩命令接口
 */
void CommandProcessor_arm::updateJointCommands(
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& position_states,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& position_commands,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& velocity_commands,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& effort_commands,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& power_commands,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& run_mode_commands,
    bool allow_disabled_simulation_execution
) {
    if (position_states.size() < num_joints_ ||
        position_commands.size() < num_joints_ ||
        velocity_commands.size() < num_joints_ ||
        effort_commands.size() < num_joints_ ||
        power_commands.size() < num_joints_ ||
        run_mode_commands.size() < num_joints_) {
        return;
    }

    const HardwareCommandMode command_mode = currentHardwareCommandMode_();

    // Always refresh the feedforward torque estimate from the latest joint state.
    computeAndLogArmTorques_();

    bool initialized_position_targets_this_cycle = false;

    // A managed SIM Heavy lease may advance the mock plant without changing
    // power feedback. REAL still requires all 14 actual drives to be enabled.
    const bool execution_allowed = all_enabled_ || allow_disabled_simulation_execution;

    if (execution_allowed) {
        if (active_control_mode_ == ControlMode::POSITION && !position_targets_initialized_) {
            syncPositionTargetsToCurrent_();
            initialized_position_targets_this_cycle = true;
        }

        if (active_control_mode_ == ControlMode::EFFORT) {
            setRobotState(RobotState::Stopped, "EFFORT mode - torque calculation only");
        } else if (initialized_position_targets_this_cycle) {
            std::array<double, 14> hold_positions;
            {
                std::lock_guard<std::mutex> lock(get_pos_mutex_);
                hold_positions = get_pos_;
            }

            for (size_t i = 0; i < num_joints_; ++i) {
                MotorState_desired_[i].position = hold_positions[i];
                MotorState_desired_[i].velocity = 0.0;
                MotorState_desired_[i].acceleration = 0.0;
            }

            setRobotState(RobotState::Stopped, "Initialized position targets from current pose after enable");
        } else if (external_position_stream_active_) {
            for (size_t i = 0; i < num_joints_; ++i) {
                MotorState_desired_[i] = external_stream_target_[i];
            }
            setRobotState(RobotState::ExecutingJoint, "Heavy external position stream");
        } else if (position_control_state_ == PositionControlState::EFFORT_TO_POSITION_HOLD ||
                   position_control_state_ == PositionControlState::WAIT_FRESH_POSITION_TARGET) {
            std::array<double, 14> hold_positions;
            {
                std::lock_guard<std::mutex> lock(get_pos_mutex_);
                hold_positions = get_pos_;
            }

            for (size_t i = 0; i < num_joints_; ++i) {
                MotorState_desired_[i].position = hold_positions[i];
                MotorState_desired_[i].velocity = 0.0;
                MotorState_desired_[i].acceleration = 0.0;
            }

            setRobotState(RobotState::Stopped, "Holding current pose until a fresh position target arrives");
        } else {
            switch (interpolation_mode_) {
                case InterpolationMode::RUCKIG:
                {
                    // const bool any_axis_enabled = std::any_of(
                    //     input_joint_plan.enabled.begin(),
                    //     input_joint_plan.enabled.end(),
                    //     [](bool enabled) { return enabled; });

                    // if (!any_axis_enabled) {
                    //     for (size_t i = 0; i < num_joints_; i++) {
                    //         MotorState_desired_[i].position = MotorState_current_[i].position;
                    //         MotorState_desired_[i].velocity = 0.0;
                    //         MotorState_desired_[i].acceleration = 0.0;
                    //     }
                    //     setRobotState(RobotState::Stopped, "RUCKIG skipped, all axes at target");
                    //     break;
                    // }

                    setRobotState(RobotState::ExecutingJoint);
                    Result res = otg_joint_plan.update(input_joint_plan, output_joint_plan);
                    if (res == Result::Working) {
                        for (size_t i = 0; i < num_joints_; i++) {
                            MotorState_desired_[i].position = output_joint_plan.new_position.at(i);
                            MotorState_desired_[i].velocity = output_joint_plan.new_velocity.at(i);
                            MotorState_desired_[i].acceleration = output_joint_plan.new_acceleration.at(i);
                        }
                        output_joint_plan.pass_to_input(input_joint_plan);
                    } else if (res == Result::Finished) {
                        for (size_t i = 0; i < num_joints_; i++) {
                            const double final_position = input_joint_plan.target_position[i];
                            MotorState_desired_[i].position = final_position;
                            MotorState_desired_[i].velocity = 0.0;
                            MotorState_desired_[i].acceleration = 0.0;

                            input_joint_plan.current_position[i] = final_position;
                            input_joint_plan.current_velocity[i] = 0.0;
                            input_joint_plan.current_acceleration[i] = 0.0;
                            input_joint_plan.target_position[i] = final_position;
                            input_joint_plan.target_velocity[i] = 0.0;
                            input_joint_plan.target_acceleration[i] = 0.0;
                            input_joint_plan.enabled[i] = false;
                        }
                        setRobotState(RobotState::Stopped, "RUCKIG finished");
                    }
                    break;
                }

                case InterpolationMode::CUBIC_POLYNOMIAL:
                default:
                    if (!planner_->all_axes_stopped()) {
                        if (CubicPlanner_planner_initialized_ == true) {
                            start_time_ = rclcpp::Clock().now().seconds();
                            CubicPlanner_planner_initialized_ = false;
                        }
                        auto current_time = rclcpp::Clock().now().seconds();
                        double time_diff = (current_time - start_time_);
                        trajectory_points = planner_->update(time_diff);
                        for (size_t i = 0; i < num_joints_; i++) {
                            MotorState_desired_[i].position = trajectory_points[i].position;
                            MotorState_desired_[i].velocity = trajectory_points[i].velocity;
                            MotorState_desired_[i].acceleration = trajectory_points[i].acceleration;
                        }
                        setRobotState(RobotState::ExecutingJoint);
                    } else {
                        std::array<double, 14> current_targets;
                        {
                            std::lock_guard<std::mutex> lock(get_pos_mutex_);
                            current_targets = get_pos_;
                        }

                        for (size_t i = 0; i < num_joints_; i++) {
                            MotorState_desired_[i].position = current_targets[i];
                            MotorState_desired_[i].velocity = 0.0;
                            MotorState_desired_[i].acceleration = 0.0;
                        }
                        setRobotState(RobotState::Stopped, "CUBIC finished");
                    }
                    break;
            }
        }
    } else {
        position_targets_initialized_ = false;
        for (size_t i = 0; i < num_joints_; i++) {
            MotorState_desired_[i].position = MotorState_current_[i].position;
            MotorState_desired_[i].velocity = 0.0;
            MotorState_desired_[i].acceleration = 0.0;
            MotorState_desired_[i].effort = 0.0;
        }
        setRobotState(RobotState::Stopped);
    }
    // std::stringstream ss;
    // ss << "Status effor:";
    for (size_t i = 0; i < num_joints_; i++) {
        power_commands[i].get().set_value(power_enabled_ ? 1.0 : 0.0);
        run_mode_commands[i].get().set_value(static_cast<double>(command_mode));

        if (active_control_mode_ == ControlMode::EFFORT) {
            effort_commands[i].get().set_value(MotorState_desired_[i].effort);
            position_commands[i].get().set_value(MotorState_current_[i].position);
            velocity_commands[i].get().set_value(MotorState_current_[i].velocity);
            // ss << " " << std::fixed << std::setprecision(4) << MotorState_desired_[i].effort;
            // RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "%f", MotorState_desired_[i].effort);
        } else {
            position_commands[i].get().set_value(MotorState_desired_[i].position);
            velocity_commands[i].get().set_value(MotorState_desired_[i].velocity);
            effort_commands[i].get().set_value(MotorState_desired_[i].effort);
        }
        
        // velocity_commands[13].get().set_value(MotorState_desired_[13].velocity);
    }
    // RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), "%s", ss.str().c_str());
    transitionPositionHoldToWait_();

    bool is_moving = false;
    bool goal_reached = true;

    if (execution_allowed) {
        if (robot_state_ == RobotState::ExecutingJoint) {
            is_moving = true;
        } else {
            for (size_t i = 0; i < num_joints_; i++) {
                if (std::abs(MotorState_current_[i].velocity) > 0.01) {
                    is_moving = true;
                    break;
                }
            }
        }

        if (active_control_mode_ == ControlMode::POSITION) {
            std::array<double, 14> current_get_pos;
            {
                std::lock_guard<std::mutex> lock(get_pos_mutex_);
                current_get_pos = get_pos_;
            }
            for (size_t i = 0; i < num_joints_ && i < current_get_pos.size(); i++) {
                double position_error = std::abs(MotorState_current_[i].position - current_get_pos[i]);
                if (position_error > M_PI) {
                    position_error = 2 * M_PI - position_error;
                }
                if (position_error > GOAL_TOLERANCE) {
                    goal_reached = false;
                    break;
                }
            }
        } else {
            goal_reached = true;
        }
    } else {
        is_moving = false;
        goal_reached = true;
    }

    if (is_moving) {
        goal_reached = false;
    }

    auto motion_status_msg = robot_control_msg::msg::ArmMotionStatus();
    motion_status_msg.is_moving = is_moving;
    motion_status_msg.goal_reached = goal_reached;
    motion_status_msg.stamp = this->now();
    motion_status_publisher_->publish(motion_status_msg);

    const auto steady_now = std::chrono::steady_clock::now();
    if (last_power_status_publish_time_.time_since_epoch().count() == 0 ||
        steady_now - last_power_status_publish_time_ >= std::chrono::milliseconds(100)) {
        robot_control_msg::msg::ArmPowerStatus power_status_msg;
        power_status_msg.stamp = this->now();
        power_status_msg.joint_names = arm_joint_names_;
        power_status_msg.command_enabled = power_enabled_ && hardware_available_;
        power_status_msg.all_enabled = all_enabled_;
        power_status_msg.status_codes.reserve(num_joints_);
        power_status_msg.enabled.reserve(num_joints_);
        std::size_t available_count = 0;

        for (size_t i = 0; i < num_joints_; ++i) {
            const int status_code =
                static_cast<int>(std::llround(MotorState_current_[i].motor_status));
            power_status_msg.status_codes.push_back(status_code);
            power_status_msg.enabled.push_back(is_motor_enabled(MotorState_current_[i].motor_status));
            if (status_code != kUnknownStatusCode) {
                ++available_count;
            }
        }

        if (available_count == 0) {
            power_status_msg.message =
                "no EtherCAT drives detected; drive status unavailable";
        } else if (available_count < num_joints_) {
            power_status_msg.message =
                "not all 14 drives are available; available=" +
                std::to_string(available_count) + "/" + std::to_string(num_joints_);
        } else {
            if (all_enabled_) {
                power_status_msg.message = "all arm motors enabled";
            } else {
                std::ostringstream error_summary;
                for (size_t i = 0; i < num_joints_; ++i) {
                    const auto error_code = static_cast<unsigned int>(
                        std::llround(MotorState_current_[i].error_code));
                    if (error_code == 0) {
                        continue;
                    }
                    if (error_summary.tellp() > 0) {
                        error_summary << ", ";
                    }
                    error_summary << arm_joint_names_[i] << "=0x"
                                  << std::hex << std::uppercase << error_code << std::dec;
                }
                power_status_msg.message = "one or more arm motors disabled";
                if (error_summary.tellp() > 0) {
                    power_status_msg.message += "; error_codes=[" +
                        error_summary.str() + "]";
                }
            }
        }
        power_status_publisher_->publish(power_status_msg);
        last_power_status_publish_time_ = steady_now;
    }

    if (last_control_mode_status_publish_time_.time_since_epoch().count() == 0 ||
        steady_now - last_control_mode_status_publish_time_ >= std::chrono::milliseconds(100)) {
        robot_control_msg::msg::ArmControlModeStatus mode_status_msg;
        mode_status_msg.stamp = this->now();
        mode_status_msg.requested_mode = getRequestedControlModeValue();
        mode_status_msg.active_mode = getActiveControlModeValue();
        mode_status_msg.position_command_ready = isPositionCommandReady();

        if (active_control_mode_ == ControlMode::EFFORT) {
            mode_status_msg.message = "EFFORT mode active";
        } else if (mode_status_msg.position_command_ready) {
            mode_status_msg.message = "POSITION mode active and ready";
        } else {
            mode_status_msg.message = "POSITION mode active; holding for a fresh position target";
        }

        control_mode_status_publisher_->publish(mode_status_msg);
        last_control_mode_status_publish_time_ = steady_now;
    }
}

// ======== 封装的小函数实现 ========

void CommandProcessor_arm::setRobotState(RobotState s, const std::string& /*reason*/) {
    // 按需安静：仅更新状态，不打印日志；模式切换时统一打印
    robot_state_ = s;
}

void CommandProcessor_arm::activateHoldForArm(int start, int end_exclusive) {
    for (int i = start; i < end_exclusive; ++i) {
        if (!hold_active_[i]) {
            // 以当前期望位置为持位基准，避免受外界扰动而改变
            hold_pos_[i] = MotorState_desired_[i].position;
        }
        hold_active_[i] = true;
    }
}

void CommandProcessor_arm::deactivateHoldForArm(int start, int end_exclusive) {
    for (int i = start; i < end_exclusive; ++i) {
        hold_active_[i] = false;
    }
}

void CommandProcessor_arm::deactivateHoldAll() {
    std::fill(hold_active_.begin(), hold_active_.end(), false);
}

const char* CommandProcessor_arm::robotStateName(RobotState s) {
    switch (s) {
        case RobotState::Idle: return "Idle";
        case RobotState::ExecutingJoint: return "ExecutingJoint";
        case RobotState::Stopped: return "Stopped";
        case RobotState::Error: return "Error";
        default: return "Unknown";
    }
}

bool CommandProcessor_arm::checkLimitMargin(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double margin_deg) {
    const double margin_rad = margin_deg * M_PI / 180.0;
    for (int k = start; k < end_exclusive; ++k) {
        double q = q_sol[k];
        double lower_dist = q - lowerPositionLimitForArmJoint_(k);
        double upper_dist = upperPositionLimitForArmJoint_(k) - q;
        if (lower_dist < margin_rad || upper_dist < margin_rad) {
            return true;
        }
    }
    return false;
}

bool CommandProcessor_arm::checkAngleDiffExceeded(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double max_angle_diff_deg) {
    const double max_angle_diff_rad = max_angle_diff_deg * M_PI / 180.0;
    for (int k = start; k < end_exclusive; ++k) {
        double q_new = q_sol[k];
        double q_old = MotorState_desired_[k].position;
        double angle_diff = std::abs(q_new - q_old);
        if (angle_diff > max_angle_diff_rad) {
            return true;
        }
    }
    return false;
}

void CommandProcessor_arm::applySolutionToMotor(const Eigen::VectorXd& q_sol, int start, int end_exclusive, double dt_control) {
    (void)dt_control;
    for (int i = start; i < end_exclusive; ++i) {
        double q_new = q_sol[i];
        MotorState_desired_[i].position = q_new;
        MotorState_desired_[i].velocity = 0.0;
        MotorState_desired_[i].acceleration = 0.0;
    }
}

void CommandProcessor_arm::holdCurrentForArm(int start, int end_exclusive) {
    for (int i = start; i < end_exclusive; ++i) {
        double q_old = MotorState_desired_[i].position;
        double clamped = std::min(std::max(q_old, lowerPositionLimitForArmJoint_(i)), upperPositionLimitForArmJoint_(i));
        MotorState_desired_[i].position = clamped;
        MotorState_desired_[i].velocity = 0.0;
        MotorState_desired_[i].acceleration = 0.0;
    }
}

// ======== 状态查询接口实现 ========
const char* CommandProcessor_arm::controlModeName(ControlMode m) {
    switch (m) {
        case ControlMode::POSITION: return "POSITION";
        case ControlMode::EFFORT:   return "EFFORT";
        default:                    return "UNKNOWN";
    }
}

const char* CommandProcessor_arm::interpolationModeName(InterpolationMode m) {
    switch (m) {
        case InterpolationMode::CUBIC_POLYNOMIAL: return "CUBIC_POLYNOMIAL";
        case InterpolationMode::RUCKIG:           return "RUCKIG";
        default:                                  return "UNKNOWN";
    }
}

CommandProcessor_arm::ArmStatus CommandProcessor_arm::getArmStatus() const {
    ArmStatus s{};

    s.control_mode = active_control_mode_;
    s.interpolation_mode = interpolation_mode_;
    s.state = robot_state_;
    s.control_mode_name = controlModeName(s.control_mode);
    s.interpolation_mode_name = interpolationModeName(s.interpolation_mode);
    s.state_name = robotStateName(s.state);

    bool moving = (robot_state_ == RobotState::ExecutingJoint);

    if (!moving && position_targets_initialized_) {
        const double vel_eps = 1e-5;
        const double pos_eps = 1e-4;
        size_t N = std::min(MotorState_desired_.size(), MotorState_current_.size());
        for (size_t i = 0; i < N; ++i) {
            double v = std::abs(MotorState_desired_[i].velocity);
            double dp = std::abs(MotorState_desired_[i].position - MotorState_current_[i].position);
            if (v > vel_eps || dp > pos_eps) {
                moving = true;
                break;
            }
        }

        if (!moving &&
            s.control_mode == ControlMode::POSITION &&
            position_control_state_ == PositionControlState::POSITION_ACTIVE &&
            s.interpolation_mode == InterpolationMode::CUBIC_POLYNOMIAL &&
            planner_) {
            try {
                if (!planner_->all_axes_stopped()) {
                    moving = true;
                }
            } catch (...) {}
        }
    }

    s.is_moving = moving;
    return s;
}

/**
 * @brief 处理新的机械臂关节绝对命令
 * @param new_command 新的机械臂关节消息
 */
bool CommandProcessor_arm::processNewArmJointCommand(robot_control_msg::msg::Robotarmjoint& new_command) {
    if (active_control_mode_ == ControlMode::EFFORT ||
        position_control_state_ == PositionControlState::EFFORT_TO_POSITION_HOLD) {
        return false;
    }

    deactivateHoldAll();

    std::array<double, 14> temp_get_pos = get_pos_;

    temp_get_pos[0] = new_command.ljoint1;
    temp_get_pos[1] = new_command.ljoint2;
    temp_get_pos[2] = new_command.ljoint3;
    temp_get_pos[3] = new_command.ljoint4;
    temp_get_pos[4] = new_command.ljoint5;
    temp_get_pos[5] = new_command.ljoint6;
    temp_get_pos[6] = new_command.ljoint7;

    temp_get_pos[7] = new_command.rjoint1;
    temp_get_pos[8] = new_command.rjoint2;
    temp_get_pos[9] = new_command.rjoint3;
    temp_get_pos[10] = new_command.rjoint4;
    temp_get_pos[11] = new_command.rjoint5;
    temp_get_pos[12] = new_command.rjoint6;
    temp_get_pos[13] = new_command.rjoint7;

    clampArmTargetsToJointLimits_(temp_get_pos);

    if (position_control_state_ == PositionControlState::WAIT_FRESH_POSITION_TARGET) {
        position_control_state_ = PositionControlState::POSITION_ACTIVE;
    }

    interpolation_mode_ = InterpolationMode::RUCKIG;
    planned_target_velocity_.fill(0.0);
    planned_target_acceleration_.fill(0.0);
    std::fill(input_joint_plan.max_velocity.begin(), input_joint_plan.max_velocity.end(), static_cast<float>(new_command.vel));
    std::fill(input_joint_plan.max_acceleration.begin(), input_joint_plan.max_acceleration.end(), static_cast<float>(new_command.acc));
    std::fill(input_joint_plan.max_jerk.begin(), input_joint_plan.max_jerk.end(), static_cast<float>(new_command.acc));

    for (size_t i = 0; i < num_joints_; i++) {
        input_joint_plan.target_position[i] = temp_get_pos[i];
        input_joint_plan.target_velocity[i] = planned_target_velocity_[i];
        input_joint_plan.target_acceleration[i] = planned_target_acceleration_[i];
        input_joint_plan.enabled[i] = true;
    }

    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        get_pos_ = temp_get_pos;
    }
    position_targets_initialized_ = true;

    RCLCPP_INFO(rclcpp::get_logger("CommandProcessor_arm"), 
                "Received arm joint command: vel=%.3f, acc=%.3f", 
                new_command.vel, new_command.acc);
    return true;
}

bool CommandProcessor_arm::processHeavyPositionCommand(
    const std::array<double, 14>& position,
    const std::array<double, 14>& target_velocity,
    const std::array<double, 14>& target_acceleration,
    bool velocity_provided,
    bool acceleration_provided,
    double max_velocity,
    double max_acceleration,
    std::string* error)
{
    if (active_control_mode_ == ControlMode::EFFORT ||
        position_control_state_ == PositionControlState::EFFORT_TO_POSITION_HOLD) {
        if (error) {*error = "arm controller is not in POSITION mode";}
        return false;
    }
    if (!std::isfinite(max_velocity) || max_velocity <= 0.0 ||
        !std::isfinite(max_acceleration) || max_acceleration <= 0.0) {
        if (error) {*error = "invalid Heavy V1 arm limits";}
        return false;
    }

    for (size_t i = 0; i < num_joints_; ++i) {
        if (!std::isfinite(position[i]) || position[i] < lowerPositionLimitForArmJoint_(i) ||
            position[i] > upperPositionLimitForArmJoint_(i)) {
            if (error) {*error = "arm position exceeds URDF limit at index " + std::to_string(i);}
            return false;
        }
        if ((velocity_provided && (!std::isfinite(target_velocity[i]) ||
            std::abs(target_velocity[i]) > max_velocity)) ||
            (acceleration_provided && (!std::isfinite(target_acceleration[i]) ||
            std::abs(target_acceleration[i]) > max_acceleration))) {
            if (error) {*error = "arm feedforward exceeds limit at index " + std::to_string(i);}
            return false;
        }
    }

    deactivateHoldAll();
    if (position_control_state_ == PositionControlState::WAIT_FRESH_POSITION_TARGET) {
        position_control_state_ = PositionControlState::POSITION_ACTIVE;
    }
    for (size_t i = 0; i < num_joints_; ++i) {
        external_stream_target_[i].position = position[i];
        external_stream_target_[i].velocity = velocity_provided ? target_velocity[i] : 0.0;
        external_stream_target_[i].acceleration =
            acceleration_provided ? target_acceleration[i] : 0.0;
        external_stream_target_[i].effort = 0.0;
    }
    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        get_pos_ = position;
    }
    position_targets_initialized_ = true;
    external_position_stream_active_ = true;
    if (error) {error->clear();}
    return true;
}

bool CommandProcessor_arm::processHeavyHoldPositionCommand(
    const std::array<double, 14>& position, std::string* error)
{
    if (active_control_mode_ != ControlMode::POSITION) {
        if (error) {*error = "arm controller is not in POSITION mode";}
        return false;
    }
    for (size_t i = 0; i < num_joints_; ++i) {
        if (!std::isfinite(position[i]) || position[i] < lowerPositionLimitForArmJoint_(i) ||
            position[i] > upperPositionLimitForArmJoint_(i)) {
            if (error) {*error = "arm HOLD position exceeds URDF limit at index " + std::to_string(i);}
            return false;
        }
        external_stream_target_[i].position = position[i];
        external_stream_target_[i].velocity = 0.0;
        external_stream_target_[i].acceleration = 0.0;
        external_stream_target_[i].effort = 0.0;
    }
    deactivateHoldAll();
    position_control_state_ = PositionControlState::POSITION_ACTIVE;
    position_targets_initialized_ = true;
    external_position_stream_active_ = true;
    {
        std::lock_guard<std::mutex> lock(get_pos_mutex_);
        get_pos_ = position;
    }
    setRobotState(RobotState::Stopped, "Heavy external position HOLD");
    if (error) {error->clear();}
    return true;
}

void CommandProcessor_arm::clearHeavyPositionStream() {
    external_position_stream_active_ = false;
}

bool CommandProcessor_arm::requestPositionHold(const std::string& reason) {
    if (active_control_mode_ != ControlMode::POSITION) {
        return false;
    }

    external_position_stream_active_ = false;
    deactivateHoldAll();
    interpolation_mode_ = InterpolationMode::RUCKIG;
    position_control_state_ = PositionControlState::POSITION_ACTIVE;
    syncPositionTargetsToCurrent_();
    setRobotState(RobotState::Stopped, reason);
    return true;
}

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/qos.hpp>

#include <robot_control_msg/msg/cartesian_execution_status.hpp>
#include <robot_control_msg/msg/end_effector_pose.hpp>
#include <robot_control_msg/msg/robotarmmovel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <robot_control_msg/msg/robotarmmovelpath.hpp>
#include <robot_control_msg/msg/robotarmservomsg.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <ruckig/ruckig.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace teleop_control
{

/*
 * 多段 MoveL 路径节点：
 * 1. 订阅上层发送的笛卡尔路径命令 `/arm_cartesian_path_cmd`
 * 2. 将“当前 TCP -> 多个 waypoint”构造成由直线段和圆弧过渡段组成的几何路径
 * 3. 左右臂分别使用 1 维 Ruckig 对“路径长度 l（单位 m）”做时间参数化
 * 4. 按采样时刻回查左右臂 TCP 目标位姿，再通过双臂 IK 求解对应关节角
 * 5. 将整条离线轨迹缓存后，按固定周期发布到 `/arm_axis_position_cmd`
 *
 * 这里特意没有把位置/姿态 6 维量直接送进 Ruckig，而是先做几何路径、再做按路径长度的时间规划，
 * 这样可以明确保证 TCP 在空间中走的是“直线 + 圆弧过渡”的轨迹。
 * 如果消息中的 blend 半径数组为空，则节点会根据相邻段长和夹角自动估算一个默认圆角半径。
 */
class ArmIKMoveLPathNode : public rclcpp::Node
{
public:
  ArmIKMoveLPathNode() : Node("arm_ik_moveL_path_node")
  {
    constexpr double kTrajectoryDtSec = 0.002;  // 2ms
    constexpr auto kTrajectoryPublishPeriod = std::chrono::milliseconds(2);

    /* ------------------- 模型与末端帧参数 ------------------- */
    urdf_path_ = this->declare_parameter<std::string>(
      "urdf_path", "");
    ee_frame_left_ = this->declare_parameter<std::string>("ee_frame_left", "lee_link");
    ee_frame_right_ = this->declare_parameter<std::string>("ee_frame_right", "ree_link");
    joint_state_topic_ = this->declare_parameter<std::string>(
      "joint_state_topic", "/arm/joint_states");
    marker_frame_id_ = this->declare_parameter<std::string>("marker_frame_id", "base_link");
    const int configured_trajectory_stride = this->declare_parameter<int>("trajectory_marker_stride", 1);
    trajectory_marker_stride_ = std::max(1, configured_trajectory_stride);
    trajectory_line_width_ = this->declare_parameter<double>("trajectory_line_width", 0.004);
    pose_axis_length_ = this->declare_parameter<double>("pose_axis_length", 0.08);
    pose_axis_shaft_diameter_ = this->declare_parameter<double>("pose_axis_shaft_diameter", 0.006);
    pose_axis_head_diameter_ = this->declare_parameter<double>("pose_axis_head_diameter", 0.012);
    if (configured_trajectory_stride != trajectory_marker_stride_) {
      RCLCPP_WARN(
        this->get_logger(),
        "trajectory_marker_stride %d is invalid, clamped to %d",
        configured_trajectory_stride,
        trajectory_marker_stride_);
    }

    if (!loadModel()) {
      rclcpp::shutdown();
      return;
    }
    
    // 离线轨迹采样周期与下发周期统一为 2ms，让轨迹点更密、关节跟随更细。
    offline_dt_ = kTrajectoryDtSec;
    const double configured_tcp_pose_publish_rate_hz =
      this->declare_parameter<double>("tcp_pose_publish_rate_hz", 50.0);
    tcp_pose_publish_rate_hz_ = std::max(1.0, configured_tcp_pose_publish_rate_hz);
    if (std::abs(configured_tcp_pose_publish_rate_hz - tcp_pose_publish_rate_hz_) > 1e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "tcp_pose_publish_rate_hz %.3f is invalid, clamped to %.3f",
        configured_tcp_pose_publish_rate_hz, tcp_pose_publish_rate_hz_);
    }

    /* ------------------- 轨迹参数初始化 ------------------- */
    // 自动圆角时，不直接给半径，而是先取相邻较短段的一部分作为切点退让距离：
    // offset = auto_blend_tangent_ratio * min(len_prev, len_next)
    // 再由 radius = offset / tan(theta / 2) 反推出默认圆角半径。
    const double configured_auto_blend_tangent_ratio =
      this->declare_parameter<double>("auto_blend_tangent_ratio", 0.05);
    auto_blend_tangent_ratio_ = std::clamp(configured_auto_blend_tangent_ratio, 0.05, 0.45);
    if (std::abs(configured_auto_blend_tangent_ratio - auto_blend_tangent_ratio_) > 1e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "auto_blend_tangent_ratio %.3f is out of range, clamped to %.3f",
        configured_auto_blend_tangent_ratio, auto_blend_tangent_ratio_);
    }

    /* ------------------- TCP 变换初始化 ------------------- */
    // TCP 默认为与末端执行器坐标系重合，后续可通过参数动态调整。
    tcp_transform_left_.setIdentity();
    tcp_transform_right_.setIdentity();

    /* ------------------- TCP 参数声明 ------------------- */
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

    /* ------------------- TCP 参数动态更新 ------------------- */
    // 当 TCP 参数被外部更新时，实时重建 TCP -> EE 的固定位姿变换。
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto on_tcp_param_change = [this]([[maybe_unused]] const rclcpp::Parameter& param) {
      updateTCPTransforms();
    };

    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_x", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_y", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_z", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_roll", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_pitch", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_left_yaw", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_x", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_y", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_z", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_roll", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_pitch", on_tcp_param_change));
    tcp_param_callbacks_.push_back(param_handler_->add_parameter_callback("tcp_right_yaw", on_tcp_param_change));
    updateTCPTransforms();

    /* ------------------- Ruckig 路径长度初始化 ------------------- */
    // 左右臂各自沿本臂路径长度 l（单位 m）做 1 维时间参数化，速度/加速度语义更直观。
    otg_left_length_ = std::make_unique<ruckig::Ruckig<1>>(offline_dt_);
    otg_right_length_ = std::make_unique<ruckig::Ruckig<1>>(offline_dt_);
    initLengthInput(length_input_template_);

    /* ------------------- ROS 通信 ------------------- */
    // 关节状态用于获取当前机械臂起始姿态。
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 50,
      std::bind(&ArmIKMoveLPathNode::jointStateCallback, this, std::placeholders::_1));

    single_cmd_sub_ = this->create_subscription<robot_control_msg::msg::Robotarmmovel>(
      "/arm_cartrsian_position_cmd", 10,
      std::bind(&ArmIKMoveLPathNode::singleCmdCallback, this, std::placeholders::_1));

    // 多段笛卡尔路径命令入口。
    path_cmd_sub_ = this->create_subscription<robot_control_msg::msg::Robotarmmovelpath>(
      "/arm_cartesian_path_cmd", 10,
      std::bind(&ArmIKMoveLPathNode::pathCmdCallback, this, std::placeholders::_1));

    // 轨迹最终仍然发布为底层关节伺服命令，保持控制链路兼容。
    joint_pub_ = this->create_publisher<robot_control_msg::msg::Robotarmservomsg>(
      "/arm_axis_position_cmd", 10);

    tcp_pose_pub_ = this->create_publisher<robot_control_msg::msg::EndEffectorPose>(
      "/arm_tcp_pose", 10);
    ee_pose_pub_ = this->create_publisher<robot_control_msg::msg::EndEffectorPose>(
      "/end_effector_pose", 10);

    rclcpp::QoS marker_qos(1);
    marker_qos.reliable();
    marker_qos.transient_local();
    single_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm_movel_visualization", marker_qos);
    path_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm_movel_path_visualization", marker_qos);

    rclcpp::QoS execution_status_qos(1);
    execution_status_qos.reliable();
    execution_status_qos.transient_local();
    execution_status_pub_ =
      this->create_publisher<robot_control_msg::msg::CartesianExecutionStatus>(
        "/arm_cartesian_path_execution_status", execution_status_qos);

    // 定时从离线缓存中逐点下发关节轨迹。
    publish_timer_ = this->create_wall_timer(
      kTrajectoryPublishPeriod, std::bind(&ArmIKMoveLPathNode::publishNext, this));

    const auto tcp_pose_publish_period = std::chrono::milliseconds(
      std::max<int>(1, static_cast<int>(std::lround(1000.0 / tcp_pose_publish_rate_hz_))));
    tcp_pose_timer_ = this->create_wall_timer(
      tcp_pose_publish_period, std::bind(&ArmIKMoveLPathNode::publishIdlePoseMarkers_, this));

    publishExecutionStatus(
      robot_control_msg::msg::CartesianExecutionStatus::IDLE, "Path node idle");
    clearVisualizationMarkers_();

    RCLCPP_INFO(
      this->get_logger(),
      "arm_ik_moveL_path_node started. Listening on /arm_cartesian_path_cmd, publishing to /arm_axis_position_cmd, auto_blend_tangent_ratio=%.3f",
      auto_blend_tangent_ratio_);
  }

private:
  // TCP 位姿的简单封装，统一用“位置 + 四元数姿态”表示。
  struct CartesianPose
  {
    Eigen::Vector3d position {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation {Eigen::Quaterniond::Identity()};
  };

  struct ArmVisualizationPoses
  {
    CartesianPose left_ee;
    CartesianPose right_ee;
    CartesianPose left_tcp;
    CartesianPose right_tcp;
  };

  enum class ActiveCommandSource
  {
    None,
    Single,
    Path
  };

  // 单臂路径由两类基本元组成：直线段和圆弧段。
  enum class PrimitiveType
  {
    Line,
    Arc,
    InPlaceOrientation
  };

  struct NormalizedPathRequest
  {
    ActiveCommandSource source {ActiveCommandSource::None};
    bool use_left {false};
    bool use_right {false};
    bool allow_in_place_orientation_segment {false};
    std::vector<CartesianPose> left_waypoints;
    std::vector<CartesianPose> right_waypoints;
    std::vector<double> left_blend_radii;
    std::vector<double> right_blend_radii;
    double vel {0.0};
    double acc {0.0};
    std::string left_orientation_source {"path_pose"};
    std::string right_orientation_source {"path_pose"};
  };

  /*
   * 单个路径元的完整几何描述：
   * - length: 当前路径元弧长/线长，单位 m
   * - cumulative_end: 从整条路径起点累计到当前路径元结束时的长度，单位 m
   * - start_pose/end_pose: 当前路径元的起止 TCP 位姿
   * - center/radial_start/axis/angle: 仅圆弧段使用，用于按弧长反求圆弧上的任一点
   */
  struct CartesianPrimitive
  {
    PrimitiveType type {PrimitiveType::Line};
    double length {0.0};
    double cumulative_end {0.0};
    double orientation_progress {0.0};
    CartesianPose start_pose;
    CartesianPose end_pose;
    Eigen::Vector3d center {Eigen::Vector3d::Zero()};
    Eigen::Vector3d radial_start {Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis {Eigen::Vector3d::UnitZ()};
    double angle {0.0};
  };

  /*
   * 一个拐点的圆弧过渡中间结果：
   * - pose_in / pose_out 是圆弧与前后直线的切点
   * - center / axis / radial_start / angle 描述圆弧本身
   * 如果 valid=false，表示该拐点退化为普通折线连接。
   */
  struct CornerBlend
  {
    bool valid {false};
    double radius {0.0};
    double angle {0.0};
    CartesianPose pose_in;
    CartesianPose pose_out;
    Eigen::Vector3d center {Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis {Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d radial_start {Eigen::Vector3d::Zero()};
  };

  // 单臂规划后的完整离线路径。若 moving=false，则整段保持 hold_pose 不动。
  struct PlannedArmPath
  {
    bool enabled {false};
    bool moving {false};
    bool auto_blend_enabled {false};
    CartesianPose hold_pose;
    std::vector<CartesianPrimitive> primitives;
    double total_length {0.0};
    double min_arc_radius {0.0};
    size_t corner_count {0};
    size_t arc_count {0};
  };

  /* === URDF 与关节索引初始化 === */
  bool loadModel()
  {
    try {
      // 从 URDF 构建 Pinocchio 模型，后续 FK/Jacobian/IK 都依赖该模型。
      pinocchio::urdf::buildModel(urdf_path_, model_);
      data_ = pinocchio::Data(model_);
      ee_id_left_ = model_.getFrameId(ee_frame_left_);
      ee_id_right_ = model_.getFrameId(ee_frame_right_);

      if (ee_id_left_ == static_cast<size_t>(-1) || ee_id_right_ == static_cast<size_t>(-1)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot find end-effector frames in URDF");
        return false;
      }

      // 建立“关节名 -> Pinocchio 索引”的查表，便于从 JointState 回填 q_current_。
      for (int i = 0; i < model_.nq; ++i) {
        const std::string joint_name = model_.names[i + 1];
        joint_index_map_[joint_name] = i;
      }

      const std::vector<std::string> left_arm_joints {
        "ljoint1", "ljoint2", "ljoint3", "ljoint4", "ljoint5", "ljoint6", "ljoint7"
      };
      const std::vector<std::string> right_arm_joints {
        "rjoint1", "rjoint2", "rjoint3", "rjoint4", "rjoint5", "rjoint6", "rjoint7"
      };
      required_planning_joint_names_.clear();
      required_planning_joint_names_.insert(
        required_planning_joint_names_.end(), left_arm_joints.begin(), left_arm_joints.end());
      required_planning_joint_names_.insert(
        required_planning_joint_names_.end(), right_arm_joints.begin(), right_arm_joints.end());
      required_planning_joint_name_set_.clear();
      required_planning_joint_name_set_.insert(
        required_planning_joint_names_.begin(), required_planning_joint_names_.end());

      // 只保留 nq/nv 范围内可直接参与 Jacobian 求解的关节索引。
      auto append_joint_indices = [this](const std::vector<std::string>& names, std::vector<int>& out_indices) {
        for (const auto& name : names) {
          const auto it = joint_index_map_.find(name);
          if (it == joint_index_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "Could not find joint: %s", name.c_str());
            continue;
          }

          if (it->second < model_.nv) {
            out_indices.push_back(it->second);
          }
        }
      };

      append_joint_indices(left_arm_joints, left_arm_indices_);
      append_joint_indices(right_arm_joints, right_arm_indices_);

      // 当前实现假定双臂各有 7 个自由度，不满足则直接拒绝启动。
      if (left_arm_indices_.size() != 7 || right_arm_indices_.size() != 7) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to resolve all arm joints. Left: %zu, Right: %zu",
          left_arm_indices_.size(), right_arm_indices_.size());
        return false;
      }

      // arm_joint_indices_ 用于后续批量处理“与机械臂相关的所有关节”。
      arm_joint_indices_.insert(arm_joint_indices_.end(), left_arm_indices_.begin(), left_arm_indices_.end());
      arm_joint_indices_.insert(arm_joint_indices_.end(), right_arm_indices_.begin(), right_arm_indices_.end());

      // q_current_ 保存当前整机关节状态快照。
      q_current_.setZero(model_.nq);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "URDF load error: %s", e.what());
      return false;
    }
  }

  /* === Ruckig 1 维路径长度输入初始化 === */
  static void initLengthInput(ruckig::InputParameter<1>& input)
  {
    // 路径长度变量 l 的单位是米。这里先填默认值，真正规划时会覆盖目标和约束。
    input.current_position.fill(0.0);
    input.current_velocity.fill(0.0);
    input.current_acceleration.fill(0.0);
    input.target_position.fill(0.0);
    input.target_velocity.fill(0.0);
    input.target_acceleration.fill(0.0);
    input.max_velocity.fill(1.0);
    input.max_acceleration.fill(1.0);
    input.max_jerk.fill(1.0);
  }

  /* === 工具函数：RPY -> 旋转矩阵 === */
  static Eigen::Matrix3d rpyToRot(double r, double p, double y)
  {
    // 采用 ZYX 旋转顺序，与现有节点保持一致。
    const Eigen::AngleAxisd ax(r, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ay(p, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd az(y, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond q = az * ay * ax;
    return q.toRotationMatrix();
  }

  // 将三角函数输入裁剪到 [-1, 1]，避免数值误差导致 acos 越界。
  static double clampUnit(double value)
  {
    return std::clamp(value, -1.0, 1.0);
  }

  // 计算两个四元数之间的最小角距离，单位 rad，用于判断姿态是否几乎相同。
  static double quaternionAngularDistance(
    const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
  {
    const double dot = std::abs(a.normalized().dot(b.normalized()));
    return 2.0 * std::acos(clampUnit(dot));
  }

  // 统一归一化四元数，并把 w 调整为非负，减少 slerp 和比较时的符号歧义。
  static Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond& q)
  {
    Eigen::Quaterniond normalized = q;
    normalized.normalize();
    if (normalized.w() < 0.0) {
      normalized.coeffs() *= -1.0;
    }
    return normalized;
  }

  static Eigen::Vector3d matrixToRpy(const Eigen::Matrix3d& rotation)
  {
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::asin(-clampUnit(rotation(2, 0)));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return {roll, pitch, yaw};
  }

  static const char* sourceName(ActiveCommandSource source)
  {
    switch (source) {
      case ActiveCommandSource::Single:
        return "single";
      case ActiveCommandSource::Path:
        return "path";
      case ActiveCommandSource::None:
      default:
        return "none";
    }
  }

  static std::string classifyIKFailureSide_(
    double left_pos_error,
    double left_rot_error,
    double right_pos_error,
    double right_rot_error,
    double pos_eps,
    double rot_eps)
  {
    const double safe_pos_eps = std::max(pos_eps, 1e-9);
    const double safe_rot_eps = std::max(rot_eps, 1e-9);
    const double left_ratio = std::max(left_pos_error / safe_pos_eps, left_rot_error / safe_rot_eps);
    const double right_ratio = std::max(right_pos_error / safe_pos_eps, right_rot_error / safe_rot_eps);

    if (left_ratio <= 1.0 && right_ratio <= 1.0) {
      return "unknown";
    }
    if (left_ratio > 1.0 && right_ratio <= 1.0) {
      return "left";
    }
    if (right_ratio > 1.0 && left_ratio <= 1.0) {
      return "right";
    }
    if (std::abs(left_ratio - right_ratio) <= 1e-6) {
      return "both";
    }
    return left_ratio > right_ratio ? "both(left_dominant)" : "both(right_dominant)";
  }

  static double normalizedIKErrorRatio_(
    double left_pos_error,
    double left_rot_error,
    double right_pos_error,
    double right_rot_error,
    double pos_eps,
    double rot_eps)
  {
    const double safe_pos_eps = std::max(pos_eps, 1e-9);
    const double safe_rot_eps = std::max(rot_eps, 1e-9);
    return std::max({
      left_pos_error / safe_pos_eps,
      left_rot_error / safe_rot_eps,
      right_pos_error / safe_pos_eps,
      right_rot_error / safe_rot_eps});
  }

  static double normQuat(double x, double y, double z, double w)
  {
    return std::abs(x) + std::abs(y) + std::abs(z) + std::abs(w);
  }

  bool isPlanningStateReady_() const
  {
    return !required_planning_joint_names_.empty() &&
      received_planning_joint_names_.size() == required_planning_joint_names_.size();
  }

  std::string missingPlanningJointSummary_() const
  {
    std::ostringstream oss;
    bool first = true;
    for (const auto& joint_name : required_planning_joint_names_) {
      if (received_planning_joint_names_.count(joint_name) > 0) {
        continue;
      }
      if (!first) {
        oss << ',';
      }
      oss << joint_name;
      first = false;
    }
    return oss.str();
  }

  std::string formatPoseSummary_(const CartesianPose& pose) const
  {
    const Eigen::Vector3d rpy = matrixToRpy(pose.orientation.toRotationMatrix());
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "pos=(" << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z() << ") "
        << "quat=(" << pose.orientation.x() << ',' << pose.orientation.y() << ','
        << pose.orientation.z() << ',' << pose.orientation.w() << ") "
        << "rpy=(" << rpy.x() << ',' << rpy.y() << ',' << rpy.z() << ')';
    return oss.str();
  }

  std::string formatSeedJointSummary_(const Eigen::VectorXd& q) const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    bool first = true;
    for (size_t i = 0; i < left_arm_indices_.size(); ++i) {
      if (!first) {
        oss << ' ';
      }
      oss << "l" << (i + 1) << '=' << q[left_arm_indices_[i]];
      first = false;
    }
    for (size_t i = 0; i < right_arm_indices_.size(); ++i) {
      if (!first) {
        oss << ' ';
      }
      oss << "r" << (i + 1) << '=' << q[right_arm_indices_[i]];
      first = false;
    }
    return oss.str();
  }

  void clearPlanningFailureMessage_()
  {
    last_planning_failure_message_.clear();
  }

  void setPlanningFailureMessage_(std::string message)
  {
    last_planning_failure_message_ = std::move(message);
  }

  std::string planningFailureMessageOr_(const std::string& fallback) const
  {
    return last_planning_failure_message_.empty() ? fallback : last_planning_failure_message_;
  }

  std::string composePlanningFailureMessage_(
    const char* stage,
    const std::string& reason,
    const NormalizedPathRequest& request,
    const std::string& extra) const
  {
    std::ostringstream oss;
    oss << "stage=" << stage
        << " source=" << sourceName(request.source)
        << " reason=" << reason;
    if (!extra.empty()) {
      oss << ' ' << extra;
    }
    return oss.str();
  }

  void logPlanningRequestStart_(
    const NormalizedPathRequest& request,
    const CartesianPose& current_left_tcp,
    const CartesianPose& current_right_tcp) const
  {
    const CartesianPose target_left =
      request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : current_left_tcp;
    const CartesianPose target_right =
      request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : current_right_tcp;

    RCLCPP_INFO(
      this->get_logger(),
      "planning_start source=%s vel=%.4f acc=%.4f left_orientation_source=%s right_orientation_source=%s left_waypoints=%zu right_waypoints=%zu",
      sourceName(request.source),
      request.vel,
      request.acc,
      request.left_orientation_source.c_str(),
      request.right_orientation_source.c_str(),
      request.left_waypoints.size(),
      request.right_waypoints.size());
    RCLCPP_INFO(
      this->get_logger(),
      "current_left_tcp: %s",
      formatPoseSummary_(current_left_tcp).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "target_left_tcp: %s",
      formatPoseSummary_(target_left).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "current_right_tcp: %s",
      formatPoseSummary_(current_right_tcp).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "target_right_tcp: %s",
      formatPoseSummary_(target_right).c_str());
  }

  void logPlanningFailure_(
    const char* stage,
    const std::string& reason,
    const NormalizedPathRequest& request,
    const CartesianPose& current_left_tcp,
    const CartesianPose& current_right_tcp,
    const CartesianPose& target_left_tcp,
    const CartesianPose& target_right_tcp,
    const std::string& extra = "")
  {
    setPlanningFailureMessage_(composePlanningFailureMessage_(stage, reason, request, extra));
    RCLCPP_WARN(
      this->get_logger(),
      "planning_failed stage=%s source=%s reason=%s vel=%.4f acc=%.4f left_orientation_source=%s right_orientation_source=%s",
      stage,
      sourceName(request.source),
      reason.c_str(),
      request.vel,
      request.acc,
      request.left_orientation_source.c_str(),
      request.right_orientation_source.c_str());
    RCLCPP_WARN(
      this->get_logger(),
      "failure_current_left_tcp: %s",
      formatPoseSummary_(current_left_tcp).c_str());
    RCLCPP_WARN(
      this->get_logger(),
      "failure_target_left_tcp: %s",
      formatPoseSummary_(target_left_tcp).c_str());
    RCLCPP_WARN(
      this->get_logger(),
      "failure_current_right_tcp: %s",
      formatPoseSummary_(current_right_tcp).c_str());
    RCLCPP_WARN(
      this->get_logger(),
      "failure_target_right_tcp: %s",
      formatPoseSummary_(target_right_tcp).c_str());
    if (!extra.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s", extra.c_str());
    }
  }

  // 当用户没有显式传 blend 半径时，按照“相邻较短段长度 + 当前拐角角度”估算一个默认半径。
  double computeAutoBlendRadius(double len_prev, double len_next, double interior) const
  {
    constexpr double kPosEps = 1e-6;
    const double shorter_segment = std::min(len_prev, len_next);
    if (shorter_segment <= kPosEps) {
      return 0.0;
    }

    const double tan_half = std::tan(interior * 0.5);
    if (std::abs(tan_half) <= kPosEps) {
      return 0.0;
    }

    const double max_offset = 0.5 * shorter_segment - 1e-6;
    if (max_offset <= kPosEps) {
      return 0.0;
    }

    // 自动模式优先固定“切点离拐点退多远”，再反推出半径。
    // 这样半径会同时受到段长和角度影响，而且天然不会超过可行几何范围。
    const double desired_offset = std::min(shorter_segment * auto_blend_tangent_ratio_, max_offset);
    if (desired_offset <= kPosEps) {
      return 0.0;
    }

    return desired_offset / tan_half;
  }

  double computeArcVelocityLimit(double radius, double max_acceleration) const
  {
    constexpr double kPosEps = 1e-6;
    if (radius <= kPosEps || max_acceleration <= kPosEps) {
      return 0.0;
    }

    // 圆弧段额外受到法向加速度 a_n = v^2 / r 的限制，因此局部允许速度为 sqrt(a * r)。
    return std::sqrt(max_acceleration * radius);
  }

  /* === 解析消息中的 Pose === */
  bool parsePose(
    const geometry_msgs::msg::Pose& msg_pose, CartesianPose& pose, std::string& error) const
  {
    // 位置直接拷贝，姿态统一转为归一化四元数。
    pose.position = Eigen::Vector3d(
      msg_pose.position.x, msg_pose.position.y, msg_pose.position.z);

    Eigen::Quaterniond q(
      msg_pose.orientation.w, msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z);
    const double norm = q.norm();
    if (norm < 1e-8) {
      error = "Pose quaternion norm is zero";
      return false;
    }

    pose.orientation = normalizeQuaternion(q);
    return true;
  }

  /* === 生成规划起始关节状态 === */
  Eigen::VectorXd makePlanningState() const
  {
    Eigen::VectorXd q_plan_start = q_current_;
    // 当前路径节点只关心机械臂相关关节；其它关节在规划中置零，避免它们污染 FK/IK。
    for (int i = 0; i < model_.nq; ++i) {
      if (std::find(arm_joint_indices_.begin(), arm_joint_indices_.end(), i) == arm_joint_indices_.end()) {
        q_plan_start[i] = 0.0;
      }
    }

    // 保留当前工程里已有的特殊关节固定值，保证与现有单段节点行为一致。
    if (model_.nq > 21) {
      q_plan_start[21] = 1.0;
    }
    if (model_.nq > 23) {
      q_plan_start[23] = 1.0;
    }
    return q_plan_start;
  }

  /* === 通过 FK 获取当前 TCP 位姿 === */
  CartesianPose getCurrentTCPPose(const Eigen::VectorXd& q_state, size_t ee_id, const pinocchio::SE3& tcp_tf)
  {
    // 先得到末端执行器 EE 的世界位姿，再右乘固定 TCP 变换得到真正控制点 TCP。
    pinocchio::forwardKinematics(model_, data_, q_state);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3 tcp_pose = data_.oMf[ee_id] * tcp_tf;

    CartesianPose pose;
    pose.position = tcp_pose.translation();
    pose.orientation = normalizeQuaternion(Eigen::Quaterniond(tcp_pose.rotation()));
    return pose;
  }

  static geometry_msgs::msg::Point toPointMsg_(const Eigen::Vector3d& point)
  {
    geometry_msgs::msg::Point msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    return msg_point;
  }

  bool computeArmVisualizationPoses_(const Eigen::VectorXd& q_state, ArmVisualizationPoses& poses)
  {
    if (q_state.size() != model_.nq) {
      return false;
    }

    pinocchio::forwardKinematics(model_, data_, q_state);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3 left_ee_pose = data_.oMf[ee_id_left_];
    const pinocchio::SE3 right_ee_pose = data_.oMf[ee_id_right_];
    const pinocchio::SE3 left_tcp_pose = left_ee_pose * tcp_transform_left_;
    const pinocchio::SE3 right_tcp_pose = right_ee_pose * tcp_transform_right_;

    poses.left_ee.position = left_ee_pose.translation();
    poses.left_ee.orientation = normalizeQuaternion(Eigen::Quaterniond(left_ee_pose.rotation()));
    poses.right_ee.position = right_ee_pose.translation();
    poses.right_ee.orientation = normalizeQuaternion(Eigen::Quaterniond(right_ee_pose.rotation()));
    poses.left_tcp.position = left_tcp_pose.translation();
    poses.left_tcp.orientation = normalizeQuaternion(Eigen::Quaterniond(left_tcp_pose.rotation()));
    poses.right_tcp.position = right_tcp_pose.translation();
    poses.right_tcp.orientation = normalizeQuaternion(Eigen::Quaterniond(right_tcp_pose.rotation()));
    return true;
  }

  bool hasTrajectoryExtent_(const std::vector<geometry_msgs::msg::Point>& points) const
  {
    if (points.size() < 2) {
      RCLCPP_DEBUG(this->get_logger(), "hasTrajectoryExtent_: points.size()=%zu < 2", points.size());
      return false;
    }

    const auto& first = points.front();
    double max_distance_sq = 0.0;
    
    for (const auto& point : points) {
      const double dx = point.x - first.x;
      const double dy = point.y - first.y;
      const double dz = point.z - first.z;
      const double distance_sq = dx * dx + dy * dy + dz * dz;
      max_distance_sq = std::max(max_distance_sq, distance_sq);
      
      if (distance_sq > 1e-9) {  // 增大阈值：从1e-12改为1e-9
        RCLCPP_DEBUG(this->get_logger(), "hasTrajectoryExtent_: distance_sq=%.6e > 1e-9, returning true", distance_sq);
        return true;
      }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "hasTrajectoryExtent_: max_distance_sq=%.6e <= 1e-9, returning false", max_distance_sq);
    return false;
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker_() const
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  void appendLineStripMarker_(
    visualization_msgs::msg::MarkerArray& marker_array,
    int marker_id,
    const std::string& marker_ns,
    const std::vector<geometry_msgs::msg::Point>& points,
    float red,
    float green,
    float blue,
    float alpha)
  {
    if (!hasTrajectoryExtent_(points)) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = this->now();
    marker.ns = marker_ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = trajectory_line_width_;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.points = points;
    marker_array.markers.push_back(std::move(marker));
  }

  void appendPoseAxisMarkers_(
    visualization_msgs::msg::MarkerArray& marker_array,
    int base_marker_id,
    const std::string& marker_ns,
    const CartesianPose& pose)
  {
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    const std::array<Eigen::Vector3d, 3> axes{
      rotation.col(0),
      rotation.col(1),
      rotation.col(2)};
    const std::array<std::array<float, 3>, 3> colors{{
      {{1.0f, 0.0f, 0.0f}},
      {{0.0f, 1.0f, 0.0f}},
      {{0.0f, 0.0f, 1.0f}},
    }};

    for (size_t axis_index = 0; axis_index < axes.size(); ++axis_index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = marker_frame_id_;
      marker.header.stamp = this->now();
      marker.ns = marker_ns;
      marker.id = base_marker_id + static_cast<int>(axis_index);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = pose_axis_shaft_diameter_;
      marker.scale.y = pose_axis_head_diameter_;
      marker.scale.z = pose_axis_head_diameter_;
      marker.color.r = colors[axis_index][0];
      marker.color.g = colors[axis_index][1];
      marker.color.b = colors[axis_index][2];
      marker.color.a = 1.0f;
      marker.points.push_back(toPointMsg_(pose.position));
      marker.points.push_back(toPointMsg_(pose.position + axes[axis_index] * pose_axis_length_));
      marker_array.markers.push_back(std::move(marker));
    }
  }

  void buildTrajectoryMarkerPoints_(
    std::vector<geometry_msgs::msg::Point>& left_tcp_points,
    std::vector<geometry_msgs::msg::Point>& right_tcp_points,
    std::vector<geometry_msgs::msg::Point>& left_ee_points,
    std::vector<geometry_msgs::msg::Point>& right_ee_points)
  {
    left_tcp_points.clear();
    right_tcp_points.clear();
    left_ee_points.clear();
    right_ee_points.clear();

    if (joint_traj_.empty()) {
      return;
    }

    const size_t stride = static_cast<size_t>(std::max(1, trajectory_marker_stride_));
    for (size_t trajectory_index = 0; trajectory_index < joint_traj_.size(); trajectory_index += stride) {
      ArmVisualizationPoses poses;
      if (!computeArmVisualizationPoses_(joint_traj_[trajectory_index], poses)) {
        continue;
      }

      left_tcp_points.push_back(toPointMsg_(poses.left_tcp.position));
      right_tcp_points.push_back(toPointMsg_(poses.right_tcp.position));
      left_ee_points.push_back(toPointMsg_(poses.left_ee.position));
      right_ee_points.push_back(toPointMsg_(poses.right_ee.position));
    }

    const size_t last_index = joint_traj_.size() - 1;
    if (last_index % stride != 0) {
      ArmVisualizationPoses poses;
      if (computeArmVisualizationPoses_(joint_traj_[last_index], poses)) {
        left_tcp_points.push_back(toPointMsg_(poses.left_tcp.position));
        right_tcp_points.push_back(toPointMsg_(poses.right_tcp.position));
        left_ee_points.push_back(toPointMsg_(poses.left_ee.position));
        right_ee_points.push_back(toPointMsg_(poses.right_ee.position));
      }
    }
  }

  void clearVisualizationMarkers_()
  {
    if (!single_visualization_pub_ && !path_visualization_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(makeDeleteAllMarker_());
    if (single_visualization_pub_) {
      single_visualization_pub_->publish(marker_array);
    }
    if (path_visualization_pub_) {
      path_visualization_pub_->publish(marker_array);
    }
  }

  void publishCurrentPoseMarkers_()
  {
    if ((!single_visualization_pub_ && !path_visualization_pub_) || !has_joint_state_) {
      return;
    }

    publishPoseMarkersForState_(makePlanningState());
  }

  void publishIdlePoseMarkers_()
  {
    if (trajectory_active_) {
      return;
    }

    publishCurrentPoseMarkers_();
  }

  void publishPoseMarkersForState_(const Eigen::VectorXd& q_state)
  {
    if (!single_visualization_pub_ && !path_visualization_pub_) {
      return;
    }

    ArmVisualizationPoses poses;
    if (!computeArmVisualizationPoses_(q_state, poses)) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    appendPoseAxisMarkers_(marker_array, 100, "left_tcp_axes", poses.left_tcp);
    appendPoseAxisMarkers_(marker_array, 103, "right_tcp_axes", poses.right_tcp);
    appendPoseAxisMarkers_(marker_array, 106, "left_ee_axes", poses.left_ee);
    appendPoseAxisMarkers_(marker_array, 109, "right_ee_axes", poses.right_ee);
    if (single_visualization_pub_) {
      single_visualization_pub_->publish(marker_array);
    }
    if (path_visualization_pub_) {
      path_visualization_pub_->publish(marker_array);
    }
  }

  void publishPlannedTrajectoryMarkers_()
  {
    clearVisualizationMarkers_();

    if ((!single_visualization_pub_ && !path_visualization_pub_) || joint_traj_.empty()) {
      publishCurrentPoseMarkers_();
      return;
    }

    std::vector<geometry_msgs::msg::Point> left_tcp_points;
    std::vector<geometry_msgs::msg::Point> right_tcp_points;
    std::vector<geometry_msgs::msg::Point> left_ee_points;
    std::vector<geometry_msgs::msg::Point> right_ee_points;
    buildTrajectoryMarkerPoints_(left_tcp_points, right_tcp_points, left_ee_points, right_ee_points);

    visualization_msgs::msg::MarkerArray marker_array;
    appendLineStripMarker_(marker_array, 0, "planned_left_tcp", left_tcp_points, 0.0f, 1.0f, 1.0f, 1.0f);
    appendLineStripMarker_(marker_array, 1, "planned_right_tcp", right_tcp_points, 1.0f, 0.55f, 0.0f, 1.0f);
    appendLineStripMarker_(marker_array, 2, "planned_left_ee", left_ee_points, 0.0f, 0.35f, 1.0f, 1.0f);
    appendLineStripMarker_(marker_array, 3, "planned_right_ee", right_ee_points, 1.0f, 0.0f, 0.0f, 1.0f);
    if (!marker_array.markers.empty()) {
      if (single_visualization_pub_) {
        single_visualization_pub_->publish(marker_array);
      }
      if (path_visualization_pub_) {
        path_visualization_pub_->publish(marker_array);
      }
    }

    publishCurrentPoseMarkers_();
  }

  void publishObservedPoseTopics_()
  {
    if (!has_joint_state_ || model_.nq == 0) {
      return;
    }

    const Eigen::VectorXd q_observed = makePlanningState();
    ArmVisualizationPoses poses;
    if (!computeArmVisualizationPoses_(q_observed, poses)) {
      return;
    }

    robot_control_msg::msg::EndEffectorPose tcp_msg;
    tcp_msg.left_ee_pose.position.x = poses.left_tcp.position.x();
    tcp_msg.left_ee_pose.position.y = poses.left_tcp.position.y();
    tcp_msg.left_ee_pose.position.z = poses.left_tcp.position.z();
    tcp_msg.left_ee_pose.orientation.x = poses.left_tcp.orientation.x();
    tcp_msg.left_ee_pose.orientation.y = poses.left_tcp.orientation.y();
    tcp_msg.left_ee_pose.orientation.z = poses.left_tcp.orientation.z();
    tcp_msg.left_ee_pose.orientation.w = poses.left_tcp.orientation.w();
    tcp_msg.right_ee_pose.position.x = poses.right_tcp.position.x();
    tcp_msg.right_ee_pose.position.y = poses.right_tcp.position.y();
    tcp_msg.right_ee_pose.position.z = poses.right_tcp.position.z();
    tcp_msg.right_ee_pose.orientation.x = poses.right_tcp.orientation.x();
    tcp_msg.right_ee_pose.orientation.y = poses.right_tcp.orientation.y();
    tcp_msg.right_ee_pose.orientation.z = poses.right_tcp.orientation.z();
    tcp_msg.right_ee_pose.orientation.w = poses.right_tcp.orientation.w();
    tcp_pose_pub_->publish(tcp_msg);

    robot_control_msg::msg::EndEffectorPose ee_msg;
    ee_msg.left_ee_pose.position.x = poses.left_ee.position.x();
    ee_msg.left_ee_pose.position.y = poses.left_ee.position.y();
    ee_msg.left_ee_pose.position.z = poses.left_ee.position.z();
    ee_msg.left_ee_pose.orientation.x = poses.left_ee.orientation.x();
    ee_msg.left_ee_pose.orientation.y = poses.left_ee.orientation.y();
    ee_msg.left_ee_pose.orientation.z = poses.left_ee.orientation.z();
    ee_msg.left_ee_pose.orientation.w = poses.left_ee.orientation.w();
    ee_msg.right_ee_pose.position.x = poses.right_ee.position.x();
    ee_msg.right_ee_pose.position.y = poses.right_ee.position.y();
    ee_msg.right_ee_pose.position.z = poses.right_ee.position.z();
    ee_msg.right_ee_pose.orientation.x = poses.right_ee.orientation.x();
    ee_msg.right_ee_pose.orientation.y = poses.right_ee.orientation.y();
    ee_msg.right_ee_pose.orientation.z = poses.right_ee.orientation.z();
    ee_msg.right_ee_pose.orientation.w = poses.right_ee.orientation.w();
    ee_pose_pub_->publish(ee_msg);
  }

  /* === 校验路径命令合法性 === */
  bool validateCommand(const robot_control_msg::msg::Robotarmmovelpath& cmd, std::string& error) const
  {
    // 至少启用一只手，否则命令没有意义。
    if (!cmd.use_left && !cmd.use_right) {
      error = "At least one arm must be enabled";
      return false;
    }

    // 线速度和线加速度都按路径长度语义解释，因此必须是正数。
    if (cmd.vel <= 0.0 || cmd.acc <= 0.0) {
      error = "vel and acc must be positive";
      return false;
    }

    // 左臂启用时，必须给 waypoint。
    // blend 半径支持两种模式：
    // 1. 显式模式：数组长度必须等于拐点数量
    // 2. 自动模式：数组为空，由节点自动估算每个拐点的圆角半径
    if (cmd.use_left) {
      if (cmd.left_waypoints.empty()) {
        error = "Left arm is enabled but left_waypoints is empty";
        return false;
      }
      if (!cmd.left_blend_radii.empty() &&
          cmd.left_blend_radii.size() != (cmd.left_waypoints.size() > 0 ? cmd.left_waypoints.size() - 1 : 0)) {
        error = "left_blend_radii size does not match left_waypoints";
        return false;
      }
    } else if (!cmd.left_waypoints.empty() || !cmd.left_blend_radii.empty()) {
      error = "Left arm is disabled but left path data is not empty";
      return false;
    }

    // 右臂规则与左臂相同。
    if (cmd.use_right) {
      if (cmd.right_waypoints.empty()) {
        error = "Right arm is enabled but right_waypoints is empty";
        return false;
      }
      if (!cmd.right_blend_radii.empty() &&
          cmd.right_blend_radii.size() != (cmd.right_waypoints.size() > 0 ? cmd.right_waypoints.size() - 1 : 0)) {
        error = "right_blend_radii size does not match right_waypoints";
        return false;
      }
    } else if (!cmd.right_waypoints.empty() || !cmd.right_blend_radii.empty()) {
      error = "Right arm is disabled but right path data is not empty";
      return false;
    }

    auto has_negative_radius = [](const std::vector<double>& radii) {
      return std::any_of(radii.begin(), radii.end(), [](double radius) { return radius < 0.0; });
    };

    // 负半径没有几何意义。
    if (has_negative_radius(cmd.left_blend_radii) || has_negative_radius(cmd.right_blend_radii)) {
      error = "Blend radius must be non-negative";
      return false;
    }

    return true;
  }

  bool validateSingleCommand(const robot_control_msg::msg::Robotarmmovel& cmd, std::string& error) const
  {
    if (cmd.vel <= 0.0 || cmd.acc <= 0.0) {
      error = "vel and acc must be positive";
      return false;
    }
    return true;
  }

  bool buildNormalizedPathRequest_(
    const robot_control_msg::msg::Robotarmmovelpath& cmd,
    NormalizedPathRequest& request,
    std::string& error) const
  {
    request = NormalizedPathRequest {};
    request.source = ActiveCommandSource::Path;
    request.use_left = cmd.use_left;
    request.use_right = cmd.use_right;
    request.allow_in_place_orientation_segment = false;
    request.left_blend_radii = cmd.left_blend_radii;
    request.right_blend_radii = cmd.right_blend_radii;
    request.vel = cmd.vel;
    request.acc = cmd.acc;

    request.left_waypoints.reserve(cmd.left_waypoints.size());
    for (const auto& waypoint_msg : cmd.left_waypoints) {
      CartesianPose pose;
      if (!parsePose(waypoint_msg, pose, error)) {
        return false;
      }
      request.left_waypoints.push_back(pose);
    }

    request.right_waypoints.reserve(cmd.right_waypoints.size());
    for (const auto& waypoint_msg : cmd.right_waypoints) {
      CartesianPose pose;
      if (!parsePose(waypoint_msg, pose, error)) {
        return false;
      }
      request.right_waypoints.push_back(pose);
    }
    return true;
  }

  bool buildNormalizedSingleRequest_(
    const robot_control_msg::msg::Robotarmmovel& cmd,
    NormalizedPathRequest& request,
    std::string& error) const
  {
    (void)error;
    request = NormalizedPathRequest {};
    request.source = ActiveCommandSource::Single;
    request.use_left = true;
    request.use_right = true;
    request.allow_in_place_orientation_segment = true;
    request.vel = cmd.vel;
    request.acc = cmd.acc;

    CartesianPose left_target;
    left_target.position = Eigen::Vector3d(cmd.lx, cmd.ly, cmd.lz);
    if (normQuat(cmd.lqx, cmd.lqy, cmd.lqz, cmd.lqw) > 1e-6) {
      left_target.orientation = normalizeQuaternion(
        Eigen::Quaterniond(cmd.lqw, cmd.lqx, cmd.lqy, cmd.lqz));
      request.left_orientation_source = "quaternion";
    } else {
      left_target.orientation = normalizeQuaternion(
        Eigen::Quaterniond(rpyToRot(cmd.lroll, cmd.lpitch, cmd.lyaw)));
      request.left_orientation_source = "rpy";
    }
    request.left_waypoints.push_back(left_target);

    CartesianPose right_target;
    right_target.position = Eigen::Vector3d(cmd.rx, cmd.ry, cmd.rz);
    if (normQuat(cmd.rqx, cmd.rqy, cmd.rqz, cmd.rqw) > 1e-6) {
      right_target.orientation = normalizeQuaternion(
        Eigen::Quaterniond(cmd.rqw, cmd.rqx, cmd.rqy, cmd.rqz));
      request.right_orientation_source = "quaternion";
    } else {
      right_target.orientation = normalizeQuaternion(
        Eigen::Quaterniond(rpyToRot(cmd.rroll, cmd.rpitch, cmd.ryaw)));
      request.right_orientation_source = "rpy";
    }
    request.right_waypoints.push_back(right_target);
    return true;
  }

  bool planSingleTrajectory(const robot_control_msg::msg::Robotarmmovel& cmd)
  {
    NormalizedPathRequest request;
    std::string error;
    if (!buildNormalizedSingleRequest_(cmd, request, error)) {
      setPlanningFailureMessage_(error);
      RCLCPP_WARN(this->get_logger(), "%s", error.c_str());
      return false;
    }
    return planTrajectory(request);
  }

  /* === 构造单臂几何路径：直线 + 圆弧过渡 === */
  bool buildArmPath(
    const char* arm_name,
    bool enabled,
    const CartesianPose& start_pose,
    const std::vector<CartesianPose>& waypoints,
    const std::vector<double>& blend_radii,
    bool allow_in_place_orientation_segment,
    PlannedArmPath& out_path,
    std::string& error)
  {
    out_path = PlannedArmPath {};
    out_path.enabled = enabled;
    out_path.auto_blend_enabled = blend_radii.empty();
    out_path.hold_pose = start_pose;

    // 未启用的手臂整条路径保持当前 TCP 位姿不动。
    if (!enabled) {
      return true;
    }

    constexpr double kPosEps = 1e-6;
    constexpr double kRotEps = 1e-4;
    constexpr double kAngleEps = 1e-4;
    std::vector<CartesianPose> anchors;
    anchors.reserve(waypoints.size() + 1);
    // 将“当前 TCP 位姿”作为路径起点，与用户提供的 waypoint 串成完整锚点序列。
    anchors.push_back(start_pose);
    anchors.insert(anchors.end(), waypoints.begin(), waypoints.end());

    out_path.corner_count = anchors.size() > 2 ? anchors.size() - 2 : 0;

    std::vector<double> translation_lengths;
    translation_lengths.reserve(anchors.size() - 1);

    for (size_t i = 1; i < anchors.size(); ++i) {
      const double distance = (anchors[i].position - anchors[i - 1].position).norm();
      const double angle_error = quaternionAngularDistance(anchors[i - 1].orientation, anchors[i].orientation);

      if (distance < kPosEps && angle_error > kRotEps) {
        if (!allow_in_place_orientation_segment) {
          error = std::string(arm_name) + " arm path contains a zero-length segment with orientation change";
          return false;
        }
        translation_lengths.push_back(0.0);
        continue;
      }

      translation_lengths.push_back(distance);
    }

    std::vector<CornerBlend> corners(anchors.size());
    for (size_t i = 1; i + 1 < anchors.size(); ++i) {
      const double len_prev = translation_lengths[i - 1];
      const double len_next = translation_lengths[i];
      if (len_prev <= kPosEps || len_next <= kPosEps) {
        continue;
      }

      const Eigen::Vector3d dir_prev = (anchors[i].position - anchors[i - 1].position) / len_prev;
      const Eigen::Vector3d dir_next = (anchors[i + 1].position - anchors[i].position) / len_next;

      // interior 是折线拐角的内角。太接近 0 或 pi 都会退化，不适合构造稳定圆弧。
      const double interior = std::acos(clampUnit((-dir_prev).dot(dir_next)));
      if (interior <= kAngleEps || interior >= (M_PI - kAngleEps)) {
        continue;
      }

      // 第 i 个锚点对应 blend_radii[i - 1]，即“前一段”和“后一段”的过渡半径。
      // 若数组为空，则进入自动圆角模式，根据段长和夹角估算默认半径。
      double requested_radius = 0.0;
      if (out_path.auto_blend_enabled) {
        requested_radius = computeAutoBlendRadius(len_prev, len_next, interior);
      } else {
        requested_radius = blend_radii[i - 1];
      }

      if (requested_radius <= kPosEps) {
        continue;
      }

      // 对于给定圆角半径 r 和内角 θ，切点到拐点的退让距离为 r * tan(θ / 2)。
      const double tan_half = std::tan(interior * 0.5);
      if (std::abs(tan_half) <= kPosEps) {
        continue;
      }

      double offset = requested_radius * tan_half;
      // 若请求半径过大，切点会越过相邻段中点，这里将其裁剪到几何可行范围内。
      const double max_offset = 0.5 * std::min(len_prev, len_next) - 1e-6;
      if (max_offset <= kPosEps) {
        continue;
      }
      offset = std::min(offset, max_offset);

      const double actual_radius = offset / tan_half;
      const Eigen::Vector3d bisector_raw = (-dir_prev + dir_next);
      if (bisector_raw.norm() <= kPosEps) {
        continue;
      }

      const double sin_half = std::sin(interior * 0.5);
      if (std::abs(sin_half) <= kPosEps) {
        continue;
      }

      CornerBlend blend;
      blend.valid = true;
      blend.radius = actual_radius;

      // 计算圆弧进入切点和退出切点。
      blend.pose_in.position = anchors[i].position - dir_prev * offset;
      blend.pose_out.position = anchors[i].position + dir_next * offset;

      // 直线段姿态与圆弧段姿态都按相邻 waypoint 的四元数做 slerp。
      // 这里的 prev_t / next_t 表示切点在相邻原始直线段中的归一化位置。
      const double prev_t = (len_prev - offset) / len_prev;
      const double next_t = offset / len_next;
      blend.pose_in.orientation =
        normalizeQuaternion(anchors[i - 1].orientation.slerp(prev_t, anchors[i].orientation));
      blend.pose_out.orientation =
        normalizeQuaternion(anchors[i].orientation.slerp(next_t, anchors[i + 1].orientation));

      // 圆心位于角平分线上，距离拐点为 r / sin(θ / 2)。
      const Eigen::Vector3d bisector = bisector_raw.normalized();
      const double center_distance = actual_radius / sin_half;
      blend.center = anchors[i].position + bisector * center_distance;
      blend.radial_start = blend.pose_in.position - blend.center;

      const Eigen::Vector3d radial_end = blend.pose_out.position - blend.center;
      // 半径向量退化时，不生成圆弧，保留为普通折线连接。
      if (blend.radial_start.norm() <= kPosEps || radial_end.norm() <= kPosEps) {
        blend.valid = false;
        corners[i] = blend;
        continue;
      }

      // 由起止半径向量叉乘得到圆弧法向，用于后续 AngleAxis 采样。
      const Eigen::Vector3d axis_raw = blend.radial_start.cross(radial_end);
      if (axis_raw.norm() <= kPosEps) {
        blend.valid = false;
        corners[i] = blend;
        continue;
      }

      blend.axis = axis_raw.normalized();
      blend.angle = std::acos(clampUnit(blend.radial_start.normalized().dot(radial_end.normalized())));
      if (blend.angle <= kAngleEps) {
        blend.valid = false;
      }

      corners[i] = blend;
    }

    double cumulative_length = 0.0;
    auto push_primitive = [&out_path, &cumulative_length](CartesianPrimitive primitive) {
      // 每个 primitive 记录“到自己结束时的累计长度”，便于按弧长反查。
      cumulative_length += primitive.length;
      primitive.cumulative_end = cumulative_length;
      out_path.primitives.push_back(std::move(primitive));
    };

    for (size_t segment_index = 0; segment_index + 1 < anchors.size(); ++segment_index) {
      // 一条原始折线段可能会被前后两个圆角切掉一部分，因此真实直线起止点要重新计算。
      const bool has_corner_before = (segment_index > 0) && corners[segment_index].valid;
      const bool has_corner_after = (segment_index + 1 < corners.size()) && corners[segment_index + 1].valid;

      CartesianPose line_start = has_corner_before ? corners[segment_index].pose_out : anchors[segment_index];
      CartesianPose line_end = has_corner_after ? corners[segment_index + 1].pose_in : anchors[segment_index + 1];

      const double line_length = (line_end.position - line_start.position).norm();
      if (line_length > kPosEps) {
        CartesianPrimitive line;
        line.type = PrimitiveType::Line;
        line.length = line_length;
        line.orientation_progress = quaternionAngularDistance(line_start.orientation, line_end.orientation);
        line.start_pose = line_start;
        line.end_pose = line_end;
        push_primitive(std::move(line));
      } else {
        const double orientation_progress = quaternionAngularDistance(line_start.orientation, line_end.orientation);
        if (allow_in_place_orientation_segment && orientation_progress > kRotEps) {
          CartesianPrimitive in_place_orientation;
          in_place_orientation.type = PrimitiveType::InPlaceOrientation;
          in_place_orientation.length = orientation_progress;
          in_place_orientation.orientation_progress = orientation_progress;
          in_place_orientation.start_pose = line_start;
          in_place_orientation.end_pose = line_end;
          push_primitive(std::move(in_place_orientation));
        }
      }

      if (has_corner_after) {
        // 若后一个拐点成功生成圆弧，则把圆弧作为独立 primitive 追加到路径里。
        CartesianPrimitive arc;
        arc.type = PrimitiveType::Arc;
        arc.length = corners[segment_index + 1].radius * corners[segment_index + 1].angle;
        arc.orientation_progress = quaternionAngularDistance(
          corners[segment_index + 1].pose_in.orientation,
          corners[segment_index + 1].pose_out.orientation);
        arc.start_pose = corners[segment_index + 1].pose_in;
        arc.end_pose = corners[segment_index + 1].pose_out;
        arc.center = corners[segment_index + 1].center;
        arc.radial_start = corners[segment_index + 1].radial_start;
        arc.axis = corners[segment_index + 1].axis;
        arc.angle = corners[segment_index + 1].angle;
        if (arc.length > kPosEps) {
          out_path.arc_count += 1;
          if (out_path.min_arc_radius <= 0.0) {
            out_path.min_arc_radius = corners[segment_index + 1].radius;
          } else {
            out_path.min_arc_radius = std::min(out_path.min_arc_radius, corners[segment_index + 1].radius);
          }
          push_primitive(std::move(arc));
        }
      }
    }

    out_path.total_length = cumulative_length;
    // hold_pose 代表该臂整条路径执行完成后的最终 TCP 位姿。
    out_path.hold_pose = anchors.back();
    out_path.moving = out_path.total_length > kPosEps;

    return true;
  }

  /* === 按累计路径长度采样单臂 TCP 位姿 === */
  CartesianPose sampleArmPath(const PlannedArmPath& path, double distance) const
  {
    // 未启用或者总长度为 0 时，整段保持 hold_pose。
    if (!path.moving || path.primitives.empty()) {
      return path.hold_pose;
    }

    const double clamped_distance = std::clamp(distance, 0.0, path.total_length);

    const CartesianPrimitive* primitive = &path.primitives.back();
    for (const auto& candidate : path.primitives) {
      if (clamped_distance <= candidate.cumulative_end + 1e-9) {
        primitive = &candidate;
        break;
      }
    }

    // 将“整条路径上的距离”映射为“当前 primitive 内部的局部距离”。
    const double primitive_start = primitive->cumulative_end - primitive->length;
    const double local_distance = std::clamp(clamped_distance - primitive_start, 0.0, primitive->length);
    const double t = primitive->length > 1e-9 ? (local_distance / primitive->length) : 1.0;

    // 姿态在每个 primitive 内做 slerp：直线段和圆弧段都遵循同一条规则。
    CartesianPose pose;
    pose.orientation = normalizeQuaternion(
      primitive->start_pose.orientation.slerp(t, primitive->end_pose.orientation));

    if (primitive->type == PrimitiveType::Line) {
      // 直线段按线性插值采样位置。
      pose.position = primitive->start_pose.position +
        (primitive->end_pose.position - primitive->start_pose.position) * t;
      return pose;
    }

    if (primitive->type == PrimitiveType::InPlaceOrientation) {
      pose.position = primitive->start_pose.position;
      return pose;
    }

    // 圆弧段按“圆心 + 旋转后的起始半径向量”恢复当前点。
    const Eigen::AngleAxisd rotation(primitive->angle * t, primitive->axis);
    pose.position = primitive->center + rotation * primitive->radial_start;
    return pose;
  }

  void logPathKinematicLimits(
    const char* arm_name, const PlannedArmPath& path, double requested_vel,
    double requested_acc, double effective_vel) const
  {
    if (!path.enabled) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "%s path: length=%.4f m, arcs=%zu, min_arc_radius=%.4f m, requested_vel=%.4f m/s, effective_vel=%.4f m/s, acc=%.4f m/s^2, jerk=%.4f m/s^3",
      arm_name,
      path.total_length,
      path.arc_count,
      path.arc_count > 0 ? path.min_arc_radius : 0.0,
      requested_vel,
      effective_vel,
      requested_acc,
      requested_acc);

    if (path.arc_count > 0 && effective_vel + 1e-9 < requested_vel) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s path velocity limited by arc radius: requested %.4f m/s -> %.4f m/s (acc=%.4f m/s^2, min_radius=%.4f m)",
        arm_name, requested_vel, effective_vel, requested_acc, path.min_arc_radius);
    }

    if (path.corner_count > 0 && path.arc_count == 0 && !path.auto_blend_enabled) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s path has hard corners without arc blend; no centripetal speed limiting is applied at those corners",
        arm_name);
    }

    if (path.corner_count > 0 && path.arc_count == 0 && path.auto_blend_enabled) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s path requested auto blend but no feasible arcs were generated; no centripetal speed limiting is applied",
        arm_name);
    }
  }

  /* === 整条双臂轨迹离线规划 === */
  bool planTrajectory(const NormalizedPathRequest& request)
  {
    // 规划过程中固定使用同一份起始关节状态，避免规划期间 JointState 更新导致起点漂移。
    Eigen::VectorXd q_plan_start = makePlanningState();

    // 先取规划起点时刻的左右臂当前 TCP 位姿，路径从这里开始。
    const CartesianPose start_pose_left = getCurrentTCPPose(q_plan_start, ee_id_left_, tcp_transform_left_);
    const CartesianPose start_pose_right = getCurrentTCPPose(q_plan_start, ee_id_right_, tcp_transform_right_);

    logPlanningRequestStart_(request, start_pose_left, start_pose_right);

    PlannedArmPath left_path;
    PlannedArmPath right_path;
    std::string error;

    if (!buildArmPath(
          "left",
          request.use_left,
          start_pose_left,
          request.left_waypoints,
          request.left_blend_radii,
          request.allow_in_place_orientation_segment,
          left_path,
          error)) {
      const CartesianPose left_target =
        request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : start_pose_left;
      const CartesianPose right_target =
        request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : start_pose_right;
      logPlanningFailure_("buildArmPath", error, request, start_pose_left, start_pose_right, left_target, right_target);
      return false;
    }

    if (!buildArmPath(
          "right",
          request.use_right,
          start_pose_right,
          request.right_waypoints,
          request.right_blend_radii,
          request.allow_in_place_orientation_segment,
          right_path,
          error)) {
      const CartesianPose left_target =
        request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : start_pose_left;
      const CartesianPose right_target =
        request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : start_pose_right;
      logPlanningFailure_("buildArmPath", error, request, start_pose_left, start_pose_right, left_target, right_target);
      return false;
    }

    std::vector<Eigen::VectorXd> planned_points;
    Eigen::VectorXd q_sol = q_plan_start;

    // 左右臂都没有位移时，仍然保留一个静止点，行为上等价于 hold。
    if (!left_path.moving && !right_path.moving) {
      planned_points.push_back(q_plan_start);
      joint_traj_ = std::move(planned_points);
      RCLCPP_INFO(this->get_logger(), "Request resulted in a stationary hold point");
      return true;
    }

    double left_effective_vel = request.vel;
    if (left_path.moving && left_path.arc_count > 0) {
      const double left_arc_vel_limit = computeArcVelocityLimit(left_path.min_arc_radius, request.acc);
      if (left_arc_vel_limit > 0.0) {
        left_effective_vel = std::min(left_effective_vel, left_arc_vel_limit);
      }
    }

    double right_effective_vel = request.vel;
    if (right_path.moving && right_path.arc_count > 0) {
      const double right_arc_vel_limit = computeArcVelocityLimit(right_path.min_arc_radius, request.acc);
      if (right_arc_vel_limit > 0.0) {
        right_effective_vel = std::min(right_effective_vel, right_arc_vel_limit);
      }
    }

    logPathKinematicLimits("Left", left_path, request.vel, request.acc, left_effective_vel);
    logPathKinematicLimits("Right", right_path, request.vel, request.acc, right_effective_vel);

    auto left_input = length_input_template_;
    auto right_input = length_input_template_;
    ruckig::OutputParameter<1> left_output;
    ruckig::OutputParameter<1> right_output;

    if (left_path.moving) {
      left_input.current_position[0] = 0.0;
      left_input.current_velocity[0] = 0.0;
      left_input.current_acceleration[0] = 0.0;
      left_input.target_position[0] = left_path.total_length;
      left_input.target_velocity[0] = 0.0;
      left_input.target_acceleration[0] = 0.0;
      left_input.max_velocity[0] = left_effective_vel;
      left_input.max_acceleration[0] = request.acc;
      left_input.max_jerk[0] = request.acc*4.0f;
    }

    if (right_path.moving) {
      right_input.current_position[0] = 0.0;
      right_input.current_velocity[0] = 0.0;
      right_input.current_acceleration[0] = 0.0;
      right_input.target_position[0] = right_path.total_length;
      right_input.target_velocity[0] = 0.0;
      right_input.target_acceleration[0] = 0.0;
      right_input.max_velocity[0] = right_effective_vel;
      right_input.max_acceleration[0] = request.acc;
      right_input.max_jerk[0] = request.acc*4.0f;
    }

    bool left_finished = !left_path.moving;
    bool right_finished = !right_path.moving;
    double left_current_length = left_finished ? left_path.total_length : 0.0;
    double right_current_length = right_finished ? right_path.total_length : 0.0;
    size_t sample_count = 0;

    while (true) {
      ruckig::Result left_result = ruckig::Result::Finished;
      if (!left_finished) {
        left_result = otg_left_length_->update(left_input, left_output);
        if (left_result < 0) {
          const CartesianPose left_target =
            request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : start_pose_left;
          const CartesianPose right_target =
            request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : start_pose_right;
          logPlanningFailure_(
            "ruckig_left",
            "Left arm path-length Ruckig failed: " + std::to_string(static_cast<int>(left_result)),
            request,
            start_pose_left,
            start_pose_right,
            left_target,
            right_target);
          return false;
        }
        left_current_length = std::clamp(left_output.new_position[0], 0.0, left_path.total_length);
      }

      ruckig::Result right_result = ruckig::Result::Finished;
      if (!right_finished) {
        right_result = otg_right_length_->update(right_input, right_output);
        if (right_result < 0) {
          const CartesianPose left_target =
            request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : start_pose_left;
          const CartesianPose right_target =
            request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : start_pose_right;
          logPlanningFailure_(
            "ruckig_right",
            "Right arm path-length Ruckig failed: " + std::to_string(static_cast<int>(right_result)),
            request,
            start_pose_left,
            start_pose_right,
            left_target,
            right_target);
          return false;
        }
        right_current_length = std::clamp(right_output.new_position[0], 0.0, right_path.total_length);
      }

      const CartesianPose tcp_pose_left = sampleArmPath(left_path, left_current_length);
      const CartesianPose tcp_pose_right = sampleArmPath(right_path, right_current_length);

      const pinocchio::SE3 desired_tcp_left(tcp_pose_left.orientation.toRotationMatrix(), tcp_pose_left.position);
      const pinocchio::SE3 desired_tcp_right(tcp_pose_right.orientation.toRotationMatrix(), tcp_pose_right.position);

      // IK 求解的是 EE 位姿，因此需要把 TCP 目标位姿乘以 TCP 逆变换还原回 EE 目标。
      const pinocchio::SE3 desired_ee_left = desired_tcp_left * tcp_transform_left_.inverse();
      const pinocchio::SE3 desired_ee_right = desired_tcp_right * tcp_transform_right_.inverse();

      // 任一点 IK 失败都视为整条轨迹不可执行，直接拒绝整个命令。
      std::string ik_failure_reason;
      if (!solveDualIK(desired_ee_left, desired_ee_right, q_sol, &ik_failure_reason)) {
        const std::string extra =
          "sample_index=" + std::to_string(sample_count) +
          " left_progress=" + std::to_string(left_current_length) +
          " right_progress=" + std::to_string(right_current_length) +
          " desired_left_tcp=" + formatPoseSummary_(tcp_pose_left) +
          " desired_right_tcp=" + formatPoseSummary_(tcp_pose_right);
        const CartesianPose left_target =
          request.use_left && !request.left_waypoints.empty() ? request.left_waypoints.back() : start_pose_left;
        const CartesianPose right_target =
          request.use_right && !request.right_waypoints.empty() ? request.right_waypoints.back() : start_pose_right;
        logPlanningFailure_(
          "ik",
          ik_failure_reason.empty() ? "IK failed during trajectory sampling" : ik_failure_reason,
          request,
          start_pose_left,
          start_pose_right,
          left_target,
          right_target,
          extra);
        return false;
      }

      planned_points.push_back(q_sol);
      ++sample_count;

      if (!left_finished && left_result == ruckig::Result::Finished) {
        left_finished = true;
        left_current_length = left_path.total_length;
      }

      if (!right_finished && right_result == ruckig::Result::Finished) {
        right_finished = true;
        right_current_length = right_path.total_length;
      }

      if (left_finished && right_finished) {
        break;
      }

      if (!left_finished) {
        left_output.pass_to_input(left_input);
      }
      if (!right_finished) {
        right_output.pass_to_input(right_input);
      }
    }

    joint_traj_ = std::move(planned_points);
    RCLCPP_INFO(
      this->get_logger(),
      "Successfully planned path trajectory with %zu points (left length %.4f m @ %.4f m/s, right length %.4f m @ %.4f m/s, acc %.4f m/s^2, jerk %.4f m/s^3)",
      joint_traj_.size(),
      left_path.total_length, left_effective_vel,
      right_path.total_length, right_effective_vel,
      request.acc, request.acc);
    return !joint_traj_.empty();
  }

  /* === 双臂 IK 求解 === */
  bool solveDualIK(
    const pinocchio::SE3& desired_left,
    const pinocchio::SE3& desired_right,
    Eigen::VectorXd& q,
    std::string* failure_reason = nullptr,
    double pos_eps = 1e-4,
    double rot_eps = 2e-4)
  {
    const int max_iter = 100;
    const std::string seed_q_summary = formatSeedJointSummary_(q);
    double initial_error_ratio = std::numeric_limits<double>::quiet_NaN();
    double best_error_ratio = std::numeric_limits<double>::infinity();

    auto computeIKErrors =
      [this, &q, &desired_left, &desired_right]()
      -> std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>>
      {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);
        const auto current_left = data_.oMf[ee_id_left_];
        const auto current_right = data_.oMf[ee_id_right_];
        return {
          pinocchio::log6(current_left.inverse() * desired_left),
          pinocchio::log6(current_right.inverse() * desired_right)};
      };

    for (int iter = 0; iter < max_iter; ++iter) {
      // 使用 log6 统一表示 6 维位姿误差：前三维是平移误差，后三维是旋转误差。
      const auto [err_left, err_right] = computeIKErrors();
      const double error_ratio = normalizedIKErrorRatio_(
        err_left.head<3>().norm(), err_left.tail<3>().norm(),
        err_right.head<3>().norm(), err_right.tail<3>().norm(),
        pos_eps, rot_eps);
      if (iter == 0) {
        initial_error_ratio = error_ratio;
      }
      best_error_ratio = std::min(best_error_ratio, error_ratio);

      if (error_ratio < 1.0) {
        return true;
      }

      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_left_full(6, model_.nv);
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_right_full(6, model_.nv);
      pinocchio::computeFrameJacobian(model_, data_, q, ee_id_left_, jacobian_left_full);
      pinocchio::computeFrameJacobian(model_, data_, q, ee_id_right_, jacobian_right_full);

      // 从整机 Jacobian 中抽取左右臂各自 7 个关节的列，独立求解双臂更新量。
      Eigen::Matrix<double, 6, 7> jacobian_left;
      Eigen::Matrix<double, 6, 7> jacobian_right;
      for (int j = 0; j < 7; ++j) {
        jacobian_left.col(j) = jacobian_left_full.col(left_arm_indices_[j]);
        jacobian_right.col(j) = jacobian_right_full.col(right_arm_indices_[j]);
      }

      // 使用固定阻尼最小二乘抑制奇异位形附近的数值发散。
      const double damp = 1e-2;
      Eigen::Matrix<double, 6, 6> lhs_left = jacobian_left * jacobian_left.transpose();
      Eigen::Matrix<double, 6, 6> lhs_right = jacobian_right * jacobian_right.transpose();
      lhs_left.diagonal().array() += damp * damp;
      lhs_right.diagonal().array() += damp * damp;

      const Eigen::VectorXd dq_left = jacobian_left.transpose() * lhs_left.ldlt().solve(err_left);
      const Eigen::VectorXd dq_right = jacobian_right.transpose() * lhs_right.ldlt().solve(err_right);

      Eigen::VectorXd dq_left_limited = dq_left;
      Eigen::VectorXd dq_right_limited = dq_right;
      const double max_step = 0.01;
      // 限制单次迭代步长，避免大步更新直接跳出可行域。
      if (dq_left_limited.norm() > max_step) {
        dq_left_limited = dq_left_limited.normalized() * max_step;
      }
      if (dq_right_limited.norm() > max_step) {
        dq_right_limited = dq_right_limited.normalized() * max_step;
      }

      for (int j = 0; j < 7; ++j) {
        const int left_index = left_arm_indices_[j];
        const int right_index = right_arm_indices_[j];
        q[left_index] += dq_left_limited[j];
        q[right_index] += dq_right_limited[j];
      }

      // 每轮更新后都裁剪到关节限位范围内。
      for (int j = 0; j < 14; ++j) {
        const int joint_index = arm_joint_indices_[j];
        q[joint_index] = std::min(
          std::max(q[joint_index], model_.lowerPositionLimit[joint_index]),
          model_.upperPositionLimit[joint_index]);
      }
    }

    const auto [final_err_left, final_err_right] = computeIKErrors();
    if (final_err_left.head<3>().norm() < pos_eps && final_err_left.tail<3>().norm() < rot_eps &&
        final_err_right.head<3>().norm() < pos_eps && final_err_right.tail<3>().norm() < rot_eps) {
      return true;
    }

    const double left_pos_error = final_err_left.head<3>().norm();
    const double left_rot_error = final_err_left.tail<3>().norm();
    const double right_pos_error = final_err_right.head<3>().norm();
    const double right_rot_error = final_err_right.tail<3>().norm();
    const double final_error_ratio = normalizedIKErrorRatio_(
      left_pos_error, left_rot_error, right_pos_error, right_rot_error, pos_eps, rot_eps);
    best_error_ratio = std::min(best_error_ratio, final_error_ratio);

    Eigen::Matrix<double, 6, Eigen::Dynamic> final_jacobian_left_full(6, model_.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> final_jacobian_right_full(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q, ee_id_left_, final_jacobian_left_full);
    pinocchio::computeFrameJacobian(
      model_, data_, q, ee_id_right_, final_jacobian_right_full);
    Eigen::Matrix<double, 6, 7> final_jacobian_left;
    Eigen::Matrix<double, 6, 7> final_jacobian_right;
    for (int j = 0; j < 7; ++j) {
      final_jacobian_left.col(j) = final_jacobian_left_full.col(left_arm_indices_[j]);
      final_jacobian_right.col(j) = final_jacobian_right_full.col(right_arm_indices_[j]);
    }
    const double left_min_singular =
      final_jacobian_left.jacobiSvd().singularValues().minCoeff();
    const double right_min_singular =
      final_jacobian_right.jacobiSvd().singularValues().minCoeff();
    constexpr double kNearSingularValue = 1e-3;
    const bool left_failed = left_pos_error >= pos_eps || left_rot_error >= rot_eps;
    const bool right_failed = right_pos_error >= pos_eps || right_rot_error >= rot_eps;
    const bool near_singular =
      (left_failed && left_min_singular < kNearSingularValue) ||
      (right_failed && right_min_singular < kNearSingularValue);
    const auto armAtJointLimit = [this](const std::vector<int>& indices, const Eigen::VectorXd& state) {
      constexpr double kLimitTolerance = 1e-6;
      return std::any_of(indices.begin(), indices.end(), [this, &state](int index) {
        return state[index] <= model_.lowerPositionLimit[index] + kLimitTolerance ||
               state[index] >= model_.upperPositionLimit[index] - kLimitTolerance;
      });
    };
    const bool left_joint_limit = armAtJointLimit(left_arm_indices_, q);
    const bool right_joint_limit = armAtJointLimit(right_arm_indices_, q);
    const char* convergence_hint =
      (left_failed && left_joint_limit) || (right_failed && right_joint_limit) ?
      "joint_limit_reached" :
      (near_singular ? "stagnated_near_singularity_or_workspace_boundary" : "iteration_limit");

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "Dual-arm IK did not converge after " << max_iter
        << " iterations failing_side="
        << classifyIKFailureSide_(
             left_pos_error, left_rot_error, right_pos_error, right_rot_error, pos_eps, rot_eps)
        << " left_pos_err=" << left_pos_error
        << " left_rot_err=" << left_rot_error
        << " right_pos_err=" << right_pos_error
        << " right_rot_err=" << right_rot_error
        << " pos_eps=" << pos_eps
        << " rot_eps=" << rot_eps
        << " initial_error_ratio=" << initial_error_ratio
        << " best_error_ratio=" << best_error_ratio
        << " final_error_ratio=" << final_error_ratio
        << " left_min_singular=" << std::scientific << std::setprecision(3)
        << left_min_singular
        << " right_min_singular=" << right_min_singular
        << " left_joint_limit=" << (left_joint_limit ? "true" : "false")
        << " right_joint_limit=" << (right_joint_limit ? "true" : "false")
        << " convergence_hint=" << convergence_hint
        << " seed_q=" << seed_q_summary
        << " final_q=" << formatSeedJointSummary_(q);

    if (failure_reason) {
      *failure_reason = oss.str();
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
    }
    return false;
  }

  /* === 订阅当前关节状态 === */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      const auto it = joint_index_map_.find(msg->name[i]);
      if (it != joint_index_map_.end()) {
        q_current_[it->second] = msg->position[i];
        if (required_planning_joint_name_set_.count(msg->name[i]) > 0) {
          received_planning_joint_names_.insert(msg->name[i]);
        }
      }
    }
    has_joint_state_ = true;
    publishObservedPoseTopics_();
  }

  void publishExecutionStatus(uint8_t state, const std::string& message)
  {
    last_execution_state_ = state;

    robot_control_msg::msg::CartesianExecutionStatus status_msg;
    status_msg.stamp = this->now();
    status_msg.state = state;
    status_msg.planned_points = static_cast<uint32_t>(last_planned_points_);
    status_msg.planned_duration_sec = last_planned_duration_sec_;
    status_msg.message = message;
    execution_status_pub_->publish(status_msg);
  }

  /* === 单段命令入口 === */
  void singleCmdCallback(const robot_control_msg::msg::Robotarmmovel::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    clearPlanningFailureMessage_();
    last_planned_points_ = 0;
    last_planned_duration_sec_ = 0.0;

    if (!isPlanningStateReady_()) {
      const std::string failure_message =
        "Cannot plan single MoveL before planning joints are ready";
      setPlanningFailureMessage_(failure_message);
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        failure_message);
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot plan single MoveL before planning joints are ready. received=%zu/%zu missing_joint_states=[%s]",
        received_planning_joint_names_.size(),
        required_planning_joint_names_.size(),
        missingPlanningJointSummary_().c_str());
      return;
    }

    if (planning_in_progress_ || trajectory_active_) {
      const std::string failure_message = "Unified planner is busy, ignoring single MoveL command";
      setPlanningFailureMessage_(failure_message);
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY,
        failure_message);
      RCLCPP_WARN(this->get_logger(), "%s", failure_message.c_str());
      return;
    }

    std::string error;
    if (!validateSingleCommand(*msg, error)) {
      const std::string failure_message = "Rejected single MoveL command: " + error;
      setPlanningFailureMessage_(failure_message);
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        failure_message);
      RCLCPP_WARN(this->get_logger(), "%s", failure_message.c_str());
      return;
    }

    publishExecutionStatus(
      robot_control_msg::msg::CartesianExecutionStatus::PLANNING,
      "Planning single MoveL trajectory");

    planning_in_progress_ = true;
    const bool ok = planSingleTrajectory(*msg);
    planning_in_progress_ = false;

    if (!ok || joint_traj_.empty()) {
      joint_traj_.clear();
      traj_index_ = 0;
      trajectory_active_ = false;
      active_command_source_ = ActiveCommandSource::None;
      publishPlannedTrajectoryMarkers_();
      const std::string failure_message = planningFailureMessageOr_("Single MoveL planning failed");
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        failure_message);
      RCLCPP_WARN(this->get_logger(), "Single MoveL planning failed: %s", failure_message.c_str());
      return;
    }

    last_planned_points_ = joint_traj_.size();
    last_planned_duration_sec_ = static_cast<double>(joint_traj_.size()) * offline_dt_;
    traj_index_ = 0;
    trajectory_active_ = true;
    active_command_source_ = ActiveCommandSource::Single;
    publishPlannedTrajectoryMarkers_();
    publishExecutionStatus(
      robot_control_msg::msg::CartesianExecutionStatus::EXECUTING,
      "Streaming planned single MoveL trajectory");
    RCLCPP_INFO(
      this->get_logger(), "Starting unified single MoveL trajectory with %zu points at %.2f ms interval",
      joint_traj_.size(), offline_dt_ * 1000.0);
  }

  /* === 路径命令入口 === */
  void pathCmdCallback(const robot_control_msg::msg::Robotarmmovelpath::SharedPtr msg)
  {
    clearPlanningFailureMessage_();
    last_planned_points_ = 0;
    last_planned_duration_sec_ = 0.0;
    if (!isPlanningStateReady_()) {
      clearVisualizationMarkers_();
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        "Cannot plan path before planning joints are ready");
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot plan path before planning joints are ready. received=%zu/%zu missing_joint_states=[%s]",
        received_planning_joint_names_.size(),
        required_planning_joint_names_.size(),
        missingPlanningJointSummary_().c_str());
      return;
    }

    // 当前设计不支持排队和抢占：忙时直接拒绝新命令。
    if (planning_in_progress_ || trajectory_active_) {
      RCLCPP_WARN(this->get_logger(), "Path planner is busy, rejecting new command");
      last_planned_points_ = 0;
      last_planned_duration_sec_ = 0.0;
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::REJECTED_BUSY,
        "Path planner is busy");
      return;
    }

    std::string error;
    if (!validateCommand(*msg, error)) {
      RCLCPP_WARN(this->get_logger(), "Rejected path command: %s", error.c_str());
      last_planned_points_ = 0;
      last_planned_duration_sec_ = 0.0;
      clearVisualizationMarkers_();
      publishCurrentPoseMarkers_();
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        "Rejected path command: " + error);
      return;
    }

    NormalizedPathRequest request;
    if (!buildNormalizedPathRequest_(*msg, request, error)) {
      const Eigen::VectorXd q_plan_start = makePlanningState();
      const CartesianPose current_left_tcp = getCurrentTCPPose(q_plan_start, ee_id_left_, tcp_transform_left_);
      const CartesianPose current_right_tcp = getCurrentTCPPose(q_plan_start, ee_id_right_, tcp_transform_right_);
      logPlanningFailure_(
        "normalize_path_request",
        error,
        request,
        current_left_tcp,
        current_right_tcp,
        current_left_tcp,
        current_right_tcp);
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        "Failed to normalize path command: " + error);
      return;
    }

    last_planned_points_ = 0;
    last_planned_duration_sec_ = 0.0;
    publishExecutionStatus(
      robot_control_msg::msg::CartesianExecutionStatus::PLANNING,
      "Planning path trajectory");

    planning_in_progress_ = true;
    const bool ok = planTrajectory(request);
    planning_in_progress_ = false;

    // 只有整条轨迹全部规划成功后，才允许进入发布阶段。
    if (!ok || joint_traj_.empty()) {
      joint_traj_.clear();
      traj_index_ = 0;
      trajectory_active_ = false;
      active_command_source_ = ActiveCommandSource::None;
      const std::string failure_message = planningFailureMessageOr_("Path trajectory planning failed");
      RCLCPP_WARN(this->get_logger(), "Path trajectory planning failed: %s", failure_message.c_str());
      last_planned_points_ = 0;
      last_planned_duration_sec_ = 0.0;
      publishPlannedTrajectoryMarkers_();
      publishExecutionStatus(
        robot_control_msg::msg::CartesianExecutionStatus::FAILED,
        failure_message);
      return;
    }

    last_planned_points_ = joint_traj_.size();
    last_planned_duration_sec_ = static_cast<double>(joint_traj_.size()) * offline_dt_;
    traj_index_ = 0;
    trajectory_active_ = true;
    active_command_source_ = ActiveCommandSource::Path;
    publishPlannedTrajectoryMarkers_();
    publishExecutionStatus(
      robot_control_msg::msg::CartesianExecutionStatus::EXECUTING,
      "Streaming planned path trajectory");
    RCLCPP_INFO(
      this->get_logger(), "Starting path trajectory publishing with %zu points at %.2f ms interval",
      joint_traj_.size(), offline_dt_ * 1000.0);
  }

  /* === 根据参数更新 TCP 变换 === */
  void updateTCPTransforms()
  {
    // 参数表示的是“末端执行器 EE 坐标系到 TCP 坐标系”的固定位姿偏移。
    const double tcp_left_x = this->get_parameter("tcp_left_x").as_double();
    const double tcp_left_y = this->get_parameter("tcp_left_y").as_double();
    const double tcp_left_z = this->get_parameter("tcp_left_z").as_double();
    const double tcp_left_roll = this->get_parameter("tcp_left_roll").as_double();
    const double tcp_left_pitch = this->get_parameter("tcp_left_pitch").as_double();
    const double tcp_left_yaw = this->get_parameter("tcp_left_yaw").as_double();

    const double tcp_right_x = this->get_parameter("tcp_right_x").as_double();
    const double tcp_right_y = this->get_parameter("tcp_right_y").as_double();
    const double tcp_right_z = this->get_parameter("tcp_right_z").as_double();
    const double tcp_right_roll = this->get_parameter("tcp_right_roll").as_double();
    const double tcp_right_pitch = this->get_parameter("tcp_right_pitch").as_double();
    const double tcp_right_yaw = this->get_parameter("tcp_right_yaw").as_double();

    // 左右臂各自维护独立的 TCP 变换矩阵。
    tcp_transform_left_.rotation() = rpyToRot(tcp_left_roll, tcp_left_pitch, tcp_left_yaw);
    tcp_transform_left_.translation() = Eigen::Vector3d(tcp_left_x, tcp_left_y, tcp_left_z);

    tcp_transform_right_.rotation() = rpyToRot(tcp_right_roll, tcp_right_pitch, tcp_right_yaw);
    tcp_transform_right_.translation() = Eigen::Vector3d(tcp_right_x, tcp_right_y, tcp_right_z);
  }

  /* === 定时发布离线关节轨迹 === */
  void publishNext()
  {
    if (!trajectory_active_ || traj_index_ >= joint_traj_.size()) {
      return;
    }

    // 每次从缓存中取一个采样点，转换成底层控制器使用的 Robotarmservomsg。
    const Eigen::VectorXd& q = joint_traj_[traj_index_++];

    robot_control_msg::msg::Robotarmservomsg msg;
    msg.ljoint1_position = q[left_arm_indices_[0]];
    msg.ljoint2_position = q[left_arm_indices_[1]];
    msg.ljoint3_position = q[left_arm_indices_[2]];
    msg.ljoint4_position = q[left_arm_indices_[3]];
    msg.ljoint5_position = q[left_arm_indices_[4]];
    msg.ljoint6_position = q[left_arm_indices_[5]];
    msg.ljoint7_position = q[left_arm_indices_[6]];
    msg.rjoint1_position = q[right_arm_indices_[0]];
    msg.rjoint2_position = q[right_arm_indices_[1]];
    msg.rjoint3_position = q[right_arm_indices_[2]];
    msg.rjoint4_position = q[right_arm_indices_[3]];
    msg.rjoint5_position = q[right_arm_indices_[4]];
    msg.rjoint6_position = q[right_arm_indices_[5]];
    msg.rjoint7_position = q[right_arm_indices_[6]];
    msg.run_mode = 2;
    msg.robot_power = true;
    msg.run_time = offline_dt_;
    msg.stamp = this->now();
    joint_pub_->publish(msg);
    publishPoseMarkersForState_(q);

    // 所有点发布完成后，清空缓存并回到空闲状态。
    if (traj_index_ >= joint_traj_.size()) {
      const ActiveCommandSource completed_source = active_command_source_;
      trajectory_active_ = false;
      joint_traj_.clear();
      traj_index_ = 0;
      active_command_source_ = ActiveCommandSource::None;
      RCLCPP_INFO(this->get_logger(), "Path trajectory published completely");
      if (completed_source == ActiveCommandSource::Path) {
        publishExecutionStatus(
          robot_control_msg::msg::CartesianExecutionStatus::STREAM_FINISHED,
          "Path trajectory published completely");
      } else if (completed_source == ActiveCommandSource::Single) {
        publishExecutionStatus(
          robot_control_msg::msg::CartesianExecutionStatus::STREAM_FINISHED,
          "Single MoveL trajectory published completely");
      }
    }
  }

  /* === 模型与索引 === */
  std::string urdf_path_;
  std::string ee_frame_left_;
  std::string ee_frame_right_;
  std::string joint_state_topic_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  size_t ee_id_left_ {0};
  size_t ee_id_right_ {0};

  std::unordered_map<std::string, int> joint_index_map_;
  std::vector<int> arm_joint_indices_;
  std::vector<int> left_arm_indices_;
  std::vector<int> right_arm_indices_;
  std::vector<std::string> required_planning_joint_names_;
  std::unordered_set<std::string> required_planning_joint_name_set_;
  std::unordered_set<std::string> received_planning_joint_names_;

  /* === 当前关节状态 === */
  Eigen::VectorXd q_current_;

  /* === TCP 与 Ruckig 配置 === */
  double offline_dt_ {0.002};
  double auto_blend_tangent_ratio_ {0.25};
  pinocchio::SE3 tcp_transform_left_;
  pinocchio::SE3 tcp_transform_right_;

  std::unique_ptr<ruckig::Ruckig<1>> otg_left_length_;
  std::unique_ptr<ruckig::Ruckig<1>> otg_right_length_;
  ruckig::InputParameter<1> length_input_template_;

  /* === 轨迹缓存与执行状态 === */
  std::vector<Eigen::VectorXd> joint_traj_;
  size_t traj_index_ {0};
  bool planning_in_progress_ {false};
  bool trajectory_active_ {false};
  ActiveCommandSource active_command_source_ {ActiveCommandSource::None};
  size_t last_planned_points_ {0};
  double last_planned_duration_sec_ {0.0};
  uint8_t last_execution_state_ {robot_control_msg::msg::CartesianExecutionStatus::IDLE};
  std::string last_planning_failure_message_;
  bool has_joint_state_ {false};

  /* === ROS 通信对象 === */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robot_control_msg::msg::Robotarmmovel>::SharedPtr single_cmd_sub_;
  rclcpp::Subscription<robot_control_msg::msg::Robotarmmovelpath>::SharedPtr path_cmd_sub_;
  rclcpp::Publisher<robot_control_msg::msg::Robotarmservomsg>::SharedPtr joint_pub_;
  rclcpp::Publisher<robot_control_msg::msg::EndEffectorPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<robot_control_msg::msg::EndEffectorPose>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr single_visualization_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_visualization_pub_;
  rclcpp::Publisher<robot_control_msg::msg::CartesianExecutionStatus>::SharedPtr execution_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr tcp_pose_timer_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> tcp_param_callbacks_;
  double tcp_pose_publish_rate_hz_ {50.0};
  std::string marker_frame_id_ {"car_link"};
  int trajectory_marker_stride_ {1};
  double trajectory_line_width_ {0.004};
  double pose_axis_length_ {0.08};
  double pose_axis_shaft_diameter_ {0.006};
  double pose_axis_head_diameter_ {0.012};
};

}  // namespace teleop_control

/* === 节点主入口 === */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop_control::ArmIKMoveLPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

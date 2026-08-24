
#include "erobot_controller/erobot_controller_arm.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <stddef.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "erobot_controller/CommandProcessor_arm.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
//erobot_controller::erobotControl
using config_type = controller_interface::interface_configuration_type;

namespace erobot_controller_arm
{

namespace {
std::string defaultArmUrdfPath()
{
  return ament_index_cpp::get_package_share_directory("robot_arm_description") + "/urdf/right_left_arm.urdf";
}

std::vector<std::string> dedupeJointNames(const std::vector<std::string>& joint_names)
{
  std::vector<std::string> unique_joint_names;
  unique_joint_names.reserve(joint_names.size());

  std::unordered_set<std::string> seen_joint_names;
  for (const auto& joint_name : joint_names) {
    if (joint_name.empty()) {
      continue;
    }

    if (seen_joint_names.insert(joint_name).second) {
      unique_joint_names.push_back(joint_name);
    }
  }

  return unique_joint_names;
}
}  // namespace

RobotController_arm::RobotController_arm() : controller_interface::ControllerInterface() {}

uint64_t RobotController_arm::submitPowerRequest(bool enable)
{
  const uint64_t generation = power_request_generation_.fetch_add(1) + 1;
  // Publish the generation before the request so update() can associate the
  // consumed request with this service call.
  pending_power_request_.store(enable ? 1 : 0);
  return generation;
}

bool RobotController_arm::waitForPowerFeedback(
  uint64_t generation,
  bool enable,
  std::chrono::milliseconds timeout,
  std::string &description)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(power_feedback_mutex_);

  auto state_matches = [this, enable]() {
    const bool all_enabled = std::all_of(
      power_enabled_states_.begin(), power_enabled_states_.end(), [](bool value) { return value; });
    if (enable) {
      return power_command_enabled_ && power_all_enabled_ && all_enabled;
    }
    return !power_command_enabled_ && !power_all_enabled_ && !all_enabled;
  };

  while (rclcpp::ok()) {
    if (power_applied_generation_.load() >= generation) {
      const bool any_available = std::any_of(
        power_status_codes_.begin(), power_status_codes_.end(), [](int code) { return code != 0; });
      if (enable && !any_available) {
        description = "no EtherCAT drives detected; enabled=0/14";
        return false;
      }
      if (state_matches()) {
        description = enable ? "enabled=14/14" : "enabled=0/14";
        return true;
      }
    }

    if (power_feedback_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      break;
    }
  }

  std::size_t enabled_count = std::count(
    power_enabled_states_.begin(), power_enabled_states_.end(), true);
  std::ostringstream status;
  status << "enabled=" << enabled_count << "/14, command_enabled="
         << (power_command_enabled_ ? "true" : "false")
         << ", all_enabled=" << (power_all_enabled_ ? "true" : "false")
         << ", status_codes=[";
  for (std::size_t i = 0; i < power_status_codes_.size(); ++i) {
    if (i != 0) {
      status << ",";
    }
    status << power_status_codes_[i];
  }
  status << "]";
  description = status.str();
  return false;
}

void RobotController_arm::updatePowerFeedbackSnapshot()
{
  std::lock_guard<std::mutex> lock(power_feedback_mutex_);
  for (std::size_t i = 0; i < power_status_codes_.size(); ++i) {
    const int status = (i < joint_mode_state_interface_.size()) ?
      static_cast<int>(std::llround(joint_mode_state_interface_[i].get().get_value())) : 0;
    power_status_codes_[i] = status;
    power_enabled_states_[i] = (status == MOTOR_STATUS_ENABLED);
  }
  power_command_enabled_ = command_processor_->isPowerEnabledRequested() &&
    command_processor_->isHardwareAvailable();
  power_all_enabled_ = command_processor_->areAllMotorsEnabled();
}

void RobotController_arm::handlePowerService(
  const std::shared_ptr<robot_control_msg::srv::SetRobotPower::Request> request,
  std::shared_ptr<robot_control_msg::srv::SetRobotPower::Response> response)
{
  std::lock_guard<std::mutex> service_lock(power_service_mutex_);
  if (!request || !response) {
    return;
  }

  if (!controller_active_) {
    response->success = false;
    response->message = "power request rejected: controller is not active";
    return;
  }

  const bool enable = request->enable;
  const uint64_t generation = submitPowerRequest(enable);
  std::string feedback;
  if (waitForPowerFeedback(generation, enable, std::chrono::milliseconds(5000), feedback)) {
    response->success = true;
    response->message = enable ?
      "Robot power enabled through controller: enabled=14/14" :
      "Robot power disabled through controller: enabled=0/14";
    return;
  }

  response->success = false;
  response->message = std::string("Robot power ") + (enable ? "enable" : "disable") +
    " failed: " + feedback;

  if (enable) {
    const uint64_t rollback_generation = submitPowerRequest(false);
    std::string rollback_feedback;
    if (waitForPowerFeedback(
        rollback_generation, false, std::chrono::milliseconds(1500), rollback_feedback)) {
      response->message += "; automatic disable rollback confirmed";
    } else {
      response->message += "; automatic disable rollback confirmation failed: " + rollback_feedback;
    }
  }
}

controller_interface::CallbackReturn RobotController_arm::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  try {
    gravity_compensation_joint_names_ = dedupeJointNames(
      auto_declare<std::vector<std::string>>(
        "gravity_compensation_joints",
        std::vector<std::string>{}));
  } catch (const rclcpp::exceptions::InvalidParameterValueException& exception) {
    // ROS 2 may parse an untyped empty YAML list (`[]`) as NOT_SET for array params.
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter 'gravity_compensation_joints' is unset or has an invalid empty-list override. "
      "Falling back to an empty list: %s",
      exception.what());
    if (!get_node()->has_parameter("gravity_compensation_joints")) {
      get_node()->declare_parameter<std::vector<std::string>>(
        "gravity_compensation_joints",
        std::vector<std::string>{});
    }
    gravity_compensation_joint_names_.clear();
  }
  
  // 声明与CommandProcessor相关的参数（在on_configure读取真实值并传入）
  auto_declare<std::string>("urdf_filename", defaultArmUrdfPath());
  auto_declare<std::string>("ee_links.left", "lee_link");
  auto_declare<std::string>("ee_links.right", "ree_link");
  auto_declare<double>("workpiece_offset.x", 0.0);
  auto_declare<double>("workpiece_offset.y", 0.0);
  auto_declare<double>("workpiece_offset.z", 0.0);
  const CommandProcessor_arm::JointLimitGuardConfig joint_limit_guard_defaults{};
  auto_declare<bool>("joint_limit_guard.enabled", joint_limit_guard_defaults.enabled);
  auto_declare<double>("joint_limit_guard.soft_zone_deg", joint_limit_guard_defaults.soft_zone_deg);
  auto_declare<double>(
      "joint_limit_guard.stiffness_nm_per_rad",
      joint_limit_guard_defaults.stiffness_nm_per_rad);
  auto_declare<double>(
      "joint_limit_guard.damping_nm_s_per_rad",
      joint_limit_guard_defaults.damping_nm_s_per_rad);
  auto_declare<double>(
      "joint_limit_guard.overrun_stiffness_scale",
      joint_limit_guard_defaults.overrun_stiffness_scale);
  auto_declare<double>(
      "joint_limit_guard.overrun_damping_scale",
      joint_limit_guard_defaults.overrun_damping_scale);
  auto_declare<double>("joint_limit_guard.max_torque_ratio", joint_limit_guard_defaults.max_torque_ratio);
  auto_declare<double>("heavy_v1.max_velocity_rad_s", 0.3);
  auto_declare<double>("heavy_v1.max_acceleration_rad_s2", 1.0);
  auto_declare<bool>("heavy_v1.allow_disabled_simulation_execution", false);

  RCLCPP_INFO(get_node()->get_logger(), "Controller arm initialized joint_names size: %zu", joint_names_.size());
  RCLCPP_INFO(get_node()->get_logger(), "Controller arm initialized command_interface_types size: %zu", command_interface_types_.size());
  
  for (const auto& joint_name : joint_names_) {
    std::string param_prefix = "motor_coefficients." + joint_name + ".";
    auto_declare<double>(param_prefix + "kp", 0.0);
    auto_declare<double>(param_prefix + "kd", 0.0);
    auto_declare<double>(param_prefix + "max_effort", 0.0);
    auto_declare<double>(param_prefix + "Torqueconstant", 0.0);
  }
  // 从参数服务器获取电机系数
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController_arm::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController_arm::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(
    joint_names_.size() * state_interface_types_.size() + gravity_compensation_joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  for (const auto& joint_name : gravity_compensation_joint_names_) {
    if (std::find(joint_names_.begin(), joint_names_.end(), joint_name) != joint_names_.end()) {
      continue;
    }
    conf.names.push_back(joint_name + "/position");
  }

  return conf;
}

controller_interface::CallbackReturn RobotController_arm::on_configure(const rclcpp_lifecycle::State &)
{
    // 1) 读取并校验 CommandProcessor_leg 所需参数
    std::string urdf_filename;
    std::string ee_left, ee_right;
    double offset_x{0.0}, offset_y{0.0}, offset_z{0.0};
    CommandProcessor_arm::JointLimitGuardConfig joint_limit_guard_config{};

    get_node()->get_parameter_or(
        "gravity_compensation_joints",
        gravity_compensation_joint_names_,
        gravity_compensation_joint_names_);
    gravity_compensation_joint_names_ = dedupeJointNames(gravity_compensation_joint_names_);
    get_node()->get_parameter("urdf_filename", urdf_filename);
    get_node()->get_parameter("ee_links.left", ee_left);
    get_node()->get_parameter("ee_links.right", ee_right);
    get_node()->get_parameter_or("workpiece_offset.x", offset_x, 0.0);
    get_node()->get_parameter_or("workpiece_offset.y", offset_y, 0.0);
    get_node()->get_parameter_or("workpiece_offset.z", offset_z, 0.0);
    get_node()->get_parameter_or("joint_limit_guard.enabled", joint_limit_guard_config.enabled, false);
    get_node()->get_parameter_or("joint_limit_guard.soft_zone_deg", joint_limit_guard_config.soft_zone_deg, 6.0);
    get_node()->get_parameter_or(
        "joint_limit_guard.stiffness_nm_per_rad",
        joint_limit_guard_config.stiffness_nm_per_rad,
        120.0);
    get_node()->get_parameter_or(
        "joint_limit_guard.damping_nm_s_per_rad",
        joint_limit_guard_config.damping_nm_s_per_rad,
        6.0);
    get_node()->get_parameter_or(
        "joint_limit_guard.overrun_stiffness_scale",
        joint_limit_guard_config.overrun_stiffness_scale,
        3.0);
    get_node()->get_parameter_or(
        "joint_limit_guard.overrun_damping_scale",
        joint_limit_guard_config.overrun_damping_scale,
        2.0);
    get_node()->get_parameter_or(
        "joint_limit_guard.max_torque_ratio",
        joint_limit_guard_config.max_torque_ratio,
        0.35);
    get_node()->get_parameter_or("heavy_v1.max_velocity_rad_s", heavy_max_velocity_, 0.3);
    get_node()->get_parameter_or("heavy_v1.max_acceleration_rad_s2", heavy_max_acceleration_, 1.0);
    get_node()->get_parameter_or(
        "heavy_v1.allow_disabled_simulation_execution",
        allow_disabled_simulation_execution_, false);

    if (!std::isfinite(heavy_max_velocity_) || heavy_max_velocity_ <= 0.0 ||
        !std::isfinite(heavy_max_acceleration_) || heavy_max_acceleration_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid Heavy V1 arm motion limits");
        return CallbackReturn::ERROR;
    }

    if (urdf_filename.empty() || ee_left.empty() || ee_right.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CommandProcessor_leg parameters missing: urdf='%s', left='%s', right='%s'",
                     urdf_filename.c_str(), ee_left.c_str(), ee_right.c_str());
        return CallbackReturn::ERROR;
    }

    // 2) 使用外部参数构造 CommandProcessor（避免其内部重复声明/读取参数）
    command_processor_ = std::make_unique<CommandProcessor_arm>(
        joint_names_.size(),
        joint_names_,
        gravity_compensation_joint_names_,
        joint_limit_guard_config,
        urdf_filename,
        ee_left,
        ee_right,
        offset_x,
        offset_y,
        offset_z);
    active_control_mode_cache_.store(static_cast<int>(command_processor_->getActiveControlMode()));

    auto twist_callback = [this](const std::shared_ptr<robot_control_msg::msg::Robotarmservomsg> msg) -> void
    {
        if (!msg) {
            RCLCPP_WARN(get_node()->get_logger(), "Received empty message");
            return;
        }
        if (heavy_lease_active_.load(std::memory_order_acquire)) {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "Ignored legacy /arm_axis_position_cmd while Heavy V1 lease is active");
            return;
        }
        cmd_buffer_.writeFromNonRT(msg);
        new_msg_ = true;
    };

    cmd_subscriber_ = get_node()->create_subscription<robot_control_msg::msg::Robotarmservomsg>(
        "/arm_axis_position_cmd", rclcpp::SystemDefaultsQoS(), twist_callback);

    // 添加机械臂关节绝对命令订阅者
    auto arm_joint_callback = [this](const std::shared_ptr<robot_control_msg::msg::Robotarmjoint> msg) -> void
    {
        if (!msg) {
            RCLCPP_WARN(get_node()->get_logger(), "Received empty arm joint message");
            new_arm_joint_msg_ = false;
            return;
        }
        if (heavy_lease_active_.load(std::memory_order_acquire)) {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "Ignored legacy /arm_joint_absolute_cmd while Heavy V1 lease is active");
            return;
        }
        arm_joint_buffer_.writeFromNonRT(msg);
        new_arm_joint_msg_ = true;
    };
    
    arm_joint_subscriber_ = get_node()->create_subscription<robot_control_msg::msg::Robotarmjoint>(
        "/arm_joint_absolute_cmd", rclcpp::SystemDefaultsQoS(), arm_joint_callback);

    rclcpp::QoS heavy_command_qos(rclcpp::KeepLast(1));
    heavy_command_qos.reliable().durability_volatile();
    heavy_command_subscriber_ = get_node()->create_subscription<
        robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1>(
        "/ubuntu_lower_gateway/internal/heavy/v1/accepted_command", heavy_command_qos,
        [this](
            const robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::SharedPtr message) {
            if (!message) {
                return;
            }
            HeavyArmSetpoint setpoint;
            setpoint.sequence = message->sequence;
            setpoint.mode = message->mode;
            setpoint.field_mask = message->field_mask;
            for (std::size_t i = 0; i < 14; ++i) {
                setpoint.position[i] = message->position[i + 1];
                setpoint.velocity[i] = message->velocity[i + 1];
                setpoint.acceleration[i] = message->acceleration[i + 1];
            }
            heavy_command_buffer_.writeFromNonRT(setpoint);
            heavy_command_generation_.fetch_add(1, std::memory_order_release);
        });
    rclcpp::QoS heavy_gate_qos(rclcpp::KeepLast(1));
    heavy_gate_qos.reliable().transient_local();
    heavy_lease_gate_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "/ubuntu_lower_gateway/internal/heavy/v1/lease_active", heavy_gate_qos,
        [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (message) {
                heavy_lease_active_.store(message->data, std::memory_order_release);
            }
        });
    heavy_ack_publisher_ = get_node()->create_publisher<std_msgs::msg::UInt64>(
        "/ubuntu_lower_gateway/internal/heavy/v1/arm_applied_sequence",
        rclcpp::QoS(10).reliable());

    auto poweron_callback = [this](const std::shared_ptr<std_msgs::msg::Bool> msg) -> void
    {
        if (!msg) {
            RCLCPP_WARN(get_node()->get_logger(), "Received empty power command");
            return;
        }
        pending_power_request_.store(msg->data ? 1 : 0);
    };

    poweron_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "/robot_poweron", rclcpp::SystemDefaultsQoS(), poweron_callback);

    auto hardware_mode_callback = [this](const std::shared_ptr<std_msgs::msg::Int32> msg) -> void
    {
        if (!msg) {
            RCLCPP_WARN(get_node()->get_logger(), "Received empty hardware mode command");
            return;
        }

        if (msg->data == 0) {
            pending_hardware_mode_request_.store(static_cast<int>(ControlMode::POSITION));
            return;
        }

        if (msg->data == 1) {
            pending_hardware_mode_request_.store(static_cast<int>(ControlMode::EFFORT));
            return;
        }

        RCLCPP_WARN(
            get_node()->get_logger(),
            "Unsupported arm hardware mode '%d', expected 0(CSP) or 1(CST)",
            msg->data);
    };

    hardware_mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        "/arm_hardware_mode", rclcpp::SystemDefaultsQoS(), hardware_mode_callback);

    auto control_mode_callback =
        [this](
            const std::shared_ptr<robot_control_msg::srv::SetArmControlMode::Request> request,
            std::shared_ptr<robot_control_msg::srv::SetArmControlMode::Response> response) -> void
    {
        if (!request || !response) {
            return;
        }

        const int requested_mode = static_cast<int>(request->mode);
        if (requested_mode != static_cast<int>(ControlMode::POSITION) &&
            requested_mode != static_cast<int>(ControlMode::EFFORT)) {
            response->success = false;
            response->active_mode = static_cast<uint8_t>(active_control_mode_cache_.load());
            response->message = "invalid control mode request";
            return;
        }

        pending_control_mode_request_.store(requested_mode);
        response->success = true;
        response->active_mode = static_cast<uint8_t>(active_control_mode_cache_.load());
        response->message = "control mode request accepted";
    };

  control_mode_service_ = get_node()->create_service<robot_control_msg::srv::SetArmControlMode>(
    "set_control_mode", control_mode_callback);

  power_service_ = get_node()->create_service<robot_control_msg::srv::SetRobotPower>(
    "/set_robot_power",
    std::bind(
      &RobotController_arm::handlePowerService, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Robot power service ready on /set_robot_power; requests use controller power path");

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController_arm::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart

  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();
  joint_motor_encoder_0_state_interface_.clear();
  joint_motor_encoder_1_state_interface_.clear();
  joint_mode_state_interface_.clear();
  joint_power_state_interface_.clear();
  joint_compensation_position_state_interface_.clear();

  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_power_command_interface_.clear();
  joint_mode_command_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  std::unordered_set<std::string> arm_joint_name_set(joint_names_.begin(), joint_names_.end());
  std::unordered_set<std::string> compensation_joint_name_set(
      gravity_compensation_joint_names_.begin(), gravity_compensation_joint_names_.end());

  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> position_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> velocity_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> effort_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> motor_encoder_0_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> motor_encoder_1_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> status_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> error_code_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> power_state_by_joint;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> compensation_position_state_by_joint;

  for (auto & interface : state_interfaces_) {
    const std::string& joint_name = interface.get_prefix_name();
    const std::string& interface_name = interface.get_interface_name();

    if (arm_joint_name_set.count(joint_name) > 0) {
      if (interface_name == "position") {
        position_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "velocity") {
        velocity_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "effort") {
        effort_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "motor_encoder_0") {
        motor_encoder_0_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "motor_encoder_1") {
        motor_encoder_1_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "status") {
        status_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "error_code") {
        error_code_state_by_joint[joint_name] = &interface;
      } else if (interface_name == "power_enable") {
        power_state_by_joint[joint_name] = &interface;
      }
      continue;
    }

    if (compensation_joint_name_set.count(joint_name) > 0 && interface_name == "position") {
      compensation_position_state_by_joint[joint_name] = &interface;
    }
  }

  auto bind_required_interfaces =
      [this](
          const std::vector<std::string>& ordered_joint_names,
          const std::unordered_map<std::string, hardware_interface::LoanedStateInterface*>& interface_by_joint,
          std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& target,
          const char* interface_label) -> bool
  {
    for (const auto& joint_name : ordered_joint_names) {
      auto it = interface_by_joint.find(joint_name);
      if (it == interface_by_joint.end()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Missing required %s state interface for joint '%s'",
            interface_label,
            joint_name.c_str());
        return false;
      }
      target.push_back(*it->second);
    }
    return true;
  };

  if (!bind_required_interfaces(joint_names_, position_state_by_joint, joint_position_state_interface_, "position") ||
      !bind_required_interfaces(joint_names_, velocity_state_by_joint, joint_velocity_state_interface_, "velocity") ||
      !bind_required_interfaces(joint_names_, status_state_by_joint, joint_mode_state_interface_, "status") ||
      !bind_required_interfaces(
          joint_names_, error_code_state_by_joint, joint_error_code_state_interface_, "error_code")) {
    return CallbackReturn::ERROR;
  }

  if (!bind_required_interfaces(
          gravity_compensation_joint_names_,
          compensation_position_state_by_joint,
          joint_compensation_position_state_interface_,
          "gravity compensation position")) {
    return CallbackReturn::ERROR;
  }

  if (effort_state_by_joint.size() >= joint_names_.size()) {
    bind_required_interfaces(joint_names_, effort_state_by_joint, joint_effort_state_interface_, "effort");
  }
  if (motor_encoder_0_state_by_joint.size() >= joint_names_.size()) {
    bind_required_interfaces(
        joint_names_, motor_encoder_0_state_by_joint, joint_motor_encoder_0_state_interface_, "motor_encoder_0");
  }
  if (motor_encoder_1_state_by_joint.size() >= joint_names_.size()) {
    bind_required_interfaces(
        joint_names_, motor_encoder_1_state_by_joint, joint_motor_encoder_1_state_interface_, "motor_encoder_1");
  }
  if (power_state_by_joint.size() >= joint_names_.size()) {
    bind_required_interfaces(joint_names_, power_state_by_joint, joint_power_state_interface_, "power_enable");
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint_name = joint_names_[i];
    MotorCoefficients coeffs;
    std::string param_prefix = "motor_coefficients." + joint_name + ".";

    // 读取各个参数，如果参数不存在则使用默认值
    get_node()->get_parameter_or(param_prefix + "kp", coeffs.kp, 0.0);
    get_node()->get_parameter_or(param_prefix + "kd", coeffs.kd, 0.0);
    get_node()->get_parameter_or(param_prefix + "max_effort", coeffs.max_effort, 0.0);
    get_node()->get_parameter_or(param_prefix + "Torqueconstant", coeffs.Torqueconstant, 0.0);
    
    // 将读取的系数存入向量
    if (i < command_processor_->motor_coeffs_.size()) {
        command_processor_->motor_coeffs_[i] = coeffs;
    }
    
    // 打印读取的参数值（可选，用于调试）
    RCLCPP_INFO(get_node()->get_logger(), 
                "关节 %s 参数: kp=%.2f, kd=%.2f, max_effort=%.2f, Torqueconstant=%.6f",
                joint_name.c_str(), coeffs.kp, coeffs.kd,
                coeffs.max_effort, coeffs.Torqueconstant);
  }
  controller_active_ = true;
  return CallbackReturn::SUCCESS;
}


controller_interface::return_type RobotController_arm::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // RCLCPP_INFO(get_node()->get_logger(), "controller runing !!!!!!!!!!!!!!!!");
  command_processor_->updateJointState(
      joint_position_state_interface_,
      joint_velocity_state_interface_,
      joint_mode_state_interface_,
      joint_error_code_state_interface_,
      joint_compensation_position_state_interface_);

  const int pending_mode = pending_control_mode_request_.exchange(-1);
  if (pending_mode != -1) {
      std::string mode_message;
      const bool accepted = command_processor_->requestControlMode(
          static_cast<ControlMode>(pending_mode), &mode_message);
      if (!mode_message.empty()) {
          if (accepted) {
              RCLCPP_INFO(get_node()->get_logger(), "%s", mode_message.c_str());
          } else {
              RCLCPP_WARN(get_node()->get_logger(), "%s", mode_message.c_str());
          }
      }
  }

  const int pending_hardware_mode = pending_hardware_mode_request_.exchange(-1);
  if (pending_hardware_mode != -1) {
      std::string mode_message;
      const bool accepted = command_processor_->requestControlMode(
          static_cast<ControlMode>(pending_hardware_mode), &mode_message);
      if (!mode_message.empty()) {
          if (accepted) {
              RCLCPP_INFO(get_node()->get_logger(), "Topic mode switch: %s", mode_message.c_str());
          } else {
              RCLCPP_WARN(get_node()->get_logger(), "Topic mode switch rejected: %s", mode_message.c_str());
          }
      }
  }

  const int pending_power = pending_power_request_.exchange(-1);
  bool power_request_applied_this_cycle = false;
  uint64_t applied_power_generation = 0;
  if (pending_power != -1) {
      std::string power_message;
      const bool accepted = command_processor_->requestPowerEnable(pending_power != 0, &power_message);
      power_request_applied_this_cycle = true;
      applied_power_generation = power_request_generation_.load();
      if (!power_message.empty()) {
          if (accepted) {
              RCLCPP_INFO(get_node()->get_logger(), "%s", power_message.c_str());
          } else {
              RCLCPP_WARN(get_node()->get_logger(), "%s", power_message.c_str());
          }
      }
  }
  
  const bool heavy_lease_active = heavy_lease_active_.load(std::memory_order_acquire);
  if (!heavy_lease_active && heavy_lease_active_last_update_) {
    command_processor_->clearHeavyPositionStream();
    command_processor_->requestPositionHold("Heavy V1 lease released: hold current arm pose");
    // The internal generation belongs to the lower-gateway process. Reset
    // the controller watermark when its retained lease gate goes false so a
    // restarted gateway can begin again at generation 1.
    last_heavy_sequence_ = 0;
    consumed_heavy_command_generation_ = heavy_command_generation_.load(
      std::memory_order_acquire);
  }
  heavy_lease_active_last_update_ = heavy_lease_active;
  std::uint64_t heavy_ack_sequence = 0;
  if (heavy_lease_active) {
    // A command accepted immediately before the lease gate changed must not
    // leak into the Heavy-owned update path.
    new_arm_joint_msg_ = false;
    new_msg_ = false;
    const auto heavy_generation = heavy_command_generation_.load(std::memory_order_acquire);
    if (heavy_generation != consumed_heavy_command_generation_) {
      const auto heavy_setpoint = *heavy_command_buffer_.readFromRT();
      consumed_heavy_command_generation_ = heavy_generation;
      if (heavy_setpoint.sequence > last_heavy_sequence_) {
        bool applied = false;
        std::string error;
        if (heavy_setpoint.mode ==
          robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_FOLLOW_POSITION) {
          const bool position_provided = (heavy_setpoint.field_mask &
            robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_POSITION) != 0U;
          const bool velocity_provided = (heavy_setpoint.field_mask &
            robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_VELOCITY) != 0U;
          const bool acceleration_provided = (heavy_setpoint.field_mask &
            robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_ACCELERATION) != 0U;
          if (!position_provided) {
            error = "Heavy FOLLOW missing position field";
          } else {
            applied = command_processor_->processHeavyPositionCommand(
              heavy_setpoint.position, heavy_setpoint.velocity, heavy_setpoint.acceleration,
              velocity_provided, acceleration_provided, heavy_max_velocity_,
              heavy_max_acceleration_, &error);
          }
        } else if (heavy_setpoint.mode ==
          robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_HOLD) {
          if ((heavy_setpoint.field_mask &
            robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::FIELD_POSITION) == 0U) {
            error = "Heavy HOLD missing position field";
          } else {
            applied = command_processor_->processHeavyHoldPositionCommand(
              heavy_setpoint.position, &error);
          }
        } else if (heavy_setpoint.mode ==
          robot_control_msg::msg::HeavyUpperBodyGatewayCommandV1::MODE_STOP) {
          applied = command_processor_->requestPositionHold("Heavy V1 STOP: hold current arm pose");
        } else {
          error = "unknown Heavy V1 arm command mode";
        }
        if (!applied && !error.empty()) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Heavy V1 arm command rejected locally: %s", error.c_str());
        }
        if (applied) {
          last_heavy_sequence_ = heavy_setpoint.sequence;
          heavy_ack_sequence = heavy_setpoint.sequence;
        }
      }
    }
  }

  // 处理机械臂关节绝对命令消息
  if(!heavy_lease_active && new_arm_joint_msg_) {
      arm_joint_buffer_msg_ = *arm_joint_buffer_.readFromRT();
      bool should_replan = false;
      if ( arm_joint_buffer_msg_ != nullptr) {
          // 使用辅助类处理新命令
          should_replan = command_processor_->processNewArmJointCommand(*arm_joint_buffer_msg_);
          RCLCPP_INFO(get_node()->get_logger(), "Received arm joint command");
      }
    new_arm_joint_msg_ = false;
    if (should_replan) {
      command_processor_->command_arm_plan(time);
    }
  }
  
  // 处理原有命令消息
  if(!heavy_lease_active && new_msg_) {
      cmd_buffer_msg_ = *cmd_buffer_.readFromRT();
      bool should_replan = false;
      if ( cmd_buffer_msg_ != nullptr) {
          should_replan = command_processor_->processNewCommand(*cmd_buffer_msg_);
      }
    new_msg_ = false;
    if (should_replan) {
      rclcpp::Time current_time = rclcpp::Clock().now();
      command_processor_->command_arm_plan(current_time);
    }
  }

  // 使用辅助类更新关节命令
  command_processor_->updateJointCommands(
      joint_position_state_interface_,
      joint_position_command_interface_,
      joint_velocity_command_interface_,
      joint_effort_command_interface_,
      joint_power_command_interface_,
      joint_mode_command_interface_,
      allow_disabled_simulation_execution_ && heavy_lease_active);
  if (heavy_ack_sequence != 0 && heavy_ack_publisher_) {
    std_msgs::msg::UInt64 ack;
    ack.data = heavy_ack_sequence;
    heavy_ack_publisher_->publish(ack);
  }
  updatePowerFeedbackSnapshot();
  if (power_request_applied_this_cycle) {
    power_applied_generation_.store(applied_power_generation);
    std::ostringstream power_write;
    power_write << "POWER REQUEST WRITE: generation=" << applied_power_generation
                << " enable=" << (pending_power != 0 ? "true" : "false")
                << " controller=" << static_cast<const void *>(this)
                << " command_processor=" << static_cast<const void *>(command_processor_.get())
                << " power_enable=[";
    for (std::size_t i = 0; i < joint_power_command_interface_.size(); ++i) {
      if (i != 0) {
        power_write << ",";
      }
      power_write << joint_power_command_interface_[i].get().get_value();
    }
    power_write << "]";
    RCLCPP_INFO(get_node()->get_logger(), "%s", power_write.str().c_str());
  }
  power_feedback_cv_.notify_all();
  active_control_mode_cache_.store(static_cast<int>(command_processor_->getActiveControlMode()));
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController_arm::on_deactivate(const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  heavy_lease_active_.store(false, std::memory_order_release);
  consumed_heavy_command_generation_ = heavy_command_generation_.load(std::memory_order_acquire);
  power_feedback_cv_.notify_all();
  for (uint8_t i = 0; i < joint_power_command_interface_.size(); ++i)
  {
    joint_power_command_interface_[i].get().set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "所有电机已安全停止");
  release_interfaces();
  command_processor_->reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace erobot_controller
//
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  erobot_controller_arm::RobotController_arm, controller_interface::ControllerInterface)

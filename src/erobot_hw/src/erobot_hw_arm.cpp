#include "erobot_hw_arm.hpp"

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <memory>
using namespace boost::interprocess;

namespace Robot_arm_hardware_interface
{
namespace
{
constexpr const char * kIghDriverPidFile = "/var/run/igh_driver.pid";
constexpr const char * kIghDriverExecutable =
  "/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver";
constexpr auto kIghDriverCheckPeriod = std::chrono::milliseconds(100);
}

bool Robot_arm_Ethercat_Hardware::is_igh_driver_alive()
{
  const auto now = std::chrono::steady_clock::now();
  if (last_igh_driver_check_.time_since_epoch().count() != 0 &&
    now - last_igh_driver_check_ < kIghDriverCheckPeriod)
  {
    return igh_driver_alive_;
  }
  last_igh_driver_check_ = now;

  int observed_pid = -1;
  std::ifstream pid_file(kIghDriverPidFile);
  if (pid_file.is_open()) {
    long long parsed_pid = -1;
    pid_file >> parsed_pid;
    if (pid_file && parsed_pid > 1 && parsed_pid <= std::numeric_limits<int>::max()) {
      observed_pid = static_cast<int>(parsed_pid);
    }
  }

  bool observed_alive = false;
  if (observed_pid > 1) {
    errno = 0;
    const bool process_exists = (::kill(observed_pid, 0) == 0 || errno == EPERM);
    if (process_exists) {
      std::ifstream command_line("/proc/" + std::to_string(observed_pid) + "/cmdline");
      std::string command;
      if (command_line.is_open()) {
        std::getline(command_line, command, '\0');
        observed_alive = command.find(kIghDriverExecutable) != std::string::npos;
      }
    }
  }

  if (observed_alive != igh_driver_alive_ || observed_pid != igh_driver_pid_) {
    if (observed_alive) {
      RCLCPP_INFO(
        rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
        "IGH driver health gate is active (pid=%d)", observed_pid);
    } else if (igh_driver_alive_ || observed_pid > 1 || igh_driver_pid_ > 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
        "IGH driver health gate failed (pid_file=%d); arm drive feedback is unavailable",
        observed_pid);
    }
  }

  igh_driver_pid_ = observed_pid;
  igh_driver_alive_ = observed_alive;
  return igh_driver_alive_;
}

//初始化硬件接口
hardware_interface::CallbackReturn Robot_arm_Ethercat_Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  //检查初始化是否成功
  if (
    hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //shm init
  init_shm_desire();//初始化用于储存期望状态的共享内存
  init_shm_real();//初始化用于储存实际状态的共享内存
  //这两个方法内部会创建共享内存对象并映射到程序的内存地址空间

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  arm_class_ = stod(info_.hardware_parameters["arm_class"]);
  sim_ = stod(info_.hardware_parameters["sim"]);
  
  switch((int)arm_class_){
    case 1:
      bais_ = 0;
      break;
    case 2:
      bais_ = 7;
      break;
    case 3:
      bais_ = 0;
      break;
    default:
      break;
  }
  
  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size());
  joint_shared_slots_.resize(info_.joints.size());

  if (info_.joints.size() != kArmAxisCount) {
    RCLCPP_FATAL(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
      "Arm hardware requires exactly %zu joints, got %zu",
      kArmAxisCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::array<bool, kArmAxisCount> shared_slot_seen{};
  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    std::size_t shared_slot = 0;
    const auto & joint_name = info_.joints[joint_index].name;
    if (!shared_axis_slot_for_joint(joint_name, shared_slot)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
        "Unknown arm joint '%s'; expected ljoint1..7 or rjoint1..7",
        joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (shared_slot_seen[shared_slot]) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
        "Duplicate arm joint mapping for '%s' (shared-memory slot %zu)",
        joint_name.c_str(), shared_slot);
      return hardware_interface::CallbackReturn::ERROR;
    }
    shared_slot_seen[shared_slot] = true;
    joint_shared_slots_[joint_index] = shared_slot;
    RCLCPP_INFO(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
      "Mapped ros2_control joint '%s' (index %zu) to IGH slot %zu / alias 0x%04zx",
      joint_name.c_str(), joint_index, shared_slot, 0x1000U + shared_slot);
  }

  hw_states_uint.resize(info_.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  hw_commands_uint.resize(info_.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());

    // 定义预期的接口类型
  std::set<std::string> expected_command_interfaces = {
    "position", "velocity", "effort", "status", "mode", "power_enable"
  };
  
  std::set<std::string> expected_state_interfaces = {
    "position", "velocity", "effort", "motor_encoder_0", "motor_encoder_1", 
    "status", "error_code", "mode", "power_enable"
  };

  // 检查每个关节的接口配置
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];

    // === 命令接口检查 ===
    std::set<std::string> actual_command_interface_names;
    for (const auto& interface : joint.command_interfaces) {
      actual_command_interface_names.insert(interface.name);
    }
    
    // 检查缺少的接口
    for (const auto& expected : expected_command_interfaces) {
      if (actual_command_interface_names.find(expected) == actual_command_interface_names.end()) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
          "Joint '%s' is missing command interface: '%s'", 
          joint.name.c_str(), expected.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    // 初始化状态值
    for (auto& state : hw_states_) {
      state = {
        .position = 0,
        .velocity = 0,
        .effort = 0,
        .motor_encoder = {
          0,
          0
        },
        .status = 0,
        .error_code = 0,
        .mode = 0,
        .power_enable = 0
      };
    }
    // === 状态接口检查 ===
    std::set<std::string> actual_state_interface_names;
    for (const auto& interface : joint.state_interfaces) {
      actual_state_interface_names.insert(interface.name);
      
      // 处理初始值（仅position接口）
      if (interface.name == "position" && !interface.initial_value.empty()) {
        try {
           hw_states_[i].position = std::stod(interface.initial_value);
           
        } 
        catch (const std::exception& e) {
          RCLCPP_INFO(
            rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
            "Failed to parse initial position for joint '%s': %s", 
            joint.name.c_str(), e.what());
        }
      }
    }
    
    // 检查缺少的接口
    for (const auto& expected : expected_state_interfaces) {
      if (actual_state_interface_names.find(expected) == actual_state_interface_names.end()) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
          "Joint '%s' is missing state interface: '%s'", 
          joint.name.c_str(), expected.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_DEBUG(
      rclcpp::get_logger("Robot5_arm_read_numbers"), "%f ..",
      hw_states_[i].position );
    // // 初始化命令值为当前状态
    // hw_commands_ = hw_states_;
    if(sim_  > 0.0){

      for (uint i = 0; i < hw_states_.size(); i++)
      {
         hw_commands_[i].position = hw_states_[i].position;
      }
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
      "Joint '%s' initialized with %zu command and %zu state interfaces",
      joint.name.c_str(), 
      joint.command_interfaces.size(), 
      joint.state_interfaces.size());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

//从共享内存读取当前关节状态
hardware_interface::CallbackReturn Robot_arm_Ethercat_Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_DEBUG(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "%.1f seconds left.",
      hw_start_sec_ - i);
  }

  //reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    //hw_states_[i] = 0;
    //hw_commands_[i] = 0;
    
    hw_commands_[i].position = hw_states_[i].position;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Robot_arm_Ethercat_Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i) 
  {
    const auto& joint = info_.joints[i];
    
    // 注册状态接口中的所有字段
    interfaces.emplace_back(joint.name, "position", &hw_states_[i].position);
    interfaces.emplace_back(joint.name, "velocity", &hw_states_[i].velocity);
    interfaces.emplace_back(joint.name, "effort", &hw_states_[i].effort);
    
    // 双编码器接口
    interfaces.emplace_back(joint.name, "motor_encoder_0", &hw_states_[i].motor_encoder[0]);
    interfaces.emplace_back(joint.name, "motor_encoder_1", &hw_states_[i].motor_encoder[1]);
    
    // 状态和控制接口 - 使用reinterpret_cast
    interfaces.emplace_back(joint.name, "status", 
                           (&hw_states_[i].status));
    interfaces.emplace_back(joint.name, "error_code",
                           (&hw_states_[i].error_code));
    interfaces.emplace_back(joint.name, "mode", 
                           (&hw_states_[i].mode));
    interfaces.emplace_back(joint.name, "power_enable", 
                           (&hw_states_[i].power_enable));
  }
  
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Robot_arm_Ethercat_Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i) 
  {
    const auto& joint = info_.joints[i];
    
    // 注册命令接口中的所有字段
    interfaces.emplace_back(joint.name, "position", &hw_commands_[i].position);
    interfaces.emplace_back(joint.name, "velocity", &hw_commands_[i].velocity);
    interfaces.emplace_back(joint.name, "effort", &hw_commands_[i].effort);
    
    // 启用状态接口
    interfaces.emplace_back(joint.name, "power_enable", 
                           (&hw_commands_[i].power_enable));
    
        // 状态和控制接口 - 使用reinterpret_cast
    interfaces.emplace_back(joint.name, "status", 
                           (&hw_commands_[i].status));
    interfaces.emplace_back(joint.name, "mode", 
                           (&hw_commands_[i].mode));
  }
  return interfaces;
}

//硬件系统进入激活状态时调用，用于初始化设备或确保设备处于正确的初始状态
hardware_interface::CallbackReturn Robot_arm_Ethercat_Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    // rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_DEBUG(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "%.1f seconds left..",
      hw_start_sec_ - i);
  }
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i].position = hw_states_[i].position;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//在硬件系统进入去激活状态时调用，用于安全地关闭设备或清理资源，以确保设备在停止操作时处于安全状态
hardware_interface::CallbackReturn Robot_arm_Ethercat_Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    //rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_DEBUG(
      rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }
  joint_shm_desire_ptr->ec_poweron = 0;
  RCLCPP_DEBUG(rclcpp::get_logger("Robot_arm_Ethercat_Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//从PLC的共享内存中获取关节的实际状态，并将这些状态转换为适合控制系统使用的格式
hardware_interface::return_type Robot_arm_Ethercat_Hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if(sim_  < 1.0)
  {
    const bool igh_driver_alive = is_igh_driver_alive();
    const bool ethercat_operational =
      igh_driver_alive && joint_shm_real_ptr != nullptr && joint_shm_real_ptr->ec_powerstate == 1;
    if (!ethercat_operational) {
      static auto last_warning = std::chrono::steady_clock::time_point{};
      const auto now = std::chrono::steady_clock::now();
      if (last_warning.time_since_epoch().count() == 0 ||
          now - last_warning >= std::chrono::seconds(2)) {
        RCLCPP_WARN(
          rclcpp::get_logger("Robot_arm_Ethercat_Hardware"),
          "EtherCAT feedback is unavailable (igh_driver_alive=%s, pid=%d, ec_powerstate=%u); "
          "exposing all arm drive statuses as UNKNOWN(0)",
          igh_driver_alive ? "true" : "false",
          igh_driver_pid_,
          joint_shm_real_ptr != nullptr ? joint_shm_real_ptr->ec_powerstate : 0);
        last_warning = now;
      }
    }
    for (uint8_t i = 0; i < hw_states_.size(); i++)
    {
        if (!ethercat_operational) {
          // A newly-created or stale shared-memory block contains zeroed/unknown
          // drive data. Never expose that as a valid disabled drive state.
          hw_states_[i].velocity = 0.0;
          hw_states_[i].effort = 0.0;
          hw_states_[i].status = 0.0;
          hw_states_[i].error_code = 0.0;
          hw_states_[i].power_enable = 0.0;
          continue;
        }
        const std::size_t shared_slot = joint_shared_slots_[i];
        get_joint_real(
          joint_shm_real_ptr->axis_state[shared_slot], hw_states_[i], ZERO, 0);
        hw_states_[i].status = double(joint_shm_real_ptr->axis_state[shared_slot].ec_ctrstate);
    }
  }
  else{
    for (uint8_t i = 0; i < hw_states_.size(); i++)
    {
        const bool enabled = hw_commands_[i].power_enable != 0.0;
        hw_states_[i].status = enabled ? 39.0 : 64.0;
        hw_states_[i].error_code = 0.0;
        hw_states_[i].power_enable = enabled ? 1.0 : 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

//将关节的命令状态转换并写入到PLC的共享内存中，以便控制硬件执行指定的动作
hardware_interface::return_type Robot_arm_Ethercat_Hardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  bool power_enable_requested = false;
  for (const auto& command : hw_commands_) {
    if (command.power_enable != 0.0) {
      power_enable_requested = true;
      break;
    }
  }
  const bool power_request_allowed = sim_ >= 1.0 || is_igh_driver_alive();
  joint_shm_desire_ptr->ec_poweron =
    power_enable_requested && power_request_allowed ? 1 : 0;

  if(sim_  < 1.0)
  {
    for(uint8_t i=0;i<hw_states_.size();i++){
      const std::size_t shared_slot = joint_shared_slots_[i];
      set_joint_desire(
        joint_shm_desire_ptr->axis_ctr[shared_slot], hw_commands_[i], ZERO, 0,
        static_cast<uint8_t>(hw_commands_[i].mode));
      // RCLCPP_INFO(rclcpp::get_logger("erobot_hw_arm_real"), "HW !!!  %d: T write:%f,read: %f",i,hw_commands_[i].effort,hw_states_[i].effort);
    }
  }
  else{
    for(uint8_t i=0;i<hw_states_.size();i++){
      hw_states_[i].position = hw_commands_[i].position;
      hw_states_[i].velocity = hw_commands_[i].velocity;
      // RCLCPP_INFO(rclcpp::get_logger("erobot_hw_arm_sim"), "HW !!! size %d: velocity %f",i,hw_commands_[i].velocity);
    }
  }
  // RCLCPP_INFO(rclcpp::get_logger("erobot_hw_arm_sim"), "HW !!! size %d: velocity %f",14,hw_commands_[13].velocity);
  return hardware_interface::return_type::OK;
}

//shm interface define
void Robot_arm_Ethercat_Hardware::init_shm_desire()
{
  boost::interprocess::permissions perm;
  perm.set_unrestricted();
  // Remove shared memory if it already exists
  // shared_memory_object::remove("Nxtos_amr_disire");

  // Create a shared memory object
	shm_desire_ptr = std::make_shared<boost::interprocess::shared_memory_object>(boost::interprocess::open_or_create, "Ethercat_axis_desire", boost::interprocess::read_write,perm);

  // Set the size of the shared memory object
  shm_desire_ptr -> truncate(sizeof(ec_app_desire_reg_t));

  // Map the shared memory object to a region of memory
	mapped_region_desire_ptr = std::make_shared<boost::interprocess::mapped_region>(*shm_desire_ptr, boost::interprocess::read_write);

  // Get a pointer to the mapped region of memory
  joint_shm_desire_ptr = static_cast<ec_app_desire_reg_t*>(mapped_region_desire_ptr->get_address());
}

void Robot_arm_Ethercat_Hardware::init_shm_real()
{
  permissions perm;
  perm.set_unrestricted();
  // Remove shared memory if it already exists
  // shared_memory_object::remove("Nxtos_amr_real");

  // Create a shared memory object
	shm_real_ptr = std::make_shared<boost::interprocess::shared_memory_object>(boost::interprocess::open_or_create, "Ethercat_axis_real", boost::interprocess::read_write,perm);

  // Set the size of the shared memory object
  shm_real_ptr -> truncate(sizeof(ec_app_real_reg_t));

  // Map the shared memory object to a region of memory
	mapped_region_real_ptr = std::make_shared<boost::interprocess::mapped_region>(*shm_real_ptr, boost::interprocess::read_write);

  // Get a pointer to the mapped region of memory
  joint_shm_real_ptr = static_cast<ec_app_real_reg_t*>(mapped_region_real_ptr->get_address());
}



}  // namespace Robot_arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  Robot_arm_hardware_interface::Robot_arm_Ethercat_Hardware, hardware_interface::SystemInterface)

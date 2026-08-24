#include "joint_hardware/hardware_state_bus.hpp"

namespace joint_hardware
{

std::shared_ptr<HardwareStateBus> HardwareStateBus::instance()
{
  static std::shared_ptr<HardwareStateBus> shared{new HardwareStateBus()};
  return shared;
}

HardwareStateBus::Snapshot HardwareStateBus::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

HardwareStateBus::JointCommand HardwareStateBus::read_command(std::size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.commands.at(index);
}

HardwareStateBus::JointState HardwareStateBus::read_state(std::size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.states.at(index);
}

HardwareStateBus::WaistStatus HardwareStateBus::read_waist_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return waist_status_;
}

void HardwareStateBus::write_command(std::size_t index, const JointCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.commands.at(index) = command;
}

void HardwareStateBus::write_state(std::size_t index, const JointState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.states.at(index) = state;
}

void HardwareStateBus::write_waist_status(const WaistStatus & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  waist_status_ = status;
}

}  // namespace joint_hardware

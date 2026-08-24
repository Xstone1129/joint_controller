#ifndef JOINT_HARDWARE__HARDWARE_STATE_BUS_HPP_
#define JOINT_HARDWARE__HARDWARE_STATE_BUS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

namespace joint_hardware
{

class HardwareStateBus
{
public:
  static constexpr std::size_t kNumAxes = 20;

  struct JointCommand
  {
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};
    uint64_t seq{0};
  };

  struct JointState
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    uint64_t seq{0};
  };

  struct Snapshot
  {
    std::array<JointCommand, kNumAxes> commands{};
    std::array<JointState, kNumAxes> states{};
  };

  struct WaistStatus
  {
    bool initialized{false};
    bool feedback_valid{false};
    bool feedback_fresh{false};
    bool fault_active{false};
    uint16_t feedback_error_code{0};
    uint64_t seq{0};
  };

  static std::shared_ptr<HardwareStateBus> instance();
  Snapshot snapshot() const;
  JointCommand read_command(std::size_t index) const;
  JointState read_state(std::size_t index) const;
  WaistStatus read_waist_status() const;
  void write_command(std::size_t index, const JointCommand & command);
  void write_state(std::size_t index, const JointState & state);
  void write_waist_status(const WaistStatus & status);

private:
  HardwareStateBus() = default;

  mutable std::mutex mutex_;
  Snapshot snapshot_{};
  WaistStatus waist_status_{};
};

inline constexpr std::size_t kWaistSharedIndex = 5;

}  // namespace joint_hardware

#endif  // JOINT_HARDWARE__HARDWARE_STATE_BUS_HPP_

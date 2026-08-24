#ifndef JOINT_HARDWARE__LIFT__CIA402_HPP_
#define JOINT_HARDWARE__LIFT__CIA402_HPP_

#include <cstdint>

namespace joint_hardware::lift
{

enum class Cia402State : uint8_t
{
  unknown,
  not_ready_to_switch_on,
  switch_on_disabled,
  ready_to_switch_on,
  switched_on,
  operation_enabled,
  quick_stop_active,
  fault_reaction_active,
  fault
};

Cia402State parse_cia402_status(uint16_t status_word) noexcept;

struct Cia402Inputs
{
  uint16_t status_word{0};
  uint16_t error_code{0};
  int8_t mode_display{0};
  int8_t expected_mode{9};
  bool link_operational{false};
  bool pdo_fresh{false};
  bool working_counter_ok{false};
  bool request_enable{true};
  // Standard CiA 402 quick-stop request. Bit 2 is cleared in 6040h,
  // producing 0x000B while the drive is still switched on.
  bool request_quick_stop{false};
  bool request_fault_reset{true};
};

struct Cia402Decision
{
  Cia402State state{Cia402State::unknown};
  uint16_t control_word{0};
  bool mode_ok{false};
  bool health_ok{false};
  bool allow_motion{false};
  bool fault{false};
  bool recovery_exhausted{false};
};

class Cia402Controller
{
public:
  explicit Cia402Controller(
    uint32_t max_fault_reset_attempts = 3, uint32_t fault_reset_backoff_cycles = 10) noexcept;

  Cia402Decision step(const Cia402Inputs & inputs) noexcept;
  void force_safe_stop() noexcept;
  void reset() noexcept;
  uint32_t fault_reset_attempts() const noexcept;

private:
  uint32_t max_fault_reset_attempts_{3};
  uint32_t fault_reset_backoff_cycles_{10};
  uint32_t fault_reset_attempts_{0};
  uint32_t fault_reset_backoff_remaining_{0};
  bool recovery_exhausted_{false};
};

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__CIA402_HPP_

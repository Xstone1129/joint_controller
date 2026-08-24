#include "joint_hardware/lift/cia402.hpp"

namespace joint_hardware::lift
{

Cia402State parse_cia402_status(uint16_t status_word) noexcept
{
  switch (status_word & 0x006fU) {
    case 0x0000:
      return Cia402State::not_ready_to_switch_on;
    case 0x0040:
      return Cia402State::switch_on_disabled;
    case 0x0021:
      return Cia402State::ready_to_switch_on;
    case 0x0023:
      return Cia402State::switched_on;
    case 0x0027:
      return Cia402State::operation_enabled;
    case 0x0007:
      return Cia402State::quick_stop_active;
    case 0x000f:
      return Cia402State::fault_reaction_active;
    case 0x0008:
      return Cia402State::fault;
    default:
      return Cia402State::unknown;
  }
}

Cia402Controller::Cia402Controller(
  uint32_t max_fault_reset_attempts, uint32_t fault_reset_backoff_cycles) noexcept
: max_fault_reset_attempts_(max_fault_reset_attempts),
  fault_reset_backoff_cycles_(fault_reset_backoff_cycles)
{
}

Cia402Decision Cia402Controller::step(const Cia402Inputs & inputs) noexcept
{
  Cia402Decision decision;
  decision.state = parse_cia402_status(inputs.status_word);
  decision.mode_ok = inputs.mode_display == inputs.expected_mode;
  decision.fault = decision.state == Cia402State::fault || inputs.error_code != 0;
  decision.health_ok = inputs.link_operational && inputs.pdo_fresh &&
    inputs.working_counter_ok && decision.mode_ok && !decision.fault;

  if (!decision.fault && decision.state != Cia402State::fault) {
    fault_reset_attempts_ = 0;
    fault_reset_backoff_remaining_ = 0;
    recovery_exhausted_ = false;
  }

  if (decision.fault) {
    if (fault_reset_backoff_remaining_ > 0) {
      --fault_reset_backoff_remaining_;
      decision.control_word = 0x0006;
    } else if (inputs.request_fault_reset && fault_reset_attempts_ < max_fault_reset_attempts_) {
      ++fault_reset_attempts_;
      fault_reset_backoff_remaining_ = fault_reset_backoff_cycles_;
      decision.control_word = 0x0080;  // Fault reset, bit 7.
    } else {
      recovery_exhausted_ = true;
      decision.control_word = 0x0006;  // Shutdown while fault remains observable.
    }
    decision.recovery_exhausted = recovery_exhausted_;
    decision.allow_motion = false;
    return decision;
  }

  if (!decision.health_ok) {
    // A communication, mode, PDO or explicit brake-disable problem always
    // requests a controlled shutdown.  The drive's configured stop/brake timing
    // then owns deceleration and BR+/BR- sequencing.
    decision.control_word = 0x0006;
    decision.allow_motion = false;
    return decision;
  }

  if (inputs.request_quick_stop) {
    // 0x000B keeps the drive powered and clears the Quick Stop bit.  The drive
    // applies its configured deceleration (605Ah/P05.06/P06.14) and then its
    // automatic BR+/BR- brake timing.  LiftHardware changes to 0x0006 only
    // after the PDO reports the configured low-speed threshold or the bounded
    // stop deadline expires.
    switch (decision.state) {
      case Cia402State::switched_on:
      case Cia402State::operation_enabled:
      case Cia402State::quick_stop_active:
        decision.control_word = 0x000b;
        break;
      default:
        decision.control_word = 0x0006;
        break;
    }
    decision.allow_motion = false;
    return decision;
  }

  if (!inputs.request_enable) {
    decision.control_word = 0x0006;
    decision.allow_motion = false;
    return decision;
  }

  switch (decision.state) {
    case Cia402State::switch_on_disabled:
    case Cia402State::not_ready_to_switch_on:
      decision.control_word = 0x0006;  // Shutdown.
      break;
    case Cia402State::ready_to_switch_on:
      decision.control_word = 0x0007;  // Switch on.
      break;
    case Cia402State::switched_on:
    case Cia402State::operation_enabled:
      decision.control_word = 0x000f;  // Enable operation.
      break;
    case Cia402State::quick_stop_active:
      decision.control_word = 0x000f;
      break;
    case Cia402State::fault_reaction_active:
    case Cia402State::fault:
    case Cia402State::unknown:
      decision.control_word = 0x0006;
      break;
  }

  decision.allow_motion = decision.state == Cia402State::operation_enabled;
  return decision;
}

void Cia402Controller::force_safe_stop() noexcept
{
  recovery_exhausted_ = false;
}

void Cia402Controller::reset() noexcept
{
  fault_reset_attempts_ = 0;
  fault_reset_backoff_remaining_ = 0;
  recovery_exhausted_ = false;
}

uint32_t Cia402Controller::fault_reset_attempts() const noexcept
{
  return fault_reset_attempts_;
}

}  // namespace joint_hardware::lift

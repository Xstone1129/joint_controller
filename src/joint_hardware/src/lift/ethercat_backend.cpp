#include "joint_hardware/lift/ethercat_backend.hpp"
#include "joint_hardware/lift/etherlab_backend.hpp"
#include "joint_hardware/lift/ethercat_sdk_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace joint_hardware::lift
{

namespace
{

void store_u16_le(uint8_t * destination, uint16_t value) noexcept
{
  destination[0] = static_cast<uint8_t>(value & 0xffU);
  destination[1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

void store_u32_le(uint8_t * destination, uint32_t value) noexcept
{
  for (std::size_t index = 0; index < sizeof(uint32_t); ++index) {
    destination[index] = static_cast<uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

uint16_t load_u16_le(const uint8_t * source) noexcept
{
  return static_cast<uint16_t>(source[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(source[1]) << 8U);
}

uint32_t load_u32_le(const uint8_t * source) noexcept
{
  uint32_t value = 0;
  for (std::size_t index = 0; index < sizeof(uint32_t); ++index) {
    value |= static_cast<uint32_t>(source[index]) << (8U * index);
  }
  return value;
}

template<typename T>
void append_le(std::vector<uint8_t> & bytes, T value)
{
  using U = std::make_unsigned_t<T>;
  const U unsigned_value = static_cast<U>(value);
  bytes.resize(sizeof(T));
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    bytes[i] = static_cast<uint8_t>((unsigned_value >> (8U * i)) & 0xffU);
  }
}

template<typename T>
bool read_le(const std::vector<uint8_t> & bytes, T & value)
{
  if (bytes.size() != sizeof(T)) {
    return false;
  }
  using U = std::make_unsigned_t<T>;
  U unsigned_value = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    unsigned_value |= static_cast<U>(bytes[i]) << (8U * i);
  }
  value = static_cast<T>(unsigned_value);
  return true;
}

}  // namespace

bool encode_lift_rx_pdo_le(const LiftRxPdo & pdo, LiftRxPdoWire & wire) noexcept
{
  store_u16_le(wire.data(), pdo.control_word);
  store_u32_le(wire.data() + 2, static_cast<uint32_t>(pdo.target_velocity_units_per_s));
  store_u16_le(wire.data() + 6, static_cast<uint16_t>(pdo.torque_feedforward));
  return true;
}

bool decode_lift_rx_pdo_le(const LiftRxPdoWire & wire, LiftRxPdo & pdo) noexcept
{
  pdo.control_word = load_u16_le(wire.data());
  pdo.target_velocity_units_per_s = static_cast<int32_t>(load_u32_le(wire.data() + 2));
  pdo.torque_feedforward = static_cast<int16_t>(load_u16_le(wire.data() + 6));
  return true;
}

bool encode_lift_tx_pdo_le(const LiftTxPdo & pdo, LiftTxPdoWire & wire) noexcept
{
  store_u16_le(wire.data(), pdo.status_word);
  store_u16_le(wire.data() + 2, pdo.error_code);
  wire[4] = static_cast<uint8_t>(pdo.mode_display);
  store_u32_le(wire.data() + 5, static_cast<uint32_t>(pdo.actual_position_units));
  store_u32_le(wire.data() + 9, static_cast<uint32_t>(pdo.actual_velocity_units_per_s));
  store_u32_le(wire.data() + 13, pdo.digital_inputs);
  return true;
}

bool decode_lift_tx_pdo_le(const LiftTxPdoWire & wire, LiftTxPdo & pdo) noexcept
{
  pdo.status_word = load_u16_le(wire.data());
  pdo.error_code = load_u16_le(wire.data() + 2);
  pdo.mode_display = static_cast<int8_t>(wire[4]);
  pdo.actual_position_units = static_cast<int32_t>(load_u32_le(wire.data() + 5));
  pdo.actual_velocity_units_per_s = static_cast<int32_t>(load_u32_le(wire.data() + 9));
  pdo.digital_inputs = load_u32_le(wire.data() + 13);
  return true;
}

MockLiftEthercatBackend::MockLiftEthercatBackend()
{
  input_.mode_display = 9;
  input_.status_word = 0x0040;
}

bool MockLiftEthercatBackend::initialize(const EthercatMasterConfig & master)
{
  master_ = master;
  if (master_.cycle_period <= std::chrono::nanoseconds::zero()) {
    error_message_ = "mock EtherCAT cycle period must be positive";
    return false;
  }
  initialized_ = true;
  configured_ = false;
  pdo_fresh_ = false;
  working_counter_ = 0;
  link_state_ = EthercatLinkState::pre_operational;
  error_message_.clear();
  return true;
}

bool MockLiftEthercatBackend::configure_slave(const EthercatSlaveConfig & slave)
{
  if (!initialized_) {
    error_message_ = "mock backend is not initialized";
    return false;
  }
  slave_ = slave;
  configured_ = true;
  // PDO mapping is only legal in Pre-Operational. start() transitions to
  // Operational after configure_lift_csv_pdos() has completed.
  link_state_ = EthercatLinkState::pre_operational;
  working_counter_ = 1;
  error_message_.clear();
  return true;
}

bool MockLiftEthercatBackend::read_sdo(
  uint16_t index, uint8_t subindex, std::vector<uint8_t> & value)
{
  if (!configured_) {
    error_message_ = "mock slave is not configured";
    return false;
  }
  switch (index) {
    case 0x608f:
      if (subindex == 1) {
        append_le(value, encoder_counts_per_rev_);
        return true;
      }
      break;
    case 0x6091:
      if (subindex == 1) {
        append_le(value, gear_ratio_numerator_);
        return true;
      }
      if (subindex == 2) {
        append_le(value, gear_ratio_denominator_);
        return true;
      }
      break;
    case 0x6092:
      if (subindex == 1) {
        append_le(value, feed_units_numerator_);
        return true;
      }
      break;
    case 0x2008:
      if (subindex == 0) {
        append_le(value, p00_08_command_units_per_rev_);
        return true;
      }
      break;
    case 0x6061:
      if (subindex == 0) {
        value.assign(1, static_cast<uint8_t>(mode_));
        return true;
      }
      break;
    case 0x6098:
      if (subindex == 0) {
        value.assign(1, static_cast<uint8_t>(homing_method_));
        return true;
      }
      break;
    case 0x6099:
      if (subindex == 1) {
        append_le(value, homing_speed_high_units_s_);
        return true;
      }
      if (subindex == 2) {
        append_le(value, homing_speed_low_units_s_);
        return true;
      }
      break;
    case 0x609a:
      if (subindex == 0) {
        append_le(value, homing_acceleration_units_s2_);
        return true;
      }
      break;
    case 0x607c:
      if (subindex == 0) {
        append_le(value, homing_offset_units_);
        return true;
      }
      break;
    case 0x2015:
      if (subindex == 0) {
        append_le(value, absolute_encoder_setting_);
        return true;
      }
      break;
    case 0x2437:
      if (subindex == 0) {
        append_le(value, brake_p04_37_ms_);
        return true;
      }
      break;
    case 0x2438:
      if (subindex == 0) {
        append_le(value, brake_p04_38_ms_);
        return true;
      }
      break;
    case 0x2439:
      if (subindex == 0) {
        append_le(value, brake_p04_39_rpm_);
        return true;
      }
      break;
    case 0x2614:
      if (subindex == 0) {
        append_le(value, brake_p06_14_ms_);
        return true;
      }
      break;
    case 0x2506:
      if (subindex == 0) {
        append_le(value, brake_p05_06_mode_);
        return true;
      }
      break;
    case 0x2510:
      if (subindex == 0) {
        append_le(value, brake_p05_10_mode_);
        return true;
      }
      break;
    default:
      break;
  }
  error_message_ = "mock SDO object is unavailable";
  return false;
}

bool MockLiftEthercatBackend::write_sdo(
  uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value)
{
  if (!configured_) {
    error_message_ = "mock slave is not configured";
    return false;
  }
  if (index == 0x6060 && subindex == 0) {
    if (value.size() != 1 || (value[0] != 6 && value[0] != 9)) {
      error_message_ = "mock only accepts HM mode 6 or CSV mode 9";
      return false;
    }
    mode_ = static_cast<int8_t>(value[0]);
    return true;
  }
  if (index == 0x6098 && subindex == 0 && value.size() == 1) {
    const int8_t method = static_cast<int8_t>(value[0]);
    if (method < -6 || method > 37 || method == 0 || method == 36) {
      error_message_ = "mock homing method is outside the LD3M manual range";
      return false;
    }
    homing_method_ = method;
    return true;
  }
  if (index == 0x6099 && (subindex == 1 || subindex == 2)) {
    uint32_t parsed = 0;
    if (!read_le(value, parsed) || parsed == 0) {
      error_message_ = "mock homing speed must be positive";
      return false;
    }
    (subindex == 1 ? homing_speed_high_units_s_ : homing_speed_low_units_s_) = parsed;
    return true;
  }
  if (index == 0x609a && subindex == 0) {
    uint32_t parsed = 0;
    if (!read_le(value, parsed) || parsed == 0) {
      error_message_ = "mock homing acceleration must be positive";
      return false;
    }
    homing_acceleration_units_s2_ = parsed;
    return true;
  }
  if (index == 0x607c && subindex == 0) {
    if (value.size() != 4) {
      error_message_ = "mock homing offset must be int32";
      return false;
    }
    uint32_t raw = 0;
    if (!read_le(value, raw)) {
      return false;
    }
    homing_offset_units_ = static_cast<int32_t>(raw);
    return true;
  }
  if (index == 0x2015 && subindex == 0) {
    if (value.size() != 2 || value[0] != 9 || value[1] != 0) {
      error_message_ = "mock only accepts P00.15=9 for drive zero";
      return false;
    }
    absolute_encoder_setting_ = 9;
    return true;
  }
  if (index == 0x2008 && subindex == 0) {
    uint32_t parsed = 0;
    if (!read_le(value, parsed) || parsed > 8388608U) {
      error_message_ = "mock 2008h value is outside the manual range";
      return false;
    }
    p00_08_command_units_per_rev_ = parsed;
    return true;
  }
  const bool mapping_index = index == 0x1C12 || index == 0x1C13 ||
    index == 0x1600 || index == 0x1601 || index == 0x1602 || index == 0x1603 ||
    index == 0x1A00 || index == 0x1A01;
  if (mapping_index) {
    const bool assignment = index == 0x1C12 || index == 0x1C13;
    const uint8_t expected_width = assignment && subindex == 0 ? 1 :
      (assignment ? 2 : (subindex == 0 ? 1 : 4));
    if (value.size() != expected_width) {
      error_message_ = "mock PDO mapping SDO width is invalid";
      return false;
    }
    return true;
  }
  uint32_t parsed = 0;
  if (!read_le(value, parsed) || parsed == 0) {
    error_message_ = "mock SDO value must be a non-zero uint32";
    return false;
  }
  if (index == 0x608f && subindex == 1) {
    encoder_counts_per_rev_ = parsed;
  } else if (index == 0x6091 && subindex == 1) {
    gear_ratio_numerator_ = parsed;
  } else if (index == 0x6091 && subindex == 2) {
    gear_ratio_denominator_ = parsed;
  } else if (index == 0x6092 && subindex == 1) {
    feed_units_numerator_ = parsed;
  } else {
    error_message_ = "mock SDO object is unavailable";
    return false;
  }
  return true;
}

bool MockLiftEthercatBackend::start()
{
  if (!configured_) {
    error_message_ = "mock slave is not configured";
    return false;
  }
  link_state_ = EthercatLinkState::operational;
  pdo_fresh_ = true;
  working_counter_ = 1;
  startup_position_index_ = 0;
  input_.status_word = error_message_.empty() ? 0x0040 : 0x0008;
  return true;
}

bool MockLiftEthercatBackend::stop()
{
  link_state_ = initialized_ ? EthercatLinkState::safe_operational : EthercatLinkState::offline;
  pdo_fresh_ = false;
  last_output_ = LiftRxPdo{};
  input_.actual_velocity_units_per_s = 0;
  input_.status_word = error_message_.empty() ? 0x0040 : 0x0008;
  return true;
}

bool MockLiftEthercatBackend::read_pdo(LiftTxPdo & input)
{
  if (startup_position_index_ < startup_position_sequence_.size()) {
    input_.actual_position_units = startup_position_sequence_[startup_position_index_++];
  }
  input = input_;
  return link_state_ == EthercatLinkState::operational;
}

bool MockLiftEthercatBackend::write_pdo(const LiftRxPdo & output)
{
  last_output_ = output;
  if (link_state_ != EthercatLinkState::operational || !pdo_fresh_) {
    return link_state_ == EthercatLinkState::operational;
  }

  if (output.control_word == 0x0080 && input_.error_code != 0) {
    input_.error_code = 0;
    input_.status_word = 0x0040;
  }

  const uint16_t command = static_cast<uint16_t>(output.control_word & 0x008fU);
  if (input_.error_code != 0) {
    input_.status_word = 0x0008;
    input_.actual_velocity_units_per_s = 0;
  } else if (command == 0x0006) {
    input_.status_word = 0x0021;  // Ready to switch on.
    input_.actual_velocity_units_per_s = 0;
  } else if (command == 0x0007) {
    input_.status_word = 0x0023;  // Switched on.
    input_.actual_velocity_units_per_s = 0;
  } else if (command == 0x000b) {
    // The real drive performs its configured quick-stop deceleration and
    // reports the Quick Stop Active state before the host sends 0x0006. The
    // default mock settles in one 10 ms exchange; tests can override the
    // reported speed to exercise the bounded P06.14 timeout.
    input_.status_word = 0x0007;
    input_.actual_velocity_units_per_s = quick_stop_velocity_units_per_s_;
  } else if (mode_ == 6 && (output.control_word & 0x0010U) != 0U && command == 0x000f) {
    input_.status_word = 0x1427;  // Operation enabled, target reached, homing attained.
    input_.actual_velocity_units_per_s = 0;
    input_.actual_position_units = homing_offset_units_;
  } else if (command == 0x000f) {
    input_.status_word = 0x0027;  // Operation enabled.
    input_.actual_velocity_units_per_s = output.target_velocity_units_per_s;
  } else {
    input_.status_word = 0x0040;
    input_.actual_velocity_units_per_s = 0;
  }

  if (input_.actual_velocity_units_per_s != 0) {
    const double cycle_s =
      std::chrono::duration<double>(master_.cycle_period).count();
    const auto delta = static_cast<int64_t>(std::llround(
        static_cast<double>(input_.actual_velocity_units_per_s) * cycle_s));
    const int64_t next = static_cast<int64_t>(input_.actual_position_units) + delta;
    input_.actual_position_units = static_cast<int32_t>(std::clamp<int64_t>(
        next, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
  }
  input_.mode_display = mode_;
  return true;
}

bool MockLiftEthercatBackend::exchange_pdo(const LiftRxPdo & output, LiftTxPdo & input)
{
  if (!write_pdo(output)) {
    input = input_;
    return false;
  }
  input = input_;
  return true;
}

EthercatLinkState MockLiftEthercatBackend::link_state() const noexcept
{
  return link_state_;
}

bool MockLiftEthercatBackend::pdo_fresh() const noexcept
{
  return pdo_fresh_;
}

uint32_t MockLiftEthercatBackend::working_counter() const noexcept
{
  return working_counter_;
}

std::string MockLiftEthercatBackend::error_message() const
{
  return error_message_;
}

void MockLiftEthercatBackend::set_online(bool online)
{
  if (online) {
    link_state_ = configured_ ? EthercatLinkState::operational : EthercatLinkState::pre_operational;
    pdo_fresh_ = configured_;
    working_counter_ = configured_ ? 1U : 0U;
  } else {
    link_state_ = EthercatLinkState::offline;
    pdo_fresh_ = false;
    working_counter_ = 0;
    input_.actual_velocity_units_per_s = 0;
  }
}

void MockLiftEthercatBackend::set_pdo_fresh(bool fresh)
{
  pdo_fresh_ = fresh;
}

void MockLiftEthercatBackend::set_working_counter(uint32_t working_counter)
{
  working_counter_ = working_counter;
}

void MockLiftEthercatBackend::inject_fault(uint16_t error_code)
{
  input_.error_code = error_code;
  input_.status_word = 0x0008;
  input_.actual_velocity_units_per_s = 0;
}

void MockLiftEthercatBackend::set_mode(int8_t mode)
{
  mode_ = mode;
  input_.mode_display = mode;
}

void MockLiftEthercatBackend::set_actual_position_units(int32_t position_units)
{
  input_.actual_position_units = position_units;
}

void MockLiftEthercatBackend::set_startup_position_sequence(
  std::vector<int32_t> position_units)
{
  startup_position_sequence_ = std::move(position_units);
  startup_position_index_ = 0;
}

void MockLiftEthercatBackend::set_digital_inputs(uint32_t digital_inputs)
{
  input_.digital_inputs = digital_inputs;
}

void MockLiftEthercatBackend::set_quick_stop_velocity_units(int32_t velocity_units_per_s)
{
  quick_stop_velocity_units_per_s_ = velocity_units_per_s;
}

void MockLiftEthercatBackend::set_brake_p06_14_ms(uint32_t stop_time_ms)
{
  brake_p06_14_ms_ = stop_time_ms;
}

const LiftRxPdo & MockLiftEthercatBackend::last_output() const noexcept
{
  return last_output_;
}

const LiftTxPdo & MockLiftEthercatBackend::current_input() const noexcept
{
  return input_;
}

bool UnavailableLiftEthercatBackend::initialize(const EthercatMasterConfig &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::configure_slave(const EthercatSlaveConfig &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::read_sdo(
  uint16_t, uint8_t, std::vector<uint8_t> &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::write_sdo(
  uint16_t, uint8_t, const std::vector<uint8_t> &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::start()
{
  return false;
}

bool UnavailableLiftEthercatBackend::stop()
{
  return true;
}

bool UnavailableLiftEthercatBackend::read_pdo(LiftTxPdo &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::write_pdo(const LiftRxPdo &)
{
  return false;
}

bool UnavailableLiftEthercatBackend::exchange_pdo(const LiftRxPdo &, LiftTxPdo &)
{
  return false;
}

EthercatLinkState UnavailableLiftEthercatBackend::link_state() const noexcept
{
  return EthercatLinkState::offline;
}

bool UnavailableLiftEthercatBackend::pdo_fresh() const noexcept
{
  return false;
}

uint32_t UnavailableLiftEthercatBackend::working_counter() const noexcept
{
  return 0;
}

std::string UnavailableLiftEthercatBackend::error_message() const
{
  return "EtherCAT SDK adapter unavailable; implement LiftEthercatBackend for the target master";
}

std::unique_ptr<LiftEthercatBackend> make_lift_ethercat_backend(const std::string & name)
{
  if (name == "mock" || name == "MockLiftEthercatBackend") {
    return std::make_unique<MockLiftEthercatBackend>();
  }
  if (name == "etherlab" || name == "igh" || name == "EtherLabLiftEthercatBackend") {
    return std::make_unique<EtherLabLiftEthercatBackend>();
  }
  // Names for a future vendor SDK are intentionally mapped to the unavailable
  // adapter until a real implementation is linked into this package.
  return std::make_unique<LiftEthercatSdkAdapter>();
}

}  // namespace joint_hardware::lift

#include "joint_hardware/lift/lift_units.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

namespace joint_hardware::lift
{

double effective_command_units_per_rev(const LiftUnitConfig & config) noexcept
{
  if (std::isfinite(config.effective_command_units_per_rev) &&
    config.effective_command_units_per_rev > 0.0)
  {
    return config.effective_command_units_per_rev;
  }
  return static_cast<double>(config.command_units_per_rev);
}

bool derive_command_units_per_rev(
  uint32_t encoder_counts_per_rev,
  uint32_t gear_ratio_numerator,
  uint32_t gear_ratio_denominator,
  uint32_t feed_units_per_rev,
  uint32_t p00_08_command_units_per_rev,
  double & units_per_rev,
  std::string & error) noexcept
{
  if (encoder_counts_per_rev == 0 || gear_ratio_numerator == 0 ||
    gear_ratio_denominator == 0 || feed_units_per_rev == 0)
  {
    error = "608F/6091/6092 values must be non-zero";
    return false;
  }

  if (p00_08_command_units_per_rev != 0) {
    units_per_rev = static_cast<double>(p00_08_command_units_per_rev);
  } else if (feed_units_per_rev != encoder_counts_per_rev) {
    // LD3M manual 9.4.4: when 6092-01 differs from 608F-01, the electronic
    // gear ratio is encoder_resolution / feed_constant, so the feed constant
    // is the effective command unit count per motor revolution.
    units_per_rev = static_cast<double>(feed_units_per_rev);
  } else {
    // When feed constant equals encoder resolution, 6091 is the active ratio:
    // encoder_units / command_units = numerator / denominator.
    units_per_rev = static_cast<double>(encoder_counts_per_rev) *
      static_cast<double>(gear_ratio_denominator) /
      static_cast<double>(gear_ratio_numerator);
  }

  if (!std::isfinite(units_per_rev) || units_per_rev < 1.0 ||
    units_per_rev > static_cast<double>(std::numeric_limits<uint32_t>::max()))
  {
    error = "effective command units/rev is outside uint32 range";
    return false;
  }
  return true;
}

bool validate_lift_units(const LiftUnitConfig & config, std::string & error)
{
  if (!std::isfinite(config.lead_mm_per_rev) || config.lead_mm_per_rev <= 0.0) {
    error = "lead_mm_per_rev must be finite and positive";
    return false;
  }
  if (!std::isfinite(config.lift_sign) || std::abs(config.lift_sign) < 0.5) {
    error = "lift_sign must be finite and non-zero (use -1 for this lift)";
    return false;
  }
  if (config.command_units_per_rev == 0 || config.encoder_counts_per_rev == 0 ||
    config.encoder_counts_denominator == 0)
  {
    error = "command units and 608F encoder ratio must be non-zero";
    return false;
  }
  if (config.gear_ratio_numerator == 0 || config.gear_ratio_denominator == 0 ||
    config.feed_units_numerator == 0 || config.feed_units_denominator == 0)
  {
    error = "608F/6091/6092 ratios must be non-zero";
    return false;
  }
  if (config.feed_units_denominator != 1) {
    error = "6092 has only subindex 01; feed_units_denominator must remain 1";
    return false;
  }
  const double sdo_units = static_cast<double>(config.feed_units_numerator) /
    static_cast<double>(config.feed_units_denominator);
  double expected_units = 0.0;
  if (!derive_command_units_per_rev(
      config.encoder_counts_per_rev, config.gear_ratio_numerator,
      config.gear_ratio_denominator, static_cast<uint32_t>(std::llround(sdo_units)),
      config.p00_08_command_units_per_rev, expected_units, error))
  {
    return false;
  }
  const double configured_units = effective_command_units_per_rev(config);
  if (std::abs(configured_units - expected_units) >
    1e-9 * std::max(1.0, expected_units))
  {
    error = "command_units_per_rev does not match validated 608F/6091/6092/P00.08";
    return false;
  }
  return true;
}

double position_units_to_m(
  int32_t actual_position_units, int32_t zero_offset_units, const LiftUnitConfig & config) noexcept
{
  const double delta_units = static_cast<double>(actual_position_units) -
    static_cast<double>(zero_offset_units);
  return config.lift_sign * delta_units /
         effective_command_units_per_rev(config) * config.lead_mm_per_rev / 1000.0;
}

double velocity_units_to_mps(
  int32_t actual_velocity_units_per_s,
  const LiftUnitConfig & config) noexcept
{
  return config.lift_sign * static_cast<double>(actual_velocity_units_per_s) /
         effective_command_units_per_rev(config) * config.lead_mm_per_rev / 1000.0;
}

int32_t wire_rpm_to_velocity_units(double wire_rpm, const LiftUnitConfig & config) noexcept
{
  const double units = wire_rpm / 60.0 * effective_command_units_per_rev(config);
  if (units >= static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return std::numeric_limits<int32_t>::max();
  }
  if (units <= static_cast<double>(std::numeric_limits<int32_t>::min())) {
    return std::numeric_limits<int32_t>::min();
  }
  return static_cast<int32_t>(std::llround(units));
}

double velocity_units_to_wire_rpm(
  int32_t velocity_units_per_s, const LiftUnitConfig & config) noexcept
{
  return static_cast<double>(velocity_units_per_s) * 60.0 /
         effective_command_units_per_rev(config);
}

double rpm_to_mps(double joint_rpm, const LiftUnitConfig & config) noexcept
{
  return config.lift_sign * joint_rpm * config.lead_mm_per_rev / 60000.0;
}

double max_velocity_mps(double max_rpm, const LiftUnitConfig & config) noexcept
{
  return std::abs(rpm_to_mps(std::abs(max_rpm), config));
}

}  // namespace joint_hardware::lift

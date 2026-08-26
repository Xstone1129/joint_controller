#ifndef JOINT_HARDWARE__LIFT__LIFT_UNITS_HPP_
#define JOINT_HARDWARE__LIFT__LIFT_UNITS_HPP_

#include <cstdint>
#include <string>

namespace joint_hardware::lift
{

struct LiftUnitConfig
{
  double lead_mm_per_rev{10.0 / 3.0};
  double lift_sign{-1.0};
  uint32_t command_units_per_rev{10000};
  // The drive may expose a rational effective unit/rev value when 6091 is
  // active.  Keep the exact value for 60FF/606C conversion and retain the
  // rounded field above for compatibility with existing parameters.
  double effective_command_units_per_rev{10000.0};
  uint32_t encoder_counts_per_rev{131072};
  uint32_t encoder_counts_denominator{1};
  uint32_t gear_ratio_numerator{1};
  uint32_t gear_ratio_denominator{1};
  uint32_t feed_units_numerator{10000};
  uint32_t feed_units_denominator{1};
  // P00.08 (2008h) has priority over the 608F/6091/6092 formula when nonzero.
  uint32_t p00_08_command_units_per_rev{0};
};

double effective_command_units_per_rev(const LiftUnitConfig & config) noexcept;

bool derive_command_units_per_rev(
  uint32_t encoder_counts_per_rev,
  uint32_t gear_ratio_numerator,
  uint32_t gear_ratio_denominator,
  uint32_t feed_units_per_rev,
  uint32_t p00_08_command_units_per_rev,
  double & units_per_rev,
  std::string & error) noexcept;

bool validate_lift_units(const LiftUnitConfig & config, std::string & error);

double position_units_to_m(
  int32_t actual_position_units, int32_t zero_offset_units, const LiftUnitConfig & config) noexcept;

double velocity_units_to_mps(
  int32_t actual_velocity_units_per_s,
  const LiftUnitConfig & config) noexcept;

int32_t wire_rpm_to_velocity_units(double wire_rpm, const LiftUnitConfig & config) noexcept;

double velocity_units_to_wire_rpm(
  int32_t velocity_units_per_s, const LiftUnitConfig & config) noexcept;

double rpm_to_mps(double joint_rpm, const LiftUnitConfig & config) noexcept;

double max_velocity_mps(double max_rpm, const LiftUnitConfig & config) noexcept;

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__LIFT_UNITS_HPP_

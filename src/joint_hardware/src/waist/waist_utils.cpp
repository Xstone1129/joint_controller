#include "joint_hardware/waist_hardware.hpp"

namespace joint_hardware
{

double WaistHardware::clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

}  // namespace joint_hardware

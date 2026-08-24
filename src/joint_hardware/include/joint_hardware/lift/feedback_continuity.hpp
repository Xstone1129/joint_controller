#pragma once

#include <cstdint>

namespace joint_hardware::lift
{

enum class FeedbackContinuityState : uint8_t
{
  invalid,
  baseline_established,
  continuous,
  jump_detected
};

struct FeedbackContinuityResult
{
  FeedbackContinuityState state{FeedbackContinuityState::invalid};
  double previous_position_m{0.0};
  double current_position_m{0.0};
  double delta_m{0.0};
  double threshold_m{0.0};
  double interval_ms{0.0};
  uint64_t epoch{0};
  uint32_t source{0};
};

class FeedbackContinuityGuard
{
public:
  FeedbackContinuityGuard() = default;
  FeedbackContinuityGuard(double threshold_m, int64_t max_interval_ns);

  void configure(double threshold_m, int64_t max_interval_ns) noexcept;
  void reset(uint32_t source) noexcept;
  void invalidate(uint32_t source) noexcept;
  FeedbackContinuityResult observe(
    double position_m, int64_t sample_time_ns, uint32_t source, bool fresh) noexcept;

  bool baseline_valid() const noexcept;
  uint64_t epoch() const noexcept;

private:
  void advance_epoch(uint32_t source) noexcept;

  double threshold_m_{0.2};
  int64_t max_interval_ns_{100000000};
  bool baseline_valid_{false};
  double previous_position_m_{0.0};
  int64_t previous_sample_time_ns_{0};
  uint64_t epoch_{0};
  uint32_t source_{0};
};

}  // namespace joint_hardware::lift

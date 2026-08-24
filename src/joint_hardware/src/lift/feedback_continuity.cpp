#include "joint_hardware/lift/feedback_continuity.hpp"

#include <cmath>

namespace joint_hardware::lift
{

FeedbackContinuityGuard::FeedbackContinuityGuard(
  double threshold_m, int64_t max_interval_ns)
{
  configure(threshold_m, max_interval_ns);
}

void FeedbackContinuityGuard::configure(double threshold_m, int64_t max_interval_ns) noexcept
{
  if (std::isfinite(threshold_m) && threshold_m > 0.0) {
    threshold_m_ = threshold_m;
  }
  if (max_interval_ns > 0) {
    max_interval_ns_ = max_interval_ns;
  }
}

void FeedbackContinuityGuard::advance_epoch(uint32_t source) noexcept
{
  ++epoch_;
  source_ = source;
  baseline_valid_ = false;
  previous_position_m_ = 0.0;
  previous_sample_time_ns_ = 0;
}

void FeedbackContinuityGuard::reset(uint32_t source) noexcept
{
  advance_epoch(source);
}

void FeedbackContinuityGuard::invalidate(uint32_t source) noexcept
{
  if (baseline_valid_ || source != source_) {
    advance_epoch(source);
  }
}

FeedbackContinuityResult FeedbackContinuityGuard::observe(
  double position_m, int64_t sample_time_ns, uint32_t source, bool fresh) noexcept
{
  FeedbackContinuityResult result;
  result.current_position_m = position_m;
  result.threshold_m = threshold_m_;

  if (!fresh || !std::isfinite(position_m) || sample_time_ns <= 0) {
    invalidate(source);
    result.epoch = epoch_;
    result.source = source_;
    return result;
  }

  if (source != source_) {
    advance_epoch(source);
  } else if (baseline_valid_ &&
    (sample_time_ns <= previous_sample_time_ns_ ||
    sample_time_ns - previous_sample_time_ns_ > max_interval_ns_))
  {
    advance_epoch(source);
  }

  result.epoch = epoch_;
  result.source = source_;
  if (!baseline_valid_) {
    previous_position_m_ = position_m;
    previous_sample_time_ns_ = sample_time_ns;
    baseline_valid_ = true;
    result.previous_position_m = position_m;
    result.state = FeedbackContinuityState::baseline_established;
    return result;
  }

  result.previous_position_m = previous_position_m_;
  result.delta_m = std::abs(position_m - previous_position_m_);
  result.interval_ms = static_cast<double>(sample_time_ns - previous_sample_time_ns_) * 1.0e-6;
  result.state = result.delta_m > threshold_m_ ?
    FeedbackContinuityState::jump_detected : FeedbackContinuityState::continuous;
  previous_position_m_ = position_m;
  previous_sample_time_ns_ = sample_time_ns;
  return result;
}

bool FeedbackContinuityGuard::baseline_valid() const noexcept
{
  return baseline_valid_;
}

uint64_t FeedbackContinuityGuard::epoch() const noexcept
{
  return epoch_;
}

}  // namespace joint_hardware::lift

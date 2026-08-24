#include <gtest/gtest.h>

#include "joint_hardware/lift/feedback_continuity.hpp"

namespace
{

using joint_hardware::lift::FeedbackContinuityGuard;
using joint_hardware::lift::FeedbackContinuityState;

constexpr uint32_t kSimulationSource = 1;
constexpr uint32_t kRealSource = 2;
constexpr int64_t kCycleNs = 10'000'000;

TEST(FeedbackContinuity, FirstFreshFrameOnlyEstablishesBaseline)
{
  FeedbackContinuityGuard guard(0.001, 100'000'000);
  guard.reset(kRealSource);
  const auto result = guard.observe(-0.001003, kCycleNs, kRealSource, true);
  EXPECT_EQ(result.state, FeedbackContinuityState::baseline_established);
  EXPECT_DOUBLE_EQ(result.current_position_m, -0.001003);
}

TEST(FeedbackContinuity, NewWorkspaceDoesNotCompareWithOldCoordinates)
{
  FeedbackContinuityGuard guard(0.2, 100'000'000);
  guard.reset(kRealSource);
  EXPECT_EQ(
    guard.observe(0.618455, kCycleNs, kRealSource, true).state,
    FeedbackContinuityState::baseline_established);
  const auto old_epoch = guard.epoch();

  guard.reset(kRealSource);
  const auto result = guard.observe(-0.001003, 2 * kCycleNs, kRealSource, true);
  EXPECT_EQ(result.state, FeedbackContinuityState::baseline_established);
  EXPECT_GT(result.epoch, old_epoch);
}

TEST(FeedbackContinuity, DetectsTrueJumpWithinOneWorkspace)
{
  FeedbackContinuityGuard guard(0.05, 100'000'000);
  guard.reset(kRealSource);
  guard.observe(0.0, kCycleNs, kRealSource, true);
  const auto result = guard.observe(0.1, 2 * kCycleNs, kRealSource, true);
  EXPECT_EQ(result.state, FeedbackContinuityState::jump_detected);
  EXPECT_DOUBLE_EQ(result.previous_position_m, 0.0);
  EXPECT_DOUBLE_EQ(result.current_position_m, 0.1);
  EXPECT_DOUBLE_EQ(result.delta_m, 0.1);
  EXPECT_DOUBLE_EQ(result.threshold_m, 0.05);
}

TEST(FeedbackContinuity, StaleRecoveryReestablishesBaseline)
{
  FeedbackContinuityGuard guard(0.05, 100'000'000);
  guard.reset(kRealSource);
  guard.observe(0.0, kCycleNs, kRealSource, true);
  EXPECT_EQ(
    guard.observe(0.0, 2 * kCycleNs, kRealSource, false).state,
    FeedbackContinuityState::invalid);
  EXPECT_EQ(
    guard.observe(0.1, 3 * kCycleNs, kRealSource, true).state,
    FeedbackContinuityState::baseline_established);
}

TEST(FeedbackContinuity, DataSourceSwitchReestablishesBaseline)
{
  FeedbackContinuityGuard guard(0.05, 100'000'000);
  guard.reset(kSimulationSource);
  guard.observe(0.0, kCycleNs, kSimulationSource, true);
  const auto result = guard.observe(0.1, 2 * kCycleNs, kRealSource, true);
  EXPECT_EQ(result.state, FeedbackContinuityState::baseline_established);
  EXPECT_EQ(result.source, kRealSource);
}

TEST(FeedbackContinuity, NormalConsecutiveFeedbackRemainsContinuous)
{
  FeedbackContinuityGuard guard(0.05, 100'000'000);
  guard.reset(kRealSource);
  guard.observe(-0.001003, kCycleNs, kRealSource, true);
  for (int sample = 2; sample <= 20; ++sample) {
    const auto result = guard.observe(
      -0.001003 - static_cast<double>(sample - 1) * 0.0001,
      sample * kCycleNs, kRealSource, true);
    EXPECT_EQ(result.state, FeedbackContinuityState::continuous);
  }
}

TEST(FeedbackContinuity, ExcessiveSampleGapStartsANewEpoch)
{
  FeedbackContinuityGuard guard(0.05, 50'000'000);
  guard.reset(kRealSource);
  guard.observe(0.0, kCycleNs, kRealSource, true);
  const auto old_epoch = guard.epoch();
  const auto result = guard.observe(0.1, 100'000'000, kRealSource, true);
  EXPECT_EQ(result.state, FeedbackContinuityState::baseline_established);
  EXPECT_GT(result.epoch, old_epoch);
}

}  // namespace

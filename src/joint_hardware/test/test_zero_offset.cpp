#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

#include "joint_hardware/lift/zero_offset_store.hpp"

namespace
{

std::filesystem::path test_path()
{
  return std::filesystem::temp_directory_path() /
         ("joint_hardware_zero_" + std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()) + ".cfg");
}

TEST(ZeroOffsetStore, AtomicRoundTripAndIdentityValidation)
{
  const auto path = test_path();
  joint_hardware::lift::ZeroOffsetRecord record;
  record.motor_id = "LVM08008H3G3-M17";
  record.slave_alias = 3;
  record.slave_position = 4;
  record.zero_offset_units = -123456;
  std::string error;
  ASSERT_TRUE(
    joint_hardware::lift::ZeroOffsetStore::save_atomic(
      path.string(), record,
      error)) << error;

  joint_hardware::lift::ZeroOffsetRecord loaded;
  EXPECT_EQ(
    joint_hardware::lift::ZeroOffsetStore::load(
      path.string(), record.motor_id, 3, 4, loaded, error),
    joint_hardware::lift::ZeroOffsetLoadResult::loaded);
  EXPECT_EQ(loaded.zero_offset_units, record.zero_offset_units);
  EXPECT_EQ(
    joint_hardware::lift::ZeroOffsetStore::load(
      path.string(), record.motor_id, 9, 4, loaded, error),
    joint_hardware::lift::ZeroOffsetLoadResult::invalid);
  std::filesystem::remove(path);
}

TEST(ZeroOffsetStore, MissingAndCorruptFilesAreNotAccepted)
{
  const auto path = test_path();
  joint_hardware::lift::ZeroOffsetRecord loaded;
  std::string error;
  EXPECT_EQ(
    joint_hardware::lift::ZeroOffsetStore::load(
      path.string(), "motor", 0, 0, loaded, error),
    joint_hardware::lift::ZeroOffsetLoadResult::missing);
  {
    std::ofstream stream(path);
    stream << "schema=1\nmotor_id=motor\n";
  }
  EXPECT_EQ(
    joint_hardware::lift::ZeroOffsetStore::load(
      path.string(), "motor", 0, 0, loaded, error),
    joint_hardware::lift::ZeroOffsetLoadResult::invalid);
  std::filesystem::remove(path);
}

}  // namespace

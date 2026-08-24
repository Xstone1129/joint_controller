#include "joint_hardware/lift/zero_offset_store.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>

namespace joint_hardware::lift
{

namespace
{

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parse_unsigned(const std::string & text, uint64_t & value)
{
  try {
    std::size_t used = 0;
    value = std::stoull(text, &used, 0);
    return used == text.size();
  } catch (...) {
    return false;
  }
}

bool parse_signed(const std::string & text, int64_t & value)
{
  try {
    std::size_t used = 0;
    value = std::stoll(text, &used, 0);
    return used == text.size();
  } catch (...) {
    return false;
  }
}

}  // namespace

ZeroOffsetLoadResult ZeroOffsetStore::load(
  const std::string & path,
  const std::string & expected_motor_id,
  uint16_t expected_alias,
  uint16_t expected_position,
  ZeroOffsetRecord & record,
  std::string & error)
{
  std::error_code existence_error;
  const bool exists = std::filesystem::exists(path, existence_error);
  if (existence_error) {
    error = "cannot inspect zero offset file: " + existence_error.message();
    return ZeroOffsetLoadResult::invalid;
  }
  if (!exists) {
    return ZeroOffsetLoadResult::missing;
  }
  std::ifstream stream(path);
  if (!stream.is_open()) {
    error = "cannot open zero offset file: " + path;
    return ZeroOffsetLoadResult::invalid;
  }

  std::map<std::string, std::string> values;
  std::string line;
  while (std::getline(stream, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.resize(comment);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      error = "zero offset file contains a line without '='";
      return ZeroOffsetLoadResult::invalid;
    }
    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    if (key.empty() || value.empty() || !values.emplace(key, value).second) {
      error = "zero offset file contains an empty or duplicate field";
      return ZeroOffsetLoadResult::invalid;
    }
  }

  const char * required[] = {
    "schema", "motor_id", "slave_alias", "slave_position", "zero_offset_units"};
  for (const char * key : required) {
    if (values.find(key) == values.end()) {
      error = "zero offset file is missing field: " + std::string(key);
      return ZeroOffsetLoadResult::invalid;
    }
  }

  uint64_t schema = 0;
  uint64_t alias = 0;
  uint64_t position = 0;
  int64_t offset = 0;
  if (!parse_unsigned(values["schema"], schema) || schema != 1 ||
    !parse_unsigned(values["slave_alias"], alias) || alias > std::numeric_limits<uint16_t>::max() ||
    !parse_unsigned(values["slave_position"], position) ||
    position > std::numeric_limits<uint16_t>::max() ||
    !parse_signed(values["zero_offset_units"], offset) ||
    offset<std::numeric_limits<int32_t>::min() || offset> std::numeric_limits<int32_t>::max())
  {
    error = "zero offset file contains an invalid numeric field";
    return ZeroOffsetLoadResult::invalid;
  }

  if (values["motor_id"] != expected_motor_id ||
    alias != expected_alias || position != expected_position)
  {
    error = "zero offset identity does not match configured motor/slave";
    return ZeroOffsetLoadResult::invalid;
  }

  record.schema = static_cast<uint32_t>(schema);
  record.motor_id = values["motor_id"];
  record.slave_alias = static_cast<uint16_t>(alias);
  record.slave_position = static_cast<uint16_t>(position);
  record.zero_offset_units = static_cast<int32_t>(offset);
  return ZeroOffsetLoadResult::loaded;
}

bool ZeroOffsetStore::save_atomic(
  const std::string & path, const ZeroOffsetRecord & record, std::string & error)
{
  if (path.empty() || record.schema != 1 || record.motor_id.empty()) {
    error = "zero offset path, schema or motor_id is invalid";
    return false;
  }

  std::error_code fs_error;
  const std::filesystem::path destination(path);
  if (destination.has_parent_path()) {
    std::filesystem::create_directories(destination.parent_path(), fs_error);
    if (fs_error) {
      error = "cannot create zero offset directory: " + fs_error.message();
      return false;
    }
  }

  const std::filesystem::path temporary = destination.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream.is_open()) {
      error = "cannot open temporary zero offset file";
      return false;
    }
    stream << "schema=1\n"
           << "motor_id=" << record.motor_id << "\n"
           << "slave_alias=" << record.slave_alias << "\n"
           << "slave_position=" << record.slave_position << "\n"
           << "zero_offset_units=" << record.zero_offset_units << "\n";
    stream.flush();
    if (!stream.good()) {
      error = "cannot write temporary zero offset file";
      return false;
    }
  }

  std::filesystem::rename(temporary, destination, fs_error);
  if (fs_error) {
    std::filesystem::remove(temporary, fs_error);
    error = "cannot atomically replace zero offset file: " + fs_error.message();
    return false;
  }
  return true;
}

}  // namespace joint_hardware::lift

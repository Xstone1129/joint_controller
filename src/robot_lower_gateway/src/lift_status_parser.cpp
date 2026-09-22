#include "robot_lower_gateway/lift_status_parser.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <json/json.h>

namespace robot_lower_gateway
{
namespace
{

bool parseObject(const std::string & text, Json::Value & root, std::string & error)
{
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::istringstream stream(text);
  if (!Json::parseFromStream(builder, stream, &root, &error)) {
    error = "invalid JSON: " + error;
    return false;
  }
  if (!root.isObject()) {
    error = "JSON root is not an object";
    return false;
  }
  return true;
}

bool requiredBool(
  const Json::Value & root, const char * name, bool & value, std::string & error)
{
  if (!root.isMember(name) || !root[name].isBool()) {
    error = std::string("missing or invalid boolean field '") + name + "'";
    return false;
  }
  value = root[name].asBool();
  return true;
}

bool requiredInteger(
  const Json::Value & root, const char * name, std::int64_t minimum,
  std::int64_t maximum, std::int64_t & value, std::string & error)
{
  if (!root.isMember(name) || !root[name].isIntegral()) {
    error = std::string("missing or invalid integer field '") + name + "'";
    return false;
  }
  value = root[name].asInt64();
  if (value < minimum || value > maximum) {
    error = std::string("integer field '") + name + "' is out of range";
    return false;
  }
  return true;
}

bool requiredDouble(
  const Json::Value & root, const char * name, double & value, std::string & error)
{
  if (!root.isMember(name) || !root[name].isNumeric()) {
    error = std::string("missing or invalid numeric field '") + name + "'";
    return false;
  }
  value = root[name].asDouble();
  if (!std::isfinite(value)) {
    error = std::string("numeric field '") + name + "' is not finite";
    return false;
  }
  return true;
}

bool requiredString(
  const Json::Value & root, const char * name, std::string & value, std::string & error)
{
  if (!root.isMember(name) || !root[name].isString()) {
    error = std::string("missing or invalid string field '") + name + "'";
    return false;
  }
  value = root[name].asString();
  return true;
}

std::string joinReasons(const std::vector<std::string> & reasons)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < reasons.size(); ++index) {
    if (index != 0) {
      stream << "; ";
    }
    stream << reasons[index];
  }
  return stream.str();
}

bool isEmptyFaultReason(const std::string & reason)
{
  return reason.empty() || reason == "none" || reason == "N/A";
}

}  // namespace

bool parseLiftDriverStatus(
  const std::string & json, LiftDriverStatus & status, std::string & error)
{
  Json::Value root;
  if (!parseObject(json, root, error)) {
    return false;
  }

  LiftDriverStatus parsed;
  std::int64_t integer_value = 0;
  if (!requiredBool(root, "feedback_fresh", parsed.feedback_fresh, error) ||
    !requiredBool(root, "ethercat_operational", parsed.ethercat_operational, error) ||
    !requiredInteger(
      root, "working_counter", std::numeric_limits<std::int32_t>::min(),
      std::numeric_limits<std::int32_t>::max(), integer_value, error))
  {
    return false;
  }
  parsed.working_counter = static_cast<std::int32_t>(integer_value);
  if (!requiredBool(root, "working_counter_ok", parsed.working_counter_ok, error) ||
    !requiredBool(root, "initialized", parsed.initialized, error) ||
    !requiredString(root, "cia402_state", parsed.cia402_state, error) ||
    !requiredInteger(root, "status_word", 0, 65535, integer_value, error))
  {
    return false;
  }
  parsed.status_word = static_cast<std::uint16_t>(integer_value);
  if (!requiredInteger(root, "error_code", 0, 65535, integer_value, error)) {
    return false;
  }
  parsed.error_code = static_cast<std::uint16_t>(integer_value);
  if (!requiredInteger(root, "mode_display", -128, 127, integer_value, error)) {
    return false;
  }
  parsed.mode_display = static_cast<std::int8_t>(integer_value);
  if (!requiredBool(root, "power_enable_command", parsed.command_enabled, error) ||
    !requiredBool(root, "power_enabled", parsed.enabled, error) ||
    !requiredBool(root, "brake_unlocked", parsed.brake_unlocked, error) ||
    !requiredBool(root, "estop_latched", parsed.estop_latched, error) ||
    !requiredBool(root, "motion_blocked", parsed.motion_blocked, error) ||
    !requiredBool(root, "quick_stop_active", parsed.quick_stop_active, error) ||
    !requiredDouble(root, "position", parsed.position, error) ||
    !requiredDouble(root, "velocity", parsed.velocity, error) ||
    !requiredDouble(root, "command_position", parsed.command_position, error) ||
    !requiredDouble(root, "command_velocity", parsed.command_velocity, error) ||
    !requiredDouble(root, "command_acceleration", parsed.command_acceleration, error) ||
    !requiredString(root, "fault_reason", parsed.fault_reason, error))
  {
    return false;
  }

  // These diagnostics were added after the original canonical LiftStatus
  // contract. Keep them optional so an older native driver remains usable,
  // while exposing the actual 60FFh target whenever it is available.
  if (root.isMember("target_rpm") || root.isMember("target_velocity_units") ||
    root.isMember("velocity_mode_active"))
  {
    if (!requiredDouble(root, "target_rpm", parsed.target_rpm, error) ||
      !requiredInteger(
        root, "target_velocity_units", std::numeric_limits<std::int32_t>::min(),
        std::numeric_limits<std::int32_t>::max(), integer_value, error) ||
      !requiredBool(root, "velocity_mode_active", parsed.velocity_mode_active, error))
    {
      return false;
    }
    parsed.target_velocity_units = static_cast<std::int32_t>(integer_value);
    parsed.pdo_target_available = true;
  }

  status = std::move(parsed);
  error.clear();
  return true;
}

bool parseLiftControlStatus(
  const std::string & json, LiftControlStatus & status, std::string & error)
{
  Json::Value root;
  if (!parseObject(json, root, error)) {
    return false;
  }
  LiftControlStatus parsed;
  if (!requiredString(root, "mode", parsed.mode, error) ||
    !requiredBool(root, "trajectory_active", parsed.trajectory_active, error) ||
    !requiredBool(root, "jog_active", parsed.jog_active, error))
  {
    return false;
  }
  // Optional: older controllers do not publish it, so a missing key is not an
  // error, but when it is present it explains a controller-latched fault.
  if (root.isMember("fault_reason") && root["fault_reason"].isString()) {
    parsed.fault_reason = root["fault_reason"].asString();
  }
  status = std::move(parsed);
  error.clear();
  return true;
}

bool liftPowerStateMatches(const LiftDriverStatus & status, bool enabled)
{
  if (!status.feedback_fresh || !status.ethercat_operational ||
    !status.working_counter_ok || !status.initialized)
  {
    return false;
  }
  const auto cia402_state_bits = static_cast<std::uint16_t>(
    status.status_word & kCia402StateMask);
  if (enabled) {
    return status.command_enabled && status.enabled && status.brake_unlocked &&
           status.cia402_state == "operation_enabled" &&
           cia402_state_bits == kCia402OperationEnabled;
  }
  const bool disabled_state = cia402_state_bits == kCia402ReadyToSwitchOn ||
    cia402_state_bits == kCia402SwitchedOn;
  return !status.command_enabled && !status.enabled && !status.brake_unlocked && disabled_state;
}

LiftSafetyStatus evaluateLiftSafety(const LiftDriverStatus & status)
{
  LiftSafetyStatus result;
  std::vector<std::string> reasons;

  const bool cia402_fault = status.cia402_state == "fault" ||
    status.cia402_state == "fault_reaction_active";
  result.fault = cia402_fault || status.error_code != 0;
  if (result.fault) {
    std::ostringstream reason;
    reason << "CiA 402 fault: state=" << status.cia402_state
           << " status_word=" << status.status_word
           << " error_code=" << status.error_code;
    reasons.push_back(reason.str());
  }

  if (status.mode_display != kExpectedLiftModeDisplay) {
    std::ostringstream reason;
    reason << "operation mode mismatch: expected_mode="
           << static_cast<int>(kExpectedLiftModeDisplay)
           << " (CSV) actual_mode=" << static_cast<int>(status.mode_display);
    reasons.push_back(reason.str());
  }
  if (status.estop_latched) {
    reasons.emplace_back("emergency stop latched");
  }
  if (status.quick_stop_active) {
    reasons.emplace_back("CiA 402 quick stop active");
  }

  // Older lift drivers used this combined latch reason even after both the
  // drive fault and mode mismatch had cleared. Current PDO facts above are
  // authoritative for those two conditions. Preserve every other explicit
  // driver-side motion latch so host safety faults are never normalized away.
  constexpr const char * kLegacyAmbiguousReason = "CiA 402 fault or mode mismatch";
  constexpr const char * kTransientPdoReason = "PDO exchange failed";
  const bool communication_recovered = status.feedback_fresh &&
    status.ethercat_operational && status.working_counter_ok && status.initialized;
  const bool recovered_transient_pdo_latch = communication_recovered &&
    status.fault_reason == kTransientPdoReason;
  if (status.motion_blocked && status.fault_reason != kLegacyAmbiguousReason &&
    !recovered_transient_pdo_latch)
  {
    if (!isEmptyFaultReason(status.fault_reason)) {
      reasons.push_back(status.fault_reason);
    } else {
      reasons.emplace_back("lift driver reported a motion safety latch");
    }
  }

  result.motion_blocked = !reasons.empty();
  result.fault_reason = joinReasons(reasons);
  return result;
}

bool liftEnablePreconditionsMet(
  const LiftDriverStatus & status, bool workspace_running, std::string & reason)
{
  if (!workspace_running) {
    reason = "lift power enable requires workspace RUNNING";
    return false;
  }
  if (!status.feedback_fresh) {
    reason = "lift power enable requires fresh PDO feedback";
    return false;
  }
  if (!status.ethercat_operational) {
    reason = "lift power enable requires EtherCAT Operational";
    return false;
  }
  if (!status.working_counter_ok) {
    reason = "lift power enable requires a complete WorkingCounter";
    return false;
  }
  if (!status.initialized) {
    reason = "lift power enable requires initialized hardware";
    return false;
  }
  const auto safety = evaluateLiftSafety(status);
  if (status.estop_latched || status.quick_stop_active || safety.fault ||
    safety.motion_blocked)
  {
    reason = safety.fault_reason.empty() ?
      "lift power enable rejected by the current safety state" : safety.fault_reason;
    return false;
  }
  reason.clear();
  return true;
}

bool liftMotionPreconditionsMet(
  const LiftDriverStatus & status, bool workspace_running, std::string & reason)
{
  if (!liftEnablePreconditionsMet(status, workspace_running, reason)) {
    return false;
  }
  if (!liftPowerStateMatches(status, true)) {
    std::ostringstream stream;
    stream << "lift motion requires confirmed Operation Enabled and brake release: "
           << "command_enabled=" << (status.command_enabled ? "true" : "false")
           << " enabled=" << (status.enabled ? "true" : "false")
           << " brake_unlocked=" << (status.brake_unlocked ? "true" : "false")
           << " cia402_state=" << status.cia402_state
           << " status_word=" << status.status_word;
    reason = stream.str();
    return false;
  }
  reason.clear();
  return true;
}

bool liftCommandRequestValid(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & values,
  double velocity_mps,
  double acceleration_mps2,
  const std::string & expected_joint_name,
  std::string & reason)
{
  if (joint_names.size() != 1 || joint_names[0] != expected_joint_name ||
    values.size() != 1 || !std::isfinite(values[0]) ||
    !std::isfinite(velocity_mps) || velocity_mps <= 0.0 ||
    !std::isfinite(acceleration_mps2) || acceleration_mps2 <= 0.0)
  {
    reason =
      "lift command requires joint_names=['" + expected_joint_name +
      "'], one finite target, vel>0 m/s and acc>0 m/s^2";
    return false;
  }
  reason.clear();
  return true;
}

}  // namespace robot_lower_gateway

#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
source "${ROS_SETUP}"

# Optional target workspace. It is sourced as an underlay and is never modified.
if [[ -n "${ROBOT_UNDERLAY_SETUP:-}" ]]; then
  if [[ ! -f "${ROBOT_UNDERLAY_SETUP}" ]]; then
    echo "Robot underlay setup not found: ${ROBOT_UNDERLAY_SETUP}" >&2
    exit 1
  fi
  source "${ROBOT_UNDERLAY_SETUP}"
fi

# Keep the invoked path instead of resolving the symlink. With colcon
# --symlink-install, resolving it would incorrectly point back into src/.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
if [[ -z "${GATEWAY_OVERLAY_SETUP:-}" ]]; then
  if [[ -f "${PACKAGE_PREFIX}/setup.bash" ]]; then
    GATEWAY_OVERLAY_SETUP="${PACKAGE_PREFIX}/setup.bash"
  else
    GATEWAY_OVERLAY_SETUP="$(dirname "${PACKAGE_PREFIX}")/setup.bash"
  fi
fi
if [[ ! -f "${GATEWAY_OVERLAY_SETUP}" ]]; then
  echo "Gateway overlay setup not found: ${GATEWAY_OVERLAY_SETUP}" >&2
  exit 1
fi
source "${GATEWAY_OVERLAY_SETUP}"

# ROS-generated setup scripts may inspect unset variables, so nounset is enabled
# only after all underlays and overlays have been sourced.
set -u
source "/home/user/joint_controller/scripts/runtime_log_env.sh"

# The gateway is a persistent systemd service started at boot, so a session
# directory chosen by runtime_log_env.sh would be frozen at boot time and would
# not match the current upper/lower runtime session.  Give its ROS logs one
# stable, always-findable location instead so lease expiries and brake-lock
# events are easy to locate during a live run.
GATEWAY_LOG_ROOT="${JUNIOR_GATEWAY_LOG_ROOT:-${HOME}/.ros/log/gateway}"
mkdir -p "${GATEWAY_LOG_ROOT}"
export ROS_LOG_DIR="${GATEWAY_LOG_ROOT}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset ROS_DISCOVERY_SERVER
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE

CONFIG_ARGUMENT=()
# Configuration is always resolved by lower_gateway.launch.py from this
# workspace's installed package share. There is deliberately no /etc override.

if [[ -n "${ROBOT_GATEWAY_ENABLE_COMMAND_PROXY:-}" ]]; then
  CONFIG_ARGUMENT+=(enable_command_proxy:="${ROBOT_GATEWAY_ENABLE_COMMAND_PROXY}")
fi
if [[ -n "${ROBOT_GATEWAY_COMMAND_PROXY_PREFIX:-}" ]]; then
  CONFIG_ARGUMENT+=(command_proxy_prefix:="${ROBOT_GATEWAY_COMMAND_PROXY_PREFIX}")
fi

exec ros2 launch robot_lower_gateway lower_gateway.launch.py "${CONFIG_ARGUMENT[@]}"

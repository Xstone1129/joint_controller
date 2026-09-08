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

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE

CONFIG_ARGUMENT=()
if [[ -n "${ROBOT_GATEWAY_CONFIG:-}" ]]; then
  if [[ ! -f "${ROBOT_GATEWAY_CONFIG}" ]]; then
    echo "Gateway config not found: ${ROBOT_GATEWAY_CONFIG}" >&2
    exit 1
  fi
  CONFIG_ARGUMENT=(config_file:="${ROBOT_GATEWAY_CONFIG}")
fi

if [[ -n "${ROBOT_GATEWAY_ENABLE_COMMAND_PROXY:-}" ]]; then
  CONFIG_ARGUMENT+=(enable_command_proxy:="${ROBOT_GATEWAY_ENABLE_COMMAND_PROXY}")
fi
if [[ -n "${ROBOT_GATEWAY_COMMAND_PROXY_PREFIX:-}" ]]; then
  CONFIG_ARGUMENT+=(command_proxy_prefix:="${ROBOT_GATEWAY_COMMAND_PROXY_PREFIX}")
fi

exec ros2 launch robot_lower_gateway lower_gateway.launch.py "${CONFIG_ARGUMENT[@]}"

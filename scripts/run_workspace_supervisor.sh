#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT=/home/user/joint_controller

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
source "${WORKSPACE_ROOT}/scripts/runtime_log_env.sh"

# The supervisor is a long-lived service, so keep its ROS logs in the session
# gateway category instead of the shared flat <session>/ros tree.
export ROS_LOG_DIR="${GATEWAY_LOG_DIR}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS

exec "${WORKSPACE_ROOT}/install/robot_control/lib/robot_control/workspace_supervisor_ubuntu" \
  --ros-args \
  -p allowed_source:="${WORKSPACE_CONTROL_ALLOWED_SOURCE:-jetson_192_168_2_10}" \
  -p status_source:="${WORKSPACE_STATUS_SOURCE:-ubuntu_192_168_2_20}" \
  -p real_hardware_readiness_timeout_ms:="${WORKSPACE_READINESS_TIMEOUT_MS:-60000}"

#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE

exec python3 "${SCRIPT_DIR}/run_workflow.py" "$@"

#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced: source scripts/source_lower_machine_env.sh" >&2
  exit 1
fi

LOWER_WORKSPACE_ROOT=/home/user/joint_controller

source /opt/ros/humble/setup.bash
if [[ ! -r "${LOWER_WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${LOWER_WORKSPACE_ROOT}/install/setup.bash is missing" >&2
  return 1
fi
source "${LOWER_WORKSPACE_ROOT}/install/setup.bash"

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/user/joint_controller/cyclonedds.xml

unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS

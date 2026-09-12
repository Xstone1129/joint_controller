#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT=/home/user/joint_controller
source "${WORKSPACE_ROOT}/scripts/runtime_log_env.sh"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS

cleanup() {
  rm -f /tmp/joint_controller_robot_command.lock
}
trap cleanup EXIT

if pgrep -u "$(id -u)" -f \
  '/opt/ros/humble/bin/ros2 launch (erobot_controller load_controller_arm\.launch\.py|robot_control robot_control\.launch\.py|robot_control joint_controller_stack_sim\.launch\.py)' \
  >/dev/null 2>&1; then
  echo "ERROR: an arm workspace launch is already running for this user." >&2
  exit 1
fi

echo "Starting managed simulation stack: SIM=1.0, RViz disabled"
ros2 launch robot_control joint_controller_stack_sim.launch.py

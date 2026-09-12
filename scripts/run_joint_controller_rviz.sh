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

USER_ID=$(id -u)
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${USER_ID}}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=${XDG_RUNTIME_DIR}/bus}"

GRAPHICAL_DISPLAY="${DISPLAY:-}"
GRAPHICAL_XAUTHORITY="${XAUTHORITY:-}"

for process_id in $(pgrep -u "${USER_ID}" -f 'gnome-shell|Xwayland|Xorg' 2>/dev/null || true); do
  if [ ! -r "/proc/${process_id}/environ" ]; then
    continue
  fi
  process_environment=$(tr '\0' '\n' < "/proc/${process_id}/environ")
  if [ -z "${GRAPHICAL_DISPLAY}" ]; then
    GRAPHICAL_DISPLAY=$(sed -n 's/^DISPLAY=//p' <<< "${process_environment}" | head -n 1)
  fi
  if [ -z "${GRAPHICAL_XAUTHORITY}" ]; then
    GRAPHICAL_XAUTHORITY=$(sed -n 's/^XAUTHORITY=//p' <<< "${process_environment}" | head -n 1)
  fi
  if [ -n "${GRAPHICAL_DISPLAY}" ] && [ -n "${GRAPHICAL_XAUTHORITY}" ]; then
    break
  fi
done

if [ -z "${GRAPHICAL_DISPLAY}" ]; then
  echo "ERROR: no active graphical DISPLAY was found for user $(id -un)." >&2
  exit 1
fi
if [ -z "${GRAPHICAL_XAUTHORITY}" ] || [ ! -r "${GRAPHICAL_XAUTHORITY}" ]; then
  echo "ERROR: no readable XAUTHORITY was found for DISPLAY ${GRAPHICAL_DISPLAY}." >&2
  exit 1
fi

export DISPLAY="${GRAPHICAL_DISPLAY}"
export XAUTHORITY="${GRAPHICAL_XAUTHORITY}"

RVIZ_CONFIG="$(ros2 pkg prefix erobot_controller)/share/erobot_controller/robot_arm/display_erobot.rviz"
if [ ! -r "${RVIZ_CONFIG}" ]; then
  echo "ERROR: RViz configuration is not readable: ${RVIZ_CONFIG}" >&2
  exit 1
fi

echo "Starting RViz on DISPLAY=${DISPLAY} with XAUTHORITY=${XAUTHORITY}"
exec /opt/ros/humble/lib/rviz2/rviz2 -d "${RVIZ_CONFIG}"

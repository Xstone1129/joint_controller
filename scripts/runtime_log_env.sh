#!/usr/bin/env bash

# Shared lower-machine runtime logging layout.
#
# Every lower entry point sources this file so ROS logs and non-ROS diagnostics
# share the same session selected by the upper machine.  The upper runtime
# writes only the basename (for example, 2026-09-08-21-52-14-junior-runtime)
# to .junior_runtime_session_name before requesting the lower workspace start.

if [[ -z "${WORKSPACE_ROOT:-}" ]]; then
  WORKSPACE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
fi

RUNTIME_LOG_USER="${JUNIOR_RUNTIME_LOG_USER:-${SUDO_USER:-}}"
if [[ -z "${RUNTIME_LOG_USER}" ]]; then
  RUNTIME_LOG_USER="$(id -un)"
  if [[ "${RUNTIME_LOG_USER}" == root ]]; then
    # systemd services run as root, while the persistent ROS log tree belongs
    # to the normal lower-machine account. Prefer the workspace owner and
    # fall back to the deployment's conventional user account.
    workspace_owner="$(stat -c '%U' "${WORKSPACE_ROOT}" 2>/dev/null || true)"
    if [[ -n "${workspace_owner}" && "${workspace_owner}" != root ]] &&
       getent passwd "${workspace_owner}" >/dev/null 2>&1; then
      RUNTIME_LOG_USER="${workspace_owner}"
    elif getent passwd user >/dev/null 2>&1; then
      RUNTIME_LOG_USER=user
    fi
  fi
fi
RUNTIME_LOG_HOME="$(getent passwd "${RUNTIME_LOG_USER}" 2>/dev/null | cut -d: -f6 || true)"
RUNTIME_LOG_HOME="${RUNTIME_LOG_HOME:-${HOME:-/home/user}}"

JUNIOR_LOG_ROOT="${JUNIOR_LOG_ROOT:-${RUNTIME_LOG_HOME}/.ros/log}"
SESSION_NAME_FILE="${JOINT_CONTROLLER_SESSION_NAME_FILE:-${WORKSPACE_ROOT}/.junior_runtime_session_name}"
timestamp="$(date '+%Y-%m-%d-%H-%M-%S')"
SESSION_SUFFIX="${JUNIOR_RUNTIME_SESSION_NAME:-${timestamp}-lower-runtime}"

if [[ -z "${JUNIOR_RUNTIME_SESSION_NAME:-}" && -r "${SESSION_NAME_FILE}" ]]; then
  session_name="$(tr -d '[:space:]' < "${SESSION_NAME_FILE}")"
  # Only accept a basename written by the upper runtime.  This prevents a
  # malformed marker from escaping the ROS log root.
  if [[ "${session_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
    SESSION_SUFFIX="${session_name}"
  fi
fi

SESSION_LOG_DIR="${JUNIOR_SESSION_LOG_DIR:-${JUNIOR_LOG_ROOT}/${SESSION_SUFFIX}}"
RUNTIME_LOG_DIR="${JOINT_CONTROLLER_LOG_ROOT:-${SESSION_LOG_DIR}/runtime}"
ROS_LOG_DIR="${JUNIOR_ROS_LOG_DIR:-${SESSION_LOG_DIR}/ros}"

# Categorized subdirectories mirror the upper-computer session layout, so a
# lower-machine session directory can be read the same way as an upper one:
# one top-level console log, one startup-diagnostic log, plus one directory per
# subsystem instead of a single flat ROS_LOG_DIR holding every node log.
ARM_LOG_DIR="${JUNIOR_ARM_LOG_DIR:-${SESSION_LOG_DIR}/arm}"
LIFT_LOG_DIR="${JUNIOR_LIFT_LOG_DIR:-${SESSION_LOG_DIR}/lift}"
ROBOT_CONTROL_LOG_DIR="${JUNIOR_ROBOT_CONTROL_LOG_DIR:-${SESSION_LOG_DIR}/robot_control}"
HARDWARE_LOG_DIR="${JUNIOR_HARDWARE_LOG_DIR:-${SESSION_LOG_DIR}/hardware}"
GATEWAY_LOG_DIR="${JUNIOR_GATEWAY_LOG_DIR:-${SESSION_LOG_DIR}/gateway}"
CONSOLE_LOG="${JUNIOR_CONSOLE_LOG:-${SESSION_LOG_DIR}/console.log}"
STARTUP_DIAGNOSTIC_LOG="${JUNIOR_STARTUP_DIAGNOSTIC_LOG:-${SESSION_LOG_DIR}/startup_diagnostic.log}"

mkdir -p "${RUNTIME_LOG_DIR}" "${ROS_LOG_DIR}" "${ARM_LOG_DIR}" "${LIFT_LOG_DIR}" \
  "${ROBOT_CONTROL_LOG_DIR}" "${HARDWARE_LOG_DIR}" "${GATEWAY_LOG_DIR}"

export JUNIOR_LOG_ROOT SESSION_NAME_FILE SESSION_SUFFIX SESSION_LOG_DIR
export JOINT_CONTROLLER_SESSION_NAME_FILE="${SESSION_NAME_FILE}"
export RUNTIME_LOG_DIR ROS_LOG_DIR
export JOINT_CONTROLLER_LOG_ROOT="${RUNTIME_LOG_DIR}"
export ROS_LOG_DIR="${ROS_LOG_DIR}"
export ARM_LOG_DIR LIFT_LOG_DIR ROBOT_CONTROL_LOG_DIR HARDWARE_LOG_DIR GATEWAY_LOG_DIR
export CONSOLE_LOG STARTUP_DIAGNOSTIC_LOG

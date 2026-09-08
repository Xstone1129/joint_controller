#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CONFIG="${HARDWARE_SERVER_CONFIG:-${SCRIPT_DIR}/hardware_io.yaml}"
HOST="${HARDWARE_SERVER_HOST:-}"
PORT="${HARDWARE_SERVER_PORT:-}"
LOCK="${HARDWARE_SERVER_LOCK:-/run/lock/junior-hardware-owner.lock}"
RUN_USER=${SUDO_USER:-user}
LOG_ROOT="${SCRIPT_DIR}/../../log/runtime"
timestamp=$(date '+%Y-%m-%d-%H-%M-%S')
SESSION_NAME_FILE="${JOINT_CONTROLLER_SESSION_NAME_FILE:-/home/user/joint_controller/.junior_runtime_session_name}"
SESSION_SUFFIX="${timestamp}-hardware-server"
if [[ -r "${SESSION_NAME_FILE}" ]]; then
  session_name=$(tr -d '[:space:]' < "${SESSION_NAME_FILE}")
  if [[ -n "${session_name}" ]]; then
    SESSION_SUFFIX="${session_name}"
  fi
fi
SESSION_LOG_DIR="${LOG_ROOT}/${SESSION_SUFFIX}"
RUNTIME_LOG_DIR="${SESSION_LOG_DIR}/runtime"
CONSOLE_LOG="${SESSION_LOG_DIR}/console.log"
mkdir -p "${RUNTIME_LOG_DIR}"
chown -R "${RUN_USER}:${RUN_USER}" "${SESSION_LOG_DIR}" 2>/dev/null || true
export JOINT_CONTROLLER_LOG_ROOT="${JOINT_CONTROLLER_LOG_ROOT:-${RUNTIME_LOG_DIR}}"
exec > >(tee -a "${CONSOLE_LOG}") 2>&1

echo "会话日志: ${SESSION_LOG_DIR}"
echo "运行日志: ${RUNTIME_LOG_DIR}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: sudo $0 [--dry-run]"
  echo "  default: protocol/config/lease server with dry-run backend"
  echo "  --real-backend: start lower-machine arm/lift EtherCAT workers"
  exit 0
fi

args=(--config "$CONFIG" --lock "$LOCK")
[[ -n "$HOST" ]] && args+=(--host "$HOST")
[[ -n "$PORT" ]] && args+=(--port "$PORT")
if [[ "${1:-}" == "--real-backend" ]]; then
  # EtherCAT setup belongs to the lower machine. The upper client only sends
  # high-level TCP commands and never sends EtherCAT configuration.
  if [[ "${HARDWARE_SERVER_SKIP_CONFIG:-0}" != "1" ]]; then
    echo "Configuring EtherCAT masters from ${CONFIG}..." >&2
    python3 "${SCRIPT_DIR}/ethercat_hardware_test.py" --config "${CONFIG}" configure
  fi
  args+=(--real-backend)
fi
exec python3 "${SCRIPT_DIR}/network/hardware_server.py" "${args[@]}"

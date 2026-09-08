#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CONFIG="${HARDWARE_SERVER_CONFIG:-${SCRIPT_DIR}/hardware_io.yaml}"
HOST="${HARDWARE_SERVER_HOST:-}"
PORT="${HARDWARE_SERVER_PORT:-}"
LOCK="${HARDWARE_SERVER_LOCK:-/run/lock/junior-hardware-owner.lock}"

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

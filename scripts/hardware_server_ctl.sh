#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT=hardware-server.service
UNIT_SOURCE="${WORKSPACE_ROOT}/systemd/${UNIT}"

usage() {
  cat <<'EOF'
Usage: scripts/hardware_server_ctl.sh <command>

Commands:
  install    Install the systemd unit (does not start it)
  enable     Enable the unit at boot
  start      Start the TCP hardware server
  stop       Stop the TCP hardware server
  pause      Pause the service (alias for stop)
  restart    Reload the unit and restart the TCP hardware server
  reload     Reload the unit and restart (use after code/unit changes)
  status     Show systemd status and the 7447 listener
  logs       Show recent service logs (LOG_LINES=100)
  follow     Follow service logs in real time
  monitor    Show live service logs and connection guidance
  clients    Show established/listening TCP sockets on port 7447
  probe      Send a protocol-level HELLO request
EOF
}

need_unit() {
  if ! systemctl cat "${UNIT}" >/dev/null 2>&1; then
    echo "${UNIT} is not installed. Run: $0 install" >&2
    exit 1
  fi
}

case "${1:-}" in
  install)
    sudo install -m 0644 "${UNIT_SOURCE}" "/etc/systemd/system/${UNIT}"
    sudo systemctl daemon-reload
    echo "Installed ${UNIT}; it was not started or enabled."
    ;;
  enable)
    need_unit
    sudo systemctl enable "${UNIT}"
    ;;
  start)
    need_unit
    sudo systemctl start "${UNIT}"
    ;;
  stop|pause)
    need_unit
    sudo systemctl stop "${UNIT}"
    ;;
  restart)
    need_unit
    sudo systemctl daemon-reload
    sudo systemctl restart "${UNIT}"
    ;;
  reload)
    need_unit
    sudo systemctl daemon-reload
    sudo systemctl restart "${UNIT}"
    echo "${UNIT} restarted; existing TCP clients must reconnect."
    ;;
  status)
    need_unit
    sudo systemctl status "${UNIT}" --no-pager
    sudo ss -ltnp | awk 'NR == 1 || /:7447([[:space:]]|$)/'
    ;;
  logs)
    need_unit
    sudo journalctl -u "${UNIT}" -n "${LOG_LINES:-100}" --no-pager
    ;;
  follow)
    need_unit
    sudo journalctl -u "${UNIT}" -f
    ;;
  monitor)
    need_unit
    echo "Monitoring ${UNIT}; client connect/request/error events appear below."
    echo "Current TCP sockets:"
    sudo ss -tnp '( sport = :7447 or dport = :7447 )' || true
    echo "Starting journal stream (Ctrl+C to stop):"
    sudo journalctl -u "${UNIT}" -f
    ;;
  clients)
    sudo ss -tnp '( sport = :7447 or dport = :7447 )'
    ;;
  probe)
    HOST=${HARDWARE_SERVER_HOST:-127.0.0.1}
    PORT=${HARDWARE_SERVER_PORT:-7447}
    python3 - "${HOST}" "${PORT}" <<'PY'
import json
import socket
import struct
import sys

host, port = sys.argv[1], int(sys.argv[2])
request = {
    "version": 1,
    "request_id": 1,
    "session_id": "hardware-server-ctl",
    "type": "HELLO",
    "payload": {},
}
body = json.dumps(request, separators=(",", ":")).encode("utf-8")

def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise RuntimeError("server closed the connection")
        data.extend(chunk)
    return bytes(data)

with socket.create_connection((host, port), timeout=5) as sock:
    sock.sendall(struct.pack("!I", len(body)) + body)
    size = struct.unpack("!I", recv_exact(sock, 4))[0]
    response = json.loads(recv_exact(sock, size).decode("utf-8"))
print(json.dumps(response, ensure_ascii=False, indent=2))
PY
    ;;
  -h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

#!/usr/bin/env bash
set -euo pipefail

interface="${ROBOT_GATEWAY_CONTROL_INTERFACE:-enp8s0}"
expected_ipv4="${ROBOT_GATEWAY_CONTROL_IPV4:-192.168.2.20}"
timeout_sec="${ROBOT_GATEWAY_NETWORK_WAIT_TIMEOUT_SEC:-0}"

if [[ -z "${interface}" || -z "${expected_ipv4}" ]]; then
  echo "ROBOT_GATEWAY_CONTROL_INTERFACE and ROBOT_GATEWAY_CONTROL_IPV4 must be set together" >&2
  exit 2
fi
if ! [[ "${timeout_sec}" =~ ^[0-9]+$ ]]; then
  echo "ROBOT_GATEWAY_NETWORK_WAIT_TIMEOUT_SEC must be a non-negative integer" >&2
  exit 2
fi

started_at=${SECONDS}
next_log_at=${SECONDS}
while true; do
  carrier=0
  address_ready=false
  if [[ -r "/sys/class/net/${interface}/carrier" ]]; then
    carrier="$(<"/sys/class/net/${interface}/carrier")"
  fi
  if [[ -d "/sys/class/net/${interface}" ]] &&
    ip -4 -o address show dev "${interface}" scope global 2>/dev/null | \
    awk -v expected="${expected_ipv4}" '
      {
        split($4, address, "/")
        if (address[1] == expected) {
          found = 1
        }
      }
      END {exit found ? 0 : 1}
    '
  then
    address_ready=true
  fi

  if [[ "${carrier}" == "1" && "${address_ready}" == "true" ]]; then
    echo "Gateway control network ready: interface=${interface} ipv4=${expected_ipv4} carrier=1"
    exit 0
  fi

  elapsed=$((SECONDS - started_at))
  if (( timeout_sec > 0 && elapsed >= timeout_sec )); then
    echo "Gateway control network unavailable after ${elapsed}s: interface=${interface} ipv4=${expected_ipv4} carrier=${carrier}" >&2
    exit 1
  fi
  if (( SECONDS >= next_log_at )); then
    echo "Waiting for gateway control network: interface=${interface} ipv4=${expected_ipv4} carrier=${carrier} address_ready=${address_ready}"
    next_log_at=$((SECONDS + 10))
  fi
  sleep 0.25
done

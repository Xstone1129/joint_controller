#!/usr/bin/env bash
set -euo pipefail

master_index="${1:-0}"
slave_position="${2:-0}"
interface_name="${3:-enp5s0}"

if [[ ! -e "/sys/class/net/${interface_name}" ]]; then
  echo "EtherCAT interface ${interface_name} does not exist" >&2
  exit 1
fi
if [[ ! -r "/sys/class/net/${interface_name}/operstate" ]]; then
  echo "Cannot inspect EtherCAT interface ${interface_name}" >&2
  exit 1
fi
operstate="$(<"/sys/class/net/${interface_name}/operstate")"
echo "EtherCAT interface: ${interface_name} (${operstate})"

command -v ethercat >/dev/null || {
  echo "ethercat CLI is not installed" >&2
  exit 1
}

echo "== EtherCAT master =="
ethercat master -m "${master_index}"
echo "== EtherCAT slaves =="
slave_output="$(ethercat slaves -m "${master_index}")"
printf '%s\n' "${slave_output}"
echo "== Selected slave PDOs =="
ethercat pdos -m "${master_index}" -p "${slave_position}"

slave_line="$(printf '%s\n' "${slave_output}" | awk -v position="${slave_position}" '$1 == position {print; exit}')"
if [[ -z "${slave_line}" ]]; then
  echo "No slave was found at position ${slave_position}; do not start ros2_control." >&2
  exit 2
fi

verbose_output="$(ethercat slaves -m "${master_index}" -v)"
identity="$(printf '%s\n' "${verbose_output}" | awk -v position="${slave_position}" '
  /^=== Master / {
    in_slave = index($0, "Slave " position) != 0
  }
  in_slave && /Vendor I[dD]:/ { vendor = $NF }
  in_slave && /Product code:/ { product = $NF }
  END {
    if (vendor != "" && product != "") {
      print vendor, product
    }
  }')"
if [[ -z "${identity}" ]]; then
  echo "Could not parse vendor/product identity from verbose slave data." >&2
  echo "The normal 'alias:position' column is not a vendor/product ID." >&2
  exit 3
fi

read -r vendor product <<< "${identity}"
printf '\nUse these confirmed launch arguments (do not copy until wiring and PDOs are checked):\n'
printf 'slave_position:=%s slave_vendor_id:=0x%s slave_product_code:=0x%s\n' \
  "${slave_position}" "${vendor#0x}" "${product#0x}"

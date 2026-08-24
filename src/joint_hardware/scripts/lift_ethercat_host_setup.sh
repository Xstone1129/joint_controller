#!/usr/bin/env bash
set -euo pipefail

interface_name="${1:-enp5s0}"
action="${2:---check}"
master_index="${3:-2}"
config_file="/usr/local/etherlab/etc/sysconfig/ethercat"

if [[ ! "${master_index}" =~ ^[0-9]+$ ]]; then
  echo "master index must be a non-negative integer: ${master_index}" >&2
  exit 1
fi

if [[ ! -f "${config_file}" ]]; then
  echo "EtherLab configuration not found: ${config_file}" >&2
  exit 1
fi
if [[ ! -e "/sys/class/net/${interface_name}" ]]; then
  echo "EtherCAT interface does not exist: ${interface_name}" >&2
  exit 1
fi
operstate="$(<"/sys/class/net/${interface_name}/operstate")"
if [[ "${operstate}" != "up" && "${operstate}" != "unknown" ]]; then
  echo "EtherCAT interface ${interface_name} is not up (state=${operstate})" >&2
  exit 1
fi

echo "config: ${config_file}"
echo "interface: ${interface_name} (${operstate})"
echo "lift master: ${master_index}"
grep -nE '^(MASTER[0-9]+_DEVICE|DEVICE_MODULES)=' "${config_file}" || true

if [[ "${action}" != "--apply" ]]; then
  echo
  echo "Read-only check complete. To apply and restart EtherLab:"
  echo "  sudo $0 ${interface_name} --apply ${master_index}"
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "--apply must be run as root (use sudo)" >&2
  exit 1
fi

backup_file="${config_file}.bak.$(date +%Y%m%d%H%M%S)"
cp -a "${config_file}" "${backup_file}"

# EtherLab stops scanning at the first empty master slot.  A lift on Master2
# therefore requires the two arm masters to remain configured as Master0/1.
for ((index = 0; index < master_index; ++index)); do
  existing_device="$({ sed -n "s/^MASTER${index}_DEVICE=\"\(.*\)\"/\1/p" "${config_file}"; } | head -n 1)"
  if [[ -z "${existing_device}" ]]; then
    echo "MASTER${index}_DEVICE must be configured before MASTER${master_index}" >&2
    cp -a "${backup_file}" "${config_file}"
    exit 1
  fi
done

master_key="MASTER${master_index}_DEVICE"
if grep -q "^${master_key}=" "${config_file}"; then
  sed -i "s/^${master_key}=.*/${master_key}=\"${interface_name}\"/" "${config_file}"
else
  last_previous_key="MASTER$((master_index - 1))_DEVICE"
  if ((master_index == 0)); then
    sed -i "/^MASTER0_DEVICE=/i ${master_key}=\"${interface_name}\"" "${config_file}"
  else
    sed -i "/^${last_previous_key}=/a ${master_key}=\"${interface_name}\"" "${config_file}"
  fi
fi
sed -i -e 's/^DEVICE_MODULES=.*/DEVICE_MODULES="generic"/' "${config_file}"

/etc/init.d/ethercat stop >/dev/null 2>&1 || true
/etc/init.d/ethercat start
sleep 1

if [[ ! -e "/dev/EtherCAT${master_index}" ]]; then
  echo "EtherLab did not create /dev/EtherCAT${master_index}; restoring ${backup_file}" >&2
  cp -a "${backup_file}" "${config_file}"
  /etc/init.d/ethercat stop >/dev/null 2>&1 || true
  exit 2
fi

echo "backup: ${backup_file}"
echo "== EtherCAT master =="
ethercat master -m "${master_index}"
echo "== EtherCAT slaves =="
ethercat slaves -m "${master_index}"

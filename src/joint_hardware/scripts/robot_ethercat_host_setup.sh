#!/usr/bin/env bash
set -euo pipefail

action="${1:---check}"
master0_interface="${2:-enp3s0}"
master1_interface="${3:-enp4s0}"
master2_interface="${4:-enp5s0}"
init_config_file="/usr/local/etherlab/etc/sysconfig/ethercat"
systemd_config_file="/usr/local/etherlab/etc/ethercat.conf"
network_manager_config="/etc/NetworkManager/conf.d/90-robot-ethercat-unmanaged.conf"
config_files=("${init_config_file}")
ethercat_started=false

stop_ethercat_on_exit() {
  if [[ "${ethercat_started}" == "true" ]]; then
    /etc/init.d/ethercat stop >/dev/null 2>&1 || true
  fi
}
trap stop_ethercat_on_exit EXIT

if [[ ! -f "${init_config_file}" ]]; then
  echo "EtherLab init.d configuration not found: ${init_config_file}" >&2
  exit 1
fi
if [[ -f "${systemd_config_file}" ]]; then
  config_files+=("${systemd_config_file}")
else
  echo "WARNING: EtherLab systemd configuration not found: ${systemd_config_file}" >&2
fi

interfaces=("${master0_interface}" "${master1_interface}" "${master2_interface}")
if [[ "${master0_interface}" == "${master1_interface}" ||
      "${master0_interface}" == "${master2_interface}" ||
      "${master1_interface}" == "${master2_interface}" ]]; then
  echo "EtherCAT master interfaces must be distinct" >&2
  exit 1
fi

for index in 0 1 2; do
  interface_name="${interfaces[index]}"
  if [[ ! "${interface_name}" =~ ^[[:alnum:]_.:-]+$ ]]; then
    echo "Invalid Master${index} interface name: ${interface_name}" >&2
    exit 1
  fi
  if [[ ! -e "/sys/class/net/${interface_name}" ]]; then
    echo "Master${index} interface does not exist: ${interface_name}" >&2
    exit 1
  fi
  operstate="$(<"/sys/class/net/${interface_name}/operstate")"
  if [[ "${index}" -lt 2 && "${operstate}" != "up" && "${operstate}" != "unknown" ]]; then
    echo "Master${index} interface ${interface_name} is not up (state=${operstate})" >&2
    exit 1
  fi
  if [[ "${index}" -eq 2 && "${operstate}" != "up" && "${operstate}" != "unknown" ]]; then
    echo "WARNING: optional Master2 interface ${interface_name} is ${operstate}" >&2
  fi
  echo "Master${index}: ${interface_name} (${operstate})"
done

for config_file in "${config_files[@]}"; do
  echo "config: ${config_file}"
  grep -nE '^(MASTER[0-9]+_DEVICE|DEVICE_MODULES|UPDOWN_INTERFACES)=' \
    "${config_file}" || true
done

if [[ "${action}" != "--apply" ]]; then
  echo
  echo "Read-only check complete. To apply the three-master robot topology:"
  echo "  sudo $0 --apply ${master0_interface} ${master1_interface} ${master2_interface}"
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "--apply must be run as root (use sudo)" >&2
  exit 1
fi

backup_suffix="bak.$(date +%Y%m%d%H%M%S)"
backup_files=()
for config_file in "${config_files[@]}"; do
  backup_file="${config_file}.${backup_suffix}"
  cp -a "${config_file}" "${backup_file}"
  backup_files+=("${backup_file}")
done

restore_config() {
  echo "Restoring EtherLab configurations" >&2
  /etc/init.d/ethercat stop >/dev/null 2>&1 || true
  ethercat_started=false
  for index in "${!config_files[@]}"; do
    cp -a "${backup_files[index]}" "${config_files[index]}"
  done
}

set_config_value() {
  local file="$1"
  local key="$2"
  local value="$3"
  if grep -qE "^#?${key}=" "${file}"; then
    sed -i -E "s|^#?${key}=.*|${key}=\"${value}\"|" "${file}"
  else
    printf '%s="%s"\n' "${key}" "${value}" >> "${file}"
  fi
}

for config_file in "${config_files[@]}"; do
  set_config_value "${config_file}" MASTER0_DEVICE "${master0_interface}"
  set_config_value "${config_file}" MASTER1_DEVICE "${master1_interface}"
  set_config_value "${config_file}" MASTER2_DEVICE "${master2_interface}"
  set_config_value "${config_file}" DEVICE_MODULES generic
  set_config_value "${config_file}" UPDOWN_INTERFACES \
    "${master0_interface} ${master1_interface} ${master2_interface}"
done

mkdir -p "$(dirname "${network_manager_config}")"
cat > "${network_manager_config}" <<EOF
[keyfile]
unmanaged-devices=interface-name:${master0_interface};interface-name:${master1_interface};interface-name:${master2_interface}
EOF
if command -v nmcli >/dev/null 2>&1; then
  for interface_name in "${interfaces[@]}"; do
    nmcli device set "${interface_name}" managed no || true
  done
fi

/etc/init.d/ethercat stop >/dev/null 2>&1 || true
if ! /etc/init.d/ethercat start; then
  restore_config
  exit 2
fi
ethercat_started=true
sleep 1

for index in 0 1 2; do
  if [[ ! -e "/dev/EtherCAT${index}" ]]; then
    echo "EtherLab did not create /dev/EtherCAT${index}" >&2
    restore_config
    exit 2
  fi
done

printf 'backup: %s\n' "${backup_files[@]}"
echo "NetworkManager unmanaged configuration: ${network_manager_config}"
echo "== EtherCAT masters =="
ethercat master
echo "== EtherCAT slaves =="
ethercat slaves
/etc/init.d/ethercat stop
ethercat_started=false

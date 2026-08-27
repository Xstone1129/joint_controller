#!/usr/bin/env bash
set -Eeuo pipefail

root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cli="${root_dir}/tools/lift_ethercat_cli"
ethercat_init=/etc/init.d/ethercat
ethercat_config=/usr/local/etherlab/etc/ethercat.conf
master_index=2
lift_min_position_m=-1.0
lift_max_position_m=1.0

if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 root 运行：sudo $0" >&2
  exit 1
fi
if [[ ! -x "${cli}" ]]; then
  echo "未找到 CLI，请先运行：${root_dir}/tools/build_lift_ethercat_cli.sh" >&2
  exit 1
fi
if [[ ! -x "${ethercat_init}" ]]; then
  echo "EtherLab init 脚本不存在：${ethercat_init}" >&2
  exit 1
fi

master_device="$(sed -n -E 's/^MASTER2_DEVICE="?([^"#]+)"?.*/\1/p' "${ethercat_config}" | head -n 1 | xargs)"
if [[ -z "${master_device}" ]]; then
  echo "未配置 MASTER2_DEVICE：${ethercat_config}" >&2
  exit 1
fi
if [[ ! -e "/sys/class/net/${master_device}" ]]; then
  echo "Master2 网口不存在：${master_device}" >&2
  exit 1
fi

# EtherLab's init script performs the actual NIC-to-master binding according
# to MASTER0/1/2_DEVICE. Bring the configured lift NIC up beforehand.
ip link set "${master_device}" up
if [[ ! -e "/dev/EtherCAT${master_index}" ]]; then
  echo "启动 EtherLab，绑定 Master${master_index} <- ${master_device} ..."
  "${ethercat_init}" start
fi

for _ in $(seq 1 50); do
  [[ -e "/dev/EtherCAT${master_index}" ]] && break
  sleep 0.1
done
if [[ ! -e "/dev/EtherCAT${master_index}" ]]; then
  echo "EtherLab 启动后仍没有 /dev/EtherCAT${master_index}" >&2
  exit 1
fi

verbose_slaves="$(timeout 5 ethercat slaves -m "${master_index}" -v)" || {
  echo "无法读取 Master${master_index} 从站信息：ethercat slaves 命令超时或失败。" >&2
  exit 1
}
identity="$(printf '%s\n' "${verbose_slaves}" | awk '
  /^=== Master / { in_slave = index($0, "Slave 0") != 0 }
  in_slave && /Vendor I[dD]:/ { vendor = $NF }
  in_slave && /Product code:/ { product = $NF }
  END { if (vendor != "" && product != "") print vendor, product }
')"
if [[ -z "${identity}" ]]; then
  echo "无法从 Master${master_index} Slave0 输出解析 Vendor/Product：" >&2
  printf '%s\n' "${verbose_slaves}" >&2
  exit 1
fi
read -r vendor_id product_code <<< "${identity}"

zero_owner="${SUDO_USER:-${USER}}"
zero_home="$(getent passwd "${zero_owner}" | awk -F: 'NR == 1 { print $6 }')"
if [[ -z "${zero_home}" ]]; then
  zero_home="${HOME}"
fi
zero_owner_uid="$(id -u "${zero_owner}")"
zero_owner_gid="$(id -g "${zero_owner}")"
zero_offset_file="${zero_home}/.local/state/joint_controller/lift_zero_offset.cfg"

echo "Master${master_index} 已绑定 ${master_device}；lift Slave0: vendor=${vendor_id}, product=${product_code}。"
echo "共享 ROS 零偏文件：${zero_offset_file}"
exec "${cli}" --master "${master_index}" --alias 0 --position 0 \
  --vendor "${vendor_id}" --product "${product_code}" \
  --min-position "${lift_min_position_m}" \
  --max-position "${lift_max_position_m}" \
  --zero-offset-file "${zero_offset_file}" \
  --zero-offset-uid "${zero_owner_uid}" \
  --zero-offset-gid "${zero_owner_gid}" "$@"

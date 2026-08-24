#!/usr/bin/env bash
set -Eeuo pipefail

root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out_file="${root_dir}/tools/lift_ethercat_cli"
include_dir="${root_dir}/src/joint_hardware/include"
source_dir="${root_dir}/src/joint_hardware/src/lift"

command -v g++ >/dev/null || { echo "g++ is required" >&2; exit 1; }
[[ -f /usr/local/etherlab/include/ecrt.h ]] || {
  echo "EtherLab headers not found: /usr/local/etherlab/include/ecrt.h" >&2
  exit 1
}

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -DJOINT_HARDWARE_HAS_ETHERLAB=1 \
  -I"${include_dir}" -I/usr/local/etherlab/include \
  "${root_dir}/tools/lift_ethercat_cli.cpp" \
  "${source_dir}/ethercat_backend.cpp" \
  "${source_dir}/ethercat_sdk_adapter.cpp" \
  "${source_dir}/etherlab_backend.cpp" \
  "${source_dir}/pdo_mapping.cpp" \
  "${source_dir}/cia402.cpp" \
  "${source_dir}/lift_units.cpp" \
  -L/usr/local/etherlab/lib -Wl,-rpath,/usr/local/etherlab/lib \
  -lethercat -pthread -o "${out_file}"

echo "Built ${out_file}"

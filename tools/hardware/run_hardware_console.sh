#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
export JUNIOR_RUNTIME_LOG_USER="${JUNIOR_RUNTIME_LOG_USER:-${SUDO_USER:-user}}"
source "${WORKSPACE_ROOT}/scripts/runtime_log_env.sh"
exec > >(tee -a "${SESSION_LOG_DIR}/console.log") 2>&1
TEST_TOOL="${SCRIPT_DIR}/ethercat_hardware_test.py"
CONFIG="${SCRIPT_DIR}/hardware_io.yaml"
ETHER_CAT_STARTED=0

usage() {
  echo "用法: sudo $0 [--check|--check-driver|--self-test|arm|lift]"
  echo "  无参数: 配置 Master0/1/2 后进入选择菜单"
  echo "  arm:    启动旧 IGH arm driver，交互测试 14 个 arm drive 的使能和点动"
  echo "  lift:   启动现有 lift EtherCAT CSV CLI"
  echo "  --check:只读检查，不修改网卡或 EtherLab 配置"
  echo "  --check-driver:只检查 IGH driver 是否已运行"
  echo "  --self-test:只运行离线配置和 ABI 检查"
}

  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "--self-test" ]]; then
  exec python3 "${TEST_TOOL}" --config "${CONFIG}" self-test
fi
if [[ "${1:-}" == "--check" ]]; then
  exec python3 "${TEST_TOOL}" --config "${CONFIG}" check
fi
if [[ "${1:-}" == "--check-driver" ]]; then
  exec python3 "${TEST_TOOL}" --config "${CONFIG}" check-driver
fi
if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 root 运行：sudo $0" >&2
  exit 1
fi

cleanup() {
  if [[ "${ETHER_CAT_STARTED}" -eq 0 ]]; then
    return
  fi
  ETHER_CAT_STARTED=0
  local init_script
  init_script=$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["ethercat"]["init_script"])' "${CONFIG}")
  "${init_script}" stop >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

python3 "${TEST_TOOL}" --config "${CONFIG}" check-driver
python3 "${TEST_TOOL}" --config "${CONFIG}" configure
ETHER_CAT_STARTED=1

choice=${1:-}
if [[ -z "${choice}" ]]; then
  if [[ ! -t 0 ]]; then
    usage >&2
    exit 2
  fi
  printf '%s\n' \
    "请选择要测试的 EtherCAT 硬件：" \
    "  1) Arm   左右手臂（Master0 + Master1）" \
    "  2) Lift  升降（Master2）" \
    "  q) 退出"
  read -r -p "选择: " choice
fi
if [[ $# -gt 0 ]]; then
  shift
fi

case "${choice,,}" in
  1|arm|arms)
    if python3 "${TEST_TOOL}" --config "${CONFIG}" arm; then
      exit 0
    else
      exit $?
    fi
    ;;
  2|lift)
    if python3 "${TEST_TOOL}" --config "${CONFIG}" lift; then
      exit 0
    else
      exit $?
    fi
    ;;
  q|quit|exit)
    exit 0
    ;;
  *)
    echo "未知选择: ${choice}" >&2
    usage >&2
    exit 2
    ;;
esac

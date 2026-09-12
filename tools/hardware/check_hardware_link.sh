#!/usr/bin/env bash
# =============================================================================
# Lower computer Heavy V1 hardware-link read-only preflight.
#
# Scope is intentionally limited to the 14 arm drives and one lift drive:
#   Master0 / enp3s0: left arm   (7 slaves)
#   Master1 / enp4s0: right arm  (7 slaves)
#   Master2 / enp5s0: lift       (1 slave)
#
# By default this is a complete, bounded discovery check: after confirming no
# controller owns EtherCAT, it temporarily brings the three dedicated NICs up,
# starts EtherLab, waits for the three masters and 7/7/1 slaves, reads the lift
# identity/PDO, then stops EtherLab and restores the original NIC state. It
# never enables drives or sends arm/lift commands. Use --read-only to skip the
# EtherLab start/stop phase.
#
# Usage:
#   sudo ./tools/hardware/check_hardware_link.sh
#   sudo ./tools/hardware/check_hardware_link.sh --bring-up
#   ./tools/hardware/check_hardware_link.sh --report /tmp/heavy_lower.md
#   ./tools/hardware/check_hardware_link.sh --config tools/hardware/hardware_io.yaml
#
# `--bring-up` is retained as an alias for the complete default discovery mode.
# `--read-only` skips the temporary EtherLab start/stop phase and only inspects
# the current state. The complete mode requires root (or passwordless sudo).
#
# Non-root callers use `sudo -n` for EtherCAT queries. If that permission is
# unavailable, the script reports the enumeration as unchecked and never asks
# for or stores a password.
# =============================================================================

set -u

ORIGINAL_ARGS=("$@")
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
CONFIG="${JUNIOR_HARDWARE_CONFIG:-${SCRIPT_DIR}/hardware_io.yaml}"
REPORT="${JUNIOR_HARDWARE_REPORT:-${SCRIPT_DIR}/reports/check_hardware_link_report.md}"
COMMAND_TIMEOUT="${JUNIOR_HARDWARE_CHECK_TIMEOUT:-12}"
ETHER_CAT_BIN="${ETHER_CAT_BIN:-/usr/bin/ethercat}"
BRING_UP=false
READ_ONLY=false
ATTACHED_EXISTING=false
ETHER_CAT_STARTED=false
ETHER_CAT_INIT="/etc/init.d/ethercat"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bring-up)
      BRING_UP=true
      shift
      ;;
    --read-only)
      READ_ONLY=true
      shift
      ;;
    --report)
      [[ $# -ge 2 ]] || { echo "--report requires a path" >&2; exit 2; }
      REPORT="$2"
      shift 2
      ;;
    --config)
      [[ $# -ge 2 ]] || { echo "--config requires a path" >&2; exit 2; }
      CONFIG="$2"
      shift 2
      ;;
    --help|-h)
      sed -n '1,33p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! "$COMMAND_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk "BEGIN {exit !(${COMMAND_TIMEOUT} > 0)}" 2>/dev/null; then
  echo "invalid JUNIOR_HARDWARE_CHECK_TIMEOUT: $COMMAND_TIMEOUT" >&2
  exit 2
fi

# The complete check needs CAP_NET_ADMIN and access to /dev/EtherCAT*. Keep
# the convenient `./check_hardware_link.sh` invocation interactive by elevating
# once here; --read-only remains usable without elevation (it reports any
# unavailable EtherCAT queries as unchecked).
if [[ "$READ_ONLY" != true && "$(id -u)" -ne 0 ]]; then
  exec sudo -- "${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")" "${ORIGINAL_ARGS[@]}"
fi

mkdir -p "$(dirname -- "$REPORT")"
now() { date '+%Y-%m-%d %H:%M:%S'; }
compact() {
  local value=${1:-}
  value=${value//$'\n'/ }
  value=${value//$'\r'/ }
  value=${value//|/\\|}
  printf '%s' "$value" | sed -E 's/[[:space:]]+/ /g; s/^ //; s/ $//'
}

ok=0
warn=0
bad=0
skip=0
mark_ok() { ok=$((ok + 1)); printf '%s\n' "$1" >> "$REPORT"; }
mark_warn() { warn=$((warn + 1)); printf '%s\n' "$1" >> "$REPORT"; }
mark_bad() { bad=$((bad + 1)); printf '%s\n' "$1" >> "$REPORT"; }
mark_skip() { skip=$((skip + 1)); printf '%s\n' "$1" >> "$REPORT"; }

if [[ "$(id -u)" -eq 0 ]]; then
  ROOT_PREFIX=()
else
  ROOT_PREFIX=(sudo -n)
fi
run_ro() {
  timeout "$COMMAND_TIMEOUT" "${ROOT_PREFIX[@]}" "$@"
}

ethercat_resources_active() {
  systemctl is-active --quiet ethercat.service 2>/dev/null ||
    pgrep -x igh_driver >/dev/null 2>&1 ||
    [[ -e /dev/EtherCAT0 || -e /dev/EtherCAT1 || -e /dev/EtherCAT2 ]]
}

# A running controller already owns EtherLab. Inspect its masters in place
# instead of restarting the service or changing NIC state underneath it.
if [[ "$READ_ONLY" != true ]] && ethercat_resources_active; then
  READ_ONLY=true
  ATTACHED_EXISTING=true
fi

declare -A ORIGINAL_ADMIN_STATE=()
LINKS_RESTORED=false

admin_state() {
  local iface=$1 detail flags
  detail="$(ip -o link show dev "$iface" 2>&1 || true)"
  [[ -n "$detail" && "$detail" != *"does not exist"* &&
     "$detail" != *"Cannot find device"* ]] || { echo missing; return; }
  flags="${detail#*<}"
  flags="${flags%%>*}"
  case ",$flags," in
    *,UP,*) echo up ;;
    *) echo down ;;
  esac
}

restore_links() {
  [[ "$BRING_UP" == true && "$LINKS_RESTORED" != true ]] || return 0
  local iface state restore_rc=0
  for iface in enp3s0 enp4s0 enp5s0; do
    state="${ORIGINAL_ADMIN_STATE[$iface]:-missing}"
    case "$state" in
      down)
        if ! run_ro ip link set dev "$iface" down >/dev/null 2>&1; then
          restore_rc=1
        fi
        ;;
      up)
        if ! run_ro ip link set dev "$iface" up >/dev/null 2>&1; then
          restore_rc=1
        fi
        ;;
    esac
  done
  LINKS_RESTORED=true
  if [[ -f "$REPORT" ]]; then
    if [[ $restore_rc -eq 0 ]]; then
      printf '\n%s\n' "- 临时网卡状态已恢复到进入脚本前的管理状态。" >> "$REPORT"
    else
      printf '\n%s\n' "- ⚠️ 临时网卡状态恢复失败；请手工核对 enp3s0/enp4s0/enp5s0。" >> "$REPORT"
    fi
  fi
  return "$restore_rc"
}

wait_for_devices() {
  local deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    if [[ -e /dev/EtherCAT0 && -e /dev/EtherCAT1 && -e /dev/EtherCAT2 ]]; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_slaves() {
  local deadline=$((SECONDS + 15)) master output link slaves all expected
  while (( SECONDS < deadline )); do
    all=true
    for master in 0 1 2; do
      case "$master" in 0|1) expected=7 ;; 2) expected=1 ;; esac
      output="$(run_ro "$ETHER_CAT_BIN" master -m "$master" 2>&1)"
      link="$(sed -n 's/^[[:space:]]*Link:[[:space:]]*//p' <<< "$output" | head -n 1)"
      slaves="$(sed -n 's/^[[:space:]]*Slaves:[[:space:]]*//p' <<< "$output" | head -n 1)"
      if [[ "$link" != "UP" || "$slaves" != "$expected" ]]; then
        all=false
        break
      fi
    done
    [[ "$all" == true ]] && return 0
    sleep 0.25
  done
  return 1
}

stop_ethercat() {
  [[ "$ETHER_CAT_STARTED" == true ]] || return 0
  local rc=0
  if [[ -x "$ETHER_CAT_INIT" ]]; then
    if ! run_ro "$ETHER_CAT_INIT" stop >/dev/null 2>&1; then
      rc=1
    fi
  else
    rc=1
  fi
  ETHER_CAT_STARTED=false
  if [[ -f "$REPORT" ]]; then
    if [[ $rc -eq 0 ]]; then
      printf '\n%s\n' "- 临时 EtherLab 主站已停止。" >> "$REPORT"
    else
      printf '\n%s\n' "- ⚠️ 临时 EtherLab 主站停止失败；请手工核对 EtherCAT 状态。" >> "$REPORT"
    fi
  fi
  return "$rc"
}

on_exit() {
  local rc=$?
  stop_ethercat || true
  restore_links || true
  trap - EXIT INT TERM
  exit "$rc"
}

{
  echo "# Heavy V1 下位机双臂/升降硬件链路自检报告"
  echo
  echo "- **检测时间**: $(now)"
  echo "- **检测对象**: Master0/1 双臂 14 轴 + Master2 升降 1 轴"
  if [[ "$ATTACHED_EXISTING" == true ]]; then
    echo "- **检测方式**: 附着到当前正在运行的 EtherCAT 主站并只读查询；不修改网卡、不启停 EtherLab、无运动命令"
  elif [[ "$READ_ONLY" == true ]]; then
    echo "- **检测方式**: 下位机本地系统信息与 EtherCAT 只读查询；无 ROS、无运动命令"
  else
    echo "- **检测方式**: 临时拉起专用网卡并启动 EtherLab 后只读枚举；无 ROS、无运动命令"
  fi
  echo "- **配置文件**: \`$(compact "$CONFIG")\`"
  echo
} > "$REPORT"

echo "===== Heavy V1 下位机双臂/升降只读预检 ====="
echo "报告: $REPORT"
if [[ "$ATTACHED_EXISTING" == true ]]; then
  echo "检测到现有 EtherCAT/控制器运行栈，自动切换为只读附着检查（不会启停主站）。"
fi

trap on_exit EXIT
trap 'exit 130' INT TERM

prepare_temporary_links() {
  [[ "$READ_ONLY" == true ]] && return 0
  BRING_UP=true
  {
    echo "## 临时网卡配置"
    echo
    echo "- 完整模式：仅临时设置三条 EtherCAT 网卡为管理状态 UP。"
    echo "- 不配置 IP、不加入 bridge；退出时停止临时 EtherLab 并恢复原管理状态。"
    echo
    echo "| 网卡 | 进入前管理状态 | 临时操作 | 判定 |"
    echo "|---|---|---|---|"
    echo
  } >> "$REPORT"

  if [[ "$(id -u)" -ne 0 ]] && ! run_ro true >/dev/null 2>&1; then
    mark_bad "| sudo 权限 | root 或 sudo -n | 无法执行临时网卡操作 | ❌ |"
    return 1
  fi
  if ethercat_resources_active; then
    mark_bad "| 资源占用 | EtherLab/igh_driver/主站设备均未运行 | 为避免抢占现有控制器，拒绝临时检测 | ❌ |"
    return 1
  fi

  local iface state
  for iface in enp3s0 enp4s0 enp5s0; do
    state="$(admin_state "$iface")"
    ORIGINAL_ADMIN_STATE[$iface]="$state"
    case "$state" in
      missing)
        mark_bad "| \`$iface\` | 存在 | 网卡不存在 | ❌ |"
        return 1
        ;;
      up)
        mark_ok "| \`$iface\` | UP | 已是 UP，保持不变 | ✅ |"
        ;;
      down)
        if run_ro ip link set dev "$iface" up >/dev/null 2>&1; then
          mark_ok "| \`$iface\` | DOWN | 已临时设置 UP | ✅ |"
        else
          mark_bad "| \`$iface\` | DOWN | 设置 UP 失败 | ❌ |"
          return 1
        fi
        ;;
      *)
        mark_bad "| \`$iface\` | $state | 无法识别管理状态 | ❌ |"
        return 1
        ;;
    esac
  done
  LINKS_PREPARED=true
}

if ! prepare_temporary_links; then
  echo "临时网卡配置失败，未执行后续检查。" >&2
  exit 1
fi

start_temporary_ethercat() {
  [[ "$READ_ONLY" == true ]] && return 0
  if [[ ! -x "$ETHER_CAT_INIT" ]]; then
    mark_bad "| EtherLab 启动脚本 | \`$ETHER_CAT_INIT\` | 不存在或不可执行 | ❌ |"
    return 1
  fi
  local start_output
  # Mark the start as attempted before invoking init.d. If it partially
  # creates a master and then fails, the EXIT trap still tears it down.
  ETHER_CAT_STARTED=true
  start_output="$(run_ro "$ETHER_CAT_INIT" start 2>&1)"
  if [[ $? -eq 0 ]]; then
    mark_ok "| EtherLab 主站 | 三主站启动 | \`$ETHER_CAT_INIT start\` 成功 | ✅ |"
    if ! wait_for_devices; then
      mark_warn "| EtherCAT 设备节点 | /dev/EtherCAT0..2 | 启动后 15 秒内未全部出现 | ⚠️ 未完成 |"
    fi
    if [[ -e /dev/EtherCAT0 && -e /dev/EtherCAT1 && -e /dev/EtherCAT2 ]]; then
      if wait_for_slaves; then
        mark_ok "| EtherCAT 从站发现 | Master0/1/2 | 7/7/1 在等待窗口内达到 | ✅ |"
      else
        mark_warn "| EtherCAT 从站发现 | Master0/1/2 | 15 秒内未达到 7/7/1；后续表格给出各主站实际值 | ⚠️ 未完成 |"
      fi
    fi
    return 0
  fi
  mark_bad "| EtherLab 主站 | 三主站启动 | $(compact "$start_output") | ❌ |"
  return 1
}

if ! start_temporary_ethercat; then
  echo "EtherLab 启动失败，未执行后续枚举检查。" >&2
  exit 1
fi

{
  echo "## 1. 配置映射"
  echo
  echo "| 资源 | Master | 网卡 | 期望从站 | 判定 |"
  echo "|---|---:|---|---:|---|"
} >> "$REPORT"

# Validate the lower workspace's own hardware_io.yaml instead of duplicating
# its mapping in two places. The fixed expected values guard against silently
# accepting a config copied from another robot.
config_rows="$(python3 - "$CONFIG" 2>&1 <<'PY'
import sys
try:
    import yaml
except ImportError as exc:
    print(f"PyYAML unavailable: {exc}")
    raise SystemExit(2)

path = sys.argv[1]
try:
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
except Exception as exc:
    print(f"cannot read config: {exc}")
    raise SystemExit(3)

masters = ((data.get("ethercat") or {}).get("masters") or {})
required = (
    ("left_arm", "左臂", 0, "enp3s0", 7),
    ("right_arm", "右臂", 1, "enp4s0", 7),
    ("lift", "升降", 2, "enp5s0", 1),
)
for key, label, index, interface, expected in required:
    item = masters.get(key)
    if not isinstance(item, dict):
        print(f"missing ethercat.masters.{key}")
        raise SystemExit(4)
    actual = (int(item.get("index", -1)), str(item.get("interface", "")),
              int(item.get("expected_slaves", -1)))
    if actual != (index, interface, expected):
        print(f"{key}: expected Master{index}/{interface}/{expected}, got {actual}")
        raise SystemExit(5)
    print(f"{key}|{label}|{index}|{interface}|{expected}")
PY
)"
config_rc=$?
if [[ $config_rc -ne 0 ]]; then
  mark_bad "| Heavy V1 EtherCAT mapping | Master0/1/2 | enp3s0/enp4s0/enp5s0 | 7/7/1 | $(compact "$config_rows") | ❌ 配置无效 |"
  config_valid=false
else
  config_valid=true
  while IFS='|' read -r key label master iface expected; do
    [[ -n "${key:-}" ]] || continue
    mark_ok "| $label | Master$master | \`$iface\` | $expected | ✅ 配置匹配 |"
  done <<< "$config_rows"
fi

if [[ "$config_valid" != true ]]; then
  mark_skip "| EtherCAT 物理/主站检查 | 需要有效配置 | 未执行 | ⏭️ |"
  mark_skip "| 升降身份/PDO 检查 | 需要有效配置 | 未执行 | ⏭️ |"
else
  {
    echo
    echo "## 2. EtherCAT 物理网卡"
    echo
    echo "| Master | 功能 | 网卡 | 链路 | IP/bridge 约束 | 判定 |"
    echo "|---:|---|---|---|---|---|"
  } >> "$REPORT"

  MASTER_OK=(false false false)
  for master in 0 1 2; do
    case "$master" in
      0) label="左臂"; iface="enp3s0" ;;
      1) label="右臂"; iface="enp4s0" ;;
      2) label="升降"; iface="enp5s0" ;;
    esac
    link_info="$(ip -br link show "$iface" 2>&1 || true)"
    link_detail="$(ip -o link show dev "$iface" 2>&1 || true)"
    addr_info="$(ip -o addr show dev "$iface" 2>&1 || true)"
    if [[ "$link_info" == *"does not exist"* || "$link_info" == *"Cannot find device"* ]]; then
      state="不存在"
    else
      state="$(awk 'NR == 1 {print $2}' <<< "$link_info")"
    fi
    [[ -n "$state" ]] || state="不存在"

    violation=""
    while read -r family address; do
      [[ -n "${family:-}" ]] || continue
      case "$family" in
        inet) violation+="IPv4 ${address}; " ;;
        inet6) [[ "$address" == fe80:* ]] || violation+="IPv6 ${address}; " ;;
      esac
    done < <(awk '{print $3, $4}' <<< "$addr_info")
    grep -qE '(^|[[:space:]])master[[:space:]]' <<< "$link_detail" &&
      violation+="已加入 bridge; "
    constraint="无普通 IP/无 bridge"
    [[ -n "$violation" ]] && constraint="$(compact "$violation")"

    if [[ "$state" == "UP" && -z "$violation" ]]; then
      mark_ok "| Master$master | $label | \`$iface\` | UP | $constraint | ✅ |"
    elif [[ "$state" == "UNKNOWN" && -z "$violation" ]]; then
      mark_warn "| Master$master | $label | \`$iface\` | UNKNOWN | $constraint | ⚠️ operstate 未知 |"
    elif [[ -n "$violation" ]]; then
      mark_bad "| Master$master | $label | \`$iface\` | ${state:-?} | \`$constraint\` | ❌ EtherCAT 口配置异常 |"
    else
      mark_bad "| Master$master | $label | \`$iface\` | ${state:-?} | $constraint | ❌ 物理链路未起来 |"
    fi
  done

  {
    echo
    echo "## 3. EtherCAT 主站、从站与设备节点"
    echo
    echo "| Master | 功能 | 设备节点 | 期望从站 | 实际 | 判定 |"
    echo "|---:|---|---|---:|---|---|"
  } >> "$REPORT"

  for master in 0 1 2; do
    case "$master" in
      0) label="左臂"; expected=7 ;;
      1) label="右臂"; expected=7 ;;
      2) label="升降"; expected=1 ;;
    esac
    device="/dev/EtherCAT${master}"
    device_info="$(if [[ -e "$device" ]]; then stat -c '%A %U:%G' "$device"; else echo MISSING; fi 2>&1)"
    if [[ "$device_info" == *MISSING* ]]; then
      mark_bad "| Master$master | $label | \`$device\` | $expected | 不存在 | ❌ 设备节点缺失 |"
      continue
    fi

    master_output="$(run_ro "$ETHER_CAT_BIN" master -m "$master" 2>&1)"
    master_rc=$?
    if [[ $master_rc -ne 0 ]]; then
      if grep -qiE 'sudo:|password is required|需要密码|not allowed' <<< "$master_output"; then
        mark_warn "| Master$master | $label | \`$device\` | $expected | $(compact "$master_output") | ⚠️ 未检查（sudo 无免密只读权限） |"
      else
        mark_bad "| Master$master | $label | \`$device\` | $expected | $(compact "$master_output") | ❌ 主站查询失败 |"
      fi
      continue
    fi

    link="$(sed -n 's/^[[:space:]]*Link:[[:space:]]*//p' <<< "$master_output" | head -n 1)"
    slaves="$(sed -n 's/^[[:space:]]*Slaves:[[:space:]]*//p' <<< "$master_output" | head -n 1)"
    [[ -n "$link" ]] || link="-"
    [[ -n "$slaves" ]] || slaves="-"
    if [[ "$link" == "UP" && "$slaves" == "$expected" ]]; then
      mark_ok "| Master$master | $label | \`$device\` | $expected | link=$link, slaves=$slaves | ✅ |"
      MASTER_OK[$master]=true
    elif [[ "$link" != "UP" ]]; then
      mark_bad "| Master$master | $label | \`$device\` | $expected | link=$link, slaves=$slaves | ❌ 主站链路未起来 |"
    else
      mark_bad "| Master$master | $label | \`$device\` | $expected | link=$link, slaves=$slaves | ❌ 从站数量不符 |"
    fi

    slave_output="$(run_ro "$ETHER_CAT_BIN" slaves -m "$master" 2>&1)"
    slave_rc=$?
    if [[ $slave_rc -eq 0 ]]; then
      slave_count="$(grep -cE '^[[:space:]]*[0-9]+[[:space:]]+[0-9]+:[0-9]+[[:space:]]' <<< "$slave_output" || true)"
      unknown_count="$(grep -cE '0x00000000:0x00000000|[[:space:]][?][?][?][[:space:]]' <<< "$slave_output" || true)"
      {
        echo
        echo "<details><summary>Master$master 从站只读快照（${slave_count:-0} 行）</summary>"
        echo
        echo '```text'
        echo "$slave_output"
        echo '```'
        echo
        echo '</details>'
      } >> "$REPORT"
      if [[ "$unknown_count" -gt 0 ]]; then
        mark_bad "| Master$master 从站身份 | 非零 vendor/product、可识别状态 | $unknown_count 个未知身份/状态行 | ❌ |"
      else
        mark_ok "| Master$master 从站身份 | 非零 vendor/product、可识别状态 | $slave_count 行均有身份 | ✅ |"
      fi
    else
      mark_warn "| Master$master 从站快照 | 只读查询 | - | - | $(compact "$slave_output") | ⚠️ 未检查 |"
    fi
  done

  {
    echo
    echo "## 4. 升降从站身份与 PDO（只读）"
    echo
    echo "| 项目 | 期望 | 实际 | 判定 |"
    echo "|---|---|---|---|"
  } >> "$REPORT"
  lift_preflight="${LIFT_ETHERCAT_PREFLIGHT:-${WORKSPACE_ROOT}/install/joint_hardware/lib/joint_hardware/lift_ethercat_preflight.sh}"
  if [[ ! -x "$lift_preflight" ]]; then
    lift_preflight="${WORKSPACE_ROOT}/src/joint_hardware/scripts/lift_ethercat_preflight.sh"
  fi
  if [[ "${MASTER_OK[2]}" != true ]]; then
    mark_skip "| Master2 升降身份/PDO | Master2 先通过主站枚举 | 未执行 | ⏭️ |"
  elif [[ ! -x "$lift_preflight" ]]; then
    mark_warn "| Master2 升降身份/PDO | 使用 lift_ethercat_preflight.sh | 脚本不存在 | ⚠️ 未检查 |"
  else
    lift_output="$(run_ro "$lift_preflight" 2 0 enp5s0 2>&1)"
    lift_rc=$?
    lift_vendor="$(grep -oE 'slave_vendor_id:=0x[0-9a-fA-F]+' <<< "$lift_output" | tail -n 1 | sed 's/.*:=//')"
    lift_product="$(grep -oE 'slave_product_code:=0x[0-9a-fA-F]+' <<< "$lift_output" | tail -n 1 | sed 's/.*:=//')"
    lift_vendor_num=0
    lift_product_num=0
    [[ -n "$lift_vendor" ]] && lift_vendor_num=$((lift_vendor))
    [[ -n "$lift_product" ]] && lift_product_num=$((lift_product))
    lift_pdo_lines="$(awk '
      /== Selected slave PDOs ==/ {seen=1; next}
      seen && /^Use these confirmed/ {exit}
      seen && /[^[:space:]]/ {count++}
      END {print count + 0}
    ' <<< "$lift_output")"
    if [[ $lift_rc -eq 0 && $lift_vendor_num -ne 0 &&
          $lift_product_num -ne 0 && "$lift_pdo_lines" -gt 0 ]]; then
      mark_ok "| Master2 升降身份/PDO | 非零 vendor/product + PDO 可读 | vendor=$lift_vendor product=$lift_product, PDO 行=$lift_pdo_lines | ✅ |"
    elif grep -qiE 'sudo:|password is required|需要密码|not allowed' <<< "$lift_output"; then
      mark_warn "| Master2 升降身份/PDO | 只读 preflight | $(compact "$lift_output") | ⚠️ 未检查（sudo 无免密只读权限） |"
    else
      mark_bad "| Master2 升降身份/PDO | 位置0 + vendor/product + PDO 可读 | $(compact "$lift_output") | ❌ |"
    fi
    {
      echo
      echo "<details><summary>Master2 升降身份/PDO 原始只读输出</summary>"
      echo
      echo '```text'
      echo "$lift_output"
      echo '```'
      echo
      echo '</details>'
    } >> "$REPORT"
  fi
fi

if [[ "$BRING_UP" == true ]]; then
  stop_ethercat || true
  restore_links || true
fi

{
  echo
  echo "## 5. 诊断边界"
  echo
  echo "- 只检查双臂 14 轴与升降 1 轴；不检查底盘、腰部、夹爪、相机、雷达或其它串口。"
  if [[ "$ATTACHED_EXISTING" == true ]]; then
    echo "- 检测附着到进入脚本前已存在的 EtherCAT 主站；不会修改网卡、启停 EtherLab、rescan、写 PDO、上电或使能。"
  elif [[ "$READ_ONLY" == true ]]; then
    echo "- 只读调用 Linux \`ip/stat\`、EtherLab \`master/slaves/pdos\` 和升降 preflight；不会启停 \`/etc/init.d/ethercat\`、rescan、写 PDO、上电或使能。"
  else
    echo "- 仅临时启停 \`/etc/init.d/ethercat\` 以完成枚举；不会 rescan、写 PDO、上电或使能。退出时会停止主站并恢复网卡管理状态。"
  fi
  echo "- 三主站物理链路和从站数量正确，只代表 EtherCAT 枚举层通过，不等于驱动 OP 或机器人允许运动。"
  echo
  echo "## 6. 汇总"
  echo
  echo "- ✅ 通过: $ok"
  echo "- ⚠️ 警告/未检查: $warn"
  echo "- ❌ 失败: $bad"
  echo "- ⏭️ 跳过: $skip"
  if [[ "$bad" -eq 0 && "$warn" -eq 0 ]]; then
    echo "- **总判定**: ✅ Heavy V1 下位机双臂/升降链路只读预检通过"
  elif [[ "$bad" -eq 0 ]]; then
    echo "- **总判定**: ⚠️ 存在未检查项目，请补充 sudo 权限或现场状态"
  else
    echo "- **总判定**: ❌ 存在链路异常，请按报告排查"
  fi
} >> "$REPORT"

cat "$REPORT"
if [[ "$bad" -gt 0 ]]; then
  exit 1
fi
if [[ "$warn" -gt 0 ]]; then
  exit 2
fi
exit 0

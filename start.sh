#!/bin/bash
set -Eeuo pipefail

WORKSPACE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS
SIM="${SIM:-0.0}"
STACK_LABEL="joint-controller-stack-real"
if awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    STACK_LABEL="joint-controller-stack-sim"
fi
USE_RVIZ=${USE_RVIZ:-true}
LIFT_ETHERCAT_INTERFACE=${LIFT_ETHERCAT_INTERFACE:-enp5s0}
LIFT_BRAKE_CONTROL_ENABLED=${LIFT_BRAKE_CONTROL_ENABLED:-false}
LIFT_BRAKE_RELEASE_WAIT_MS=${LIFT_BRAKE_RELEASE_WAIT_MS:--1}
if [ -z "${ENABLE_EFFORT_MODE_SWITCH+x}" ]; then
    if awk "BEGIN {exit !(${SIM} > 0.0)}"; then
        ENABLE_EFFORT_MODE_SWITCH=true
    else
        ENABLE_EFFORT_MODE_SWITCH=false
    fi
fi
RUN_USER=${SUDO_USER:-$(whoami)}
RUN_HOME=$(getent passwd "${RUN_USER}" | cut -d: -f6)
LIFT_ZERO_OFFSET_FILE=${LIFT_ZERO_OFFSET_FILE:-${RUN_HOME}/.local/state/joint_controller/lift_zero_offset.cfg}
LOG_ROOT="${WORKSPACE_ROOT}/log/runtime"
timestamp=$(date '+%Y-%m-%d-%H-%M-%S')
SESSION_LOG_DIR="${LOG_ROOT}/${timestamp}-${STACK_LABEL}"
RUNTIME_LOG_DIR="${SESSION_LOG_DIR}/runtime"
ROS_LOG_DIR="${SESSION_LOG_DIR}/ros"
LOG_DIR="${RUNTIME_LOG_DIR}"
IGH_DRIVER_BIN="${WORKSPACE_ROOT}/src/erobot_igh_driver/build/igh_driver"
GRAPHICAL_DISPLAY=${DISPLAY:-}
GRAPHICAL_XAUTHORITY=${XAUTHORITY:-}
IGH_PID=""
CHILD_PIDS=()
CHILD_LABELS=()
CLEANUP_STARTED=false
HARDWARE_OWNER_LOCK=/run/lock/junior-hardware-owner.lock
HARDWARE_OWNER_FD_OPEN=false

resolve_graphical_session() {
    if [ -n "${GRAPHICAL_DISPLAY}" ] && [ -n "${GRAPHICAL_XAUTHORITY}" ]; then
        return
    fi

    local pid
    local process_env
    for pid in $(pgrep -u "${RUN_USER}" -f 'gnome-shell|Xwayland' 2>/dev/null); do
        [ -r "/proc/${pid}/environ" ] || continue
        process_env=$(tr '\0' '\n' < "/proc/${pid}/environ")
        if [ -z "${GRAPHICAL_DISPLAY}" ]; then
            GRAPHICAL_DISPLAY=$(printf '%s\n' "${process_env}" | sed -n 's/^DISPLAY=//p' | head -n 1)
        fi
        if [ -z "${GRAPHICAL_XAUTHORITY}" ]; then
            GRAPHICAL_XAUTHORITY=$(printf '%s\n' "${process_env}" | sed -n 's/^XAUTHORITY=//p' | head -n 1)
        fi
        if [ -n "${GRAPHICAL_DISPLAY}" ] && [ -n "${GRAPHICAL_XAUTHORITY}" ]; then
            break
        fi
    done

    GRAPHICAL_DISPLAY=${GRAPHICAL_DISPLAY:-:0}
    GRAPHICAL_XAUTHORITY=${GRAPHICAL_XAUTHORITY:-${RUN_HOME}/.Xauthority}
}

resolve_graphical_session

mkdir -p "${RUNTIME_LOG_DIR}" "${ROS_LOG_DIR}"
chown -R "${RUN_USER}:${RUN_USER}" "${SESSION_LOG_DIR}"
install -d -m 0700 -o "${RUN_USER}" -g "${RUN_USER}" "$(dirname -- "${LIFT_ZERO_OFFSET_FILE}")"

run_as_user_bg() {
    local command="$1"
    local log_file="$2"
    local label="$3"

    # Keep every child in the service cgroup. sudo+setsid previously allowed
    # ROS launch processes to survive after the REAL systemd unit stopped.
    if [ "${EUID}" -eq 0 ]; then
        /usr/sbin/runuser -u "${RUN_USER}" -- \
            env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
            RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
            CYCLONEDDS_URI="${CYCLONEDDS_URI}" \
            ROS_LOG_DIR="${ROS_LOG_DIR}" \
            DISPLAY="${GRAPHICAL_DISPLAY}" XAUTHORITY="${GRAPHICAL_XAUTHORITY}" \
            bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE; source ${WORKSPACE_ROOT}/install/setup.bash && exec ${command}" > "${log_file}" 2>&1 < /dev/null &
    else
        env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
            RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
            CYCLONEDDS_URI="${CYCLONEDDS_URI}" \
            ROS_LOG_DIR="${ROS_LOG_DIR}" \
            DISPLAY="${GRAPHICAL_DISPLAY}" XAUTHORITY="${GRAPHICAL_XAUTHORITY}" \
            bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE; source ${WORKSPACE_ROOT}/install/setup.bash && exec ${command}" > "${log_file}" 2>&1 < /dev/null &
    fi
    CHILD_PIDS+=("$!")
    CHILD_LABELS+=("${label}")
}

cleanup_stack() {
    local exit_code=$?
    trap - EXIT INT TERM

    if [ "${CLEANUP_STARTED}" = false ]; then
        CLEANUP_STARTED=true
        echo "Cleaning up managed robot stack..."
        SIM="${SIM}" "${WORKSPACE_ROOT}/stop.sh" || true
    fi

    exit "${exit_code}"
}

acquire_hardware_owner() {
    if [ "${SIM}" != "0.0" ]; then
        return 0
    fi
    if [ "${EUID}" -ne 0 ]; then
        echo "ERROR: REAL hardware owner lock requires root." >&2
        return 1
    fi
    install -d -m 0770 "$(dirname -- "${HARDWARE_OWNER_LOCK}")"
    eval "exec 9>\"${HARDWARE_OWNER_LOCK}\""
    if ! flock -n 9; then
        echo "ERROR: hardware is owned by non-ROS Direct server or another controller." >&2
        eval 'exec 9>&-'
        return 1
    fi
    HARDWARE_OWNER_FD_OPEN=true
}

monitor_stack() {
    while true; do
        if [ -n "${IGH_PID}" ] && ! kill -0 "${IGH_PID}" 2>/dev/null; then
            echo "ERROR: IGH driver exited; see ${LOG_DIR}/igh_driver.log"
            return 1
        fi

        for index in "${!CHILD_PIDS[@]}"; do
            if ! kill -0 "${CHILD_PIDS[${index}]}" 2>/dev/null; then
                echo "ERROR: ${CHILD_LABELS[${index}]} exited; see runtime logs in ${LOG_DIR}"
                return 1
            fi
        done

        sleep 1
    done
}

stack_launches_running() {
    pgrep -u "${RUN_USER}" -f \
        '/opt/ros/humble/bin/ros2 launch (erobot_controller load_controller_arm\.launch\.py|robot_control (robot_control|joint_controller_stack_sim)\.launch\.py|joint_hardware lift_ethercat\.launch\.py)' \
        > /dev/null 2>&1
}

cleanup_stale_stack() {
    local deadline

    if ! stack_launches_running; then
        return 0
    fi

    # A duplicate controller stack can retain EtherCAT and publish stale ROS
    # endpoints. stop.sh first requests a controlled power disable, then
    # terminates only the processes owned by this robot stack.
    echo "检测到旧的机器人控制 ROS 进程，先执行受控清理..."
    "${WORKSPACE_ROOT}/stop.sh"

    deadline=$((SECONDS + 15))
    while stack_launches_running; do
        if [ "${SECONDS}" -ge "${deadline}" ]; then
            echo "ERROR: old robot stack ROS processes did not exit after controlled cleanup."
            echo "Refusing to start a second controller stack."
            return 1
        fi
        sleep 0.2
    done
    echo "旧的机器人控制 ROS 进程已退出。"
}

if awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    TARGET_STACK_UNIT="joint-controller-stack.service"
else
    TARGET_STACK_UNIT="joint-controller-stack-real.service"
fi

# When called manually while the same stack is already managed by systemd,
# restart the unit as a single transaction. This lets systemd finish the old
# cgroup cleanup before a new start.sh instance is created. INVOCATION_ID is
# set for the start.sh instance that systemd itself launches.
if [ -z "${INVOCATION_ID:-}" ] && systemctl is-active --quiet "${TARGET_STACK_UNIT}"; then
    echo "${TARGET_STACK_UNIT} 已在运行，执行受控 restart..."
    if [ "${EUID}" -eq 0 ]; then
        exec /usr/bin/systemctl restart "${TARGET_STACK_UNIT}"
    fi
    exec /usr/bin/sudo /usr/bin/systemctl restart "${TARGET_STACK_UNIT}"
fi

cleanup_stale_stack

trap cleanup_stack EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

acquire_hardware_owner

echo "============ 机器人启动脚本开始: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}, RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}, CYCLONEDDS_URI=${CYCLONEDDS_URI} ============"
echo "模式: sim=${SIM}, use_rviz=${USE_RVIZ}, enable_effort_mode_switch=${ENABLE_EFFORT_MODE_SWITCH}"
echo "图形会话: DISPLAY=${GRAPHICAL_DISPLAY}, XAUTHORITY=${GRAPHICAL_XAUTHORITY}"
echo "会话日志: ${SESSION_LOG_DIR}"
date

sleep 1

if awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    echo "仿真模式：跳过 EtherCAT 和 IGH 驱动"
else
    if [ "${LIFT_BRAKE_CONTROL_ENABLED}" = "true" ] &&
       ! [[ "${LIFT_BRAKE_RELEASE_WAIT_MS}" =~ ^[0-9]+$ ]]; then
        echo "ERROR: lift power authorization requires a confirmed non-negative LIFT_BRAKE_RELEASE_WAIT_MS."
        exit 1
    fi
    if [ "${EUID}" -ne 0 ]; then
        echo "真机模式需要 root 权限，转交给 joint-controller-stack-real.service 启动..."
        # Direct interactive launches still use the managed systemd unit. This
        # keeps IGH/ROS cleanup in one cgroup while allowing ./start.sh to prompt
        # for sudo normally from the operator's terminal.
        trap - EXIT INT TERM
        exec /usr/bin/sudo /usr/bin/systemctl start joint-controller-stack-real.service
    fi

    echo "启动 EtherCAT..."
    # 启动 EtherCAT
    /etc/init.d/ethercat start

    sleep 5

    for master_index in 0 1 2; do
        if [ ! -e "/dev/EtherCAT${master_index}" ]; then
            echo "ERROR: required EtherCAT Master${master_index} is unavailable (/dev/EtherCAT${master_index} missing)."
            echo "Restore the robot topology with:"
            echo "  sudo ${WORKSPACE_ROOT}/install/joint_hardware/lib/joint_hardware/robot_ethercat_host_setup.sh --apply enp3s0 enp4s0 ${LIFT_ETHERCAT_INTERFACE}"
            /etc/init.d/ethercat stop || true
            exit 1
        fi
    done

    echo "只读检查 EtherCAT 主站和14个机械臂从站..."
    /usr/bin/ethercat master || {
        echo "ERROR: unable to query the EtherCAT masters."
        /etc/init.d/ethercat stop || true
        exit 1
    }
    if ! ETHERCAT_SLAVES_OUTPUT=$(/usr/bin/ethercat slaves 2>&1); then
        printf '%s\n' "${ETHERCAT_SLAVES_OUTPUT}"
        echo "ERROR: unable to scan EtherCAT slaves."
        /etc/init.d/ethercat stop || true
        exit 1
    fi
    printf '%s\n' "${ETHERCAT_SLAVES_OUTPUT}"
    read -r MASTER0_DRIVES MASTER1_DRIVES MASTER2_DRIVES ARM_DRIVES < <(
        printf '%s\n' "${ETHERCAT_SLAVES_OUTPUT}" | awk '
            /^Master0$/ {master = 0; next}
            /^Master1$/ {master = 1; next}
            /^Master2$/ {master = 2; next}
            /^Master[3-9][0-9]*$/ {master = -1; next}
            /^[[:space:]]*[0-9]+[[:space:]]+[0-9]+:[0-9]+[[:space:]]/ {
                if (master == 0) {master0++; arms++}
                if (master == 1) {master1++; arms++}
                if (master == 2) {master2++}
            }
            END {print master0 + 0, master1 + 0, master2 + 0, arms + 0}
        '
    )
    if [ "${MASTER0_DRIVES}" -ne 7 ] || [ "${MASTER1_DRIVES}" -ne 7 ] || \
       [ "${ARM_DRIVES}" -ne 14 ]; then
        echo "ERROR: not all 14 arm EtherCAT drives are available; Master0=${MASTER0_DRIVES}/7, Master1=${MASTER1_DRIVES}/7, total=${ARM_DRIVES}/14."
        /etc/init.d/ethercat stop || true
        exit 1
    fi
    if [ "${MASTER2_DRIVES}" -ne 1 ]; then
        echo "ERROR: lift EtherCAT topology mismatch; Master2=${MASTER2_DRIVES}/1."
        /etc/init.d/ethercat stop || true
        exit 1
    fi
    echo "EtherCAT discovery confirmed: arms Master0=7/7, Master1=7/7; lift Master2=1/1."

    echo "只读确认 Master2 升降从站身份和 PDO..."
    if ! LIFT_PREFLIGHT_OUTPUT=$(
        "${WORKSPACE_ROOT}/install/joint_hardware/lib/joint_hardware/lift_ethercat_preflight.sh" \
            2 0 "${LIFT_ETHERCAT_INTERFACE}" 2>&1
    ); then
        printf '%s\n' "${LIFT_PREFLIGHT_OUTPUT}"
        echo "ERROR: lift EtherCAT preflight failed; REAL startup refused."
        /etc/init.d/ethercat stop || true
        exit 1
    fi
    printf '%s\n' "${LIFT_PREFLIGHT_OUTPUT}"
    LIFT_VENDOR_ID=$(printf '%s\n' "${LIFT_PREFLIGHT_OUTPUT}" | sed -n \
        's/.*slave_vendor_id:=\([^[:space:]]*\).*/\1/p' | tail -n 1)
    LIFT_PRODUCT_CODE=$(printf '%s\n' "${LIFT_PREFLIGHT_OUTPUT}" | sed -n \
        's/.*slave_product_code:=\([^[:space:]]*\).*/\1/p' | tail -n 1)
    if [ -z "${LIFT_VENDOR_ID}" ] || [ -z "${LIFT_PRODUCT_CODE}" ]; then
        echo "ERROR: could not obtain the lift vendor/product identity from the read-only scan."
        /etc/init.d/ethercat stop || true
        exit 1
    fi

    echo "启动 IGH 驱动..."
    if [ ! -x "${IGH_DRIVER_BIN}" ]; then
        echo "ERROR: workspace IGH driver is missing or not executable: ${IGH_DRIVER_BIN}"
        echo "Build it with: cmake -S ${WORKSPACE_ROOT}/src/erobot_igh_driver -B ${WORKSPACE_ROOT}/src/erobot_igh_driver/build && cmake --build ${WORKSPACE_ROOT}/src/erobot_igh_driver/build"
        /etc/init.d/ethercat stop || true
        exit 1
    fi
    # Keep the CiA402 transition/error log; it is required to diagnose 0x603F faults.
    "${IGH_DRIVER_BIN}" \
        > "${LOG_DIR}/igh_driver.log" 2>&1 < /dev/null &
    IGH_PID=$!
    printf '%s\n' "${IGH_PID}" > /var/run/igh_driver.pid
    sleep 1

    # The REAL systemd unit must not report success when the background IGH
    # process has already exited.  The old oneshot unit hid this failure and
    # left the EtherCAT master in Idle/PREOP while ROS appeared to be running.
    if ! kill -0 "${IGH_PID}" 2>/dev/null; then
        echo "ERROR: IGH driver exited during startup; see ${LOG_DIR}/igh_driver.log"
        rm -f /var/run/igh_driver.pid
        /etc/init.d/ethercat stop || true
        exit 1
    fi

    # A driver can remain alive while waiting forever for PREOP slaves to
    # reach OP.  Treat an initialization SDO failure as a startup failure so
    # the supervisor does not expose a partially initialized REAL stack.
    if grep -qE 'Failed to execute SDO download|OD write fail:' "${LOG_DIR}/igh_driver.log"; then
        echo "ERROR: IGH driver reported an SDO initialization failure; see ${LOG_DIR}/igh_driver.log"
        "${WORKSPACE_ROOT}/stop.sh" || true
        exit 1
    fi
fi

echo "启动 arm 节点..."
run_as_user_bg \
    "ros2 launch erobot_controller load_controller_arm.launch.py sim:=${SIM} use_rviz:=${USE_RVIZ} enable_effort_mode_switch:=${ENABLE_EFFORT_MODE_SWITCH}" \
    "${LOG_DIR}/robot_arm.log" \
    "arm controller launch"
sleep 2

if ! awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    echo "启动 lift EtherCAT 控制器（保持失能）..."
    run_as_user_bg \
        "ros2 launch joint_hardware lift_ethercat.launch.py backend:=etherlab namespace:=lift master_index:=2 slave_alias:=0 slave_position:=0 slave_vendor_id:=${LIFT_VENDOR_ID} slave_product_code:=${LIFT_PRODUCT_CODE} ethercat_interface:=${LIFT_ETHERCAT_INTERFACE} brake_control_enabled:=${LIFT_BRAKE_CONTROL_ENABLED} brake_release_wait_ms:=${LIFT_BRAKE_RELEASE_WAIT_MS} use_persistent_zero_offset:=true zero_offset_file:=${LIFT_ZERO_OFFSET_FILE}" \
        "${LOG_DIR}/lift_controller.log" \
        "lift controller launch"
fi

echo "启动 arm_ik 服务节点..."
run_as_user_bg \
    "ros2 launch robot_control robot_control.launch.py" \
    "${LOG_DIR}/robot_srv.log" \
    "robot control launch"

echo "启动完成"

# start.sh is the systemd MainPID. Keep it alive and fail the unit whenever a
# required child exits. The EXIT trap performs bounded cleanup in every path.
monitor_stack

#!/usr/bin/env bash
set -u

WORKSPACE_ROOT=/home/user/joint_controller
RUN_USER=${SUDO_USER:-$(id -un)}
SIM="${SIM:-0.0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/user/joint_controller/cyclonedds.xml}"
unset FASTDDS_BUILTIN_TRANSPORTS

if [ "${EUID}" -ne 0 ] && ! awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    echo "stop.sh needs root privileges; re-running with sudo..."
    exec sudo bash "$0" "$@"
fi

if [ "${EUID}" -ne 0 ]; then
    echo "Simulation cleanup running as ${RUN_USER}; EtherCAT will not be touched."
fi

collect_matching_pids() {
    local pattern
    local pid
    for pattern in "$@"; do
        while read -r pid; do
            [ -n "${pid}" ] || continue
            [ "${pid}" = "$$" ] && continue
            [ "${pid}" = "${PPID}" ] && continue
            printf '%s\n' "${pid}"
        done < <(pgrep -f -- "${pattern}" 2>/dev/null || true)
    done | sort -nu
}

terminate_pid_set() {
    local label=$1
    shift
    local pids=("$@")
    local deadline
    local survivors=()

    [ "${#pids[@]}" -gt 0 ] || return 0
    echo "Terminating ${label}: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true

    deadline=$((SECONDS + 5))
    while [ "${SECONDS}" -lt "${deadline}" ]; do
        survivors=()
        for pid in "${pids[@]}"; do
            kill -0 "${pid}" 2>/dev/null && survivors+=("${pid}")
        done
        [ "${#survivors[@]}" -eq 0 ] && return 0
        sleep 0.2
    done

    echo "Forcing remaining ${label}: ${survivors[*]}"
    kill -KILL "${survivors[@]}" 2>/dev/null || true
}

request_power_disable() {
    local output

    if [ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        echo "WARNING: workspace setup is unavailable; cannot request controller power disable."
        return 1
    fi

    echo "Requesting controller power disable before stopping ROS and EtherCAT..."
    if output=$(
        /usr/bin/timeout 10s /usr/sbin/runuser -u "${RUN_USER}" -- \
            env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
            RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
            CYCLONEDDS_URI="${CYCLONEDDS_URI}" \
            bash -lc \
            "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE; source /opt/ros/humble/setup.bash; source ${WORKSPACE_ROOT}/install/setup.bash; ros2 service call /set_robot_power robot_control_msg/srv/SetRobotPower '{enable: false}'" \
            2>&1
    ); then
        printf '%s\n' "${output}"
        if printf '%s\n' "${output}" | grep -Eq 'success=(True|true)|success:[[:space:]]*true'; then
            echo "Controller power disable confirmed."
            return 0
        fi
    else
        printf '%s\n' "${output}"
    fi

    echo "WARNING: controller power disable was not confirmed; continuing process cleanup."
    echo "WARNING: verify the physical emergency stop and drive power state before any restart."
    return 1
}

echo "Stopping managed robot processes..."

mapfile -t IGH_PIDS < <(
    if [ -s /var/run/igh_driver.pid ]; then
        sed -n '1p' /var/run/igh_driver.pid
    fi
    collect_matching_pids '/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver'
)
mapfile -t IGH_PIDS < <(printf '%s\n' "${IGH_PIDS[@]}" | sed '/^$/d' | sort -nu)

# Keep the cyclic driver alive long enough for the controller's disable
# request and its statusword feedback to complete.
if [ "${#IGH_PIDS[@]}" -gt 0 ]; then
    request_power_disable || true
else
    echo "IGH driver is not running; skipping software disable confirmation."
fi

PROCESS_PATTERNS=(
    '/opt/ros/humble/bin/ros2 launch erobot_controller load_controller_arm\.launch\.py'
    '/opt/ros/humble/bin/ros2 launch robot_control robot_control\.launch\.py'
    '/opt/ros/humble/bin/ros2 launch robot_control joint_controller_stack_sim\.launch\.py'
    '/opt/ros/humble/bin/ros2 launch joint_hardware lift_ethercat\.launch\.py'
    'ros2_control_node'
    'controller_manager/spawner'
    'robot_state_publisher'
    'rviz2'
    'cartesian_.*control_srv'
    'joint_.*control_srv'
    'cartesian_moveL'
    'teleop_control_node'
    'robot_power_service'
    'arm_control_mode_service'
    'bms_reader_node'
    'hardware_reader'
    'joy_node'
    'canopen'
    'load_controller_leg'
)
mapfile -t ROBOT_PIDS < <(collect_matching_pids "${PROCESS_PATTERNS[@]}")
terminate_pid_set "ROS/CAN robot processes" "${ROBOT_PIDS[@]}"

terminate_pid_set "IGH driver" "${IGH_PIDS[@]}"

if ! awk "BEGIN {exit !(${SIM} > 0.0)}"; then
    echo "Stopping EtherCAT service..."
    /etc/init.d/ethercat stop >/dev/null 2>&1 || true
    ip link set down can2 >/dev/null 2>&1 || true
    rm -f /var/run/igh_driver.pid
fi
rm -f /tmp/joint_controller_robot_command.lock

echo "Managed robot processes stopped."

#!/usr/bin/env bash
set -Eeuo pipefail

# Remove logs created by this workspace without touching unrelated files in
# /tmp.  The project root is resolved from this script, so it is safe to run
# the script from any working directory.
WORKSPACE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
WORKSPACE_LOG_DIR="${WORKSPACE_ROOT}/log"
RUN_USER=${SUDO_USER:-$(id -un)}
RUN_HOME=$(getent passwd "${RUN_USER}" | cut -d: -f6 || true)
RUN_HOME=${RUN_HOME:-${HOME}}
ROS_LOG_DIR="${RUN_HOME}/.ros/log"

DRY_RUN=false

usage() {
    cat <<'EOF'
Usage: ./clearlog.sh [--dry-run]

Remove this workspace's colcon/runtime logs, the current user's default ROS 2
logs, and the project's known temporary ROS/EtherCAT diagnostic logs. The
script does not stop running processes.

Options:
  -n, --dry-run  Show what would be removed without deleting anything.
  -h, --help     Show this help text.
EOF
}

while (($# > 0)); do
    case "$1" in
        -n|--dry-run)
            DRY_RUN=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

managed_processes_running() {
    local pattern
    local pid
    local patterns=(
        "${WORKSPACE_ROOT}/start.sh"
        "${WORKSPACE_ROOT}/src/erobot_igh_driver/build/igh_driver"
        "${WORKSPACE_ROOT}/build/erobot_igh_driver/igh_driver"
        "${WORKSPACE_ROOT}/tools/lift_ethercat_cli"
        '/opt/ros/humble/bin/ros2 launch erobot_controller load_controller_arm\.launch\.py'
        '/opt/ros/humble/bin/ros2 launch robot_control robot_control\.launch\.py'
        '/opt/ros/humble/bin/ros2 launch robot_control joint_controller_stack_sim\.launch\.py'
        '/opt/ros/humble/bin/ros2 launch joint_hardware lift_ethercat\.launch\.py'
    )

    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -n "${pid}" && "${pid}" != "$$" ]] || continue
            return 0
        done < <(pgrep -f -- "${pattern}" 2>/dev/null || true)
    done
    return 1
}

remove_path() {
    local path=$1

    if [[ ! -e "${path}" && ! -L "${path}" ]]; then
        return 0
    fi

    if [[ "${DRY_RUN}" == true ]]; then
        printf 'Would remove: %s\n' "${path}"
    else
        rm -rf -- "${path}"
        printf 'Removed: %s\n' "${path}"
    fi
}

clear_directory() {
    local directory=$1
    local preserve_name=${2:-}
    local entry

    [[ -d "${directory}" ]] || return 0
    while IFS= read -r -d '' entry; do
        [[ -n "${preserve_name}" && "$(basename -- "${entry}")" == "${preserve_name}" ]] && continue
        remove_path "${entry}"
    done < <(find "${directory}" -mindepth 1 -maxdepth 1 -print0)
}

if [[ "${DRY_RUN}" != true ]] && managed_processes_running; then
    echo "ERROR: the managed robot stack is running; stop it before clearing logs." >&2
    echo "       Run ./stop.sh and then ./clearlog.sh." >&2
    exit 1
fi

if [[ "${DRY_RUN}" == true ]]; then
    echo "Dry run: no files will be deleted."
fi

# Keep COLCON_IGNORE so colcon continues to treat this directory as generated
# output. It is recreated automatically by colcon when needed, but retaining
# it also keeps the directory harmless between builds.
clear_directory "${WORKSPACE_LOG_DIR}" "COLCON_IGNORE"

# Lower runtime and hardware logs now live under the same per-session ROS root
# as the ROS 2 logs. Preserve the .ros directory and any non-log configuration
# below it.
clear_directory "${ROS_LOG_DIR}"

# CMake/CTest stores its per-package test output below build/*/Testing. Keep
# the build products themselves, but remove only the generated test logs.
if [[ -d "${WORKSPACE_ROOT}/build" ]]; then
    while IFS= read -r -d '' directory; do
        clear_directory "${directory}"
    done < <(find "${WORKSPACE_ROOT}/build" -type d -path '*/Testing/Temporary' -print0)
fi

# These directories are explicitly selected by the project's CMake tests.
for directory in \
    /tmp/joint_hardware_ros_logs \
    /tmp/robot_lower_gateway_ros_logs \
    /tmp/robot_lower_gateway_lift_status \
    /tmp/robot_lower_gateway_lift_zero_proxy \
    /tmp/robot_lower_gateway_residency; do
    clear_directory "${directory}"
done

# Remove legacy /tmp files left by versions before the per-session .ros/log
# layout. New driver and lift traces are removed by clear_directory above.
for path in /tmp/heavy_v1_igh_driver*.log /tmp/lift_ethercat_cli.csv; do
    [[ -e "${path}" || -L "${path}" ]] || continue
    remove_path "${path}"
done

if [[ "${DRY_RUN}" == true ]]; then
    echo "Dry run complete. No files were deleted. Run ./clearlog.sh without --dry-run to remove them."
else
    echo "Project logs cleared."
fi

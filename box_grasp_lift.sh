#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"

JOINT_VEL="${JOINT_VEL:-0.5}"
JOINT_ACC="${JOINT_ACC:-0.5}"
LIFT_Z="${LIFT_Z:-0.10}"
LIFT_VEL="${LIFT_VEL:-0.02}"
LIFT_ACC="${LIFT_ACC:-0.05}"
MAX_CARTESIAN_INCREMENT="${MAX_CARTESIAN_INCREMENT:-0.05}"

JOINT_SERVICE="/arm_absolute_control"
CARTESIAN_INC_SERVICE="/cartesian_increment_control"
POWER_SERVICE="/set_robot_power"

wait_for_service() {
    local service_name="$1"
    echo "Waiting for ${service_name} ..."
    local attempt
    for ((attempt = 1; attempt <= 50; attempt++)); do
        if timeout 2 ros2 service list 2>/dev/null | grep -Fqx -- "${service_name}"; then
            return 0
        fi
        sleep 0.2
    done
    echo "ERROR: timed out waiting for ${service_name}" >&2
    return 1
}

call_service_expect_success() {
    local output
    if ! output=$(ros2 service call "$@" 2>&1); then
        printf '%s\n' "${output}"
        return 1
    fi
    printf '%s\n' "${output}"
    if ! grep -Eq 'success=(True|true)|success: (True|true)' <<<"${output}"; then
        echo "ERROR: service response did not report success" >&2
        return 1
    fi
}

set_power_on() {
    wait_for_service "${POWER_SERVICE}"
    call_service_expect_success "${POWER_SERVICE}" robot_control_msg/srv/SetRobotPower "{enable: true}"
}

move_joints() {
    local label="$1"
    shift
    local lj1="$1" lj2="$2" lj3="$3" lj4="$4" lj5="$5" lj6="$6" lj7="$7"
    local rj1="$8" rj2="$9" rj3="${10}" rj4="${11}" rj5="${12}" rj6="${13}" rj7="${14}"

    echo
    echo "==> ${label}"
    call_service_expect_success "${JOINT_SERVICE}" robot_control_msg/srv/JointAbsoluteControl "{
      ljoint1: ${lj1},
      ljoint2: ${lj2},
      ljoint3: ${lj3},
      ljoint4: ${lj4},
      ljoint5: ${lj5},
      ljoint6: ${lj6},
      ljoint7: ${lj7},
      rjoint1: ${rj1},
      rjoint2: ${rj2},
      rjoint3: ${rj3},
      rjoint4: ${rj4},
      rjoint5: ${rj5},
      rjoint6: ${rj6},
      rjoint7: ${rj7},
      vel: ${JOINT_VEL},
      acc: ${JOINT_ACC}
    }"
}

move_cartesian_z() {
    local label="$1"
    local dz="$2"
    local remaining="${dz}"
    local part=1
    local step

    while awk -v value="${remaining}" 'BEGIN { exit ! (value > 1e-9 || value < -1e-9) }'; do
        step=$(awk -v value="${remaining}" -v max="${MAX_CARTESIAN_INCREMENT}" \
            'BEGIN { if (value > max) value = max; else if (value < -max) value = -max; printf "%.6f", value }')
        echo
        echo "==> ${label} (part ${part}): dz=${step} m"
        call_service_expect_success "${CARTESIAN_INC_SERVICE}" robot_control_msg/srv/CartesianIncrementControl "{
          lx: 0.0,
          ly: 0.0,
          lz: ${step},
          lroll: 0.0,
          lpitch: 0.0,
          lyaw: 0.0,
          lqx: 0.0,
          lqy: 0.0,
          lqz: 0.0,
          lqw: 0.0,
          rx: 0.0,
          ry: 0.0,
          rz: ${step},
          rroll: 0.0,
          rpitch: 0.0,
          ryaw: 0.0,
          rqx: 0.0,
          rqy: 0.0,
          rqz: 0.0,
          rqw: 0.0,
          vel: ${LIFT_VEL},
          acc: ${LIFT_ACC}
        }"
        remaining=$(awk -v value="${remaining}" -v increment="${step}" \
            'BEGIN { printf "%.6f", value - increment }')
        part=$((part + 1))
    done
}

confirm_continue() {
    echo
    read -r -p "Box is lifted and holding. Confirm manually, then press Enter to lower and reverse. Type 'q' then Enter to stop here: " reply
    if [[ "${reply}" == "q" || "${reply}" == "Q" ]]; then
        echo "Stopped after lift. Robot will keep holding current command."
        exit 0
    fi
}

main() {
    echo "Box grasp/lift flow starting with ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "Joint vel/acc=${JOINT_VEL}/${JOINT_ACC}, lift dz=${LIFT_Z}, lift vel/acc=${LIFT_VEL}/${LIFT_ACC}"

    wait_for_service "${JOINT_SERVICE}"
    wait_for_service "${CARTESIAN_INC_SERVICE}"
    set_power_on

    # Forward flow:
    # 1) lj2/rj2 open to 0.5/-0.5 while joint7 reaches its target; others remain zero.
    move_joints "pre-grasp: joint2 open, joint7 to target" \
        0.0 0.5 0.0 0.0 0.0 0.0 0.1 \
        0.0 -0.5 0.0 0.0 0.0 0.0 -0.15

    # 2) Move joints in the requested order: 3, 4, 5, 6, then 2 to final target.
    move_joints "joint3 to target" \
        0.0 0.5 -0.15 0.0 0.0 0.0 0.1 \
        0.0 -0.5 0.15 0.0 0.0 0.0 -0.15

    move_joints "joint4 to target" \
        0.0 0.5 -0.15 -1.5 0.0 0.0 0.1 \
        0.0 -0.5 0.15 1.45 0.0 0.0 -0.15

    move_joints "joint5 to target" \
        0.0 0.5 -0.15 -1.5 -3.14 0.0 0.1 \
        0.0 -0.5 0.15 1.45 -3.0 0.0 -0.15

    move_joints "joint6 to target" \
        0.0 0.5 -0.15 -1.5 -3.14 0.12 0.1 \
        0.0 -0.5 0.15 1.45 -3.0 -0.05 -0.15

    move_joints "joint2 close to final target" \
        0.0 0.12 -0.15 -1.5 -3.14 0.12 0.1 \
        0.0 -0.12 0.15 1.45 -3.0 -0.05 -0.15

    move_cartesian_z "lift box" "${LIFT_Z}"
    confirm_continue
    move_cartesian_z "lower box" "-${LIFT_Z}"

    # Reverse flow.
    move_joints "reverse joint2 to pre-grasp" \
        0.0 0.5 -0.15 -1.5 -3.14 0.12 0.1 \
        0.0 -0.5 0.15 1.45 -3.0 -0.05 -0.15

    move_joints "reverse joint6 to zero" \
        0.0 0.5 -0.15 -1.5 -3.14 0.0 0.1 \
        0.0 -0.5 0.15 1.45 -3.0 0.0 -0.15

    move_joints "reverse joint5 to zero" \
        0.0 0.5 -0.15 -1.5 0.0 0.0 0.1 \
        0.0 -0.5 0.15 1.45 0.0 0.0 -0.15

    move_joints "reverse joint4 to zero" \
        0.0 0.5 -0.15 0.0 0.0 0.0 0.1 \
        0.0 -0.5 0.15 0.0 0.0 0.0 -0.15

    move_joints "reverse joint3 to zero" \
        0.0 0.5 0.0 0.0 0.0 0.0 0.1 \
        0.0 -0.5 0.0 0.0 0.0 0.0 -0.15

    move_joints "return joint2 and joint7 to zero" \
        0.0 0.0 0.0 0.0 0.0 0.0 0.0 \
        0.0 0.0 0.0 0.0 0.0 0.0 0.0

    echo
    echo "Box grasp/lift flow finished."
}

main "$@"

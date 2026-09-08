#!/usr/bin/env bash
set -euo pipefail

PREFIX="${ROBOT_GATEWAY_COMMAND_PROXY_PREFIX:-/ubuntu_lower_gateway}"
if [[ "${PREFIX}" == "~" || "${PREFIX}" == "~/" ]]; then
  PREFIX="/ubuntu_lower_gateway"
fi
PREFIX="${PREFIX%/}"

if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: verify_gateway_proxy.sh [proxy-prefix]

Checks proxy service names and ROS service types. It never calls a service.
The default prefix is /ubuntu_lower_gateway.
EOF
  exit 0
fi
if [[ -n "${1:-}" ]]; then
  PREFIX="${1%/}"
fi

declare -a SERVICES=(
  "set_robot_power|robot_control_msg/srv/SetRobotPower"
  "set_control_mode|robot_control_msg/srv/SetArmControlMode"
  "joint_batch_control|robot_control_msg/srv/JointBatchControl"
  "joint_absolute_control|robot_control_msg/srv/JointAbsoluteControl"
  "cartesian_increment_control|robot_control_msg/srv/CartesianIncrementControl"
  "cartesian_absolute_control|robot_control_msg/srv/CartesianAbsoluteControl"
)

echo "Checking gateway proxy under ${PREFIX}"
echo "No service request will be sent."
failed=0
for entry in "${SERVICES[@]}"; do
  IFS='|' read -r name expected_type <<<"${entry}"
  service="${PREFIX}/${name}"
  actual_type=""
  if actual_type="$(ros2 service type "${service}" 2>/dev/null)" &&
    [[ "${actual_type}" == "${expected_type}" ]]; then
    printf 'OK   %-58s %s\n' "${service}" "${actual_type}"
  else
    printf 'FAIL %-58s expected %s (got %s)\n' \
      "${service}" "${expected_type}" "${actual_type:-missing}"
    failed=1
  fi
done

if ! ros2 node info /ubuntu_lower_gateway >/dev/null 2>&1; then
  echo "FAIL /ubuntu_lower_gateway node is not discoverable" >&2
  failed=1
else
  echo "OK   /ubuntu_lower_gateway node is discoverable"
fi

if (( failed != 0 )); then
  echo "Gateway proxy verification failed" >&2
  exit 1
fi
echo "Gateway proxy topology is valid"

#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=/home/user/joint_controller
CONFIRMATION_FLAG=--motors-powered-off

if [[ "${1:-}" != "${CONFIRMATION_FLAG}" ]]; then
  echo "Refusing to run without ${CONFIRMATION_FLAG}." >&2
  echo "This repair stops every managed arm stack before replacing its systemd units." >&2
  exit 2
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Re-running as root..."
  exec sudo "${BASH_SOURCE[0]}" "${CONFIRMATION_FLAG}"
fi

echo "Stopping managed SIM/REAL stacks before replacing their unit files..."
for unit in \
  robot.service \
  joint-controller-stack.service \
  joint-controller-stack-sim.service \
  joint-controller-stack-real.service \
  joint-controller-rviz.service; do
  systemctl stop "${unit}" 2>/dev/null || true
done

# The pre-fix units could leave ROS or IGH descendants outside systemd's
# cgroup. stop.sh catches only those robot processes and never starts a stack.
"${WORKSPACE_ROOT}/stop.sh" || true

"${WORKSPACE_ROOT}/scripts/install_workspace_control_services.sh"

# `systemctl enable --now` does not restart an already-active gateway. Reload
# both long-lived processes so they use the just-installed unit and workspace.
systemctl restart robot-lower-gateway.service
systemctl restart joint-controller-supervisor.service

sleep 2

verify_unit() {
  local unit="$1"
  local expected="$2"
  local actual
  actual=$(systemctl is-active "${unit}" 2>/dev/null || true)
  printf '%s: %s (expected %s)\n' "${unit}" "${actual:-unknown}" "${expected}"
  [[ "${actual}" == "${expected}" ]]
}

verification_failed=false
verify_unit robot-lower-gateway.service active || verification_failed=true
verify_unit joint-controller-supervisor.service active || verification_failed=true
verify_unit joint-controller-stack-real.service inactive || verification_failed=true
verify_unit joint-controller-stack-sim.service inactive || verification_failed=true
verify_unit joint-controller-stack.service inactive || verification_failed=true
verify_unit joint-controller-rviz.service inactive || verification_failed=true

if [[ "${verification_failed}" == true ]]; then
  echo "Repair installed the unit files, but the final stopped-state verification failed." >&2
  exit 1
fi

echo "Repair complete: gateway and workspace supervisor restarted; SIM/REAL stacks remain stopped."

#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=/home/user/joint_controller
SYSTEMD_DIR=/etc/systemd/system
GATEWAY_SHARE="${WORKSPACE_ROOT}/install/robot_lower_gateway/share/robot_lower_gateway"
LIFT_SHARE="${WORKSPACE_ROOT}/install/joint_hardware/share/joint_hardware"
GATEWAY_TEMPLATE="${GATEWAY_SHARE}/systemd/robot-lower-gateway.service.in"
GATEWAY_RUNNER="${WORKSPACE_ROOT}/install/robot_lower_gateway/lib/robot_lower_gateway/run_gateway.sh"

if [[ ! -f "${GATEWAY_TEMPLATE}" || ! -x "${GATEWAY_RUNNER}" ||
  ! -f "${GATEWAY_SHARE}/config/erobot_v1.yaml" ||
  ! -f "${LIFT_SHARE}/config/lift_hardware.yaml" ]]; then
  echo "robot_lower_gateway is not built; build it before installing workspace services." >&2
  exit 1
fi

if systemctl list-unit-files robot.service --no-legend 2>/dev/null | \
  grep -q '^robot.service'; then
  echo "Disabling legacy robot.service to prevent an unmanaged duplicate ROS stack."
  sudo systemctl disable --now robot.service
  sudo systemctl reset-failed robot.service 2>/dev/null || true
fi

sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/joint-controller-stack.service" \
  "${SYSTEMD_DIR}/joint-controller-stack.service"
sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/joint-controller-stack-sim.service" \
  "${SYSTEMD_DIR}/joint-controller-stack-sim.service"
sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/joint-controller-stack-real.service" \
  "${SYSTEMD_DIR}/joint-controller-stack-real.service"
sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/joint-controller-rviz.service" \
  "${SYSTEMD_DIR}/joint-controller-rviz.service"
sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/joint-controller-supervisor.service" \
  "${SYSTEMD_DIR}/joint-controller-supervisor.service"
sudo install -m 0644 \
  "${WORKSPACE_ROOT}/systemd/hardware-server.service" \
  "${SYSTEMD_DIR}/hardware-server.service"

GATEWAY_GROUP="$(id -gn user)"
TEMP_GATEWAY_UNIT="$(mktemp)"
trap 'rm -f "${TEMP_GATEWAY_UNIT}"' EXIT
sed \
  -e 's|@ROBOT_USER@|user|g' \
  -e "s|@ROBOT_GROUP@|${GATEWAY_GROUP}|g" \
  -e "s|@GATEWAY_WORKSPACE@|${WORKSPACE_ROOT}|g" \
  -e "s|@GATEWAY_RUNNER@|${GATEWAY_RUNNER}|g" \
  "${GATEWAY_TEMPLATE}" > "${TEMP_GATEWAY_UNIT}"
sudo install -m 0644 "${TEMP_GATEWAY_UNIT}" \
  "${SYSTEMD_DIR}/robot-lower-gateway.service"

sudo systemctl daemon-reload

# Only the supervisor listens at boot. The SIM/REAL stacks must always be
# selected explicitly by an allowed workspace command.
sudo systemctl disable joint-controller-stack.service 2>/dev/null || true
sudo systemctl disable joint-controller-stack-sim.service 2>/dev/null || true
sudo systemctl disable joint-controller-stack-real.service 2>/dev/null || true
sudo systemctl disable joint-controller-rviz.service 2>/dev/null || true
sudo systemctl reset-failed joint-controller-stack-sim.service 2>/dev/null || true
sudo systemctl reset-failed joint-controller-stack-real.service 2>/dev/null || true
sudo systemctl enable joint-controller-supervisor.service
sudo systemctl enable --now robot-lower-gateway.service
sudo systemctl restart joint-controller-supervisor.service

echo "Installed workspace control services."
echo "Boot policy: supervisor enabled; SIM/REAL stacks disabled until requested."
echo "Supervisor: systemctl status joint-controller-supervisor.service"
echo "Lower gateway: systemctl status robot-lower-gateway.service"
echo "TCP hardware server: systemctl status hardware-server.service"
echo "Simulation stack: systemctl status joint-controller-stack-sim.service"
echo "Simulation RViz: systemctl status joint-controller-rviz.service"
echo "Real stack: systemctl status joint-controller-stack-real.service"

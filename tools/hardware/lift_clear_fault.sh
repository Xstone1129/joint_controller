#!/usr/bin/env bash
# =============================================================================
# Clear a latched lift motion fault WITHOUT restarting the stack.
#
# Background: LiftController latches Mode::fault (e.g. gate_lost_during_motion,
# which is what an over-travel event produces when the drive drops out of
# Operation Enabled).  The ONLY way out of that latch is re-activation --
# LiftController::on_activate() resets mode_ to hold and logs
#   [LIFT_FAULT] state=cleared_by_activate previous_mode=fault last_code=...
# so a deactivate/activate cycle of the ros2_control controller `lift_controller`
# clears it.  Until then `lift power on` keeps failing with
# "brake release refused while a latched lift motion fault is active".
#
# This is NOT for a drive-side protective latch: if the LD3M itself is stuck
# (603Fh != 0, or it never reaches Operation Enabled), the drive needs a power
# cycle and this script cannot help.
#
# Safety: run it with the lift at rest.  Re-activation does NOT enable the drive
# (that still requires an explicit lift power request), it only resets the
# controller state machine.
#
# Usage:
#   ./tools/hardware/lift_clear_fault.sh                 # clear and verify
#   ./tools/hardware/lift_clear_fault.sh --status-only   # just report
# =============================================================================
set -u

CONTROLLER_MANAGER="${LIFT_CONTROLLER_MANAGER:-/lift/controller_manager}"
CONTROLLER="${LIFT_CONTROLLER_NAME:-lift_controller}"
STATUS_ONLY=false
[[ "${1:-}" == "--status-only" ]] && STATUS_ONLY=true

step() { printf '\n== %s\n' "$1"; }
die() { printf 'ERROR: %s\n' "$1" >&2; exit 1; }

command -v ros2 >/dev/null 2>&1 || die "ros2 not on PATH; source /opt/ros/humble/setup.bash and the workspace first"

# The ros2 CLI daemon keeps whatever DDS context it was started with; after a
# DDS config change it answers "xmlrpc Fault: !rclpy.ok()" and every query comes
# back empty, which looks exactly like "the controller manager is unreachable".
# Stopping it makes the commands below use direct discovery.
ros2 daemon stop >/dev/null 2>&1 || true

step "controllers on ${CONTROLLER_MANAGER}"
ros2 control list_controllers --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null || \
  die "controller manager ${CONTROLLER_MANAGER} not reachable (is the lift stack running?)"

$STATUS_ONLY && exit 0

step "deactivate ${CONTROLLER}"
ros2 control switch_controllers --controller-manager "${CONTROLLER_MANAGER}" \
  --deactivate "${CONTROLLER}" || die "deactivate refused (see the reason printed above)"

step "activate ${CONTROLLER}  (this clears a latched Mode::fault)"
ros2 control switch_controllers --controller-manager "${CONTROLLER_MANAGER}" \
  --activate "${CONTROLLER}" || die "activate refused (see the reason printed above)"

step "verify"
ros2 control list_controllers --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null
cat <<'NOTE'

Expect a warning in the lift controller log:
  [LIFT_FAULT] state=cleared_by_activate previous_mode=fault last_code=gate_lost_during_motion ...
If it appears, the latch is gone -- `lift power on` should work again.
If the drive still refuses, check the DRIVE side: read 603Fh (error code) and
6041h (status word).  A non-zero error code or a drive that never reaches
Operation Enabled means a protective latch that only a drive power cycle clears.
NOTE

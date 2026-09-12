# Standalone Hardware Tools

This directory contains the ROS-free EtherCAT hardware console for `heavy_v1`.

```bash
cd /home/yuling/xiaweiji/joint_controller
sudo ./tools/hardware/run_hardware_console.sh
```

The console configures Master0=`enp3s0` for the left arm, Master1=`enp4s0`
for the right arm, and Master2=`enp5s0` for the lift. It then lets the operator
select the interactive arm test or the lift CLI. `--check` is read-only and
`--self-test` only validates the local YAML and shared-memory ABI.

For the complete lower-computer discovery preflight covering only the two arms
and the lift, run:

```bash
./tools/hardware/check_hardware_link.sh
```

It checks the configured Master0/1/2 interfaces, `/dev/EtherCAT0..2`,
7/7/1 discovered slaves, and the lift vendor/product/PDO preflight. It does
not inspect the base, waist, hands, sensors, or any other serial device, and
it never enables drives or sends a control command. The default run temporarily
brings up the three dedicated NICs and starts EtherLab only for discovery, then
stops EtherLab and restores the NIC state. The complete mode requests sudo once
when needed. Use `--report` to write the Markdown report elsewhere; use
`--read-only` to skip the EtherLab start/stop phase.

`--bring-up` is retained as an alias for the complete discovery mode. To run
only the old interface-state check, use:

```bash
./tools/hardware/check_hardware_link.sh --read-only
```

The complete mode refuses to run when the EtherLab service/masters or
`igh_driver` is already running, assigns no IP, creates no bridge, and restores
each interface's original administrative state on exit or Ctrl-C.

Select `arm` to start the legacy IGH driver without ROS. The first screen shows
all 14 joints from top to bottom. Use Up/Down and Enter to select one joint.
The control screen uses `E` to enable the arm drive group, `W`/`S` for slow
positive/negative jog steps, `H` to return the selected joint to its logical
zero, `Z` to save the selected joint's current position as its logical zero,
Space to hold the measured position, and `Q` to disable and return to the
14-joint screen. The current legacy driver has one global power request, so
`E` enables all 14 drives while `W`/`S` changes only the selected joint target.
`jog_step_rad` in `hardware_io.yaml` controls the displacement per `W`/`S`
press and defaults to `0.005` rad. Arm zero offsets are saved to the configured
`zero_offset_file`. Before `E`, the console waits for valid feedback from all
14 drives and synchronizes every target to its actual position.
`max_speed_rad_s` is a global target slew limit shared by `W`/`S` and `H`.
When a commanded direction has a position error but no measured progress for
`limit_stall_timeout_s`, the console clamps that target to the actual position
and ignores more presses in the blocked direction until the opposite direction
is selected. `limit_progress_tolerance_rad` controls the smaller encoder
increment that counts as progress for ZeroLegacy drives.
The terminal page is redrawn only after a key action, not continuously.

Files in this directory:

- `run_hardware_console.sh`: single user-facing entry point.
- `ethercat_hardware_test.py`: configuration, EtherLab setup, and arm test logic.
- `hardware_io.yaml`: EtherCAT master and heavy_v1 binding configuration.
- `test_ethercat_hardware.py`: offline tests for the binding and ABI.

每次启动 `igh_driver` 都会写入当前会话的
`~/.ros/log/<runtime-id>/runtime/heavy_v1_igh_driver-*.log`；最近一次日志通过同目录的
`heavy_v1_igh_driver.latest.log` 查看，不会再把多次运行追加到 `/tmp` 的同一个文件。

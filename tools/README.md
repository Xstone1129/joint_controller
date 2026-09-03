# Lift EtherCAT CLI

`lift_ethercat_cli` is a standalone lift diagnostic and motion tool. It does
not start ROS, a controller manager, or any other workspace process. It reads
the EtherLab system configuration to select Master2's configured interface,
discovers the lift slave identity, and runs its own 100 Hz PDO loop.

Build this tool directly, then run only this program as root while no other
process owns EtherCAT Master2:

```bash
cd /home/user/joint_controller
./tools/build_lift_ethercat_cli.sh
sudo ./tools/lift_ethercat_cli
```

The build script compiles only this CLI and the ROS-independent EtherCAT lift
driver sources. `colcon build` is not required.

The unified hardware console is the hardware entry point. It configures all
three EtherCAT masters, then starts the standalone lift CLI when `lift` is
selected:

```bash
./tools/build_lift_ethercat_cli.sh
sudo ./tools/hardware/run_hardware_console.sh lift
```

The console does not start ROS. The low-level CLI binary remains in `tools/`
because the unified console invokes it for lift testing.

The CLI uses immediate single-key commands: `e` (enable/hold), `d` (disable),
`z` (set the current encoder position as zero), `u`/`j` (hold-to-jog up/down,
release-to-stop), arrow up/down (run a 100 mm position task), space (stop), and
`q` (quit). The configured travel range is `[-1 m, 0 m]`, so after `z` at zero,
use down/`j` to move into the valid range.

Its default profile is `0.015 m/s` maximum velocity and `0.033333333 m/s^2`
acceleration. These values use the same effective carriage travel as the ROS
lift path: 10 mm screw lead through a 3:1 reduction, or 3.333333333 mm per
motor revolution. Override them for a test with `--speed MPS` and `--accel MPS2`.

The current configured coordinate range is `[-1.0, 0.0] m`. After pressing
`z`, the current point becomes `0 m`, which is the configured positive limit.
Use down/`j` to move into the valid range; up/`u` is correctly clamped at that
upper limit. `e` only enables and holds the current position.

The tool always starts disabled. It sends zero CSV velocity and CiA402 shutdown
on `stop`, exit, Ctrl-C, or an EtherCAT PDO failure.

## Unified EtherCAT hardware console

The combined ROS-free heavy_v1 console is in `tools/hardware/`. There is only
one hardware console entry point; use:

```bash
sudo ./tools/hardware/run_hardware_console.sh
```

It configures the three EtherCAT masters and provides the arm or lift test
selection. Run `./tools/hardware/run_hardware_console.sh --check` for a
read-only host check, or `./tools/hardware/run_hardware_console.sh --self-test`
for the offline configuration and shared-memory ABI tests.

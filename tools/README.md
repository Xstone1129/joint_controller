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

Use `run_lift_ethercat_cli.sh` as the hardware entry point. It reads
`MASTER2_DEVICE` from the EtherLab system configuration, brings that NIC up,
starts EtherLab when `/dev/EtherCAT2` is absent, verifies the Master2 device,
and then execs the standalone CLI:

```bash
./tools/build_lift_ethercat_cli.sh
sudo ./tools/run_lift_ethercat_cli.sh
```

The launcher does not start ROS. It also does not stop EtherLab on exit,
because stopping the service would tear down the other configured EtherCAT
masters used by the robot arms.

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

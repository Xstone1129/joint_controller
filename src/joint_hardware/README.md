# joint_hardware

This package provides the `joint_hardware/JointHardware` generic plugin, the
`joint_hardware/LiftHardware` EtherCAT/CiA 402 CSV plugin, and the
`joint_hardware/WaistHardware` USB-FDCAN plugin.

The lift has no STM32 serial/CAN dependency. `LiftEthercatBackend` is the SDK-neutral
boundary; `MockLiftEthercatBackend` is used for offline tests, while
`EtherLabLiftEthercatBackend` uses the open-source IgH/EtherLab master when its
optional `ecrt.h` development package is present. `UnavailableLiftEthercatBackend`
and `LiftEthercatSdkAdapter` remain explicit placeholders for other masters and
never claim a connection.

Use the plugin in a ros2_control description with:

```xml
<hardware>
  <plugin>joint_hardware/JointHardware</plugin>
  <param name="simulate">true</param>
</hardware>
```

The default for `simulate` is `false`, so a real hardware launch fails during
activation unless `ethercat_backend=etherlab` is selected and the EtherLab master
is running.

## Lift notes

### Real-hardware checks

After starting `lift_ethercat.launch.py` with the real backend, the standalone
smoke test can be copied to a test host and run with only the ROS 2 CLI:

```bash
python3 test/test_lift_real_motion.py --direction up --duration 1 --speed-rpm 10
```

It requires typing `MOVE` unless `--yes` is supplied, sends a bounded jog, and
always requests zero velocity, controller STOP, and brake lock on exit.  The
brake service must be enabled in the launch/configuration (`brake_control_enabled:=true`)
unless `--no-enable` is used.

For an interactive continuously refreshed terminal inside the workspace:

```bash
ros2 run joint_hardware lift_keyboard_teleop
```

Press `e`/`d` to request enable/disable, hold the arrow keys to jog, press
SPACE to stop, and `q` to stop, disable, and exit.  Enable requests still obey
the hardware brake-control parameter.

The lift uses CSV (`6060h=9`, confirmed by `6061h=9`) and the following PDOs:

- RxPDO: `6040h`, `60FFh`, optional `60B2h`.
- TxPDO: `6041h`, `603Fh`, `6061h`, `6064h`, `606Ch`, reserved `60FDh` input.

`608Fh`, `6091h` and `6092h` are read during configure. The position and velocity
conversion uses the validated 6091/6092 feed ratio, a 10 mm/rev screw, the 3:1
reduction, and `lift_sign=-1`; the effective carriage travel is 3.333333333
mm/rev at the motor. The 17-bit encoder count (`131072`) is not used as the
60FFh unit. The mechanical velocity limit is 1440 rpm (0.080 m/s), and
software limits are `[-1.0, 0.0] m`.
The effective command units/rev follow the manual: nonzero `2008h(P00.08)` wins;
otherwise `6092:01` wins when it differs from `608F:01`, and `6091:01/02` is used
only when those two values are equal.

The LD3M manual exposes one 8-byte RPDO and one 17-byte TPDO through `5004h`. The
package includes a raw frame layer in `lift/ethercat_frame.hpp` and
`lift/ethercat_frame.cpp`:

- Ethernet EtherType is `0x88A4` (the helper encodes the 14-byte Ethernet header).
- The EtherCAT frame header is a little-endian 11-bit payload length plus frame type `1`.
- Datagram headers encode `LWR/LRD` and the other standard commands, 32-bit address,
  11-bit data length, interrupt field, optional `more` flag, payload, and 16-bit
  Working Counter.
- `lift/ethercat_coe.*` encodes the six-byte mailbox header, CoE service `2/3`, and
  expedited SDO upload/download/abort messages in little-endian form.
- `lift/pdo_mapping.*` applies the manual's Pre-Operational sequence to map
  `1601h: 6040/60FF/60B2` and `1A00: 6041/603F/6061/6064/606C/60FD`, then assigns
  them through `1C12h/1C13h`. The plan is executed only during configuration.

Logical PDO addresses are assigned by the master's FMMU/SyncManager setup, not by
the drive manual. The raw frame/CoE helpers are intentionally only codecs; they do
not implement a complete raw-socket master. The optional EtherLab adapter registers
the fixed PDO entries with an IgH domain, calls `ecrt_master_receive/process` and
`ecrt_domain_queue/send` in the 100 Hz cycle, and performs SDO transfers only during
configuration. It checks the master link, slave OP state, domain working counter and
PDO freshness before allowing a non-zero 60FFh value onto the wire.

The cyclic backend exposes separate `read_pdo()` and `write_pdo()` operations. The
ros2_control `read()` receives the previous frame and `write()` queues the newly
calculated command immediately in the same 10 ms cycle, so ordinary position and
velocity commands do not wait for an additional EtherCAT period. The compatibility
`exchange_pdo()` helper remains available for offline adapters and frame tests.

## EtherLab setup

Build output prints whether the optional adapter was enabled. Set
`ethercat_backend: etherlab`, `ethercat_master_index`, `slave_alias`,
`slave_position`, `slave_vendor_id` and `slave_product_code` from the actual
`ethercat slaves` scan. The LD3M manual does not specify the vendor/product IDs;
do not guess them. The EtherLab kernel master selects the physical NIC through its
own `/etc/ethercat.conf`/sysconfig, so `ethercat_interface` is retained as a
configuration/documentation field but is not a userspace socket selector.

The drive's ESI file must also provide the vendor-specific Distributed Clock
`AssignActivate` word. Set `dc_assign_activate` only after confirming that value;
zero deliberately disables guessed DC configuration. The 10 ms cycle is enforced
by `LiftHardware` and passed to EtherLab as its send interval.

The installed host helper checks the interface without changing system state. On
the robot computer Master0 and Master1 are reserved for the two arm chains; the
lift uses Master2 on `enp5s0`. The helper preserves those arm bindings, backs up
the EtherLab sysconfig and verifies `/dev/EtherCAT2` before accepting the change:

```bash
source /home/user/joint_controller/install/setup.bash
ros2 run joint_hardware lift_ethercat_host_setup.sh enp5s0 --check
sudo /home/user/joint_controller/install/joint_hardware/lib/joint_hardware/lift_ethercat_host_setup.sh \
  enp5s0 --apply 2
```

Before a first enable, run the following as root on the controller computer:

```bash
ethercat master
ethercat slaves
ethercat pdos -p <position>
```

The package also installs a read-only preflight helper. It prints the master,
selected slave identity and PDO mapping, then emits the vendor/product launch
arguments without changing any EtherCAT state:

```bash
ros2 run joint_hardware lift_ethercat_preflight.sh 2 0
```

For a complete offline smoke test, start the included launch with its safe
defaults:

```bash
ros2 launch joint_hardware lift_ethercat.launch.py backend:=mock
```

For a real drive, first replace the zero vendor/product values with the output
of the preflight command, confirm the PDO layout and ESI Distributed Clock
word, and confirm the measured P04.38 release delay. Then start with the
brake request service authorized but the actual brake request still disabled:

```bash
ros2 launch joint_hardware lift_ethercat.launch.py \
  backend:=etherlab master_index:=2 slave_position:=0 \
  slave_vendor_id:=0x<VENDOR> slave_product_code:=0x<PRODUCT> \
  ethercat_interface:=enp5s0 \
  command_units_per_rev:=<CONFIRMED_UNITS_PER_REV> \
  brake_control_enabled:=true brake_release_wait_ms:=<P04.38_MS> \
  brake_p04_37_ms:=150 brake_p04_39_rpm:=30 brake_p06_14_ms:=500
```

Observe `/joint/lift/driver_status` and the joint state until the link is
Operational, the working counter is complete, `6061h=9`, and PDO feedback is
fresh. Only then request the automatic drive-controlled brake release:

```bash
ros2 service call /lift_brake_command std_srvs/srv/SetBool "{data: true}"
```

`brake_control_enabled` authorizes Operation Enabled requests but does not
release the brake at startup. Normal motion is owned by `LiftController`, which
writes `joint_motor/power_enable` through ros2_control. `/lift_brake_command`
remains a compatibility and safety entry point; an external disable is latched
and cannot be overwritten by a command that was already high. A new false-to-
true command edge is required before re-enable. A position command is still
clamped to `[-1.0, 0.0] m` and the outer loop never exceeds 1440 rpm (0.080 m/s).

The launch file exposes the validated unit and safety values directly. If the
drive reports a command-unit value other than 10000, pass the confirmed value
from `6092h`/`2008h` as `command_units_per_rev:=...`; a mismatch is rejected at
configuration rather than silently changing the scale. The optional 60FD limit
inputs are disabled by default. To enable them, set
`limit_switch_enabled:=true` and explicitly provide
`limit_switch_positive_bit` and/or `limit_switch_negative_bit` from the actual
LD3M DI assignment; no bit number is guessed by this package.

## Unified motion controller

`joint_hardware/LiftController` is the only normal-motion owner. Absolute,
relative, `JointTrajectory`, streaming and Jog inputs all pass through the same
one-axis Ruckig OTG and claim the synchronized `position`, `velocity`,
`acceleration` and `power_enable` command interfaces of `joint_motor`.
`LiftHardware` then applies
the second-layer P/D outer loop, terminal slowdown, overshoot protection, the
1440 rpm limit, slope limit and final software-direction checks before converting
the result to `60FFh` units/s.

The nominal limits are:

| Parameter | Value | Unit |
| --- | ---: | --- |
| control/EtherCAT period | 10 | ms |
| software travel | -1.0 to 0.0 | m |
| screw lead | 10 | mm/rev |
| reduction ratio | 3:1 | dimensionless |
| effective carriage travel | 3.333333333 | mm/rev |
| `lift_sign` | -1 | dimensionless |
| motor speed | 1440 | rpm |
| linear velocity | 0.080 | m/s |
| Ruckig acceleration | 0.033333333 | m/s^2 |
| Ruckig jerk | 0.4 | m/s^3 |
| default velocity scale | 0.80 | dimensionless |
| hardware speed slope | 600 | rpm/s (0.033333333 m/s^2) |
| Jog publish rate | 50 | Hz |
| Jog keepalive timeout | 0.45 | s |

An absolute or relative command can be sent with:

```bash
ros2 service call /joint/lift/command robot_control_msg/srv/SelectedJointControl \
  "{joint_names: [joint_motor], values: [-0.050], relative: false, vel: 0.010, acc: 0.02}"

ros2 service call /joint/lift/command robot_control_msg/srv/SelectedJointControl \
  "{joint_names: [joint_motor], values: [0.001], relative: true, vel: 0.005, acc: 0.02}"
```

`vel` selects a per-command velocity scale but never exceeds the configured
0.080 m/s mechanical limit. A positive `acc` can tighten, but never raise, the
global 0.033333333 m/s^2 acceleration limit; jerk remains bounded by the controller
YAML. A `JointTrajectory` for `joint_motor` is accepted on
`/joint/lift/trajectory`; streaming `JointTrajectoryPoint` targets use
`/joint/lift/stream`. Do not publish directly to a forward command controller or
call an EtherCAT backend from application code.

For manual Jog, start the terminal helper. The first arrow event starts the Jog;
holding the key supplies a 50 Hz keepalive, and no key event for 150 ms requests
a bounded Ruckig stop. With the 10 mm/rev screw, 30 rpm is 0.005 m/s:

```bash
ros2 run joint_hardware lift_keyboard_teleop --ros-args -p speed_rpm:=30.0
```

Up increases the ROS position and Down decreases it. Space requests a soft stop;
`Q` requests the same stop and exits. A direction reversal first stops the old
direction. Jog loss, terminal exit and keepalive expiry all converge to HOLD and
then request controlled disable so the LD3M closes its automatic brake. A new
command always runs through the enable/brake gate again.

Before any trajectory clock or Ruckig target advances, the controller holds the
measured position, writes `power_enable=true`, and requires fresh PDO, EtherCAT
Operational, complete WKC, zero `603Fh`, `6061h=9`, the confirmed
`power_enable` state interface, CiA 402 Operation Enabled and inferred brake
release for at least 0.1 s. Driver status older than 2 s cannot satisfy this gate.
The trajectory starts at `t=0` only after the gate. After the final position and
actual velocity remain stable for 1 s, the controller writes
`power_enable=false` for controlled disable and automatic brake closure.

The lift uses its own `/lift/controller_manager`, so it does not collide with
the dual-arm `/arm/controller_manager`. The managed SIM workspace starts the
mock lift automatically. REAL startup requires Master2, exactly one lift slave,
and a successful read-only identity/PDO preflight; it starts the lift with
`LIFT_BRAKE_CONTROL_ENABLED=false` by default and never powers it automatically.
After the physical brake wiring and P04.38 value are verified, a systemd
override may set `LIFT_BRAKE_CONTROL_ENABLED=true` together with a non-negative
`LIFT_BRAKE_RELEASE_WAIT_MS`; setting only the boolean is rejected.

## Stop and emergency stop

`/joint/lift/stop` and `/joint/lift/hold` are soft-stop services. They cancel the
active trajectory/Jog/stream source, request zero velocity through Ruckig, preserve
the velocity/acceleration/jerk limits, latch HOLD at measured position, then request
controlled disable. They do not replay the old target or immediately reverse after
an overshoot.

```bash
ros2 service call /joint/lift/stop std_srvs/srv/Trigger "{}"
```

`/joint/safety/estop` is a separate locked emergency-stop input. On the next
100 Hz cycle it rejects every nonzero `60FFh`, clears normal and debug motion,
requests CiA 402 Quick Stop (`6040h=0x000B`), and then changes to shutdown
(`0x0006`) after the configured low-speed/deadline condition. It remains latched
across communication recovery. Brake release, reset velocity and zero requests
are rejected while latched.

```bash
ros2 service call /joint/safety/estop std_srvs/srv/Trigger "{}"
ros2 service call /joint/safety/reset std_srvs/srv/Trigger "{}"
```

Safety reset succeeds only with Operational EtherCAT, fresh PDO, complete WKC,
CSV mode, no active drive error, near-zero feedback speed and cleared upper
commands. It returns to HOLD at the measured position and never restores the old
trajectory. This workspace has no separate `SafetyState` message or supervisor;
the two services are the integration boundary that a global safety supervisor
must call.

Software Quick Stop is not a safety-certified emergency stop. Personnel safety
requires a wired physical emergency stop, drive STO, independent vertical-axis
fall protection and validation of the actual P05/P06 stopping configuration.
Loss of EtherCAT is reported as loss of control, not as proof that the axis has
stopped safely.

## Waist hardware

`WaistHardware` owns exactly one joint, `joint_qugan`, and exports synchronized
position, velocity and acceleration command interfaces plus position and velocity
state interfaces. The bridge transport uses the firmware packet format with CRC8,
requires a successful CAN bitrate acknowledgement, and reports link loss instead
of continuing to accept commands. Vendor feedback `0x300 | node_id`, CANopen SDO
responses, position conversion, command clamping and manufacturer error bits are
handled in the package.

Edit the USB device, node ID and bridge channel for the actual installation, then
start the standalone controller with:

```bash
source /home/user/joint_controller/install/setup.bash
ros2 launch joint_hardware waist_hardware.launch.py \
  waist_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_307A335D3435-if00 \
  waist_node_id:=1 waist_channel:=2
```

Activation opens the bridge, confirms the configured 5 Mbit/s waist CAN-FD channel,
reads the current position, enables at that measured target, and only then accepts
trajectory commands. Do not run this launch until the mechanism is supported and
the node/channel/direction/limits have been checked. Deactivation sends the vendor
disable/brake frame and `6040h=0` before closing the serial link.

Monitor feedback and send a bounded trajectory with:

```bash
ros2 topic echo /joint/waist/driver_status
ros2 topic echo /joint_states

ros2 action send_goal /waist_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_qugan], points: [{positions: [0.1], velocities: [0.0], accelerations: [0.0], time_from_start: {sec: 3}}]}}"
```

The maintenance services are `/waist_clear_error` and
`/waist_set_zero_position`. Setting zero writes the vendor object `2531h:00` while
disabled and leaves the drive disabled, so it must only be used with the mechanism
at a confirmed mechanical zero. Runtime limits and direction are configured in
`config/waist_hardware.yaml`; the same position limits must be kept in the robot
URDF and trajectory controller configuration.

Use these status topics during testing:

```bash
ros2 topic echo /joint_states
ros2 topic echo /joint/lift/driver_status
ros2 topic echo /joint/lift/control_status
```

The driver JSON includes PDO age/freshness, link/WKC state, initialization,
CiA 402 status/error/mode, inferred brake state, estop/Quick Stop state, measured
and commanded position/velocity/acceleration, filtered velocity, target rpm,
`60FFh` units, reset takeover flags, zero-offset validity and fault reason. JSON
construction and publication occur on a non-real-time timer, not in `read()` or
`write()`.

The compatibility brake service remains available. `false` is always a fail-safe
controlled stop request:

```bash
ros2 service call /lift_brake_command std_srvs/srv/SetBool "{data: false}"
```

The next 10 ms cycle sends zero `60FFh` with Quick Stop, then sends shutdown after
606Ch is below the P04.39 threshold or P06.14 expires. The drive, not this
software, controls the BR+/BR- coil and P04.37 power-off delay.

Confirm the expected vendor/product, PDO mapping and OP transition with no
mechanical load. Then connect the LD3M `EtherCAT IN` to the master, leave `OUT`
unused for a single drive, and start the ROS 2 stack with the configured YAML/Xacro.
The first command cycles are zero velocity; CiA 402 must reach Operation enabled
before the outer loop can request motion. The Quick Stop optimization is covered
by the Mock backend and must be rechecked on the real drive after any change to
P04.37/P04.39/P06.14 or P05.06/P05.10.

`/lift_brake_command` keeps its original `SetBool` contract. `true` requests CiA 402
Operation enabled and lets the LD3M-EC drive release BR+/BR-; `false` is always
accepted as a fail-safe lock request and runs the bounded Quick Stop/disable path.
`brake_unlocked` is only an inference from CiA 402 state and the configured P04.38
release delay, never a mechanical feedback claim. The status JSON also exposes
`brake_stop_phase` (`idle`, `quick_stop`, or `disabled`) and
`brake_stop_timeout`. Do not drive the brake through 60FEh or text commands.
Confirm P04.37/P04.38, P04.39/P06.14 and P05.06/P05.10 on the drive before
connecting a vertical load.

Automatic fault reset is limited by `max_fault_reset_attempts` and separated by
`fault_reset_backoff_ms` (100 ms by default). During the backoff the state machine
keeps a zero target and shutdown control word. Attempts and exhaustion are visible
in `/joint/lift/driver_status`; automatic reset is disabled while estop is latched.

The status also reports `motion_blocked` and the current
`mode_mismatch_cycles`. `6061h` mode transitions are tolerated for the bounded
`mode_mismatch_debounce_cycles` window (five 10 ms cycles by default), but a
persistent mismatch while an explicit brake release is requested latches a safe
stop. A brake release request does not clear that latch; restart the controller or
use the explicit safety-reset service after the drive is back in CSV/Operational
with a fresh, stationary feedback sample.

### Recovering from a software-position-limit violation

Crossing `position_min_m` or `position_max_m` still causes an immediate zero
velocity, CiA 402 Quick Stop, and controlled disable. It does not permanently
lock both directions: after an explicit motion request and brake gate, only a
velocity command pointing back into the valid range is accepted. Recovery is
limited by `position_limit_recovery_max_rpm` (300 rpm by default for supervised
recovery from the currently uncalibrated lower position). An enable-only
request, HOLD target, outward command, and `/lift_reset_velocity` cannot move an
axis while it is outside the software range. Jump, overspeed, EtherCAT, mode,
and drive faults remain latched safety faults.

The status JSON exposes `position_limit_violation`, `limit_recovery_active`,
and `position_limit_recovery_max_rpm`. ROS uses a host-managed coordinate
frame by default. The persisted offset is subtracted from LD3M `6064h`; this
does not modify the encoder or its absolute-position calibration.

During `configure`, the EtherCAT backend reads the manual's corresponding CoE
objects `2437h`, `2438h`, `2439h`, `2614h`, `2506h` and `2510h`. The configured
P04.37/P04.39/P06.14 values must match the drive. P04.38 must be supplied as a
confirmed launch parameter; leaving it at `-1` deliberately keeps motion inhibited
even though the object is readable. `drive_default` for P05.06/P05.10 records the
drive value without guessing a stop mode.

Wire the motor brake to the LD3M-EC BR+/BR- output and provide the specified 15--24 V
logic/brake supply with enough continuous and inrush capacity for the coil. Verify the
M17 magnetic encoder/brake cable pinout and brake polarity against the actual harness
and drive manual; this software does not infer polarity or force a digital output. A
vertical axis can fall when the brake supply, drive, or wiring is faulty, so add an
independent mechanical fall-arrest measure before real testing.

`/lift_reset_zero` stores a host-side `zero_offset_units` record with an atomic file
replace from the non-real-time service thread after the controlled disable is observed;
the 100 Hz read/write path only applies the completed offset. It intentionally does
not change the drive's encoder state.

The LD3M maintenance services are:

```bash
# Run the configured CiA 402 HM sequence, then return to CSV.
ros2 service call /lift_home std_srvs/srv/Trigger "{}"

# Set the current stationary platform position as ROS zero. This preserves the
# drive encoder state and leaves the axis disabled.
ros2 service call /lift_set_zero_position std_srvs/srv/Trigger "{}"

# Only at a confirmed mechanical zero, stage LD3M multi-turn calibration
# (2015h=9). The drive must then be power-cycled/restarted and revalidated.
ros2 service call /lift_set_drive_zero std_srvs/srv/Trigger "{}"
```

`/lift_reset_zero` is an alias for the host-coordinate operation. It requires
fresh stationary feedback and a controlled disable. `/lift_set_drive_zero` is
absolute-encoder maintenance only: it must be used solely after mechanical
homing, stays disabled, and does not report a `6064h` value as an immediate
completion check. HM uses
`6060h=6`, `6098h`, `6099h:01/:02`, `609Ah` and `607Ch`; completion requires
`6041h` bits 10 and 12. The configured `homing_method` must match the installed
HOME-SWITCH/limit wiring. A failed or timed-out operation leaves the drive disabled.
The `driver_status` JSON reports `homing_active`, `homing_complete` and
`homing_failed`.
The managed REAL launch persists this record at
`~/.local/state/joint_controller/lift_zero_offset.cfg`; do not place a production
calibration under `/tmp`, because it would disappear after reboot.
Corrupt or mismatched records stop configuration rather than silently changing the
coordinate frame. No physical limit switch is assumed by default; the vertical
axis still has a fall hazard without a correctly powered and wired BR+/BR- brake.

For an offline launch, include `description/lift.ros2_control.xacro` and set the
hardware parameter `ethercat_backend` to `mock`. A future non-EtherLab master only
needs to implement the methods declared in `lift/ethercat_backend.hpp`:
initialize, configure_slave, read_sdo, write_sdo, start, stop, read_pdo,
write_pdo, exchange_pdo, link_state, pdo_fresh, working_counter and
error_message.

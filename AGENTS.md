# Lower Workspace Agent Instructions

## Numeric Representation

- Keep all continuous physical and control quantities floating point end to end: joint and Cartesian positions, velocities, accelerations, jerk, effort/torque/force, distances, angles, dimensions, rates, durations, timeouts, limits, tolerances, gains, thresholds, ratios, scales, and progress values.
- Prefer ROS `float64`, C++ `double`, Python `float`, and floating-point YAML defaults such as `0.0` and `100.0`. Validate finite values and do not convert logical motion quantities through integer parsing, rounding, or truncation.
- Retain integers only for intrinsically discrete values or external protocol layouts: booleans, enum/state/mode values, IDs, PIDs, sequence/generation/revision values, indexes/lengths/counts, masks, error/status codes, byte widths, and raw EtherCAT/CAN register or wire-unit encodings.
- `float32` may remain at a required vendor or ROS wire boundary; new application-facing continuous fields should use `float64` and be handled as `double` internally where practical. Preserve explicit range checks when converting to raw hardware units.
- The lift logical path must preserve floating-point position/velocity/acceleration values through command parsing, gateway forwarding, controller logic, and feedback. Only the final device-unit serialization may use integral types.

## Validation

- When changing a shared `robot_control_msg` interface, make the matching change in the upper workspace and rebuild both sides with `--symlink-install --parallel-workers 12`.
- Do not use real-hardware motion for numeric-type validation. Validate logical motion and protocol behavior with isolated tests or simulation, and preserve the existing fail-closed safety behavior.

## Execution Environment

This repository is checked out on more than one machine, and the same rules must hold whether the agent runs **remotely** (from the dev box, through an sshfs mount) or **locally** (on the machine that owns the code). Work out which context you are in before you touch anything.

### Machines

| Machine | ssh alias | Repo path | ROS |
| --- | --- | --- | --- |
| `5060ti` — agent host / dev box | — | no `~/joint_controller`; it mounts the others under `/home/yuling/remote/` | Jazzy |
| `master` — upper machine | `master` | `~/joint_controller` | Humble |
| `slave` — lower machine, drives the hardware | `slave` | `~/joint_controller` = `/home/user/joint_controller` | Humble |

From `5060ti`: `/home/yuling/remote/slave` is `slave:/home/user`, `/home/yuling/remote/master` is `master:/home/yuling`, `/home/yuling/remote/aliyun` is `aliyun:/`.

### Which context am I in?

- `hostname` — `5060ti` is the agent host; `user-Default-string` is the lower machine (`whoami` = `user`).
- A `pwd` under `/home/yuling/remote/<alias>/` means a **remote** session: the tree belongs to that other machine.
- `stat -f -c %T <workspace>` prints `fuseblk` for an sshfs mount and `ext2`/`ext4` for a real local disk.
- `ls /opt/ros` — `humble` on `master`/`slave`, `jazzy` on `5060ti`. Never source or mix the two.

### Remote session (through the mount)

- The mounted tree **is** the remote machine's live filesystem: every write takes effect there immediately, and there is no separate copy to sync back.
- **Never run a build through the mount.** cmake records the mount path instead of the remote machine's native path, so `build/<pkg>/CMakeCache.txt` no longer matches, the build aborts before compiling, and whatever it does write lands on the remote machine carrying paths that are invalid there. (This once replaced the lower machine's `install/setup.bash` prefix chain with this host's Jazzy/`junior_ws` chain.)
- Build natively over ssh instead:

  ```bash
  ssh -o ClearAllForwardings=yes slave 'bash -c "cd ~/joint_controller && source /opt/ros/humble/setup.bash && colcon build --packages-select <pkg> --symlink-install --parallel-workers 12"'
  ```

  `-o ClearAllForwardings=yes` is needed because the `slave` alias also defines `LocalForward 7890`, which the existing sshfs mount already holds.
- Prefer `ls`/`grep` on explicit paths; a recursive scan of a mount is slow enough to time out.

### Local session (on `master` or `slave`)

- The workspace is that machine's own `~/joint_controller` on a local disk, so build here directly, with its own ROS:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --parallel-workers 12          # or --packages-select <pkg> when one package changed
  source install/setup.bash
  ```

- Restart the ROS stack afterwards so rebuilt libraries are actually loaded.

### Both contexts

- `build/`, `install/` and `log/` hold generated artifacts. Never hand-edit or delete them to fix a problem — rebuild instead; `install/` is gitignored, so git cannot restore it. `--packages-select` still regenerates the prefix-level `setup.*`/`local_setup.*`, and `local_setup.bash` enumerates every installed package at source time.
- Shared `robot_control_msg` interface changes must be mirrored to the upper workspace on `master` and both sides rebuilt (see Validation).
- Access details, including the lower-machine credentials, live in `.agents/slave-access.md` (untracked — see `.gitignore`).

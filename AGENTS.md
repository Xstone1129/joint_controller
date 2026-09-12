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

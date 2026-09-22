"""GENERATED - do not edit by hand.

Source of truth: src/joint_hardware/src/lift/cia402.cpp::parse_cia402_status
Regenerate:  python3 tools/hardware/generate_lift_console_states.py

This file used to be an inlined copy of that table inside lift_console.py, and a
missing entry (0x0028, the LD3M powered-but-disabled status word 0x0638) made the
hardware console report the state as unknown while the runtime knew it, which sent
a whole debugging session after a non-existent tool bug.
"""

CIA402_STATE_MASK = 0x006F

CIA402_STATE_NAMES = {
    0x0000: "not_ready_to_switch_on",
    0x0040: "switch_on_disabled",
    0x0021: "ready_to_switch_on",
    0x0023: "switched_on",
    0x0027: "operation_enabled",
    0x0007: "quick_stop_active",
    0x000F: "fault_reaction_active",
    0x0028: "fault",
    0x0008: "fault",
}

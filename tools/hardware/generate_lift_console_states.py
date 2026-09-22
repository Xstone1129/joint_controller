from pathlib import Path
src = Path("/home/user/joint_controller/src/joint_hardware/src/lift/cia402.cpp").read_text(encoding="utf-8")
import re
import re as _re
_m = _re.search(r"switch \(status_word & (0x[0-9a-fA-F]+)U\)", src)
assert _m, "could not find the parse_cia402_status mask in cia402.cpp"
STATUS_MASK = int(_m.group(1), 16)
body = src[_m.start():]
body = body[:body.index("default:")]
pairs = re.findall(r"case\s+0x([0-9a-fA-F]+)\s*:\s*\n\s*return\s+Cia402State::([a-z_]+);", body)
assert pairs, "no cases parsed"
out = Path("/home/user/joint_controller/tools/hardware/lift_console_states.py")
lines = [
    '"""GENERATED - do not edit by hand.',
    "",
    "Source of truth: src/joint_hardware/src/lift/cia402.cpp::parse_cia402_status",
    "Regenerate:  python3 tools/hardware/generate_lift_console_states.py",
    "",
    "This file used to be an inlined copy of that table inside lift_console.py, and a",
    "missing entry (0x0028, the LD3M powered-but-disabled status word 0x0638) made the",
    "hardware console report the state as unknown while the runtime knew it, which sent",
    "a whole debugging session after a non-existent tool bug.",
    '"""',
    "",
    "CIA402_STATE_MASK = 0x%04X" % STATUS_MASK,
    "",
    "CIA402_STATE_NAMES = {",
]
for code, name in pairs:
    lines.append(f'    0x{int(code, 16):04X}: "{name}",')
lines += ["}", ""]
out.write_text("\n".join(lines), encoding="utf-8")
print(f"已生成 {out}  ({len(pairs)} 个状态)")

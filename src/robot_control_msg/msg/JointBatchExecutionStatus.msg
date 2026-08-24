builtin_interfaces/Time stamp

uint8 EXECUTING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint8 REJECTED=3

uint32 command_seq
uint8 state
uint8 source_mode
bool accepted
float64[14] target_joints
string message

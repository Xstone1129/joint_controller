builtin_interfaces/Time stamp

uint8 IDLE=0
uint8 PLANNING=1
uint8 EXECUTING=2
uint8 STREAM_FINISHED=3
uint8 FAILED=4
uint8 REJECTED_BUSY=5

uint8 state
uint32 planned_points
float64 planned_duration_sec
string message

uint8 STATUS=0
uint8 START=1
uint8 STOP=2

# Stack mode selected for START. The default value SIMULATION=0 is safe for
# zero-initialized requests from older callers after they are rebuilt.
uint8 SIMULATION=0
uint8 REAL=1

builtin_interfaces/Time stamp
uint32 seq
uint8 command
uint8 mode
string source

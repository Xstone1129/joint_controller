uint16 PROTOCOL_MAJOR=1
uint16 PROTOCOL_MINOR=0

uint8 MODE_UNKNOWN=0
uint8 MODE_POSITION=1

uint8 WORKSPACE_STOPPED=0
uint8 WORKSPACE_SIMULATION=1
uint8 WORKSPACE_REAL=2

std_msgs/Header header
uint16 protocol_major
uint16 protocol_minor
uint32 layout_crc32
uint64 feedback_sequence
uint64 applied_command_sequence
string active_owner_id
string active_lease_id
uint8 workspace_state
uint8 control_mode
uint16 position_valid_mask
uint16 velocity_valid_mask
uint16 arm_torque_enabled_mask
uint16 resource_fault_mask
bool lift_drive_enabled
float64[15] position
float64[15] velocity
int32[15] fault_code
float64 command_age_ms

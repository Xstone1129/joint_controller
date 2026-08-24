uint16 PROTOCOL_MAJOR=1
uint16 PROTOCOL_MINOR=0

uint8 MODE_HOLD=0
uint8 MODE_FOLLOW_POSITION=1
uint8 MODE_STOP=2

uint8 FIELD_POSITION=1
uint8 FIELD_VELOCITY=2
uint8 FIELD_ACCELERATION=4

std_msgs/Header header
uint16 protocol_major
uint16 protocol_minor
uint32 layout_crc32
string owner_id
string lease_id
uint64 sequence
uint8 mode
uint8 field_mask
float64[15] position
float64[15] velocity
float64[15] acceleration

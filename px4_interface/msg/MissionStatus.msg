# px4_interface/msg/MissionStatus.msg

string mission_id

uint8 STATE_IDLE = 0
uint8 STATE_ACTIVE = 1
uint8 STATE_PAUSED = 2
uint8 STATE_EMERGENCY = 3
uint8 STATE_COMPLETED = 4
uint8 STATE_ABORTED = 5

uint8 state
string[] drone_ids

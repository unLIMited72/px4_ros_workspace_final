# px4_interface/msg/MissionCommand.msg

uint8 CMD_START=0
uint8 CMD_PAUSE=1
uint8 CMD_RESUME=2
uint8 CMD_EMERGENCY_RETURN=3
uint8 CMD_ABORT=4

string mission_id
uint8 command

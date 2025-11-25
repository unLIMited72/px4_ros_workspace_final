# px4_interface/msg/GyroControl.msg

string drone_id

uint8 COMMAND_CONTROL=0
uint8 COMMAND_TAKEOFF=1
uint8 COMMAND_LAND=2
uint8 command

float32 target_altitude_m   # only for TAKEOFF, else 0
float32 yaw_deg             # optional, NaN or 0 if unused
float32 vx_mps              # optional for CONTROL
float32 vy_mps              # optional for CONTROL

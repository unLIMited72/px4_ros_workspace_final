# px4_interface/msg/MissionPlanForDrone.msg

string mission_id
string drone_id
px4_interface/MissionWaypoint[] waypoints

float32 cruise_altitude_m
float32 cruise_speed_mps

uint8 LANDING_HOME = 0
uint8 LANDING_LAST_WAYPOINT = 1
uint8 landing_mode

uint8 SPACING_DISTANCE = 0
uint8 spacing_type
float32 spacing_value

bool sequential_launch
bool order_by_id
bool heading_to_next_wp

# px4_interface/msg/MissionPlan.msg

# (UI -> planner)

# Landing mode
uint8 LANDING_HOME=0
uint8 LANDING_LAST_WAYPOINT=1

# Spacing type
uint8 SPACING_DISTANCE=0   # meters
uint8 SPACING_TIME=1       # seconds

string mission_id
string[] drone_ids
px4_interface/MissionWaypoint[] waypoints

float32 cruise_altitude_m
float32 cruise_speed_mps

uint8 landing_mode
uint8 spacing_type
float32 spacing_value

bool sequential_launch
bool order_by_id
bool heading_to_next_wp

# px4_interface/msg/PX4Status.msg

uint64 timestamp
string drone_id

bool heartbeat
float32 battery_percentage
bool flight_ready
bool armed
uint8 status_in_flight   # 0=Normal, 1=Warning, 2=Danger

float64 latitude
float64 longitude
float32 heading_deg

# enum definitions
uint8 FLIGHT_STATUS_NORMAL = 0
uint8 FLIGHT_STATUS_WARNING = 1
uint8 FLIGHT_STATUS_DANGER = 2

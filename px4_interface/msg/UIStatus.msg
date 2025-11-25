# px4_interface/msg/UIStatus.msg

uint64 timestamp
string[] drone_ids
bool[] heartbeats
float32[] battery_percentages
bool[] flight_readies
bool[] armeds
uint8[] status_in_flights

float64[] latitudes
float64[] longitudes
float32[] heading_degs

# enum definitions (PX4Status와 동일)
uint8 FLIGHT_STATUS_NORMAL = 0
uint8 FLIGHT_STATUS_WARNING = 1
uint8 FLIGHT_STATUS_DANGER = 2

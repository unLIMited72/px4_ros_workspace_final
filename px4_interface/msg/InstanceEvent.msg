# px4_interface/msg/InstanceEvent.msg

string drone_id      # ex: "px4_0", "px4_1"
string event         # "join" or "leave"
uint64 timestamp     # us 

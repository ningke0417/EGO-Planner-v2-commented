std_msgs/Header header
uint64 session
uint64 sequence
uint32 drone_id
float64[3] position
# Latched after this agent has fresh two-endpoint KEEP evidence for every edge.
bool startup_ready

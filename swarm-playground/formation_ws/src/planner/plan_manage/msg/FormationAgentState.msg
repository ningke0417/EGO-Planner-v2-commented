std_msgs/Header header
uint64 session
uint64 sequence
uint32 drone_id
float64[3] position
uint32[4] assignment_versions
uint32 slot_id
# Latched after this agent has fresh two-endpoint KEEP evidence for every edge.
bool startup_ready

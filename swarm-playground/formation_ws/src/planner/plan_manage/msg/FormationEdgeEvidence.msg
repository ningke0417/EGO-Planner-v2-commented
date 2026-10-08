# Both endpoints must grant a fresh lease; this is not a command.
std_msgs/Header header
uint64 session
uint64 sequence
uint32 assignment_generation
uint32 observer_id
uint32 peer_id
uint8 CUT=0
uint8 KEEP=1
uint8 decision
float64 score
float64 visibility
float64 distance

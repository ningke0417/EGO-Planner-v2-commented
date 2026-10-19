# Command execution feedback, not a substitute for measured vehicle odometry.
# WAITING is readiness only: its zero vectors are NOT a vehicle position.
uint8 WAITING=0
uint8 RUNNING=1
uint8 HOLDING=2
uint8 FINISHED=3
std_msgs/Header header
uint64 session
uint64 sequence
int32 drone_id
int32 trajectory_id
time trajectory_start_time
float64 trajectory_duration
float64 elapsed
float64[3] position
float64[3] velocity
float64[3] acceleration
uint8 mode
bool require_replan
bool heartbeat_fresh

# Immutable runtime-sized member assignment transaction. REQUEST/BID gather local-map
# certified cost rows. Only a matching unanimous ACK certificate allows COMMIT.
uint8 REQUEST=1
uint8 BID=2
uint8 PREPARE=3
uint8 ACK=4
uint8 COMMIT=5
uint8 ABORT=6
std_msgs/Header header
uint64 formation_digest
uint64 sender_session
uint64 sequence
uint32 sender_id
uint32 proposer_id
uint64 proposer_session
uint64 transaction_id
uint8 phase
time snapshot_stamp
uint64 base_epoch
uint64 assignment_epoch
uint64 base_digest
uint64 assignment_digest
uint32[] base_assignment
uint32[] drone_to_slot
uint64[] member_sessions
uint64[] member_sequences
float64[] positions
float64[] velocities
float64[] slot_goals
# Row-major physical drone -> slot cost. Negative means not locally certified.
float64[] costs
float64 gain
uint64[] ack_sessions
uint64[] ack_sequences
float64[] ack_stamps

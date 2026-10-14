# A pair owns one front/rear column; all agents acknowledge before commit.
uint8 PREPARE=1
uint8 ACK=2
uint8 COMMIT=3
std_msgs/Header header
uint64 sender_session
uint64 sequence
uint32 sender_id
uint32 column
uint32 front_drone
uint32 rear_drone
uint32 base_version
uint64 proposer_session
uint64 transaction_id
uint8 phase
float64 gain

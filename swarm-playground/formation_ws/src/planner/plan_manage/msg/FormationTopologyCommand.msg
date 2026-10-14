uint8 INTACT=0
uint8 SPLIT=1

std_msgs/Header header
uint64 epoch
uint8 mode
string components
uint32 edge_count
uint32 edge_keep_mask
uint32 weak_edge_mask
uint32[4] assignment_versions
uint32[8] drone_to_slot
float64 formation_scale
float64 progress_sync_scale
string reason

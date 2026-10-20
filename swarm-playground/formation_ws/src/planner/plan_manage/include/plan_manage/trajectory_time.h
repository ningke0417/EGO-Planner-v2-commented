#pragma once

namespace ego_planner {

// Convert a time relative to our trajectory into the peer's trajectory time
// at the same absolute instant. Do not clamp: callers retain their time window.
inline double peerTrajectoryTime(double self_start_time,
                                 double self_relative_time,
                                 double peer_start_time) {
  return self_relative_time + (self_start_time - peer_start_time);
}

}  // namespace ego_planner

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Eigen>

namespace ego_planner {

enum class ExecutionMode : uint8_t { RUNNING = 1, HOLDING = 2, FINISHED = 3 };

struct ExecutionFeedback {
  uint64_t session{0}, sequence{0};
  int trajectory_id{0};
  double trajectory_start_time{0.0}, duration{0.0};
  double stamp{0.0}, received_time{0.0};
  ExecutionMode mode{ExecutionMode::HOLDING};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool require_replan{false};

  bool seen() const { return session != 0 || sequence != 0 || stamp != 0.0; }
};

struct PeerPrediction {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool valid{false}, uncertain{true}, stationary{true}, from_execution{false};
  double trajectory_time{0.0};
};

// The same physical instant expressed on the peer's trajectory clock. In
// particular, a later self start ADDS elapsed peer time; it never subtracts it.
inline double peerTime(double self_start, double self_relative, double peer_start) {
  return self_start + self_relative - peer_start;
}

inline bool executionFeedbackNewer(const ExecutionFeedback& candidate,
                                   const ExecutionFeedback& current) {
  if (candidate.session == 0 || candidate.sequence == 0 ||
      !std::isfinite(candidate.stamp) || candidate.stamp <= 0.0 ||
      candidate.session < current.session) return false;
  if (candidate.session > current.session) return true;
  return candidate.sequence > current.sequence && candidate.stamp >= current.stamp;
}

inline bool executionFeedbackFresh(const ExecutionFeedback& feedback,
                                   double reference_now, double ttl) {
  if (!feedback.seen() || feedback.session == 0 || feedback.sequence == 0 ||
      !std::isfinite(reference_now) || !std::isfinite(ttl) || ttl <= 0.0 ||
      !std::isfinite(feedback.stamp) || feedback.stamp <= 0.0 ||
      !std::isfinite(feedback.received_time) || feedback.received_time <= 0.0 ||
      feedback.stamp > reference_now + 0.05 ||
      feedback.received_time > reference_now + 0.05 ||
      reference_now - feedback.stamp > ttl ||
      reference_now - feedback.received_time > ttl ||
      !feedback.position.allFinite() || !feedback.velocity.allFinite() ||
      !feedback.acceleration.allFinite()) return false;
  return feedback.mode == ExecutionMode::RUNNING ||
         feedback.mode == ExecutionMode::HOLDING ||
         feedback.mode == ExecutionMode::FINISHED;
}

// Pure, ROS-independent common prediction. `reference_now` determines whether
// evidence is fresh; the possibly far-future query does NOT age that evidence.
// A held/finished aircraft stays at its reported position. Stale, mismatching
// or replan-required execution freezes the last reported position and marks
// uncertainty. Neither an expired trajectory nor its nonzero end velocity is
// permission to linearly extrapolate a peer forever.
template <typename Trajectory>
inline PeerPrediction predictPeerTrajectory(
    const Trajectory& trajectory, int trajectory_id, double start, double duration,
    double last_receive_time, const ExecutionFeedback& feedback,
    double absolute_query, double reference_now,
    double execution_ttl = 0.5, double trajectory_ttl = 1.5) {
  PeerPrediction result;
  if (!std::isfinite(absolute_query) || !std::isfinite(reference_now) ||
      !std::isfinite(execution_ttl) || execution_ttl <= 0.0 ||
      !std::isfinite(trajectory_ttl) || trajectory_ttl <= 0.0) return result;

  const double actual_duration = trajectory.getPieceNum() > 0
                                     ? trajectory.getTotalDuration() : 0.0;
  const bool has_trajectory = trajectory_id > 0 && trajectory.getPieceNum() > 0 &&
      std::isfinite(start) && start > 0.0 && std::isfinite(duration) && duration > 0.0 &&
      std::isfinite(actual_duration) && actual_duration > 0.0 &&
      std::abs(actual_duration - duration) <= 1e-4;
  const bool fresh_execution = executionFeedbackFresh(feedback, reference_now, execution_ttl);
  if (feedback.seen() && feedback.position.allFinite()) {
    result.position = feedback.position;
    result.valid = true;
    result.from_execution = true;
    if (!fresh_execution) return result;
    result.uncertain = false;
    if (feedback.mode == ExecutionMode::HOLDING ||
        feedback.mode == ExecutionMode::FINISHED) return result;
    const bool matches = has_trajectory && feedback.trajectory_id == trajectory_id &&
        std::isfinite(feedback.trajectory_start_time) &&
        std::isfinite(feedback.duration) &&
        std::abs(feedback.trajectory_start_time - start) <= 1e-4 &&
        std::abs(feedback.duration - duration) <= 1e-4;
    if (!matches || feedback.require_replan) {
      result.uncertain = true;
      return result;
    }
  } else if (feedback.seen()) {
    // Malformed execution evidence must not silently re-enable legacy motion.
    // A trajectory start is the only available finite, conservative anchor.
    if (has_trajectory) {
      result.position = trajectory.getPos(0.0);
      result.valid = result.position.allFinite();
    }
    return result;
  } else if (!has_trajectory) return result;

  double query = absolute_query;
  if (!feedback.seen()) {
    const bool fresh_receive = std::isfinite(last_receive_time) && last_receive_time > 0.0 &&
        last_receive_time <= reference_now + 0.05 &&
        reference_now - last_receive_time <= trajectory_ttl;
    result.uncertain = !fresh_receive;
    if (!fresh_receive) {
      // Legacy packet loss: freeze where the peer was predicted when its last
      // trajectory packet arrived, rather than inventing continued execution.
      query = std::isfinite(last_receive_time) && last_receive_time > 0.0
                  ? last_receive_time : start;
    }
  }
  result.trajectory_time = std::max(0.0, std::min(duration, query - start));
  result.position = trajectory.getPos(result.trajectory_time);
  result.velocity.setZero();
  result.acceleration.setZero();
  result.stationary = result.uncertain || query < start || query >= start + duration;
  if (!result.stationary) {
    result.velocity = trajectory.getVel(result.trajectory_time);
    result.acceleration = trajectory.getAcc(result.trajectory_time);
  }
  result.valid = result.position.allFinite() && result.velocity.allFinite() &&
                 result.acceleration.allFinite();
  if (!result.valid) {
    result.uncertain = true;
    result.stationary = true;
    result.velocity.setZero();
    result.acceleration.setZero();
  }
  return result;
}

} // namespace ego_planner

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace ego_planner {
namespace distributed {
constexpr int kAgents = 8;
constexpr int kEdges = 10;
constexpr int kColumns = 4;
using Assignment = std::array<int, kAgents>; // physical drone -> logical slot
inline Assignment identityAssignment() {
  Assignment result{};
  for (int id = 0; id < kAgents; ++id) result[id] = id;
  return result;
}
inline bool validAssignment(const Assignment& assignment) {
  std::array<bool, kAgents> seen{};
  for (int slot : assignment) {
    if (slot < 0 || slot >= kAgents || seen[slot]) return false;
    seen[slot] = true;
  }
  return true;
}
inline int owner(const Assignment& assignment, int slot) {
  for (int id = 0; id < kAgents; ++id)
    if (assignment[id] == slot) return id;
  return -1;
}
inline bool swapColumn(Assignment* assignment, int column) {
  if (!assignment || !validAssignment(*assignment) ||
      column < 0 || column >= kColumns) return false;
  const int front = owner(*assignment, column);
  const int rear = owner(*assignment, column + kColumns);
  std::swap((*assignment)[front], (*assignment)[rear]);
  return true;
}
inline uint32_t assignmentSignature(const std::array<uint32_t, kColumns>& versions) {
  if (versions == std::array<uint32_t, kColumns>{{0, 0, 0, 0}}) return 0;
  uint32_t hash = 2166136261u;
  for (uint32_t version : versions) {
    hash = (hash ^ version) * 16777619u;
  }
  return hash;
}
inline Assignment assignmentFromVersions(
    const std::array<uint32_t, kColumns>& versions) {
  Assignment assignment = identityAssignment();
  for (int column = 0; column < kColumns; ++column)
    if (versions[column] & 1u) swapColumn(&assignment, column);
  return assignment;
}
// Positive means swapping the front and rear slots reduces squared tracking
// distance. The common formation center cancels out of this difference.
inline double columnSwapGain(double front_x, double front_y,
                             double rear_x, double rear_y,
                             double forward_x, double forward_y) {
  if (!std::isfinite(front_x) || !std::isfinite(rear_x) ||
      !std::isfinite(front_y) || !std::isfinite(rear_y) ||
      !std::isfinite(forward_x) || !std::isfinite(forward_y)) return 0.0;
  return 2.0 * ((rear_x-front_x)*forward_x +
                (rear_y-front_y)*forward_y);
}
inline const std::array<std::array<int, 2>, kEdges>& edges() {
  static const std::array<std::array<int, 2>, kEdges> value{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{4, 5}}, {{5, 6}}, {{6, 7}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
  return value;
}
inline bool edgeTouchesColumn(int edge, int column) {
  if (edge < 0 || edge >= kEdges || column < 0 || column >= kColumns)
    return false;
  return edges()[edge][0] % kColumns == column ||
         edges()[edge][1] % kColumns == column;
}
template <typename Versions>
inline bool edgeVersionsMatch(int edge,
    const std::array<uint32_t, kColumns>& local,
    const Versions& remote) {
  if (edge < 0 || edge >= kEdges) return false;
  return local[edges()[edge][0] % kColumns] ==
             remote[edges()[edge][0] % kColumns] &&
         local[edges()[edge][1] % kColumns] ==
             remote[edges()[edge][1] % kColumns];
}
// Per-writer ordering survives expiry. Replaying a packet never renews a lease.
struct Lease {
  uint64_t session = 0, sequence = 0;
  double stamp = 0.0;
  bool keep = false, weak = false;
  bool accept(uint64_t boot, uint64_t seq, double sent, double now,
              double ttl, bool decision, bool weak_decision = false) {
    if (!std::isfinite(sent) || !std::isfinite(now) || ttl <= 0.0 ||
        boot == 0 || seq == 0 || sent <= 0.0 || sent > now + 0.05 ||
        now - sent > ttl || boot < session ||
        (boot == session && seq <= sequence)) return false;
    session = boot; sequence = seq; stamp = sent; keep = decision;
    weak = decision && weak_decision;
    return true;
  }
  bool fresh(double now, double ttl) const {
    return session != 0 && now >= stamp - 0.05 && now - stamp <= ttl;
  }
};
inline bool localEdgeEvidenceSafe(bool kept, bool visibility_reconnected,
                                  double distance, double visibility,
                                  double score, double split_distance,
                                  double reconnect_distance, double split_score) {
  if (!std::isfinite(distance) || !std::isfinite(visibility) ||
      !std::isfinite(score) || split_distance <= 0.0 ||
      reconnect_distance <= 0.0 ||
      split_score < 0.0 || split_score > 1.0 || visibility <= 0.0)
    return false;
  if (distance > (kept ? split_distance : reconnect_distance)) return false;
  // As in centralized weak recovery, reconnection is a local visibility and
  // distance decision. Overlap becomes a split signal after it has recovered.
  return !kept || visibility_reconnected || score >= split_score;
}
// A cut edge may use the wider weak-recovery range only after the vehicles
// have separated and then steadily approached each other. Ordinary close
// reconnects do not need this gate.
class WeakRecoveryApproach {
 public:
  void reset() { *this = WeakRecoveryApproach(); }
  void cut(double distance) {
    reset();
    if (std::isfinite(distance) && distance > 0.0)
      peak_distance_ = distance;
  }
  void interrupt() {
    approach_since_ = 0.0;
    last_distance_ = 0.0;
  }
  bool update(double distance, double now, double split_distance,
              double min_separation, double min_reversal,
              double approach_hold) {
    if (!std::isfinite(distance) || distance <= 0.0 ||
        !std::isfinite(now) || now <= 0.0 ||
        !std::isfinite(split_distance) || split_distance <= 0.0 ||
        min_separation < 0.0 || min_reversal <= 0.0 ||
        approach_hold < 0.0) {
      approach_since_ = 0.0;
      return false;
    }
    if (distance > peak_distance_) peak_distance_ = distance;
    const bool separated = peak_distance_ >= split_distance + min_separation;
    const bool reversed = peak_distance_ - distance >= min_reversal;
    // Small changes in odometry must not restart a valid approach.
    const bool moving_away = last_distance_ > 0.0 && distance > last_distance_ + 0.03;
    if (!separated || !reversed || moving_away) approach_since_ = 0.0;
    else if (approach_since_ == 0.0) approach_since_ = now;
    last_distance_ = distance;
    return approach_since_ > 0.0 && now - approach_since_ >= approach_hold;
  }
 private:
  double peak_distance_{0.0};
  double last_distance_{0.0};
  double approach_since_{0.0};
};
class EdgeTable {
 public:
  bool updateAt(int edge, int observer, int peer, const Assignment& assignment,
                uint64_t session, uint64_t sequence, double stamp, double now,
                double ttl, bool keep, bool weak = false) {
    if (edge < 0 || edge >= kEdges || !validAssignment(assignment)) return false;
    const int a = owner(assignment, edges()[edge][0]);
    const int b = owner(assignment, edges()[edge][1]);
    if (observer == a && peer == b)
      return leases_[edge][0].accept(session, sequence, stamp, now, ttl, keep, weak);
    if (observer == b && peer == a)
      return leases_[edge][1].accept(session, sequence, stamp, now, ttl, keep, weak);
    return false;
  }
  bool update(int observer, int peer, uint64_t session, uint64_t sequence,
              double stamp, double now, double ttl, bool keep) {
    for (int edge = 0; edge < kEdges; ++edge) {
      const auto& pair = edges()[edge];
      for (int endpoint = 0; endpoint < 2; ++endpoint)
        if (pair[endpoint] == observer && pair[1 - endpoint] == peer)
          return leases_[edge][endpoint].accept(
              session, sequence, stamp, now, ttl, keep);
    }
    return false;
  }
  uint32_t mask(double now, double ttl) const {
    uint32_t result = 0;
    for (int edge = 0; edge < kEdges; ++edge) {
      const auto& pair = leases_[edge];
      if (pair[0].keep && pair[1].keep &&
          pair[0].fresh(now, ttl) && pair[1].fresh(now, ttl))
        result |= uint32_t{1} << edge;
    }
    return result;
  }
  uint32_t weakMask(double now, double ttl) const {
    uint32_t result = 0;
    for (int edge = 0; edge < kEdges; ++edge) {
      const auto& pair = leases_[edge];
      if (pair[0].keep && pair[1].keep &&
          pair[0].fresh(now, ttl) && pair[1].fresh(now, ttl) &&
          (pair[0].weak || pair[1].weak))
        result |= uint32_t{1} << edge;
    }
    return result;
  }
  void clear() { leases_ = {}; }
  void clearColumn(int column) {
    for (int edge = 0; edge < kEdges; ++edge)
      if (edgeTouchesColumn(edge, column)) leases_[edge] = {};
  }
 private:
  std::array<std::array<Lease, 2>, kEdges> leases_{};
};
inline std::string components(uint32_t mask, const Assignment& assignment) {
  std::array<int, kAgents> labels{};
  for (int id = 0; id < kAgents; ++id) labels[id] = id;
  for (int edge = 0; edge < kEdges; ++edge) {
    if (!(mask & (uint32_t{1} << edge))) continue;
    const int first = labels[owner(assignment, edges()[edge][0])];
    const int second = labels[owner(assignment, edges()[edge][1])];
    for (int& label : labels) if (label == second) label = first;
  }
  std::ostringstream out;
  std::array<bool, kAgents> emitted{};
  bool first_group = true;
  for (int id = 0; id < kAgents; ++id) {
    if (emitted[labels[id]]) continue;
    emitted[labels[id]] = true;
    if (!first_group) out << "|";
    first_group = false;
    out << "{";
    bool first = true;
    for (int member = 0; member < kAgents; ++member) {
      if (labels[member] != labels[id]) continue;
      if (!first) out << ",";
      first = false;
      out << member;
    }
    out << "}";
  }
  return out.str();
}
inline std::string components(uint32_t mask) {
  return components(mask, identityAssignment());
}
}  // namespace distributed
}  // namespace ego_planner

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <traj_utils/formation_grid.h>

namespace ego_planner {
namespace distributed {
// One immutable description is configured before constructing any agent state.
// Function-local storage keeps the header-only helpers consistent across TUs.
struct FormationDescription {
  int rows = 2, columns = 4, agents = 8, edge_count = 10;
  double row_spacing = 1.0, column_spacing = 1.0;
  std::vector<std::array<int, 2>> edge_pairs = rectangularFormationEdges(2, 4);
  std::vector<std::array<double, 3>> slot_offsets = rectangularFormationOffsets(2, 4, 1.0, 1.0);
};
inline FormationDescription& formationStorage() {
  static FormationDescription description;
  return description;
}
inline const FormationDescription& formation() { return formationStorage(); }
inline int agentCount() { return formation().agents; }
inline int edgeCount() { return formation().edge_count; }
inline int rowCount() { return formation().rows; }
inline int columnCount() { return formation().columns; }
// These compatibility names remain convenient in loops; they are not template
// dimensions. Containers use the runtime counts instead of separate binaries.
struct RuntimeCount {
  enum Kind { Agents, Edges, Columns } kind;
  operator int() const {
    return kind == Agents ? agentCount() : kind == Edges ? edgeCount() : columnCount();
  }
};
constexpr RuntimeCount kAgents{RuntimeCount::Agents};
constexpr RuntimeCount kEdges{RuntimeCount::Edges};
constexpr RuntimeCount kColumns{RuntimeCount::Columns};
inline void configureFormation(int rows, int columns, double row_spacing = 1.0,
                               double column_spacing = 1.0) {
  if (rows <= 0 || columns <= 0 || rows > 16 || columns > 16 ||
      rows * columns < 2 || rows * columns > 16 ||
      rows * (columns - 1) + (rows - 1) * columns > 31 ||
      !std::isfinite(row_spacing) || row_spacing <= 0 ||
      !std::isfinite(column_spacing) || column_spacing <= 0)
    throw std::invalid_argument("formation requires 2..16 slots and at most 31 grid edges");
  FormationDescription description;
  description.rows = rows; description.columns = columns;
  description.agents = rows * columns;
  description.row_spacing = row_spacing; description.column_spacing = column_spacing;
  description.edge_pairs = rectangularFormationEdges(rows, columns);
  description.slot_offsets = rectangularFormationOffsets(rows, columns, row_spacing, column_spacing);
  description.edge_count = static_cast<int>(description.edge_pairs.size());
  formationStorage() = description;
}
inline uint32_t fullMask() { return (uint32_t{1} << edgeCount()) - uint32_t{1}; }
inline uint64_t formationDigest() {
  uint64_t hash = 14695981039346656037ull;
  const auto append = [&hash](uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash = (hash ^ ((value >> (8 * byte)) & 255u)) * 1099511628211ull;
    }
  };
  append(static_cast<uint64_t>(rowCount())); append(static_cast<uint64_t>(columnCount()));
  uint64_t bits = 0;
  std::memcpy(&bits, &formation().row_spacing, sizeof(bits)); append(bits);
  std::memcpy(&bits, &formation().column_spacing, sizeof(bits)); append(bits);
  return hash ? hash : 1;
}
using Assignment = std::vector<int>; // physical drone -> logical slot
inline Assignment identityAssignment() {
  Assignment result(agentCount());
  for (int id = 0; id < kAgents; ++id) result[id] = id;
  return result;
}
inline bool validAssignment(const Assignment& assignment) {
  if (assignment.size() != static_cast<size_t>(agentCount())) return false;
  std::vector<bool> seen(agentCount(), false);
  for (int slot : assignment) {
    if (slot < 0 || slot >= kAgents || seen[slot]) return false;
    seen[slot] = true;
  }
  return true;
}
inline int owner(const Assignment& assignment, int slot) {
  if (!validAssignment(assignment)) return -1;
  for (int id = 0; id < kAgents; ++id)
    if (assignment[id] == slot) return id;
  return -1;
}
inline bool swapColumn(Assignment* assignment, int column) {
  if (rowCount() != 2 || !assignment || !validAssignment(*assignment) ||
      column < 0 || column >= kColumns) return false;
  const int front = owner(*assignment, column);
  const int rear = owner(*assignment, column + kColumns);
  std::swap((*assignment)[front], (*assignment)[rear]);
  return true;
}
template <typename Versions>
inline uint32_t assignmentSignature(const Versions& versions) {
  if (versions.size() != static_cast<size_t>(columnCount())) return 0;
  if (std::all_of(versions.begin(), versions.end(), [](uint32_t v) { return v == 0; })) return 0;
  uint32_t hash = 2166136261u;
  for (uint32_t version : versions) hash = (hash ^ version) * 16777619u;
  return hash;
}
template <typename Versions>
inline Assignment assignmentFromVersions(const Versions& versions) {
  if (versions.size() != static_cast<size_t>(columnCount())) return {};
  Assignment assignment = identityAssignment();
  if (rowCount() != 2) {
    if (!std::all_of(versions.begin(), versions.end(), [](uint32_t v) { return v == 0; })) return {};
    return assignment;
  }
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
inline const std::vector<std::array<int, 2>>& edges() {
  return formation().edge_pairs;
}
inline bool edgeTouchesColumn(int edge, int column) {
  if (edge < 0 || edge >= kEdges || column < 0 || column >= kColumns)
    return false;
  return edges()[edge][0] % kColumns == column ||
         edges()[edge][1] % kColumns == column;
}
template <typename LocalVersions, typename Versions>
inline bool edgeVersionsMatch(int edge,
    const LocalVersions& local,
    const Versions& remote) {
  if (edge < 0 || edge >= kEdges ||
      local.size() != static_cast<size_t>(columnCount()) ||
      remote.size() != static_cast<size_t>(columnCount())) return false;
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
// Reconnecting and weak edges are certified by visibility/distance. A newly
// reconnected edge retains that existing overlap exemption during startup, so
// expensive overlap work cannot starve the heartbeats needed for the barrier.
// Once startup has passed, score recovery is evaluated normally; an ordinary
// non-exempt connection always requires its current overlap score.
inline bool needsOverlapScore(bool kept, bool weak_reconnected,
                              bool visibility_reconnected, bool startup_ready) {
  return kept && !weak_reconnected && (!visibility_reconnected || startup_ready);
}
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
  EdgeTable() : leases_(edgeCount()) {}
  bool updateAt(int edge, int observer, int peer, const Assignment& assignment,
                uint64_t session, uint64_t sequence, double stamp, double now,
                double ttl, bool keep, bool weak = false) {
    if (leases_.size() != static_cast<size_t>(edgeCount())) clear();
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
    if (leases_.size() != static_cast<size_t>(edgeCount())) clear();
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
    if (leases_.size() != static_cast<size_t>(edgeCount())) return result;
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
    if (leases_.size() != static_cast<size_t>(edgeCount())) return result;
    for (int edge = 0; edge < kEdges; ++edge) {
      const auto& pair = leases_[edge];
      if (pair[0].keep && pair[1].keep &&
          pair[0].fresh(now, ttl) && pair[1].fresh(now, ttl) &&
          (pair[0].weak || pair[1].weak))
        result |= uint32_t{1} << edge;
    }
    return result;
  }
  void clear() { leases_.assign(edgeCount(), {}); }
  void clearColumn(int column) {
    if (leases_.size() != static_cast<size_t>(edgeCount())) clear();
    for (int edge = 0; edge < kEdges; ++edge)
      if (edgeTouchesColumn(edge, column)) leases_[edge] = {};
  }
 private:
  std::vector<std::array<Lease, 2>> leases_;
};
inline std::string components(uint32_t mask, const Assignment& assignment) {
  if (!validAssignment(assignment)) return {};
  std::vector<int> labels(agentCount());
  for (int id = 0; id < kAgents; ++id) labels[id] = id;
  for (int edge = 0; edge < kEdges; ++edge) {
    if (!(mask & (uint32_t{1} << edge))) continue;
    const int first = labels[owner(assignment, edges()[edge][0])];
    const int second = labels[owner(assignment, edges()[edge][1])];
    for (int& label : labels) if (label == second) label = first;
  }
  std::ostringstream out;
  std::vector<bool> emitted(agentCount(), false);
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

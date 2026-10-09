#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace ego_planner {
namespace distributed {
constexpr int kAgents = 8;
constexpr int kEdges = 10;
inline const std::array<std::array<int, 2>, kEdges>& edges() {
  static const std::array<std::array<int, 2>, kEdges> value{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{4, 5}}, {{5, 6}}, {{6, 7}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
  return value;
}
// Per-writer ordering survives expiry. Replaying a packet never renews a lease.
struct Lease {
  uint64_t session = 0, sequence = 0;
  double stamp = 0.0;
  bool keep = false;
  bool accept(uint64_t boot, uint64_t seq, double sent, double now,
              double ttl, bool decision) {
    if (!std::isfinite(sent) || !std::isfinite(now) || ttl <= 0.0 ||
        boot == 0 || seq == 0 || sent <= 0.0 || sent > now + 0.05 ||
        now - sent > ttl || boot < session ||
        (boot == session && seq <= sequence)) return false;
    session = boot; sequence = seq; stamp = sent; keep = decision;
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
class EdgeTable {
 public:
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
 private:
  std::array<std::array<Lease, 2>, kEdges> leases_{};
};
inline std::string components(uint32_t mask) {
  std::array<int, kAgents> labels{};
  for (int id = 0; id < kAgents; ++id) labels[id] = id;
  for (int edge = 0; edge < kEdges; ++edge) {
    if (!(mask & (uint32_t{1} << edge))) continue;
    const int first = labels[edges()[edge][0]];
    const int second = labels[edges()[edge][1]];
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
}  // namespace distributed
}  // namespace ego_planner

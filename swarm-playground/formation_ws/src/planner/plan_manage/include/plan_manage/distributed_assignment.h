#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <plan_manage/distributed_edges.h>

namespace ego_planner {
namespace distributed {

// A row belongs to one physical aircraft; a column names a logical slot.
// Positive infinity is an unavailable/unobserved route, never a cheap route.
using AssignmentCosts = std::array<std::array<double, kAgents>, kAgents>;

struct AssignmentSolution {
  bool feasible = false;
  Assignment assignment = identityAssignment();
  double current_cost = std::numeric_limits<double>::infinity();
  double candidate_cost = std::numeric_limits<double>::infinity();
  double penalized_cost = std::numeric_limits<double>::infinity();
  double net_gain = 0.0;
};

// A collision-free, stable digest of the complete permutation (Lehmer rank).
// Zero is reserved for invalid/uninitialized assignments. Epochs are separate:
// revisiting a permutation has the same digest, not the same transaction ID.
inline uint64_t fullAssignmentSignature(const Assignment& assignment) {
  if (!validAssignment(assignment)) return 0;
  uint64_t rank = 0;
  for (int id = 0; id < kAgents; ++id) {
    uint64_t smaller = 0;
    for (int other = id + 1; other < kAgents; ++other)
      if (assignment[other] < assignment[id]) ++smaller;
    rank = rank * static_cast<uint64_t>(kAgents - id) + smaller;
  }
  return rank + 1;
}

inline bool assignmentCostsEqual(double a, double b) {
  if (!std::isfinite(a) || !std::isfinite(b)) return a == b;
  const double scale = std::max(1.0, std::max(std::abs(a), std::abs(b)));
  return std::abs(a - b) <= 1e-10 * scale;
}

// Exact 8x8 assignment by subset dynamic programming. Switching penalty is
// charged once per aircraft that changes slots. Current wins a numerical tie;
// other ties choose the lexicographically smallest drone->slot permutation.
// The caller must only supply routes certified by that aircraft's own map.
inline AssignmentSolution solveAssignment(const AssignmentCosts& costs,
                                          const Assignment& current,
                                          double switching_penalty = 0.0) {
  AssignmentSolution result;
  result.assignment = current;
  if (!validAssignment(current) || !std::isfinite(switching_penalty) ||
      switching_penalty < 0.0) return result;
  for (const auto& row : costs)
    for (double cost : row)
      if (std::isnan(cost) || cost == -std::numeric_limits<double>::infinity())
        return result;

  constexpr int count = 1 << kAgents;
  const double infinity = std::numeric_limits<double>::infinity();
  std::array<double, count> suffix;
  suffix.fill(infinity);
  suffix[count - 1] = 0.0;
  for (int mask = count - 2; mask >= 0; --mask) {
    int id = 0;
    for (int bits = mask; bits; bits >>= 1) id += bits & 1;
    for (int slot = 0; slot < kAgents; ++slot) {
      if (mask & (1 << slot)) continue;
      const double cost = costs[id][slot] +
          (slot == current[id] ? 0.0 : switching_penalty) +
          suffix[mask | (1 << slot)];
      if (std::isfinite(cost) && cost < suffix[mask]) suffix[mask] = cost;
    }
  }
  if (!std::isfinite(suffix[0])) return result;

  result.current_cost = 0.0;
  for (int id = 0; id < kAgents; ++id)
    result.current_cost += costs[id][current[id]];
  if (assignmentCostsEqual(result.current_cost, suffix[0])) {
    result.feasible = true;
    result.candidate_cost = result.current_cost;
    result.penalized_cost = result.current_cost;
    return result;
  }

  int mask = 0;
  result.candidate_cost = 0.0;
  result.penalized_cost = 0.0;
  for (int id = 0; id < kAgents; ++id) {
    bool selected = false;
    for (int slot = 0; slot < kAgents; ++slot) {
      if (mask & (1 << slot)) continue;
      const double penalty = slot == current[id] ? 0.0 : switching_penalty;
      const double cost = costs[id][slot] + penalty +
                          suffix[mask | (1 << slot)];
      if (!std::isfinite(cost) ||
          !assignmentCostsEqual(cost, suffix[mask])) continue;
      result.assignment[id] = slot;
      result.candidate_cost += costs[id][slot];
      result.penalized_cost += costs[id][slot] + penalty;
      mask |= 1 << slot;
      selected = true;
      break;
    }
    if (!selected) return AssignmentSolution(); // Fail closed on arithmetic issues.
  }
  result.feasible = true;
  result.net_gain = result.current_cost - result.penalized_cost;
  return result;
}

// Both thresholds are required. A blocked/unobserved current route cannot
// certify a finite improvement and must not initiate an offer. The solver may
// still expose its finite alternative for diagnostics. Zero gain never wins.
inline bool assignmentGainSufficient(const AssignmentSolution& solution,
                                     double minimum_gain,
                                     double minimum_relative_gain) {
  if (!solution.feasible || !validAssignment(solution.assignment) ||
      !std::isfinite(solution.penalized_cost) ||
      !std::isfinite(minimum_gain) || minimum_gain < 0.0 ||
      !std::isfinite(minimum_relative_gain) || minimum_relative_gain < 0.0 ||
      !std::isfinite(solution.net_gain) || solution.net_gain <= 0.0 ||
      !std::isfinite(solution.current_cost))
    return false;
  return solution.net_gain >= minimum_gain &&
      solution.net_gain >= minimum_relative_gain *
          std::max(std::abs(solution.current_cost), 1e-9);
}

// Local anti-chatter gate; it grants no authority to commit a proposal.
// The protocol must still obtain fresh unanimous confirmations. In particular,
// remaining_goal_distance is the minimum over aircraft CHANGING their slots;
// arrived, unchanged aircraft are fixed by unavailable entries in their cost
// rows, rather than preventing the remaining aircraft from reassigning.
// Failed evaluations reset hold.
class AssignmentGate {
 public:
  bool ready(const AssignmentSolution& solution, double now,
             double stable_hold, double cooldown,
             double remaining_goal_distance, double lock_distance,
             double minimum_gain, double minimum_relative_gain) {
    if (!std::isfinite(now) || now <= 0.0 ||
        !std::isfinite(stable_hold) || stable_hold < 0.0 ||
        !std::isfinite(cooldown) || cooldown < 0.0 ||
        !std::isfinite(remaining_goal_distance) ||
        !std::isfinite(lock_distance) || lock_distance < 0.0 ||
        remaining_goal_distance <= lock_distance ||
        (last_commit_ > 0.0 && now - last_commit_ < cooldown) ||
        now < last_evaluation_ ||
        !assignmentGainSufficient(solution, minimum_gain,
                                  minimum_relative_gain)) {
      resetCandidate();
      if (std::isfinite(now)) last_evaluation_ = now;
      return false;
    }
    last_evaluation_ = now;
    const uint64_t digest = fullAssignmentSignature(solution.assignment);
    if (candidate_digest_ != digest || candidate_since_ <= 0.0) {
      candidate_digest_ = digest;
      candidate_since_ = now;
    }
    return now - candidate_since_ >= stable_hold;
  }

  void committed(double now) {
    if (std::isfinite(now) && now > 0.0) last_commit_ = now;
    resetCandidate();
  }
  void resetCandidate() { candidate_digest_ = 0; candidate_since_ = 0.0; }

 private:
  double last_commit_ = 0.0;
  double last_evaluation_ = 0.0;
  double candidate_since_ = 0.0;
  uint64_t candidate_digest_ = 0;
};

} // namespace distributed
} // namespace ego_planner

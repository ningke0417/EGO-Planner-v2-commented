#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <random>
#include <set>

#include <plan_manage/distributed_assignment.h>

using namespace ego_planner::distributed;

namespace {
AssignmentCosts costsFor(const Assignment& optimum, double unwanted = 20.0) {
  AssignmentCosts costs;
  for (int id = 0; id < kAgents; ++id) {
    costs[id].fill(unwanted);
    costs[id][optimum[id]] = 1.0;
  }
  return costs;
}
AssignmentSolution beneficial() {
  return solveAssignment(costsFor(Assignment{{3, 2, 1, 0, 7, 6, 5, 4}}),
                         identityAssignment());
}
} // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DistributedAssignment, ReversedColumnsNeedFullEightAgentPermutation) {
  const Assignment wanted{{3, 2, 1, 0, 7, 6, 5, 4}};
  const auto result = solveAssignment(costsFor(wanted), identityAssignment());
  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(wanted, result.assignment);
  EXPECT_EQ(8.0, result.candidate_cost);
  EXPECT_EQ(152.0, result.net_gain);
  EXPECT_TRUE(validAssignment(result.assignment));
}

TEST(DistributedAssignment, SupportsThreeCycleNotJustPairSwaps) {
  const Assignment wanted{{1, 2, 0, 3, 4, 5, 6, 7}};
  const auto result = solveAssignment(costsFor(wanted), identityAssignment(), 0.5);
  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(wanted, result.assignment);
  EXPECT_EQ(8.0, result.candidate_cost);
  EXPECT_EQ(9.5, result.penalized_cost);
  EXPECT_EQ(55.5, result.net_gain);
}

TEST(DistributedAssignment, SixArrivedSlotsStayReservedWhileTwoOthersReassign) {
  const Assignment wanted{{3, 1, 2, 0, 4, 5, 6, 7}};
  auto costs = costsFor(wanted);
  for (int id : {1, 2, 4, 5, 6, 7}) {
    costs[id].fill(std::numeric_limits<double>::infinity());
    costs[id][id] = 0.01; // Arrived member may keep only its current slot.
  }
  const auto result = solveAssignment(costs, identityAssignment(), 0.1);
  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(wanted, result.assignment);
  for (int id : {1, 2, 4, 5, 6, 7}) EXPECT_EQ(id, result.assignment[id]);
  AssignmentGate gate;
  // The relevant distance is that of the two moving members, not the 0.01m
  // distance of six already parked members whose rows reserve their slots.
  EXPECT_TRUE(gate.ready(result, 10.0, 0.0, 0.0, 2.0, 0.8, 1.0, 0.05));
}

TEST(DistributedAssignment, CurrentPermutationWinsTiesAndSwitchPenalty) {
  const Assignment current{{7, 6, 5, 4, 3, 2, 1, 0}};
  AssignmentCosts equal;
  for (auto& row : equal) row.fill(1.0);
  EXPECT_EQ(current, solveAssignment(equal, current).assignment);
  auto costs = costsFor(Assignment{{1, 0, 2, 3, 4, 5, 6, 7}}, 1.1);
  const auto preserved = solveAssignment(costs, identityAssignment(), 0.2);
  ASSERT_TRUE(preserved.feasible);
  EXPECT_EQ(identityAssignment(), preserved.assignment);
  EXPECT_EQ(0.0, preserved.net_gain);
}

TEST(DistributedAssignment, OtherTiesAreDeterministicLexicographic) {
  AssignmentCosts costs;
  for (auto& row : costs) row.fill(1.0);
  costs[0][0] = 10.0;
  const Assignment expected{{1, 0, 2, 3, 4, 5, 6, 7}};
  EXPECT_EQ(expected, solveAssignment(costs, identityAssignment()).assignment);
}

TEST(DistributedAssignment, ExactSolverMatchesEnumerationWithForbiddenRoutes) {
  std::mt19937 random(17017);
  for (int sample = 0; sample < 16; ++sample) {
    auto current = identityAssignment();
    std::shuffle(current.begin(), current.end(), random);
    AssignmentCosts costs;
    for (auto& row : costs)
      for (double& cost : row) {
        const int draw = random() % 13;
        cost = draw == 12 ? std::numeric_limits<double>::infinity() : draw;
      }
    const double penalty = static_cast<double>(sample % 4);
    double best_cost = std::numeric_limits<double>::infinity();
    Assignment best = identityAssignment();
    Assignment permutation = identityAssignment();
    do {
      double cost = 0.0;
      for (int id = 0; id < kAgents; ++id)
        cost += costs[id][permutation[id]] +
                (permutation[id] == current[id] ? 0.0 : penalty);
      if (cost < best_cost) { best_cost = cost; best = permutation; }
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    double current_cost = 0.0;
    for (int id = 0; id < kAgents; ++id) current_cost += costs[id][current[id]];
    if (std::isfinite(current_cost) && current_cost == best_cost) best = current;
    const auto result = solveAssignment(costs, current, penalty);
    ASSERT_EQ(std::isfinite(best_cost), result.feasible) << sample;
    ASSERT_TRUE(result.feasible);
    EXPECT_EQ(best, result.assignment) << sample;
    EXPECT_EQ(best_cost, result.penalized_cost) << sample;
  }
}

TEST(DistributedAssignment, DuplicateCurrentNanAndNegativeInfinityFailClosed) {
  const auto costs = costsFor(identityAssignment());
  auto duplicate = identityAssignment();
  duplicate[1] = 0;
  EXPECT_FALSE(solveAssignment(costs, duplicate).feasible);
  auto invalid = costs;
  invalid[0][7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solveAssignment(invalid, identityAssignment()).feasible);
  invalid[0][7] = -std::numeric_limits<double>::infinity();
  EXPECT_FALSE(solveAssignment(invalid, identityAssignment()).feasible);
  EXPECT_FALSE(solveAssignment(costs, identityAssignment(), -0.1).feasible);
}

TEST(DistributedAssignment, UnreachableRowsAndConflictingOnlyChoiceFailClosed) {
  auto costs = costsFor(identityAssignment());
  costs[2].fill(std::numeric_limits<double>::infinity());
  EXPECT_FALSE(solveAssignment(costs, identityAssignment()).feasible);
  costs[2][0] = 0.0;
  costs[0].fill(std::numeric_limits<double>::infinity());
  costs[0][0] = 0.0;
  EXPECT_FALSE(solveAssignment(costs, identityAssignment()).feasible);
}

TEST(DistributedAssignment, UnreachableCurrentDoesNotCertifyGainForAnOffer) {
  const Assignment wanted{{1, 0, 2, 3, 4, 5, 6, 7}};
  auto costs = costsFor(wanted, std::numeric_limits<double>::infinity());
  const auto result = solveAssignment(costs, identityAssignment());
  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(wanted, result.assignment);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), result.current_cost);
  EXPECT_FALSE(assignmentGainSufficient(result, 10.0, 0.1));
  AssignmentGate gate;
  EXPECT_FALSE(gate.ready(result, 10.0, 0.0, 0.0, 5.0, 1.0, 10.0, 0.1));
}

TEST(DistributedAssignment, RequiresBothAbsoluteAndRelativeImprovement) {
  const auto result = beneficial();
  EXPECT_TRUE(assignmentGainSufficient(result, 20.0, 0.1));
  EXPECT_FALSE(assignmentGainSufficient(result, 153.0, 0.1));
  EXPECT_FALSE(assignmentGainSufficient(result, 20.0, 0.96));
  EXPECT_FALSE(assignmentGainSufficient(result, -1.0, 0.1));
  auto flat = result;
  flat.net_gain = 0.0;
  EXPECT_FALSE(assignmentGainSufficient(flat, 0.0, 0.0));
}

TEST(DistributedAssignment, HoldCooldownAndNearGoalLockAreIndependent) {
  AssignmentGate gate;
  const auto result = beneficial();
  EXPECT_FALSE(gate.ready(result, 10.0, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(result, 10.4, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_TRUE(gate.ready(result, 10.6, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  gate.committed(10.6);
  EXPECT_FALSE(gate.ready(result, 12.0, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(result, 12.7, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_TRUE(gate.ready(result, 13.3, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(result, 13.4, 0.5, 2.0, 1.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(result, 13.5, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_TRUE(gate.ready(result, 14.1, 0.5, 2.0, 5.0, 1.0, 1.0, 0.1));
}

TEST(DistributedAssignment, CandidateChangeOrBadSampleRestartsHold) {
  AssignmentGate gate;
  const auto first = beneficial();
  const auto second = solveAssignment(
      costsFor(Assignment{{1, 2, 0, 3, 4, 5, 6, 7}}), identityAssignment());
  EXPECT_FALSE(gate.ready(first, 10.0, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(second, 10.4, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(second, 10.6, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_TRUE(gate.ready(second, 11.0, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(AssignmentSolution(), 11.1, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(second, 11.2, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(second, 10.9, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(gate.ready(second, 11.3, 0.5, 0.0, 5.0, 1.0, 1.0, 0.1));
}

TEST(DistributedAssignment, FullPermutationDigestIsStableAndUnique) {
  Assignment assignment = identityAssignment();
  EXPECT_EQ(1u, fullAssignmentSignature(assignment));
  std::set<uint64_t> digests;
  do {
    const auto digest = fullAssignmentSignature(assignment);
    EXPECT_NE(0u, digest);
    EXPECT_EQ(digest, fullAssignmentSignature(assignment));
    EXPECT_TRUE(digests.insert(digest).second);
  } while (std::next_permutation(assignment.begin(), assignment.end()));
  EXPECT_EQ(40320u, digests.size());
  assignment[1] = 0;
  EXPECT_EQ(0u, fullAssignmentSignature(assignment));
}

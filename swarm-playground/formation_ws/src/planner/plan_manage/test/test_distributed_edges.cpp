#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <plan_manage/distributed_edges.h>
using namespace ego_planner::distributed;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DistributedEdges, RequiresBothEndpointsAndExpires) {
  EdgeTable t;
  EXPECT_EQ(0u, t.mask(10, 1));
  EXPECT_TRUE(t.update(0,1,1,1,10,10,1,true));
  EXPECT_EQ(0u, t.mask(10,1));
  EXPECT_TRUE(t.update(1,0,2,1,10,10,1,true));
  EXPECT_EQ(1u, t.mask(10,1));
  EXPECT_EQ(0u, t.mask(11.01,1));
  EXPECT_FALSE(t.update(0,1,1,1,10,10.8,1,true));
  EXPECT_EQ(0u, t.mask(11.01,1));
}
TEST(DistributedEdges, CutWinsAndOrderingSurvivesExpiry) {
  EdgeTable t;
  t.update(0,1,1,1,10,10,1,true);
  t.update(1,0,2,1,10,10,1,true);
  EXPECT_TRUE(t.update(0,1,1,2,10.1,10.1,1,false));
  EXPECT_EQ(0u,t.mask(10.1,1));
  EXPECT_FALSE(t.update(0,1,1,1,10,10.2,1,true));
  EXPECT_FALSE(t.update(0,1,1,2,12,12,1,true));
  EXPECT_EQ(0u,t.mask(12,1));
}
TEST(DistributedEdges, NewSessionRejectsOldPackets) {
  Lease l;
  EXPECT_TRUE(l.accept(100,20,10,10,1,true));
  EXPECT_TRUE(l.accept(101,1,10.1,10.1,1,false));
  EXPECT_FALSE(l.accept(100,21,10.2,10.2,1,true));
  EXPECT_FALSE(l.keep);
}
TEST(DistributedEdges, InvalidAndDelayedEvidenceRejected) {
  EdgeTable t;
  EXPECT_FALSE(t.update(0,7,1,1,10,10,1,true));
  EXPECT_FALSE(t.update(-1,0,1,1,10,10,1,true));
  EXPECT_FALSE(t.update(0,1,1,1,8,10,1,true));
  EXPECT_FALSE(t.update(0,1,1,1,12,10,1,true));
  EXPECT_FALSE(t.update(0,1,0,1,10,10,1,true));
  EXPECT_FALSE(t.update(0,1,1,1,std::numeric_limits<double>::quiet_NaN(),10,1,true));
}
TEST(DistributedEdges, DeterministicComponents) {
  EXPECT_EQ("{0}|{1}|{2}|{3}|{4}|{5}|{6}|{7}",components(0));
  EXPECT_EQ("{0,1,2,3,4,5,6,7}",components(1023));
  EXPECT_EQ("{0,1,2,3}|{4,5,6,7}",components(63));
}
TEST(DistributedEdges, VisibilityReconnectDoesNotRequireOverlap) {
  EXPECT_TRUE(localEdgeEvidenceSafe(false, false, 1.6, 1.0, 0.0,
                                    1.8, 1.7, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(false, false, 1.71, 1.0, 1.0,
                                     1.8, 1.7, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(false, false, 1.0, 0.0, 1.0,
                                     1.8, 1.7, 0.34));
  EXPECT_TRUE(localEdgeEvidenceSafe(true, true, 1.7, 1.0, 0.0,
                                    1.8, 1.7, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(true, false, 1.7, 1.0, 0.33,
                                     1.8, 1.7, 0.34));
  EXPECT_TRUE(localEdgeEvidenceSafe(true, false, 1.7, 1.0, 0.34,
                                    1.8, 1.7, 0.34));
}
TEST(DistributedEdges, WeakReconnectAllowsVisibleRecoveryAtThreeMeters) {
  EXPECT_TRUE(localEdgeEvidenceSafe(false, false, 2.5, 1.0, 0.0,
                                    1.8, 3.0, 0.34));
  EXPECT_TRUE(localEdgeEvidenceSafe(true, true, 2.5, 1.0, 0.0,
                                    3.0, 3.0, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(true, true, 2.5, 0.0, 1.0,
                                     3.0, 3.0, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(true, false, 1.9, 1.0, 1.0,
                                     1.8, 3.0, 0.34));
}
TEST(DistributedEdges, WeakRecoveryRequiresSustainedApproach) {
  WeakRecoveryApproach approach;
  approach.cut(1.81);
  EXPECT_FALSE(approach.update(1.95, 10.0, 1.8, 0.2, 0.25, 0.6));
  // The recorded wall-front edge was still separating when it reached 2.1 m.
  EXPECT_FALSE(approach.update(2.12, 10.3, 1.8, 0.2, 0.25, 0.6));
  EXPECT_FALSE(approach.update(2.30, 10.45, 1.8, 0.2, 0.25, 0.6));
  EXPECT_FALSE(approach.update(2.85, 10.6, 1.8, 0.2, 0.25, 0.6));
  EXPECT_FALSE(approach.update(2.54, 10.9, 1.8, 0.2, 0.25, 0.6));
  EXPECT_FALSE(approach.update(2.38, 11.2, 1.8, 0.2, 0.25, 0.6));
  EXPECT_TRUE(approach.update(2.22, 11.51, 1.8, 0.2, 0.25, 0.6));
  EXPECT_FALSE(approach.update(2.35, 11.8, 1.8, 0.2, 0.25, 0.6));
  approach.cut(2.35);
  EXPECT_FALSE(approach.update(2.25, 12.1, 1.8, 0.2, 0.25, 0.6));
  approach.interrupt();
  EXPECT_FALSE(approach.update(2.00, 13.1, 1.8, 0.2, 0.25, 0.6));
}
TEST(DistributedEdges, WeakMaskNeedsTwoFreshKeepLeases) {
  EdgeTable table;
  auto assignment = identityAssignment();
  EXPECT_TRUE(table.updateAt(1, 1, 2, assignment, 1, 1, 10, 10, 1, true, true));
  EXPECT_EQ(0u, table.weakMask(10, 1));
  EXPECT_TRUE(table.updateAt(1, 2, 1, assignment, 2, 1, 10, 10, 1, true));
  EXPECT_EQ(2u, table.weakMask(10, 1));
  EXPECT_TRUE(table.updateAt(1, 1, 2, assignment, 1, 2, 10.1, 10.1, 1, true));
  EXPECT_EQ(0u, table.weakMask(10.1, 1));
}
TEST(DistributedEdges, IndependentTablesConvergeDespiteDeliveryOrder) {
  EdgeTable a,b;
  for (int e=0; e<kEdges; ++e) {
    const auto pair=edges()[e];
    for (int end=0; end<2; ++end)
      a.update(pair[end],pair[1-end],pair[end]+1,1,10,10,1,true);
  }
  for (int e=kEdges-1; e>=0; --e) {
    const auto pair=edges()[e];
    for (int end=1; end>=0; --end)
      b.update(pair[end],pair[1-end],pair[end]+1,1,10,10,1,true);
  }
  EXPECT_EQ(1023u,a.mask(10,1));
  EXPECT_EQ(a.mask(10,1),b.mask(10,1));
  // Failure of member 0 only expires its incident edges.
  for (int e=0; e<kEdges; ++e)
    for (int end=0; end<2; ++end) {
      const auto pair=edges()[e];
      if (pair[end]!=0) a.update(pair[end],pair[1-end],pair[end]+1,2,10.9,10.9,1,true);
    }
  EXPECT_EQ(1023u & ~65u,a.mask(11.1,1));
}

TEST(DistributedEdges, PairSwapPreservesPermutationAndRemapsOnlyIncidentEdges) {
  Assignment assignment = identityAssignment();
  std::vector<uint32_t> versions(columnCount(), 0);
  EXPECT_TRUE(swapColumn(&assignment, 1));
  ++versions[1];
  EXPECT_TRUE(validAssignment(assignment));
  EXPECT_EQ(5, owner(assignment, 1));
  EXPECT_EQ(1, owner(assignment, 5));
  EXPECT_EQ(assignment, assignmentFromVersions(versions));
  EXPECT_EQ("{0,1,2,3,4,5,6,7}", components(1023, assignment));
  EdgeTable table;
  EXPECT_FALSE(table.updateAt(0, 0, 1, assignment, 1, 1, 10, 10, 1, true));
  EXPECT_TRUE(table.updateAt(0, 0, 5, assignment, 1, 1, 10, 10, 1, true));
  EXPECT_TRUE(table.updateAt(0, 5, 0, assignment, 2, 1, 10, 10, 1, true));
  EXPECT_EQ(1u, table.mask(10, 1));
  table.clearColumn(2);
  EXPECT_EQ(1u, table.mask(10, 1));
  table.clearColumn(1);
  EXPECT_EQ(0u, table.mask(10, 1));
}

TEST(DistributedEdges, EvidenceChecksOnlyVersionsOfItsLogicalEndpoints) {
  std::vector<uint32_t> local{{0, 1, 0, 0}};
  std::vector<uint32_t> same_edge{{0, 1, 7, 0}};
  std::vector<uint32_t> stale_edge{{0, 0, 7, 0}};
  EXPECT_TRUE(edgeVersionsMatch(0, local, same_edge));
  EXPECT_FALSE(edgeVersionsMatch(0, local, stale_edge));
  EXPECT_TRUE(edgeVersionsMatch(9, local, stale_edge));
}

TEST(DistributedEdges, InvertedColumnHasPositiveSwapGain) {
  // Original front drone at x=-0.05, rear drone at x=+0.05.
  EXPECT_NEAR(0.2, columnSwapGain(-0.05, 0, 0.05, 0, 1, 0), 1e-9);
  EXPECT_LT(columnSwapGain(0.5, 0, -0.5, 0, 1, 0), 0);
}

TEST(DistributedEdges, RuntimeRectanglesShareCanonicalSlotsAndEndpointLeases) {
  const auto saved = formation();
  struct Restore { FormationDescription saved; ~Restore() { formationStorage() = saved; } } restore{saved};
  const std::array<std::array<int, 3>, 4> cases{{{{2, 3, 7}}, {{2, 4, 10}},
                                               {{3, 4, 17}}, {{4, 4, 24}}}};
  for (const auto& shape : cases) {
    configureFormation(shape[0], shape[1]);
    const int count = shape[0] * shape[1];
    ASSERT_EQ(count, agentCount());
    ASSERT_EQ(shape[2], edgeCount());
    ASSERT_EQ(static_cast<size_t>(count), formation().slot_offsets.size());
    EXPECT_DOUBLE_EQ(0.5 * (shape[0] - 1), formation().slot_offsets.front()[0]);
    EXPECT_DOUBLE_EQ(-0.5 * (shape[1] - 1), formation().slot_offsets.front()[1]);
    EXPECT_DOUBLE_EQ(-formation().slot_offsets.front()[0], formation().slot_offsets.back()[0]);
    EXPECT_DOUBLE_EQ(-formation().slot_offsets.front()[1], formation().slot_offsets.back()[1]);
    std::set<std::array<int, 2>> unique;
    EdgeTable table;
    auto assignment = identityAssignment();
    std::reverse(assignment.begin(), assignment.end());
    for (int edge = 0; edge < edgeCount(); ++edge) {
      const auto pair = edges()[edge];
      ASSERT_GE(pair[0], 0); ASSERT_LT(pair[1], count);
      ASSERT_TRUE(unique.insert(pair).second);
      const int a = owner(assignment, pair[0]), b = owner(assignment, pair[1]);
      EXPECT_TRUE(table.updateAt(edge, a, b, assignment, a + 1, 1, 10, 10, 1, true));
      EXPECT_TRUE(table.updateAt(edge, b, a, assignment, b + 1, 1, 10, 10, 1, true));
    }
    EXPECT_EQ(fullMask(), table.mask(10, 1));
    EXPECT_EQ(0u, table.mask(11.1, 1));
    const auto connected = components(fullMask(), assignment);
    EXPECT_EQ(1, static_cast<int>(std::count(connected.begin(), connected.end(), '{')));
    assignment.pop_back();
    EXPECT_FALSE(validAssignment(assignment));
  }
}

TEST(DistributedEdges, ConfigurationDigestSeparatesShapesAndSpacing) {
  const auto saved = formation();
  struct Restore { FormationDescription saved; ~Restore() { formationStorage() = saved; } } restore{saved};
  configureFormation(2, 4);
  const auto standard = formationDigest();
  configureFormation(4, 2);
  EXPECT_NE(standard, formationDigest());
  configureFormation(2, 4, 1.0, 1.1);
  EXPECT_NE(standard, formationDigest());
  configureFormation(2, 4);
  EXPECT_EQ(standard, formationDigest());
  EXPECT_THROW(configureFormation(0, 4), std::invalid_argument);
  EXPECT_THROW(configureFormation(5, 4), std::invalid_argument);
  EXPECT_THROW(configureFormation(2, 4, 0.0, 1.0), std::invalid_argument);
  EXPECT_EQ(standard, formationDigest());
}

TEST(DistributedEdges, ReconnectAndWeakEvidenceDoNotNeedOverlapWork) {
  EXPECT_FALSE(needsOverlapScore(false, false, false, false));
  EXPECT_FALSE(needsOverlapScore(false, false, false, true));
  EXPECT_FALSE(needsOverlapScore(true, true, false, true));
  EXPECT_TRUE(localEdgeEvidenceSafe(false, false, 2.5, 1.0, 0.0,
                                    1.8, 4.0, 0.34));
  EXPECT_TRUE(localEdgeEvidenceSafe(true, true, 2.5, 1.0, 0.0,
                                    4.0, 4.0, 0.34));
}

TEST(DistributedEdges, StartupDefersOnlyExistingVisibilityExemption) {
  EXPECT_FALSE(needsOverlapScore(true, false, true, false));
  EXPECT_TRUE(needsOverlapScore(true, false, true, true));
  EXPECT_TRUE(needsOverlapScore(true, false, false, false));
  EXPECT_TRUE(needsOverlapScore(true, false, false, true));
  EXPECT_TRUE(localEdgeEvidenceSafe(true, true, 1.0, 1.0, 0.0,
                                    1.8, 4.0, 0.34));
  EXPECT_FALSE(localEdgeEvidenceSafe(true, false, 1.0, 1.0, 0.0,
                                     1.8, 4.0, 0.34));
}

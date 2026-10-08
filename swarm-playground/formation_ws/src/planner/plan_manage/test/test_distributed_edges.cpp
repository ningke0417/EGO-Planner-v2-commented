#include <gtest/gtest.h>
#include <limits>
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

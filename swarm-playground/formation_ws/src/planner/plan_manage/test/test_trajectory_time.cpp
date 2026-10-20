#include <gtest/gtest.h>
#include <plan_manage/trajectory_time.h>

using ego_planner::peerTrajectoryTime;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TrajectoryTime, SameStartKeepsRelativeTime) {
  EXPECT_DOUBLE_EQ(2.0, peerTrajectoryTime(100.0, 2.0, 100.0));
}

TEST(TrajectoryTime, LaterSelfStartAdvancesPeerTime) {
  // Our t=2 is absolute time 102, hence t=3 on the peer starting at 99.
  EXPECT_DOUBLE_EQ(3.0, peerTrajectoryTime(100.0, 2.0, 99.0));
}

TEST(TrajectoryTime, EarlierSelfStartReducesPeerTime) {
  EXPECT_DOUBLE_EQ(1.0, peerTrajectoryTime(99.0, 2.0, 100.0));
}

TEST(TrajectoryTime, BothTimesDescribeTheSameAbsoluteInstant) {
  const double starts[] = {99.0, 100.0, 101.0, 1792392558.25};
  for (double self_start : starts) {
    for (double offset : {-2.0, 0.0, 2.0}) {
      const double peer_start = self_start + offset;
      const double peer_time = peerTrajectoryTime(self_start, 0.125, peer_start);
      EXPECT_DOUBLE_EQ(self_start + 0.125, peer_start + peer_time);
    }
  }
}

TEST(TrajectoryTime, DoesNotSkipPeerThatIsAlreadyRunning) {
  // The old minus sign gave -0.5 and incorrectly skipped this check.
  const double peer_time = peerTrajectoryTime(100.0, 0.5, 99.0);
  EXPECT_DOUBLE_EQ(1.5, peer_time);
  EXPECT_TRUE(peer_time > 0.0 && peer_time < 2.0);
}

TEST(TrajectoryTime, DoesNotIncludePeerBeforeItStarts) {
  // The old minus sign gave 1.5 and incorrectly included this check.
  const double peer_time = peerTrajectoryTime(100.0, 0.5, 101.0);
  EXPECT_DOUBLE_EQ(-0.5, peer_time);
  EXPECT_FALSE(peer_time > 0.0 && peer_time < 2.0);
}

TEST(TrajectoryTime, DoesNotIncludePeerAfterItEnds) {
  // The old minus sign gave 0.5 instead of an already-expired peer time.
  const double peer_time = peerTrajectoryTime(100.0, 1.5, 99.0);
  EXPECT_DOUBLE_EQ(2.5, peer_time);
  EXPECT_FALSE(peer_time > 0.0 && peer_time < 2.0);
}

TEST(TrajectoryTime, KeepsOriginalStrictStartBoundary) {
  const double peer_time = peerTrajectoryTime(100.0, 1.0, 101.0);
  EXPECT_DOUBLE_EQ(0.0, peer_time);
  EXPECT_FALSE(peer_time > 0.0 && peer_time < 2.0);
}

TEST(TrajectoryTime, KeepsOriginalStrictEndBoundary) {
  const double peer_time = peerTrajectoryTime(100.0, 3.0, 101.0);
  EXPECT_DOUBLE_EQ(2.0, peer_time);
  EXPECT_FALSE(peer_time > 0.0 && peer_time < 2.0);
}

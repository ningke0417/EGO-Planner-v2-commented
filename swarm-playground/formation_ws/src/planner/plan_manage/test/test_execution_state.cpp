#include <gtest/gtest.h>

#include <stdexcept>
#include <traj_utils/execution_state.hpp>

using namespace ego_planner;

namespace {
struct LinearTrajectory {
  int pieces = 1;
  int getPieceNum() const { return pieces; }
  double getTotalDuration() const { return 2.0; }
  Eigen::Vector3d getPos(double t) const {
    if (pieces == 0 || t < 0.0 || t > 2.0) throw std::out_of_range("unclamped trajectory query");
    return Eigen::Vector3d(t, 2.0 * t, 1.5);
  }
  Eigen::Vector3d getVel(double t) const {
    getPos(t);
    return Eigen::Vector3d(1.0, 2.0, 0.0); // Intentionally nonzero at the end.
  }
  Eigen::Vector3d getAcc(double t) const {
    getPos(t);
    return Eigen::Vector3d::Zero();
  }
};
ExecutionFeedback running() {
  ExecutionFeedback f;
  f.session = 100;
  f.sequence = 20;
  f.trajectory_id = 7;
  f.trajectory_start_time = 100.0;
  f.duration = 2.0;
  f.stamp = 100.5;
  f.received_time = 100.51;
  f.position = Eigen::Vector3d(0.5, 1.0, 1.5);
  f.velocity = Eigen::Vector3d(1.0, 2.0, 0.0);
  f.mode = ExecutionMode::RUNNING;
  return f;
}
PeerPrediction predict(const ExecutionFeedback& feedback, double query = 101.25,
                       double now = 100.6, double received = 100.1) {
  return predictPeerTrajectory(LinearTrajectory(), 7, 100.0, 2.0, received,
                               feedback, query, now);
}
void expectPosition(const Eigen::Vector3d& expected, const PeerPrediction& p) {
  ASSERT_TRUE(p.valid);
  EXPECT_TRUE(p.position.isApprox(expected, 1e-10));
}
} // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(ExecutionState, UnequalTrajectoryStartsUseTheSameAbsoluteInstant) {
  EXPECT_DOUBLE_EQ(12.0, peerTime(100.0, 2.0, 90.0));
  EXPECT_DOUBLE_EQ(-8.0, peerTime(90.0, 2.0, 100.0));
  const double absolute = 101.25;
  EXPECT_DOUBLE_EQ(absolute - 100.0, peerTime(100.5, absolute - 100.5, 100.0));
  expectPosition(Eigen::Vector3d(1.25, 2.5, 1.5), predict(running(), absolute));
}

TEST(ExecutionState, FreshMatchingRunningUsesPolynomialAtAbsoluteQuery) {
  const auto p = predict(running());
  expectPosition(Eigen::Vector3d(1.25, 2.5, 1.5), p);
  EXPECT_FALSE(p.uncertain);
  EXPECT_FALSE(p.stationary);
  EXPECT_TRUE(p.from_execution);
  EXPECT_TRUE(p.velocity.isApprox(Eigen::Vector3d(1.0, 2.0, 0.0)));
}

TEST(ExecutionState, FutureStartClampsPositionAndDoesNotPretendAlreadyMoving) {
  auto f = running();
  f.stamp = f.received_time = 99.0;
  const auto p = predict(f, 99.5, 99.0);
  expectPosition(Eigen::Vector3d(0.0, 0.0, 1.5), p);
  EXPECT_TRUE(p.stationary);
  EXPECT_TRUE(p.velocity.isZero());
  EXPECT_FALSE(p.uncertain);
}

TEST(ExecutionState, NonzeroEndVelocityNeverCreatesLinearExtrapolation) {
  const auto p = predict(running(), 110.0);
  expectPosition(Eigen::Vector3d(2.0, 4.0, 1.5), p);
  EXPECT_TRUE(p.velocity.isZero());
  EXPECT_TRUE(p.acceleration.isZero());
  EXPECT_TRUE(p.stationary);
  const auto legacy = predict(ExecutionFeedback(), 110.0);
  expectPosition(Eigen::Vector3d(2.0, 4.0, 1.5), legacy);
  EXPECT_TRUE(legacy.velocity.isZero());
}

TEST(ExecutionState, FreshHoldingOverridesNewCachedPolynomialAndOldIdentity) {
  auto f = running();
  f.mode = ExecutionMode::HOLDING;
  f.trajectory_id = 6;
  f.position = Eigen::Vector3d(9.0, 8.0, 7.0);
  const auto p = predict(f, 120.0);
  expectPosition(f.position, p);
  EXPECT_TRUE(p.stationary);
  EXPECT_TRUE(p.velocity.isZero());
  EXPECT_FALSE(p.uncertain);
  LinearTrajectory empty;
  empty.pieces = 0;
  expectPosition(f.position,
      predictPeerTrajectory(empty, 0, 0.0, 0.0, 0.0, f, 120.0, 100.6));
}

TEST(ExecutionState, FinishedPositionPersistsWhenTrajectoryEndsOrFeedbackAges) {
  auto f = running();
  f.mode = ExecutionMode::FINISHED;
  f.position = Eigen::Vector3d(2.0, 4.0, 1.5);
  const auto fresh = predict(f, 1000.0);
  expectPosition(f.position, fresh);
  EXPECT_FALSE(fresh.uncertain);
  const auto stale = predict(f, 1000.0, 110.0);
  expectPosition(f.position, stale);
  EXPECT_TRUE(stale.uncertain);
  EXPECT_TRUE(stale.velocity.isZero());
}

TEST(ExecutionState, SourceStampAndReceiveTimeBothHaveToRemainFresh) {
  auto f = running();
  f.stamp = 99.0;
  f.received_time = 100.59;
  const auto old_source = predict(f);
  expectPosition(f.position, old_source);
  EXPECT_TRUE(old_source.uncertain);
  f = running(); f.received_time = 99.0;
  const auto old_transport = predict(f);
  expectPosition(f.position, old_transport);
  EXPECT_TRUE(old_transport.uncertain);
  f = running(); f.stamp = 101.0;
  EXPECT_TRUE(predict(f).uncertain);
}

TEST(ExecutionState, MismatchedTrajectoryOrReplanRequestFreezesReportedPosition) {
  auto f = running();
  ++f.trajectory_id;
  auto p = predict(f);
  expectPosition(f.position, p);
  EXPECT_TRUE(p.uncertain);
  EXPECT_TRUE(p.velocity.isZero());
  f = running(); f.trajectory_start_time += 0.1;
  EXPECT_TRUE(predict(f).uncertain);
  f = running(); f.duration += 0.1;
  EXPECT_TRUE(predict(f).uncertain);
  f = running(); f.require_replan = true;
  p = predict(f);
  expectPosition(f.position, p);
  EXPECT_TRUE(p.uncertain);
}

TEST(ExecutionState, LegacyUsesReceiveTtlAndNeverAgesEvidenceByFutureQuery) {
  const ExecutionFeedback unseen;
  const auto fresh = predict(unseen, 101.5, 100.2, 100.1);
  expectPosition(Eigen::Vector3d(1.5, 3.0, 1.5), fresh);
  EXPECT_FALSE(fresh.uncertain);
  const auto stale = predict(unseen, 110.0, 103.0, 100.1);
  expectPosition(Eigen::Vector3d(0.1, 0.2, 1.5), stale);
  EXPECT_TRUE(stale.uncertain);
  EXPECT_TRUE(stale.velocity.isZero());
  const auto expired_at_receive = predict(unseen, 110.0, 106.0, 103.0);
  expectPosition(Eigen::Vector3d(2.0, 4.0, 1.5), expired_at_receive);
  EXPECT_TRUE(expired_at_receive.velocity.isZero());
}

TEST(ExecutionState, MalformedFeedbackCannotSilentlyReenableLegacyMotion) {
  auto f = running();
  f.velocity.x() = std::numeric_limits<double>::quiet_NaN();
  expectPosition(f.position, predict(f));
  EXPECT_TRUE(predict(f).uncertain);
  f.position.x() = std::numeric_limits<double>::quiet_NaN();
  const auto p = predict(f);
  expectPosition(Eigen::Vector3d(0.0, 0.0, 1.5), p);
  EXPECT_TRUE(p.uncertain);
  f = running(); f.mode = static_cast<ExecutionMode>(0);
  EXPECT_TRUE(predict(f).uncertain);
}

TEST(ExecutionState, MissingTrajectoryAndNonfiniteQueriesFailClosed) {
  LinearTrajectory empty; empty.pieces = 0;
  EXPECT_FALSE(predictPeerTrajectory(empty, 0, 0.0, 0.0, 0.0,
                                     ExecutionFeedback(), 101.0, 100.6).valid);
  EXPECT_FALSE(predict(running(), std::numeric_limits<double>::quiet_NaN()).valid);
  EXPECT_FALSE(predictPeerTrajectory(LinearTrajectory(), 7, 100.0, 2.0, 100.1,
                                     running(), 101.0, 100.6, -1.0).valid);
  // The cache's claimed duration cannot authorize evaluation outside the real
  // polynomial; mismatching metadata freezes valid execution evidence.
  const auto mismatched = predictPeerTrajectory(LinearTrajectory(), 7, 100.0, 3.0,
                                                 100.1, running(), 110.0, 100.6);
  expectPosition(running().position, mismatched);
  EXPECT_TRUE(mismatched.uncertain);
}

TEST(ExecutionState, OutOfOrderSessionsSequencesAndSourceTimesAreRejected) {
  const auto old = running();
  auto next = old;
  EXPECT_FALSE(executionFeedbackNewer(next, old));
  --next.sequence;
  EXPECT_FALSE(executionFeedbackNewer(next, old));
  next = old; ++next.sequence;
  EXPECT_TRUE(executionFeedbackNewer(next, old));
  next.stamp -= 0.1;
  EXPECT_FALSE(executionFeedbackNewer(next, old));
  next = old; --next.session; next.sequence += 100;
  EXPECT_FALSE(executionFeedbackNewer(next, old));
  next = old; ++next.session; next.sequence = 1;
  EXPECT_TRUE(executionFeedbackNewer(next, old));
}

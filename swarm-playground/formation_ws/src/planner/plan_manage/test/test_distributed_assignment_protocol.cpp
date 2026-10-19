#include <gtest/gtest.h>

#include <plan_manage/distributed_reassignment.h>

namespace ego_planner {
namespace distributed {
// Exercise the exact runtime validators without a ROS master or private macro.
struct AssignmentProtocolTestAccess {
  using Protocol = AllAgentAssignment;
  using Message = Protocol::Message;
  static bool proposal(const Message& m) { return Protocol::validProposal(m); }
  static bool certificate(const Message& m) { return Protocol::validCertificate(m); }
  static bool same(const Message& a, const Message& b) { return Protocol::same(a, b); }
  static bool requestSame(const Message& a, const Message& b) {
    return Protocol::requestSame(a, b);
  }
  static bool token(const Message& a, const Message& b) { return Protocol::sameToken(a, b); }
  static std::string encode(const Message& m) { return Protocol::encode(m); }
  static bool decode(const std::string& text, Message* m) { return Protocol::decode(text, m); }
};
} // namespace distributed
} // namespace ego_planner

using namespace ego_planner::distributed;
using Access = AssignmentProtocolTestAccess;
using Message = AllAgentAssignment::Message;

namespace {
Message prepared() {
  Message m;
  m.header.stamp = ros::Time(11, 0);
  m.snapshot_stamp = ros::Time(10, 0);
  m.phase = Message::PREPARE;
  m.sender_id = m.proposer_id = 1;
  m.sender_session = m.proposer_session = 102;
  m.sequence = 8;
  m.transaction_id = 5;
  m.base_epoch = 1;
  m.assignment_epoch = 2;
  const auto base = identityAssignment();
  const Assignment next{{1, 2, 0, 3, 4, 5, 6, 7}};
  for (int id = 0; id < kAgents; ++id) {
    m.base_assignment[id] = base[id];
    m.drone_to_slot[id] = next[id];
    m.member_sessions[id] = id + 101;
    m.member_sequences[id] = 20 + id;
    for (int slot = 0; slot < kAgents; ++slot)
      m.costs[id * kAgents + slot] = slot == next[id] ? 1.0 : 20.0;
  }
  m.base_digest = fullAssignmentSignature(base);
  m.assignment_digest = fullAssignmentSignature(next);
  m.gain = 55.5;
  return m;
}
Message committed() {
  Message m = prepared();
  m.phase = Message::COMMIT;
  for (int id = 0; id < kAgents; ++id) {
    m.ack_sessions[id] = m.member_sessions[id];
    m.ack_sequences[id] = 100 + id;
    m.ack_stamps[id] = 10.8;
  }
  return m;
}
} // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(AssignmentProtocol, FullSnapshotPermutationAndDigestMustAgree) {
  auto m = prepared();
  EXPECT_TRUE(Access::proposal(m));
  m.drone_to_slot[0] = m.drone_to_slot[1];
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); ++m.assignment_digest;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); ++m.base_digest;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.assignment_epoch = m.base_epoch + 2;
  EXPECT_FALSE(Access::proposal(m));
}

TEST(AssignmentProtocol, CoordinatorAndImmutableMembershipAreValidated) {
  auto m = prepared();
  m.proposer_id = 2;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); ++m.proposer_session;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.member_sequences[7] = 0;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.member_sessions[7] = 0;
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.snapshot_stamp = ros::Time();
  EXPECT_FALSE(Access::proposal(m));
}

TEST(AssignmentProtocol, NonfiniteStatesGoalsCostsAndGainAreRejected) {
  auto m = prepared();
  m.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.velocities[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.slot_goals[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.costs[7] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Access::proposal(m));
  m = prepared(); m.costs[7] = -1.0;
  EXPECT_TRUE(Access::proposal(m)); // The on-wire unavailable sentinel is finite.
  m.gain = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Access::proposal(m));
}

TEST(AssignmentProtocol, CertificateRequiresAllEightOriginalMemberVotes) {
  const auto valid = committed();
  EXPECT_TRUE(Access::certificate(valid));
  for (int id = 0; id < kAgents; ++id) {
    auto missing = valid;
    missing.ack_sequences[id] = 0;
    EXPECT_FALSE(Access::certificate(missing));
    auto restarted = valid;
    ++restarted.ack_sessions[id];
    EXPECT_FALSE(Access::certificate(restarted));
  }
  auto no_decision = valid;
  no_decision.phase = Message::PREPARE;
  EXPECT_FALSE(Access::certificate(no_decision));
  no_decision = valid; no_decision.gain = 0.0;
  EXPECT_FALSE(Access::certificate(no_decision));
}

TEST(AssignmentProtocol, VoteTimesMustBelongToSnapshotAndDecision) {
  auto m = committed();
  m.ack_stamps[4] = 9.99;
  EXPECT_FALSE(Access::certificate(m));
  m = committed(); m.ack_stamps[4] = 11.06;
  EXPECT_FALSE(Access::certificate(m));
  m = committed(); m.ack_stamps[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Access::certificate(m));
  m = committed(); m.header.stamp = ros::Time();
  EXPECT_FALSE(Access::certificate(m));
}

TEST(AssignmentProtocol, RetransmissionDoesNotMutateReservedProposal) {
  const auto a = prepared();
  auto b = a;
  b.header.stamp = ros::Time(12, 0);
  b.sender_id = 7; b.sender_session = 108; ++b.sequence;
  b.phase = Message::ACK;
  EXPECT_TRUE(Access::same(a, b));
  b.costs[5] += 0.1;
  EXPECT_FALSE(Access::same(a, b));
  b = a; ++b.member_sequences[5];
  EXPECT_FALSE(Access::same(a, b));
  b = a; ++b.transaction_id;
  EXPECT_FALSE(Access::same(a, b));
  b = a; b.positions[5] += 0.01;
  EXPECT_FALSE(Access::same(a, b));
}

TEST(AssignmentProtocol, BidsShareSnapshotButOnlyFillCostRows) {
  const auto a = prepared();
  auto b = a;
  b.costs[5] += 0.1;
  b.assignment_digest = fullAssignmentSignature(identityAssignment());
  b.drone_to_slot = b.base_assignment;
  EXPECT_TRUE(Access::requestSame(a, b));
  b.member_sessions[3] += 1;
  EXPECT_FALSE(Access::requestSame(a, b));
  b = a; b.slot_goals[4] += 0.1;
  EXPECT_FALSE(Access::requestSame(a, b));
}

TEST(AssignmentProtocol, AbortTokenSurvivesNewCoordinatorSenderSession) {
  const auto a = prepared();
  auto aborted = a;
  aborted.phase = Message::ABORT;
  ++aborted.sender_session;
  ++aborted.sequence;
  EXPECT_TRUE(Access::token(a, aborted));
  EXPECT_TRUE(Access::same(a, aborted));
  // A delayed altered packet is still blacklisted by its transaction token,
  // but must not release a differently reserved immutable proposal.
  aborted.costs[0] += 0.1;
  EXPECT_TRUE(Access::token(a, aborted));
  EXPECT_FALSE(Access::same(a, aborted));
  aborted = a; ++aborted.transaction_id;
  EXPECT_FALSE(Access::token(a, aborted));
  aborted = a; ++aborted.proposer_session;
  EXPECT_FALSE(Access::token(a, aborted));
}

TEST(AssignmentProtocol, JournalEncodingPreservesCertificatesAndRejectsTruncation) {
  const auto a = committed();
  const auto encoded = Access::encode(a);
  Message b;
  ASSERT_TRUE(Access::decode(encoded, &b));
  EXPECT_TRUE(Access::same(a, b));
  EXPECT_TRUE(Access::certificate(b));
  EXPECT_EQ(a.ack_sessions, b.ack_sessions);
  EXPECT_EQ(a.ack_sequences, b.ack_sequences);
  EXPECT_EQ(a.ack_stamps, b.ack_stamps);
  EXPECT_FALSE(Access::decode(encoded.substr(0, encoded.size() - 2), &b));
  EXPECT_FALSE(Access::decode(encoded + "00", &b));
  EXPECT_FALSE(Access::decode("xyz", &b));
  EXPECT_FALSE(Access::decode(std::string(20002, '0'), &b));
}

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <ros/master.h>
#include <plan_manage/distributed_reassignment.h>

namespace ego_planner {
namespace distributed {
struct AssignmentProtocolTestAccess {
  using Protocol = AllAgentAssignment;
  using Message = Protocol::Message;
  static bool aborted(const Protocol& p, const Message& m) { return p.isAborted(m); }
  static bool locked(const Protocol& p) { return static_cast<bool>(p.locked_); }
  static void latePrepare(Protocol& p, const Message& m) { p.reserve(m); }
  static void lateCommit(Protocol& p, Message m) { p.commit(m); }
};
} // namespace distributed
} // namespace ego_planner

using namespace ego_planner::distributed;
using Protocol = AllAgentAssignment;
using Message = Protocol::Message;
using Access = AssignmentProtocolTestAccess;

namespace {
void require(bool condition, const std::string& why) {
  if (!condition) throw std::runtime_error(why);
}

struct Fixture {
  std::array<std::unique_ptr<Protocol>, kAgents> nodes;
  std::array<uint64_t, kAgents> epochs{}, sessions{}, state_sequences{};
  std::array<Assignment, kAgents> assignments;
  std::array<std::array<double, 3>, kAgents> positions{}, goals{};
  std::array<unsigned, kAgents> certifications{};
  bool block_last_vote = true;
  std::vector<Message> prepares, aborts, commits;
  ros::Subscriber observer;

  Fixture(ros::NodeHandle nh) {
    for (int slot = 0; slot < kAgents; ++slot)
      goals[slot] = {{slot < kColumns ? 10.0 : 9.0,
                       -1.5 + slot % kColumns, 1.5}};
    setPositions(Assignment{{3, 2, 1, 0, 7, 6, 5, 4}});
    for (int id = 0; id < kAgents; ++id) {
      epochs[id] = 1;
      sessions[id] = id + 101;
      state_sequences[id] = 1;
      assignments[id] = identityAssignment();
      create(id);
    }
    observer = nh.subscribe<Message>("/swarm/assignment", 1024,
        [this](const Message::ConstPtr& m) {
          if (m->phase == Message::PREPARE) prepares.push_back(*m);
          if (m->phase == Message::ABORT) aborts.push_back(*m);
          if (m->phase == Message::COMMIT) commits.push_back(*m);
        });
  }

  void setPositions(const Assignment& desired) {
    for (int id = 0; id < kAgents; ++id) {
      positions[id] = goals[desired[id]];
      positions[id][0] -= 2.0;
    }
  }
  Protocol::View view() {
    Protocol::View result{};
    const double now = ros::Time::now().toSec();
    for (int id = 0; id < kAgents; ++id) {
      auto& m = result[id];
      m.lease.session = sessions[id];
      m.lease.sequence = state_sequences[id];
      m.lease.stamp = now;
      m.epoch = epochs[id];
      m.assignment = assignments[id];
      m.digest = fullAssignmentSignature(assignments[id]);
      m.position = positions[id];
      m.goal = goals[assignments[id][id]];
      m.goal_valid = m.applied = true;
    }
    return result;
  }
  void create(int id) {
    ros::NodeHandle scoped("/assignment_protocol_test/drone_" + std::to_string(id));
    scoped.setParam("reassign_period", 0.1);
    scoped.setParam("reassign_stable_hold", 0.0);
    scoped.setParam("swap_cooldown", 0.3);
    scoped.setParam("swap_gain_min", 0.1);
    scoped.setParam("reassign_relative_gain", 0.01);
    scoped.setParam("reassign_switch_penalty", 0.05);
    scoped.setParam("reassign_heading_weight", 0.0);
    scoped.setParam("reassign_lock_distance", 0.2);
    scoped.setParam("reassign_goal_window", 8.0);
    scoped.setParam("reassign_transaction_timeout", 4.0);
    scoped.setParam("evidence_timeout", 1.5);
    nodes[id].reset(new Protocol(scoped, id, sessions[id], epochs[id], assignments[id],
        [this]() { return view(); },
        [this, id](const std::array<double, 3>& target) {
          // Each cost row starts with slot zero, but this fixture's member7
          // candidate is slot4/slot7. Thus repeat REQUESTs also get eight valid
          // BID certifications even if initial topic connections lose a row.
          if (target == goals[0]) certifications[id] = 0;
          const unsigned call = certifications[id]++;
          // Eight initial calls produce this member's BID row. Only its later
          // candidate vote is denied, so others must not mistake bids for ACKs.
          return !(id == kAgents - 1 && block_last_vote && call >= kAgents);
        },
        [this, id](uint64_t epoch, const Assignment& assignment) {
          epochs[id] = epoch;
          assignments[id] = assignment;
        }));
  }
  void tick() {
    for (auto& sequence : state_sequences) ++sequence;
    for (auto& node : nodes) node->tick(true);
    ros::spinOnce();
  }
  bool all(uint64_t epoch, const Assignment& assignment) const {
    for (int id = 0; id < kAgents; ++id)
      if (epochs[id] != epoch || assignments[id] != assignment) return false;
    return true;
  }
  bool allAborted(const Message& token) const {
    for (const auto& node : nodes)
      if (!Access::aborted(*node, token) || Access::locked(*node)) return false;
    return true;
  }
  void checkDelayedPackets(const Message& old, uint64_t epoch, const Assignment& assignment) {
    require(all(epoch, assignment), "targets changed before an eight-vote commit");
    for (auto& node : nodes) {
      Access::latePrepare(*node, old);
      require(!Access::locked(*node), "aborted late PREPARE reacquired lock");
      Access::lateCommit(*node, old);
    }
    require(all(epoch, assignment), "aborted late COMMIT changed targets");
  }
};
} // namespace

int main(int argc, char** argv) {
  const char* master = std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master) != "http://127.0.0.1:11339") {
    std::cerr << "Refuse to run outside dedicated ROS_MASTER_URI=http://127.0.0.1:11339\n";
    return 2;
  }
  ros::init(argc, argv, "distributed_assignment_protocol_runtime");
  ros::NodeHandle nh;
  try {
    require(ros::master::check(), "start a fresh dedicated master on port 11339 first");
    require(!nh.hasParam("/distributed_assignment_journal"),
            "dedicated test master already has a journal; use a fresh test master");
    Fixture test(nh);
    const Assignment reversed{{3, 2, 1, 0, 7, 6, 5, 4}};
    unsigned stage = 0;
    Message first, second;
    bool have_first = false, have_second = false, restarted = false;
    double stage_start = ros::WallTime::now().toSec();
    const auto deadline = ros::WallTime::now() + ros::WallDuration(30.0);
    ros::WallRate rate(100);
    while (ros::ok() && ros::WallTime::now() < deadline) {
      test.tick();
      if (!have_first && !test.prepares.empty()) { first = test.prepares.front(); have_first = true; }
      if (stage == 0 && have_first) {
        require(test.all(1, identityAssignment()), "seven votes unexpectedly committed first transaction");
        if (test.allAborted(first)) {
          require(ros::WallTime::now().toSec() - first.snapshot_stamp.toSec() >= 3.8,
                  "participant released a prepared vote before coordinator timeout");
          test.checkDelayedPackets(first, 1, identityAssignment());
          test.block_last_vote = false;
          stage = 1;
          std::cout << "PASS: missing eighth ACK -> durable coordinator ABORT, old targets retained\n";
        }
      } else if (stage == 1 && test.all(2, reversed)) {
        test.block_last_vote = true;
        test.certifications.fill(0);
        test.setPositions(identityAssignment());
        stage = 2;
        stage_start = ros::WallTime::now().toSec();
        std::cout << "PASS: retry obtains all eight ACKs and commits cross-column assignment\n";
      } else if (stage == 2) {
        require(test.all(2, reversed), "second transaction committed despite denied eighth vote");
        for (const auto& candidate : test.prepares)
          if (candidate.base_epoch == 2) { second = candidate; have_second = true; break; }
        if (have_second && !restarted && Access::locked(*test.nodes[2])) {
          test.nodes[2].reset();
          test.sessions[2] += 1000;
          test.create(2);
          restarted = true;
          std::cout << "PASS: coordinator restarted with persisted prepared transaction\n";
        }
        if (restarted && test.allAborted(second)) {
          test.checkDelayedPackets(second, 2, reversed);
          test.block_last_vote = false;
          stage = 3;
          std::cout << "PASS: recovered coordinator ABORTs old-session reservation and releases peers\n";
        }
        require(ros::WallTime::now().toSec() - stage_start < 12.0,
                "coordinator restart did not resolve prepared transaction");
      } else if (stage == 3 && test.all(3, identityAssignment())) {
        // Restart a different participant after two commits: its full chain,
        // final mapping and both abort blacklists must survive process state.
        test.nodes[1].reset();
        test.sessions[1] += 1000;
        test.create(1);
        require(test.all(3, identityAssignment()), "journal restart lost committed mapping");
        require(Access::aborted(*test.nodes[1], first) && Access::aborted(*test.nodes[1], second),
                "journal restart forgot aborted tokens");
        std::vector<std::string> chain;
        require(nh.getParam("/distributed_assignment_journal/drone_1/commits", chain) && chain.size() == 2,
                "journal did not retain the full two-commit certificate chain");
        std::cout << "PASS: retry after recovery commits; two-epoch history and abort blacklist restored\n";
        // Six parked members must reserve their own slots without freezing a
        // useful cross-column reassignment for the two remaining aircraft.
        const Assignment two_left{{3, 1, 2, 0, 4, 5, 6, 7}};
        for (int id = 0; id < kAgents; ++id) test.positions[id] = test.goals[two_left[id]];
        test.positions[0][0] -= 2.0;
        test.positions[3][0] -= 2.0;
        stage = 4;
      } else if (stage == 4 && test.all(4, Assignment{{3, 1, 2, 0, 4, 5, 6, 7}})) {
        for (int id : {1, 2, 4, 5, 6, 7})
          require(test.assignments[id][id] == id, "an arrived aircraft was displaced from its locked slot");
        std::cout << "PASS: six arrived slots stay locked while the two remaining aircraft reassign\n";
        return 0;
      }
      rate.sleep();
    }
    throw std::runtime_error("protocol runtime test timed out at stage " + std::to_string(stage));
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}

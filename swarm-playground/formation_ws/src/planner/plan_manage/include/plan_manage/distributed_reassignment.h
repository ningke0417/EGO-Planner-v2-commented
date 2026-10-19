#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ego_planner/FormationAssignment.h>
#include <plan_manage/distributed_assignment.h>

namespace ego_planner {
namespace distributed {

// Trusted-member, rotating-coordinator agreement, not Byzantine consensus or
// atomic physical activation. Each bid is certified using only its owner's map.
class AllAgentAssignment {
 public:
  using Message = ego_planner::FormationAssignment;
  struct Member {
    Lease lease;
    Assignment assignment{identityAssignment()};
    uint64_t epoch{0}, digest{0};
    std::array<double, 3> position{}, velocity{}, goal{};
    bool goal_valid{false}, applied{false};
  };
  using View = std::array<Member, kAgents>;
  using ReadView = std::function<View()>;
  using Certify = std::function<bool(const std::array<double, 3>&)>;
  using Apply = std::function<void(uint64_t, const Assignment&)>;

  AllAgentAssignment(ros::NodeHandle nh, int id, uint64_t session,
                     uint64_t& epoch, Assignment& assignment,
                     ReadView view, Certify certify, Apply apply)
      : nh_(nh), id_(id), session_(session), epoch_(epoch), assignment_(assignment),
        view_(view), certify_(certify), apply_(apply) {
    nh_.param("swap_gain_min", min_gain_, 0.15);
    nh_.param("swap_cooldown", cooldown_, 8.0);
    nh_.param("reassign_period", period_, 0.5);
    nh_.param("reassign_stable_hold", hold_, 0.6);
    nh_.param("reassign_relative_gain", relative_gain_, 0.05);
    nh_.param("reassign_switch_penalty", switch_penalty_, 0.15);
    nh_.param("reassign_heading_weight", heading_weight_, 0.2);
    nh_.param("reassign_goal_window", goal_window_, 8.0);
    nh_.param("reassign_lock_distance", lock_distance_, 0.8);
    nh_.param("reassign_transaction_timeout", transaction_timeout_, 4.0);
    nh_.param("evidence_timeout", ttl_, 1.5);
    if (!std::isfinite(min_gain_) || !std::isfinite(cooldown_) ||
        !std::isfinite(period_) || !std::isfinite(hold_) ||
        !std::isfinite(relative_gain_) || !std::isfinite(switch_penalty_) ||
        !std::isfinite(heading_weight_) || !std::isfinite(goal_window_) ||
        !std::isfinite(lock_distance_) || !std::isfinite(transaction_timeout_) ||
        !std::isfinite(ttl_) || ttl_ <= 0 || transaction_timeout_ <= ttl_ ||
        min_gain_ < 0 || cooldown_ < 0 ||
        period_ <= 0 || hold_ < 0 || relative_gain_ < 0 || switch_penalty_ < 0 ||
        heading_weight_ < 0 || lock_distance_ < 0 || goal_window_ <= lock_distance_)
      throw std::runtime_error("invalid eight-agent reassignment parameters");
    // Per-agent ROS parameter journal survives a process restart in this master.
    // It is not durable storage across loss/restart of the ROS master itself.
    journal_ = "/distributed_assignment_journal/drone_" + std::to_string(id_);
    std::string saved;
    std::vector<std::string> entries;
    bool legacy_fallback = false;
    if (nh_.hasParam(journal_ + "/commits")) {
      if (!nh_.getParam(journal_ + "/commits", entries))
        throw std::runtime_error("invalid assignment commit journal type");
    } else if (nh_.getParam(journal_ + "/committed", saved)) {
      entries.push_back(saved);
      legacy_fallback = true;
    }
    for (const auto& entry : entries) {
      Message msg;
      if (!decode(entry, &msg))
        throw std::runtime_error("invalid committed assignment encoding");
      if (legacy_fallback) msg.phase = Message::COMMIT;
      if (!validCertificate(msg) ||
          (history_.empty() && !legacy_fallback &&
           (msg.base_epoch != 1 || mapping(msg.base_assignment) != identityAssignment())) ||
          (!history_.empty() &&
           (msg.base_epoch != history_.back().assignment_epoch ||
            msg.base_assignment != history_.back().drone_to_slot)))
        throw std::runtime_error("invalid committed assignment chain; refuse unsafe reset");
      history_.push_back(msg);
    }
    if (!history_.empty()) {
      epoch_ = history_.back().assignment_epoch;
      assignment_ = mapping(history_.back().drone_to_slot);
      gate_.committed(ros::Time::now().toSec());
    }
    entries.clear();
    if (nh_.hasParam(journal_ + "/aborts") &&
        !nh_.getParam(journal_ + "/aborts", entries))
      throw std::runtime_error("invalid assignment abort journal type");
    for (const auto& entry : entries) {
      Message msg;
      if (!decode(entry, &msg) || !validProposal(msg) ||
          msg.phase != Message::ABORT || msg.sender_id != msg.proposer_id ||
          isCommitted(msg))
        throw std::runtime_error("invalid aborted assignment journal; refuse unsafe reset");
      aborted_.push_back(msg);
    }
    if (nh_.getParam(journal_ + "/prepared", saved)) {
      Message msg;
      if (!decode(saved, &msg) || !validProposal(msg))
        throw std::runtime_error("invalid prepared assignment journal; refuse unsafe reset");
      if (msg.base_epoch == epoch_ && !isAborted(msg)) {
        locked_.reset(new Message(msg));
        ROS_WARN("[ASSIGNMENT_WAIT] drone=%d recovered unresolved vote; awaiting same decision", id_);
      }
    }
    pub_ = nh_.advertise<Message>("/swarm/assignment", 64);
    sub_ = nh_.subscribe("/swarm/assignment", 128, &AllAgentAssignment::receive, this);
  }

  void tick(bool armed) {
    const auto now = ros::Time::now();
    const View members = view_();
    replay(members, now);
    if (locked_) {
      // Participants never release their own vote on a timeout. Only the
      // unique live process for the rotating coordinator's physical ID may
      // make and journal one durable COMMIT/ABORT decision. A restarted
      // coordinator resolves its previous-session reservation by ABORT, never
      // by inventing missing votes. This assumes trusted, unique member IDs.
      if (locked_->proposer_id == static_cast<unsigned>(id_) &&
          due(last_send_, now, 0.25)) {
        if (locked_->proposer_session != session_) {
          abort(*locked_, true);
          last_send_ = now;
          return;
        }
        bool complete = true;
        for (int i = 0; i < kAgents; ++i)
          complete = complete && acks_[i].fresh(now.toSec(), ttl_) &&
              members[i].lease.fresh(now.toSec(), ttl_) &&
              members[i].lease.session == acks_[i].session;
        if (complete) {
          Message decision = *locked_;
          decision.header.stamp = now;
          decision.phase = Message::COMMIT;
          for (int i = 0; i < kAgents; ++i) {
            decision.ack_sessions[i] = acks_[i].session;
            decision.ack_sequences[i] = acks_[i].sequence;
            decision.ack_stamps[i] = acks_[i].stamp;
          }
          if (validCertificate(decision)) {
            commit(decision);
            publish(decision, Message::COMMIT);
          }
        } else if ((now - locked_->snapshot_stamp).toSec() >= transaction_timeout_) {
          abort(*locked_, true);
        } else {
          publish(*locked_, Message::PREPARE);
        }
        last_send_ = now;
      }
      return;
    }
    if (!armed || !ready(members, now)) {
      gate_.resetCandidate();
      collecting_.reset();
      return;
    }
    if (id_ != static_cast<int>(epoch_ % kAgents) ||
        !due(last_request_, now, period_)) return;
    if (collecting_ && (now - collecting_->snapshot_stamp).toSec() > ttl_)
      gate_.resetCandidate();
    if (collecting_ && (now - collecting_->snapshot_stamp).toSec() <= ttl_) {
      bool all_bids = true;
      for (const auto& bid : bids_) all_bids = all_bids && bid.fresh(now.toSec(), ttl_);
      if (all_bids) {
        AssignmentCosts costs{};
        for (int i = 0; i < kAgents; ++i)
          for (int j = 0; j < kAgents; ++j) {
            const double value = collecting_->costs[i * kAgents + j];
            costs[i][j] = value >= 0 ? value : std::numeric_limits<double>::infinity();
          }
        const auto solution = solveAssignment(costs, assignment_, switch_penalty_);
        double remaining = std::numeric_limits<double>::infinity();
        for (int i = 0; i < kAgents; ++i)
          if (solution.assignment[i] != assignment_[i])
            remaining = std::min(remaining, distance(members[i].position, members[i].goal));
        if (gate_.ready(solution, now.toSec(), hold_, cooldown_, remaining,
                        lock_distance_, min_gain_, relative_gain_)) {
          Message proposal = *collecting_;
          for (int i = 0; i < kAgents; ++i) proposal.drone_to_slot[i] = solution.assignment[i];
          proposal.assignment_digest = fullAssignmentSignature(solution.assignment);
          proposal.gain = solution.net_gain;
          // Reserve/journal before any PREPARE is sent; never replace this
          // proposal with a different mapping at this base epoch.
          reserve(proposal);
          publish(proposal, Message::PREPARE);
          collecting_.reset();
          last_send_ = now;
          return;
        }
      } else if ((now - collecting_->snapshot_stamp).toSec() < period_ + 0.25) return;
      if (!all_bids) gate_.resetCandidate();
    }
    request(members, now);
  }

 private:
  friend struct AssignmentProtocolTestAccess;
  template <typename Array> static Assignment mapping(const Array& a) {
    Assignment out{};
    for (int i = 0; i < kAgents; ++i) out[i] = a[i];
    return out;
  }
  static double distance(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    double sum = 0;
    for (int j = 0; j < 3; ++j) sum += (a[j] - b[j]) * (a[j] - b[j]);
    return std::sqrt(sum);
  }
  static bool due(const ros::Time& last, const ros::Time& now, double interval) {
    return last.isZero() || (now - last).toSec() >= interval;
  }
  bool ready(const View& members, const ros::Time& now) const {
    for (const auto& m : members) {
      for (double value : m.position) if (!std::isfinite(value)) return false;
      for (double value : m.velocity) if (!std::isfinite(value)) return false;
      for (double value : m.goal) if (!std::isfinite(value)) return false;
      const double remaining = distance(m.position, m.goal);
      if (!m.lease.fresh(now.toSec(), ttl_) || !m.goal_valid || !m.applied ||
          m.epoch != epoch_ || m.digest != fullAssignmentSignature(assignment_) ||
          m.assignment != assignment_ ||
          remaining > goal_window_) return false;
    }
    return true;
  }
  static bool same(const Message& a, const Message& b) {
    return a.base_epoch == b.base_epoch && a.assignment_epoch == b.assignment_epoch &&
        a.base_digest == b.base_digest && a.assignment_digest == b.assignment_digest &&
        a.base_assignment == b.base_assignment && a.drone_to_slot == b.drone_to_slot &&
        a.proposer_id == b.proposer_id && a.proposer_session == b.proposer_session &&
        a.transaction_id == b.transaction_id && a.snapshot_stamp == b.snapshot_stamp &&
        a.member_sessions == b.member_sessions && a.member_sequences == b.member_sequences &&
        a.positions == b.positions && a.velocities == b.velocities &&
        a.slot_goals == b.slot_goals && a.costs == b.costs && a.gain == b.gain;
  }
  static bool requestSame(const Message& a, const Message& b) {
    return a.base_epoch == b.base_epoch && a.base_digest == b.base_digest &&
        a.base_assignment == b.base_assignment &&
        a.proposer_id == b.proposer_id && a.proposer_session == b.proposer_session &&
        a.transaction_id == b.transaction_id && a.snapshot_stamp == b.snapshot_stamp &&
        a.member_sessions == b.member_sessions && a.member_sequences == b.member_sequences &&
        a.positions == b.positions && a.velocities == b.velocities && a.slot_goals == b.slot_goals;
  }
  static bool validProposal(const Message& m) {
    if (m.base_epoch == 0 || m.base_epoch == std::numeric_limits<uint64_t>::max() ||
        m.assignment_epoch != m.base_epoch + 1 || m.transaction_id == 0 ||
        m.proposer_session == 0 || m.proposer_id != m.base_epoch % kAgents ||
        m.snapshot_stamp.isZero() ||
        !validAssignment(mapping(m.base_assignment)) || !validAssignment(mapping(m.drone_to_slot)) ||
        m.base_digest != fullAssignmentSignature(mapping(m.base_assignment)) ||
        m.assignment_digest != fullAssignmentSignature(mapping(m.drone_to_slot))) return false;
    for (int i = 0; i < kAgents; ++i)
      if (m.member_sessions[i] == 0 || m.member_sequences[i] == 0) return false;
    if (m.member_sessions[m.proposer_id] != m.proposer_session) return false;
    for (double value : m.positions) if (!std::isfinite(value)) return false;
    for (double value : m.velocities) if (!std::isfinite(value)) return false;
    for (double value : m.slot_goals) if (!std::isfinite(value)) return false;
    for (double value : m.costs) if (!std::isfinite(value)) return false;
    return std::isfinite(m.gain);
  }
  static bool validCertificate(const Message& m) {
    if (!validProposal(m) || m.header.stamp.isZero() ||
        m.phase != Message::COMMIT || m.gain <= 0.0 ||
        m.base_assignment == m.drone_to_slot) return false;
    for (int i = 0; i < kAgents; ++i)
      if (m.ack_sessions[i] != m.member_sessions[i] || m.ack_sequences[i] == 0 ||
          !std::isfinite(m.ack_stamps[i]) || m.ack_stamps[i] < m.snapshot_stamp.toSec() ||
          m.ack_stamps[i] > m.header.stamp.toSec() + 0.05) return false;
    return true;
  }
  // An epoch-scoped coordinator transaction ID identifies the decision even
  // if a delayed packet tries to alter other fields in that transaction.
  static bool sameToken(const Message& a, const Message& b) {
    return a.base_epoch == b.base_epoch && a.proposer_id == b.proposer_id &&
        a.proposer_session == b.proposer_session && a.transaction_id == b.transaction_id;
  }
  bool isAborted(const Message& m) const {
    for (const auto& aborted : aborted_) if (sameToken(aborted, m)) return true;
    return false;
  }
  bool isCommitted(const Message& m) const {
    for (const auto& decision : history_) if (sameToken(decision, m)) return true;
    return false;
  }
  void saveChain(const std::string& key, const std::vector<Message>& chain) {
    std::vector<std::string> encoded;
    encoded.reserve(chain.size());
    for (const auto& message : chain) encoded.push_back(encode(message));
    nh_.setParam(journal_ + key, encoded);
  }
  bool matchesCurrent(const Message& m) const {
    return m.base_epoch == epoch_ && m.base_digest == fullAssignmentSignature(assignment_) &&
           mapping(m.base_assignment) == assignment_;
  }
  bool liveSnapshot(const Message& m, const View& members, const ros::Time& now) const {
    if (!ready(members, now) || !matchesCurrent(m) || now < m.snapshot_stamp ||
        (now - m.snapshot_stamp).toSec() > ttl_) return false;
    for (int i = 0; i < kAgents; ++i) {
      const auto& member = members[i];
      if (member.lease.session != m.member_sessions[i] ||
          member.lease.sequence < m.member_sequences[i]) return false;
      std::array<double, 3> old_position{}, goal{};
      for (int j = 0; j < 3; ++j) {
        old_position[j] = m.positions[3 * i + j];
        goal[j] = m.slot_goals[3 * assignment_[i] + j];
      }
      if (distance(member.position, old_position) > 0.6 || distance(member.goal, goal) > 0.05)
        return false;
    }
    return true;
  }
  void request(const View& members, const ros::Time& now) {
    collecting_.reset(new Message());
    auto& m = *collecting_;
    m.proposer_id = id_; m.proposer_session = session_;
    m.transaction_id = ++transactions_;
    m.base_epoch = epoch_; m.assignment_epoch = epoch_ + 1;
    m.base_digest = m.assignment_digest = fullAssignmentSignature(assignment_);
    m.snapshot_stamp = now;
    for (int i = 0; i < kAgents; ++i) {
      m.base_assignment[i] = m.drone_to_slot[i] = assignment_[i];
      m.member_sessions[i] = members[i].lease.session;
      m.member_sequences[i] = members[i].lease.sequence;
      for (int j = 0; j < 3; ++j) {
        m.positions[3 * i + j] = members[i].position[j];
        m.velocities[3 * i + j] = members[i].velocity[j];
        m.slot_goals[3 * assignment_[i] + j] = members[i].goal[j];
      }
    }
    std::fill(m.costs.begin(), m.costs.end(), -1.0);
    bids_ = {};
    publish(m, Message::REQUEST);
    last_request_ = now;
  }
  bool ownCosts(const Message& m, std::array<double, kAgents>* row) const {
    std::array<double, 3> p{}, v{};
    for (int j = 0; j < 3; ++j) { p[j] = m.positions[3 * id_ + j]; v[j] = m.velocities[3 * id_ + j]; }
    double speed = 0;
    for (double value : v) speed += value * value;
    speed = std::sqrt(speed);
    std::array<double, 3> current_goal{};
    for (int j = 0; j < 3; ++j)
      current_goal[j] = m.slot_goals[3 * assignment_[id_] + j];
    const bool parked = distance(p, current_goal) <= lock_distance_;
    for (int slot = 0; slot < kAgents; ++slot) {
      // Arrived aircraft reserve their existing slot, not the entire auction.
      // Other aircraft may still correct a lateral inversion in free slots.
      if (parked && slot != assignment_[id_]) { (*row)[slot] = -1; continue; }
      std::array<double, 3> goal{};
      for (int j = 0; j < 3; ++j) goal[j] = m.slot_goals[3 * slot + j];
      if (!certify_(goal)) { (*row)[slot] = -1; continue; }
      const double length = distance(p, goal);
      double projection = 0;
      if (speed > 0.05)
        for (int j = 0; j < 3; ++j) projection += (goal[j] - p[j]) * v[j] / speed;
      (*row)[slot] = length + (speed > 0.05 ? heading_weight_ * std::max(0.0, length - projection) : 0.0);
    }
    // A blocked current target cannot supply a finite baseline for claiming an
    // improvement. Leave obstacle-routing/recovery to the trajectory planner.
    return (*row)[assignment_[id_]] >= 0;
  }
  bool candidateLocallyValid(const Message& m, const View& members, const ros::Time& now) const {
    if (!liveSnapshot(m, members, now)) return false;
    if (m.drone_to_slot[id_] != static_cast<unsigned>(assignment_[id_]) &&
        distance(members[id_].position, members[id_].goal) <= lock_distance_)
      return false;
    AssignmentCosts costs{};
    for (int i = 0; i < kAgents; ++i)
      for (int j = 0; j < kAgents; ++j)
        costs[i][j] = m.costs[kAgents * i + j] < 0
                          ? std::numeric_limits<double>::infinity() : m.costs[kAgents * i + j];
    const auto solution = solveAssignment(costs, assignment_, switch_penalty_);
    if (!assignmentGainSufficient(solution, min_gain_, relative_gain_) ||
        solution.assignment != mapping(m.drone_to_slot) ||
        std::abs(solution.net_gain - m.gain) > 1e-6) return false;
    std::array<double, 3> own_target{};
    for (int j = 0; j < 3; ++j) own_target[j] = m.slot_goals[3 * m.drone_to_slot[id_] + j];
    if (!certify_(own_target)) return false;
    // ACK only if this is exactly this agent's own immutable cost row, not a
    // row invented or altered by a coordinator.
    if (!last_bid_ || !requestSame(*last_bid_, m)) return false;
    for (int j = 0; j < kAgents; ++j)
      if (last_bid_->costs[id_ * kAgents + j] != m.costs[id_ * kAgents + j]) return false;
    return true;
  }
  void reserve(const Message& m) {
    if (isAborted(m) || isCommitted(m)) return;
    if (locked_) {
      if (!same(*locked_, m)) throw std::runtime_error("conflicting assignment reservation");
      return;
    }
    nh_.setParam(journal_ + "/prepared", encode(m));
    locked_.reset(new Message(m));
    acks_ = {};
  }
  void commit(Message m) {
    if (isAborted(m) || !matchesCurrent(m) || (locked_ && !same(*locked_, m))) return;
    // Persist the certificate before changing targets or sending a decision.
    m.header.stamp = ros::Time::now();
    m.phase = Message::COMMIT;
    history_.push_back(m);
    saveChain("/commits", history_);
    nh_.setParam(journal_ + "/committed", encode(m));
    nh_.deleteParam(journal_ + "/prepared");
    const Assignment next = mapping(m.drone_to_slot);
    apply_(m.assignment_epoch, next);
    gate_.committed(ros::Time::now().toSec());
    locked_.reset(); collecting_.reset(); last_bid_.reset();
    ROS_INFO("[ASSIGNMENT_COMMITTED] drone=%d epoch=%llu gain=%.3f", id_,
             static_cast<unsigned long long>(epoch_), m.gain);
  }
  void abort(Message m, bool originating) {
    if (isCommitted(m) || (locked_ && sameToken(*locked_, m) && !same(*locked_, m))) return;
    if (originating) {
      if (m.proposer_id != static_cast<unsigned>(id_)) return;
      m.header.stamp = ros::Time::now();
      m.sender_id = id_; m.sender_session = session_;
      m.sequence = ++sequence_; m.phase = Message::ABORT;
    }
    if (!isAborted(m)) {
      aborted_.push_back(m);
      saveChain("/aborts", aborted_); // Decision durable before unlock or send.
    }
    if (locked_ && same(*locked_, m)) {
      nh_.deleteParam(journal_ + "/prepared");
      locked_.reset(); collecting_.reset(); last_bid_.reset();
      gate_.resetCandidate();
    }
    if (originating) {
      pub_.publish(m);
      ROS_WARN("[ASSIGNMENT_ABORTED] drone=%d epoch=%llu transaction=%llu; retain targets and retry",
               id_, static_cast<unsigned long long>(m.base_epoch),
               static_cast<unsigned long long>(m.transaction_id));
    }
  }
  Message publish(Message m, int phase) {
    m.header.stamp = ros::Time::now(); m.header.frame_id = "world";
    m.sender_id = id_; m.sender_session = session_; m.sequence = ++sequence_;
    m.phase = phase;
    pub_.publish(m);
    return m;
  }
  void replay(const View& members, const ros::Time& now) {
    if (!due(last_replay_, now, 0.5)) return;
    bool behind = false;
    uint64_t oldest = epoch_;
    for (const auto& m : members) {
      if (m.lease.fresh(now.toSec(), ttl_)) oldest = std::min(oldest, m.epoch);
      behind = behind || !m.applied || m.epoch != epoch_ || m.assignment != assignment_;
    }
    if (behind) {
      for (const auto& decision : history_)
        if (decision.base_epoch >= oldest) publish(decision, Message::COMMIT);
    }
    // Only the coordinator itself relays its durable ABORT records. Other
    // members retain them for replay rejection, but cannot decide an abort.
    for (const auto& decision : aborted_)
      if (decision.proposer_id == static_cast<unsigned>(id_) &&
          decision.base_epoch >= oldest) publish(decision, Message::ABORT);
    last_replay_ = now;
  }
  void receive(const Message::ConstPtr& incoming) {
    const auto& m = *incoming;
    const auto now = ros::Time::now();
    if (m.sender_id >= kAgents || m.phase < Message::REQUEST || m.phase > Message::ABORT ||
        m.header.stamp.isZero() || (now - m.header.stamp).toSec() > ttl_ ||
        (m.header.stamp - now).toSec() > 0.05 || !validProposal(m)) return;
    if (!writers_[m.sender_id].accept(m.sender_session, m.sequence, m.header.stamp.toSec(),
                                     now.toSec(), ttl_, true)) return;
    if (m.phase == Message::ABORT) {
      // A recovered coordinator may have a newer sender session than the
      // immutable proposer session. Unique live physical IDs are required.
      if (m.sender_id == m.proposer_id) abort(m, false);
      return;
    }
    if (isAborted(m)) return;
    if (m.phase == Message::COMMIT) {
      if (validCertificate(m)) commit(m);
      return;
    }
    if (!matchesCurrent(m) || m.sender_session != m.member_sessions[m.sender_id]) return;
    const View members = view_();
    if (!members[m.sender_id].lease.fresh(now.toSec(), ttl_) ||
        members[m.sender_id].lease.session != m.sender_session) return;
    if (m.phase == Message::REQUEST) {
      if (locked_ || m.sender_id != m.proposer_id || m.sender_session != m.proposer_session ||
          !liveSnapshot(m, members, now)) return;
      Message bid = m;
      std::array<double, kAgents> row{};
      if (!ownCosts(m, &row)) return;
      for (int slot = 0; slot < kAgents; ++slot) bid.costs[id_ * kAgents + slot] = row[slot];
      last_bid_.reset(new Message(bid));
      publish(bid, Message::BID);
    } else if (m.phase == Message::BID) {
      if (!collecting_ || id_ != static_cast<int>(m.proposer_id) ||
          !requestSame(*collecting_, m)) return;
      for (int slot = 0; slot < kAgents; ++slot)
        collecting_->costs[m.sender_id * kAgents + slot] = m.costs[m.sender_id * kAgents + slot];
      bids_[m.sender_id].accept(m.sender_session, m.sequence, m.header.stamp.toSec(), now.toSec(), ttl_, true);
    } else if (m.phase == Message::PREPARE) {
      if (m.sender_id != m.proposer_id || m.sender_session != m.proposer_session ||
          (locked_ && !same(*locked_, m)) || !candidateLocallyValid(m, members, now)) return;
      reserve(m);
      const Message ack = publish(m, Message::ACK);
      // Own vote is also recorded locally, independent of ROS loopback order.
      acks_[id_].accept(session_, ack.sequence, ack.header.stamp.toSec(), now.toSec(), ttl_, true);
    } else if (m.phase == Message::ACK) {
      if (!locked_ || !same(*locked_, m)) return;
      acks_[m.sender_id].accept(m.sender_session, m.sequence, m.header.stamp.toSec(), now.toSec(), ttl_, true);
    }
  }
  static std::string encode(const Message& message) {
    std::vector<uint8_t> data(ros::serialization::serializationLength(message));
    ros::serialization::OStream stream(data.data(), data.size());
    ros::serialization::serialize(stream, message);
    static const char* digits = "0123456789abcdef";
    std::string result;
    result.reserve(2 * data.size());
    for (uint8_t byte : data) { result.push_back(digits[byte >> 4]); result.push_back(digits[byte & 15]); }
    return result;
  }
  static bool decode(const std::string& text, Message* message) {
    if (text.empty() || text.size() % 2 || text.size() > 20000) return false;
    std::vector<uint8_t> data;
    const auto digit = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    for (size_t i = 0; i < text.size(); i += 2) {
      const int a = digit(text[i]), b = digit(text[i + 1]);
      if (a < 0 || b < 0) return false;
      data.push_back(static_cast<uint8_t>(16 * a + b));
    }
    try {
      ros::serialization::IStream stream(data.data(), data.size());
      ros::serialization::deserialize(stream, *message);
      return stream.getLength() == 0;
    } catch (const std::exception&) { return false; }
  }

  ros::NodeHandle nh_;
  int id_;
  uint64_t session_, &epoch_;
  Assignment& assignment_;
  ReadView view_; Certify certify_; Apply apply_;
  ros::Publisher pub_; ros::Subscriber sub_;
  double min_gain_, cooldown_, period_, hold_, relative_gain_, switch_penalty_, heading_weight_;
  double goal_window_, lock_distance_, ttl_, transaction_timeout_;
  uint64_t sequence_{0}, transactions_{0};
  std::string journal_;
  std::unique_ptr<Message> collecting_, locked_, last_bid_;
  std::array<Lease, kAgents> writers_{}, bids_{}, acks_{};
  std::vector<Message> history_;
  std::vector<Message> aborted_;
  AssignmentGate gate_;
  ros::Time last_request_, last_send_, last_replay_;
};
}  // namespace distributed
}  // namespace ego_planner

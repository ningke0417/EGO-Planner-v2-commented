#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ego_planner/FormationAgentState.h>
#include <ego_planner/FormationEdgeEvidence.h>
#include <ego_planner/FormationTopologyCommand.h>
#include <ego_planner/FormationTopologyAck.h>
#include <ego_planner/FormationSlotSwap.h>
#include <plan_manage/distributed_edges.h>
#include <starconvex_adapter/original_star_space.hpp>

namespace {
namespace de = ego_planner::distributed;
using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::OriginalStarSpace;
using starconvex_adapter::subtract;
using starconvex_adapter::norm;
class Agent {
 public:
  Agent() : nh_("~"), session_(ros::WallTime::now().toNSec()) {
    nh_.param("drone_id", id_, -1);
    nh_.param("evidence_timeout", ttl_, 1.5);
    nh_.param("map_timeout", map_timeout_, 1.0);
    nh_.param("reconnect_hold", reconnect_hold_, 0.8);
    nh_.param("weak_reconnect_hold", weak_reconnect_hold_, 0.3);
    nh_.param("weak_to_normal_hold", weak_to_normal_hold_, 0.5);
    nh_.param("weak_approach_hold", weak_approach_hold_, 0.6);
    nh_.param("weak_min_separation", weak_min_separation_, 0.2);
    nh_.param("weak_min_reversal", weak_min_reversal_, 0.25);
    nh_.param("cut_hold", cut_hold_, 1.0);
    nh_.param("formation_scale", formation_scale_, 0.15);
    nh_.param("split_distance", split_distance_, 1.8);
    nh_.param("merge_distance", merge_distance_, 1.7);
    nh_.param("weak_reconnect_distance", weak_reconnect_distance_, 3.0);
    nh_.param("weak_formation_scale", weak_formation_scale_, 0.22);
    nh_.param("split_score", split_score_, 0.34);
    nh_.param("swap_enabled", swap_enabled_, false);
    nh_.param("/distributed_swap_enabled", swap_enabled_, swap_enabled_);
    nh_.param("swap_gain_min", swap_gain_min_, 0.15);
    nh_.param("swap_eval_period", swap_eval_period_, 0.5);
    nh_.param("swap_cooldown", swap_cooldown_, 8.0);
    nh_.param("formation_forward_x", forward_x_, 1.0);
    nh_.param("formation_forward_y", forward_y_, 0.0);
    const double forward_norm = std::hypot(forward_x_, forward_y_);
    if (id_ < 0 || id_ >= de::kAgents || ttl_ <= 0 ||
        map_timeout_ <= 0 || reconnect_hold_ < 0 ||
        weak_reconnect_hold_ < 0 || weak_to_normal_hold_ < 0 ||
        cut_hold_ < 0 ||
        weak_approach_hold_ < 0 || weak_min_separation_ < 0 ||
        weak_min_reversal_ <= 0 ||
        formation_scale_ < 0 ||
        merge_distance_ <= 0 || split_distance_ < merge_distance_ ||
        weak_reconnect_distance_ < merge_distance_ ||
        weak_formation_scale_ < formation_scale_ ||
        split_score_ < 0 || split_score_ > 1 ||
        swap_gain_min_ < 0 || swap_eval_period_ <= 0 || swap_cooldown_ < 0 ||
        !std::isfinite(forward_norm) || forward_norm < 1e-6)
      throw std::runtime_error("invalid distributed agent parameters");
    // The planner is the source of truth for occupancy geometry. The separate
    // observed-free grid certifies visibility; its coarser cells are not
    // upsampled into invented fine-resolution observations.
    const std::string planner = "/drone_"+std::to_string(id_)+"_ego_planner_node/grid_map/";
    double planner_resolution, map_x, map_y, map_z, ground;
    if (!nh_.getParam(planner+"resolution", planner_resolution) ||
        !nh_.getParam(planner+"map_size_x", map_x) ||
        !nh_.getParam(planner+"map_size_y", map_y) ||
        !nh_.getParam(planner+"map_size_z", map_z) ||
        !nh_.getParam(planner+"ground_height", ground) ||
        !nh_.getParam("/drone_"+std::to_string(id_)+"_observed_map/resolution", resolution_) ||
        !std::isfinite(map_x) || !std::isfinite(map_y) || !std::isfinite(map_z) ||
        map_x <= 0 || map_y <= 0 || map_z <= 0 ||
        !std::isfinite(resolution_) || resolution_ <= 0)
      throw std::runtime_error("distributed agent requires local planner/observed grid parameters");
    planner_layout_ = starconvex_adapter::VoxelGridLayout(
        planner_resolution, {-map_x/2.0,-map_y/2.0,ground}, true);
    planner_size_ = {map_x,map_y,map_z};
    nh_.param<std::string>(planner+"frame_id", map_frame_, "world");
    forward_x_ /= forward_norm;
    forward_y_ /= forward_norm;
    assignment_ = de::identityAssignment();
    geometry_.delta_floor = -0.5;
    geometry_.delta_roof = 0.5;
    geometry_.frontier_floor = 0.0;
    geometry_.frontier_roof = 3.0;
    geometry_.raycast_resolution = planner_layout_.resolution;
    state_pub_ = nh_.advertise<ego_planner::FormationAgentState>("/swarm/agent_state", 32);
    edge_pub_ = nh_.advertise<ego_planner::FormationEdgeEvidence>("/swarm/edge_evidence", 64);
    command_pub_ = nh_.advertise<ego_planner::FormationTopologyCommand>("topology", 1);
    swap_pub_ = nh_.advertise<ego_planner::FormationSlotSwap>("/swarm/slot_swap", 32);
    odom_sub_ = nh_.subscribe("odom", 1, &Agent::odom, this);
    state_sub_ = nh_.subscribe("/swarm/agent_state", 64, &Agent::state, this);
    evidence_sub_ = nh_.subscribe("/swarm/edge_evidence", 128, &Agent::evidence, this);
    swap_sub_ = nh_.subscribe("/swarm/slot_swap", 128, &Agent::swapMessage, this);
    for (int id = 0; id < de::kAgents; ++id)
      applied_subs_[id] = nh_.subscribe<ego_planner::FormationTopologyAck>(
          "/drone_" + std::to_string(id) + "/formation_applied", 1,
          [this, id](const ego_planner::FormationTopologyAck::ConstPtr& msg) {
            if (msg->drone_id != static_cast<unsigned>(id) ||
                msg->header.stamp <= applied_stamps_[id] ||
                msg->header.stamp > ros::Time::now() + ros::Duration(0.05)) return;
            applied_stamps_[id] = msg->header.stamp;
            std::copy(msg->assignment_versions.begin(),
                      msg->assignment_versions.end(),
                      applied_versions_[id].begin());
          });
    occupied_sub_ = nh_.subscribe("occupied", 1, &Agent::occupied, this);
    free_sub_ = nh_.subscribe("free", 1, &Agent::freeCloud, this);
    inflated_sub_ = nh_.subscribe("inflated", 1, &Agent::inflatedCloud, this);
    ROS_INFO_STREAM("[DISTRIBUTED_GEOMETRY] drone=" << id_
                    << " occupancy=local_planner_snapshot resolution=" << planner_resolution
                    << " observed_free_resolution=" << resolution_
                    << " extra_corridor_radius=0");
    timer_ = nh_.createTimer(ros::Duration(0.1), &Agent::tick, this);
  }
 private:
  void odom(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    ego_planner::FormationAgentState value;
    value.header = msg->header;
    // Never turn an old sensor sample into a fresh peer heartbeat.
    value.session = session_;
    value.sequence = ++state_sequence_;
    value.drone_id = id_;
    value.position = {{p.x, p.y, p.z}};
    std::copy(versions_.begin(), versions_.end(),
              value.assignment_versions.begin());
    value.slot_id = assignment_[id_];
    value.startup_ready = local_startup_ready_;
    state(ego_planner::FormationAgentState::ConstPtr(new ego_planner::FormationAgentState(value)));
    state_pub_.publish(value);
  }
  void state(const ego_planner::FormationAgentState::ConstPtr& msg) {
    if (msg->drone_id >= de::kAgents) return;
    for (double value : msg->position) if (!std::isfinite(value)) return;
    const double now = ros::Time::now().toSec();
    auto& lease = state_leases_[msg->drone_id];
    if (lease.accept(msg->session, msg->sequence, msg->header.stamp.toSec(),
                     now, ttl_, true)) {
      positions_[msg->drone_id] = {msg->position[0], msg->position[1], msg->position[2]};
      std::copy(msg->assignment_versions.begin(), msg->assignment_versions.end(),
                peer_versions_[msg->drone_id].begin());
    }
    startup_leases_[msg->drone_id].accept(
        msg->session, msg->sequence, msg->header.stamp.toSec(), now, ttl_,
        msg->startup_ready);
  }
  void evidence(const ego_planner::FormationEdgeEvidence::ConstPtr& msg) {
    if (!de::edgeVersionsMatch(msg->edge_index, versions_,
                               msg->assignment_versions) ||
        msg->decision > ego_planner::FormationEdgeEvidence::KEEP ||
        (msg->weak_recovery && msg->decision !=
             ego_planner::FormationEdgeEvidence::KEEP) ||
        !std::isfinite(msg->score) || !std::isfinite(msg->visibility) ||
        !std::isfinite(msg->distance)) return;
    table_.updateAt(msg->edge_index, msg->observer_id, msg->peer_id,
                    assignment_, msg->session, msg->sequence,
                    msg->header.stamp.toSec(), ros::Time::now().toSec(), ttl_,
                    msg->decision == ego_planner::FormationEdgeEvidence::KEEP,
                    msg->weak_recovery);
  }
  void occupied(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != map_frame_ || msg->header.stamp <= occupied_stamp_ ||
        msg->header.stamp > ros::Time::now()+ros::Duration(0.05)) return;
    occupied_stamp_ = msg->header.stamp;
    rebuild();
  }
  void freeCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != map_frame_ || msg->header.stamp <= free_stamp_ ||
        msg->header.stamp > ros::Time::now()+ros::Duration(0.05)) return;
    pcl::fromROSMsg(*msg, free_);
    free_stamp_ = msg->header.stamp;
    rebuild();
  }
  void inflatedCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != map_frame_ || msg->header.stamp <= inflated_stamp_ ||
        msg->header.stamp > ros::Time::now()+ros::Duration(0.05)) return;
    pcl::fromROSMsg(*msg, inflated_);
    inflated_stamp_ = msg->header.stamp;
    rebuild();
  }
  void rebuild() {
    // Both layers must refer to the same local sensor update.
    if (occupied_stamp_.isZero() || occupied_stamp_ != free_stamp_ || inflated_stamp_.isZero()) return;
    field_.reset(new VoxelObstacleField(inflated_, planner_layout_, free_,
                 starconvex_adapter::VoxelGridLayout(resolution_), planner_size_));
    field_stamp_ = std::min(occupied_stamp_,inflated_stamp_);
  }
  bool corridor(const Vec3& a, const Vec3& b) const {
    // The planner's occupied cells already include the vehicle safety margin.
    // Traverse the centerline's cells without adding another 0.1 m band.
    return field_->knownFreeSegment(a,b);
  }
  double overlap(const Vec3& a, const Vec3& b) const {
    OriginalStarSpace first(a, starconvex_adapter::generateBoundaryCloud(a, *field_, geometry_));
    OriginalStarSpace second(b, starconvex_adapter::generateBoundaryCloud(b, *field_, geometry_));
    unsigned va = 0, vb = 0, intersection = 0;
    const double z0 = std::max(geometry_.frontier_floor, std::min(a.z,b.z)-0.5);
    const double z1 = std::min(geometry_.frontier_roof, std::max(a.z,b.z)+0.5);
    for (double x = std::min(a.x,b.x)-5.0+0.125; x < std::max(a.x,b.x)+5.0; x += 0.25)
      for (double y = std::min(a.y,b.y)-5.0+0.125; y < std::max(a.y,b.y)+5.0; y += 0.25)
        for (double z = z0+0.05; z < z1; z += 0.1) {
          const Vec3 p{x,y,z};
          if (!field_->knownFree(p)) continue;
          const bool ia = first.contains(p), ib = second.contains(p);
          va += ia; vb += ib; intersection += ia && ib;
        }
    return std::min(va,vb) ? static_cast<double>(intersection)/std::min(va,vb) : 0;
  }
  struct PendingSwap {
    uint64_t transaction = 0, proposer_session = 0;
    int front = -1, rear = -1;
    uint32_t base_version = 0;
    double gain = 0.0;
    ros::Time started, last_sent;
    std::array<de::Lease, de::kAgents> acks{};
  };
  bool swapReady(int column, int front, int rear, double* gain) const {
    const ros::Time now = ros::Time::now();
    if (!startup_ready_ || !field_ || now < field_stamp_ ||
        (now-field_stamp_).toSec() > map_timeout_ ||
        column < 0 || column >= de::kColumns ||
        de::owner(assignment_, column) != front ||
        de::owner(assignment_, column+de::kColumns) != rear ||
        !state_leases_[front].fresh(now.toSec(), ttl_) ||
        !state_leases_[rear].fresh(now.toSec(), ttl_)) return false;
    const double value = de::columnSwapGain(
        positions_[front].x, positions_[front].y,
        positions_[rear].x, positions_[rear].y, forward_x_, forward_y_);
    if (gain) *gain = value;
    if (value < swap_gain_min_) return false;
    de::Assignment candidate = assignment_;
    de::swapColumn(&candidate, column);
    // Every endpoint certifies the new logical edges incident to itself.
    // Unaffected endpoints do not need to repeat this check.
    for (const auto& edge : de::edges()) {
      if (edge[0] != column && edge[1] != column &&
          edge[0] != column+de::kColumns &&
          edge[1] != column+de::kColumns) continue;
      const int a = de::owner(candidate, edge[0]);
      const int b = de::owner(candidate, edge[1]);
      if (id_ != a && id_ != b) continue;
      const int peer = id_ == a ? b : a;
      if (!state_leases_[peer].fresh(now.toSec(), ttl_) ||
          norm(subtract(positions_[id_], positions_[peer])) > weak_reconnect_distance_ ||
          !corridor(positions_[id_], positions_[peer])) return false;
    }
    return true;
  }
  void publishSwap(int column, int phase, const PendingSwap& tx) {
    ego_planner::FormationSlotSwap message;
    message.header.stamp = ros::Time::now();
    message.sender_session = session_;
    message.sequence = ++swap_sequence_;
    message.sender_id = id_;
    message.column = column;
    message.front_drone = tx.front;
    message.rear_drone = tx.rear;
    message.base_version = tx.base_version;
    message.proposer_session = tx.proposer_session;
    message.transaction_id = tx.transaction;
    message.phase = phase;
    message.gain = tx.gain;
    swap_pub_.publish(message);
  }
  void applySwap(int column, const PendingSwap& tx) {
    if (versions_[column] != tx.base_version) return;
    de::swapColumn(&assignment_, column);
    ++versions_[column];
    table_.clearColumn(column);
    for (int edge = 0; edge < de::kEdges; ++edge)
      if (de::edgeTouchesColumn(edge, column)) {
        kept_[edge] = false;
        weak_reconnected_[edge] = false;
        visibility_reconnected_[edge] = false;
        candidate_weak_[edge] = false;
        approach_[edge].reset();
        ready_since_[edge] = ros::Time();
        cut_since_[edge] = ros::Time();
        normal_since_[edge] = ros::Time();
      }
    candidate_since_[column] = ros::Time();
    swap_armed_ = false;
    full_since_swap_ = false;
    last_swap_ = ros::Time::now();
    // Retain the ordered commit chain so a later agent restart can replay
    // every missing version, even after everybody once acknowledged it.
    history_[column].push_back(tx);
    ROS_WARN("[SLOT_SWAP] drone=%d column=%d version=%u front=%d rear=%d gain=%.3f",
             id_, column, versions_[column], tx.front, tx.rear, tx.gain);
  }
  void swapMessage(const ego_planner::FormationSlotSwap::ConstPtr& msg) {
    if (!swap_enabled_) return;
    const ros::Time now = ros::Time::now();
    if (msg->sender_id >= de::kAgents || msg->column >= de::kColumns ||
        msg->front_drone >= de::kAgents || msg->rear_drone >= de::kAgents ||
        msg->transaction_id == 0 || msg->proposer_session == 0 ||
        !std::isfinite(msg->gain) ||
        (msg->phase != msg->COMMIT && msg->gain < swap_gain_min_) ||
        msg->header.stamp.isZero() || (now-msg->header.stamp).toSec() > ttl_ ||
        (msg->header.stamp-now).toSec() > 0.05 ||
        msg->phase < msg->PREPARE || msg->phase > msg->COMMIT) return;
    auto& writer = swap_writers_[msg->sender_id];
    if (!writer.accept(msg->sender_session, msg->sequence,
                       msg->header.stamp.toSec(), now.toSec(), ttl_, true)) return;
    const int column = msg->column;
    auto& pending = pending_[column];
    if (msg->phase == msg->COMMIT) {
      // A committed transaction is gossiped by every recipient. This also
      // repairs a missed direct commit when its original proposer disappears.
      if (versions_[column] != msg->base_version ||
          de::owner(assignment_, column) != static_cast<int>(msg->front_drone) ||
          de::owner(assignment_, column+de::kColumns) != static_cast<int>(msg->rear_drone))
        return;
      PendingSwap tx;
      tx.transaction = msg->transaction_id;
      tx.proposer_session = msg->proposer_session;
      tx.front = msg->front_drone;
      tx.rear = msg->rear_drone;
      tx.base_version = msg->base_version;
      tx.gain = msg->gain;
      applySwap(column, tx);
      pending = PendingSwap();
      return;
    }
    if (msg->base_version != versions_[column] || !startup_ready_ ||
        de::owner(assignment_, column) != static_cast<int>(msg->front_drone) ||
        de::owner(assignment_, column+de::kColumns) != static_cast<int>(msg->rear_drone) ||
        (static_cast<int>(msg->sender_id) !=
            std::min(static_cast<int>(msg->front_drone),
                     static_cast<int>(msg->rear_drone)) &&
         msg->phase == msg->PREPARE)) return;
    if (msg->phase == msg->PREPARE) {
      double local_gain = 0;
      if (!swapReady(column, msg->front_drone, msg->rear_drone,
                     &local_gain)) return;
      if (pending.transaction != 0 &&
          (pending.transaction != msg->transaction_id ||
           pending.proposer_session != msg->proposer_session)) return;
      pending.transaction = msg->transaction_id;
      pending.proposer_session = msg->proposer_session;
      pending.front = msg->front_drone;
      pending.rear = msg->rear_drone;
      pending.base_version = msg->base_version;
      pending.gain = msg->gain;
      if (pending.started.isZero()) pending.started = now;
      publishSwap(column, ego_planner::FormationSlotSwap::ACK, pending);
    } else if (msg->phase == msg->ACK &&
               pending.transaction == msg->transaction_id &&
               pending.proposer_session == msg->proposer_session &&
               pending.front == static_cast<int>(msg->front_drone) &&
               pending.rear == static_cast<int>(msg->rear_drone)) {
      pending.acks[msg->sender_id].accept(msg->sender_session, msg->sequence,
          msg->header.stamp.toSec(), now.toSec(), ttl_, true);
    }
  }
  void evaluateSwaps(const ros::Time& now) {
    if (!swap_enabled_ || !startup_ready_) return;
    for (int column = 0; column < de::kColumns; ++column) {
      auto& pending = pending_[column];
      if (pending.transaction != 0 &&
          (now-pending.started).toSec() > 2.5) pending = PendingSwap();
      if (!history_[column].empty()) {
        bool all_applied = true;
        uint32_t replay_base = versions_[column] - 1;
        for (int id = 0; id < de::kAgents; ++id) {
          const bool state_fresh = state_leases_[id].fresh(now.toSec(), ttl_);
          if (state_fresh)
            replay_base = std::min(replay_base, peer_versions_[id][column]);
          all_applied = all_applied && state_fresh &&
              peer_versions_[id][column] >= versions_[column] &&
              !applied_stamps_[id].isZero() &&
              now >= applied_stamps_[id] &&
              (now-applied_stamps_[id]).toSec() <= ttl_ &&
              applied_versions_[id][column] >= versions_[column];
        }
        if (!all_applied && replay_base < history_[column].size()) {
          auto& replay = history_[column][replay_base];
          if (replay.last_sent.isZero() ||
              (now-replay.last_sent).toSec() >= 0.5) {
            publishSwap(column, ego_planner::FormationSlotSwap::COMMIT, replay);
            replay.last_sent = now;
          }
        }
      }
      if (pending.transaction != 0) {
        const int leader = std::min(pending.front, pending.rear);
        if (id_ == leader &&
            (pending.last_sent.isZero() ||
             (now-pending.last_sent).toSec() >= 0.25)) {
          bool all_acked = true;
          for (const auto& ack : pending.acks)
            all_acked = all_acked && ack.keep && ack.fresh(now.toSec(), ttl_);
          if (all_acked) {
            publishSwap(column, ego_planner::FormationSlotSwap::COMMIT, pending);
            applySwap(column, pending);
            pending = PendingSwap();
          } else {
            publishSwap(column, ego_planner::FormationSlotSwap::PREPARE, pending);
            pending.last_sent = now;
          }
        }
        continue;
      }
      if (!swap_armed_ || (!last_swap_.isZero() &&
          (now-last_swap_).toSec() < swap_cooldown_)) continue;
      const int front = de::owner(assignment_, column);
      const int rear = de::owner(assignment_, column+de::kColumns);
      if (id_ != std::min(front, rear)) continue;
      double gain = 0;
      if (!swapReady(column, front, rear, &gain)) {
        candidate_since_[column] = ros::Time();
        continue;
      }
      if (candidate_since_[column].isZero()) candidate_since_[column] = now;
      if ((now-candidate_since_[column]).toSec() < swap_eval_period_) continue;
      pending.transaction = ++transaction_sequence_;
      pending.transaction ^= session_;
      if (pending.transaction == 0) ++pending.transaction;
      pending.proposer_session = session_;
      pending.front = front;
      pending.rear = rear;
      pending.base_version = versions_[column];
      pending.gain = gain;
      pending.started = now;
      publishSwap(column, ego_planner::FormationSlotSwap::PREPARE, pending);
      pending.last_sent = now;
    }
  }
  void evaluate(const ros::Time& now) {
    const bool local_ready = field_ && now >= field_stamp_ &&
        (now-field_stamp_).toSec() <= map_timeout_ &&
        state_leases_[id_].fresh(now.toSec(), ttl_);
    for (int edge = 0; edge < de::kEdges; ++edge) {
      const auto& pair = de::edges()[edge];
      const int first = de::owner(assignment_, pair[0]);
      const int second = de::owner(assignment_, pair[1]);
      if (first != id_ && second != id_) continue;
      const int peer = first == id_ ? second : first;
      double score = 0, visibility = 0, distance = 0;
      bool approaching = false;
      bool safe = local_ready && state_leases_[peer].fresh(now.toSec(), ttl_);
      if (safe) {
        distance = norm(subtract(positions_[id_], positions_[peer]));
        if (!kept_[edge])
          approaching = approach_[edge].update(
              distance, now.toSec(), split_distance_, weak_min_separation_,
              weak_min_reversal_, weak_approach_hold_);
        const double limit = !kept_[edge] || weak_reconnected_[edge]
                                 ? weak_reconnect_distance_
                                 : split_distance_;
        safe = distance <= limit;
      }
      if (safe) {
        try {
          visibility = corridor(positions_[id_], positions_[peer]) ? 1.0 : 0.0;
          if (visibility > 0) score = overlap(positions_[id_], positions_[peer]);
          safe = de::localEdgeEvidenceSafe(
              kept_[edge], weak_reconnected_[edge] ||
                               visibility_reconnected_[edge],
              distance, visibility, score,
              weak_reconnected_[edge] ? weak_reconnect_distance_
                                      : split_distance_,
              weak_reconnect_distance_, split_score_);
        } catch (const std::exception& error) {
          ROS_WARN_THROTTLE(2.0, "Local edge geometry unavailable: %s", error.what());
          safe = false;
        }
      }
      if (!safe) {
        if (kept_[edge]) {
          cut_since_[edge] = now;
          approach_[edge].cut(distance);
        } else approach_[edge].interrupt();
        kept_[edge] = false;
        weak_reconnected_[edge] = false;
        visibility_reconnected_[edge] = false;
        candidate_weak_[edge] = false;
        ready_since_[edge] = ros::Time();
        normal_since_[edge] = ros::Time();
      } else if (!kept_[edge]) {
        const bool weak_candidate = distance > merge_distance_;
        if (candidate_weak_[edge] != weak_candidate) {
          ready_since_[edge] = ros::Time();
          candidate_weak_[edge] = weak_candidate;
        }
        if (weak_candidate && !approaching) {
          ready_since_[edge] = ros::Time();
        } else {
          if (ready_since_[edge].isZero()) ready_since_[edge] = now;
          const double required_hold = weak_candidate
                                           ? weak_reconnect_hold_
                                           : reconnect_hold_;
          if ((now-ready_since_[edge]).toSec() >= required_hold &&
              (cut_since_[edge].isZero() ||
               (now-cut_since_[edge]).toSec() >= cut_hold_)) {
            kept_[edge] = true;
            weak_reconnected_[edge] = weak_candidate;
            visibility_reconnected_[edge] = !weak_candidate;
            normal_since_[edge] = ros::Time();
          }
        }
      } else if (weak_reconnected_[edge] && distance <= merge_distance_) {
        if (normal_since_[edge].isZero()) normal_since_[edge] = now;
        if ((now-normal_since_[edge]).toSec() >= weak_to_normal_hold_) {
          weak_reconnected_[edge] = false;
          visibility_reconnected_[edge] = true;
          approach_[edge].reset();
        }
      } else if (weak_reconnected_[edge]) {
        normal_since_[edge] = ros::Time();
      } else if (visibility_reconnected_[edge] && score >= split_score_) {
        visibility_reconnected_[edge] = false;
      }
      ego_planner::FormationEdgeEvidence value;
      value.header.stamp = now;
      value.session = session_;
      value.sequence = ++edge_sequence_;
      std::copy(versions_.begin(), versions_.end(),
                value.assignment_versions.begin());
      value.assignment_generation = de::assignmentSignature(versions_);
      value.edge_index = edge;
      value.observer_id = id_;
      value.peer_id = peer;
      value.decision = kept_[edge] ? value.KEEP : value.CUT;
      value.weak_recovery = kept_[edge] && weak_reconnected_[edge];
      value.score = score; value.visibility = visibility; value.distance = distance;
      table_.updateAt(edge, id_, peer, assignment_, session_, value.sequence,
                      now.toSec(), now.toSec(), ttl_, kept_[edge],
                      value.weak_recovery);
      edge_pub_.publish(value);
    }
  }
  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (last_evaluation_.isZero() || (now-last_evaluation_).toSec() >= 0.30) {
      evaluate(now);
      last_evaluation_ = now;
    }
    const uint32_t mask_before_swap = table_.mask(now.toSec(), ttl_);
    constexpr uint32_t full_mask =
        (uint32_t{1} << de::kEdges) - uint32_t{1};
    if (!local_startup_ready_ && mask_before_swap == full_mask) {
      local_startup_ready_ = true;
      ROS_INFO("Distributed startup: local full-edge evidence is ready");
    }
    if (!startup_ready_ && local_startup_ready_) {
      bool all_ready = true;
      for (const auto& lease : startup_leases_)
        all_ready = all_ready && lease.keep && lease.fresh(now.toSec(), ttl_);
      if (all_ready) {
        startup_ready_ = true;
        ROS_INFO("Distributed startup barrier passed for all %d agents",
                 de::kAgents);
      }
    }
    if (startup_ready_) {
      if (mask_before_swap == full_mask) full_since_swap_ = true;
      else if (full_since_swap_) swap_armed_ = true;
      evaluateSwaps(now);
    }
    // applySwap() invalidates incident evidence. Publish the new assignment
    // only with a mask recomputed from that same post-swap edge table.
    const uint32_t mask = table_.mask(now.toSec(), ttl_);
    ego_planner::FormationTopologyCommand command;
    command.header.stamp = ros::Time::now();
    command.epoch = command.header.stamp.toNSec();
    // Always use the optimizer's relative-edge branch, including a fully connected graph.
    command.mode = command.SPLIT;
    command.edge_count = de::kEdges;
    command.edge_keep_mask = mask;
    command.weak_edge_mask = table_.weakMask(now.toSec(), ttl_);
    std::copy(versions_.begin(), versions_.end(),
              command.assignment_versions.begin());
    for (int id = 0; id < de::kAgents; ++id)
      command.drone_to_slot[id] = assignment_[id];
    command.components = de::components(command.edge_keep_mask, assignment_);
    const bool weak_recovery = command.weak_edge_mask != 0;
    command.formation_scale = weak_recovery ? weak_formation_scale_
                                            : formation_scale_;
    command.progress_sync_scale = 0.0;
    command.reason = startup_ready_ ? "DISTRIBUTED_ENDPOINT_LEASES"
                                    : "DISTRIBUTED_STARTUP_WAIT";
    command_pub_.publish(command);
  }
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, state_sub_, evidence_sub_, swap_sub_, occupied_sub_, free_sub_, inflated_sub_;
  std::array<ros::Subscriber, de::kAgents> applied_subs_{};
  ros::Publisher state_pub_, edge_pub_, command_pub_, swap_pub_;
  ros::Timer timer_;
  int id_;
  double ttl_, map_timeout_, reconnect_hold_, weak_reconnect_hold_;
  double weak_to_normal_hold_, cut_hold_;
  double weak_approach_hold_, weak_min_separation_, weak_min_reversal_;
  double resolution_;
  starconvex_adapter::VoxelGridLayout planner_layout_;
  Vec3 planner_size_;
  std::string map_frame_;
  double formation_scale_, split_distance_, merge_distance_, split_score_;
  double weak_reconnect_distance_, weak_formation_scale_;
  double swap_gain_min_, swap_eval_period_, swap_cooldown_, forward_x_, forward_y_;
  bool swap_enabled_{false}, swap_armed_{false}, full_since_swap_{false};
  uint64_t session_, state_sequence_{0}, edge_sequence_{0}, swap_sequence_{0},
      transaction_sequence_{0};
  de::Assignment assignment_{};
  std::array<uint32_t, de::kColumns> versions_{};
  std::array<std::array<uint32_t, de::kColumns>, de::kAgents> peer_versions_{};
  std::array<std::array<uint32_t, de::kColumns>, de::kAgents> applied_versions_{};
  std::array<ros::Time, de::kAgents> applied_stamps_{};
  std::array<de::Lease, de::kAgents> swap_writers_{};
  std::array<PendingSwap, de::kColumns> pending_{};
  std::array<std::vector<PendingSwap>, de::kColumns> history_{};
  std::array<ros::Time, de::kColumns> candidate_since_{};
  ros::Time last_swap_;
  std::array<Vec3, de::kAgents> positions_{};
  std::array<de::Lease, de::kAgents> state_leases_{};
  std::array<de::Lease, de::kAgents> startup_leases_{};
  de::EdgeTable table_;
  std::array<bool, de::kEdges> kept_{};
  std::array<bool, de::kEdges> weak_reconnected_{};
  std::array<bool, de::kEdges> visibility_reconnected_{};
  std::array<bool, de::kEdges> candidate_weak_{};
  std::array<de::WeakRecoveryApproach, de::kEdges> approach_{};
  std::array<ros::Time, de::kEdges> ready_since_{}, cut_since_{}, normal_since_{};
  pcl::PointCloud<pcl::PointXYZ> free_, inflated_;
  ros::Time occupied_stamp_, free_stamp_, inflated_stamp_, field_stamp_, last_evaluation_;
  std::unique_ptr<VoxelObstacleField> field_;
  starconvex_adapter::GeometryOptions geometry_;
  bool local_startup_ready_{false}, startup_ready_{false};
};
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "distributed_formation_agent");
  try { Agent agent; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}

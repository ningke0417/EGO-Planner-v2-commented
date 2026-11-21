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
#include <plan_manage/distributed_reassignment.h>
#include <starconvex_adapter/original_star_space.hpp>

namespace {
namespace de = ego_planner::distributed;
using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::OriginalStarSpace;
using starconvex_adapter::subtract;
using starconvex_adapter::norm;
// Only identical geometry may share a decoded cloud/voxel field. Sensor headers
// are deliberately excluded: a real new frame may contain the same geometry,
// but its source timestamp must still pass the normal monotonicity guards.
bool sameCloudGeometry(const sensor_msgs::PointCloud2& first,
                       const sensor_msgs::PointCloud2& second) {
  if (first.width != second.width || first.height != second.height ||
      first.is_bigendian != second.is_bigendian ||
      first.point_step != second.point_step || first.row_step != second.row_step ||
      first.is_dense != second.is_dense || first.fields.size() != second.fields.size())
    return false;
  for (size_t index = 0; index < first.fields.size(); ++index) {
    const auto& a = first.fields[index];
    const auto& b = second.fields[index];
    if (a.name != b.name || a.offset != b.offset ||
        a.datatype != b.datatype || a.count != b.count)
      return false;
  }
  return first.data == second.data;
}
class Agent {
 public:
  Agent() : nh_("~"), session_(ros::WallTime::now().toNSec()) {
    int rows, columns;
    double row_spacing, column_spacing;
    nh_.param("formation_rows", rows, 2);
    nh_.param("formation_columns", columns, 4);
    nh_.param("row_spacing", row_spacing, 1.0);
    nh_.param("column_spacing", column_spacing, 1.0);
    de::configureFormation(rows, columns, row_spacing, column_spacing);
    initializeStorage();
    nh_.param("drone_id", id_, -1);
    nh_.param("evidence_timeout", ttl_, 1.5);
    nh_.param("map_timeout", map_timeout_, 1.0);
    nh_.param("reconnect_hold", reconnect_hold_, 0.8);
    nh_.param("weak_reconnect_hold", weak_reconnect_hold_, 0.3);
    nh_.param("weak_to_normal_hold", weak_to_normal_hold_, 0.2);
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
    nh_.param<std::string>("swap_scope", swap_scope_, "all");
    if (swap_scope_ != "all" && swap_scope_ != "column")
      throw std::runtime_error("swap_scope must be all or column");
    if (swap_scope_ == "column" && de::rowCount() != 2)
      throw std::runtime_error("column swaps require a two-row formation; use swap_scope=all");
    assignment_epoch_ = swap_scope_ == "all" ? 1 : 0;
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
                msg->formation_digest != de::formationDigest() ||
                msg->edge_count != static_cast<unsigned>(de::edgeCount()) ||
                msg->assignment_versions.size() != static_cast<size_t>(de::columnCount()) ||
                msg->drone_to_slot.size() != static_cast<size_t>(de::agentCount()) ||
                msg->header.stamp <= applied_stamps_[id] ||
                msg->header.stamp > ros::Time::now() + ros::Duration(0.05)) return;
            applied_stamps_[id] = msg->header.stamp;
            std::copy(msg->assignment_versions.begin(),
                      msg->assignment_versions.end(),
                      applied_versions_[id].begin());
            applied_epochs_[id] = msg->assignment_epoch;
            applied_digests_[id] = msg->assignment_digest;
            for (int drone = 0; drone < de::kAgents; ++drone)
              applied_assignments_[id][drone] = msg->drone_to_slot[drone];
            if (id == id_) {
              own_goal_valid_ = msg->goal_valid;
              for (int j = 0; j < 3; ++j) {
                own_goal_[j] = msg->assigned_goal[j];
                own_goal_valid_ = own_goal_valid_ && std::isfinite(own_goal_[j]);
              }
            }
          });
    occupied_sub_ = nh_.subscribe("occupied", 1, &Agent::occupied, this);
    free_sub_ = nh_.subscribe("free", 1, &Agent::freeCloud, this);
    inflated_sub_ = nh_.subscribe("inflated", 1, &Agent::inflatedCloud, this);
    ROS_INFO_STREAM("[DISTRIBUTED_GEOMETRY] drone=" << id_
                    << " occupancy=local_planner_snapshot resolution=" << planner_resolution
                    << " observed_free_resolution=" << resolution_
                    << " extra_corridor_radius=0");
    if (swap_enabled_ && swap_scope_ == "all") {
      reassignment_.reset(new de::AllAgentAssignment(
          nh_, id_, session_, assignment_epoch_, assignment_,
          [this]() { return assignmentView(); },
          [this](const std::array<double, 3>& goal) {
            const auto now = ros::Time::now();
            return field_ && now >= field_stamp_ &&
                (now - field_stamp_).toSec() <= map_timeout_ &&
                state_leases_[id_].fresh(now.toSec(), ttl_) &&
                corridor(positions_[id_], Vec3{goal[0], goal[1], goal[2]});
          },
          [this](uint64_t epoch, const de::Assignment& assignment) {
            applyFullAssignment(epoch, assignment);
          }));
    }
    ROS_INFO_STREAM("[ASSIGNMENT_CONFIG] drone=" << id_ << " enabled=" << swap_enabled_
                    << " scope=" << swap_scope_ << " epoch=" << assignment_epoch_);
    timer_ = nh_.createTimer(ros::Duration(0.1), &Agent::tick, this);
  }
 private:
  void initializeStorage() {
    const size_t agents = de::agentCount(), columns = de::columnCount();
    const size_t edges = de::edgeCount();
    applied_subs_.resize(agents);
    assignment_ = de::identityAssignment();
    versions_.assign(columns, 0);
    peer_versions_.assign(agents, std::vector<uint32_t>(columns, 0));
    applied_versions_.assign(agents, std::vector<uint32_t>(columns, 0));
    applied_stamps_.resize(agents);
    peer_epochs_.assign(agents, 0); peer_digests_.assign(agents, 0);
    applied_epochs_.assign(agents, 0); applied_digests_.assign(agents, 0);
    peer_assignments_.assign(agents, de::identityAssignment());
    applied_assignments_.assign(agents, de::identityAssignment());
    velocities_.resize(agents); goals_.resize(agents);
    goal_valid_.assign(agents, false);
    swap_writers_.resize(agents);
    pending_.resize(columns); history_.resize(columns);
    candidate_since_.resize(columns);
    positions_.resize(agents);
    state_leases_.resize(agents); startup_leases_.resize(agents);
    table_.clear();
    kept_.assign(edges, false); weak_reconnected_.assign(edges, false);
    visibility_reconnected_.assign(edges, false); candidate_weak_.assign(edges, false);
    approach_.resize(edges);
    overlap_cache_.resize(edges);
    ready_since_.resize(edges); cut_since_.resize(edges); normal_since_.resize(edges);
  }
  void odom(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    ego_planner::FormationAgentState value;
    value.header = msg->header;
    value.formation_digest = de::formationDigest();
    // Never turn an old sensor sample into a fresh peer heartbeat.
    value.session = session_;
    value.sequence = ++state_sequence_;
    value.drone_id = id_;
    value.position = {{p.x, p.y, p.z}};
    const auto& v = msg->twist.twist.linear;
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return;
    value.velocity = {{v.x, v.y, v.z}};
    value.assignment_versions.assign(versions_.begin(), versions_.end());
    value.slot_id = assignment_[id_];
    value.assignment_epoch = assignment_epoch_;
    value.assignment_digest = assignment_epoch_ ? de::fullAssignmentSignature(assignment_) : 0;
    value.drone_to_slot.assign(assignment_.begin(), assignment_.end());
    std::copy(own_goal_.begin(), own_goal_.end(), value.assigned_goal.begin());
    value.goal_valid = own_goal_valid_ &&
        applied_epochs_[id_] == assignment_epoch_ &&
        applied_assignments_[id_] == assignment_;
    value.startup_ready = local_startup_ready_;
    state(ego_planner::FormationAgentState::ConstPtr(new ego_planner::FormationAgentState(value)));
    state_pub_.publish(value);
  }
  void state(const ego_planner::FormationAgentState::ConstPtr& msg) {
    if (msg->formation_digest != de::formationDigest() ||
        msg->drone_id >= static_cast<unsigned>(de::agentCount()) ||
        msg->assignment_versions.size() != static_cast<size_t>(de::columnCount()) ||
        msg->drone_to_slot.size() != static_cast<size_t>(de::agentCount())) return;
    for (double value : msg->position) if (!std::isfinite(value)) return;
    for (double value : msg->velocity) if (!std::isfinite(value)) return;
    for (double value : msg->assigned_goal) if (!std::isfinite(value)) return;
    de::Assignment peer_assignment(de::agentCount());
    std::copy(msg->drone_to_slot.begin(), msg->drone_to_slot.end(), peer_assignment.begin());
    if (!de::validAssignment(peer_assignment) ||
        msg->slot_id != static_cast<unsigned>(peer_assignment[msg->drone_id])) return;
    if (msg->assignment_epoch) {
      if (msg->assignment_digest != de::fullAssignmentSignature(peer_assignment)) return;
    } else if (msg->assignment_digest != 0 ||
               peer_assignment != de::assignmentFromVersions(msg->assignment_versions)) return;
    const double now = ros::Time::now().toSec();
    auto& lease = state_leases_[msg->drone_id];
    if (lease.accept(msg->session, msg->sequence, msg->header.stamp.toSec(),
                     now, ttl_, true)) {
      positions_[msg->drone_id] = {msg->position[0], msg->position[1], msg->position[2]};
      std::copy(msg->assignment_versions.begin(), msg->assignment_versions.end(),
                peer_versions_[msg->drone_id].begin());
      peer_assignments_[msg->drone_id] = peer_assignment;
      peer_epochs_[msg->drone_id] = msg->assignment_epoch;
      peer_digests_[msg->drone_id] = msg->assignment_digest;
      for (int j = 0; j < 3; ++j) {
        velocities_[msg->drone_id][j] = msg->velocity[j];
        goals_[msg->drone_id][j] = msg->assigned_goal[j];
      }
      goal_valid_[msg->drone_id] = msg->goal_valid;
    }
    startup_leases_[msg->drone_id].accept(
        msg->session, msg->sequence, msg->header.stamp.toSec(), now, ttl_,
        msg->startup_ready);
  }
  void evidence(const ego_planner::FormationEdgeEvidence::ConstPtr& msg) {
    if (msg->formation_digest != de::formationDigest() ||
        msg->assignment_versions.size() != static_cast<size_t>(de::columnCount()) ||
        msg->assignment_epoch != assignment_epoch_ ||
        msg->assignment_digest != (assignment_epoch_ ? de::fullAssignmentSignature(assignment_) : 0) ||
        !de::edgeVersionsMatch(msg->edge_index, versions_,
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
    if (!free_geometry_message_ || !sameCloudGeometry(*free_geometry_message_, *msg)) {
      pcl::fromROSMsg(*msg, free_);
      ++free_geometry_generation_;
    }
    free_geometry_message_ = msg;
    free_stamp_ = msg->header.stamp;
    rebuild();
  }
  void inflatedCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != map_frame_ || msg->header.stamp <= inflated_stamp_ ||
        msg->header.stamp > ros::Time::now()+ros::Duration(0.05)) return;
    if (!inflated_geometry_message_ || !sameCloudGeometry(*inflated_geometry_message_, *msg)) {
      pcl::fromROSMsg(*msg, inflated_);
      ++inflated_geometry_generation_;
    }
    inflated_geometry_message_ = msg;
    inflated_stamp_ = msg->header.stamp;
    rebuild();
  }
  void rebuild() {
    // Both layers must refer to the same local sensor update.
    if (occupied_stamp_.isZero() || occupied_stamp_ != free_stamp_ || inflated_stamp_.isZero()) return;
    if (!field_ || field_free_generation_ != free_geometry_generation_ ||
        field_inflated_generation_ != inflated_geometry_generation_) {
      field_.reset(new VoxelObstacleField(inflated_, planner_layout_, free_,
                   starconvex_adapter::VoxelGridLayout(resolution_), planner_size_));
      field_free_generation_ = free_geometry_generation_;
      field_inflated_generation_ = inflated_geometry_generation_;
      ++field_geometry_generation_;
    }
    // Reusing identical contents never renews a cached sensor sample. Freshness
    // still comes from actual, paired observed frames and the planner's source
    // timestamp, including when one of those sources has stopped advancing.
    field_stamp_ = std::min(occupied_stamp_,inflated_stamp_);
  }
  bool corridor(const Vec3& a, const Vec3& b) const {
    // The planner's occupied cells already include the vehicle safety margin.
    // Traverse the centerline's cells without adding another 0.1 m band.
    return field_->knownFreeSegment(a,b);
  }
  double overlap(int edge, int first_id, int second_id,
                 std::vector<std::unique_ptr<OriginalStarSpace>>& hulls) {
    const Vec3& a = positions_[first_id];
    const Vec3& b = positions_[second_id];
    auto& cached = overlap_cache_[edge];
    // Freshness is checked by evaluate() before reaching here. The score is a
    // pure function of the exact field contents and the two endpoint poses;
    // identical new sensor frames need not repeat an identical calculation.
    if (cached.valid && cached.field_generation == field_geometry_generation_ &&
        cached.first_id == first_id && cached.second_id == second_id &&
        cached.first.x == a.x && cached.first.y == a.y && cached.first.z == a.z &&
        cached.second.x == b.x && cached.second.y == b.y && cached.second.z == b.z)
      return cached.score;
    const auto hull = [this, &hulls](int id) -> OriginalStarSpace& {
      if (!hulls[id])
        hulls[id].reset(new OriginalStarSpace(
            positions_[id], starconvex_adapter::generateBoundaryCloud(
                positions_[id], *field_, geometry_)));
      return *hulls[id];
    };
    auto& first = hull(first_id);
    auto& second = hull(second_id);
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
    const double score = std::min(va,vb)
        ? static_cast<double>(intersection)/std::min(va,vb) : 0;
    cached.valid = true;
    cached.field_generation = field_geometry_generation_;
    cached.first_id = first_id; cached.second_id = second_id;
    cached.first = a; cached.second = b; cached.score = score;
    return score;
  }
  de::AllAgentAssignment::View assignmentView() const {
    de::AllAgentAssignment::View view(de::agentCount());
    const auto now = ros::Time::now();
    for (int id = 0; id < de::kAgents; ++id) {
      auto& m = view[id];
      m.lease = state_leases_[id];
      m.assignment = peer_assignments_[id];
      m.epoch = peer_epochs_[id]; m.digest = peer_digests_[id];
      m.position = {{positions_[id].x, positions_[id].y, positions_[id].z}};
      m.velocity = velocities_[id]; m.goal = goals_[id];
      m.goal_valid = goal_valid_[id];
      m.applied = !applied_stamps_[id].isZero() && now >= applied_stamps_[id] &&
          (now - applied_stamps_[id]).toSec() <= ttl_ &&
          applied_epochs_[id] == assignment_epoch_ &&
          applied_digests_[id] == de::fullAssignmentSignature(assignment_) &&
          applied_assignments_[id] == assignment_;
    }
    return view;
  }
  void applyFullAssignment(uint64_t epoch, const de::Assignment& assignment) {
    assignment_epoch_ = epoch;
    assignment_ = assignment;
    // Arbitrary permutations can change every physical endpoint. Never reuse
    // old-column evidence or an approach timer under a new assignment epoch.
    table_.clear();
    std::fill(kept_.begin(), kept_.end(), false);
    std::fill(weak_reconnected_.begin(), weak_reconnected_.end(), false);
    std::fill(visibility_reconnected_.begin(), visibility_reconnected_.end(), false);
    std::fill(candidate_weak_.begin(), candidate_weak_.end(), false);
    for (auto& approach : approach_) approach.reset();
    std::fill(ready_since_.begin(), ready_since_.end(), ros::Time());
    std::fill(cut_since_.begin(), cut_since_.end(), ros::Time());
    std::fill(normal_since_.begin(), normal_since_.end(), ros::Time());
    own_goal_valid_ = false;
    last_evaluation_ = ros::Time();
    last_swap_ = ros::Time::now();
  }
  struct PendingSwap {
    uint64_t transaction = 0, proposer_session = 0;
    int front = -1, rear = -1;
    uint32_t base_version = 0;
    double gain = 0.0;
    ros::Time started, last_sent;
    std::vector<de::Lease> acks = std::vector<de::Lease>(de::agentCount());
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
    message.formation_digest = de::formationDigest();
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
    if (!swap_enabled_ || swap_scope_ != "column" ||
        msg->formation_digest != de::formationDigest()) return;
    const ros::Time now = ros::Time::now();
    if (msg->sender_id >= static_cast<unsigned>(de::agentCount()) ||
        msg->column >= static_cast<unsigned>(de::columnCount()) ||
        msg->front_drone >= static_cast<unsigned>(de::agentCount()) ||
        msg->rear_drone >= static_cast<unsigned>(de::agentCount()) ||
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
    if (!swap_enabled_ || !startup_ready_ || swap_scope_ != "column") return;
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
    // All work in this callback sees one field/pose snapshot. Reuse its hulls
    // between incident edges, then discard them before accepting any new input.
    std::vector<std::unique_ptr<OriginalStarSpace>> hulls(de::agentCount());
    for (int edge = 0; edge < de::kEdges; ++edge) {
      const auto& pair = de::edges()[edge];
      const int first = de::owner(assignment_, pair[0]);
      const int second = de::owner(assignment_, pair[1]);
      if (first != id_ && second != id_) continue;
      const int peer = first == id_ ? second : first;
      double score = 0, visibility = 0, distance = 0;
      bool score_computed = false;
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
          if (visibility > 0 && de::needsOverlapScore(
                  kept_[edge], weak_reconnected_[edge],
                  visibility_reconnected_[edge], startup_ready_)) {
            score = overlap(edge, id_, peer, hulls);
            score_computed = true;
          }
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
      } else if (visibility_reconnected_[edge] && score_computed && score >= split_score_) {
        visibility_reconnected_[edge] = false;
      }
      ego_planner::FormationEdgeEvidence value;
      value.header.stamp = now;
      value.formation_digest = de::formationDigest();
      value.session = session_;
      value.sequence = ++edge_sequence_;
      value.assignment_versions.assign(versions_.begin(), versions_.end());
      value.assignment_generation = de::assignmentSignature(versions_);
      value.assignment_epoch = assignment_epoch_;
      value.assignment_digest = assignment_epoch_ ? de::fullAssignmentSignature(assignment_) : 0;
      value.edge_index = edge;
      value.observer_id = id_;
      value.peer_id = peer;
      value.decision = kept_[edge] ? value.KEEP : value.CUT;
      value.weak_recovery = kept_[edge] && weak_reconnected_[edge];
      // Zero is also the diagnostic placeholder when this state does not use
      // overlap. It must never be mistaken for a measured recovery score.
      value.score = score; value.visibility = visibility; value.distance = distance;
      table_.updateAt(edge, id_, peer, assignment_, session_, value.sequence,
                      now.toSec(), now.toSec(), ttl_, kept_[edge],
                      value.weak_recovery);
      edge_pub_.publish(value);
    }
  }
  void tick(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    if (last_evaluation_.isZero() || (now-last_evaluation_).toSec() >= 0.30) {
      evaluate(now);
      // CPU-heavy geometry must not make expired endpoint leases appear fresh
      // by checking them against the callback's earlier start timestamp.
      now = ros::Time::now();
      last_evaluation_ = now;
    }
    const uint32_t mask_before_swap = table_.mask(now.toSec(), ttl_);
    const uint32_t full_mask = de::fullMask();
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
                 de::agentCount());
      }
    }
    if (startup_ready_) {
      if (mask_before_swap == full_mask) full_since_swap_ = true;
      else if (full_since_swap_) swap_armed_ = true;
      evaluateSwaps(now);
    }
    if (reassignment_) reassignment_->tick(startup_ready_ && swap_armed_);
    // applySwap() invalidates incident evidence. Publish the new assignment
    // only with a mask recomputed from that same post-swap edge table. Refresh
    // time after assignment work as well, so expired leases cannot be renewed
    // by attaching a later command timestamp to an earlier mask check.
    now = ros::Time::now();
    const uint32_t mask = table_.mask(now.toSec(), ttl_);
    ego_planner::FormationTopologyCommand command;
    command.header.stamp = now;
    command.formation_digest = de::formationDigest();
    command.epoch = command.header.stamp.toNSec();
    // Always use the optimizer's relative-edge branch, including a fully connected graph.
    command.mode = command.SPLIT;
    command.edge_count = de::kEdges;
    command.edge_keep_mask = mask;
    command.weak_edge_mask = table_.weakMask(now.toSec(), ttl_);
    command.assignment_versions.assign(versions_.begin(), versions_.end());
    command.assignment_epoch = assignment_epoch_;
    command.assignment_digest = assignment_epoch_ ? de::fullAssignmentSignature(assignment_) : 0;
    command.drone_to_slot.assign(assignment_.begin(), assignment_.end());
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
  std::vector<ros::Subscriber> applied_subs_;
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
  std::string swap_scope_;
  uint64_t assignment_epoch_{0};
  std::unique_ptr<de::AllAgentAssignment> reassignment_;
  uint64_t session_, state_sequence_{0}, edge_sequence_{0}, swap_sequence_{0},
      transaction_sequence_{0};
  de::Assignment assignment_{};
  std::vector<uint32_t> versions_;
  std::vector<std::vector<uint32_t>> peer_versions_, applied_versions_;
  std::vector<ros::Time> applied_stamps_;
  std::vector<uint64_t> peer_epochs_, peer_digests_, applied_epochs_, applied_digests_;
  std::vector<de::Assignment> peer_assignments_, applied_assignments_;
  std::vector<std::array<double, 3>> velocities_, goals_;
  std::vector<bool> goal_valid_;
  std::array<double, 3> own_goal_{};
  bool own_goal_valid_{false};
  std::vector<de::Lease> swap_writers_;
  std::vector<PendingSwap> pending_;
  std::vector<std::vector<PendingSwap>> history_;
  std::vector<ros::Time> candidate_since_;
  ros::Time last_swap_;
  std::vector<Vec3> positions_;
  std::vector<de::Lease> state_leases_, startup_leases_;
  de::EdgeTable table_;
  std::vector<bool> kept_, weak_reconnected_, visibility_reconnected_, candidate_weak_;
  std::vector<de::WeakRecoveryApproach> approach_;
  struct OverlapCache {
    bool valid{false};
    uint64_t field_generation{0};
    int first_id{-1}, second_id{-1};
    Vec3 first, second;
    double score{0};
  };
  std::vector<OverlapCache> overlap_cache_;
  std::vector<ros::Time> ready_since_, cut_since_, normal_since_;
  pcl::PointCloud<pcl::PointXYZ> free_, inflated_;
  sensor_msgs::PointCloud2::ConstPtr free_geometry_message_, inflated_geometry_message_;
  uint64_t free_geometry_generation_{0}, inflated_geometry_generation_{0};
  uint64_t field_free_generation_{0}, field_inflated_generation_{0};
  uint64_t field_geometry_generation_{0};
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

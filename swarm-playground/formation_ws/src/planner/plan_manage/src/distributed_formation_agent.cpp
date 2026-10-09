#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ego_planner/FormationAgentState.h>
#include <ego_planner/FormationEdgeEvidence.h>
#include <ego_planner/FormationTopologyCommand.h>
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
    nh_.param("cut_hold", cut_hold_, 1.0);
    nh_.param("resolution", resolution_, 0.2);
    nh_.param("inflation", inflation_, 0.2);
    nh_.param("formation_scale", formation_scale_, 0.15);
    nh_.param("split_distance", split_distance_, 1.8);
    nh_.param("merge_distance", merge_distance_, 1.7);
    nh_.param("weak_reconnect_distance", weak_reconnect_distance_, 3.0);
    nh_.param("weak_formation_scale", weak_formation_scale_, 0.22);
    nh_.param("split_score", split_score_, 0.34);
    if (id_ < 0 || id_ >= de::kAgents || ttl_ <= 0 ||
        map_timeout_ <= 0 || reconnect_hold_ < 0 ||
        weak_reconnect_hold_ < 0 || cut_hold_ < 0 ||
        resolution_ <= 0 || inflation_ < 0 || formation_scale_ < 0 ||
        merge_distance_ <= 0 || split_distance_ < merge_distance_ ||
        weak_reconnect_distance_ < merge_distance_ ||
        weak_formation_scale_ < formation_scale_ ||
        split_score_ < 0 || split_score_ > 1)
      throw std::runtime_error("invalid distributed agent parameters");
    geometry_.delta_floor = -0.5;
    geometry_.delta_roof = 0.5;
    geometry_.frontier_floor = 0.0;
    geometry_.frontier_roof = 3.0;
    geometry_.raycast_resolution = resolution_;
    state_pub_ = nh_.advertise<ego_planner::FormationAgentState>("/swarm/agent_state", 32);
    edge_pub_ = nh_.advertise<ego_planner::FormationEdgeEvidence>("/swarm/edge_evidence", 64);
    command_pub_ = nh_.advertise<ego_planner::FormationTopologyCommand>("topology", 1);
    odom_sub_ = nh_.subscribe("odom", 1, &Agent::odom, this);
    state_sub_ = nh_.subscribe("/swarm/agent_state", 64, &Agent::state, this);
    evidence_sub_ = nh_.subscribe("/swarm/edge_evidence", 128, &Agent::evidence, this);
    occupied_sub_ = nh_.subscribe("occupied", 1, &Agent::occupied, this);
    free_sub_ = nh_.subscribe("free", 1, &Agent::freeCloud, this);
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
                     now, ttl_, true))
      positions_[msg->drone_id] = {msg->position[0], msg->position[1], msg->position[2]};
    startup_leases_[msg->drone_id].accept(
        msg->session, msg->sequence, msg->header.stamp.toSec(), now, ttl_,
        msg->startup_ready);
  }
  void evidence(const ego_planner::FormationEdgeEvidence::ConstPtr& msg) {
    if (msg->assignment_generation != 0 ||
        msg->decision > ego_planner::FormationEdgeEvidence::KEEP ||
        !std::isfinite(msg->score) || !std::isfinite(msg->visibility) ||
        !std::isfinite(msg->distance)) return;
    table_.update(msg->observer_id, msg->peer_id, msg->session, msg->sequence,
                  msg->header.stamp.toSec(), ros::Time::now().toSec(), ttl_,
                  msg->decision == ego_planner::FormationEdgeEvidence::KEEP);
  }
  void occupied(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.stamp <= occupied_stamp_) return;
    pcl::fromROSMsg(*msg, occupied_);
    occupied_stamp_ = msg->header.stamp;
    rebuild();
  }
  void freeCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.stamp <= free_stamp_) return;
    pcl::fromROSMsg(*msg, free_);
    free_stamp_ = msg->header.stamp;
    rebuild();
  }
  void rebuild() {
    // Both layers must refer to the same local sensor update.
    if (occupied_stamp_.isZero() || occupied_stamp_ != free_stamp_) return;
    field_.reset(new VoxelObstacleField(occupied_, free_, resolution_, inflation_));
    field_stamp_ = occupied_stamp_;
  }
  bool corridor(const Vec3& a, const Vec3& b) const {
    const double length = norm(subtract(b, a));
    const double planar = std::hypot(b.x - a.x, b.y - a.y);
    const double px = planar > 1e-6 ? -(b.y - a.y) * 0.1 / planar : 0.1;
    const double py = planar > 1e-6 ? (b.x - a.x) * 0.1 / planar : 0.0;
    const std::array<Vec3, 5> offsets{{{0,0,0}, {px,py,0}, {-px,-py,0},
                                             {0,0,0.1}, {0,0,-0.1}}};
    const int samples = std::max(1, static_cast<int>(std::ceil(length / (resolution_ * 0.5))));
    for (const auto& offset : offsets)
      for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / samples;
        if (!field_->knownFree({a.x + t*(b.x-a.x) + offset.x,
                               a.y + t*(b.y-a.y) + offset.y,
                               a.z + t*(b.z-a.z) + offset.z})) return false;
      }
    return true;
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
  void evaluate(const ros::Time& now) {
    const bool local_ready = field_ && now >= field_stamp_ &&
        (now-field_stamp_).toSec() <= map_timeout_ &&
        state_leases_[id_].fresh(now.toSec(), ttl_);
    for (int edge = 0; edge < de::kEdges; ++edge) {
      const auto& pair = de::edges()[edge];
      if (pair[0] != id_ && pair[1] != id_) continue;
      const int peer = pair[0] == id_ ? pair[1] : pair[0];
      double score = 0, visibility = 0, distance = 0;
      bool safe = local_ready && state_leases_[peer].fresh(now.toSec(), ttl_);
      if (safe) {
        distance = norm(subtract(positions_[id_], positions_[peer]));
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
        if (kept_[edge]) cut_since_[edge] = now;
        kept_[edge] = false;
        weak_reconnected_[edge] = false;
        visibility_reconnected_[edge] = false;
        ready_since_[edge] = ros::Time();
      } else if (!kept_[edge]) {
        if (ready_since_[edge].isZero()) ready_since_[edge] = now;
        // Match centralized weak recovery's short visibility window without
        // weakening the ordinary close-range reconnect debounce.
        const double required_hold = distance > merge_distance_
                                         ? weak_reconnect_hold_
                                         : reconnect_hold_;
        if ((now-ready_since_[edge]).toSec() >= required_hold &&
            (cut_since_[edge].isZero() ||
             (now-cut_since_[edge]).toSec() >= cut_hold_)) {
          kept_[edge] = true;
          weak_reconnected_[edge] = distance > merge_distance_;
          visibility_reconnected_[edge] = !weak_reconnected_[edge];
        }
      } else if (weak_reconnected_[edge] && distance <= merge_distance_) {
        weak_reconnected_[edge] = false;
        visibility_reconnected_[edge] = true;
      } else if (visibility_reconnected_[edge] && score >= split_score_) {
        visibility_reconnected_[edge] = false;
      }
      ego_planner::FormationEdgeEvidence value;
      value.header.stamp = now;
      value.session = session_;
      value.sequence = ++edge_sequence_;
      value.observer_id = id_;
      value.peer_id = peer;
      value.decision = kept_[edge] ? value.KEEP : value.CUT;
      value.score = score; value.visibility = visibility; value.distance = distance;
      table_.update(id_, peer, session_, value.sequence, now.toSec(), now.toSec(), ttl_, kept_[edge]);
      edge_pub_.publish(value);
    }
  }
  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (last_evaluation_.isZero() || (now-last_evaluation_).toSec() >= 0.30) {
      evaluate(now);
      last_evaluation_ = now;
    }
    const uint32_t mask = table_.mask(now.toSec(), ttl_);
    constexpr uint32_t full_mask =
        (uint32_t{1} << de::kEdges) - uint32_t{1};
    if (!local_startup_ready_ && mask == full_mask) {
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
    ego_planner::FormationTopologyCommand command;
    command.header.stamp = ros::Time::now();
    command.epoch = command.header.stamp.toNSec();
    // Always use the optimizer's relative-edge branch, including a fully connected graph.
    command.mode = command.SPLIT;
    command.edge_count = de::kEdges;
    command.edge_keep_mask = mask;
    command.components = de::components(command.edge_keep_mask);
    const bool weak_recovery = std::any_of(
        weak_reconnected_.begin(), weak_reconnected_.end(),
        [](bool value) { return value; });
    command.formation_scale = weak_recovery ? weak_formation_scale_
                                            : formation_scale_;
    command.progress_sync_scale = 0.0;
    command.reason = startup_ready_ ? "DISTRIBUTED_ENDPOINT_LEASES"
                                    : "DISTRIBUTED_STARTUP_WAIT";
    command_pub_.publish(command);
  }
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, state_sub_, evidence_sub_, occupied_sub_, free_sub_;
  ros::Publisher state_pub_, edge_pub_, command_pub_;
  ros::Timer timer_;
  int id_;
  double ttl_, map_timeout_, reconnect_hold_, weak_reconnect_hold_, cut_hold_;
  double resolution_, inflation_;
  double formation_scale_, split_distance_, merge_distance_, split_score_;
  double weak_reconnect_distance_, weak_formation_scale_;
  uint64_t session_, state_sequence_{0}, edge_sequence_{0};
  std::array<Vec3, de::kAgents> positions_{};
  std::array<de::Lease, de::kAgents> state_leases_{};
  std::array<de::Lease, de::kAgents> startup_leases_{};
  de::EdgeTable table_;
  std::array<bool, de::kEdges> kept_{};
  std::array<bool, de::kEdges> weak_reconnected_{};
  std::array<bool, de::kEdges> visibility_reconnected_{};
  std::array<ros::Time, de::kEdges> ready_since_{}, cut_since_{};
  pcl::PointCloud<pcl::PointXYZ> occupied_, free_;
  ros::Time occupied_stamp_, free_stamp_, field_stamp_, last_evaluation_;
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

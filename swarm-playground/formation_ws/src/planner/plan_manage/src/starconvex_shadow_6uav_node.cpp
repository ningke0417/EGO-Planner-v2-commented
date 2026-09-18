#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ego_planner/FormationTopologyCommand.h>
#include <starconvex_adapter/original_star_space.hpp>

namespace {

using starconvex_adapter::GeometryOptions;
using starconvex_adapter::OriginalStarSpace;
using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::norm;
using starconvex_adapter::subtract;

constexpr std::size_t kUavCount = 6;
constexpr std::size_t kEdgeCount = 7;//只用7条最近邻边做拓扑判定
// The existing six-UAV formation is a 2x3 grid.  Only nearest-neighbour
// formation constraints participate in topology decisions.
const std::array<std::array<int, 2>, kEdgeCount> kSlotEdges{{
    {{0, 1}}, {{1, 2}}, {{3, 4}}, {{4, 5}},
    {{0, 3}}, {{1, 4}}, {{2, 5}}}};
std::array<std::array<int, 2>, kEdgeCount> gEdges = kSlotEdges;

double planarDistance(const Vec3& first, const Vec3& second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}

geometry_msgs::Point pointMessage(const Vec3& value) {
  geometry_msgs::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

std::string edgeName(std::size_t edge) {
  return std::to_string(gEdges[edge][0]) + std::to_string(gEdges[edge][1]);
}

class SixUavShadow {
 public:
  SixUavShadow() : private_nh_("~") {
    loadParameters();
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      const std::string topic =
          "/drone_" + std::to_string(uav) + "_" + odom_suffix_;
      odom_subscribers_[uav] = nh_.subscribe<nav_msgs::Odometry>(
          topic, 1,
          [this, uav](const nav_msgs::Odometry::ConstPtr& message) {
            positions_[uav] = {message->pose.pose.position.x,
                               message->pose.pose.position.y,
                               message->pose.pose.position.z};
            odom_stamps_[uav] = ros::Time::now();
            odom_seen_[uav] = true;
          });
    }
    obstacle_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        obstacle_cloud_topic_, 1, &SixUavShadow::obstacleCloudCallback, this);
    known_free_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        known_free_cloud_topic_, 1, &SixUavShadow::knownFreeCloudCallback, this);
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &SixUavShadow::assignmentCallback, this);
    topology_command_subscriber_ =
        nh_.subscribe<ego_planner::FormationTopologyCommand>(
            "/formation_backend/topology_command", 1,
            &SixUavShadow::topologyCommandCallback, this);
    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
    score_pub_ =
        private_nh_.advertise<std_msgs::Float64MultiArray>("scores", 1, true);
    merge_score_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>(
        "merge_scores", 1, true);
    summary_pub_ = private_nh_.advertise<std_msgs::String>("summary", 1, true);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
        "markers", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &SixUavShadow::timerCallback, this);
    ROS_INFO_STREAM(
        "Six-UAV StarConvex criterion ready: grid edges="
        "01,12,34,45,03,14,25 split="
        << split_threshold_ << " hard_split=" << hard_split_threshold_
        << " merge=" << merge_threshold_
        << " distance_split=" << distance_split_threshold_ << " m"
        << " distance_merge=" << distance_merge_threshold_ << " m"
        << " persistence=" << persistence_distance_
        << " m front_to_rear_propagation="
        << (rear_split_propagation_enabled_ ? "enabled" : "disabled")
        << " factor=" << rear_split_propagation_factor_
        << " speed=" << rear_split_propagation_speed_ << " m/s");
  }

 private:
  enum class EdgeState { KEEP, CUT };

  void topologyCommandCallback(
      const ego_planner::FormationTopologyCommand::ConstPtr& message) {
    if (message->mode != ego_planner::FormationTopologyCommand::INTACT ||
        message->reason != "VISIBILITY_MERGED_AFTER_WEAK_RECOVERY") {
      return;
    }

    // The bridge has already checked every current formation edge against the
    // synchronized known-free map, distance gate and hard-safety gate.  Align
    // the criterion hysteresis with that accepted visibility merge so a stale
    // CUT state cannot immediately command another split merely because its
    // overlap score has not yet climbed to merge_threshold_.
    edge_states_.fill(EdgeState::KEEP);
    pending_distances_.fill(0.0);
    pending_seen_.fill(false);
    edge_cut_since_.fill(ros::Time());
    reconnect_candidate_since_.fill(ros::Time());
    rear_forced_cut_until_.fill(ros::Time());
    ROS_WARN_STREAM("[6UAV_CRITERION] Accepted visibility merge epoch="
                    << message->epoch
                    << "; synchronized all criterion edges to KEEP.");
  }

  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    const bool versioned = message->data.size() == kUavCount + 1;
    if (!versioned && message->data.size() != kUavCount) {
      ROS_ERROR_STREAM("[6UAV_CRITERION] Reject assignment with size "
                       << message->data.size());
      return;
    }
    const uint32_t generation = versioned ? message->data[0] : 0;
    if ((versioned && generation <= assignment_generation_) ||
        (!versioned && assignment_received_))
      return;
    const std::size_t offset = versioned ? 1 : 0;
    std::array<int, kUavCount> slot_to_drone{};
    std::array<bool, kUavCount> seen{{false, false, false, false, false, false}};
    for (std::size_t drone = 0; drone < kUavCount; ++drone) {
      const std::size_t slot = message->data[drone + offset];
      if (slot >= kUavCount || seen[slot]) {
        ROS_ERROR("[6UAV_CRITERION] Reject non-permutation assignment");
        return;
      }
      seen[slot] = true;
      slot_to_drone[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      gEdges[edge][0] = slot_to_drone[kSlotEdges[edge][0]];
      gEdges[edge][1] = slot_to_drone[kSlotEdges[edge][1]];
    }
    // edge_states_ is indexed by slot edge, just like kSlotEdges.  Keep the
    // hysteresis state across a physical-drone reassignment; resetting every
    // edge to KEEP publishes a transient INTACT result during an active split.
    pending_distances_.fill(0.0);
    pending_seen_.fill(false);
    rear_forced_cut_until_.fill(ros::Time());
    assignment_generation_ = generation;
    assignment_received_ = true;
    ROS_WARN_STREAM("[6UAV_CRITERION] generation=" << generation
                    << " slot topology mapped to physical edges: "
                    << edgeName(0) << "," << edgeName(1) << ","
                    << edgeName(2) << "," << edgeName(3) << ","
                    << edgeName(4) << "," << edgeName(5) << ","
                    << edgeName(6));
  }

  void loadParameters() {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param<std::string>("obstacle_cloud_topic",
                                   obstacle_cloud_topic_,
                                   "/swarm/observed_occupied");
    private_nh_.param<std::string>("known_free_cloud_topic",
                                   known_free_cloud_topic_,
                                   "/swarm/observed_free");
    private_nh_.param("map_timeout", map_timeout_, 1.0);
    private_nh_.param("map_sync_tolerance", map_sync_tolerance_, 0.05);
    private_nh_.param("map_warmup_duration", map_warmup_duration_, 2.0);
    private_nh_.param("voxel_resolution", voxel_resolution_, 0.10);
    private_nh_.param("publish_rate", publish_rate_, 1.0);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    private_nh_.param("obstacle_inflation", obstacle_inflation_, 0.20);
    private_nh_.param("sample_xy", sample_xy_, 0.25);
    private_nh_.param("sample_z", sample_z_, 0.10);
    private_nh_.param("narrow_clearance", narrow_clearance_, 0.60);
    private_nh_.param("narrow_weight", narrow_weight_, 0.45);
    private_nh_.param("known_free_corridor_radius",
                      known_free_corridor_radius_, 0.20);
    private_nh_.param("split_threshold", split_threshold_, 0.34);
    private_nh_.param("hard_split_threshold", hard_split_threshold_, 0.20);
    private_nh_.param("merge_threshold", merge_threshold_, 0.70);
    private_nh_.param("distance_split_threshold",
                      distance_split_threshold_, 1.80);
    private_nh_.param("distance_merge_threshold",
                      distance_merge_threshold_, 1.70);
    private_nh_.param("persistence_distance", persistence_distance_, 0.05);
    private_nh_.param("edge_cut_min_hold_duration",
                      edge_cut_min_hold_duration_, 1.0);
    private_nh_.param("edge_reconnect_persistence",
                      edge_reconnect_persistence_, 0.8);
    private_nh_.param("startup_topology_gate_enabled",
                      startup_topology_gate_enabled_, true);
    private_nh_.param("startup_edge_ready_persistence",
                      startup_edge_ready_persistence_, 0.5);
    private_nh_.param("startup_topology_gate_max_wait",
                      startup_topology_gate_max_wait_, 3.0);
    private_nh_.param("rear_split_propagation_enabled",
                      rear_split_propagation_enabled_, true);
    private_nh_.param("rear_split_propagation_factor",
                      rear_split_propagation_factor_, 1.3);
    private_nh_.param("rear_split_propagation_speed",
                      rear_split_propagation_speed_, 1.0);
    private_nh_.param("rear_split_propagation_min_time",
                      rear_split_propagation_min_time_, 0.3);
    private_nh_.param("rear_split_propagation_max_time",
                      rear_split_propagation_max_time_, 6.0);
    private_nh_.param("sensor_range", geometry_.sensor_range, 5.0);
    private_nh_.param("azimuth_step", geometry_.azimuth_step, 0.10);
    private_nh_.param("raycast_resolution", geometry_.raycast_resolution, 0.10);
    private_nh_.param("vertical_resolution", geometry_.vertical_resolution, 0.20);
    private_nh_.param("frontier_floor", geometry_.frontier_floor, 0.50);
    private_nh_.param("frontier_roof", geometry_.frontier_roof, 1.50);
    private_nh_.param("delta_floor", geometry_.delta_floor, -0.50);
    private_nh_.param("delta_roof", geometry_.delta_roof, 1.50);
    if (publish_rate_ <= 0.0 || voxel_resolution_ <= 0.0 ||
        obstacle_inflation_ < 0.0 || sample_xy_ <= 0.0 || sample_z_ <= 0.0 ||
        narrow_clearance_ < 0.0 || narrow_weight_ < 0.0 ||
        narrow_weight_ > 1.0 || known_free_corridor_radius_ < 0.0 ||
        split_threshold_ < 0.0 ||
        merge_threshold_ <= split_threshold_ || merge_threshold_ > 1.0 ||
        hard_split_threshold_ < 0.0 ||
        hard_split_threshold_ > split_threshold_ ||
        distance_merge_threshold_ <= 0.0 ||
        distance_merge_threshold_ >= distance_split_threshold_ ||
        distance_split_threshold_ <= 0.0 ||
        persistence_distance_ < 0.0 ||
        edge_cut_min_hold_duration_ < 0.0 ||
        edge_reconnect_persistence_ < 0.0 ||
        startup_edge_ready_persistence_ < 0.0 ||
        startup_topology_gate_max_wait_ <= 0.0 ||
        rear_split_propagation_factor_ <= 0.0 ||
        rear_split_propagation_speed_ <= 0.0 ||
        rear_split_propagation_min_time_ < 0.0 ||
        rear_split_propagation_max_time_ <
            rear_split_propagation_min_time_ ||
        map_timeout_ <= 0.0 || map_sync_tolerance_ < 0.0 ||
        map_warmup_duration_ < 0.0 ||
        geometry_.sensor_range <= 0.0 ||
        geometry_.frontier_roof < geometry_.frontier_floor ||
        geometry_.delta_roof < geometry_.delta_floor) {
      throw std::runtime_error("invalid six-UAV StarConvex parameters");
    }
  }

  void obstacleCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::fromROSMsg(*message, latest_occupied_cloud_);
    occupied_source_stamp_ = message->header.stamp;
    occupied_seen_ = true;
    rebuildObstacleFields();
  }

  void knownFreeCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::fromROSMsg(*message, latest_known_free_cloud_);
    known_free_source_stamp_ = message->header.stamp;
    known_free_seen_ = true;
    rebuildObstacleFields();
  }

  void rebuildObstacleFields() {
    if (!occupied_seen_ || !known_free_seen_) return;
    if (std::fabs((occupied_source_stamp_ - known_free_source_stamp_).toSec()) >
        map_sync_tolerance_) {
      ROS_WARN_THROTTLE(2.0,
                        "Six-UAV occupied/free map layers are not synchronized.");
      return;
    }
    try {
      obstacle_field_.reset(new VoxelObstacleField(
          latest_occupied_cloud_, latest_known_free_cloud_, voxel_resolution_,
          obstacle_inflation_));
      narrow_obstacle_field_.reset(new VoxelObstacleField(
          latest_occupied_cloud_, latest_known_free_cloud_, voxel_resolution_,
          obstacle_inflation_ + narrow_clearance_));
      map_seen_ = true;
      map_update_receipt_stamp_ = ros::Time::now();
      ROS_INFO_STREAM_THROTTLE(
          2.0, "Six-UAV observed map ready: occupied_points="
                   << latest_occupied_cloud_.size()
                   << " known_free_points=" << latest_known_free_cloud_.size()
                   << " safety_voxels="
                   << obstacle_field_->occupiedVoxelCount()
                   << " known_free_voxels="
                   << obstacle_field_->knownFreeVoxelCount()
                   << " narrow_voxels="
                   << narrow_obstacle_field_->occupiedVoxelCount());
    } catch (const std::exception& error) {
      map_seen_ = false;
      obstacle_field_.reset();
      narrow_obstacle_field_.reset();
      ROS_ERROR_STREAM("Unable to build six-UAV obstacle field: "
                       << error.what());
    }
  }

  bool inputsReady() const {
    if (!map_seen_) return false;
    const ros::Time now = ros::Time::now();
    if ((now - map_update_receipt_stamp_).toSec() > map_timeout_) return false;
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      if (!odom_seen_[uav] ||
          (now - odom_stamps_[uav]).toSec() > odom_timeout_) {
        return false;
      }
    }
    return true;
  }

  bool segmentBlocked(const VoxelObstacleField& field, const Vec3& first,
                      const Vec3& second) const {
    if (!field.knownFree(first) || !field.knownFree(second)) return true;
    const Vec3 hit = field.firstUnsafe(first, second, voxel_resolution_);
    return norm(subtract(hit, first)) + 0.5 * voxel_resolution_ <
           norm(subtract(second, first));
  }

  bool knownFreeCorridor(const Vec3& first, const Vec3& second) const {
    if (known_free_corridor_radius_ <= 0.0)
      return !segmentBlocked(*obstacle_field_, first, second);
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double planar_length = std::hypot(dx, dy);
    const double px = planar_length > 1e-9
                          ? -dy / planar_length * known_free_corridor_radius_
                          : known_free_corridor_radius_;
    const double py = planar_length > 1e-9
                          ? dx / planar_length * known_free_corridor_radius_
                          : 0.0;
    const std::array<Vec3, 5> offsets{{
        {0.0, 0.0, 0.0},
        {px, py, 0.0},
        {-px, -py, 0.0},
        {0.0, 0.0, known_free_corridor_radius_},
        {0.0, 0.0, -known_free_corridor_radius_}}};
    for (const auto& offset : offsets) {
      const Vec3 offset_first{first.x + offset.x, first.y + offset.y,
                              first.z + offset.z};
      const Vec3 offset_second{second.x + offset.x, second.y + offset.y,
                               second.z + offset.z};
      if (segmentBlocked(*obstacle_field_, offset_first, offset_second))
        return false;
    }
    return true;
  }

  double visibilityWeight(const Vec3& first, const Vec3& second,
                          double* clearance) const {
    if (!knownFreeCorridor(first, second)) {
      *clearance = -voxel_resolution_;
      return 0.0;
    }
    if (segmentBlocked(*narrow_obstacle_field_, first, second)) {
      *clearance = 0.5 * narrow_clearance_;
      return narrow_weight_;
    }
    *clearance = narrow_clearance_ + voxel_resolution_;
    return 1.0;
  }

  std::array<double, kEdgeCount> calculateScores(
      std::array<double, kEdgeCount>* overlaps,
      std::array<double, kEdgeCount>* clearances,
      std::array<double, kEdgeCount>* visibility_weights) {
    std::array<std::unique_ptr<OriginalStarSpace>, kUavCount> spaces;
    last_safety_violation_ = false;
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      if (!obstacle_field_->knownFree(positions_[uav])) {
        last_safety_violation_ = true;
        continue;
      }
      spaces[uav].reset(new OriginalStarSpace(
          positions_[uav], starconvex_adapter::generateBoundaryCloud(
                               positions_[uav], *obstacle_field_, geometry_)));
    }

    double minimum_x = positions_[0].x;
    double maximum_x = positions_[0].x;
    double minimum_y = positions_[0].y;
    double maximum_y = positions_[0].y;
    double minimum_z = positions_[0].z + geometry_.delta_floor;
    double maximum_z = positions_[0].z + geometry_.delta_roof;
    for (const auto& position : positions_) {
      minimum_x = std::min(minimum_x, position.x);
      maximum_x = std::max(maximum_x, position.x);
      minimum_y = std::min(minimum_y, position.y);
      maximum_y = std::max(maximum_y, position.y);
      minimum_z = std::min(minimum_z, position.z + geometry_.delta_floor);
      maximum_z = std::max(maximum_z, position.z + geometry_.delta_roof);
    }
    minimum_x -= geometry_.sensor_range;
    maximum_x += geometry_.sensor_range;
    minimum_y -= geometry_.sensor_range;
    maximum_y += geometry_.sensor_range;
    minimum_z = std::max(minimum_z, geometry_.frontier_floor);
    maximum_z = std::min(maximum_z, geometry_.frontier_roof);

    const int count_x = std::max(
        1, static_cast<int>(std::ceil((maximum_x - minimum_x) / sample_xy_)));
    const int count_y = std::max(
        1, static_cast<int>(std::ceil((maximum_y - minimum_y) / sample_xy_)));
    const int count_z = std::max(
        1, static_cast<int>(std::ceil((maximum_z - minimum_z) / sample_z_)));
    std::array<std::size_t, kUavCount> volumes{};
    std::array<std::size_t, kEdgeCount> intersections{};
    for (int ix = 0; ix < count_x; ++ix) {
      const double x = minimum_x + (ix + 0.5) * sample_xy_;
      for (int iy = 0; iy < count_y; ++iy) {
        const double y = minimum_y + (iy + 0.5) * sample_xy_;
        for (int iz = 0; iz < count_z; ++iz) {
          const double z = minimum_z + (iz + 0.5) * sample_z_;
          std::array<bool, kUavCount> inside{};
          for (std::size_t uav = 0; uav < kUavCount; ++uav) {
            inside[uav] = spaces[uav] &&
                          obstacle_field_->knownFree({x, y, z}) &&
                          spaces[uav]->contains({x, y, z});
            volumes[uav] += static_cast<std::size_t>(inside[uav]);
          }
          for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
            intersections[edge] += static_cast<std::size_t>(
                inside[gEdges[edge][0]] && inside[gEdges[edge][1]]);
          }
        }
      }
    }

    std::array<double, kEdgeCount> scores{};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const std::size_t smaller =
          std::min(volumes[gEdges[edge][0]], volumes[gEdges[edge][1]]);
      (*overlaps)[edge] =
          smaller ? static_cast<double>(intersections[edge]) /
                        static_cast<double>(smaller)
                  : 0.0;
      (*visibility_weights)[edge] = visibilityWeight(
          positions_[gEdges[edge][0]], positions_[gEdges[edge][1]],
          &(*clearances)[edge]);
      scores[edge] = (*overlaps)[edge] * (*visibility_weights)[edge];
    }
    return scores;
  }

  Vec3 formationCenter() const {
    Vec3 center;
    for (const auto& position : positions_) {
      center.x += position.x / static_cast<double>(kUavCount);
      center.y += position.y / static_cast<double>(kUavCount);
      center.z += position.z / static_cast<double>(kUavCount);
    }
    return center;
  }

  void resetPending(std::size_t edge) {
    pending_distances_[edge] = 0.0;
    pending_seen_[edge] = false;
  }

  void cutEdge(std::size_t edge, const ros::Time& now) {
    if (edge_states_[edge] != EdgeState::CUT) {
      edge_states_[edge] = EdgeState::CUT;
      edge_cut_since_[edge] = now;
    } else if (edge_cut_since_[edge].isZero()) {
      edge_cut_since_[edge] = now;
    }
    reconnect_candidate_since_[edge] = ros::Time();
    resetPending(edge);
  }

  double rearForcedRemaining(std::size_t rear_edge,
                             const ros::Time& now) const {
    if (rear_forced_cut_until_[rear_edge].isZero() ||
        rear_forced_cut_until_[rear_edge] <= now) {
      return 0.0;
    }
    return (rear_forced_cut_until_[rear_edge] - now).toSec();
  }

  bool rearForcedCutActive(std::size_t rear_edge,
                           const ros::Time& now) const {
    return rearForcedRemaining(rear_edge, now) > 0.0;
  }

  double edgeMidpointDistance(std::size_t first_edge,
                              std::size_t second_edge) const {
    const Vec3 first_midpoint{
        0.5 * (positions_[gEdges[first_edge][0]].x +
               positions_[gEdges[first_edge][1]].x),
        0.5 * (positions_[gEdges[first_edge][0]].y +
               positions_[gEdges[first_edge][1]].y),
        0.0};
    const Vec3 second_midpoint{
        0.5 * (positions_[gEdges[second_edge][0]].x +
               positions_[gEdges[second_edge][1]].x),
        0.5 * (positions_[gEdges[second_edge][0]].y +
               positions_[gEdges[second_edge][1]].y),
        0.0};
    return planarDistance(first_midpoint, second_midpoint);
  }

  void triggerRearSplitPropagation(std::size_t front_edge,
                                   std::size_t rear_edge,
                                   const ros::Time& now) {
    if (!rear_split_propagation_enabled_) return;
    const double raw_duration =
        edgeMidpointDistance(front_edge, rear_edge) /
        rear_split_propagation_speed_ * rear_split_propagation_factor_;
    const double duration =
        std::min(rear_split_propagation_max_time_,
                 std::max(rear_split_propagation_min_time_, raw_duration));
    const ros::Time until = now + ros::Duration(duration);
    if (rear_forced_cut_until_[rear_edge].isZero() ||
        until > rear_forced_cut_until_[rear_edge]) {
      rear_forced_cut_until_[rear_edge] = until;
    }
    cutEdge(rear_edge, now);
    ROS_WARN_STREAM("[6UAV_SPLIT_PROPAGATION] Front edge "
                    << gEdges[front_edge][0] << "-"
                    << gEdges[front_edge][1] << " cut; force rear edge "
                    << gEdges[rear_edge][0] << "-"
                    << gEdges[rear_edge][1] << " cut for " << duration
                    << " s.");
  }

  void updateEdgeStates(const std::array<double, kEdgeCount>& scores,
                        const std::array<double, kEdgeCount>& clearances,
                        const Vec3& center) {
    const ros::Time now = ros::Time::now();
    const std::array<bool, 2> front_was_keep{{
        edge_states_[0] == EdgeState::KEEP,
        edge_states_[1] == EdgeState::KEEP}};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const double edge_distance = planarDistance(
          positions_[gEdges[edge][0]], positions_[gEdges[edge][1]]);
      if ((edge == 2 || edge == 3) && rearForcedCutActive(edge, now)) {
        cutEdge(edge, now);
        continue;
      }
      if (edge_states_[edge] == EdgeState::KEEP &&
          (scores[edge] <= hard_split_threshold_ ||
           clearances[edge] <= 0.0 ||
           edge_distance >= distance_split_threshold_)) {
        cutEdge(edge, now);
        continue;
      }
      if (edge_states_[edge] == EdgeState::CUT) {
        resetPending(edge);
        if (edge_cut_since_[edge].isZero()) edge_cut_since_[edge] = now;
        if ((now - edge_cut_since_[edge]).toSec() <
            edge_cut_min_hold_duration_) {
          reconnect_candidate_since_[edge] = ros::Time();
          continue;
        }
        const bool reconnect_safe =
            scores[edge] >= merge_threshold_ &&
            edge_distance <= distance_merge_threshold_;
        if (!reconnect_safe) {
          reconnect_candidate_since_[edge] = ros::Time();
          continue;
        }
        if (reconnect_candidate_since_[edge].isZero())
          reconnect_candidate_since_[edge] = now;
        if ((now - reconnect_candidate_since_[edge]).toSec() >=
            edge_reconnect_persistence_) {
          edge_states_[edge] = EdgeState::KEEP;
          edge_cut_since_[edge] = ros::Time();
          reconnect_candidate_since_[edge] = ros::Time();
          ROS_INFO_STREAM("[6UAV_EDGE_RECONNECT] edge " << edgeName(edge)
                          << " was continuously safe for "
                          << edge_reconnect_persistence_ << " s");
        }
        continue;
      }
      const bool candidate = scores[edge] <= split_threshold_;
      if (!candidate) {
        resetPending(edge);
        continue;
      }
      if (pending_seen_[edge]) {
        pending_distances_[edge] +=
            planarDistance(center, pending_centers_[edge]);
      }
      pending_centers_[edge] = center;
      pending_seen_[edge] = true;
      if (pending_distances_[edge] + 1e-9 >= persistence_distance_) {
        cutEdge(edge, now);
      }
    }

    if (front_was_keep[0] && edge_states_[0] == EdgeState::CUT)
      triggerRearSplitPropagation(0, 2, now);  // 0-1 propagates to 3-4.
    if (front_was_keep[1] && edge_states_[1] == EdgeState::CUT)
      triggerRearSplitPropagation(1, 3, now);  // 1-2 propagates to 4-5.
  }

  std::vector<std::vector<int>> connectedComponents() const {
    std::array<int, kUavCount> parent{{0, 1, 2, 3, 4, 5}};
    const auto find_root = [&parent](int value) {
      int root = value;
      while (parent[root] != root) root = parent[root];
      while (parent[value] != value) {
        const int next = parent[value];
        parent[value] = root;
        value = next;
      }
      return root;
    };
    const auto unite = [&parent, &find_root](int first, int second) {
      const int first_root = find_root(first);
      const int second_root = find_root(second);
      if (first_root != second_root) parent[second_root] = first_root;
    };
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (edge_states_[edge] == EdgeState::KEEP)
        unite(gEdges[edge][0], gEdges[edge][1]);
    }
    std::vector<std::vector<int>> components;
    std::array<int, kUavCount> component_index{{-1, -1, -1, -1, -1, -1}};
    for (int id = 0; id < static_cast<int>(kUavCount); ++id) {
      const int root = find_root(id);
      if (component_index[root] < 0) {
        component_index[root] = static_cast<int>(components.size());
        components.push_back({});
      }
      components[component_index[root]].push_back(id);
    }
    return components;
  }

  std::string componentText(
      const std::vector<std::vector<int>>& components) const {
    std::ostringstream text;
    for (std::size_t component = 0; component < components.size(); ++component) {
      if (component) text << "|";
      text << "{";
      for (std::size_t index = 0; index < components[component].size(); ++index) {
        if (index) text << ",";
        text << components[component][index];
      }
      text << "}";
    }
    return text.str();
  }

  std::string displayState(
      const std::vector<std::vector<int>>& components) const {
    if (components.size() > 1) return "SPLIT";
    if (std::any_of(edge_states_.begin(), edge_states_.end(),
                    [](EdgeState state) { return state == EdgeState::CUT; }))
      return "PARTIAL_CUT";
    if (std::any_of(pending_seen_.begin(), pending_seen_.end(),
                    [](bool pending) { return pending; }))
      return "WATCH";
    return "INTACT";
  }

  void publishMarkers(const std::array<double, kEdgeCount>& scores,
                      const std::string& state,
                      const std::string& components) {
    visualization_msgs::MarkerArray output;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    output.markers.push_back(clear);
    visualization_msgs::Marker label;
    label.header.frame_id = frame_id_;
    label.header.stamp = ros::Time::now();
    label.ns = "six_uav_criterion_state";
    label.id = 1;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.orientation.w = 1.0;
    const Vec3 center = formationCenter();
    label.pose.position = pointMessage({center.x, center.y, center.z + 1.30});
    label.scale.z = 0.26;
    label.color.a = 1.0;
    label.color.r = state == "SPLIT" ? 1.0 : 0.1;
    label.color.g = state == "SPLIT" ? 0.1 : 0.9;
    std::ostringstream text;
    text << "CRITERION " << state << "  minS=" << std::fixed
         << std::setprecision(2)
         << *std::min_element(scores.begin(), scores.end()) << "  "
         << components;
    label.text = text.str();
    output.markers.push_back(label);
    marker_pub_.publish(output);
  }

  void publishMessages(const std::array<double, kEdgeCount>& scores,
                       const std::array<double, kEdgeCount>& overlaps,
                       const std::array<double, kEdgeCount>& clearances,
                       const std::array<double, kEdgeCount>& visibility,
                       const std::string& state,
                       const std::string& components, double compute_time) {
    std_msgs::String state_message;
    state_message.data = state;
    state_pub_.publish(state_message);
    std_msgs::Float64MultiArray score_message;
    score_message.data.assign(scores.begin(), scores.end());
    score_pub_.publish(score_message);
    // Six-UAV merge uses the same product score with a higher threshold.  This
    // separate topic keeps bag-analysis interfaces parallel with the 4-UAV run.
    merge_score_pub_.publish(score_message);

    double max_edge_distance = 0.0;
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      max_edge_distance = std::max(
          max_edge_distance,
          planarDistance(positions_[gEdges[edge][0]], positions_[gEdges[edge][1]]));
    }
    const Vec3 center = formationCenter();
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(4)
            << "{\"assignment_generation\":" << assignment_generation_
            << ",\"state\":\"" << state << "\",\"components\":\""
            << components << "\",\"center_x\":" << center.x
            << ",\"center_y\":" << center.y
            << ",\"cut_score\":"
            << *std::min_element(scores.begin(), scores.end())
            << ",\"max_edge_distance\":" << max_edge_distance;
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const std::string name = edgeName(edge);
      summary << ",\"score_" << name << "\":" << scores[edge]
              << ",\"merge_score_" << name << "\":" << scores[edge]
              << ",\"overlap_" << name << "\":" << overlaps[edge]
              << ",\"clearance_" << name << "\":" << clearances[edge]
              << ",\"visibility_weight_" << name << "\":"
              << visibility[edge]
              << ",\"distance_" << name << "\":"
              << planarDistance(positions_[gEdges[edge][0]],
                                positions_[gEdges[edge][1]])
              << ",\"pending_distance_" << name << "\":"
              << pending_distances_[edge];
    }
    summary << ",\"compute_seconds\":" << compute_time
            << ",\"rear_propagation_remaining_34\":"
            << rearForcedRemaining(2, ros::Time::now())
            << ",\"rear_propagation_remaining_45\":"
            << rearForcedRemaining(3, ros::Time::now())
            << ",\"safety_region_violation\":"
            << (last_safety_violation_ ? "true" : "false") << "}";
    std_msgs::String summary_message;
    summary_message.data = summary.str();
    summary_pub_.publish(summary_message);
    publishMarkers(scores, state, components);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!inputsReady()) {
      inputs_ready_since_ = ros::Time();
      ROS_WARN_THROTTLE(
          2.0,
          "Six-UAV criterion is waiting for fresh observed-map layers and six fresh odometry topics.");
      return;
    }
    const ros::Time now = ros::Time::now();
    if (inputs_ready_since_.isZero()) inputs_ready_since_ = now;
    const double warmup_elapsed = (now - inputs_ready_since_).toSec();
    if (warmup_elapsed < map_warmup_duration_) {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "Six-UAV criterion is warming up the observed map: "
                   << warmup_elapsed << "/" << map_warmup_duration_ << " s");
      return;
    }
    const ros::WallTime start = ros::WallTime::now();
    try {
      std::array<double, kEdgeCount> overlaps{};
      std::array<double, kEdgeCount> clearances{};
      std::array<double, kEdgeCount> visibility{};
      const auto scores =
          calculateScores(&overlaps, &clearances, &visibility);
      const Vec3 center = formationCenter();

      if (startup_topology_gate_enabled_ && !startup_topology_gate_released_) {
        if (startup_topology_gate_started_.isZero())
          startup_topology_gate_started_ = now;
        const bool all_edges_observed =
            std::all_of(visibility.begin(), visibility.end(),
                        [](double value) { return value > 0.0; });
        if (all_edges_observed) {
          if (startup_edges_ready_since_.isZero())
            startup_edges_ready_since_ = now;
        } else {
          startup_edges_ready_since_ = ros::Time();
        }
        const bool observed_ready =
            !startup_edges_ready_since_.isZero() &&
            (now - startup_edges_ready_since_).toSec() >=
                startup_edge_ready_persistence_;
        const bool timed_out =
            (now - startup_topology_gate_started_).toSec() >=
            startup_topology_gate_max_wait_;
        if (!observed_ready && !timed_out) {
          const double compute_time =
              (ros::WallTime::now() - start).toSec();
          publishMessages(scores, overlaps, clearances, visibility, "INTACT",
                          "INTACT", compute_time);
          ROS_INFO_STREAM_THROTTLE(
              0.5, "[6UAV_STARTUP_GATE] Holding INTACT while initial edges "
                       "become observed; elapsed="
                       << (now - startup_topology_gate_started_).toSec()
                       << "/" << startup_topology_gate_max_wait_ << " s");
          return;
        }
        startup_topology_gate_released_ = true;
        ROS_WARN_STREAM(
            "[6UAV_STARTUP_GATE] Released by "
            << (observed_ready ? "observed initial edges" : "timeout")
            << "; automatic split/merge is now active.");
      }

      updateEdgeStates(scores, clearances, center);
      const auto component_list = connectedComponents();
      const std::string components = componentText(component_list);
      const std::string state = displayState(component_list);
      const double compute_time = (ros::WallTime::now() - start).toSec();
      publishMessages(scores, overlaps, clearances, visibility, state,
                      components, compute_time);
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[six_uav_shadow] " << state << " min=" << std::fixed
                                    << std::setprecision(3)
                                    << *std::min_element(scores.begin(),
                                                        scores.end())
                                    << " components=" << components
                                    << " compute=" << compute_time << " s");
      if (last_safety_violation_)
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "At least one six-UAV key point is inside the inflated obstacle.");
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "Six-UAV StarConvex computation failed: " << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  ros::Subscriber obstacle_cloud_subscriber_;
  ros::Subscriber known_free_cloud_subscriber_;
  ros::Subscriber assignment_subscriber_;
  ros::Subscriber topology_command_subscriber_;
  ros::Publisher state_pub_;
  ros::Publisher score_pub_;
  ros::Publisher merge_score_pub_;
  ros::Publisher summary_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;
  std::array<Vec3, kUavCount> positions_{};
  std::array<ros::Time, kUavCount> odom_stamps_{};
  std::array<bool, kUavCount> odom_seen_{{false, false, false,
                                         false, false, false}};
  std::array<EdgeState, kEdgeCount> edge_states_{{
      EdgeState::KEEP, EdgeState::KEEP, EdgeState::KEEP, EdgeState::KEEP,
      EdgeState::KEEP, EdgeState::KEEP, EdgeState::KEEP}};
  std::array<double, kEdgeCount> pending_distances_{};
  std::array<bool, kEdgeCount> pending_seen_{};
  std::array<Vec3, kEdgeCount> pending_centers_{};
  GeometryOptions geometry_;
  std::string odom_suffix_;
  std::string frame_id_;
  std::string obstacle_cloud_topic_;
  std::string known_free_cloud_topic_;
  pcl::PointCloud<pcl::PointXYZ> latest_occupied_cloud_;
  pcl::PointCloud<pcl::PointXYZ> latest_known_free_cloud_;
  ros::Time occupied_source_stamp_;
  ros::Time known_free_source_stamp_;
  ros::Time map_update_receipt_stamp_;
  bool occupied_seen_{false};
  bool known_free_seen_{false};
  double map_timeout_{1.0};
  double map_sync_tolerance_{0.05};
  double map_warmup_duration_{2.0};
  double voxel_resolution_{0.10};
  double publish_rate_{1.0};
  double odom_timeout_{2.0};
  double obstacle_inflation_{0.20};
  double sample_xy_{0.25};
  double sample_z_{0.10};
  double narrow_clearance_{0.60};
  double narrow_weight_{0.45};
  double known_free_corridor_radius_{0.20};
  double split_threshold_{0.34};
  double hard_split_threshold_{0.20};
  double merge_threshold_{0.70};
  double distance_split_threshold_{1.80};
  double distance_merge_threshold_{1.70};
  double persistence_distance_{0.05};
  double edge_cut_min_hold_duration_{1.0};
  double edge_reconnect_persistence_{0.8};
  bool startup_topology_gate_enabled_{true};
  bool startup_topology_gate_released_{false};
  double startup_edge_ready_persistence_{0.5};
  double startup_topology_gate_max_wait_{3.0};
  ros::Time startup_topology_gate_started_;
  ros::Time startup_edges_ready_since_;
  bool rear_split_propagation_enabled_{true};
  double rear_split_propagation_factor_{1.3};
  double rear_split_propagation_speed_{1.0};
  double rear_split_propagation_min_time_{0.3};
  double rear_split_propagation_max_time_{6.0};
  std::array<ros::Time, kEdgeCount> rear_forced_cut_until_{};
  std::array<ros::Time, kEdgeCount> edge_cut_since_{};
  std::array<ros::Time, kEdgeCount> reconnect_candidate_since_{};
  bool map_seen_{false};
  ros::Time inputs_ready_since_;
  bool last_safety_violation_{false};
  uint32_t assignment_generation_{0};
  bool assignment_received_{false};
  std::unique_ptr<VoxelObstacleField> obstacle_field_;
  std::unique_ptr<VoxelObstacleField> narrow_obstacle_field_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "starconvex_shadow_6uav");
  try {
    SixUavShadow node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start six-UAV StarConvex criterion: "
                     << error.what());
    return 1;
  }
  return 0;
}

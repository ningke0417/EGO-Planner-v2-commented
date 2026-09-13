#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <starconvex_adapter/original_star_space.hpp>

namespace {

using starconvex_adapter::GeometryOptions;
using starconvex_adapter::OriginalStarSpace;
using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::norm;
using starconvex_adapter::subtract;

constexpr std::size_t kUavCount = 4;
constexpr std::size_t kEdgeCount = 4;
const std::array<std::array<int, 2>, kEdgeCount> kSlotEdges{
    {{{0, 1}}, {{1, 3}}, {{3, 2}}, {{2, 0}}}};
std::array<std::array<int, 2>, kEdgeCount> gEdges = kSlotEdges;

double planarDistance(const Vec3& first, const Vec3& second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}//计算 XY 平面距离

geometry_msgs::Point pointMessage(const Vec3& value) {
  geometry_msgs::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

class FourUavShadow {
 public:
  FourUavShadow() : private_nh_("~") {
    loadParameters();
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      const std::string topic = "/drone_" + std::to_string(uav) + "_" + odom_suffix_;
      odom_subscribers_[uav] = nh_.subscribe<nav_msgs::Odometry>(
          topic, 1, [this, uav](const nav_msgs::Odometry::ConstPtr& message) {
            odometryCallback(message, uav);
          });
      const std::string group_topic =
          "/formation_backend/drone_" + std::to_string(uav) + "_group";
      backend_group_subscribers_[uav] = nh_.subscribe<std_msgs::String>(
          group_topic, 1, [this, uav](const std_msgs::String::ConstPtr& message) {
            backendGroupCallback(message, uav);
          });
    }
    if (obstacle_model_ == "pointcloud") {
      obstacle_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          obstacle_cloud_topic_, 1, &FourUavShadow::obstacleCloudCallback,
          this);
    }
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &FourUavShadow::assignmentCallback, this);
    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
    score_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("scores", 1, true);
    merge_score_pub_ =
        private_nh_.advertise<std_msgs::Float64MultiArray>("merge_scores", 1, true);
    summary_pub_ = private_nh_.advertise<std_msgs::String>("summary", 1, true);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &FourUavShadow::timerCallback, this);
    ROS_INFO_STREAM("Four-UAV StarConvex shadow ready. obstacle_model="
                    << obstacle_model_
                    << ". It publishes suggestions only; it does not modify "
                       "EGO-Planner. split=" << split_threshold_
                    << ", merge=" << merge_threshold_
                    << ", persistence=" << persistence_distance_
                    << " m, shadow_front_merge_min_speed="
                    << merge_direction_min_speed_ << " m/s");
  }

	 private:
	  enum class EdgeState { KEEP, CUT };//每条边只有两种状态：KEEP 或 CUT

  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    if (assignment_locked_ || message->data.size() != kUavCount) return;
    std::array<int, kUavCount> drone_for_slot{{-1, -1, -1, -1}};
    for (std::size_t drone = 0; drone < kUavCount; ++drone) {
      const std::size_t slot = message->data[drone];
      if (slot >= kUavCount || drone_for_slot[slot] >= 0) {
        ROS_ERROR("[INITIAL_ASSIGNMENT] criterion rejected invalid permutation");
        return;
      }
      drone_for_slot[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      gEdges[edge] = {{drone_for_slot[kSlotEdges[edge][0]],
                       drone_for_slot[kSlotEdges[edge][1]]}};
    }
    assignment_locked_ = true;
    ROS_WARN("[INITIAL_ASSIGNMENT] criterion slot topology mapped to physical UAV IDs");
  }

  void loadParameters() {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param<std::string>("obstacle_model", obstacle_model_, "pillar");
    private_nh_.param<std::string>("obstacle_cloud_topic", obstacle_cloud_topic_,
                                   "/map_generator/global_cloud");
    private_nh_.param("voxel_resolution", voxel_resolution_, 0.10);
    private_nh_.param("publish_rate", publish_rate_, 1.0);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    private_nh_.param("pillar_x", pillar_x_, 0.0);
    private_nh_.param("pillar_y", pillar_y_, 0.0);
    private_nh_.param("pillar_radius", pillar_radius_, 1.0);
    private_nh_.param("obstacle_inflation", obstacle_inflation_, 0.2);
    private_nh_.param("sample_xy", sample_xy_, 0.25);
    private_nh_.param("sample_z", sample_z_, 0.10);
    private_nh_.param("narrow_clearance", narrow_clearance_, 0.60);
    private_nh_.param("narrow_weight", narrow_weight_, 0.45);
    private_nh_.param("split_threshold", split_threshold_, 0.34);
    private_nh_.param("hard_split_threshold", hard_split_threshold_, 0.20);
    private_nh_.param("merge_threshold", merge_threshold_, 0.75);
    private_nh_.param("merge_direction_min_speed",
                      merge_direction_min_speed_, 0.05);
    private_nh_.param("persistence_distance", persistence_distance_, 0.05);
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
    if ((obstacle_model_ != "pillar" && obstacle_model_ != "pointcloud") ||
        publish_rate_ <= 0.0 || pillar_radius_ <= 0.0 || sample_xy_ <= 0.0 ||
        sample_z_ <= 0.0 || geometry_.sensor_range <= 0.0 ||
        voxel_resolution_ <= 0.0 || merge_threshold_ <= split_threshold_ ||
        hard_split_threshold_ < 0.0 ||
        hard_split_threshold_ > split_threshold_ ||
        merge_direction_min_speed_ < 0.0 ||
        persistence_distance_ < 0.0 ||
        rear_split_propagation_factor_ <= 0.0 ||
        rear_split_propagation_speed_ <= 0.0 ||
        rear_split_propagation_min_time_ < 0.0 ||
        rear_split_propagation_max_time_ < rear_split_propagation_min_time_ ||
        geometry_.frontier_roof < geometry_.frontier_floor ||
        geometry_.delta_roof < geometry_.delta_floor) {
      throw std::runtime_error("invalid four-UAV StarConvex shadow parameters");
    }
  }

  void obstacleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    try {
      obstacle_field_.reset(new VoxelObstacleField(
          cloud, voxel_resolution_, obstacle_inflation_));
      narrow_obstacle_field_.reset(new VoxelObstacleField(
          cloud, voxel_resolution_, obstacle_inflation_ + narrow_clearance_));
      map_seen_ = true;
      ROS_INFO_STREAM("Point-cloud StarConvex field ready from "
                      << obstacle_cloud_topic_ << ": input_points="
                      << cloud.size() << ", safety_voxels="
                      << obstacle_field_->occupiedVoxelCount()
                      << ", narrow_voxels="
                      << narrow_obstacle_field_->occupiedVoxelCount());
    } catch (const std::exception& error) {
      map_seen_ = false;
      obstacle_field_.reset();
      narrow_obstacle_field_.reset();
      ROS_ERROR_STREAM("Unable to build point-cloud obstacle field: "
                       << error.what());
    }
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message,
                        std::size_t uav) {
    positions_[uav] = {message->pose.pose.position.x,
                       message->pose.pose.position.y,
                       message->pose.pose.position.z};
    velocities_[uav] = {message->twist.twist.linear.x,
                        message->twist.twist.linear.y,
                        message->twist.twist.linear.z};
    odom_stamps_[uav] = ros::Time::now();
    odom_seen_[uav] = true;
  }

  void backendGroupCallback(const std_msgs::String::ConstPtr& message,
                            std::size_t uav) {
    backend_groups_[uav] = message->data;
    backend_group_seen_[uav] = true;
  }

	  std::string backendState() const {
	    for (const bool seen : backend_group_seen_) {
	      if (!seen) return "WAITING";
	    }
    const bool intact = std::all_of(
        backend_groups_.begin(), backend_groups_.end(),
        [](const std::string& group) { return group == "INTACT"; });
    if (intact) return "INTACT";
	    std::ostringstream text;
	    text << "SPLIT";
	    for (std::size_t id = 0; id < kUavCount; ++id) {
	      text << (id == 0 ? " " : ",") << id << ":" << backend_groups_[id];
	    }
	    return text.str();
	  }

	  bool sameBackendGroup(int first, int second) const {
	    if (first < 0 || second < 0 ||
	        first >= static_cast<int>(kUavCount) ||
	        second >= static_cast<int>(kUavCount))
	      return false;
	    if (!backend_group_seen_[first] || !backend_group_seen_[second])
	      return false;
	    if (backend_groups_[first] == "INTACT" &&
	        backend_groups_[second] == "INTACT")
	      return true;
	    return backend_groups_[first] == backend_groups_[second];
	  }

  bool inputsReady() const {
    if (obstacle_model_ == "pointcloud" && !map_seen_) return false;
    const ros::Time now = ros::Time::now();
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      if (!odom_seen_[uav] || (now - odom_stamps_[uav]).toSec() > odom_timeout_) {
        return false;
      }
    }
    return true;
  }

  double visibilityWeight(const Vec3& first, const Vec3& second,
                          double* clearance) const {
    if (obstacle_model_ == "pointcloud") {
      const auto segment_blocked = [this, &first, &second](
                                       const VoxelObstacleField& field) {
        if (field.occupied(first) || field.occupied(second)) return true;
        const Vec3 hit = field.firstHit(first, second, voxel_resolution_);
        return norm(subtract(hit, first)) + 0.5 * voxel_resolution_ <
               norm(subtract(second, first));
      };
      if (segment_blocked(*obstacle_field_)) {
        *clearance = -voxel_resolution_;
        return 0.0;
      }
      if (segment_blocked(*narrow_obstacle_field_)) {
        *clearance = 0.5 * narrow_clearance_;
        return narrow_weight_;
      }
      *clearance = narrow_clearance_ + voxel_resolution_;
      return 1.0;
    }

    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    double projection = 0.0;
    if (length_squared > 1e-12) {
      projection = ((pillar_x_ - first.x) * dx +
                    (pillar_y_ - first.y) * dy) /
                   length_squared;
      projection = std::max(0.0, std::min(1.0, projection));
    }
    const double closest_x = first.x + projection * dx;
    const double closest_y = first.y + projection * dy;
    *clearance = std::hypot(closest_x - pillar_x_, closest_y - pillar_y_) -
                 (pillar_radius_ + obstacle_inflation_);
    if (*clearance <= 0.0) return 0.0;
    if (*clearance < narrow_clearance_) return narrow_weight_;
    return 1.0;
  }

  std::array<double, kEdgeCount> calculateScores(
      std::array<double, kEdgeCount>* overlap,
      std::array<double, kEdgeCount>* clearance,
      std::array<double, kEdgeCount>* visibility_weights,
      std::array<double, kEdgeCount>* front_overlap,
      std::array<double, kEdgeCount>* merge_scores,
      std::array<bool, kEdgeCount>* merge_direction_valid) {
    const double inflated_radius = pillar_radius_ + obstacle_inflation_;
    std::array<std::unique_ptr<OriginalStarSpace>, kUavCount> spaces;
    last_safety_violation_ = false;
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      const bool safety_violation =
          obstacle_model_ == "pointcloud"
              ? obstacle_field_->occupied(positions_[uav])
              : std::hypot(positions_[uav].x - pillar_x_,
                           positions_[uav].y - pillar_y_) <= inflated_radius;
      if (safety_violation) {
        // Offline calibration excludes these snapshots.  Online, represent an
        // invalid key point as empty free space instead of dropping the whole
        // criterion update with an exception.
        last_safety_violation_ = true;
        continue;
      }
      if (obstacle_model_ == "pointcloud") {
        spaces[uav].reset(new OriginalStarSpace(
            positions_[uav], starconvex_adapter::generateBoundaryCloud(
                                 positions_[uav], *obstacle_field_, geometry_)));
      } else {
        spaces[uav].reset(new OriginalStarSpace(
            positions_[uav], pillar_x_, pillar_y_, inflated_radius, geometry_));
      }
    }

    double minimum_x = positions_[0].x;
    double maximum_x = positions_[0].x;
    double minimum_y = positions_[0].y;
    double maximum_y = positions_[0].y;
    for (const auto& position : positions_) {
      minimum_x = std::min(minimum_x, position.x);
      maximum_x = std::max(maximum_x, position.x);
      minimum_y = std::min(minimum_y, position.y);
      maximum_y = std::max(maximum_y, position.y);
    }
    minimum_x -= geometry_.sensor_range;
    maximum_x += geometry_.sensor_range;
    minimum_y -= geometry_.sensor_range;
    maximum_y += geometry_.sensor_range;
    const double minimum_z = std::max(
        positions_[0].z + geometry_.delta_floor, geometry_.frontier_floor);
    const double maximum_z = std::min(
        positions_[0].z + geometry_.delta_roof, geometry_.frontier_roof);
    const int count_x = static_cast<int>(
        std::ceil((maximum_x - minimum_x) / sample_xy_));
    const int count_y = static_cast<int>(
        std::ceil((maximum_y - minimum_y) / sample_xy_));
    const int count_z = std::max(
        1, static_cast<int>(std::ceil((maximum_z - minimum_z) / sample_z_)));

    std::array<std::size_t, kUavCount> volume_counts{{0, 0, 0, 0}};
    std::array<std::size_t, kEdgeCount> intersection_counts{};
    std::array<std::array<std::size_t, 2>, kEdgeCount> front_volume_counts{};
    std::array<std::size_t, kEdgeCount> front_intersection_counts{};
    std::array<Vec3, kEdgeCount> pair_midpoints{};
    std::array<Vec3, kEdgeCount> merge_directions{};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const int first = gEdges[edge][0];
      const int second = gEdges[edge][1];
      pair_midpoints[edge] = {
          0.5 * (positions_[first].x + positions_[second].x),
          0.5 * (positions_[first].y + positions_[second].y),
          0.5 * (positions_[first].z + positions_[second].z)};
      const double velocity_x =
          0.5 * (velocities_[first].x + velocities_[second].x);
      const double velocity_y =
          0.5 * (velocities_[first].y + velocities_[second].y);
      const double speed = std::hypot(velocity_x, velocity_y);
      (*merge_direction_valid)[edge] = speed >= merge_direction_min_speed_;
      if ((*merge_direction_valid)[edge]) {
        merge_directions[edge] = {velocity_x / speed, velocity_y / speed, 0.0};
      }
    }
    for (int ix = 0; ix < count_x; ++ix) {
      const double x = minimum_x + (ix + 0.5) * sample_xy_;
      for (int iy = 0; iy < count_y; ++iy) {
        const double y = minimum_y + (iy + 0.5) * sample_xy_;
        for (int iz = 0; iz < count_z; ++iz) {
          const double z = minimum_z + (iz + 0.5) * sample_z_;
          std::array<bool, kUavCount> inside{{false, false, false, false}};
          for (std::size_t uav = 0; uav < kUavCount; ++uav) {
            inside[uav] = spaces[uav] && spaces[uav]->contains({x, y, z});
            volume_counts[uav] += static_cast<std::size_t>(inside[uav]);
          }
          for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
            intersection_counts[edge] += static_cast<std::size_t>(
                inside[gEdges[edge][0]] && inside[gEdges[edge][1]]);
            if (!(*merge_direction_valid)[edge]) continue;
            const double forward_projection =
                (x - pair_midpoints[edge].x) * merge_directions[edge].x +
                (y - pair_midpoints[edge].y) * merge_directions[edge].y;
            if (forward_projection < 0.0) continue;
            const bool first_inside = inside[gEdges[edge][0]];
            const bool second_inside = inside[gEdges[edge][1]];
            front_volume_counts[edge][0] +=
                static_cast<std::size_t>(first_inside);
            front_volume_counts[edge][1] +=
                static_cast<std::size_t>(second_inside);
            front_intersection_counts[edge] +=
                static_cast<std::size_t>(first_inside && second_inside);
          }
        }
      }
    }

    std::array<double, kEdgeCount> scores{};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const std::size_t first_count = volume_counts[gEdges[edge][0]];
      const std::size_t second_count = volume_counts[gEdges[edge][1]];
      const std::size_t smaller = std::min(first_count, second_count);
      (*overlap)[edge] = smaller
                             ? static_cast<double>(intersection_counts[edge]) /
                                   static_cast<double>(smaller)
                             : 0.0;
      const double weight = visibilityWeight(
          positions_[gEdges[edge][0]], positions_[gEdges[edge][1]],
          &(*clearance)[edge]);
      (*visibility_weights)[edge] = weight;
      scores[edge] = (*overlap)[edge] * weight;
      const std::size_t front_smaller =
          std::min(front_volume_counts[edge][0], front_volume_counts[edge][1]);
      (*front_overlap)[edge] =
          (*merge_direction_valid)[edge] && front_smaller
              ? static_cast<double>(front_intersection_counts[edge]) /
                    static_cast<double>(front_smaller)
              : 0.0;
      (*merge_scores)[edge] = (*front_overlap)[edge] * weight;
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

	  void resetEdgePending(std::size_t edge) {
	    pending_distances_[edge] = 0.0;
	    pending_has_previous_[edge] = false;
	  }

  double rearForcedRemaining(const ros::Time& now) const {
    if (rear_forced_cut_until_.isZero() || rear_forced_cut_until_ <= now) {
      return 0.0;
    }
    return (rear_forced_cut_until_ - now).toSec();
  }

  bool rearForcedCutActive(const ros::Time& now) const {
    return rearForcedRemaining(now) > 0.0;
  }

  void triggerRearSplitPropagation(const ros::Time& now) {
    if (!rear_split_propagation_enabled_) return;
    constexpr std::size_t kRearEdge = 2;  // Slot edge 2 is the rear row.
    const double front_rear_distance =
        planarDistance(positions_[0], positions_[2]);
    const double raw_duration =
        front_rear_distance / rear_split_propagation_speed_ *
        rear_split_propagation_factor_;
    const double duration =
        std::min(rear_split_propagation_max_time_,
                 std::max(rear_split_propagation_min_time_, raw_duration));
    const ros::Time until = now + ros::Duration(duration);
    if (rear_forced_cut_until_.isZero() || until > rear_forced_cut_until_) {
      rear_forced_cut_until_ = until;
    }
    edge_states_[kRearEdge] = EdgeState::CUT;
    resetEdgePending(kRearEdge);
  }
	
	  void updateEdgeStates(const std::array<double, kEdgeCount>& scores,
	                        const std::array<double, kEdgeCount>& clearance,
	                        const Vec3& center) {
    const ros::Time now = ros::Time::now();
    const bool front_was_keep = edge_states_[0] == EdgeState::KEEP;
	    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (edge == 2 && rearForcedCutActive(now)) {
        edge_states_[edge] = EdgeState::CUT;
        resetEdgePending(edge);
        continue;
      }

	      if (edge_states_[edge] == EdgeState::KEEP &&
	          (scores[edge] <= hard_split_threshold_ || clearance[edge] <= 0.0)) {
	        edge_states_[edge] = EdgeState::CUT;
	        resetEdgePending(edge);
	        continue;
	      }
	
	      const bool should_toggle =
	          edge_states_[edge] == EdgeState::KEEP
	              ? scores[edge] <= split_threshold_
	              : scores[edge] >= merge_threshold_;
	      if (!should_toggle) {
	        resetEdgePending(edge);
	        continue;
	      }
	      if (pending_has_previous_[edge]) {
	        pending_distances_[edge] +=
	            planarDistance(center, pending_previous_centers_[edge]);
	      }
	      pending_previous_centers_[edge] = center;
	      pending_has_previous_[edge] = true;
	      if (pending_distances_[edge] + 1e-9 >= persistence_distance_) {
	        edge_states_[edge] = edge_states_[edge] == EdgeState::KEEP
	                                  ? EdgeState::CUT
	                                  : EdgeState::KEEP;
	        resetEdgePending(edge);
	      }
	    }

    if (front_was_keep && edge_states_[0] == EdgeState::CUT) {
      triggerRearSplitPropagation(now);
    }
	  }
	
	  std::array<int, kUavCount> componentParents() const {
	    std::array<int, kUavCount> parent{{0, 1, 2, 3}};
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
	      if (edge_states_[edge] == EdgeState::KEEP) {
	        unite(gEdges[edge][0], gEdges[edge][1]);
	      }
	    }
	    for (int id = 0; id < static_cast<int>(kUavCount); ++id) {
	      parent[id] = find_root(id);
	    }
	    return parent;
	  }
	
	  std::vector<std::vector<int>> connectedComponents() const {
	    const auto parent = componentParents();
	    std::vector<std::vector<int>> components;
	    std::array<int, kUavCount> component_index{{-1, -1, -1, -1}};
	    for (int id = 0; id < static_cast<int>(kUavCount); ++id) {
	      const int root = parent[id];
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
	      if (component > 0) text << "|";
	      text << "{";
	      for (std::size_t idx = 0; idx < components[component].size(); ++idx) {
	        if (idx > 0) text << ",";
	        text << components[component][idx];
	      }
	      text << "}";
	    }
	    return text.str();
	  }
	
	  std::string displayState(
	      const std::vector<std::vector<int>>& components) const {
	    if (components.size() > 1) return "SPLIT";
	    const bool any_cut = std::any_of(
	        edge_states_.begin(), edge_states_.end(),
	        [](EdgeState state) { return state == EdgeState::CUT; });
	    if (any_cut) return "PARTIAL_CUT";
	    const bool any_pending = std::any_of(
	        pending_has_previous_.begin(), pending_has_previous_.end(),
	        [](bool pending) { return pending; });
	    return any_pending ? "WATCH" : "INTACT";
	  }
	
	  std_msgs::ColorRGBA edgeColor(double score, EdgeState edge_state,
	                                bool pending) const {
	    std_msgs::ColorRGBA color;
	    color.a = 1.0;
	    if (edge_state == EdgeState::CUT) {
	      color.r = 1.0;
	      color.g = 0.05;
	    } else if (pending || score <= split_threshold_) {
	      color.r = 1.0;
	      color.g = 0.65;
    } else {
      color.g = 0.9;
      color.b = 0.2;
    }
    return color;
  }

	  void publishMarkers(const std::array<double, kEdgeCount>& scores,
	                      const std::array<double, kEdgeCount>& merge_scores,
	                      const std::array<bool, kEdgeCount>& merge_direction_valid,
	                      const std::string& display_state,
	                      const std::string& components_text) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 1;
	    const std::string backend_state = backendState();
	    const bool backend_intact = backend_state == "INTACT";
	    const bool backend_split = backend_state.find("SPLIT") == 0;
	    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
	      const bool cut = edge_states_[edge] == EdgeState::CUT;
	      visualization_msgs::Marker line;
	      line.header.frame_id = frame_id_;
	      line.header.stamp = ros::Time::now();
	      line.ns = "four_uav_shadow_edges";
	      line.id = id++;
	      line.type = visualization_msgs::Marker::LINE_LIST;
	      line.action = visualization_msgs::Marker::ADD;
	      line.scale.x = 0.07;
	      line.color = edgeColor(scores[edge], edge_states_[edge],
	                             pending_has_previous_[edge]);
	      if (!cut) {
	        line.points.push_back(pointMessage(positions_[gEdges[edge][0]]));
	        line.points.push_back(pointMessage(positions_[gEdges[edge][1]]));
	        array.markers.push_back(line);
	      }
	
	      visualization_msgs::Marker label;
	      label.header = line.header;
      label.ns = "four_uav_shadow_scores";
      label.id = id++;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.scale.z = 0.22;
      label.color = line.color;
      label.pose.orientation.w = 1.0;
      label.pose.position.x = 0.5 * (positions_[gEdges[edge][0]].x +
                                     positions_[gEdges[edge][1]].x);
      label.pose.position.y = 0.5 * (positions_[gEdges[edge][0]].y +
                                     positions_[gEdges[edge][1]].y);
	      label.pose.position.z = 0.5 * (positions_[gEdges[edge][0]].z +
	                                     positions_[gEdges[edge][1]].z) + 0.25;
	      std::ostringstream text;
      if (cut && edge == 2 && rearForcedCutActive(ros::Time::now())) {
        text << "PROP_CUT ";
      } else if (cut) text << "CUT ";
	      else if (pending_has_previous_[edge]) text << "WATCH ";
	      text << gEdges[edge][0] << '-' << gEdges[edge][1] << " S="
	           << std::fixed << std::setprecision(2) << scores[edge] << " M=";
      if (merge_direction_valid[edge]) {
        text << merge_scores[edge];
      } else {
        text << "NA";
      }
	      label.text = text.str();
	      array.markers.push_back(label);
    }

    if (backend_intact || backend_split) {
      visualization_msgs::Marker backend_edges;
      backend_edges.header.frame_id = frame_id_;
      backend_edges.header.stamp = ros::Time::now();
      backend_edges.ns = "four_uav_backend_edges";
      backend_edges.id = id++;
      backend_edges.type = visualization_msgs::Marker::LINE_LIST;
      backend_edges.action = visualization_msgs::Marker::ADD;
      backend_edges.scale.x = 0.035;
      backend_edges.color.a = 1.0;
      backend_edges.color.g = 0.9;
      backend_edges.color.b = 1.0;
	      for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
	        if (backend_split &&
	            !sameBackendGroup(gEdges[edge][0], gEdges[edge][1]))
	          continue;
	        backend_edges.points.push_back(pointMessage(positions_[gEdges[edge][0]]));
	        backend_edges.points.push_back(pointMessage(positions_[gEdges[edge][1]]));
	      }
      array.markers.push_back(backend_edges);
    }

    visualization_msgs::Marker state_label;
    state_label.header.frame_id = frame_id_;
    state_label.header.stamp = ros::Time::now();
    state_label.ns = "four_uav_shadow_state";
    state_label.id = id++;
    state_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    state_label.action = visualization_msgs::Marker::ADD;
    state_label.pose.orientation.w = 1.0;
    const Vec3 center = formationCenter();
    state_label.pose.position = pointMessage({center.x, center.y, center.z + 0.8});
    state_label.scale.z = 0.35;
    state_label.color.a = 1.0;
	    state_label.color.r = display_state == "SPLIT" ? 1.0 : 0.1;
	    state_label.color.g = display_state == "SPLIT" ? 0.1 : 0.9;
	    state_label.color.b = 0.2;
	    state_label.text =
	        "CRITERION " + display_state + "  components=" + components_text;
    const double rear_remaining = rearForcedRemaining(ros::Time::now());
    if (rear_remaining > 0.0) {
      std::ostringstream propagation_text;
      propagation_text << "\nFRONT->REAR hold 2-3 "
                       << std::fixed << std::setprecision(1)
                       << rear_remaining << "s";
      state_label.text += propagation_text.str();
    }
	    array.markers.push_back(state_label);
	    marker_pub_.publish(array);
	  }

	  void publishMessages(const std::array<double, kEdgeCount>& scores,
	                       const std::array<double, kEdgeCount>& overlap,
	                       const std::array<double, kEdgeCount>& clearance,
	                       const std::array<double, kEdgeCount>& visibility_weights,
	                       const std::array<double, kEdgeCount>& front_overlap,
	                       const std::array<double, kEdgeCount>& merge_scores,
	                       const std::array<bool, kEdgeCount>& merge_direction_valid,
	                       const std::string& display_state,
	                       const std::string& components_text,
	                       double compute_time) {
    std_msgs::String state_message;
    state_message.data = display_state;
    state_pub_.publish(state_message);

    std_msgs::Float64MultiArray score_message;
    score_message.data.assign(scores.begin(), scores.end());
    score_pub_.publish(score_message);

    std_msgs::Float64MultiArray merge_score_message;
    merge_score_message.data.assign(merge_scores.begin(), merge_scores.end());
    merge_score_pub_.publish(merge_score_message);

	    std::ostringstream summary;
	    summary << std::fixed << std::setprecision(4)
	            << "{\"state\":\"" << display_state
	            << "\",\"obstacle_model\":\"" << obstacle_model_
	            << "\",\"components\":\"" << components_text << "\""
	            << ",\"center_x\":" << formationCenter().x
	            << ",\"center_y\":" << formationCenter().y
            << ",\"score_01\":" << scores[0]
            << ",\"score_13\":" << scores[1]
            << ",\"score_32\":" << scores[2]
            << ",\"score_20\":" << scores[3]
	            << ",\"merge_score_01\":" << merge_scores[0]
	            << ",\"merge_score_13\":" << merge_scores[1]
	            << ",\"merge_score_32\":" << merge_scores[2]
	            << ",\"merge_score_20\":" << merge_scores[3]
	            << ",\"distance_01\":"
              << planarDistance(positions_[0], positions_[1])
	            << ",\"distance_13\":"
              << planarDistance(positions_[1], positions_[3])
	            << ",\"distance_32\":"
              << planarDistance(positions_[3], positions_[2])
	            << ",\"distance_20\":"
              << planarDistance(positions_[2], positions_[0])
	            << ",\"front_overlap_01\":" << front_overlap[0]
	            << ",\"front_overlap_13\":" << front_overlap[1]
	            << ",\"front_overlap_32\":" << front_overlap[2]
	            << ",\"front_overlap_20\":" << front_overlap[3]
	            << ",\"merge_direction_valid_01\":"
              << (merge_direction_valid[0] ? "true" : "false")
	            << ",\"merge_direction_valid_13\":"
              << (merge_direction_valid[1] ? "true" : "false")
	            << ",\"merge_direction_valid_32\":"
              << (merge_direction_valid[2] ? "true" : "false")
	            << ",\"merge_direction_valid_20\":"
              << (merge_direction_valid[3] ? "true" : "false")
	            << ",\"merge_score_shadow_only\":true"
	            // Legacy aliases retained for old bags and analysis scripts.
            << ",\"score_12\":" << scores[1]
            << ",\"score_23\":" << scores[2]
            << ",\"score_30\":" << scores[3]
	            << ",\"cut_edges\":\"";
	    bool first_cut = true;
	    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
	      if (edge_states_[edge] != EdgeState::CUT) continue;
	      if (!first_cut) summary << ",";
	      first_cut = false;
	      summary << gEdges[edge][0] << "-" << gEdges[edge][1];
	    }
	    summary << "\""
	            << ",\"cut_score\":"
	            << *std::min_element(scores.begin(), scores.end())
	            << ",\"overlap_01\":" << overlap[0]
	            << ",\"overlap_13\":" << overlap[1]
	            << ",\"overlap_32\":" << overlap[2]
	            << ",\"overlap_20\":" << overlap[3]
	            << ",\"clearance_01\":" << clearance[0]
	            << ",\"clearance_13\":" << clearance[1]
	            << ",\"clearance_32\":" << clearance[2]
	            << ",\"clearance_20\":" << clearance[3]
	            << ",\"visibility_weight_01\":" << visibility_weights[0]
	            << ",\"visibility_weight_13\":" << visibility_weights[1]
	            << ",\"visibility_weight_32\":" << visibility_weights[2]
	            << ",\"visibility_weight_20\":" << visibility_weights[3]
	            // Legacy aliases retained for old bags and analysis scripts.
	            << ",\"overlap_12\":" << overlap[1]
	            << ",\"overlap_30\":" << overlap[3]
	            << ",\"clearance_12\":" << clearance[1]
	            << ",\"clearance_30\":" << clearance[3]
	            << ",\"pending_distance_01\":" << pending_distances_[0]
	            << ",\"pending_distance_13\":" << pending_distances_[1]
	            << ",\"pending_distance_32\":" << pending_distances_[2]
	            << ",\"pending_distance_20\":" << pending_distances_[3]
	            // Legacy aliases retained for old bags and analysis scripts.
	            << ",\"pending_distance_12\":" << pending_distances_[1]
	            << ",\"pending_distance_23\":" << pending_distances_[2]
	            << ",\"pending_distance_30\":" << pending_distances_[3]
            << ",\"rear_propagation_remaining\":"
            << rearForcedRemaining(ros::Time::now())
	            << ",\"compute_seconds\":" << compute_time
            << ",\"safety_region_violation\":"
            << (last_safety_violation_ ? "true" : "false")
            << ",\"planner_feedback\":false}";
	    std_msgs::String summary_message;
	    summary_message.data = summary.str();
	    summary_pub_.publish(summary_message);
	    publishMarkers(scores, merge_scores, merge_direction_valid,
                   display_state, components_text);
	  }

  void timerCallback(const ros::TimerEvent&) {
    if (!inputsReady()) {
      ROS_WARN_THROTTLE(
          2.0,
          "Four-UAV shadow is waiting for four fresh odometry topics and its obstacle map.");
      return;
    }
    const ros::WallTime start = ros::WallTime::now();
    try {
      std::array<double, kEdgeCount> overlap{};
      std::array<double, kEdgeCount> clearance{};
      std::array<double, kEdgeCount> visibility_weights{};
      std::array<double, kEdgeCount> front_overlap{};
      std::array<double, kEdgeCount> merge_scores{};
      std::array<bool, kEdgeCount> merge_direction_valid{};
	      const auto scores = calculateScores(
          &overlap, &clearance, &visibility_weights, &front_overlap, &merge_scores,
          &merge_direction_valid);
	      const Vec3 center = formationCenter();
	      updateEdgeStates(scores, clearance, center);
	      const auto components = connectedComponents();
	      const std::string components_text = componentText(components);
	      const std::string display_state = displayState(components);
	      const double elapsed = (ros::WallTime::now() - start).toSec();
	      publishMessages(scores, overlap, clearance, visibility_weights, front_overlap,
                      merge_scores, merge_direction_valid, display_state,
                      components_text, elapsed);
      if (last_safety_violation_) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "At least one UAV key point is inside the inflated obstacle; "
                 "its StarConvex free space is treated as empty.");
      }
	      ROS_INFO_STREAM_THROTTLE(
	          1.0, "[four_uav_shadow] " << display_state << " cut_min="
	                                    << std::fixed << std::setprecision(3)
	                                    << *std::min_element(scores.begin(),
	                                                         scores.end())
	                                    << " components=" << components_text
	                                    << " center=(" << center.x << ","
	                                    << center.y << ") compute=" << elapsed << " s");
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(2.0, "Four-UAV StarConvex computation failed: "
                                         << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  std::array<ros::Subscriber, kUavCount> backend_group_subscribers_;
  ros::Subscriber obstacle_cloud_subscriber_;
  ros::Subscriber assignment_subscriber_;
  ros::Publisher state_pub_;
  ros::Publisher score_pub_;
  ros::Publisher merge_score_pub_;
  ros::Publisher summary_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;
  std::array<Vec3, kUavCount> positions_;
  std::array<Vec3, kUavCount> velocities_;
  std::array<ros::Time, kUavCount> odom_stamps_;
  std::array<bool, kUavCount> odom_seen_{{false, false, false, false}};
  std::array<std::string, kUavCount> backend_groups_;
  std::array<bool, kUavCount> backend_group_seen_{{false, false, false, false}};
  GeometryOptions geometry_;
  std::string odom_suffix_;
  std::string frame_id_;
  std::string obstacle_model_ = "pillar";
  std::string obstacle_cloud_topic_ = "/map_generator/global_cloud";
  double publish_rate_ = 1.0;
  double odom_timeout_ = 2.0;
  double pillar_x_ = 0.0;
  double pillar_y_ = 0.0;
  double pillar_radius_ = 1.0;
  double obstacle_inflation_ = 0.2;
  double voxel_resolution_ = 0.10;
  double sample_xy_ = 0.25;
  double sample_z_ = 0.10;
  double narrow_clearance_ = 0.60;
  double narrow_weight_ = 0.45;
	  double split_threshold_ = 0.34;
	  double hard_split_threshold_ = 0.20;
	  double merge_threshold_ = 0.75;
	  double merge_direction_min_speed_ = 0.05;
	  double persistence_distance_ = 0.05;
  bool rear_split_propagation_enabled_ = true;
  double rear_split_propagation_factor_ = 1.3;
  double rear_split_propagation_speed_ = 1.0;
  double rear_split_propagation_min_time_ = 0.3;
  double rear_split_propagation_max_time_ = 6.0;
  ros::Time rear_forced_cut_until_;
	  std::array<EdgeState, kEdgeCount> edge_states_{
	      {EdgeState::KEEP, EdgeState::KEEP, EdgeState::KEEP, EdgeState::KEEP}};
	  std::array<double, kEdgeCount> pending_distances_{{0.0, 0.0, 0.0, 0.0}};
	  std::array<bool, kEdgeCount> pending_has_previous_{
	      {false, false, false, false}};
	  std::array<Vec3, kEdgeCount> pending_previous_centers_;
  bool last_safety_violation_ = false;
  bool map_seen_ = false;
  bool assignment_locked_ = false;
  std::unique_ptr<VoxelObstacleField> obstacle_field_;
  std::unique_ptr<VoxelObstacleField> narrow_obstacle_field_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "starconvex_shadow_4uav");
  try {
    FourUavShadow node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start four-UAV StarConvex shadow: " << error.what());
    return 1;
  }
  return 0;
}

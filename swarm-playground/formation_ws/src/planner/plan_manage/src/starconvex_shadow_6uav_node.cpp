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

constexpr std::size_t kUavCount = 6;
constexpr std::size_t kEdgeCount = 7;
// The existing six-UAV formation is a 2x3 grid.  Only nearest-neighbour
// formation constraints participate in topology decisions.
const std::array<std::array<int, 2>, kEdgeCount> kEdges{{
    {{0, 1}}, {{1, 2}}, {{3, 4}}, {{4, 5}},
    {{0, 3}}, {{1, 4}}, {{2, 5}}}};

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
  return std::to_string(kEdges[edge][0]) + std::to_string(kEdges[edge][1]);
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
        << " persistence=" << persistence_distance_
        << " m front_to_rear_propagation="
        << (rear_split_propagation_enabled_ ? "enabled" : "disabled")
        << " factor=" << rear_split_propagation_factor_
        << " speed=" << rear_split_propagation_speed_ << " m/s");
  }

 private:
  enum class EdgeState { KEEP, CUT };

  void loadParameters() {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param<std::string>("obstacle_cloud_topic",
                                   obstacle_cloud_topic_,
                                   "/map_generator/global_cloud");
    private_nh_.param("voxel_resolution", voxel_resolution_, 0.10);
    private_nh_.param("publish_rate", publish_rate_, 1.0);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    private_nh_.param("obstacle_inflation", obstacle_inflation_, 0.20);
    private_nh_.param("sample_xy", sample_xy_, 0.25);
    private_nh_.param("sample_z", sample_z_, 0.10);
    private_nh_.param("narrow_clearance", narrow_clearance_, 0.60);
    private_nh_.param("narrow_weight", narrow_weight_, 0.45);
    private_nh_.param("split_threshold", split_threshold_, 0.34);
    private_nh_.param("hard_split_threshold", hard_split_threshold_, 0.20);
    private_nh_.param("merge_threshold", merge_threshold_, 0.70);
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
    if (publish_rate_ <= 0.0 || voxel_resolution_ <= 0.0 ||
        obstacle_inflation_ < 0.0 || sample_xy_ <= 0.0 || sample_z_ <= 0.0 ||
        narrow_clearance_ < 0.0 || narrow_weight_ < 0.0 ||
        narrow_weight_ > 1.0 || split_threshold_ < 0.0 ||
        merge_threshold_ <= split_threshold_ || merge_threshold_ > 1.0 ||
        hard_split_threshold_ < 0.0 ||
        hard_split_threshold_ > split_threshold_ ||
        persistence_distance_ < 0.0 ||
        rear_split_propagation_factor_ <= 0.0 ||
        rear_split_propagation_speed_ <= 0.0 ||
        rear_split_propagation_min_time_ < 0.0 ||
        rear_split_propagation_max_time_ <
            rear_split_propagation_min_time_ ||
        geometry_.sensor_range <= 0.0 ||
        geometry_.frontier_roof < geometry_.frontier_floor ||
        geometry_.delta_roof < geometry_.delta_floor) {
      throw std::runtime_error("invalid six-UAV StarConvex parameters");
    }
  }

  void obstacleCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    try {
      obstacle_field_.reset(new VoxelObstacleField(
          cloud, voxel_resolution_, obstacle_inflation_));
      narrow_obstacle_field_.reset(new VoxelObstacleField(
          cloud, voxel_resolution_, obstacle_inflation_ + narrow_clearance_));
      map_seen_ = true;
      ROS_INFO_STREAM("Six-UAV obstacle fields ready: points=" << cloud.size()
                      << " safety_voxels="
                      << obstacle_field_->occupiedVoxelCount()
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
    if (field.occupied(first) || field.occupied(second)) return true;
    const Vec3 hit = field.firstHit(first, second, voxel_resolution_);
    return norm(subtract(hit, first)) + 0.5 * voxel_resolution_ <
           norm(subtract(second, first));
  }

  double visibilityWeight(const Vec3& first, const Vec3& second,
                          double* clearance) const {
    if (segmentBlocked(*obstacle_field_, first, second)) {
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
      if (obstacle_field_->occupied(positions_[uav])) {
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
            inside[uav] = spaces[uav] && spaces[uav]->contains({x, y, z});
            volumes[uav] += static_cast<std::size_t>(inside[uav]);
          }
          for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
            intersections[edge] += static_cast<std::size_t>(
                inside[kEdges[edge][0]] && inside[kEdges[edge][1]]);
          }
        }
      }
    }

    std::array<double, kEdgeCount> scores{};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const std::size_t smaller =
          std::min(volumes[kEdges[edge][0]], volumes[kEdges[edge][1]]);
      (*overlaps)[edge] =
          smaller ? static_cast<double>(intersections[edge]) /
                        static_cast<double>(smaller)
                  : 0.0;
      (*visibility_weights)[edge] = visibilityWeight(
          positions_[kEdges[edge][0]], positions_[kEdges[edge][1]],
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
        0.5 * (positions_[kEdges[first_edge][0]].x +
               positions_[kEdges[first_edge][1]].x),
        0.5 * (positions_[kEdges[first_edge][0]].y +
               positions_[kEdges[first_edge][1]].y),
        0.0};
    const Vec3 second_midpoint{
        0.5 * (positions_[kEdges[second_edge][0]].x +
               positions_[kEdges[second_edge][1]].x),
        0.5 * (positions_[kEdges[second_edge][0]].y +
               positions_[kEdges[second_edge][1]].y),
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
    edge_states_[rear_edge] = EdgeState::CUT;
    resetPending(rear_edge);
    ROS_WARN_STREAM("[6UAV_SPLIT_PROPAGATION] Front edge "
                    << kEdges[front_edge][0] << "-"
                    << kEdges[front_edge][1] << " cut; force rear edge "
                    << kEdges[rear_edge][0] << "-"
                    << kEdges[rear_edge][1] << " cut for " << duration
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
      if ((edge == 2 || edge == 3) && rearForcedCutActive(edge, now)) {
        edge_states_[edge] = EdgeState::CUT;
        resetPending(edge);
        continue;
      }
      if (edge_states_[edge] == EdgeState::KEEP &&
          (scores[edge] <= hard_split_threshold_ ||
           clearances[edge] <= 0.0)) {
        edge_states_[edge] = EdgeState::CUT;
        resetPending(edge);
        continue;
      }
      const bool candidate = edge_states_[edge] == EdgeState::KEEP
                                 ? scores[edge] <= split_threshold_
                                 : scores[edge] >= merge_threshold_;
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
        edge_states_[edge] = edge_states_[edge] == EdgeState::KEEP
                                 ? EdgeState::CUT
                                 : EdgeState::KEEP;
        resetPending(edge);
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
        unite(kEdges[edge][0], kEdges[edge][1]);
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
    int marker_id = 1;
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      visualization_msgs::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = ros::Time::now();
      line.ns = "six_uav_criterion_edges";
      line.id = marker_id++;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.06;
      line.color.a = 1.0;
      const bool cut = edge_states_[edge] == EdgeState::CUT;
      line.color.r = cut ? 1.0 : 0.1;
      line.color.g = cut ? 0.05 : 0.9;
      if (!cut) {
        line.points.push_back(pointMessage(positions_[kEdges[edge][0]]));
        line.points.push_back(pointMessage(positions_[kEdges[edge][1]]));
        output.markers.push_back(line);
      }
      visualization_msgs::Marker label;
      label.header = line.header;
      label.ns = "six_uav_criterion_scores";
      label.id = marker_id++;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.scale.z = 0.18;
      label.color = line.color;
      label.pose.orientation.w = 1.0;
      label.pose.position.x = 0.5 * (positions_[kEdges[edge][0]].x +
                                     positions_[kEdges[edge][1]].x);
      label.pose.position.y = 0.5 * (positions_[kEdges[edge][0]].y +
                                     positions_[kEdges[edge][1]].y);
      label.pose.position.z = 0.5 * (positions_[kEdges[edge][0]].z +
                                     positions_[kEdges[edge][1]].z) + 0.22;
      std::ostringstream text;
      if (cut && (edge == 2 || edge == 3) &&
          rearForcedCutActive(edge, ros::Time::now())) {
        text << "PROP_CUT ";
      } else if (cut) {
        text << "CUT ";
      }
      text << kEdges[edge][0] << "-" << kEdges[edge][1] << " S="
           << std::fixed << std::setprecision(2) << scores[edge];
      label.text = text.str();
      output.markers.push_back(label);
    }
    visualization_msgs::Marker label;
    label.header.frame_id = frame_id_;
    label.header.stamp = ros::Time::now();
    label.ns = "six_uav_criterion_state";
    label.id = marker_id;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.orientation.w = 1.0;
    const Vec3 center = formationCenter();
    label.pose.position = pointMessage({center.x, center.y, center.z + 0.8});
    label.scale.z = 0.32;
    label.color.a = 1.0;
    label.color.r = state == "SPLIT" ? 1.0 : 0.1;
    label.color.g = state == "SPLIT" ? 0.1 : 0.9;
    label.text = "6UAV CRITERION " + state + " components=" + components;
    const ros::Time now = ros::Time::now();
    const double rear_34_remaining = rearForcedRemaining(2, now);
    const double rear_45_remaining = rearForcedRemaining(3, now);
    if (rear_34_remaining > 0.0 || rear_45_remaining > 0.0) {
      std::ostringstream propagation;
      propagation << "\nFRONT->REAR 01->34=" << std::fixed
                  << std::setprecision(1) << rear_34_remaining
                  << "s 12->45=" << rear_45_remaining << "s";
      label.text += propagation.str();
    }
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
          planarDistance(positions_[kEdges[edge][0]], positions_[kEdges[edge][1]]));
    }
    const Vec3 center = formationCenter();
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(4)
            << "{\"state\":\"" << state << "\",\"components\":\""
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
              << planarDistance(positions_[kEdges[edge][0]],
                                positions_[kEdges[edge][1]])
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
      ROS_WARN_THROTTLE(
          2.0,
          "Six-UAV criterion is waiting for the map and six fresh odometry topics.");
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
  double voxel_resolution_{0.10};
  double publish_rate_{1.0};
  double odom_timeout_{2.0};
  double obstacle_inflation_{0.20};
  double sample_xy_{0.25};
  double sample_z_{0.10};
  double narrow_clearance_{0.60};
  double narrow_weight_{0.45};
  double split_threshold_{0.34};
  double hard_split_threshold_{0.20};
  double merge_threshold_{0.70};
  double persistence_distance_{0.05};
  bool rear_split_propagation_enabled_{true};
  double rear_split_propagation_factor_{1.3};
  double rear_split_propagation_speed_{1.0};
  double rear_split_propagation_min_time_{0.3};
  double rear_split_propagation_max_time_{6.0};
  std::array<ros::Time, kEdgeCount> rear_forced_cut_until_{};
  bool map_seen_{false};
  bool last_safety_violation_{false};
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

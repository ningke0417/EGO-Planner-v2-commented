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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <starconvex_adapter/original_star_space.hpp>

namespace {

using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::norm;
using starconvex_adapter::subtract;

constexpr std::size_t kUavCount = 4;
constexpr std::size_t kGroupCount = 2;
constexpr double kPi = 3.14159265358979323846;
const std::array<std::array<int, 2>, kGroupCount> kGroupMembers{
    {{{0, 1}}, {{2, 3}}}};

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double angleDistance(double first, double second) {
  return std::fabs(wrapAngle(first - second));
}

geometry_msgs::Point pointMessage(double x, double y, double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

struct Candidate {
  int ray = -1;
  double angle = 0.0;
  double common_radius = 0.0;
  double usable_length = 0.0;
  double score = -std::numeric_limits<double>::infinity();
};

struct GroupIntent {
  int group = -1;
  Vec3 centroid;
  double goal_angle = 0.0;
  double direct_common_radius = 0.0;
  Candidate local_best;
  Candidate selected;
  bool angle_limit_applied = false;
  bool emergency_turn = false;
};

class SubgroupRouteIntent {
 public:
  SubgroupRouteIntent() : private_nh_("~") {
    loadParameters();
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      const std::string odom_topic =
          "/drone_" + std::to_string(uav) + "_" + odom_suffix_;
      odom_subscribers_[uav] = nh_.subscribe<nav_msgs::Odometry>(
          odom_topic, 1,
          [this, uav](const nav_msgs::Odometry::ConstPtr& message) {
            positions_[uav] = {message->pose.pose.position.x,
                               message->pose.pose.position.y,
                               message->pose.pose.position.z};
            odom_stamps_[uav] = ros::Time::now();
            odom_seen_[uav] = true;
          });

      const std::string group_topic =
          "/formation_backend/drone_" + std::to_string(uav) + "_group";
      group_subscribers_[uav] = nh_.subscribe<std_msgs::String>(
          group_topic, 1,
          [this, uav](const std_msgs::String::ConstPtr& message) {
            backend_groups_[uav] = message->data;
            backend_group_seen_[uav] = true;
          });
    }
    obstacle_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        obstacle_cloud_topic_, 1,
        &SubgroupRouteIntent::obstacleCallback, this);
    intent_publisher_ = private_nh_.advertise<std_msgs::Float64MultiArray>(
        "intents", 1, true);
    summary_publisher_ =
        private_nh_.advertise<std_msgs::String>("summary", 1, true);
    marker_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("markers", 1,
                                                               true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &SubgroupRouteIntent::timerCallback, this);
    ROS_INFO_STREAM(
        "Four-UAV subgroup route-intent ready. mode="
        << (feedback_enabled_ ? "GUIDANCE_ENABLED" : "SHADOW_ONLY")
        << ", candidates=" << ray_count_ << ", goal=(" << goal_x_ << ","
        << goal_y_ << "), rate=" << publish_rate_ << " Hz");
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param<std::string>("obstacle_cloud_topic",
                                   obstacle_cloud_topic_,
                                   "/map_generator/global_cloud");
    private_nh_.param("voxel_resolution", voxel_resolution_, 0.10);
    private_nh_.param("obstacle_inflation", obstacle_inflation_, 0.35);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    private_nh_.param("publish_rate", publish_rate_, 5.0);
    private_nh_.param("goal_x", goal_x_, 6.0);
    private_nh_.param("goal_y", goal_y_, 0.0);
    private_nh_.param("sensor_range", sensor_range_, 5.0);
    private_nh_.param("raycast_resolution", raycast_resolution_, 0.10);
    private_nh_.param("ray_count", ray_count_, 72);
    private_nh_.param("guide_lookahead", guide_lookahead_, 2.0);
    private_nh_.param("guide_clearance", guide_clearance_, 0.15);
    private_nh_.param("forward_half_angle", forward_half_angle_, 1.5708);
    private_nh_.param("weight_goal", weight_goal_, 0.55);
    private_nh_.param("weight_clearance", weight_clearance_, 0.35);
    private_nh_.param("weight_continuity", weight_continuity_, 0.15);
    private_nh_.param("weight_lateral", weight_lateral_, 0.20);
    private_nh_.param("coordination_trigger_distance",
                      coordination_trigger_distance_, 2.15);
    private_nh_.param("coordination_conflict_penalty",
                      coordination_conflict_penalty_, 1.0);
    private_nh_.param("coordination_side_deadband",
                      coordination_side_deadband_, 0.12);
    private_nh_.param("coordination_min_angle", coordination_min_angle_,
                      0.35);
    private_nh_.param("feedback_enabled", feedback_enabled_, false);
    private_nh_.param("lock_release_distance", lock_release_distance_, 3.0);
    private_nh_.param("lock_release_persistence",
                      lock_release_persistence_, 1.0);
    private_nh_.param("lock_min_duration", lock_min_duration_, 2.0);
    private_nh_.param("locked_max_angle_step", locked_max_angle_step_,
                      0.261799);
    private_nh_.param("angle_limit_min_usable_length",
                      angle_limit_min_usable_length_, 1.0);
    private_nh_.param("release_fade_duration", release_fade_duration_, 1.5);
    private_nh_.param("show_debug_text", show_debug_text_, false);

    if (voxel_resolution_ <= 0.0 || obstacle_inflation_ < 0.0 ||
        odom_timeout_ <= 0.0 || publish_rate_ <= 0.0 ||
        sensor_range_ <= 0.0 || raycast_resolution_ <= 0.0 ||
        ray_count_ < 8 || guide_lookahead_ <= 0.0 ||
        guide_clearance_ < 0.0 || forward_half_angle_ <= 0.0 ||
        forward_half_angle_ > kPi || coordination_trigger_distance_ <= 0.0 ||
        coordination_conflict_penalty_ < 0.0 ||
        coordination_side_deadband_ < 0.0 ||
        coordination_min_angle_ < 0.0 || lock_release_distance_ <= 0.0 ||
        lock_release_persistence_ < 0.0 || lock_min_duration_ < 0.0 ||
        locked_max_angle_step_ <= 0.0 || locked_max_angle_step_ > kPi ||
        angle_limit_min_usable_length_ < 0.0 ||
        release_fade_duration_ < 0.0) {
      throw std::runtime_error("invalid subgroup route-intent parameters");
    }
  }

  void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    try {
      obstacle_field_.reset(new VoxelObstacleField(
          cloud, voxel_resolution_, obstacle_inflation_));
      map_seen_ = true;
      ROS_INFO_STREAM("Route-intent obstacle field ready: input_points="
                      << cloud.size() << ", inflated_voxels="
                      << obstacle_field_->occupiedVoxelCount());
    } catch (const std::exception& error) {
      obstacle_field_.reset();
      map_seen_ = false;
      ROS_ERROR_STREAM("Unable to build route-intent obstacle field: "
                       << error.what());
    }
  }

	  bool backendSplit() const {
	    for (const bool seen : backend_group_seen_) {
	      if (!seen) return false;
	    }
	    return backend_groups_[0] == "G0" && backend_groups_[1] == "G0" &&
	           backend_groups_[2] == "G1" && backend_groups_[3] == "G1";
	  }

	  bool backendAnySplit() const {
	    for (const bool seen : backend_group_seen_) {
	      if (!seen) return false;
	    }
	    return std::any_of(backend_groups_.begin(), backend_groups_.end(),
	                       [](const std::string& group) {
	                         return group != "INTACT";
	                       });
	  }

	  bool backendLegacySplit() const {
	    return backendSplit();
	  }

  bool inputsReady() const {
    if (!map_seen_ || !obstacle_field_) return false;
    const ros::Time now = ros::Time::now();
    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
      if (!odom_seen_[uav] ||
          (now - odom_stamps_[uav]).toSec() > odom_timeout_) {
        return false;
      }
    }
    return true;
  }

  Vec3 groupCentroid(std::size_t group) const {
    Vec3 centroid;
    for (const int member : kGroupMembers[group]) {
      centroid.x += 0.5 * positions_[member].x;
      centroid.y += 0.5 * positions_[member].y;
      centroid.z += 0.5 * positions_[member].z;
    }
    return centroid;
  }

  Vec3 swarmCentroid() const {
    Vec3 centroid;
    for (const auto& position : positions_) {
      centroid.x += position.x / static_cast<double>(kUavCount);
      centroid.y += position.y / static_cast<double>(kUavCount);
      centroid.z += position.z / static_cast<double>(kUavCount);
    }
    return centroid;
  }

  double freeDistance(int uav, double angle) const {
    const Vec3& origin = positions_[uav];
    const Vec3 endpoint{origin.x + sensor_range_ * std::cos(angle),
                        origin.y + sensor_range_ * std::sin(angle), origin.z};
    const Vec3 hit = obstacle_field_->firstHit(
        origin, endpoint, raycast_resolution_);
    return std::min(sensor_range_, norm(subtract(hit, origin)));
  }

	  double commonFreeDistance(std::size_t group, double angle) const {
	    double common = sensor_range_;
	    for (const int member : kGroupMembers[group]) {
	      common = std::min(common, freeDistance(member, angle));
	    }
	    return common;
	  }

	  Candidate bestCandidateForDrone(std::size_t uav) const {
	    Candidate best;
	    const double goal_angle =
	        std::atan2(goal_y_ - positions_[uav].y, goal_x_ - positions_[uav].x);
	    const double previous_angle = previous_drone_angle_valid_[uav]
	                                      ? previous_drone_angles_[uav]
	                                      : goal_angle;
	    for (int ray = 0; ray < ray_count_; ++ray) {
	      Candidate candidate;
	      candidate.ray = ray;
	      candidate.angle = 2.0 * kPi * static_cast<double>(ray) /
	                        static_cast<double>(ray_count_);
	      const double goal_error = angleDistance(candidate.angle, goal_angle);
	      if (goal_error > forward_half_angle_) continue;
	      candidate.common_radius = freeDistance(static_cast<int>(uav),
	                                             candidate.angle);
	      candidate.usable_length =
	          std::max(0.0, candidate.common_radius - guide_clearance_);
	      const double progress = std::cos(goal_error);
	      const double clearance_score =
	          clamp01(candidate.usable_length / sensor_range_);
	      const double continuity =
	          angleDistance(candidate.angle, previous_angle) / kPi;
	      candidate.score = weight_goal_ * progress +
	                        weight_clearance_ * clearance_score -
	                        weight_continuity_ * continuity;
	      if (candidate.score > best.score) best = candidate;
	    }
	    if (best.ray < 0) {
	      throw std::runtime_error("route-intent drone has no forward candidate");
	    }
	    return best;
	  }

  int lateralSide(double angle, double reference_angle) const {
    const double lateral = std::sin(wrapAngle(angle - reference_angle));
    if (lateral > coordination_side_deadband_) return 1;
    if (lateral < -coordination_side_deadband_) return -1;
    return 0;
  }

  std::array<std::vector<Candidate>, kGroupCount> buildCandidates(
      std::array<GroupIntent, kGroupCount>* intents) const {
    std::array<std::vector<Candidate>, kGroupCount> candidates;
    const Vec3 swarm_center = swarmCentroid();
    const double swarm_goal_angle =
        std::atan2(goal_y_ - swarm_center.y, goal_x_ - swarm_center.x);
    const Vec3 goal_lateral{-std::sin(swarm_goal_angle),
                            std::cos(swarm_goal_angle), 0.0};

    for (std::size_t group = 0; group < kGroupCount; ++group) {
      GroupIntent& intent = (*intents)[group];
      intent.group = static_cast<int>(group);
      intent.centroid = groupCentroid(group);
      intent.goal_angle = std::atan2(goal_y_ - intent.centroid.y,
                                     goal_x_ - intent.centroid.x);
      intent.direct_common_radius =
          commonFreeDistance(group, intent.goal_angle);
      const double previous_angle = previous_angle_valid_[group]
                                        ? previous_angles_[group]
                                        : intent.goal_angle;
      const Vec3 centroid_offset = subtract(intent.centroid, swarm_center);
      const double lateral_offset =
          centroid_offset.x * goal_lateral.x +
          centroid_offset.y * goal_lateral.y;
      const double desired_lateral_sign =
          lateral_offset > 1e-3 ? 1.0 : (lateral_offset < -1e-3 ? -1.0 : 0.0);

      for (int ray = 0; ray < ray_count_; ++ray) {
        Candidate candidate;
        candidate.ray = ray;
        candidate.angle =
            2.0 * kPi * static_cast<double>(ray) /
            static_cast<double>(ray_count_);
        const double goal_error =
            angleDistance(candidate.angle, intent.goal_angle);
        if (goal_error > forward_half_angle_) continue;

        candidate.common_radius = commonFreeDistance(group, candidate.angle);
        candidate.usable_length =
            std::max(0.0, candidate.common_radius - guide_clearance_);
        const double progress = std::cos(goal_error);
        const double clearance_score =
            clamp01(candidate.usable_length / sensor_range_);
        const double continuity =
            angleDistance(candidate.angle, previous_angle) / kPi;
        const double candidate_lateral =
            std::sin(wrapAngle(candidate.angle - swarm_goal_angle));
        const double lateral_score =
            desired_lateral_sign * candidate_lateral;
        candidate.score = weight_goal_ * progress +
                          weight_clearance_ * clearance_score -
                          weight_continuity_ * continuity +
                          weight_lateral_ * lateral_score;
        candidates[group].push_back(candidate);
        if (candidate.score > intent.local_best.score) {
          intent.local_best = candidate;
        }
      }
      if (candidates[group].empty()) {
        throw std::runtime_error("route-intent group has no forward candidate");
      }
    }
    return candidates;
  }

  bool pairConflict(const Candidate& first, const Candidate& second,
                    double reference_angle) const {
    const int first_side = lateralSide(first.angle, reference_angle);
    const int second_side = lateralSide(second.angle, reference_angle);
    if (first_side == 0 || second_side == 0 || first_side == second_side) {
      return true;
    }
    return angleDistance(first.angle, second.angle) < coordination_min_angle_;
  }

  Candidate bestCandidateForSide(const std::vector<Candidate>& candidates,
                                 int side,
                                 double reference_angle) const {
    Candidate best;
    for (const Candidate& candidate : candidates) {
      if (lateralSide(candidate.angle, reference_angle) != side) continue;
      if (candidate.score > best.score) best = candidate;
    }
    return best;
  }

  Candidate rateLimitedCandidateForSide(
      const std::vector<Candidate>& candidates, int side,
      double reference_angle, std::size_t group, bool* limit_applied,
      bool* emergency_turn) const {
    const Candidate unrestricted =
        bestCandidateForSide(candidates, side, reference_angle);
    *limit_applied = false;
    *emergency_turn = false;
    if (unrestricted.ray < 0 || !previous_angle_valid_[group] ||
        angleDistance(unrestricted.angle, previous_angles_[group]) <=
            locked_max_angle_step_) {
      return unrestricted;
    }

    Candidate limited;
    for (const Candidate& candidate : candidates) {
      if (lateralSide(candidate.angle, reference_angle) != side) continue;
      if (angleDistance(candidate.angle, previous_angles_[group]) >
          locked_max_angle_step_)
        continue;
      if (candidate.usable_length < angle_limit_min_usable_length_) continue;
      if (candidate.score > limited.score) limited = candidate;
    }
    if (limited.ray >= 0) {
      *limit_applied = true;
      return limited;
    }

    // Never synthesize an unrated intermediate direction.  If no ray inside
    // the normal angular step remains sufficiently clear, use the best
    // raycast-certified direction on the locked side immediately.
    *emergency_turn = true;
    return unrestricted;
  }

  double feedbackGain() const {
    if (!feedback_enabled_ || !route_locked_) return 0.0;
    if (release_fade_started_.isZero() || release_fade_duration_ <= 0.0)
      return 1.0;
    return clamp01(1.0 -
                   (ros::Time::now() - release_fade_started_).toSec() /
                       release_fade_duration_);
  }

  void resetRouteLock() {
    route_locked_ = false;
    locked_sides_ = {{0, 0}};
    lock_started_ = ros::Time();
	    release_candidate_started_ = ros::Time();
	    release_fade_started_ = ros::Time();
	    previous_angle_valid_ = {{false, false}};
	    previous_drone_angle_valid_ = {{false, false, false, false}};
	  }

  std::array<GroupIntent, kGroupCount> computeIntents(
      bool* coordination_active, bool* conflict_avoided) {
    std::array<GroupIntent, kGroupCount> intents;
    const auto candidates = buildCandidates(&intents);
    const bool obstacle_requires_coordination =
        intents[0].direct_common_radius < coordination_trigger_distance_ ||
        intents[1].direct_common_radius < coordination_trigger_distance_;
    const Vec3 swarm_center = swarmCentroid();
    const double reference_angle =
        std::atan2(goal_y_ - swarm_center.y, goal_x_ - swarm_center.x);
    *conflict_avoided = false;

    // Release only after both members of both groups have had a clear direct
    // route toward the final goal for a persistent interval.  Then fade the
    // optimizer feedback instead of dropping its weight from full to zero in
    // one update.  If clearance is lost during the fade, restore full weight.
    if (route_locked_) {
      const ros::Time now = ros::Time::now();
      const bool minimum_lock_elapsed =
          (now - lock_started_).toSec() >= lock_min_duration_;
      const bool all_direct_routes_clear =
          intents[0].direct_common_radius >= lock_release_distance_ &&
          intents[1].direct_common_radius >= lock_release_distance_;
      if (!release_fade_started_.isZero()) {
        if (!all_direct_routes_clear) {
          ROS_WARN("[route_intent] Direct route lost during release fade; restoring full guidance.");
          release_fade_started_ = ros::Time();
          release_candidate_started_ = ros::Time();
        } else if (release_fade_duration_ <= 0.0 ||
                   (now - release_fade_started_).toSec() >=
                       release_fade_duration_) {
          ROS_WARN("[route_intent] Release fade completed; unlocking subgroup sides.");
          route_locked_ = false;
          locked_sides_ = {{0, 0}};
          release_candidate_started_ = ros::Time();
          release_fade_started_ = ros::Time();
        }
      } else if (minimum_lock_elapsed && all_direct_routes_clear) {
        if (release_candidate_started_.isZero())
          release_candidate_started_ = now;
        if ((now - release_candidate_started_).toSec() >=
            lock_release_persistence_) {
          ROS_WARN("[route_intent] Direct goal visibility persisted; fading subgroup guidance.");
          release_fade_started_ = now;
        }
      } else {
        release_candidate_started_ = ros::Time();
      }
    }

    // At the first obstructed instant, choose the best non-conflicting pair
    // and lock only its two homotopy-side signs.  Angles remain free to evolve
    // with local visibility, so this is not a hard-coded wall route.
    if (!route_locked_ && obstacle_requires_coordination) {
      double best_joint_score = -std::numeric_limits<double>::infinity();
      Candidate best_first;
      Candidate best_second;
      bool best_conflict = true;
      for (const Candidate& first : candidates[0]) {
        for (const Candidate& second : candidates[1]) {
          const bool conflict = pairConflict(first, second, reference_angle);
          const double joint_score =
              first.score + second.score -
              (conflict ? coordination_conflict_penalty_ : 0.0);
          const bool better = joint_score > best_joint_score + 1e-12;
          const bool tie = std::fabs(joint_score - best_joint_score) <= 1e-12;
          const bool deterministic_tie_break =
              tie && (first.ray < best_first.ray ||
                      (first.ray == best_first.ray &&
                       second.ray < best_second.ray));
          if (better || deterministic_tie_break) {
            best_joint_score = joint_score;
            best_first = first;
            best_second = second;
            best_conflict = conflict;
          }
        }
      }
      *conflict_avoided = !best_conflict;
      if (!best_conflict) {
        const int first_side = lateralSide(best_first.angle, reference_angle);
        const int second_side = lateralSide(best_second.angle, reference_angle);
        if (first_side != 0 && second_side != 0 &&
            first_side != second_side) {
          route_locked_ = true;
          locked_sides_ = {{first_side, second_side}};
          lock_started_ = ros::Time::now();
          release_candidate_started_ = ros::Time();
          release_fade_started_ = ros::Time();
          ROS_WARN("[route_intent] Locked autonomous subgroup sides: G0=%d G1=%d",
                   first_side, second_side);
        }
      }
    }

    if (route_locked_) {
      for (std::size_t group = 0; group < kGroupCount; ++group) {
        const Candidate locked_best = rateLimitedCandidateForSide(
            candidates[group], locked_sides_[group], reference_angle, group,
            &intents[group].angle_limit_applied,
            &intents[group].emergency_turn);
        intents[group].selected =
            locked_best.ray >= 0 ? locked_best : intents[group].local_best;
        if (intents[group].emergency_turn) {
          ROS_WARN_THROTTLE(
              1.0,
              "[route_intent] G%zu has no safe ray inside the normal angle step; emergency turn allowed.",
              group);
        }
      }
      *conflict_avoided = true;
    } else {
      for (std::size_t group = 0; group < kGroupCount; ++group)
        intents[group].selected = intents[group].local_best;
    }

    *coordination_active = route_locked_ || obstacle_requires_coordination;

    for (std::size_t group = 0; group < kGroupCount; ++group) {
      previous_angles_[group] = intents[group].selected.angle;
      previous_angle_valid_[group] = true;
    }
    return intents;
  }

  std_msgs::ColorRGBA groupColor(std::size_t group, double alpha = 1.0) const {
    std_msgs::ColorRGBA color;
    color.a = alpha;
    if (group == 0) {
      color.g = 0.85;
      color.b = 1.0;
    } else {
      color.r = 0.95;
      color.b = 0.9;
    }
    return color;
  }

  void publishClear(const std::string& state) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    marker_publisher_.publish(markers);

    std_msgs::String summary;
    summary.data = "{\"mode\":\"" +
                   std::string(feedback_enabled_ ? "GUIDANCE_ENABLED"
                                                 : "SHADOW_ONLY") +
                   "\",\"state\":\"" + state + "\"}";
    summary_publisher_.publish(summary);
  }

	  void publish(const std::array<GroupIntent, kGroupCount>& intents,
	               bool coordination_active, bool conflict_avoided) {
    const ros::Time stamp = ros::Time::now();
    const double feedback_gain = feedbackGain();
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    std_msgs::Float64MultiArray values;

    int marker_id = 1;
    for (std::size_t group = 0; group < kGroupCount; ++group) {
      const GroupIntent& intent = intents[group];
      const double arrow_length =
          std::min(guide_lookahead_, intent.selected.usable_length);

      visualization_msgs::Marker local_arrow;
      local_arrow.header.frame_id = frame_id_;
      local_arrow.header.stamp = stamp;
      local_arrow.ns = "route_intent_local_best";
      local_arrow.id = marker_id++;
      local_arrow.type = visualization_msgs::Marker::ARROW;
      local_arrow.action = visualization_msgs::Marker::ADD;
      local_arrow.scale.x = 0.035;
      local_arrow.scale.y = 0.10;
      local_arrow.scale.z = 0.10;
      local_arrow.color = groupColor(group, 0.35);
      local_arrow.points.push_back(pointMessage(
          intent.centroid.x, intent.centroid.y, intent.centroid.z + 0.45));
      local_arrow.points.push_back(pointMessage(
          intent.centroid.x +
              std::min(guide_lookahead_, intent.local_best.usable_length) *
                  std::cos(intent.local_best.angle),
          intent.centroid.y +
              std::min(guide_lookahead_, intent.local_best.usable_length) *
                  std::sin(intent.local_best.angle),
          intent.centroid.z + 0.45));
      markers.markers.push_back(local_arrow);

      visualization_msgs::Marker selected_arrow = local_arrow;
      selected_arrow.ns = "route_intent_coordinated";
      selected_arrow.id = marker_id++;
      selected_arrow.scale.x = 0.10;
      selected_arrow.scale.y = 0.22;
      selected_arrow.scale.z = 0.22;
      selected_arrow.color = groupColor(group);
      selected_arrow.points.clear();
      selected_arrow.points.push_back(pointMessage(
          intent.centroid.x, intent.centroid.y, intent.centroid.z + 0.65));
      selected_arrow.points.push_back(pointMessage(
          intent.centroid.x + arrow_length * std::cos(intent.selected.angle),
          intent.centroid.y + arrow_length * std::sin(intent.selected.angle),
          intent.centroid.z + 0.65));
      markers.markers.push_back(selected_arrow);

      if (show_debug_text_) {
        visualization_msgs::Marker label;
        label.header = selected_arrow.header;
        label.ns = "route_intent_text";
        label.id = marker_id++;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.orientation.w = 1.0;
        label.pose.position = pointMessage(
            intent.centroid.x, intent.centroid.y, intent.centroid.z + 1.0);
        label.scale.z = 0.24;
        label.color = groupColor(group);
        std::ostringstream text;
        text << (feedback_enabled_ ? "GUIDE " : "SHADOW ") << "G" << group
             << (!release_fade_started_.isZero()
                     ? " FADE"
                     : (route_locked_ ? " LOCK" : " FREE"))
             << " gain=" << std::fixed << std::setprecision(2) << feedback_gain
             << " angle="
             << std::setprecision(1)
             << intent.selected.angle * 180.0 / kPi << "deg free="
             << std::setprecision(2) << intent.selected.common_radius
             << " score=" << intent.selected.score;
        label.text = text.str();
        markers.markers.push_back(label);
      }

      values.data.push_back(static_cast<double>(group));
      values.data.push_back(static_cast<double>(kGroupMembers[group][0]));
      values.data.push_back(static_cast<double>(kGroupMembers[group][1]));
      values.data.push_back(intent.centroid.x);
      values.data.push_back(intent.centroid.y);
      values.data.push_back(intent.selected.angle);
      values.data.push_back(intent.selected.common_radius);
      values.data.push_back(intent.selected.usable_length);
      values.data.push_back(intent.selected.score);
      values.data.push_back(coordination_active ? 1.0 : 0.0);
      values.data.push_back(route_locked_ ? 1.0 : 0.0);
      values.data.push_back(static_cast<double>(locked_sides_[group]));
      // This field was originally a Boolean feedback_active flag.  Keeping
      // the same 13-value schema while publishing a [0,1] gain preserves old
      // bag readers (gain > 0 means active) and enables a smooth release.
      values.data.push_back(feedback_gain);
	  }

	    std::ostringstream summary;
    summary << std::fixed << std::setprecision(4)
            << "{\"mode\":\""
            << (feedback_enabled_ ? "GUIDANCE_ENABLED" : "SHADOW_ONLY")
            << "\",\"state\":\"ACTIVE\""
            << ",\"coordination_active\":"
            << (coordination_active ? "true" : "false")
            << ",\"conflict_avoided\":"
            << (conflict_avoided ? "true" : "false")
            << ",\"route_locked\":"
            << (route_locked_ ? "true" : "false")
            << ",\"release_fading\":"
            << (!release_fade_started_.isZero() ? "true" : "false")
            << ",\"feedback_gain\":" << feedback_gain
            << ",\"groups\":[";
    for (std::size_t group = 0; group < kGroupCount; ++group) {
      if (group > 0) summary << ',';
      const GroupIntent& intent = intents[group];
      summary << "{\"id\":" << group << ",\"members\":["
              << kGroupMembers[group][0] << ',' << kGroupMembers[group][1]
              << "],\"centroid\":[" << intent.centroid.x << ','
              << intent.centroid.y << "],\"local_angle\":"
              << intent.local_best.angle << ",\"selected_angle\":"
              << intent.selected.angle << ",\"common_radius\":"
              << intent.selected.common_radius << ",\"usable_length\":"
              << intent.selected.usable_length << ",\"score\":"
              << intent.selected.score << ",\"locked_side\":"
              << locked_sides_[group] << ",\"feedback_active\":"
              << (feedback_gain > 0.0 ? "true" : "false")
              << ",\"feedback_gain\":" << feedback_gain
              << ",\"angle_limit_applied\":"
              << (intent.angle_limit_applied ? "true" : "false")
              << ",\"emergency_turn\":"
              << (intent.emergency_turn ? "true" : "false")
              << '}';
    }
    summary << "]}";

    std_msgs::String summary_message;
    summary_message.data = summary.str();
    marker_publisher_.publish(markers);
    intent_publisher_.publish(values);
	    summary_publisher_.publish(summary_message);
	    ROS_INFO_STREAM_THROTTLE(1.0, "[route_intent] " << summary.str());
	  }

	  void publishDynamicPerDrone() {
	    const ros::Time stamp = ros::Time::now();
	    visualization_msgs::MarkerArray markers;
	    visualization_msgs::Marker clear;
	    clear.action = visualization_msgs::Marker::DELETEALL;
	    markers.markers.push_back(clear);
	    std_msgs::Float64MultiArray values;
	    int marker_id = 1;
	
	    for (std::size_t uav = 0; uav < kUavCount; ++uav) {
	      const Candidate selected = bestCandidateForDrone(uav);
	      previous_drone_angles_[uav] = selected.angle;
	      previous_drone_angle_valid_[uav] = true;
	      const double gain =
	          feedback_enabled_ && selected.common_radius < lock_release_distance_
	              ? 1.0
	              : 0.0;
	      const double arrow_length =
	          std::min(guide_lookahead_, selected.usable_length);
	
	      visualization_msgs::Marker arrow;
	      arrow.header.frame_id = frame_id_;
	      arrow.header.stamp = stamp;
	      arrow.ns = "route_intent_dynamic_drone";
	      arrow.id = marker_id++;
	      arrow.type = visualization_msgs::Marker::ARROW;
	      arrow.action = visualization_msgs::Marker::ADD;
	      arrow.scale.x = 0.09;
	      arrow.scale.y = 0.20;
	      arrow.scale.z = 0.20;
	      arrow.color = groupColor(uav);
	      arrow.points.push_back(pointMessage(
	          positions_[uav].x, positions_[uav].y, positions_[uav].z + 0.75));
	      arrow.points.push_back(pointMessage(
	          positions_[uav].x + arrow_length * std::cos(selected.angle),
	          positions_[uav].y + arrow_length * std::sin(selected.angle),
	          positions_[uav].z + 0.75));
	      markers.markers.push_back(arrow);
	
      if (show_debug_text_) {
        visualization_msgs::Marker label;
        label.header = arrow.header;
        label.ns = "route_intent_dynamic_text";
        label.id = marker_id++;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.orientation.w = 1.0;
        label.pose.position = pointMessage(
            positions_[uav].x, positions_[uav].y, positions_[uav].z + 1.1);
        label.scale.z = 0.22;
        label.color = arrow.color;
        std::ostringstream text;
        text << (feedback_enabled_ ? "GUIDE " : "SHADOW ") << "D" << uav
             << " gain=" << std::fixed << std::setprecision(2) << gain
             << " angle=" << std::setprecision(1)
             << selected.angle * 180.0 / kPi << "deg free="
             << std::setprecision(2) << selected.common_radius
             << " score=" << selected.score;
        label.text = text.str();
        markers.markers.push_back(label);
      }
	
	      values.data.push_back(static_cast<double>(uav));
	      values.data.push_back(static_cast<double>(uav));
	      values.data.push_back(static_cast<double>(uav));
	      values.data.push_back(positions_[uav].x);
	      values.data.push_back(positions_[uav].y);
	      values.data.push_back(selected.angle);
	      values.data.push_back(selected.common_radius);
	      values.data.push_back(selected.usable_length);
	      values.data.push_back(selected.score);
	      values.data.push_back(1.0);
	      values.data.push_back(1.0);
	      values.data.push_back(0.0);
	      values.data.push_back(gain);
	    }
	
	    std::ostringstream summary;
	    summary << std::fixed << std::setprecision(4)
	            << "{\"mode\":\""
	            << (feedback_enabled_ ? "GUIDANCE_ENABLED" : "SHADOW_ONLY")
	            << "\",\"state\":\"ACTIVE_DYNAMIC_PER_DRONE\""
	            << ",\"records\":4}";
	    std_msgs::String summary_message;
	    summary_message.data = summary.str();
	    marker_publisher_.publish(markers);
	    intent_publisher_.publish(values);
	    summary_publisher_.publish(summary_message);
	    ROS_INFO_STREAM_THROTTLE(1.0, "[route_intent] " << summary.str());
	  }

	  void timerCallback(const ros::TimerEvent&) {
    if (!inputsReady()) {
      ROS_WARN_THROTTLE(
          2.0, "Route-intent shadow is waiting for map and fresh odometry.");
      publishClear("WAITING_INPUT");
      return;
    }
	    if (!backendAnySplit()) {
	      resetRouteLock();
	      publishClear("WAITING_BACKEND_SPLIT");
	      return;
	    }
	    try {
	      if (!backendLegacySplit()) {
	        resetRouteLock();
	        publishDynamicPerDrone();
	        return;
	      }
	      bool coordination_active = false;
      bool conflict_avoided = false;
      const auto intents =
          computeIntents(&coordination_active, &conflict_avoided);
      publish(intents, coordination_active, conflict_avoided);
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(2.0,
                                "Route-intent computation failed: "
                                    << error.what());
      publishClear("ERROR");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  std::array<ros::Subscriber, kUavCount> group_subscribers_;
  ros::Subscriber obstacle_subscriber_;
  ros::Publisher intent_publisher_;
  ros::Publisher summary_publisher_;
  ros::Publisher marker_publisher_;
  ros::Timer timer_;
  std::array<Vec3, kUavCount> positions_;
  std::array<ros::Time, kUavCount> odom_stamps_;
  std::array<bool, kUavCount> odom_seen_{{false, false, false, false}};
	  std::array<std::string, kUavCount> backend_groups_;
	  std::array<bool, kUavCount> backend_group_seen_{{false, false, false, false}};
	  std::array<double, kGroupCount> previous_angles_{{0.0, 0.0}};
	  std::array<bool, kGroupCount> previous_angle_valid_{{false, false}};
	  std::array<double, kUavCount> previous_drone_angles_{{0.0, 0.0, 0.0, 0.0}};
	  std::array<bool, kUavCount> previous_drone_angle_valid_{
	      {false, false, false, false}};
  std::unique_ptr<VoxelObstacleField> obstacle_field_;
  bool map_seen_ = false;
  std::string odom_suffix_ = "visual_slam/odom";
  std::string frame_id_ = "world";
  std::string obstacle_cloud_topic_ = "/map_generator/global_cloud";
  double voxel_resolution_ = 0.10;
  double obstacle_inflation_ = 0.35;
  double odom_timeout_ = 2.0;
  double publish_rate_ = 5.0;
  double goal_x_ = 6.0;
  double goal_y_ = 0.0;
  double sensor_range_ = 5.0;
  double raycast_resolution_ = 0.10;
  int ray_count_ = 72;
  double guide_lookahead_ = 2.0;
  double guide_clearance_ = 0.15;
  double forward_half_angle_ = 1.5708;
  double weight_goal_ = 0.55;
  double weight_clearance_ = 0.35;
  double weight_continuity_ = 0.15;
  double weight_lateral_ = 0.20;
  double coordination_trigger_distance_ = 2.15;
  double coordination_conflict_penalty_ = 1.0;
  double coordination_side_deadband_ = 0.12;
  double coordination_min_angle_ = 0.35;
  bool feedback_enabled_ = false;
  bool show_debug_text_ = false;
  bool route_locked_ = false;
  std::array<int, kGroupCount> locked_sides_{{0, 0}};
  ros::Time lock_started_;
  ros::Time release_candidate_started_;
  ros::Time release_fade_started_;
  double lock_release_distance_ = 3.0;
  double lock_release_persistence_ = 1.0;
  double lock_min_duration_ = 2.0;
  double locked_max_angle_step_ = 0.261799;
  double angle_limit_min_usable_length_ = 1.0;
  double release_fade_duration_ = 1.5;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "subgroup_route_intent_4uav");
  try {
    SubgroupRouteIntent node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start subgroup route-intent shadow: "
                     << error.what());
    return 1;
  }
  return 0;
}

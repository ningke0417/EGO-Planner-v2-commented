#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

struct Box {
  double center_x;
  double center_y;
  double size_x;
  double size_y;
  bool large;
  std::string kind;
};

double sample(std::mt19937* generator, double lower, double upper) {
  return std::uniform_real_distribution<double>(lower, upper)(*generator);
}

bool overlaps(const Box& candidate, const Box& existing,
              double separation) {
  return std::abs(candidate.center_x - existing.center_x) <
             0.5 * (candidate.size_x + existing.size_x) + separation &&
         std::abs(candidate.center_y - existing.center_y) <
             0.5 * (candidate.size_y + existing.size_y) + separation;
}

void appendBox(const Box& box, double ground_z, double height,
               double resolution, pcl::PointCloud<pcl::PointXYZ>* cloud) {
  const int steps_x =
      std::max(1, static_cast<int>(std::ceil(box.size_x / resolution)));
  const int steps_y =
      std::max(1, static_cast<int>(std::ceil(box.size_y / resolution)));
  const int steps_z = std::max(
      1, static_cast<int>(std::ceil((height - ground_z) / resolution)));
  for (int ix = 0; ix <= steps_x; ++ix) {
    const double x = box.center_x - 0.5 * box.size_x +
                     ix * box.size_x / steps_x;
    for (int iy = 0; iy <= steps_y; ++iy) {
      const double y = box.center_y - 0.5 * box.size_y +
                       iy * box.size_y / steps_y;
      for (int iz = 0; iz <= steps_z; ++iz) {
        const double z = ground_z + iz * (height - ground_z) / steps_z;
        cloud->push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }
}

visualization_msgs::Marker makeMarker(const Box& box, double ground_z,
                                      double height,
                                      const std::string& frame_id,
                                      const ros::Time& stamp, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sparse_random_obstacles";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = box.center_x;
  marker.pose.position.y = box.center_y;
  marker.pose.position.z = 0.5 * (ground_z + height);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = box.size_x;
  marker.scale.y = box.size_y;
  marker.scale.z = height - ground_z;
  marker.color.r = box.kind == "narrow_gap" ? 0.70 :
                   (box.kind == "stacked" ? 0.48 :
                    (box.large ? 0.32 : 0.55));
  marker.color.g = box.kind == "narrow_gap" ? 0.42 :
                   (box.kind == "stacked" ? 0.32 :
                    (box.large ? 0.42 : 0.55));
  marker.color.b = box.kind == "narrow_gap" ? 0.18 :
                   (box.kind == "stacked" ? 0.62 :
                    (box.large ? 0.58 : 0.55));
  marker.color.a = 0.35;
  return marker;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "sparse_random_obstacle_map");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int seed = 17;
  int large_count = 3;
  int small_count = 4;
  double map_size_y = 14.0;
  double start_x = -10.0;
  double goal_x = 10.0;
  double endpoint_clearance = 3.0;
  double boundary_clearance = 0.8;
  double obstacle_separation = 0.7;
  double large_longitudinal_separation = 3.5;
  double small_center_y_limit = 2.0;
  double large_size_x_min = 0.6;
  double large_size_x_max = 1.2;
  double large_size_y_min = 2.5;
  double large_size_y_max = 4.5;
  double goal_side_large_size_y_max = 0.0;
  bool center_goal_side_large = false;
  double goal_side_large_center_y = 0.0;
  bool structured_challenge_enabled = false;
  double stacked_cluster_x = -2.0;
  double stacked_cluster_y = -0.3;
  double stacked_block_size = 1.2;
  double stacked_block_step = 0.75;
  double narrow_gap_x = 4.8;
  double narrow_gap_size_y = 1.6;
  double narrow_gap_wall_size_x = 0.8;
  double narrow_gap_segment_size_y = 4.6;
  double small_size_min = 0.45;
  double small_size_max = 0.9;
  double resolution = 0.1;
  double ground_z = -0.1;
  double height = 3.0;
  std::string frame_id = "world";

  private_nh.param("seed", seed, seed);
  private_nh.param("large_obstacle_count", large_count, large_count);
  private_nh.param("small_obstacle_count", small_count, small_count);
  private_nh.param("map_size_y", map_size_y, map_size_y);
  private_nh.param("start_x", start_x, start_x);
  private_nh.param("goal_x", goal_x, goal_x);
  private_nh.param("endpoint_clearance", endpoint_clearance,
                   endpoint_clearance);
  private_nh.param("boundary_clearance", boundary_clearance,
                   boundary_clearance);
  private_nh.param("obstacle_separation", obstacle_separation,
                   obstacle_separation);
  private_nh.param("large_longitudinal_separation",
                   large_longitudinal_separation,
                   large_longitudinal_separation);
  private_nh.param("small_center_y_limit", small_center_y_limit,
                   small_center_y_limit);
  private_nh.param("large_size_x_min", large_size_x_min,
                   large_size_x_min);
  private_nh.param("large_size_x_max", large_size_x_max,
                   large_size_x_max);
  private_nh.param("large_size_y_min", large_size_y_min,
                   large_size_y_min);
  private_nh.param("large_size_y_max", large_size_y_max,
                   large_size_y_max);
  private_nh.param("goal_side_large_size_y_max",
                   goal_side_large_size_y_max,
                   goal_side_large_size_y_max);
  private_nh.param("center_goal_side_large", center_goal_side_large,
                   center_goal_side_large);
  private_nh.param("goal_side_large_center_y", goal_side_large_center_y,
                   goal_side_large_center_y);
  private_nh.param("structured_challenge_enabled",
                   structured_challenge_enabled,
                   structured_challenge_enabled);
  private_nh.param("stacked_cluster_x", stacked_cluster_x,
                   stacked_cluster_x);
  private_nh.param("stacked_cluster_y", stacked_cluster_y,
                   stacked_cluster_y);
  private_nh.param("stacked_block_size", stacked_block_size,
                   stacked_block_size);
  private_nh.param("stacked_block_step", stacked_block_step,
                   stacked_block_step);
  private_nh.param("narrow_gap_x", narrow_gap_x, narrow_gap_x);
  private_nh.param("narrow_gap_size_y", narrow_gap_size_y,
                   narrow_gap_size_y);
  private_nh.param("narrow_gap_wall_size_x", narrow_gap_wall_size_x,
                   narrow_gap_wall_size_x);
  private_nh.param("narrow_gap_segment_size_y",
                   narrow_gap_segment_size_y,
                   narrow_gap_segment_size_y);
  private_nh.param("small_size_min", small_size_min, small_size_min);
  private_nh.param("small_size_max", small_size_max, small_size_max);
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("ground_z", ground_z, ground_z);
  private_nh.param("height", height, height);
  private_nh.param<std::string>("frame_id", frame_id, frame_id);

  const double course_min_x = std::min(start_x, goal_x) + endpoint_clearance;
  const double course_max_x = std::max(start_x, goal_x) - endpoint_clearance;
  const bool invalid =
      large_count < 0 || small_count < 0 || map_size_y <= 0.0 ||
      endpoint_clearance < 0.0 || boundary_clearance < 0.0 ||
      obstacle_separation < 0.0 || large_size_x_min <= 0.0 ||
      large_longitudinal_separation < 0.0 || small_center_y_limit <= 0.0 ||
      large_size_x_max < large_size_x_min || large_size_y_min <= 0.0 ||
      large_size_y_max < large_size_y_min || small_size_min <= 0.0 ||
      small_size_max < small_size_min || resolution <= 0.0 ||
      goal_side_large_size_y_max < 0.0 ||
      (structured_challenge_enabled &&
       (stacked_block_size <= 0.0 || stacked_block_step <= 0.0 ||
        narrow_gap_size_y <= 0.0 || narrow_gap_wall_size_x <= 0.0 ||
        narrow_gap_segment_size_y <= 0.0)) ||
      height <= ground_z || course_max_x <= course_min_x ||
      map_size_y <= 2.0 * boundary_clearance + large_size_y_max;
  if (invalid) {
    ROS_FATAL("Invalid sparse-random-map geometry parameters.");
    return 1;
  }

  std::mt19937 generator(static_cast<std::mt19937::result_type>(seed));
  std::vector<Box> boxes;
  std::vector<Box> placement_reservations;
  const int total_count = large_count + small_count;
  boxes.reserve(static_cast<std::size_t>(total_count + 5));

  if (structured_challenge_enabled) {
    // Three overlapping blocks form the sketched cluster: a smaller block on
    // the left, a lower horizontal block, and a larger block on the right.
    boxes.push_back({stacked_cluster_x + 0.33 * stacked_block_step,
                     stacked_cluster_y + 1.40 * stacked_block_step,
                     0.75 * stacked_block_size,
                     0.75 * stacked_block_size, false, "stacked"});
    boxes.push_back({stacked_cluster_x - 0.60 * stacked_block_step,
                     stacked_cluster_y,
                     stacked_block_size,
                     1.25 * stacked_block_size, false, "stacked"});
    boxes.push_back({stacked_cluster_x + 0.47 * stacked_block_step,
                     stacked_cluster_y - 1.53 * stacked_block_step,
                     1.08 * stacked_block_size,
                     1.08 * stacked_block_size, false, "stacked"});

    // Retain the former diagonal cluster footprint as a sampling reservation.
    // Changing only the visible three-block shape therefore does not reshuffle
    // the other seeded random obstacles.
    placement_reservations.push_back(
        {stacked_cluster_x - stacked_block_step,
         stacked_cluster_y - stacked_block_step,
         stacked_block_size, stacked_block_size, false, "reservation"});
    placement_reservations.push_back(
        {stacked_cluster_x, stacked_cluster_y,
         stacked_block_size, stacked_block_size, false, "reservation"});
    placement_reservations.push_back(
        {stacked_cluster_x + stacked_block_step,
         stacked_cluster_y + stacked_block_step,
         stacked_block_size, stacked_block_size, false, "reservation"});

    // Two transverse wall segments leave one narrow passage on the centerline.
    const double gap_segment_offset =
        0.5 * (narrow_gap_size_y + narrow_gap_segment_size_y);
    boxes.push_back({narrow_gap_x, -gap_segment_offset,
                     narrow_gap_wall_size_x, narrow_gap_segment_size_y,
                     false, "narrow_gap"});
    boxes.push_back({narrow_gap_x, gap_segment_offset,
                     narrow_gap_wall_size_x, narrow_gap_segment_size_y,
                     false, "narrow_gap"});

    for (const Box& fixture : boxes) {
      if (fixture.center_x - 0.5 * fixture.size_x < course_min_x ||
          fixture.center_x + 0.5 * fixture.size_x > course_max_x ||
          std::abs(fixture.center_y) + 0.5 * fixture.size_y >
              0.5 * map_size_y - boundary_clearance) {
        ROS_FATAL("Structured challenge obstacle is outside the course.");
        return 1;
      }
    }
  }
  const std::size_t random_box_begin = boxes.size();

  for (int index = 0; index < total_count; ++index) {
    const bool large = index < large_count;
    bool placed = false;
    for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
      Box candidate;
      candidate.large = large;
      candidate.kind = large ? "random_large" : "random_small";
      if (large) {
        candidate.size_x =
            sample(&generator, large_size_x_min, large_size_x_max);
        candidate.size_y =
            sample(&generator, large_size_y_min, large_size_y_max);
      } else {
        candidate.size_x = sample(&generator, small_size_min, small_size_max);
        candidate.size_y = sample(&generator, small_size_min, small_size_max);
      }

      const double x_margin = 0.5 * candidate.size_x;
      const double y_margin = boundary_clearance + 0.5 * candidate.size_y;
      const double center_y_bound =
          large ? 0.5 * map_size_y - y_margin
                : std::min(small_center_y_limit,
                           0.5 * map_size_y - y_margin);
      candidate.center_x = sample(&generator, course_min_x + x_margin,
                                  course_max_x - x_margin);
      candidate.center_y =
          sample(&generator, -center_y_bound, center_y_bound);

      placed = true;
      for (const Box& existing : boxes) {
        if (overlaps(candidate, existing, obstacle_separation) ||
            (candidate.large && existing.large &&
             std::abs(candidate.center_x - existing.center_x) <
                 large_longitudinal_separation)) {
          placed = false;
          break;
        }
      }
      if (placed) {
        for (const Box& reservation : placement_reservations) {
          if (overlaps(candidate, reservation, obstacle_separation)) {
            placed = false;
            break;
          }
        }
      }
      if (placed) boxes.push_back(candidate);
    }
    if (!placed) {
      ROS_FATAL_STREAM("Unable to place obstacle " << index
                       << "; reduce counts, sizes, or separation.");
      return 1;
    }

    // Finalize the goal-side large obstacle before sampling small obstacles,
    // so all later separation checks use its actual geometry.
    if (index + 1 == large_count) {
      auto nearest = boxes.begin() + random_box_begin;
      for (auto iterator = nearest + 1; iterator != boxes.end();
           ++iterator) {
        if (std::abs(iterator->center_x - goal_x) <
            std::abs(nearest->center_x - goal_x)) {
          nearest = iterator;
        }
      }
      if (goal_side_large_size_y_max > 0.0) {
        nearest->size_y =
            std::min(nearest->size_y, goal_side_large_size_y_max);
      }
      if (center_goal_side_large) {
        if (std::abs(goal_side_large_center_y) + 0.5 * nearest->size_y >
            0.5 * map_size_y - boundary_clearance) {
          ROS_FATAL("Goal-side large obstacle does not fit at requested y.");
          return 1;
        }
        nearest->center_y = goal_side_large_center_y;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const Box& box : boxes)
    appendBox(box, ground_z, height, resolution, &cloud);
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  message.header.stamp = ros::Time::now();

  ros::Publisher cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1, true);
  ros::Publisher marker_publisher =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/map_generator/sparse_random_geometry", 1, true);
  cloud_publisher.publish(message);

  visualization_msgs::MarkerArray markers;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    markers.markers.push_back(makeMarker(boxes[i], ground_z, height, frame_id,
                                         message.header.stamp,
                                         static_cast<int>(i)));
    ROS_INFO_STREAM("[sparse_random_obstacle " << i << "] center=("
                    << boxes[i].center_x << "," << boxes[i].center_y
                    << ") size=(" << boxes[i].size_x << ","
                    << boxes[i].size_y << ") kind="
                    << boxes[i].kind);
  }
  marker_publisher.publish(markers);

  ROS_INFO_STREAM("Sparse random map ready: seed=" << seed
                  << " obstacles=" << boxes.size() << " points="
                  << cloud.size() << " endpoint_clearance="
                  << endpoint_clearance);
  ros::spin();
  return 0;
}

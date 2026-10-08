#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

#ifndef FORMATION_UAV_COUNT
#define FORMATION_UAV_COUNT 6
#endif

constexpr std::size_t kUavCount = FORMATION_UAV_COUNT;
constexpr double kPi = 3.14159265358979323846;

struct VoxelIndex {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash {
  std::size_t operator()(const VoxelIndex& index) const {
    std::size_t seed = std::hash<std::int64_t>()(index.x);
    seed ^= std::hash<std::int64_t>()(index.y) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>()(index.z) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    return seed;
  }
};

class SwarmObservedMap {
 public:
  SwarmObservedMap() : private_nh_("~") {
    private_nh_.param("local_drone_id", local_drone_id_, -1);
    if (local_drone_id_ < -1 || local_drone_id_ >= static_cast<int>(kUavCount))
      throw std::runtime_error("invalid local_drone_id");
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("cloud_topic_suffix", cloud_suffix_,
                                   "pcl_render_node/cloud");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param("resolution", resolution_, 0.10);
    private_nh_.param("sensor_range", sensor_range_, 5.0);
    private_nh_.param("horizontal_fov", horizontal_fov_, kPi);
    private_nh_.param("vertical_fov", vertical_fov_, kPi / 3.0);
    private_nh_.param("azimuth_step", azimuth_step_, 0.10);
    private_nh_.param("elevation_step", elevation_step_, 0.12);
    private_nh_.param("min_update_period", min_update_period_, 0.15);
    private_nh_.param("min_observations_per_uav",
                      min_observations_per_uav_, 3);
    private_nh_.param("odom_timeout", odom_timeout_, 1.0);
    private_nh_.param("map_size_x", map_size_x_, 24.0);
    private_nh_.param("map_size_y", map_size_y_, 14.0);
    private_nh_.param("map_size_z", map_size_z_, 3.0);
    private_nh_.param("ground_height", ground_height_, -0.01);
    if (resolution_ <= 0.0 || sensor_range_ <= 0.0 ||
        horizontal_fov_ <= 0.0 || horizontal_fov_ > 2.0 * kPi ||
        vertical_fov_ <= 0.0 || vertical_fov_ >= kPi ||
        azimuth_step_ <= 0.0 || elevation_step_ <= 0.0 ||
        min_update_period_ < 0.0 || min_observations_per_uav_ < 1 ||
        odom_timeout_ <= 0.0 ||
        map_size_x_ <= 0.0 || map_size_y_ <= 0.0 || map_size_z_ <= 0.0) {
      throw std::runtime_error("invalid six-UAV observed-map parameters");
    }

    for (std::size_t id = 0; id < kUavCount; ++id) {
      if (local_drone_id_ >= 0 && static_cast<int>(id) != local_drone_id_) continue;
      const std::string prefix = "/drone_" + std::to_string(id) + "_";
      odom_subscribers_[id] = nh_.subscribe<nav_msgs::Odometry>(
          prefix + odom_suffix_, 10,
          [this, id](const nav_msgs::Odometry::ConstPtr& message) {
            if (local_drone_id_ >= 0 &&
                (message->header.stamp.isZero() ||
                 (ros::Time::now() - message->header.stamp).toSec() > odom_timeout_ ||
                 (message->header.stamp - ros::Time::now()).toSec() > 0.05)) return;
            poses_[id] = *message;
            odom_received_[id] = true;
            odom_receipt_stamps_[id] = ros::Time::now();
          });
      cloud_subscribers_[id] = nh_.subscribe<sensor_msgs::PointCloud2>(
          prefix + cloud_suffix_, 1,
          [this, id](const sensor_msgs::PointCloud2::ConstPtr& message) {
            cloudCallback(id, message);
          });
    }

    occupied_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/swarm/observed_occupied", 1, true);
    free_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/swarm/observed_free", 1, true);
    publish_timer_ = nh_.createTimer(ros::Duration(0.2),
                                     &SwarmObservedMap::publishMap, this);
    ROS_INFO_STREAM("[6UAV_OBSERVED_MAP] Ready. resolution=" << resolution_
                    << " range=" << sensor_range_ << " frame=" << frame_id_);
  }

 private:
  VoxelIndex indexOf(double x, double y, double z) const {
    return {static_cast<std::int64_t>(std::llround(x / resolution_)),
            static_cast<std::int64_t>(std::llround(y / resolution_)),
            static_cast<std::int64_t>(std::llround(z / resolution_))};
  }

  bool insideMap(double x, double y, double z) const {
    return x >= -0.5 * map_size_x_ && x <= 0.5 * map_size_x_ &&
           y >= -0.5 * map_size_y_ && y <= 0.5 * map_size_y_ &&
           z >= ground_height_ && z <= ground_height_ + map_size_z_;
  }

  pcl::PointXYZ pointOf(const VoxelIndex& index) const {
    return pcl::PointXYZ(index.x * resolution_, index.y * resolution_,
                         index.z * resolution_);
  }

  static double yawOf(const nav_msgs::Odometry& odom) {
    const auto& q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void observeRay(double ox, double oy, double oz, double dx, double dy,
                  double dz, double free_distance) {
    for (double distance = 0.0; distance <= free_distance + 1e-9;
         distance += resolution_) {
      const double x = ox + distance * dx;
      const double y = oy + distance * dy;
      const double z = oz + distance * dz;
      if (!insideMap(x, y, z)) break;
      const VoxelIndex index = indexOf(x, y, z);
      if (occupied_.count(index) == 0) known_free_.insert(index);
    }
  }

  void cloudCallback(std::size_t id,
                     const sensor_msgs::PointCloud2::ConstPtr& message) {
    const ros::Time now = ros::Time::now();
    if (local_drone_id_ >= 0 &&
        (message->header.stamp.isZero() ||
         message->header.stamp <= last_observation_stamp_ ||
         (now - message->header.stamp).toSec() > odom_timeout_ ||
         (message->header.stamp - now).toSec() > 0.05)) return;
    if (!last_cloud_updates_[id].isZero() &&
        (now - last_cloud_updates_[id]).toSec() < min_update_period_) {
      return;
    }
    if (!odom_received_[id] ||
        (now - odom_receipt_stamps_[id]).toSec() > odom_timeout_) {
      ROS_WARN_THROTTLE(2.0,
                        "[6UAV_OBSERVED_MAP] Cloud ignored: odometry stale.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    const auto& position = poses_[id].pose.pose.position;
    const double yaw = yawOf(poses_[id]);
    const int azimuth_count = std::max(
        1, static_cast<int>(std::ceil(horizontal_fov_ / azimuth_step_)));
    const int elevation_count = std::max(
        1, static_cast<int>(std::ceil(vertical_fov_ / elevation_step_)));
    const std::size_t bin_count = static_cast<std::size_t>(azimuth_count) *
                                  static_cast<std::size_t>(elevation_count);
    std::vector<double> nearest_ranges(
        bin_count, std::numeric_limits<double>::infinity());
    std::vector<VoxelIndex> nearest_hits(bin_count);
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !insideMap(point.x, point.y, point.z)) {
        continue;
      }
      const double distance = std::sqrt(
          std::pow(point.x - position.x, 2) +
          std::pow(point.y - position.y, 2) +
          std::pow(point.z - position.z, 2));
      if (distance < 1e-9 || distance > sensor_range_ + resolution_) continue;
      const double bearing = std::atan2(point.y - position.y,
                                        point.x - position.x);
      const double relative_azimuth =
          std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));
      const double elevation = std::asin(std::max(
          -1.0, std::min(1.0, (point.z - position.z) / distance)));
      if (std::fabs(relative_azimuth) > 0.5 * horizontal_fov_ + 1e-9 ||
          std::fabs(elevation) > 0.5 * vertical_fov_ + 1e-9)
        continue;
      const int azimuth_index = std::min(
          azimuth_count - 1,
          std::max(0, static_cast<int>((relative_azimuth +
                                        0.5 * horizontal_fov_) /
                                       horizontal_fov_ * azimuth_count)));
      const int elevation_index = std::min(
          elevation_count - 1,
          std::max(0, static_cast<int>((elevation + 0.5 * vertical_fov_) /
                                       vertical_fov_ * elevation_count)));
      const std::size_t bin = static_cast<std::size_t>(elevation_index) *
                                  static_cast<std::size_t>(azimuth_count) +
                              static_cast<std::size_t>(azimuth_index);
      if (distance < nearest_ranges[bin]) {
        nearest_ranges[bin] = distance;
        nearest_hits[bin] = indexOf(point.x, point.y, point.z);
      }
    }

    std::unordered_set<VoxelIndex, VoxelIndexHash> current_hits;
    for (int elevation_index = 0; elevation_index < elevation_count;
         ++elevation_index) {
      const double elevation = -0.5 * vertical_fov_ +
          (elevation_index + 0.5) * vertical_fov_ / elevation_count;
      for (int azimuth_index = 0; azimuth_index < azimuth_count;
           ++azimuth_index) {
        const double azimuth = -0.5 * horizontal_fov_ +
            (azimuth_index + 0.5) * horizontal_fov_ / azimuth_count;
        const std::size_t bin = static_cast<std::size_t>(elevation_index) *
                                    static_cast<std::size_t>(azimuth_count) +
                                static_cast<std::size_t>(azimuth_index);
        const bool has_hit = std::isfinite(nearest_ranges[bin]);
        const double free_distance = has_hit
            ? std::max(0.0, nearest_ranges[bin] - 1.5 * resolution_)
            : sensor_range_;
        const double horizontal = std::cos(elevation);
        observeRay(position.x, position.y, position.z,
                   horizontal * std::cos(yaw + azimuth),
                   horizontal * std::sin(yaw + azimuth),
                   std::sin(elevation), free_distance);
        if (has_hit) current_hits.insert(nearest_hits[bin]);
      }
    }

    for (const auto& index : current_hits) {
      occupied_.insert(index);
      known_free_.erase(index);
    }
    last_cloud_updates_[id] = now;
    ++observation_counts_[id];
    last_observation_stamp_ = local_drone_id_ >= 0 ? message->header.stamp : now;
    have_observation_ = true;
  }

  sensor_msgs::PointCloud2 cloudMessage(
      const std::unordered_set<VoxelIndex, VoxelIndexHash>& voxels,
      const ros::Time& stamp) const {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(voxels.size());
    for (const auto& voxel : voxels) cloud.push_back(pointOf(voxel));
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = stamp;
    return message;
  }

  void publishMap(const ros::TimerEvent&) {
    if (!have_observation_) return;
    for (std::size_t id = 0; id < kUavCount; ++id) {
      if (local_drone_id_ >= 0 && static_cast<int>(id) != local_drone_id_) continue;
      if (observation_counts_[id] < min_observations_per_uav_) {
        ROS_INFO_STREAM_THROTTLE(
            1.0, "[6UAV_OBSERVED_MAP] Warmup: waiting for "
                     << min_observations_per_uav_
                     << " observations from every UAV.");
        return;
      }
    }
    occupied_publisher_.publish(cloudMessage(occupied_, last_observation_stamp_));
    free_publisher_.publish(cloudMessage(known_free_, last_observation_stamp_));
    ROS_INFO_STREAM_THROTTLE(
        2.0, "[6UAV_OBSERVED_MAP] occupied=" << occupied_.size()
                                               << " known_free="
                                               << known_free_.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  int local_drone_id_{-1};
  std::array<ros::Subscriber, kUavCount> cloud_subscribers_;
  ros::Publisher occupied_publisher_;
  ros::Publisher free_publisher_;
  ros::Timer publish_timer_;
  std::array<nav_msgs::Odometry, kUavCount> poses_;
  std::array<bool, kUavCount> odom_received_{};
  std::array<ros::Time, kUavCount> odom_receipt_stamps_;
  std::array<ros::Time, kUavCount> last_cloud_updates_;
  std::array<int, kUavCount> observation_counts_{};
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> known_free_;
  ros::Time last_observation_stamp_;
  bool have_observation_{false};
  std::string odom_suffix_;
  std::string cloud_suffix_;
  std::string frame_id_;
  double resolution_{0.10};
  double sensor_range_{5.0};
  double horizontal_fov_{kPi};
  double vertical_fov_{kPi / 3.0};
  double azimuth_step_{0.10};
  double elevation_step_{0.12};
  double min_update_period_{0.15};
  int min_observations_per_uav_{3};
  double odom_timeout_{1.0};
  double map_size_x_{24.0};
  double map_size_y_{14.0};
  double map_size_z_{3.0};
  double ground_height_{-0.01};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_observed_map_6uav");
  try {
    SwarmObservedMap node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start six-UAV observed map: " << error.what());
    return 1;
  }
  return 0;
}

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
};

void appendBox(const Box& box, double ground_z, double height,
               double resolution, pcl::PointCloud<pcl::PointXYZ>* cloud) {
  const int steps_x = static_cast<int>(std::ceil(box.size_x / resolution));
  const int steps_y = static_cast<int>(std::ceil(box.size_y / resolution));
  const int steps_z =
      static_cast<int>(std::ceil((height - ground_z) / resolution));
  for (int ix = 0; ix <= steps_x; ++ix) {
    const double x = box.center_x - 0.5 * box.size_x +
                     ix * box.size_x / steps_x;
    for (int iy = 0; iy <= steps_y; ++iy) {
      const double y = box.center_y - 0.5 * box.size_y +
                       iy * box.size_y / steps_y;
      for (int iz = 0; iz <= steps_z; ++iz) {
        const double z =
            ground_z + iz * (height - ground_z) / steps_z;
        cloud->push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }
}

visualization_msgs::Marker makeMarker(
    const Box& box, double ground_z, double height,
    const std::string& frame_id, const ros::Time& stamp,
    const std::string& marker_namespace, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
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
  marker.color.r = 0.45;
  marker.color.g = 0.45;
  marker.color.b = 0.50;
  marker.color.a = 0.18;
  return marker;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fixed_split_merge_wall_map");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double first_x = 0.0;
  double first_y = 0.0;
  double first_size_x = 0.5;
  double first_size_y = 3.0;
  double second_x = 4.0;
  double second_y = 0.0;
  double second_size_x = 0.5;
  double second_total_size_y = 10.0;
  double second_gap_size_y = 3.0;
  double resolution = 0.1;
  double ground_z = -0.1;
  double height = 3.0;
  double safety_distance = 0.0;
  bool randomize = false;
  int random_seed = 1;
  std::string frame_id = "world";

  private_nh.param("first_wall_x", first_x, first_x);
  private_nh.param("first_wall_y", first_y, first_y);
  private_nh.param("first_wall_size_x", first_size_x, first_size_x);
  private_nh.param("first_wall_size_y", first_size_y, first_size_y);
  private_nh.param("second_wall_x", second_x, second_x);
  private_nh.param("second_wall_y", second_y, second_y);
  private_nh.param("second_wall_size_x", second_size_x, second_size_x);
  private_nh.param("second_wall_total_size_y", second_total_size_y,
                   second_total_size_y);
  private_nh.param("second_wall_gap_size_y", second_gap_size_y,
                   second_gap_size_y);
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("ground_z", ground_z, ground_z);
  private_nh.param("height", height, height);
  private_nh.param("safety_distance", safety_distance, safety_distance);
  private_nh.param("randomize", randomize, randomize);
  private_nh.param("random_seed", random_seed, random_seed);
  private_nh.param<std::string>("frame_id", frame_id, frame_id);

  if (randomize) {
    std::mt19937 generator(static_cast<std::mt19937::result_type>(random_seed));
    first_x = std::uniform_real_distribution<double>(-3.0, -1.0)(generator);
    first_y = std::uniform_real_distribution<double>(-0.5, 0.5)(generator);
    first_size_x = std::uniform_real_distribution<double>(0.4, 0.8)(generator);
    first_size_y = std::uniform_real_distribution<double>(3.0, 4.5)(generator);
    second_x = std::uniform_real_distribution<double>(4.5, 6.5)(generator);
    second_y = std::uniform_real_distribution<double>(-0.4, 0.4)(generator);
    second_size_x = std::uniform_real_distribution<double>(0.4, 0.8)(generator);
    second_gap_size_y =
        std::uniform_real_distribution<double>(2.0, 2.8)(generator);
  }

  if (first_size_x <= 0.0 || first_size_y <= 0.0 ||
      second_size_x <= 0.0 || second_total_size_y <= 0.0 ||
      second_gap_size_y <= 0.0 ||
      second_gap_size_y >= second_total_size_y || resolution <= 0.0 ||
      height <= ground_z || safety_distance < 0.0) {
    ROS_FATAL("Invalid split/merge wall geometry.");
    return 1;
  }

  const double segment_size_y =
      0.5 * (second_total_size_y - second_gap_size_y);
  const double segment_offset_y =
      0.5 * (second_gap_size_y + segment_size_y);
  const std::vector<Box> boxes = {
      {first_x, first_y, first_size_x, first_size_y},
      {second_x, second_y - segment_offset_y, second_size_x,
       segment_size_y},
      {second_x, second_y + segment_offset_y, second_size_x,
       segment_size_y},
  };

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
  ros::Publisher geometry_publisher =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/map_generator/wall_geometry", 1, true);
  cloud_publisher.publish(message);

  visualization_msgs::MarkerArray geometry;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    geometry.markers.push_back(makeMarker(
        boxes[i], ground_z, height, frame_id,
        message.header.stamp, "physical_split_merge_walls",
        static_cast<int>(i)));
  }
  geometry_publisher.publish(geometry);

  ROS_INFO_STREAM(
      "[split_merge_wall_geometry] randomize=" << (randomize ? "true" : "false")
      << " seed=" << random_seed << " first_center=(" << first_x << ","
      << first_y << ") first_size=(" << first_size_x << ","
      << first_size_y << ") second_x=" << second_x
      << " second_total_y=" << second_total_size_y
      << " second_gap_y=" << second_gap_size_y
      << " effective_gap_y="
      << second_gap_size_y - 2.0 * safety_distance << " height=["
      << ground_z << "," << height << "] points=" << cloud.size());

  ros::spin();
  return 0;
}

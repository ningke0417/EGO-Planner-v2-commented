#include <cmath>
#include <cstdint>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fixed_wall_map");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  double center_x = 0.0;
  double center_y = 0.0;
  double size_x = 0.5;
  double size_y = 5.0;
  double resolution = 0.1;
  double ground_z = -0.1;
  double height = 3.0;
  double safety_distance = 0.0;
  std::string frame_id = "world";
  private_nh.param("wall_x", center_x, center_x);
  private_nh.param("wall_y", center_y, center_y);
  private_nh.param("size_x", size_x, size_x);
  private_nh.param("size_y", size_y, size_y);
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("ground_z", ground_z, ground_z);
  private_nh.param("height", height, height);
  private_nh.param("safety_distance", safety_distance, safety_distance);
  private_nh.param<std::string>("frame_id", frame_id, frame_id);
  if (size_x <= 0.0 || size_y <= 0.0 || resolution <= 0.0 ||
      height <= ground_z) {
    ROS_FATAL("Invalid fixed-wall geometry.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  const int steps_x = static_cast<int>(std::ceil(size_x / resolution));
  const int steps_y = static_cast<int>(std::ceil(size_y / resolution));
  const int steps_z =
      static_cast<int>(std::ceil((height - ground_z) / resolution));
  for (int ix = 0; ix <= steps_x; ++ix) {
    const double x = center_x - 0.5 * size_x + ix * size_x / steps_x;
    for (int iy = 0; iy <= steps_y; ++iy) {
      const double y = center_y - 0.5 * size_y + iy * size_y / steps_y;
      for (int iz = 0; iz <= steps_z; ++iz) {
        const double z = ground_z + iz * (height - ground_z) / steps_z;
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  message.header.stamp = ros::Time::now();
  ros::Publisher publisher =
      nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1, true);
  ros::Publisher geometry_publisher =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/map_generator/wall_geometry", 1, true);
  publisher.publish(message);

  visualization_msgs::MarkerArray geometry;
  visualization_msgs::Marker physical;
  physical.header.frame_id = frame_id;
  physical.header.stamp = message.header.stamp;
  physical.ns = "physical_wall";
  physical.id = 0;
  physical.type = visualization_msgs::Marker::CUBE;
  physical.action = visualization_msgs::Marker::ADD;
  physical.pose.position.x = center_x;
  physical.pose.position.y = center_y;
  physical.pose.position.z = 0.5 * (ground_z + height);
  physical.pose.orientation.w = 1.0;
  physical.scale.x = size_x;
  physical.scale.y = size_y;
  physical.scale.z = height - ground_z;
  physical.color.r = 0.45;
  physical.color.g = 0.45;
  physical.color.b = 0.50;
  physical.color.a = 0.18;
  geometry.markers.push_back(physical);

  visualization_msgs::Marker guard = physical;
  guard.ns = "inflated_wall_guard";
  guard.id = 1;
  guard.scale.x = size_x + 2.0 * safety_distance;
  guard.scale.y = size_y + 2.0 * safety_distance;
  guard.scale.z = height - ground_z + 0.02;
  guard.color.r = 1.0;
  guard.color.g = 0.75;
  guard.color.b = 0.0;
  guard.color.a = 0.12;
  geometry.markers.push_back(guard);
  geometry_publisher.publish(geometry);

  ROS_INFO_STREAM("[fixed_wall_geometry] center=(" << center_x << ","
                  << center_y << ") physical_bounds_x=["
                  << center_x - 0.5 * size_x << ","
                  << center_x + 0.5 * size_x << "] physical_bounds_y=["
                  << center_y - 0.5 * size_y << ","
                  << center_y + 0.5 * size_y << "] safety="
                  << safety_distance << " guard_bounds_x=["
                  << center_x - 0.5 * size_x - safety_distance << ","
                  << center_x + 0.5 * size_x + safety_distance
                  << "] guard_bounds_y=["
                  << center_y - 0.5 * size_y - safety_distance << ","
                  << center_y + 0.5 * size_y + safety_distance
                  << "] height=[" << ground_z << "," << height
                  << "] points=" << cloud.size());
  ros::spin();
  return 0;
}

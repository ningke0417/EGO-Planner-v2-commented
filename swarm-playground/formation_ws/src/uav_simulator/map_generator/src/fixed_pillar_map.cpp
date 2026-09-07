#include <cmath>
#include <cstdint>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fixed_pillar_map");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  double center_x = 0.0;
  double center_y = 0.0;
  double radius = 1.0;
  double resolution = 0.1;
  double ground_z = -0.1;
  double height = 3.0;
  std::string frame_id = "world";
  private_nh.param("pillar_x", center_x, center_x);
  private_nh.param("pillar_y", center_y, center_y);
  private_nh.param("pillar_radius", radius, radius);
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("ground_z", ground_z, ground_z);
  private_nh.param("height", height, height);
  private_nh.param<std::string>("frame_id", frame_id, frame_id);
  if (radius <= 0.0 || resolution <= 0.0 || height <= ground_z) {
    ROS_FATAL("Invalid fixed-pillar geometry.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  const int radial_steps = static_cast<int>(std::ceil(radius / resolution));
  const int vertical_steps =
      static_cast<int>(std::ceil((height - ground_z) / resolution));
  for (int ix = -radial_steps; ix <= radial_steps; ++ix) {
    const double dx = ix * resolution;
    for (int iy = -radial_steps; iy <= radial_steps; ++iy) {
      const double dy = iy * resolution;
      if (dx * dx + dy * dy > radius * radius) continue;
      for (int iz = 0; iz <= vertical_steps; ++iz) {
        cloud.push_back(pcl::PointXYZ(center_x + dx, center_y + dy,
                                      ground_z + iz * resolution));
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
  publisher.publish(message);
  ROS_INFO_STREAM("Published one fixed pillar at (" << center_x << ", "
                  << center_y << "), radius=" << radius << ", points="
                  << cloud.size());
  ros::spin();
  return 0;
}

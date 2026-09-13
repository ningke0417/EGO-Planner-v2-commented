#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <path_searching/hungarian_assignment.hpp>
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>

namespace
{

constexpr int kUavCount = 4;

class InitialFormationAssignment
{
public:
  InitialFormationAssignment() : private_nh_("~")
  {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param("settle_duration", settle_duration_, 0.5);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    if (settle_duration_ < 0.0 || odom_timeout_ <= 0.0)
      throw std::runtime_error("invalid initial assignment timing parameters");

    std::vector<double> start;
    if (!private_nh_.getParam("formation_start", start) || start.size() != 3)
      throw std::runtime_error("formation_start must contain three values");
    formation_start_ = Eigen::Vector3d(start[0], start[1], start[2]);
    for (int slot = 0; slot < kUavCount; ++slot)
    {
      std::vector<double> offset;
      if (!private_nh_.getParam("formation_slot" + std::to_string(slot), offset) ||
          offset.size() != 3)
        throw std::runtime_error("each formation_slot parameter must contain three values");
      slot_positions_[slot] = formation_start_ +
          Eigen::Vector3d(offset[0], offset[1], offset[2]);

      const std::string topic = "/drone_" + std::to_string(slot) + "_" + odom_suffix_;
      odom_subscribers_[slot] = nh_.subscribe<nav_msgs::Odometry>(
          topic, 1, [this, slot](const nav_msgs::Odometry::ConstPtr &message) {
            positions_[slot] = Eigen::Vector3d(message->pose.pose.position.x,
                                                message->pose.pose.position.y,
                                                message->pose.pose.position.z);
            odom_stamps_[slot] = ros::Time::now();
            if (!odom_seen_[slot])
            {
              odom_seen_[slot] = true;
              first_all_seen_stamp_ = ros::Time();
            }
          });
    }
    assignment_publisher_ = nh_.advertise<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1, true);
    timer_ = nh_.createTimer(ros::Duration(0.05),
                             &InitialFormationAssignment::timerCallback, this);
  }

private:
  void timerCallback(const ros::TimerEvent &)
  {
    if (published_)
      return;
    const ros::Time now = ros::Time::now();
    for (int id = 0; id < kUavCount; ++id)
    {
      if (!odom_seen_[id] || (now - odom_stamps_[id]).toSec() > odom_timeout_)
      {
        ROS_WARN_THROTTLE(2.0, "[INITIAL_ASSIGNMENT] waiting for four fresh odometry topics");
        first_all_seen_stamp_ = ros::Time();
        return;
      }
    }
    if (first_all_seen_stamp_.isZero())
    {
      first_all_seen_stamp_ = now;
      return;
    }
    if ((now - first_all_seen_stamp_).toSec() < settle_duration_)
      return;

    Eigen::MatrixXd cost(kUavCount, kUavCount);
    for (int id = 0; id < kUavCount; ++id)
      for (int slot = 0; slot < kUavCount; ++slot)
        cost(id, slot) = (positions_[id] - slot_positions_[slot]).squaredNorm();

    const Eigen::VectorXi assignment =
        path_searching::hungarianAssignment(cost);
    std_msgs::UInt32MultiArray message;
    message.data.resize(kUavCount);
    double total_cost = 0.0;
    for (int id = 0; id < kUavCount; ++id)
    {
      message.data[id] = static_cast<uint32_t>(assignment(id));
      total_cost += cost(id, assignment(id));
    }
    assignment_publisher_.publish(message);
    published_ = true;
    ROS_WARN_STREAM("[INITIAL_ASSIGNMENT] frozen drone->slot mapping ["
                    << assignment.transpose() << "] squared_distance_cost="
                    << total_cost);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  ros::Publisher assignment_publisher_;
  ros::Timer timer_;
  std::array<Eigen::Vector3d, kUavCount> positions_;
  std::array<Eigen::Vector3d, kUavCount> slot_positions_;
  std::array<ros::Time, kUavCount> odom_stamps_;
  std::array<bool, kUavCount> odom_seen_{{false, false, false, false}};
  Eigen::Vector3d formation_start_{Eigen::Vector3d::Zero()};
  ros::Time first_all_seen_stamp_;
  std::string odom_suffix_;
  double settle_duration_{0.5};
  double odom_timeout_{2.0};
  bool published_{false};
};

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initial_formation_assignment_4uav");
  try
  {
    InitialFormationAssignment node;
    ros::spin();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL_STREAM("Unable to start initial formation assignment: " << error.what());
    return 1;
  }
  return 0;
}

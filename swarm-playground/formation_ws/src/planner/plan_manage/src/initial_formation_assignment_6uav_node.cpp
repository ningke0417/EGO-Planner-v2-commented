#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ego_planner/FormationTopologyCommand.h>
#include <nav_msgs/Odometry.h>
#include <path_searching/hungarian_assignment.hpp>
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>

namespace
{

#ifndef FORMATION_UAV_COUNT
#define FORMATION_UAV_COUNT 6
#endif

constexpr int kUavCount = FORMATION_UAV_COUNT;

class InitialFormationAssignment
{
public:
  InitialFormationAssignment() : private_nh_("~")
  {
    for (int id = 0; id < kUavCount; ++id)
      current_assignment_[id] = id;
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param("settle_duration", settle_duration_, 0.5);
    private_nh_.param("odom_timeout", odom_timeout_, 2.0);
    private_nh_.param("reassignment_interval", reassignment_interval_, 1.0);
    private_nh_.param("reassignment_stable_count", reassignment_stable_count_, 2);
    private_nh_.param("prediction_horizon", prediction_horizon_, 0.75);
    private_nh_.param("switch_penalty", switch_penalty_, 0.25);
    private_nh_.param("minimum_improvement_ratio",
                      minimum_improvement_ratio_, 0.15);
    if (settle_duration_ < 0.0 || odom_timeout_ <= 0.0 ||
        reassignment_interval_ <= 0.0 || reassignment_stable_count_ < 1 ||
        prediction_horizon_ < 0.0 || switch_penalty_ < 0.0 ||
        minimum_improvement_ratio_ < 0.0 ||
        minimum_improvement_ratio_ >= 1.0)
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
            velocities_[slot] = Eigen::Vector3d(message->twist.twist.linear.x,
                                                message->twist.twist.linear.y,
                                                message->twist.twist.linear.z);
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
    topology_subscriber_ = nh_.subscribe<ego_planner::FormationTopologyCommand>(
        "/formation_backend/topology_command", 10,
        &InitialFormationAssignment::topologyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.05),
                             &InitialFormationAssignment::timerCallback, this);
  }

private:
  bool odometryReady(const ros::Time &now) const
  {
    for (int id = 0; id < kUavCount; ++id)
      if (!odom_seen_[id] || (now - odom_stamps_[id]).toSec() > odom_timeout_)
        return false;
    return true;
  }

  void publishAssignment(const Eigen::VectorXi &assignment,
                         const std::string &reason)
  {
    std_msgs::UInt32MultiArray message;
    message.data.resize(kUavCount + 1);
    message.data[0] = ++assignment_generation_;
    for (int id = 0; id < kUavCount; ++id)
    {
      message.data[id + 1] = static_cast<uint32_t>(assignment(id));
      current_assignment_[id] = assignment(id);
    }
    assignment_publisher_.publish(message);
    ROS_WARN_STREAM("[FORMATION_ASSIGNMENT] generation="
                    << assignment_generation_ << " reason=" << reason
                    << " drone->slot=[" << assignment.transpose() << "]");
  }

  void topologyCallback(
      const ego_planner::FormationTopologyCommand::ConstPtr &message)
  {
    if (message->epoch == last_topology_epoch_)
      return;
    const bool was_intact =
        last_topology_mode_ == ego_planner::FormationTopologyCommand::INTACT;
    last_topology_epoch_ = message->epoch;
    last_topology_mode_ = message->mode;

    if (message->mode == ego_planner::FormationTopologyCommand::INTACT)
    {
      if (split_cycle_active_)
      {
        reassignment_locked_for_split_cycle_ = true;
        reassignment_pending_ = false;
        ROS_INFO_STREAM("[FORMATION_ASSIGNMENT] split/merge cycle finalized by epoch="
                        << message->epoch << " reason=" << message->reason);
      }
      return;
    }

    const bool starts_new_split_cycle =
        message->reason == "CRITERION_SPLIT" ||
        message->reason == "CRITERION_TIMEOUT_FAILSAFE" ||
        message->reason == "POST_MERGE_HARD_UNSAFE_FAILSAFE" ||
        message->reason == "POST_MERGE_BASELINE_TIMEOUT_FAILSAFE";
    if (message->mode == ego_planner::FormationTopologyCommand::SPLIT &&
        starts_new_split_cycle && was_intact)
    {
      split_cycle_active_ = true;
      reassignment_locked_for_split_cycle_ = false;
      keep_assignment_published_for_split_cycle_ = false;
      reassignment_pending_ = false;
      ROS_INFO("[FORMATION_ASSIGNMENT] new split/merge cycle detected");
    }

    if (!published_ || message->reason != "VISIBILITY_WEAK_RECONNECT")
    {
      if (message->reason != "WEAK_RECOVERY_RAMP_MID" &&
          message->reason != "WEAK_RECOVERY_RAMP_FINAL")
        reassignment_pending_ = false;
      return;
    }
    if (!split_cycle_active_)
    {
      ROS_WARN_STREAM("[FORMATION_ASSIGNMENT] ignore weak reconnect epoch="
                      << message->epoch << " without an active criterion split cycle");
      return;
    }
    if (reassignment_locked_for_split_cycle_)
    {
      ROS_WARN_STREAM("[FORMATION_ASSIGNMENT] ignore repeated weak reconnect epoch="
                      << message->epoch
                      << "; changed assignment already committed in this split cycle");
      return;
    }
    reassignment_pending_ = true;
    stable_count_ = 0;
    last_evaluation_stamp_ = ros::Time();
    ROS_WARN_STREAM("[FORMATION_ASSIGNMENT] merge reassignment armed by topology epoch="
                    << message->epoch);
  }

  std::string matrixText(const Eigen::MatrixXd &matrix) const
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(3) << "[";
    for (int row = 0; row < matrix.rows(); ++row)
    {
      if (row) output << ";";
      output << "[";
      for (int column = 0; column < matrix.cols(); ++column)
      {
        if (column) output << ",";
        output << matrix(row, column);
      }
      output << "]";
    }
    output << "]";
    return output.str();
  }

  void evaluateReassignment(const ros::Time &now)
  {
    if (!reassignment_pending_ || !odometryReady(now))
      return;
    if (!last_evaluation_stamp_.isZero() &&
        (now - last_evaluation_stamp_).toSec() < reassignment_interval_)
      return;
    last_evaluation_stamp_ = now;

    std::array<Eigen::Vector3d, kUavCount> predicted;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int id = 0; id < kUavCount; ++id)
    {
      predicted[id] = positions_[id] + prediction_horizon_ * velocities_[id];
      center += predicted[id] / static_cast<double>(kUavCount);
    }

    Eigen::MatrixXd raw_cost(kUavCount, kUavCount);
    Eigen::MatrixXd assignment_cost(kUavCount, kUavCount);
    for (int id = 0; id < kUavCount; ++id)
    {
      for (int slot = 0; slot < kUavCount; ++slot)
      {
        const Eigen::Vector3d target =
            center + (slot_positions_[slot] - formation_start_);
        raw_cost(id, slot) = (predicted[id] - target).squaredNorm();
        assignment_cost(id, slot) =
            raw_cost(id, slot) +
            (current_assignment_[id] == slot ? 0.0 : switch_penalty_);
      }
    }

    Eigen::VectorXi candidate =
        path_searching::hungarianAssignment(assignment_cost);
    double current_cost = 0.0;
    double candidate_cost = 0.0;
    bool changed = false;
    for (int id = 0; id < kUavCount; ++id)
    {
      current_cost += raw_cost(id, current_assignment_[id]);
      candidate_cost += raw_cost(id, candidate(id));
      changed = changed || candidate(id) != current_assignment_[id];
    }
    const bool improvement_accepted =
        changed &&
        candidate_cost <= current_cost * (1.0 - minimum_improvement_ratio_);
    if (!improvement_accepted)
    {
      changed = false;
      candidate.resize(kUavCount);
      for (int id = 0; id < kUavCount; ++id)
        candidate(id) = current_assignment_[id];
    }

    if (last_candidate_.size() == kUavCount &&
        (last_candidate_ - candidate).squaredNorm() == 0)
      ++stable_count_;
    else
    {
      last_candidate_ = candidate;
      stable_count_ = 1;
    }

    ROS_INFO_STREAM("[FORMATION_ASSIGNMENT] merge candidate=["
                    << candidate.transpose() << "] stable=" << stable_count_
                    << "/" << reassignment_stable_count_
                    << " current_cost=" << current_cost
                    << " candidate_cost=" << candidate_cost
                    << " improvement_accepted="
                    << (improvement_accepted ? "true" : "false")
                    << " raw_cost=" << matrixText(raw_cost)
                    << " assignment_cost=" << matrixText(assignment_cost));
    if (stable_count_ < reassignment_stable_count_)
      return;

    if (changed)
    {
      publishAssignment(candidate, "MERGE_REASSIGN");
      reassignment_pending_ = false;
      reassignment_locked_for_split_cycle_ = true;
      return;
    }

    // MERGE_KEEP is an acknowledgement for the bridge, not a terminal
    // decision.  Keep evaluating while weak recovery is active so a later,
    // genuinely shorter assignment can still be committed.
    if (!keep_assignment_published_for_split_cycle_)
    {
      publishAssignment(candidate, "MERGE_KEEP");
      keep_assignment_published_for_split_cycle_ = true;
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    if (published_)
    {
      evaluateReassignment(now);
      return;
    }
    if (!odometryReady(now))
    {
      ROS_WARN_THROTTLE(2.0, "[INITIAL_ASSIGNMENT] waiting for six fresh odometry topics");
      first_all_seen_stamp_ = ros::Time();
      return;
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

    const Eigen::VectorXi assignment = path_searching::hungarianAssignment(cost);
    double total_cost = 0.0;
    for (int id = 0; id < kUavCount; ++id)
      total_cost += cost(id, assignment(id));
    publishAssignment(assignment, "INITIAL");
    published_ = true;
    ROS_WARN_STREAM("[INITIAL_ASSIGNMENT] squared_distance_cost=" << total_cost);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  ros::Subscriber topology_subscriber_;
  ros::Publisher assignment_publisher_;
  ros::Timer timer_;
  std::array<Eigen::Vector3d, kUavCount> positions_;
  std::array<Eigen::Vector3d, kUavCount> velocities_;
  std::array<Eigen::Vector3d, kUavCount> slot_positions_;
  std::array<ros::Time, kUavCount> odom_stamps_;
  std::array<bool, kUavCount> odom_seen_{};
  Eigen::Vector3d formation_start_{Eigen::Vector3d::Zero()};
  ros::Time first_all_seen_stamp_;
  std::string odom_suffix_;
  double settle_duration_{0.5};
  double odom_timeout_{2.0};
  double reassignment_interval_{1.0};
  int reassignment_stable_count_{2};
  double prediction_horizon_{0.75};
  double switch_penalty_{0.25};
  double minimum_improvement_ratio_{0.15};
  std::array<int, kUavCount> current_assignment_{};
  Eigen::VectorXi last_candidate_;
  ros::Time last_evaluation_stamp_;
  uint32_t assignment_generation_{0};
  uint64_t last_topology_epoch_{0};
  uint8_t last_topology_mode_{ego_planner::FormationTopologyCommand::INTACT};
  int stable_count_{0};
  bool reassignment_pending_{false};
  bool split_cycle_active_{false};
  bool reassignment_locked_for_split_cycle_{false};
  bool keep_assignment_published_for_split_cycle_{false};
  bool published_{false};
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initial_formation_assignment_6uav");
  try
  {
    InitialFormationAssignment node;
    ros::spin();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL_STREAM("Unable to start six-UAV initial formation assignment: "
                     << error.what());
    return 1;
  }
  return 0;
}

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ego_planner/FormationTopologyAck.h>
#include <ego_planner/FormationTopologyCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_manage/distributed_edges.h>

namespace {
namespace topology = ego_planner::distributed;
using Marker = visualization_msgs::Marker;
class Visualizer {
 public:
  Visualizer() : nh_("~") {
    nh_.param<std::string>("odom_topic_suffix", odom_suffix_, "visual_slam/odom");
    nh_.param<std::string>("frame_id", frame_, "world");
    nh_.param("state_timeout", state_timeout_, 1.5);
    nh_.param("odom_timeout", odom_timeout_, 1.0);
    double rate;
    nh_.param("publish_rate", rate, 10.0);
    if (!std::isfinite(rate) || rate <= 0 ||
        !std::isfinite(state_timeout_) || state_timeout_ <= 0 ||
        !std::isfinite(odom_timeout_) || odom_timeout_ <= 0)
      throw std::runtime_error("invalid visualizer timing parameters");
    lifetime_ = std::max(0.5, 3.0 / rate);
    for (int id = 0; id < topology::kAgents; ++id) {
      const std::string prefix = "/drone_" + std::to_string(id);
      state_sub_[id] = nh_.subscribe<ego_planner::FormationTopologyAck>(
          prefix + "/formation_applied", 1,
          [this, id](const ego_planner::FormationTopologyAck::ConstPtr& msg) {
            auto& a = agents_[id];
            if (msg->drone_id != static_cast<unsigned>(id) ||
                msg->edge_count != topology::kEdges ||
                msg->edge_keep_mask >= (1u << topology::kEdges) ||
                msg->mode != ego_planner::FormationTopologyCommand::SPLIT ||
                !validStamp(msg->header.stamp, a.state_stamp, state_timeout_))
              return;
            a.mask = msg->edge_keep_mask;
            a.state_stamp = msg->header.stamp;
            a.state_received = ros::WallTime::now();
          });
      odom_sub_[id] = nh_.subscribe<nav_msgs::Odometry>(
          prefix + "_" + odom_suffix_, 1,
          [this, id](const nav_msgs::Odometry::ConstPtr& msg) {
            auto& a = agents_[id];
            const auto& p = msg->pose.pose.position;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !validStamp(msg->header.stamp, a.odom_stamp, odom_timeout_)) return;
            // Positions follow the existing simulator's common world-frame contract.
            a.position = p;
            a.odom_stamp = msg->header.stamp;
            a.odom_received = ros::WallTime::now();
          });
    }
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &Visualizer::tick, this);
  }
 private:
  struct Agent {
    geometry_msgs::Point position;
    uint32_t mask{0};
    ros::Time state_stamp, odom_stamp;
    ros::WallTime state_received, odom_received;
  };
  bool validStamp(const ros::Time& stamp, const ros::Time& previous, double timeout) const {
    const double age = (ros::Time::now() - stamp).toSec();
    return !stamp.isZero() && stamp > previous && age >= -0.05 && age <= timeout;
  }
  bool fresh(const ros::Time& stamp, const ros::WallTime& received,
             double timeout) const {
    const double age = (ros::Time::now() - stamp).toSec();
    return !stamp.isZero() && age >= -0.05 && age <= timeout &&
           (ros::WallTime::now() - received).toSec() <= timeout;
  }
  bool fresh(const Agent& agent) const {
    return fresh(agent.state_stamp, agent.state_received, state_timeout_) &&
           fresh(agent.odom_stamp, agent.odom_received, odom_timeout_);
  }
  Marker marker(const std::string& ns, int id, int type) const {
    Marker m;
    m.header.frame_id = frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns; m.id = id; m.type = type; m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    m.lifetime = ros::Duration(lifetime_);
    return m;
  }
  void tick(const ros::TimerEvent&) {
    bool show_cut = true;
    nh_.param("show_cut_edges", show_cut, true);
    bool show_labels = false;
    nh_.param("show_edge_labels", show_labels, false);
    visualization_msgs::MarkerArray output;
    std::array<int, 4> counts{}; // connected, disagreement, cut, stale/missing
    for (int edge = 0; edge < topology::kEdges; ++edge) {
      const auto pair = topology::edges()[edge];
      const auto& a = agents_[pair[0]];
      const auto& b = agents_[pair[1]];
      const bool first = a.mask & (1u << edge), second = b.mask & (1u << edge);
      const int state = !fresh(a) || !fresh(b) ? 3 :
                        first != second ? 1 : first ? 0 : 2;
      ++counts[state];
      Marker line = marker("distributed_edges", edge, Marker::LINE_LIST);
      Marker label = marker("distributed_edge_labels", edge, Marker::TEXT_VIEW_FACING);
      // Never draw an edge at invented coordinates for an unobserved endpoint.
      if (a.odom_stamp.isZero() || b.odom_stamp.isZero() || (state == 2 && !show_cut)) {
        line.action = label.action = Marker::DELETE;
      } else {
        static const std::array<std::array<float, 3>, 4> colors{{
            {{0.15f, 0.95f, 0.95f}}, {{1.0f, 0.8f, 0.0f}},
            {{1.0f, 0.10f, 0.05f}}, {{0.65f, 0.65f, 0.65f}}}};
        static const std::array<const char*, 4> names{{"ON", "MISMATCH", "CUT", "STALE"}};
        line.color.r = colors[state][0];
        line.color.g = colors[state][1];
        line.color.b = colors[state][2];
        line.color.a = state == 0 ? 1.0 : 0.8;
        line.scale.x = state == 0 ? 0.085 : 0.045;
        if (state == 0) {
          line.points = {a.position, b.position};
        } else {
          // Match the centralized view: four dashes, slightly above the edge.
          for (int dash = 0; dash < 4; ++dash) {
            for (double ratio : {0.25 * dash, 0.25 * dash + 0.14}) {
              geometry_msgs::Point point;
              point.x = a.position.x * (1-ratio) + b.position.x * ratio;
              point.y = a.position.y * (1-ratio) + b.position.y * ratio;
              point.z = a.position.z * (1-ratio) + b.position.z * ratio + 0.02;
              line.points.push_back(point);
            }
          }
        }
        label.color = line.color;
        label.scale.z = 0.14;
        label.pose.position.x = (a.position.x + b.position.x) / 2;
        label.pose.position.y = (a.position.y + b.position.y) / 2;
        label.pose.position.z = (a.position.z + b.position.z) / 2 + 0.18;
        label.text = std::to_string(pair[0]) + "-" + std::to_string(pair[1]) + " " + names[state];
        if (!fresh(a.odom_stamp, a.odom_received, odom_timeout_) ||
            !fresh(b.odom_stamp, b.odom_received, odom_timeout_))
          label.text += " (last pose)";
      }
      if (!show_labels) label.action = Marker::DELETE;
      output.markers.push_back(line);
      output.markers.push_back(label);
    }
    Marker legend = marker("distributed_legend", 0, Marker::TEXT_VIEW_FACING);
    legend.color.r = 1.0;
    legend.color.g = counts[3] ? 0.55 : 0.9;
    legend.color.b = counts[3] ? 0.1 : 0.2;
    legend.scale.z = 0.24;
    int positions_seen = 0;
    for (const auto& a : agents_) if (!a.odom_stamp.isZero()) {
      legend.pose.position.x += a.position.x;
      legend.pose.position.y += a.position.y;
      legend.pose.position.z += a.position.z;
      ++positions_seen;
    }
    if (positions_seen) {
      legend.pose.position.x /= positions_seen;
      legend.pose.position.y /= positions_seen;
      legend.pose.position.z /= positions_seen;
    }
    legend.pose.position.z += 0.90;
    legend.text = "APPLIED  ON " + std::to_string(counts[0]) +
        "  CUT " + std::to_string(counts[2]);
    if (counts[1]) legend.text += "  MISMATCH " + std::to_string(counts[1]);
    if (counts[3]) legend.text += "  STALE " + std::to_string(counts[3]);
    output.markers.push_back(legend);
    pub_.publish(output);
  }
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::array<ros::Subscriber, topology::kAgents> state_sub_, odom_sub_;
  std::array<Agent, topology::kAgents> agents_{};
  std::string odom_suffix_, frame_;
  double state_timeout_, odom_timeout_, lifetime_;
};
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "distributed_formation_visualizer");
  try { Visualizer visualizer; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("%s", error.what()); return 1; }
  return 0;
}

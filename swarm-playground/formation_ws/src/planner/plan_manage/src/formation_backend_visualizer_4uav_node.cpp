#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr std::size_t kUavCount = 4;
const std::array<std::array<int, 2>, 4> kSlotEdges{
    {{{0, 1}}, {{1, 3}}, {{3, 2}}, {{2, 0}}}};
std::array<std::array<int, 2>, 4> gEdges = kSlotEdges;

class BackendVisualizer {
 public:
  BackendVisualizer() : private_nh_("~") {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param<std::string>("obstacle_shape", obstacle_shape_, "pillar");
    private_nh_.param("pillar_x", pillar_x_, 0.0);
    private_nh_.param("pillar_y", pillar_y_, 0.0);
    private_nh_.param("pillar_radius", pillar_radius_, 1.0);
    private_nh_.param("wall_x", wall_x_, 0.0);
    private_nh_.param("wall_y", wall_y_, 0.0);
    private_nh_.param("wall_size_x", wall_size_x_, 0.5);
    private_nh_.param("wall_size_y", wall_size_y_, 5.0);
    private_nh_.param("safety_distance", safety_distance_, 0.35);
    private_nh_.param("desired_pair_distance", desired_pair_distance_, 1.0);
    private_nh_.param("max_pair_error", max_pair_error_, 0.35);
    private_nh_.param("side_deadband", side_deadband_, 0.20);
    for (std::size_t id = 0; id < kUavCount; ++id) {
      odom_subscribers_[id] = nh_.subscribe<nav_msgs::Odometry>(
          "/drone_" + std::to_string(id) + "_" + odom_suffix_, 1,
          [this, id](const nav_msgs::Odometry::ConstPtr& message) {
            positions_[id] = message->pose.pose.position;
            odom_seen_[id] = true;
          });
      group_subscribers_[id] = nh_.subscribe<std_msgs::String>(
          "/formation_backend/drone_" + std::to_string(id) + "_group", 1,
          [this, id](const std_msgs::String::ConstPtr& message) {
            groups_[id] = message->data;
            group_seen_[id] = true;
          });
    }
    auto_split_status_subscriber_ = nh_.subscribe<std_msgs::String>(
        "/formation_backend/auto_split_status", 1,
        [this](const std_msgs::String::ConstPtr& message) {
          auto_split_status_ = message->data;
        });
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &BackendVisualizer::assignmentCallback, this);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/formation_backend/markers", 1, true);
    subgroup_summary_publisher_ =
        nh_.advertise<std_msgs::String>("/formation_backend/subgroup_summary",
                                        1, true);
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &BackendVisualizer::timerCallback, this);
  }

 private:
  struct PairDiagnostics {
    double distance = 0.0;
    double spacing_error = 0.0;
    double min_safety_clearance = std::numeric_limits<double>::infinity();
    int first_side = 0;
    int second_side = 0;
    bool same_side = false;
    bool spacing_ok = true;
    bool clearance_ok = true;
  };

  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    if (assignment_locked_ || message->data.size() != kUavCount) return;
    std::array<int, kUavCount> drone_for_slot{{-1, -1, -1, -1}};
    for (std::size_t drone = 0; drone < kUavCount; ++drone) {
      const std::size_t slot = message->data[drone];
      if (slot >= kUavCount || drone_for_slot[slot] >= 0) return;
      drone_for_slot[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < gEdges.size(); ++edge) {
      gEdges[edge] = {{drone_for_slot[kSlotEdges[edge][0]],
                       drone_for_slot[kSlotEdges[edge][1]]}};
    }
    assignment_locked_ = true;
  }

	  std::string state() const {
	    for (std::size_t id = 0; id < kUavCount; ++id) {
	      if (!odom_seen_[id] || !group_seen_[id]) return "WAITING";
	    }
	    bool intact = true;
	    for (const auto& group : groups_) intact &= group == "INTACT";
	    if (intact) return "INTACT";
	    std::ostringstream text;
	    text << "SPLIT";
	    for (std::size_t id = 0; id < kUavCount; ++id) {
	      text << (id == 0 ? " " : ",") << id << ":" << groups_[id];
	    }
	    return text.str();
	  }

	  bool sameBackendGroup(int first, int second) const {
	    if (first < 0 || second < 0 ||
	        first >= static_cast<int>(kUavCount) ||
	        second >= static_cast<int>(kUavCount))
	      return false;
	    if (!group_seen_[first] || !group_seen_[second]) return false;
	    if (groups_[first] == "INTACT" && groups_[second] == "INTACT") return true;
	    return groups_[first] == groups_[second];
	  }

  geometry_msgs::Point midpoint(int first, int second, double dz) const {
    geometry_msgs::Point point;
    point.x = 0.5 * (positions_[first].x + positions_[second].x);
    point.y = 0.5 * (positions_[first].y + positions_[second].y);
    point.z = 0.5 * (positions_[first].z + positions_[second].z) + dz;
    return point;
  }

  double signedSafetyClearance(const geometry_msgs::Point& point) const {
    if (obstacle_shape_ == "wall") {
      const double dx = std::fabs(point.x - wall_x_) - 0.5 * wall_size_x_;
      const double dy = std::fabs(point.y - wall_y_) - 0.5 * wall_size_y_;
      const double outside_x = std::max(0.0, dx);
      const double outside_y = std::max(0.0, dy);
      const bool inside_wall_xy = dx <= 0.0 && dy <= 0.0;
      const double raw_clearance =
          inside_wall_xy ? std::max(dx, dy) : std::hypot(outside_x, outside_y);
      return raw_clearance - safety_distance_;
    }
    return std::hypot(point.x - pillar_x_, point.y - pillar_y_) -
           (pillar_radius_ + safety_distance_);
  }

  int routeSide(const geometry_msgs::Point& point) const {
    const double side_coordinate =
        obstacle_shape_ == "wall" ? point.y - wall_y_ : point.y - pillar_y_;
    if (side_coordinate > side_deadband_) return 1;
    if (side_coordinate < -side_deadband_) return -1;
    return 0;
  }

  std::string sideText(int side) const {
    if (side > 0) return "+";
    if (side < 0) return "-";
    return "0";
  }

  PairDiagnostics pairDiagnostics(int first, int second) const {
    PairDiagnostics diagnostic;
    diagnostic.distance = std::hypot(positions_[first].x - positions_[second].x,
                                     positions_[first].y - positions_[second].y);
    diagnostic.spacing_error =
        std::fabs(diagnostic.distance - desired_pair_distance_);
    diagnostic.first_side = routeSide(positions_[first]);
    diagnostic.second_side = routeSide(positions_[second]);
    diagnostic.same_side = diagnostic.first_side != 0 &&
                           diagnostic.first_side == diagnostic.second_side;
    diagnostic.spacing_ok = diagnostic.spacing_error <= max_pair_error_;
    diagnostic.min_safety_clearance = std::min(
        signedSafetyClearance(positions_[first]),
        signedSafetyClearance(positions_[second]));
    diagnostic.clearance_ok = diagnostic.min_safety_clearance >= 0.0;
    return diagnostic;
  }

  std_msgs::ColorRGBA pairColor(const PairDiagnostics& diagnostic,
                                bool split) const {
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    if (split && (!diagnostic.same_side || !diagnostic.clearance_ok)) {
      color.r = 1.0;
      return color;
    }
    if (split && !diagnostic.spacing_ok) {
      color.r = 1.0;
      color.g = 0.65;
      return color;
    }
    color.g = 0.9;
    color.b = 0.2;
    return color;
  }

  std::string formatPairText(const std::string& name,
                             const PairDiagnostics& diagnostic) const {
    std::ostringstream text;
    text << name << " d=" << std::fixed << std::setprecision(2)
         << diagnostic.distance << " e=" << diagnostic.spacing_error
         << " side=" << sideText(diagnostic.first_side)
         << "/" << sideText(diagnostic.second_side)
         << " clr=" << diagnostic.min_safety_clearance;
    if (!diagnostic.same_side) text << " SIDE";
    if (!diagnostic.spacing_ok) text << " SHAPE";
    if (!diagnostic.clearance_ok) text << " SAFE";
    return text.str();
  }

  void publishSubgroupSummary(const std::string& backend_state,
                              const PairDiagnostics& g0,
                              const PairDiagnostics& g1) {
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(4)
            << "{\"state\":\"" << backend_state << "\""
            << ",\"obstacle_shape\":\"" << obstacle_shape_ << "\""
            << ",\"g0_distance\":" << g0.distance
            << ",\"g0_spacing_error\":" << g0.spacing_error
            << ",\"g0_same_side\":" << (g0.same_side ? "true" : "false")
            << ",\"g0_side_first\":" << g0.first_side
            << ",\"g0_side_second\":" << g0.second_side
            << ",\"g0_min_safety_clearance\":" << g0.min_safety_clearance
            << ",\"g1_distance\":" << g1.distance
            << ",\"g1_spacing_error\":" << g1.spacing_error
            << ",\"g1_same_side\":" << (g1.same_side ? "true" : "false")
            << ",\"g1_side_first\":" << g1.first_side
            << ",\"g1_side_second\":" << g1.second_side
            << ",\"g1_min_safety_clearance\":" << g1.min_safety_clearance
            << "}";
    std_msgs::String message;
    message.data = summary.str();
    subgroup_summary_publisher_.publish(message);
    ROS_INFO_STREAM_THROTTLE(1.0, "[formation_backend_diag] " << message.data);
  }

	  void timerCallback(const ros::TimerEvent&) {
	    const std::string backend_state = state();
	    if (backend_state == "WAITING") return;
	    const bool split = backend_state.find("SPLIT") == 0;
    const PairDiagnostics g0 = pairDiagnostics(gEdges[0][0], gEdges[0][1]);
    const PairDiagnostics g1 = pairDiagnostics(gEdges[2][0], gEdges[2][1]);
    publishSubgroupSummary(backend_state, g0, g1);

    visualization_msgs::MarkerArray array;

	    std_msgs::Header marker_header;
	    marker_header.frame_id = frame_id_;
	    marker_header.stamp = ros::Time::now();
	    for (std::size_t edge = 0; edge < gEdges.size(); ++edge) {
	      visualization_msgs::Marker edges;
      edges.header = marker_header;
      edges.ns = "formation_backend_edges";
      edges.id = static_cast<int>(edge);
      edges.type = visualization_msgs::Marker::LINE_LIST;
      if (split && !sameBackendGroup(gEdges[edge][0], gEdges[edge][1])) {
        edges.action = visualization_msgs::Marker::DELETE;
        array.markers.push_back(edges);
        continue;
      }
      edges.action = visualization_msgs::Marker::ADD;
      edges.scale.x = 0.06;
      edges.color.a = 1.0;
      edges.color.g = 0.9;
      edges.color.b = 1.0;
      if (edge == 0) edges.color = pairColor(g0, split);
      if (edge == 2) edges.color = pairColor(g1, split);
      edges.points.push_back(positions_[gEdges[edge][0]]);
      edges.points.push_back(positions_[gEdges[edge][1]]);
      array.markers.push_back(edges);
    }

    visualization_msgs::Marker state_label;
    state_label.header = marker_header;
    state_label.ns = "formation_backend_state";
    state_label.id = 100;
    state_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    state_label.action = visualization_msgs::Marker::ADD;
    state_label.pose.orientation.w = 1.0;
    state_label.pose.position = midpoint(1, 2, 1.35);
    state_label.scale.z = 0.24;
    state_label.color.a = 1.0;
    state_label.color.g = split ? 0.8 : 1.0;
    state_label.color.b = split ? 1.0 : 0.2;
    state_label.text = "BACKEND " + backend_state;
    if (!auto_split_status_.empty())
      state_label.text += "\n" + auto_split_status_;
    array.markers.push_back(state_label);
    marker_publisher_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  std::array<ros::Subscriber, kUavCount> group_subscribers_;
  ros::Subscriber auto_split_status_subscriber_;
  ros::Subscriber assignment_subscriber_;
  ros::Publisher marker_publisher_;
  ros::Publisher subgroup_summary_publisher_;
	  ros::Timer timer_;
	  std::array<geometry_msgs::Point, kUavCount> positions_;
	  std::array<std::string, kUavCount> groups_;
  std::array<bool, kUavCount> odom_seen_{{false, false, false, false}};
  std::array<bool, kUavCount> group_seen_{{false, false, false, false}};
  std::string odom_suffix_;
  std::string frame_id_;
  std::string auto_split_status_;
  std::string obstacle_shape_ = "pillar";
  double pillar_x_ = 0.0;
  double pillar_y_ = 0.0;
  double pillar_radius_ = 1.0;
  double wall_x_ = 0.0;
  double wall_y_ = 0.0;
  double wall_size_x_ = 0.5;
  double wall_size_y_ = 5.0;
  double safety_distance_ = 0.35;
  double desired_pair_distance_ = 1.0;
  double max_pair_error_ = 0.35;
  double side_deadband_ = 0.20;
  bool assignment_locked_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_backend_visualizer_4uav");
  BackendVisualizer visualizer;
  ros::spin();
  return 0;
}

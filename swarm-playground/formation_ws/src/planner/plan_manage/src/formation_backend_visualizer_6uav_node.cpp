#include <algorithm>
#include <array>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr std::size_t kUavCount = 6;
constexpr std::size_t kEdgeCount = 7;
const std::array<std::array<int, 2>, kEdgeCount> kSlotEdges{{
    {{0, 1}}, {{1, 2}}, {{3, 4}}, {{4, 5}},
    {{0, 3}}, {{1, 4}}, {{2, 5}}}};
std::array<std::array<int, 2>, kEdgeCount> gEdges = kSlotEdges;

class SixUavBackendVisualizer {
 public:
  SixUavBackendVisualizer() : private_nh_("~") {
    private_nh_.param<std::string>("odom_topic_suffix", odom_suffix_,
                                   "visual_slam/odom");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");
    private_nh_.param("publish_rate", publish_rate_, 10.0);
    if (publish_rate_ <= 0.0) publish_rate_ = 10.0;

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
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &SixUavBackendVisualizer::assignmentCallback, this);
    marker_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
        "markers", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &SixUavBackendVisualizer::timerCallback, this);
    ROS_INFO("Six-UAV backend topology visualizer ready.");
  }

 private:
  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    const bool versioned = message->data.size() == kUavCount + 1;
    if (!versioned && message->data.size() != kUavCount) {
      ROS_ERROR_STREAM("[6UAV_VISUALIZER] Reject assignment with size "
                       << message->data.size());
      return;
    }
    const uint32_t generation = versioned ? message->data[0] : 0;
    if ((versioned && generation <= assignment_generation_) ||
        (!versioned && assignment_received_))
      return;
    const std::size_t offset = versioned ? 1 : 0;
    std::array<int, kUavCount> slot_to_drone{};
    std::array<bool, kUavCount> seen{{false, false, false, false, false, false}};
    for (std::size_t drone = 0; drone < kUavCount; ++drone) {
      const std::size_t slot = message->data[drone + offset];
      if (slot >= kUavCount || seen[slot]) {
        ROS_ERROR("[6UAV_VISUALIZER] Reject non-permutation assignment");
        return;
      }
      seen[slot] = true;
      slot_to_drone[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      gEdges[edge][0] = slot_to_drone[kSlotEdges[edge][0]];
      gEdges[edge][1] = slot_to_drone[kSlotEdges[edge][1]];
    }
    assignment_generation_ = generation;
    assignment_received_ = true;
    ROS_INFO("[6UAV_VISUALIZER] Applied formation assignment generation=%u.",
             generation);
  }

  std_msgs::ColorRGBA groupColor(const std::string& group,
                                 double alpha = 1.0) const {
    // Keep the most common component IDs visually stable across split/merge.
    static const std::array<std::array<double, 3>, 6> palette{{
        {{0.10, 0.80, 1.00}}, {{1.00, 0.35, 0.20}},
        {{0.35, 1.00, 0.30}}, {{0.95, 0.35, 1.00}},
        {{1.00, 0.85, 0.15}}, {{0.45, 0.45, 1.00}}}};
    std::size_t index = 0;
    if (group != "INTACT" && !group.empty()) {
      const char last = group.back();
      if (last >= '0' && last <= '9') {
        index = static_cast<std::size_t>(last - '0') % palette.size();
      } else {
        for (const char value : group)
          index = (index * 33 + static_cast<unsigned char>(value)) %
                  palette.size();
      }
    }
    std_msgs::ColorRGBA color;
    color.r = palette[index][0];
    color.g = palette[index][1];
    color.b = palette[index][2];
    color.a = alpha;
    return color;
  }

  bool allOdometrySeen() const {
    return std::all_of(odom_seen_.begin(), odom_seen_.end(),
                       [](bool value) { return value; });
  }

  bool allGroupsSeen() const {
    return std::all_of(group_seen_.begin(), group_seen_.end(),
                       [](bool value) { return value; });
  }

  bool sameGroup(int first, int second) const {
    if (!group_seen_[first] || !group_seen_[second]) return false;
    if (groups_[first] == "INTACT" && groups_[second] == "INTACT") return true;
    return !groups_[first].empty() && groups_[first] == groups_[second];
  }

  geometry_msgs::Point interpolatedPoint(int first, int second, double ratio,
                                         double dz = 0.0) const {
    geometry_msgs::Point point;
    point.x = positions_[first].x * (1.0 - ratio) + positions_[second].x * ratio;
    point.y = positions_[first].y * (1.0 - ratio) + positions_[second].y * ratio;
    point.z = positions_[first].z * (1.0 - ratio) + positions_[second].z * ratio +
              dz;
    return point;
  }

  geometry_msgs::Point formationCenter(double dz = 0.0) const {
    geometry_msgs::Point center;
    for (const auto& position : positions_) {
      center.x += position.x / static_cast<double>(kUavCount);
      center.y += position.y / static_cast<double>(kUavCount);
      center.z += position.z / static_cast<double>(kUavCount);
    }
    center.z += dz;
    return center;
  }

  std::string topologyText() const {
    if (!allGroupsSeen()) return "WAITING FOR GROUP STATE";
    const bool all_intact =
        std::all_of(groups_.begin(), groups_.end(),
                    [](const std::string& group) { return group == "INTACT"; });
    if (all_intact) return "INTACT {0,1,2,3,4,5}";

    const bool any_intact =
        std::any_of(groups_.begin(), groups_.end(),
                    [](const std::string& group) { return group == "INTACT"; });
    std::map<std::string, std::vector<int>> members;
    for (std::size_t id = 0; id < kUavCount; ++id)
      members[groups_[id]].push_back(static_cast<int>(id));

    std::ostringstream text;
    text << (any_intact ? "TRANSITION " : "SPLIT ");
    bool first_group = true;
    for (const auto& entry : members) {
      if (!first_group) text << "  ";
      text << (entry.first.empty() ? "?" : entry.first) << "={";
      for (std::size_t index = 0; index < entry.second.size(); ++index) {
        if (index) text << ",";
        text << entry.second[index];
      }
      text << "}";
      first_group = false;
    }
    return text.str();
  }

  void addEdgeMarkers(const std_msgs::Header& header,
                      visualization_msgs::MarkerArray* output) const {
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const int first = gEdges[edge][0];
      const int second = gEdges[edge][1];
      const bool known = group_seen_[first] && group_seen_[second];
      const bool connected = sameGroup(first, second);

      visualization_msgs::Marker line;
      line.header = header;
      line.ns = connected
                    ? "six_uav_backend_connected_edges"
                    : (known ? "six_uav_backend_cut_edges"
                             : "six_uav_backend_unknown_edges");
      line.id = static_cast<int>(edge);
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = connected ? 0.085 : 0.045;
      line.color.a = connected ? 1.0 : 0.8;
      line.color.r = connected ? 0.15 : (known ? 1.0 : 0.65);
      line.color.g = connected ? 0.95 : (known ? 0.10 : 0.65);
      line.color.b = connected ? 0.95 : (known ? 0.05 : 0.65);
      if (connected) {
        line.points.push_back(interpolatedPoint(first, second, 0.0));
        line.points.push_back(interpolatedPoint(first, second, 1.0));
      } else {
        // Dashed red segments keep a cut edge visible without suggesting an
        // active formation constraint.
        for (int dash = 0; dash < 4; ++dash) {
          const double begin = 0.25 * dash;
          line.points.push_back(interpolatedPoint(first, second, begin, 0.02));
          line.points.push_back(
              interpolatedPoint(first, second, begin + 0.14, 0.02));
        }
      }
      output->markers.push_back(line);
    }
  }

  void addUavMarkers(const std_msgs::Header& header,
                     visualization_msgs::MarkerArray* output) const {
    for (std::size_t id = 0; id < kUavCount; ++id) {
      const std::string group = group_seen_[id] ? groups_[id] : "?";
      visualization_msgs::Marker halo;
      halo.header = header;
      halo.ns = "six_uav_backend_group_halos";
      halo.id = static_cast<int>(id);
      halo.type = visualization_msgs::Marker::SPHERE;
      halo.action = visualization_msgs::Marker::ADD;
      halo.pose.orientation.w = 1.0;
      halo.pose.position = positions_[id];
      halo.pose.position.z -= 0.08;
      halo.scale.x = 0.46;
      halo.scale.y = 0.46;
      halo.scale.z = 0.08;
      halo.color = groupColor(group, 0.75);
      output->markers.push_back(halo);

      visualization_msgs::Marker label;
      label.header = header;
      label.ns = "six_uav_backend_group_labels";
      label.id = static_cast<int>(id);
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.orientation.w = 1.0;
      label.pose.position = positions_[id];
      label.pose.position.z += 0.42;
      label.scale.z = 0.22;
      label.color = groupColor(group);
      label.text = "U" + std::to_string(id) + "  " + group;
      output->markers.push_back(label);
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!allOdometrySeen()) {
      ROS_WARN_THROTTLE(
          2.0, "Six-UAV backend visualizer is waiting for six odometry topics.");
      return;
    }

    visualization_msgs::MarkerArray output;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    output.markers.push_back(clear);

    std_msgs::Header header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();
    addEdgeMarkers(header, &output);
    addUavMarkers(header, &output);

    visualization_msgs::Marker state_label;
    state_label.header = header;
    state_label.ns = "six_uav_backend_state";
    state_label.id = 0;
    state_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    state_label.action = visualization_msgs::Marker::ADD;
    state_label.pose.orientation.w = 1.0;
    state_label.pose.position = formationCenter(0.90);
    state_label.scale.z = 0.24;
    state_label.color.a = 1.0;
    state_label.color.r = 1.0;
    state_label.color.g = allGroupsSeen() ? 0.9 : 0.55;
    state_label.color.b = allGroupsSeen() ? 0.2 : 0.1;
    state_label.text = "APPLIED " + topologyText() + "  G" +
                       std::to_string(assignment_generation_);
    output.markers.push_back(state_label);

    marker_publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::array<ros::Subscriber, kUavCount> odom_subscribers_;
  std::array<ros::Subscriber, kUavCount> group_subscribers_;
  ros::Subscriber assignment_subscriber_;
  ros::Publisher marker_publisher_;
  ros::Timer timer_;
  std::array<geometry_msgs::Point, kUavCount> positions_{};
  std::array<std::string, kUavCount> groups_{};
  std::array<bool, kUavCount> odom_seen_{{false, false, false,
                                         false, false, false}};
  std::array<bool, kUavCount> group_seen_{{false, false, false,
                                          false, false, false}};
  std::string odom_suffix_;
  std::string frame_id_;
  double publish_rate_{10.0};
  uint32_t assignment_generation_{0};
  bool assignment_received_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_backend_visualizer_6uav");
  SixUavBackendVisualizer visualizer;
  ros::spin();
  return 0;
}

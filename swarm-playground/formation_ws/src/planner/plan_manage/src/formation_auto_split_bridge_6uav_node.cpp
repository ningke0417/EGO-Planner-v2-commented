#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ego_planner/FormationTopologyAck.h>
#include <ego_planner/FormationTopologyCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

namespace {

constexpr std::size_t kUavCount = 6;
constexpr std::size_t kEdgeCount = 7;
const std::array<std::array<int, 2>, kEdgeCount> kSlotEdges{{
    {{0, 1}}, {{1, 2}}, {{3, 4}}, {{4, 5}},
    {{0, 3}}, {{1, 4}}, {{2, 5}}}};
std::array<std::array<int, 2>, kEdgeCount> gEdges = kSlotEdges;
const char kAllUavComponent[] = "{0,1,2,3,4,5}";

class SixUavAutoSplitBridge {
 public:
  SixUavAutoSplitBridge() : private_nh_("~") {
    private_nh_.param<std::string>("criterion_summary_topic",
                                   criterion_summary_topic_,
                                   "/starconvex_shadow_6uav/summary");
    private_nh_.param("criterion_timeout", criterion_timeout_, 2.0);
    private_nh_.param("split_ready_persistence", split_ready_persistence_, 0.4);
    private_nh_.param("topology_change_persistence",
                      topology_change_persistence_, 0.4);
    private_nh_.param("auto_merge_enabled", auto_merge_enabled_, true);
    private_nh_.param("merge_max_edge_distance",
                      merge_max_edge_distance_, 3.0);
    private_nh_.param("merge_ready_persistence",
                      merge_ready_persistence_, 0.3);
    private_nh_.param("weak_reconnect_enabled",
                      weak_reconnect_enabled_, true);
    private_nh_.param("weak_reconnect_ready_persistence",
                      weak_reconnect_ready_persistence_, 0.3);
    private_nh_.param("weak_reconnect_visibility_threshold",
                      weak_reconnect_visibility_threshold_, 1.0);
    private_nh_.param("weak_reconnect_max_edge_distance",
                      weak_reconnect_max_edge_distance_, 3.00);
    private_nh_.param("weak_reconnect_formation_weight_scale",
                      weak_reconnect_formation_weight_scale_, 0.22);
    private_nh_.param("weak_reconnect_mid_formation_weight_scale",
                      weak_reconnect_mid_formation_weight_scale_, 0.25);
    private_nh_.param("weak_reconnect_final_formation_weight_scale",
                      weak_reconnect_final_formation_weight_scale_, 0.30);
    private_nh_.param("weak_reconnect_ramp_step_duration",
                      weak_reconnect_ramp_step_duration_, 0.5);
    private_nh_.param("split_formation_weight_scale",
                      split_formation_weight_scale_, 0.15);
    private_nh_.param("split_progress_sync_weight_scale",
                      split_progress_sync_weight_scale_, 0.0);
    private_nh_.param("secondary_hard_split_threshold",
                      secondary_hard_split_threshold_, 0.20);
    private_nh_.param("secondary_distance_split_threshold",
                      secondary_distance_split_threshold_, 1.80);
    private_nh_.param("ack_timeout", ack_timeout_, 2.0);
    if (criterion_timeout_ <= 0.0 || split_ready_persistence_ < 0.0 ||
        topology_change_persistence_ < 0.0 || merge_max_edge_distance_ <= 0.0 ||
        merge_ready_persistence_ < 0.0 ||
        weak_reconnect_ready_persistence_ < 0.0 ||
        weak_reconnect_visibility_threshold_ < 0.0 ||
        weak_reconnect_visibility_threshold_ > 1.0 ||
        weak_reconnect_max_edge_distance_ <= 0.0 ||
        weak_reconnect_formation_weight_scale_ < 0.0 ||
        weak_reconnect_mid_formation_weight_scale_ <
            weak_reconnect_formation_weight_scale_ ||
        weak_reconnect_final_formation_weight_scale_ <
            weak_reconnect_mid_formation_weight_scale_ ||
        weak_reconnect_ramp_step_duration_ <= 0.0 ||
        split_formation_weight_scale_ < 0.0 ||
        split_progress_sync_weight_scale_ < 0.0 ||
        secondary_hard_split_threshold_ < 0.0 ||
        secondary_hard_split_threshold_ > 1.0 ||
        secondary_distance_split_threshold_ <= 0.0 || ack_timeout_ <= 0.0) {
      throw std::runtime_error("invalid six-UAV auto-split parameters");
    }

    summary_subscriber_ = nh_.subscribe<std_msgs::String>(
        criterion_summary_topic_, 1,
        &SixUavAutoSplitBridge::summaryCallback, this);
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &SixUavAutoSplitBridge::assignmentCallback, this);
    assignment_ack_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/assignment_ack", 30,
        &SixUavAutoSplitBridge::assignmentAckCallback, this);
    ack_subscriber_ = nh_.subscribe<ego_planner::FormationTopologyAck>(
        "/formation_backend/topology_ack", 30,
        &SixUavAutoSplitBridge::ackCallback, this);
    command_publisher_ =
        nh_.advertise<ego_planner::FormationTopologyCommand>(
            "/formation_backend/topology_command", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/formation_backend/auto_split_status", 1, true);
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &SixUavAutoSplitBridge::timerCallback, this);
    next_epoch_ = ros::Time::now().toNSec();
    issueCommand(ego_planner::FormationTopologyCommand::INTACT, "INTACT",
                 "SIX_UAV_BRIDGE_STARTUP");
    publishStatus("6UAV AUTO_SPLIT ARMED");
    ROS_INFO_STREAM("[6UAV_AUTO_SPLIT] Armed. split_scale="
                    << split_formation_weight_scale_
                    << " merge_source=observed_known_free_corridor"
                    << " merge_max_edge_distance="
                    << merge_max_edge_distance_
                    << " weak_reconnect="
                    << (weak_reconnect_enabled_ ? "enabled" : "disabled")
                    << " weak_visibility_threshold="
                    << weak_reconnect_visibility_threshold_
                    << " weak_max_edge_distance="
                    << weak_reconnect_max_edge_distance_
                    << " weak_visibility_hold="
                    << weak_reconnect_ready_persistence_
                    << " weak_scale="
                    << weak_reconnect_formation_weight_scale_
                    << "->" << weak_reconnect_mid_formation_weight_scale_
                    << "->" << weak_reconnect_final_formation_weight_scale_
                    << " ramp_step=" << weak_reconnect_ramp_step_duration_
                    << " s");
  }

 private:
  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    const bool versioned = message->data.size() == kUavCount + 1;
    if (!versioned && message->data.size() != kUavCount) {
      ROS_ERROR_STREAM("[6UAV_AUTO_SPLIT] Reject assignment with size "
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
        ROS_ERROR("[6UAV_AUTO_SPLIT] Reject non-permutation assignment");
        return;
      }
      seen[slot] = true;
      slot_to_drone[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      gEdges[edge][0] = slot_to_drone[kSlotEdges[edge][0]];
      gEdges[edge][1] = slot_to_drone[kSlotEdges[edge][1]];
    }
    criterion_edge_metrics_valid_ = false;
    candidate_components_.clear();
    candidate_since_ = ros::Time();
    assignment_generation_ = generation;
    assignment_received_ = true;
    assignment_acknowledgements_.fill(false);
    ROS_WARN_STREAM("[6UAV_AUTO_SPLIT] assignment generation=" << generation
                    << " mapped slot topology to physical edges: "
                    << gEdges[0][0] << gEdges[0][1] << ","
                    << gEdges[1][0] << gEdges[1][1] << ","
                    << gEdges[2][0] << gEdges[2][1] << ","
                    << gEdges[3][0] << gEdges[3][1] << ","
                    << gEdges[4][0] << gEdges[4][1] << ","
                    << gEdges[5][0] << gEdges[5][1] << ","
                    << gEdges[6][0] << gEdges[6][1]);
  }

  void assignmentAckCallback(
      const std_msgs::UInt32MultiArray::ConstPtr& message) {
    if (message->data.size() != 2 ||
        message->data[0] != assignment_generation_ ||
        message->data[1] >= kUavCount)
      return;
    assignment_acknowledgements_[message->data[1]] = true;
  }

  bool assignmentConfirmed() const {
    return assignment_received_ &&
        std::all_of(assignment_acknowledgements_.begin(),
                    assignment_acknowledgements_.end(),
                    [](bool value) { return value; });
  }

  std::string extractString(const std::string& text,
                            const std::string& key) const {
    const std::string needle = "\"" + key + "\":\"";
    const std::size_t begin = text.find(needle);
    if (begin == std::string::npos) return "";
    const std::size_t value_begin = begin + needle.size();
    const std::size_t value_end = text.find('"', value_begin);
    if (value_end == std::string::npos) return "";
    return text.substr(value_begin, value_end - value_begin);
  }

  bool extractNumber(const std::string& text, const std::string& key,
                     double* value) const {
    const std::string needle = "\"" + key + "\":";
    const std::size_t begin = text.find(needle);
    if (begin == std::string::npos) return false;
    const std::size_t value_begin = begin + needle.size();
    std::size_t value_end = text.find(',', value_begin);
    if (value_end == std::string::npos) value_end = text.find('}', value_begin);
    if (value_end == std::string::npos || value_end <= value_begin) return false;
    try {
      *value = std::stod(text.substr(value_begin, value_end - value_begin));
    } catch (const std::exception&) {
      return false;
    }
    return std::isfinite(*value);
  }

  bool parseComponents(const std::string& text,
                       std::string* canonical) const {
    if (text.empty() || text == "INTACT") return false;
    std::array<bool, kUavCount> seen{{false, false, false, false, false, false}};
    std::vector<std::vector<int>> groups;
    std::size_t cursor = 0;
    while (cursor < text.size()) {
      if (text[cursor] != '{') return false;
      const std::size_t close = text.find('}', cursor + 1);
      if (close == std::string::npos) return false;
      const std::string body = text.substr(cursor + 1, close - cursor - 1);
      if (body.empty()) return false;
      std::vector<int> group;
      std::stringstream stream(body);
      std::string token;
      while (std::getline(stream, token, ',')) {
        try {
          std::size_t used = 0;
          const int id = std::stoi(token, &used);
          if (used != token.size() || id < 0 ||
              id >= static_cast<int>(kUavCount) || seen[id])
            return false;
          seen[id] = true;
          group.push_back(id);
        } catch (const std::exception&) {
          return false;
        }
      }
      std::sort(group.begin(), group.end());
      groups.push_back(group);
      cursor = close + 1;
      if (cursor == text.size()) break;
      if (text[cursor] != '|') return false;
      ++cursor;
    }
    if (!std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }) ||
        groups.size() < 2)
      return false;
    std::sort(groups.begin(), groups.end(),
              [](const std::vector<int>& first,
                 const std::vector<int>& second) {
                return first.front() < second.front();
              });
    std::ostringstream output;
    for (std::size_t group = 0; group < groups.size(); ++group) {
      if (group) output << "|";
      output << "{";
      for (std::size_t index = 0; index < groups[group].size(); ++index) {
        if (index) output << ",";
        output << groups[group][index];
      }
      output << "}";
    }
    *canonical = output.str();
    return true;
  }

  bool componentAssignments(
      const std::string& text,
      std::array<int, kUavCount>* assignments) const {
    assignments->fill(-1);
    int group = 0;
    std::size_t cursor = 0;
    while (cursor < text.size()) {
      if (text[cursor] != '{') return false;
      const std::size_t close = text.find('}', cursor + 1);
      if (close == std::string::npos) return false;
      std::stringstream stream(text.substr(cursor + 1, close - cursor - 1));
      std::string token;
      while (std::getline(stream, token, ',')) {
        try {
          std::size_t used = 0;
          const int id = std::stoi(token, &used);
          if (used != token.size() || id < 0 ||
              id >= static_cast<int>(kUavCount) || (*assignments)[id] >= 0)
            return false;
          (*assignments)[id] = group;
        } catch (const std::exception&) {
          return false;
        }
      }
      ++group;
      cursor = close + 1;
      if (cursor == text.size()) break;
      if (text[cursor] != '|') return false;
      ++cursor;
    }
    return std::all_of(assignments->begin(), assignments->end(),
                       [](int value) { return value >= 0; });
  }

  std::string componentsFromEdges(
      const std::array<bool, kEdgeCount>& connected) const {
    std::array<int, kUavCount> parent{{0, 1, 2, 3, 4, 5}};
    const auto find_root = [&parent](int value) {
      int root = value;
      while (parent[root] != root) root = parent[root];
      while (parent[value] != value) {
        const int next = parent[value];
        parent[value] = root;
        value = next;
      }
      return root;
    };
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (!connected[edge]) continue;
      const int first_root = find_root(gEdges[edge][0]);
      const int second_root = find_root(gEdges[edge][1]);
      if (first_root != second_root) parent[second_root] = first_root;
    }
    std::vector<std::vector<int>> components;
    std::array<int, kUavCount> component_index{{-1, -1, -1, -1, -1, -1}};
    for (int id = 0; id < static_cast<int>(kUavCount); ++id) {
      const int root = find_root(id);
      if (component_index[root] < 0) {
        component_index[root] = static_cast<int>(components.size());
        components.push_back({});
      }
      components[component_index[root]].push_back(id);
    }
    std::ostringstream output;
    for (std::size_t component = 0; component < components.size(); ++component) {
      if (component) output << "|";
      output << "{";
      for (std::size_t index = 0; index < components[component].size(); ++index) {
        if (index) output << ",";
        output << components[component][index];
      }
      output << "}";
    }
    return output.str();
  }

  std::string topologyWithSecondaryCutGuard(
      const std::string& candidate) const {
    std::array<int, kUavCount> active_groups;
    std::array<int, kUavCount> candidate_groups;
    if (!criterion_edge_metrics_valid_ ||
        !componentAssignments(active_command_.components, &active_groups) ||
        !componentAssignments(candidate, &candidate_groups))
      return active_command_.components;

    std::array<bool, kEdgeCount> effective_edges{};
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const int first = gEdges[edge][0];
      const int second = gEdges[edge][1];
      const bool active_connected = active_groups[first] == active_groups[second];
      const bool candidate_connected =
          candidate_groups[first] == candidate_groups[second];
      if (!active_connected || candidate_connected) {
        // Recovered edges are allowed immediately; unchanged cut edges stay cut.
        effective_edges[edge] = candidate_connected;
        continue;
      }
      const bool hard_cut =
          edge_scores_[edge] <= secondary_hard_split_threshold_ ||
          edge_clearances_[edge] <= 0.0 || edge_visibility_[edge] <= 0.0 ||
          edge_distances_[edge] >= secondary_distance_split_threshold_;
      effective_edges[edge] = !hard_cut;
      if (!hard_cut) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[6UAV_TOPOLOGY_GUARD] Preserve edge "
                     << gEdges[edge][0] << "-" << gEdges[edge][1]
                     << " during secondary split: S=" << edge_scores_[edge]
                     << " clearance=" << edge_clearances_[edge]
                     << " visibility=" << edge_visibility_[edge]
                     << " distance=" << edge_distances_[edge]);
      }
    }
    return componentsFromEdges(effective_edges);
  }

  void summaryCallback(const std_msgs::String::ConstPtr& message) {
    const std::string state = extractString(message->data, "state");
    const std::string components = extractString(message->data, "components");
    double generation_value = 0.0;
    double center_x = 0.0;
    double max_edge_distance = 0.0;
    if (state.empty() ||
        !extractNumber(message->data, "assignment_generation",
                       &generation_value) ||
        generation_value < 0.0 ||
        generation_value >
            static_cast<double>(std::numeric_limits<uint32_t>::max()) ||
        generation_value != std::floor(generation_value) ||
        !extractNumber(message->data, "center_x", &center_x) ||
        !extractNumber(message->data, "max_edge_distance",
                       &max_edge_distance)) {
      ROS_WARN_THROTTLE(2.0, "[6UAV_AUTO_SPLIT] Invalid criterion summary.");
      return;
    }
    const uint32_t summary_generation =
        static_cast<uint32_t>(generation_value);
    if (!assignment_received_ || summary_generation != assignment_generation_) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[6UAV_AUTO_SPLIT] Waiting for criterion generation "
                   << assignment_generation_ << "; received generation "
                   << summary_generation);
      return;
    }
    std::string canonical = "INTACT";
    if (state == "SPLIT" && !parseComponents(components, &canonical)) {
      ROS_WARN_THROTTLE(2.0,
                        "[6UAV_AUTO_SPLIT] Invalid split components: %s",
                        components.c_str());
      return;
    }
    criterion_state_ = state;
    criterion_components_ = canonical;
    center_x_ = center_x;
    max_edge_distance_ = max_edge_distance;
    criterion_edge_metrics_valid_ = true;
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const std::string name = std::to_string(gEdges[edge][0]) +
                               std::to_string(gEdges[edge][1]);
      criterion_edge_metrics_valid_ =
          extractNumber(message->data, "score_" + name,
                        &edge_scores_[edge]) &&
          extractNumber(message->data, "clearance_" + name,
                        &edge_clearances_[edge]) &&
          extractNumber(message->data, "visibility_weight_" + name,
                        &edge_visibility_[edge]) &&
          extractNumber(message->data, "distance_" + name,
                        &edge_distances_[edge]) &&
          criterion_edge_metrics_valid_;
    }
    criterion_assignment_generation_ = summary_generation;
    criterion_stamp_ = ros::Time::now();
  }

  void ackCallback(
      const ego_planner::FormationTopologyAck::ConstPtr& message) {
    if (message->epoch != active_command_.epoch ||
        message->drone_id >= kUavCount ||
        message->mode != active_command_.mode ||
        message->components != active_command_.components ||
        std::fabs(message->formation_scale -
                  active_command_.formation_scale) > 1e-6 ||
        std::fabs(message->progress_sync_scale -
                  active_command_.progress_sync_scale) > 1e-6)
      return;
    acknowledgements_[message->drone_id] = true;
    if (commandConfirmed()) {
      std::ostringstream status;
      status << "6UAV "
             << (active_command_.mode ==
                         ego_planner::FormationTopologyCommand::SPLIT
                     ? "SPLIT"
                     : "INTACT")
             << " CONFIRMED epoch=" << active_command_.epoch
             << " components=" << active_command_.components;
      publishStatus(status.str());
      ROS_INFO_STREAM("[6UAV_AUTO_SPLIT] " << status.str());
    }
  }

  bool commandConfirmed() const {
    return std::all_of(acknowledgements_.begin(), acknowledgements_.end(),
                       [](bool value) { return value; });
  }

  void issueCommand(uint8_t mode, const std::string& components,
                    const std::string& reason,
                    double formation_scale = -1.0) {
    if (mode == ego_planner::FormationTopologyCommand::SPLIT &&
        reason == "CRITERION_SPLIT" &&
        active_command_.mode == ego_planner::FormationTopologyCommand::INTACT) {
      reassignment_completed_for_split_cycle_ = false;
    }
    active_command_.header.stamp = ros::Time::now();
    active_command_.epoch = ++next_epoch_;
    active_command_.mode = mode;
    active_command_.components = components;
    active_command_.formation_scale =
        mode == ego_planner::FormationTopologyCommand::SPLIT
            ? (formation_scale >= 0.0 ? formation_scale
                                      : split_formation_weight_scale_)
            : 0.0;
    active_command_.progress_sync_scale =
        mode == ego_planner::FormationTopologyCommand::SPLIT
            ? split_progress_sync_weight_scale_
            : 0.0;
    active_command_.reason = reason;
    acknowledgements_.fill(false);
    command_stamp_ = ros::Time::now();
    last_command_publish_ = command_stamp_;
    command_publisher_.publish(active_command_);
    candidate_components_.clear();
    candidate_since_ = ros::Time(0);
    merge_ready_since_ = ros::Time(0);
    weak_reconnect_ready_since_ = ros::Time(0);
    std::ostringstream status;
    status << "6UAV COMMAND "
           << (mode == ego_planner::FormationTopologyCommand::SPLIT
                   ? "SPLIT"
                   : "INTACT")
           << " epoch=" << active_command_.epoch
           << " components=" << components << " reason=" << reason;
    publishStatus(status.str());
    ROS_WARN_STREAM("[6UAV_AUTO_SPLIT] " << status.str());
  }

  void updateCandidate(const std::string& components, const ros::Time& now) {
    if (candidate_components_ == components && !candidate_since_.isZero())
      return;
    candidate_components_ = components;
    candidate_since_ = now;
  }

  void clearCandidate() {
    candidate_components_.clear();
    candidate_since_ = ros::Time(0);
  }

  bool weakReconnectReady() const {
    if (!criterion_edge_metrics_valid_) return false;
    std::array<int, kUavCount> groups;
    if (!componentAssignments(active_command_.components, &groups))
      return false;

    bool has_cross_group_edge = false;
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const int first = gEdges[edge][0];
      const int second = gEdges[edge][1];
      if (groups[first] == groups[second]) continue;
      has_cross_group_edge = true;
      if (edge_visibility_[edge] + 1e-9 <
          weak_reconnect_visibility_threshold_) {
        return false;
      }
      if (edge_distances_[edge] > weak_reconnect_max_edge_distance_) {
        return false;
      }
    }
    return has_cross_group_edge;
  }

  bool anyHardUnsafeEdge() const {
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (edge_visibility_[edge] <= 0.0 || edge_clearances_[edge] <= 0.0)
        return true;
    }
    return false;
  }

  bool finalVisibilityMergeReady() const {
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (edge_visibility_[edge] + 1e-9 <
              weak_reconnect_visibility_threshold_ ||
          edge_distances_[edge] > merge_max_edge_distance_) {
        return false;
      }
    }
    return true;
  }

  void leaveWeakRecovery(const std::string& components,
                         const std::string& reason) {
    weak_recovery_active_ = false;
    weak_recovery_ramp_since_ = ros::Time(0);
    weak_reconnect_ready_since_ = ros::Time(0);
    const std::string fallback = components.empty()
                                     ? weak_recovery_source_components_
                                     : components;
    weak_recovery_source_components_.clear();
    issueCommand(ego_planner::FormationTopologyCommand::SPLIT,
                 fallback, reason);
  }

  bool handleWeakRecovery(const ros::Time& now) {
    if (!weak_recovery_active_) return false;

    if (assignment_generation_ < minimum_merge_assignment_generation_ ||
        !assignmentConfirmed()) {
      publishStatus("6UAV WEAK_RECOVERY WAIT_REASSIGNMENT_ACK");
      return true;
    }
    awaiting_merge_assignment_ = false;
    reassignment_completed_for_split_cycle_ = true;

    // The assignment and criterion summaries travel on separate ROS topics.
    // After remapping gEdges, the most recent summary can still describe the
    // previous generation.  Waiting here avoids interpreting "no matching
    // metrics yet" as a hard visibility failure and immediately rolling back.
    if (!criterion_edge_metrics_valid_ ||
        criterion_assignment_generation_ != assignment_generation_) {
      publishStatus("6UAV WEAK_RECOVERY WAIT_MATCHING_CRITERION");
      return true;
    }

    if (anyHardUnsafeEdge()) {
      std::string fallback;
      if (criterion_state_ == "SPLIT")
        parseComponents(criterion_components_, &fallback);
      ROS_WARN_STREAM(
          "[6UAV_WEAK_RECOVERY] Hard visibility loss; restoring topology "
          << (fallback.empty() ? weak_recovery_source_components_ : fallback));
      leaveWeakRecovery(fallback, "WEAK_RECOVERY_HARD_VISIBILITY_ROLLBACK");
      return true;
    }

    // Once reassignment is acknowledged, merge from visibility and distance
    // directly.  Waiting for criterion_state_ == INTACT would also require
    // every StarConvex overlap score to reach merge_threshold_ and every cut
    // edge to shrink to distance_merge_threshold_, even though all formation
    // edges are already collision-free and mutually visible.
    if (!finalVisibilityMergeReady()) {
      merge_ready_since_ = ros::Time(0);
      publishStatus("6UAV WEAK_RECOVERY ACTIVE WAIT_FINAL_VISIBILITY_GATE");
      return true;
    }
    if (merge_ready_since_.isZero()) {
      merge_ready_since_ = now;
      publishStatus("6UAV WEAK_RECOVERY FINAL_HOLD");
      return true;
    }
    if ((now - merge_ready_since_).toSec() < merge_ready_persistence_) {
      publishStatus("6UAV WEAK_RECOVERY FINAL_HOLD");
      return true;
    }

    weak_recovery_active_ = false;
    weak_recovery_ramp_since_ = ros::Time(0);
    weak_recovery_source_components_.clear();
    issueCommand(ego_planner::FormationTopologyCommand::INTACT, "INTACT",
                 "VISIBILITY_MERGED_AFTER_WEAK_RECOVERY");
    return true;
  }

  bool maybeStartWeakRecovery(const ros::Time& now) {
    if (!weak_reconnect_enabled_ || !auto_merge_enabled_ ||
        active_command_.mode != ego_planner::FormationTopologyCommand::SPLIT ||
        active_command_.components == kAllUavComponent) {
      weak_reconnect_ready_since_ = ros::Time(0);
      return false;
    }

    if (!weakReconnectReady()) {
      weak_reconnect_ready_since_ = ros::Time(0);
      return false;
    }

    if (weak_reconnect_ready_since_.isZero()) {
      weak_reconnect_ready_since_ = now;
      publishStatus("6UAV WEAK_RECOVERY READY_HOLD");
      return true;
    }
    if ((now - weak_reconnect_ready_since_).toSec() <
        weak_reconnect_ready_persistence_) {
      publishStatus("6UAV WEAK_RECOVERY READY_HOLD");
      return true;
    }

    weak_recovery_source_components_ = active_command_.components;
    weak_recovery_active_ = true;
    weak_recovery_ramp_since_ = ros::Time(0);
    minimum_merge_assignment_generation_ =
        assignment_generation_ +
        (reassignment_completed_for_split_cycle_ ? 0u : 1u);
    awaiting_merge_assignment_ = !reassignment_completed_for_split_cycle_;
    issueCommand(ego_planner::FormationTopologyCommand::SPLIT,
                 kAllUavComponent, "VISIBILITY_WEAK_RECONNECT",
                 weak_reconnect_formation_weight_scale_);
    publishStatus("6UAV WEAK_RECOVERY COMMAND_SENT");
    ROS_WARN_STREAM(
        "[6UAV_WEAK_RECOVERY] All cross-group edges remained visible and "
        "close enough for "
        << weak_reconnect_ready_persistence_
        << " s; enabling one-component weak formation scale="
        << weak_reconnect_formation_weight_scale_
        << " source=" << weak_recovery_source_components_);
    return true;
  }

  void evaluateLegacyDirectMerge(const ros::Time& now,
                                 bool backend_split) {
    if (weak_reconnect_enabled_ || !backend_split || !auto_merge_enabled_ ||
        criterion_state_ != "INTACT") {
      merge_ready_since_ = ros::Time(0);
      return;
    }
    if (max_edge_distance_ > merge_max_edge_distance_) {
      merge_ready_since_ = ros::Time(0);
      return;
    }
    if (merge_ready_since_.isZero()) merge_ready_since_ = now;
    if ((now - merge_ready_since_).toSec() >= merge_ready_persistence_) {
      issueCommand(ego_planner::FormationTopologyCommand::INTACT, "INTACT",
                   "CRITERION_MERGED_AFTER_WALL");
    }
  }

  void evaluateCriterion(const ros::Time& now) {
    if (criterion_stamp_.isZero() ||
        (now - criterion_stamp_).toSec() > criterion_timeout_) {
      clearCandidate();
      merge_ready_since_ = ros::Time(0);
      weak_reconnect_ready_since_ = ros::Time(0);
      if (weak_recovery_active_ && commandConfirmed()) {
        ROS_ERROR("[6UAV_WEAK_RECOVERY] Criterion timed out; restoring split topology.");
        leaveWeakRecovery("", "WEAK_RECOVERY_CRITERION_TIMEOUT");
      }
      return;
    }
    if (!commandConfirmed()) return;

    const bool backend_split =
        active_command_.mode == ego_planner::FormationTopologyCommand::SPLIT;
    if (handleWeakRecovery(now)) return;

    if (backend_split && maybeStartWeakRecovery(now)) return;

    if (criterion_state_ == "SPLIT") {
      merge_ready_since_ = ros::Time(0);
      const std::string desired_components =
          backend_split
              ? topologyWithSecondaryCutGuard(criterion_components_)
              : criterion_components_;
      // A one-component SPLIT command is the weak-recovery mode.  Do not let
      // ordinary component tracking enter it without the explicit visibility,
      // distance, wall and persistence gates above.
      if (weak_reconnect_enabled_ && backend_split &&
          desired_components == kAllUavComponent) {
        clearCandidate();
        publishStatus("6UAV WEAK_RECOVERY WAIT_SAFE_GATE");
        return;
      }
      if (backend_split && desired_components == active_command_.components) {
        clearCandidate();
        return;
      }
      updateCandidate(desired_components, now);
      const double required = backend_split ? topology_change_persistence_
                                            : split_ready_persistence_;
      if ((now - candidate_since_).toSec() >= required) {
        issueCommand(ego_planner::FormationTopologyCommand::SPLIT,
                     desired_components,
                     backend_split ? "CRITERION_TOPOLOGY_CHANGED"
                                   : "CRITERION_SPLIT");
      }
      return;
    }

    clearCandidate();
    evaluateLegacyDirectMerge(now, backend_split);
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!commandConfirmed() &&
        (now - last_command_publish_).toSec() >= 0.5) {
      active_command_.header.stamp = now;
      command_publisher_.publish(active_command_);
      last_command_publish_ = now;
    }
    if (!commandConfirmed() &&
        (now - command_stamp_).toSec() >= ack_timeout_) {
      std::ostringstream missing;
      for (std::size_t id = 0; id < kUavCount; ++id) {
        if (!acknowledgements_[id]) missing << (missing.tellp() > 0 ? "," : "") << id;
      }
      ROS_ERROR_THROTTLE(
          1.0,
          "[6UAV_AUTO_SPLIT] Waiting for topology ACK epoch=%lu missing=%s",
          static_cast<unsigned long>(active_command_.epoch),
          missing.str().c_str());
    }
    evaluateCriterion(now);
  }

  void publishStatus(const std::string& value) {
    if (value == last_status_) return;
    last_status_ = value;
    std_msgs::String message;
    message.data = value;
    status_publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber summary_subscriber_;
  ros::Subscriber ack_subscriber_;
  ros::Subscriber assignment_subscriber_;
  ros::Subscriber assignment_ack_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher status_publisher_;
  ros::Timer timer_;
  std::string criterion_summary_topic_;
  std::string criterion_state_;
  std::string criterion_components_{"INTACT"};
  std::string candidate_components_;
  std::string last_status_;
  double center_x_{0.0};
  double max_edge_distance_{0.0};
  double criterion_timeout_{2.0};
  double split_ready_persistence_{0.4};
  double topology_change_persistence_{0.4};
  bool auto_merge_enabled_{true};
  double merge_max_edge_distance_{3.0};
  double merge_ready_persistence_{0.3};
  bool weak_reconnect_enabled_{true};
  double weak_reconnect_ready_persistence_{0.3};
  double weak_reconnect_visibility_threshold_{1.0};
  double weak_reconnect_max_edge_distance_{3.00};
  double weak_reconnect_formation_weight_scale_{0.22};
  double weak_reconnect_mid_formation_weight_scale_{0.25};
  double weak_reconnect_final_formation_weight_scale_{0.30};
  double weak_reconnect_ramp_step_duration_{0.5};
  double split_formation_weight_scale_{0.15};
  double split_progress_sync_weight_scale_{0.0};
  double secondary_hard_split_threshold_{0.20};
  double secondary_distance_split_threshold_{1.80};
  double ack_timeout_{2.0};
  uint64_t next_epoch_{0};
  ego_planner::FormationTopologyCommand active_command_;
  std::array<bool, kUavCount> acknowledgements_{{false, false, false,
                                                 false, false, false}};
  ros::Time criterion_stamp_;
  ros::Time candidate_since_;
  ros::Time merge_ready_since_;
  ros::Time weak_reconnect_ready_since_;
  ros::Time weak_recovery_ramp_since_;
  ros::Time command_stamp_;
  ros::Time last_command_publish_;
  std::array<double, kEdgeCount> edge_scores_{};
  std::array<double, kEdgeCount> edge_clearances_{};
  std::array<double, kEdgeCount> edge_visibility_{};
  std::array<double, kEdgeCount> edge_distances_{};
  bool criterion_edge_metrics_valid_{false};
  uint32_t criterion_assignment_generation_{0};
  uint32_t assignment_generation_{0};
  uint32_t minimum_merge_assignment_generation_{0};
  std::array<bool, kUavCount> assignment_acknowledgements_{{
      false, false, false, false, false, false}};
  bool assignment_received_{false};
  bool awaiting_merge_assignment_{false};
  bool reassignment_completed_for_split_cycle_{false};
  bool weak_recovery_active_{false};
  std::string weak_recovery_source_components_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_auto_split_bridge_6uav");
  try {
    SixUavAutoSplitBridge bridge;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start six-UAV auto-split bridge: "
                     << error.what());
    return 1;
  }
  return 0;
}

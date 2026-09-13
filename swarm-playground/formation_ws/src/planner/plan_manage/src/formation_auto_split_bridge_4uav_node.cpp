#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ego_planner/FormationTopologyAck.h>
#include <ego_planner/FormationTopologyCommand.h>
#include <std_msgs/UInt32MultiArray.h>

namespace {

constexpr std::size_t kUavCount = 4;
constexpr std::size_t kEdgeCount = 4;
constexpr std::array<std::array<int, 2>, kEdgeCount> kSlotEdges{{
    {{0, 1}},
    {{1, 3}},
    {{3, 2}},
    {{2, 0}},
}};
std::array<std::array<int, 2>, kEdgeCount> gEdges = kSlotEdges;

class AutoSplitBridge {
 public:
  AutoSplitBridge() : private_nh_("~") {
    private_nh_.param<std::string>("criterion_state_topic", criterion_state_topic_,
                                   "/starconvex_shadow_4uav/state");
    private_nh_.param<std::string>("criterion_summary_topic",
                                   criterion_summary_topic_,
                                   "/starconvex_shadow_4uav/summary");
    private_nh_.param("ack_timeout", ack_timeout_, 2.0);
    private_nh_.param("auto_merge_enabled", auto_merge_enabled_, true);
    private_nh_.param("merge_wall_x", merge_wall_x_, 0.0);
    private_nh_.param("merge_wall_half_x", merge_wall_half_x_, 0.25);
    private_nh_.param("merge_after_wall_margin", merge_after_wall_margin_, 1.5);
    private_nh_.param("merge_ready_persistence", merge_ready_persistence_, 1.0);
    private_nh_.param("merge_candidate_min_score",
                      merge_candidate_min_score_, 0.35);
    private_nh_.param("post_merge_intact_persistence",
                      post_merge_intact_persistence_, 1.0);
    private_nh_.param("staged_merge_enabled", staged_merge_enabled_, true);
    private_nh_.param("row_merge_after_wall_margin",
                      row_merge_after_wall_margin_, 0.5);
    private_nh_.param("row_merge_ready_persistence",
                      row_merge_ready_persistence_, 0.5);
    private_nh_.param("row_merge_score_threshold",
                      row_merge_score_threshold_, 0.70);
    private_nh_.param("split_formation_weight_scale",
                      split_formation_weight_scale_, 0.0);
    private_nh_.param("split_progress_sync_weight_scale",
                      split_progress_sync_weight_scale_, 0.0);
    private_nh_.param("row_merge_formation_weight_scale",
                      row_merge_formation_weight_scale_, 0.25);
    private_nh_.param("autonomous_edge_merge_enabled",
                      autonomous_edge_merge_enabled_, true);
    private_nh_.param("edge_soft_reconnect_score_threshold",
                      edge_soft_reconnect_score_threshold_, 0.50);
    private_nh_.param("edge_merge_score_threshold",
                      edge_merge_score_threshold_, 0.70);
    private_nh_.param("edge_merge_ready_persistence",
                      edge_merge_ready_persistence_, 0.5);
    private_nh_.param("edge_full_merge_ready_persistence",
                      edge_full_merge_ready_persistence_, 0.5);
    private_nh_.param("edge_merge_formation_weight_scale",
                      edge_merge_formation_weight_scale_, 0.15);
    private_nh_.param("edge_merge_max_distance",
                      edge_merge_max_distance_, 4.0);
    private_nh_.param("edge_full_merge_max_distance",
                      edge_full_merge_max_distance_, 2.5);
    private_nh_.param("edge_merge_use_wall_gate",
                      edge_merge_use_wall_gate_, false);
    private_nh_.param("global_intact_soft_reconnect_enabled",
                      global_intact_soft_reconnect_enabled_, false);
    private_nh_.param("global_intact_soft_reconnect_persistence",
                      global_intact_soft_reconnect_persistence_, 0.4);
    private_nh_.param("edge_cut_score_threshold",
                      edge_cut_score_threshold_, 0.34);
    private_nh_.param("edge_hard_cut_score_threshold",
                      edge_hard_cut_score_threshold_, 0.20);
    private_nh_.param("edge_cut_ready_persistence",
                      edge_cut_ready_persistence_, 1.2);
    private_nh_.param("edge_cut_stall_progress",
                      edge_cut_stall_progress_, 0.10);
    private_nh_.param("edge_cut_fallback_persistence",
                      edge_cut_fallback_persistence_, 3.0);
    private_nh_.param("split_min_merge_delay",
                      split_min_merge_delay_, 0.8);
    if (ack_timeout_ <= 0.0 || merge_wall_half_x_ < 0.0 ||
        merge_after_wall_margin_ < 0.0 || merge_ready_persistence_ < 0.0 ||
        merge_candidate_min_score_ < 0.0 || merge_candidate_min_score_ > 1.0 ||
        post_merge_intact_persistence_ < 0.0 ||
        row_merge_after_wall_margin_ < 0.0 ||
        row_merge_ready_persistence_ < 0.0 ||
        row_merge_score_threshold_ < 0.0 || row_merge_score_threshold_ > 1.0 ||
        split_formation_weight_scale_ < 0.0 ||
        split_progress_sync_weight_scale_ < 0.0 ||
        row_merge_formation_weight_scale_ < 0.0 ||
        edge_soft_reconnect_score_threshold_ < 0.0 ||
        edge_soft_reconnect_score_threshold_ > 1.0 ||
        edge_merge_score_threshold_ < edge_soft_reconnect_score_threshold_ ||
        edge_merge_score_threshold_ > 1.0 ||
        edge_merge_ready_persistence_ < 0.0 ||
        edge_full_merge_ready_persistence_ < 0.0 ||
        edge_merge_formation_weight_scale_ < 0.0 ||
        edge_merge_max_distance_ <= 0.0 ||
        edge_full_merge_max_distance_ <= 0.0 ||
        global_intact_soft_reconnect_persistence_ < 0.0 ||
        edge_cut_score_threshold_ < 0.0 ||
        edge_cut_score_threshold_ > 1.0 ||
        edge_hard_cut_score_threshold_ < 0.0 ||
        edge_hard_cut_score_threshold_ > edge_cut_score_threshold_ ||
        edge_cut_ready_persistence_ < 0.0 ||
        edge_cut_stall_progress_ < 0.0 ||
        edge_cut_fallback_persistence_ < edge_cut_ready_persistence_ ||
        split_min_merge_delay_ < 0.0) {
      throw std::runtime_error("invalid formation auto-split bridge parameters");
    }

    criterion_subscriber_ = nh_.subscribe<std_msgs::String>(
        criterion_state_topic_, 1, &AutoSplitBridge::criterionCallback, this);
    criterion_summary_subscriber_ = nh_.subscribe<std_msgs::String>(
        criterion_summary_topic_, 1, &AutoSplitBridge::criterionSummaryCallback,
        this);
    topology_ack_subscriber_ = nh_.subscribe<ego_planner::FormationTopologyAck>(
        "/formation_backend/topology_ack", 20,
        &AutoSplitBridge::topologyAckCallback, this);
    assignment_subscriber_ = nh_.subscribe<std_msgs::UInt32MultiArray>(
        "/formation_backend/initial_assignment", 1,
        &AutoSplitBridge::assignmentCallback, this);

    // One latched message carries a complete topology snapshot.  Re-publishing
    // while waiting for ACKs keeps the same epoch and is idempotent downstream.
    topology_command_publisher_ =
        nh_.advertise<ego_planner::FormationTopologyCommand>(
            "/formation_backend/topology_command", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/formation_backend/auto_split_status", 1, true);
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &AutoSplitBridge::timerCallback, this);
    next_epoch_ = ros::Time::now().toNSec();
    issueTopologyCommand(ego_planner::FormationTopologyCommand::INTACT,
                         "INTACT", 0.0, 0.0, "BRIDGE_STARTUP");
    publishStatus("AUTO_SPLIT ARMED");
    ROS_INFO_STREAM("[AUTO_SPLIT] Armed. Split is criterion-driven; merge is "
                    << (auto_merge_enabled_ ? "enabled." : "disabled.")
                    << " staged_merge=" << (staged_merge_enabled_ ? "true" : "false")
                    << " autonomous_edge_merge="
                    << (autonomous_edge_merge_enabled_ ? "true" : "false")
                    << " global_intact_soft_reconnect="
                    << (global_intact_soft_reconnect_enabled_ ? "true" : "false"));
  }

 private:
  std::string extractJsonString(const std::string& text,
                                const std::string& key) const {
    const std::string needle = "\"" + key + "\":\"";
    const std::size_t begin = text.find(needle);
    if (begin == std::string::npos) return "";
    const std::size_t value_begin = begin + needle.size();
    const std::size_t value_end = text.find('"', value_begin);
    if (value_end == std::string::npos) return "";
    return text.substr(value_begin, value_end - value_begin);
  }

  bool extractJsonNumber(const std::string& text,
                         const std::string& key,
                         double* value) const {
    const std::string needle = "\"" + key + "\":";
    const std::size_t begin = text.find(needle);
    if (begin == std::string::npos) return false;
    const std::size_t value_begin = begin + needle.size();
    std::size_t value_end = text.find(',', value_begin);
    if (value_end == std::string::npos)
      value_end = text.find('}', value_begin);
    if (value_end == std::string::npos || value_end <= value_begin)
      return false;
    try {
      *value = std::stod(text.substr(value_begin, value_end - value_begin));
    } catch (const std::exception&) {
      return false;
    }
    return std::isfinite(*value);
  }

  bool extractJsonNumberWithLegacyAlias(const std::string& text,
                                        const std::string& key,
                                        const std::string& legacy_key,
                                        double* value) const {
    return extractJsonNumber(text, key, value) ||
           extractJsonNumber(text, legacy_key, value);
  }

  bool extractJsonBool(const std::string& text,
                       const std::string& key,
                       bool* value) const {
    const std::string needle = "\"" + key + "\":";
    const std::size_t begin = text.find(needle);
    if (begin == std::string::npos) return false;
    const std::size_t value_begin = begin + needle.size();
    if (text.compare(value_begin, 4, "true") == 0) {
      *value = true;
      return true;
    }
    if (text.compare(value_begin, 5, "false") == 0) {
      *value = false;
      return true;
    }
    return false;
  }

  void assignmentCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
    if (assignment_locked_ || message->data.size() != kUavCount) return;
    std::array<int, kUavCount> drone_for_slot{{-1, -1, -1, -1}};
    for (std::size_t drone = 0; drone < kUavCount; ++drone) {
      const std::size_t slot = message->data[drone];
      if (slot >= kUavCount || drone_for_slot[slot] >= 0) {
        ROS_ERROR("[INITIAL_ASSIGNMENT] split bridge rejected invalid permutation");
        return;
      }
      drone_for_slot[slot] = static_cast<int>(drone);
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      gEdges[edge] = {{drone_for_slot[kSlotEdges[edge][0]],
                       drone_for_slot[kSlotEdges[edge][1]]}};
    }
    assignment_locked_ = true;
    active_edges_.fill(true);
    active_edges_valid_ = true;
    ROS_WARN("[INITIAL_ASSIGNMENT] split bridge slot topology mapped to physical UAV IDs");
  }

  void criterionCallback(const std_msgs::String::ConstPtr& message) {
    if (waiting_criterion_recovery_ || split_requested_ ||
        message->data != "SPLIT")
      return;

    // The state topic does not contain the component snapshot.  Wait for the
    // summary callback so the first split cannot accidentally use an old or
    // empty topology.
    publishStatus("AUTO_SPLIT WAIT_COMPONENTS");
  }

  void topologyAckCallback(
      const ego_planner::FormationTopologyAck::ConstPtr& message) {
    if (message->drone_id >= kUavCount) {
      ROS_WARN_THROTTLE(1.0, "[AUTO_SPLIT] Ignore ACK from invalid drone %u",
                        message->drone_id);
      return;
    }
    const std::size_t id = message->drone_id;
    ack_epochs_[id] = message->epoch;
    ack_modes_[id] = message->mode;
    ack_components_[id] = message->components;
    ack_formation_scales_[id] = message->formation_scale;
    ack_progress_scales_[id] = message->progress_sync_scale;
    ack_seen_[id] = true;
  }

  void criterionSummaryCallback(const std_msgs::String::ConstPtr& message) {
    const std::string state = extractJsonString(message->data, "state");
    const std::string components = extractJsonString(message->data, "components");
    double center_x = 0.0;
    const bool has_center_x = extractJsonNumber(message->data, "center_x", &center_x);
    double center_y = 0.0;
    const bool has_center_y = extractJsonNumber(message->data, "center_y", &center_y);
    double cut_score = 0.0;
    const bool has_cut_score =
        extractJsonNumber(message->data, "cut_score", &cut_score);
    double score_01 = 0.0;
    const bool has_score_01 =
        extractJsonNumber(message->data, "score_01", &score_01);
    double score_32 = 0.0;
    const bool has_score_32 = extractJsonNumberWithLegacyAlias(
        message->data, "score_32", "score_23", &score_32);
    std::array<double, kEdgeCount> scores{{0.0, 0.0, 0.0, 0.0}};
    const bool has_all_scores =
        extractJsonNumber(message->data, "score_01", &scores[0]) &&
        extractJsonNumberWithLegacyAlias(message->data, "score_13", "score_12",
                                         &scores[1]) &&
        extractJsonNumberWithLegacyAlias(message->data, "score_32", "score_23",
                                         &scores[2]) &&
        extractJsonNumberWithLegacyAlias(message->data, "score_20", "score_30",
                                         &scores[3]);
    std::array<double, kEdgeCount> merge_scores{{0.0, 0.0, 0.0, 0.0}};
    const bool has_all_merge_scores =
        extractJsonNumber(message->data, "merge_score_01", &merge_scores[0]) &&
        extractJsonNumber(message->data, "merge_score_13", &merge_scores[1]) &&
        extractJsonNumber(message->data, "merge_score_32", &merge_scores[2]) &&
        extractJsonNumber(message->data, "merge_score_20", &merge_scores[3]);
    std::array<double, kEdgeCount> edge_distances{{0.0, 0.0, 0.0, 0.0}};
    const bool has_all_edge_distances =
        extractJsonNumber(message->data, "distance_01", &edge_distances[0]) &&
        extractJsonNumber(message->data, "distance_13", &edge_distances[1]) &&
        extractJsonNumber(message->data, "distance_32", &edge_distances[2]) &&
        extractJsonNumber(message->data, "distance_20", &edge_distances[3]);
    std::array<double, kEdgeCount> clearances{{0.0, 0.0, 0.0, 0.0}};
    const bool has_all_clearances =
        extractJsonNumber(message->data, "clearance_01", &clearances[0]) &&
        extractJsonNumberWithLegacyAlias(message->data, "clearance_13",
                                         "clearance_12", &clearances[1]) &&
        extractJsonNumber(message->data, "clearance_32", &clearances[2]) &&
        extractJsonNumberWithLegacyAlias(message->data, "clearance_20",
                                         "clearance_30", &clearances[3]);
    std::array<double, kEdgeCount> visibility_weights{{0.0, 0.0, 0.0, 0.0}};
    const bool has_all_visibility_weights =
        extractJsonNumber(message->data, "visibility_weight_01",
                          &visibility_weights[0]) &&
        extractJsonNumber(message->data, "visibility_weight_13",
                          &visibility_weights[1]) &&
        extractJsonNumber(message->data, "visibility_weight_32",
                          &visibility_weights[2]) &&
        extractJsonNumber(message->data, "visibility_weight_20",
                          &visibility_weights[3]);
    std::array<bool, kEdgeCount> merge_direction_valid{{false, false, false, false}};
    const bool has_all_merge_direction_valid =
        extractJsonBool(message->data, "merge_direction_valid_01",
                        &merge_direction_valid[0]) &&
        extractJsonBool(message->data, "merge_direction_valid_13",
                        &merge_direction_valid[1]) &&
        extractJsonBool(message->data, "merge_direction_valid_32",
                        &merge_direction_valid[2]) &&
        extractJsonBool(message->data, "merge_direction_valid_20",
                        &merge_direction_valid[3]);
    const std::string cut_edges = extractJsonString(message->data, "cut_edges");

    if (state == "SPLIT" && components.empty()) {
      publishStatus("AUTO_SPLIT WAIT_COMPONENTS");
      return;
    }

    if (waiting_criterion_recovery_) {
      updatePostMergeRecovery(state);
      return;
    }

    if (autonomous_edge_merge_enabled_ && has_all_scores &&
        has_all_merge_scores && has_all_edge_distances &&
        has_all_clearances && has_all_visibility_weights &&
        has_all_merge_direction_valid &&
        maybePublishAutonomousEdgeMerge(
            state, components, cut_edges, scores, merge_scores,
            edge_distances, clearances, visibility_weights,
            merge_direction_valid, has_center_x && has_center_y,
            center_x, center_y)) {
      return;
    }

    if (state != "SPLIT") {
      maybeRequestMerge(state, has_center_x, center_x, has_cut_score, cut_score);
      return;
    }

    if (maybePublishStagedMerge(has_center_x, center_x,
                                has_score_01, score_01,
                                has_score_32, score_32)) {
      return;
    }

    if (!staged_merge_enabled_ &&
        maybeRequestMerge(state, has_center_x, center_x, has_cut_score,
                          cut_score)) {
      return;
    }

    staged_merge_candidate_since_ = ros::Time();
    staged_merge_candidate_components_.clear();
    requestSplitTopology(components,
                         activeSplitFormationScale(has_center_x, center_x),
                         activeSplitProgressSyncScale(has_center_x, center_x),
                         "AUTO_SPLIT COMMAND_SENT");
    merge_candidate_since_ = ros::Time();
    if (split_confirmed_ && !merge_requested_) {
      publishStatus("AUTO_SPLIT CONFIRMED");
    }
  }

  bool splitConfirmed() const {
    return commandConfirmed(ego_planner::FormationTopologyCommand::SPLIT);
  }

  bool acknowledgementInconsistent() const {
    for (std::size_t id = 0; id < kUavCount; ++id) {
      if (!ack_seen_[id] || ack_epochs_[id] != active_command_.epoch)
        continue;
      if (!ackMatchesActiveCommand(id)) return true;
    }
    return false;
  }

  bool mergeConfirmed() const {
    return commandConfirmed(ego_planner::FormationTopologyCommand::INTACT);
  }

  bool ackMatchesActiveCommand(const std::size_t id) const {
    return ack_seen_[id] && ack_epochs_[id] == active_command_.epoch &&
           ack_modes_[id] == active_command_.mode &&
           ack_components_[id] == active_command_.components &&
           std::fabs(ack_formation_scales_[id] -
                     active_command_.formation_scale) < 1e-6 &&
           std::fabs(ack_progress_scales_[id] -
                     active_command_.progress_sync_scale) < 1e-6;
  }

  bool commandConfirmed(const uint8_t mode) const {
    if (active_command_.mode != mode) return false;
    for (std::size_t id = 0; id < kUavCount; ++id) {
      if (!ackMatchesActiveCommand(id)) return false;
    }
    return true;
  }

  void updatePostMergeRecovery(const std::string& criterion_state) {
    if (criterion_state != "INTACT") {
      post_merge_intact_since_ = ros::Time();
      publishStatus("AUTO_MERGE WAIT_CRITERION_INTACT");
      return;
    }

    const ros::Time now = ros::Time::now();
    if (post_merge_intact_since_.isZero()) {
      post_merge_intact_since_ = now;
      publishStatus("AUTO_MERGE INTACT_HOLD");
      return;
    }
    if ((now - post_merge_intact_since_).toSec() <
        post_merge_intact_persistence_) {
      publishStatus("AUTO_MERGE INTACT_HOLD");
      return;
    }

    waiting_criterion_recovery_ = false;
    post_merge_intact_since_ = ros::Time();
    publishStatus("AUTO_SPLIT ARMED");
    ROS_WARN("[AUTO_MERGE] Criterion recovered to INTACT; auto split is armed again.");
  }

  void publishActiveCommand() {
    active_command_.header.stamp = ros::Time::now();
    topology_command_publisher_.publish(active_command_);
  }

  void issueTopologyCommand(const uint8_t mode,
                            const std::string& components,
                            const double formation_scale,
                            const double progress_sync_scale,
                            const std::string& reason) {
    const bool same_payload =
        active_command_.epoch != 0 && active_command_.mode == mode &&
        active_command_.components == components &&
        std::fabs(active_command_.formation_scale - formation_scale) < 1e-6 &&
        std::fabs(active_command_.progress_sync_scale - progress_sync_scale) < 1e-6;
    if (same_payload) {
      publishActiveCommand();
      return;
    }

    const uint64_t now_epoch = ros::Time::now().toNSec();
    next_epoch_ = std::max(next_epoch_ + 1, now_epoch);
    active_command_.epoch = next_epoch_;
    active_command_.mode = mode;
    active_command_.components = components;
    active_command_.formation_scale = formation_scale;
    active_command_.progress_sync_scale = progress_sync_scale;
    active_command_.reason = reason;
    ack_seen_.fill(false);
    publishActiveCommand();
    ROS_WARN_STREAM("[TOPOLOGY_COMMAND] epoch=" << active_command_.epoch
                    << " mode="
                    << (mode == ego_planner::FormationTopologyCommand::SPLIT
                            ? "SPLIT" : "INTACT")
                    << " components=" << components
                    << " formation_scale=" << formation_scale
                    << " progress_sync_scale=" << progress_sync_scale
                    << " reason=" << reason);
  }

  void publishSplitCommand(const std::string& reason) {
    issueTopologyCommand(ego_planner::FormationTopologyCommand::SPLIT,
                         latest_components_,
                         current_split_formation_weight_scale_,
                         current_split_progress_sync_weight_scale_, reason);
  }

  void publishMergeCommand() {
    issueTopologyCommand(ego_planner::FormationTopologyCommand::INTACT,
                         "INTACT", 0.0, 0.0, "MERGE");
  }

  std::string edgeText(std::size_t edge) const {
    return std::to_string(gEdges[edge][0]) + "-" +
           std::to_string(gEdges[edge][1]);
  }

  bool parseComponentGroups(const std::string& components,
                            std::array<int, kUavCount>* groups) const {
    groups->fill(-1);
    std::array<bool, kUavCount> seen{{false, false, false, false}};
    int component = 0;
    bool in_component = false;
    bool reading_number = false;
    int value = -1;
    for (const char ch : components) {
      if (ch == '{') {
        in_component = true;
        reading_number = false;
        value = -1;
      } else if (ch >= '0' && ch <= '9' && in_component) {
        if (!reading_number) {
          reading_number = true;
          value = 0;
        }
        value = value * 10 + (ch - '0');
      } else {
        if (reading_number) {
          if (value < 0 || value >= static_cast<int>(kUavCount) ||
              seen[value]) {
            return false;
          }
          (*groups)[value] = component;
          seen[value] = true;
        }
        reading_number = false;
        value = -1;
        if (ch == '}' && in_component) {
          in_component = false;
          ++component;
        }
      }
    }
    if (reading_number) {
      if (value < 0 || value >= static_cast<int>(kUavCount) || seen[value])
        return false;
      (*groups)[value] = component;
      seen[value] = true;
    }
    if (component <= 0 ||
        !std::all_of(groups->begin(), groups->end(),
                     [](int group) { return group >= 0; })) {
      return false;
    }

    std::vector<int> old_labels;
    for (int& group : *groups) {
      const auto found = std::find(old_labels.begin(), old_labels.end(), group);
      if (found == old_labels.end()) {
        old_labels.push_back(group);
        group = static_cast<int>(old_labels.size()) - 1;
      } else {
        group = static_cast<int>(std::distance(old_labels.begin(), found));
      }
    }
    return true;
  }

  std::string componentsFromGroups(
      const std::array<int, kUavCount>& groups) const {
    const int component_count =
        *std::max_element(groups.begin(), groups.end()) + 1;
    std::ostringstream text;
    for (int component = 0; component < component_count; ++component) {
      if (component > 0) text << "|";
      text << "{";
      bool first = true;
      for (std::size_t id = 0; id < kUavCount; ++id) {
        if (groups[id] != component) continue;
        if (!first) text << ",";
        text << id;
        first = false;
      }
      text << "}";
    }
    return text.str();
  }

  bool canonicalizeComponents(const std::string& input,
                              std::string* canonical,
                              std::array<int, kUavCount>* groups) const {
    if (!parseComponentGroups(input, groups)) return false;
    *canonical = componentsFromGroups(*groups);
    return true;
  }

  void updateActiveEdgesFromComponents(const std::string& components) {
    std::array<int, kUavCount> groups;
    if (!parseComponentGroups(components, &groups)) {
      active_edges_valid_ = false;
      return;
    }
    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      active_edges_[edge] =
          groups[gEdges[edge][0]] == groups[gEdges[edge][1]];
    }
    active_edges_valid_ = true;
  }

  std::string componentsFromActiveEdges() const {
    std::array<int, kUavCount> parent{{0, 1, 2, 3}};
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
    const auto unite = [&parent, &find_root](int first, int second) {
      const int first_root = find_root(first);
      const int second_root = find_root(second);
      if (first_root != second_root) parent[second_root] = first_root;
    };

    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      if (active_edges_[edge]) unite(gEdges[edge][0], gEdges[edge][1]);
    }

    std::vector<std::vector<int>> components;
    std::array<int, kUavCount> component_index{{-1, -1, -1, -1}};
    for (int id = 0; id < static_cast<int>(kUavCount); ++id) {
      const int root = find_root(id);
      if (component_index[root] < 0) {
        component_index[root] = static_cast<int>(components.size());
        components.push_back({});
      }
      components[component_index[root]].push_back(id);
    }

    std::ostringstream text;
    for (std::size_t component = 0; component < components.size(); ++component) {
      if (component > 0) text << "|";
      text << "{";
      for (std::size_t idx = 0; idx < components[component].size(); ++idx) {
        if (idx > 0) text << ",";
        text << components[component][idx];
      }
      text << "}";
    }
    return text.str();
  }

  bool allActiveEdgesConnected() const {
    return std::all_of(active_edges_.begin(), active_edges_.end(),
                       [](bool active) { return active; });
  }

  bool edgeCutByCriterion(std::size_t edge,
                          const std::string& criterion_components,
                          const std::string& cut_edges) const {
    if (cut_edges.find(edgeText(edge)) != std::string::npos) return true;

    std::array<int, kUavCount> groups;
    if (!parseComponentGroups(criterion_components, &groups)) return false;
    return groups[gEdges[edge][0]] != groups[gEdges[edge][1]];
  }

  void resetEdgeMergeHold(std::size_t edge) {
    edge_merge_candidate_since_[edge] = ros::Time();
  }

  void resetEdgeCutHold(std::size_t edge) {
    edge_cut_candidate_since_[edge] = ros::Time();
    edge_cut_candidate_progress_[edge] = 0.0;
    edge_cut_candidate_has_previous_center_[edge] = false;
  }

  void accumulateEdgeCutProgress(std::size_t edge,
                                 bool has_center,
                                 double center_x,
                                 double center_y) {
    if (!has_center) return;
    if (edge_cut_candidate_has_previous_center_[edge]) {
      edge_cut_candidate_progress_[edge] += std::hypot(
          center_x - edge_cut_candidate_previous_x_[edge],
          center_y - edge_cut_candidate_previous_y_[edge]);
    }
    edge_cut_candidate_previous_x_[edge] = center_x;
    edge_cut_candidate_previous_y_[edge] = center_y;
    edge_cut_candidate_has_previous_center_[edge] = true;
  }

  bool initialMergeDelayElapsed(const ros::Time& now) const {
    if (split_confirmed_time_.isZero()) return false;
    return (now - split_confirmed_time_).toSec() >= split_min_merge_delay_;
  }

  bool mergeWallGateReady(bool has_center_x, double center_x) const {
    if (!edge_merge_use_wall_gate_) return true;
    const double merge_x =
        merge_wall_x_ + merge_wall_half_x_ + merge_after_wall_margin_;
    return has_center_x && center_x >= merge_x;
  }

  bool maybePublishAutonomousEdgeMerge(
      const std::string& criterion_state,
      const std::string& criterion_components,
      const std::string& cut_edges,
      const std::array<double, kEdgeCount>& scores,
      const std::array<double, kEdgeCount>& merge_scores,
      const std::array<double, kEdgeCount>& edge_distances,
      const std::array<double, kEdgeCount>& clearances,
      const std::array<double, kEdgeCount>& visibility_weights,
      const std::array<bool, kEdgeCount>& merge_direction_valid,
      bool has_center,
      double center_x,
      double center_y) {
    if (!auto_merge_enabled_ || !split_requested_ || !split_confirmed_ ||
        merge_requested_) {
      return false;
    }
    if (!active_edges_valid_) updateActiveEdgesFromComponents(latest_components_);
    if (!active_edges_valid_) return false;

    bool topology_changed = false;
    bool topology_cut = false;
    bool topology_reconnected = false;
    bool holding_edge = false;
    bool holding_cut = false;
    bool protecting_topology = false;
    const ros::Time now = ros::Time::now();
    const bool gate_ready = mergeWallGateReady(has_center, center_x);
    const bool merge_delay_elapsed = initialMergeDelayElapsed(now);

    // A globally INTACT criterion means all four visibility/overlap edges are
    // simultaneously recoverable.  Waiting for four independent per-edge
    // persistence windows makes the formation start closing unnecessarily
    // late.  Restore the complete topology once, but only with the weak split
    // weight.  The strong INTACT backend is still gated below by every edge's
    // score and merge direction.
    if (global_intact_soft_reconnect_enabled_ &&
        criterion_state == "INTACT" && merge_delay_elapsed && gate_ready &&
        !allActiveEdgesConnected()) {
      if (global_intact_soft_reconnect_since_.isZero()) {
        global_intact_soft_reconnect_since_ = now;
        publishStatus("AUTO_MERGE GLOBAL_INTACT_HOLD");
        return true;
      }
      if ((now - global_intact_soft_reconnect_since_).toSec() <
          global_intact_soft_reconnect_persistence_) {
        publishStatus("AUTO_MERGE GLOBAL_INTACT_HOLD");
        return true;
      }

      active_edges_.fill(true);
      edge_recovery_phase_ = true;
      edge_merge_candidate_since_.fill(ros::Time());
      edge_cut_candidate_since_.fill(ros::Time());
      edge_cut_candidate_progress_.fill(0.0);
      edge_cut_candidate_has_previous_center_.fill(false);
      full_merge_candidate_since_ = ros::Time();
      requestSplitTopology("{0,1,2,3}",
                           edge_merge_formation_weight_scale_,
                           split_progress_sync_weight_scale_,
                           "AUTO_MERGE GLOBAL_INTACT_SOFT_RECONNECT");
      ROS_WARN_STREAM("[AUTO_MERGE] Global criterion remained INTACT for "
                      << global_intact_soft_reconnect_persistence_
                      << " s. Restoring all edges with weak formation scale "
                      << edge_merge_formation_weight_scale_ << ".");
      return true;
    }
    if (criterion_state != "INTACT" || !merge_delay_elapsed || !gate_ready) {
      global_intact_soft_reconnect_since_ = ros::Time();
    }

    for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
      const bool criterion_cuts =
          criterion_state == "SPLIT" &&
          edgeCutByCriterion(edge, criterion_components, cut_edges);
      if (!edge_recovery_phase_ && active_edges_[edge] && criterion_cuts &&
          scores[edge] < edge_cut_score_threshold_) {
        resetEdgeMergeHold(edge);
        holding_cut = true;
        const bool hard_safety_cut =
            scores[edge] <= edge_hard_cut_score_threshold_ ||
            visibility_weights[edge] <= 0.0 || clearances[edge] <= 0.0;
        if (hard_safety_cut) {
          active_edges_[edge] = false;
          resetEdgeCutHold(edge);
          full_merge_candidate_since_ = ros::Time();
          topology_changed = true;
          topology_cut = true;
          ROS_WARN_STREAM("[AUTO_EDGE] Immediate hard cut " << edgeText(edge)
                          << " S=" << scores[edge]
                          << " clearance=" << clearances[edge]
                          << " visibility_weight=" << visibility_weights[edge]);
          continue;
        }
        if (edge_cut_candidate_since_[edge].isZero()) {
          edge_cut_candidate_since_[edge] = now;
          accumulateEdgeCutProgress(edge, has_center, center_x, center_y);
          continue;
        }
        accumulateEdgeCutProgress(edge, has_center, center_x, center_y);
        const double cut_elapsed =
            (now - edge_cut_candidate_since_[edge]).toSec();
        if (cut_elapsed < edge_cut_ready_persistence_) {
          continue;
        }
        const bool stalled =
            has_center &&
            edge_cut_candidate_progress_[edge] <= edge_cut_stall_progress_;
        const bool fallback_elapsed =
            cut_elapsed >= edge_cut_fallback_persistence_;
        if (!stalled && !fallback_elapsed) {
          protecting_topology = true;
          continue;
        }
        active_edges_[edge] = false;
        const double observed_progress = edge_cut_candidate_progress_[edge];
        resetEdgeCutHold(edge);
        full_merge_candidate_since_ = ros::Time();
        topology_changed = true;
        topology_cut = true;
        ROS_WARN_STREAM("[AUTO_EDGE] Delayed cut " << edgeText(edge)
                        << " after " << cut_elapsed << " s, progress="
                        << observed_progress << " m, cause="
                        << (stalled ? "STALL" : "PERSISTENT_LOW_SCORE"));
        continue;
      }
      resetEdgeCutHold(edge);

      if (!active_edges_[edge]) {
        const bool direction_ready =
            merge_direction_valid[edge] || criterion_state == "INTACT";
        // At very low speed the forward half-space is undefined and the
        // shadow node intentionally publishes M=0/NA.  Once the global
        // criterion is INTACT, use the direction-independent overlap score S
        // instead; otherwise a vehicle that has nearly stopped at the goal can
        // make this edge impossible to recover forever.
        const double effective_merge_score =
            merge_direction_valid[edge] ? merge_scores[edge] : scores[edge];
        // A score above the 0.45 narrow-clearance ceiling cannot come from that
        // visibility branch, so it is safe to establish a weak, still-SPLIT
        // connection before the criterion edge itself reaches KEEP at 0.70.
        // Requiring both S and M prevents a favorable forward half-space from
        // hiding poor direction-independent overlap.
        const bool locally_recoverable =
            direction_ready &&
            scores[edge] >= edge_soft_reconnect_score_threshold_ &&
            effective_merge_score >= edge_soft_reconnect_score_threshold_ &&
            edge_distances[edge] <= edge_merge_max_distance_;
        if (!merge_delay_elapsed || !gate_ready ||
            !locally_recoverable) {
          resetEdgeMergeHold(edge);
          continue;
        }
        holding_edge = true;
        if (edge_merge_candidate_since_[edge].isZero()) {
          edge_merge_candidate_since_[edge] = now;
          continue;
        }
        if ((now - edge_merge_candidate_since_[edge]).toSec() <
            edge_merge_ready_persistence_) {
          continue;
        }
        active_edges_[edge] = true;
        edge_recovery_phase_ = true;
        resetEdgeMergeHold(edge);
        resetEdgeCutHold(edge);
        topology_changed = true;
        topology_reconnected = true;
      }
    }

    const std::string next_components = componentsFromActiveEdges();
    // Always commit a weak topology change before considering the strong
    // INTACT command.  Previously, if the final two cut edges recovered in the
    // same callback, allActiveEdgesConnected() returned early and skipped this
    // update, leaving the two subgroups without any attraction while waiting
    // for the final 0.70 threshold.
    if (topology_changed) {
      const double formation_scale =
          topology_reconnected ? edge_merge_formation_weight_scale_
                               : split_formation_weight_scale_;
      requestSplitTopology(next_components,
                           formation_scale,
                           split_progress_sync_weight_scale_,
                           topology_reconnected
                               ? "AUTO_MERGE EDGE_RECOVERY"
                               : "AUTO_SPLIT EDGE_CUT");
      ROS_WARN_STREAM("[AUTO_EDGE] phase="
                      << (edge_recovery_phase_ ? "RECOVERING" : "CUTTING")
                      << " transition="
                      << (topology_reconnected
                              ? "RECONNECT"
                              : (topology_cut ? "CUT" : "NONE"))
                      << " topology=" << next_components
                      << " formation_scale=" << formation_scale);
      return true;
    }

    if (allActiveEdgesConnected()) {
      bool all_edges_final_ready = criterion_state == "INTACT";
      for (std::size_t edge = 0; edge < kEdgeCount; ++edge) {
        const bool direction_ready =
            merge_direction_valid[edge] || criterion_state == "INTACT";
        const double effective_merge_score =
            merge_direction_valid[edge] ? merge_scores[edge] : scores[edge];
        all_edges_final_ready =
            all_edges_final_ready && direction_ready &&
            scores[edge] >= edge_merge_score_threshold_ &&
            effective_merge_score >= edge_merge_score_threshold_ &&
            edge_distances[edge] <= edge_full_merge_max_distance_;
      }
      if (!merge_delay_elapsed || !gate_ready || !all_edges_final_ready) {
        full_merge_candidate_since_ = ros::Time();
        publishStatus("AUTO_MERGE EDGE_FULL_WAIT_SAFE");
        return true;
      }
      if (full_merge_candidate_since_.isZero()) {
        full_merge_candidate_since_ = now;
        publishStatus("AUTO_MERGE EDGE_FULL_HOLD");
        return true;
      }
      if ((now - full_merge_candidate_since_).toSec() <
          edge_full_merge_ready_persistence_) {
        publishStatus("AUTO_MERGE EDGE_FULL_HOLD");
        return true;
      }

      merge_requested_ = true;
      merge_request_time_ = now;
      publishMergeCommand();
      publishStatus("AUTO_MERGE COMMAND_SENT");
      ROS_WARN_STREAM("[AUTO_MERGE] All criterion edges recovered above "
                      << edge_merge_score_threshold_
                      << ". Commanding backend merge.");
      return true;
    }

    full_merge_candidate_since_ = ros::Time();
    if (holding_edge) {
      publishStatus("AUTO_MERGE EDGE_READY_HOLD");
    } else if (protecting_topology) {
      publishStatus("AUTO_SPLIT TOPOLOGY_PROTECTED");
    } else if (holding_cut) {
      publishStatus("AUTO_SPLIT EDGE_CUT_HOLD");
    } else {
      publishStatus("AUTO_SPLIT CONFIRMED");
    }
    return true;
  }

  double activeSplitFormationScale(bool has_center_x, double center_x) const {
    (void)has_center_x;
    (void)center_x;
    return split_formation_weight_scale_;
  }

  double activeSplitProgressSyncScale(bool has_center_x,
                                      double center_x) const {
    (void)has_center_x;
    (void)center_x;
    return split_progress_sync_weight_scale_;
  }

  void requestSplitTopology(const std::string& components,
                            const double formation_weight_scale,
                            const double progress_sync_weight_scale,
                            const std::string& status) {
    std::string canonical_components;
    std::array<int, kUavCount> groups;
    if (!canonicalizeComponents(components, &canonical_components, &groups)) {
      ROS_ERROR_STREAM("[AUTO_SPLIT] Refusing invalid components '"
                       << components << "'");
      return;
    }
    const bool topology_changed =
        latest_components_ != canonical_components ||
        std::fabs(current_split_formation_weight_scale_ -
                  formation_weight_scale) > 1e-6 ||
        std::fabs(current_split_progress_sync_weight_scale_ -
                  progress_sync_weight_scale) > 1e-6;
    latest_components_ = canonical_components;
    current_split_formation_weight_scale_ = formation_weight_scale;
    current_split_progress_sync_weight_scale_ = progress_sync_weight_scale;
    updateActiveEdgesFromComponents(latest_components_);
    if (!split_requested_) {
      split_requested_ = true;
      edge_recovery_phase_ = false;
      request_time_ = ros::Time::now();
      publishSplitCommand(status);
      publishStatus(status);
      ROS_WARN_STREAM("[AUTO_SPLIT] Dynamic topology requested: "
                      << latest_components_);
      return;
    }
    if (topology_changed) {
      split_confirmed_ = false;
      request_time_ = ros::Time::now();
      publishSplitCommand(status);
      publishStatus(status);
      const bool recovery_update =
          status.find("AUTO_MERGE") != std::string::npos;
      ROS_WARN_STREAM("[" << (recovery_update ? "AUTO_MERGE" : "AUTO_SPLIT")
                      << "] Staged topology requested: "
                      << latest_components_
                      << " formation_scale="
                      << current_split_formation_weight_scale_
                      << " progress_sync_scale="
                      << current_split_progress_sync_weight_scale_);
      return;
    }
  }

  bool maybeRequestMerge(const std::string& criterion_state,
                         bool has_center_x,
                         double center_x,
                         bool has_cut_score,
                         double cut_score) {
    if (!auto_merge_enabled_ || !split_requested_ || !split_confirmed_ ||
        merge_requested_) {
      return false;
    }
    const double merge_x =
        merge_wall_x_ + merge_wall_half_x_ + merge_after_wall_margin_;
    const bool criterion_intact = criterion_state == "INTACT";
    const bool merge_candidate_score =
        criterion_state == "SPLIT" && has_cut_score &&
        cut_score >= merge_candidate_min_score_;
    const bool merge_candidate =
        criterion_intact || merge_candidate_score;
    const bool passed_wall = has_center_x && center_x >= merge_x;
    if (!merge_candidate || !passed_wall) {
      merge_candidate_since_ = ros::Time();
      if (split_confirmed_ && criterion_state != "SPLIT") {
        publishStatus(merge_candidate ? "AUTO_MERGE WAIT_PASS_WALL"
                                      : "AUTO_MERGE WAIT_SAFE");
      } else if (split_confirmed_ && criterion_state == "SPLIT" &&
                 passed_wall) {
        publishStatus("AUTO_MERGE WAIT_SCORE");
      } else if (split_confirmed_ && criterion_state == "SPLIT" &&
                 merge_candidate_score) {
        publishStatus("AUTO_MERGE WAIT_PASS_WALL");
      }
      return false;
    }

    const ros::Time now = ros::Time::now();
    if (merge_candidate_since_.isZero()) {
      merge_candidate_since_ = now;
      publishStatus("AUTO_MERGE READY_HOLD");
      return true;
    }
    if ((now - merge_candidate_since_).toSec() < merge_ready_persistence_) {
      publishStatus("AUTO_MERGE READY_HOLD");
      return true;
    }

    merge_requested_ = true;
    merge_request_time_ = now;
    publishMergeCommand();
    publishStatus("AUTO_MERGE COMMAND_SENT");
    ROS_WARN_STREAM("[AUTO_MERGE] Criterion is " << criterion_state
                    << ", cut_score=" << cut_score
                    << ", center_x="
                    << center_x << " passed merge_x=" << merge_x
                    << ". Commanding backend merge.");
    return true;
  }

  bool maybePublishStagedMerge(bool has_center_x,
                               double center_x,
                               bool has_score_01,
                               double score_01,
                               bool has_score_32,
                               double score_32) {
    if (!staged_merge_enabled_ || !auto_merge_enabled_ ||
        !split_requested_ || !split_confirmed_ || merge_requested_) {
      return false;
    }

    const double row_merge_x =
        merge_wall_x_ + merge_wall_half_x_ + row_merge_after_wall_margin_;
    if (!has_center_x || center_x < row_merge_x) {
      staged_merge_candidate_since_ = ros::Time();
      staged_merge_candidate_components_.clear();
      return false;
    }

    const bool front_ready =
        has_score_01 && score_01 >= row_merge_score_threshold_;
    const bool rear_ready =
        has_score_32 && score_32 >= row_merge_score_threshold_;
    if (!front_ready && !rear_ready) {
      staged_merge_candidate_since_ = ros::Time();
      staged_merge_candidate_components_.clear();
      return false;
    }

    std::string staged_components;
    if (front_ready && rear_ready) {
      staged_components = "{0,1}|{2,3}";
    } else if (front_ready) {
      staged_components = "{0,1}|{2}|{3}";
    } else {
      staged_components = "{0}|{1}|{2,3}";
    }

    const ros::Time now = ros::Time::now();
    if (staged_components != staged_merge_candidate_components_) {
      staged_merge_candidate_components_ = staged_components;
      staged_merge_candidate_since_ = now;
      publishStatus("AUTO_MERGE ROW_READY_HOLD");
      return true;
    }

    if ((now - staged_merge_candidate_since_).toSec() <
        row_merge_ready_persistence_) {
      publishStatus("AUTO_MERGE ROW_READY_HOLD");
      return true;
    }

    requestSplitTopology(staged_components,
                         row_merge_formation_weight_scale_,
                         split_progress_sync_weight_scale_,
                         "AUTO_MERGE ROW_TOPOLOGY");
    return true;
  }

  void publishStatus(const std::string& status) {
    if (status == last_status_) return;
    last_status_ = status;
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
  }

  bool statusStartsWith(const std::string& prefix) const {
    return last_status_.compare(0, prefix.size(), prefix) == 0;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (merge_requested_) {
      publishMergeCommand();
      if (mergeConfirmed()) {
        split_requested_ = false;
        split_confirmed_ = false;
        merge_requested_ = false;
        waiting_criterion_recovery_ = true;
        latest_components_.clear();
        active_edges_.fill(true);
        active_edges_valid_ = false;
        split_confirmed_time_ = ros::Time();
        merge_candidate_since_ = ros::Time();
        staged_merge_candidate_since_ = ros::Time();
        edge_merge_candidate_since_.fill(ros::Time());
        edge_cut_candidate_since_.fill(ros::Time());
        edge_cut_candidate_progress_.fill(0.0);
        edge_cut_candidate_has_previous_center_.fill(false);
        edge_recovery_phase_ = false;
        full_merge_candidate_since_ = ros::Time();
        global_intact_soft_reconnect_since_ = ros::Time();
        staged_merge_candidate_components_.clear();
        post_merge_intact_since_ = ros::Time();
        publishStatus("AUTO_MERGE WAIT_CRITERION_INTACT");
        ROS_WARN("[AUTO_MERGE] All four planners returned to INTACT backend.");
        return;
      }
      publishStatus("AUTO_MERGE COMMAND_SENT");
      if ((ros::Time::now() - merge_request_time_).toSec() > ack_timeout_) {
        ROS_ERROR_THROTTLE(2.0,
                           "[AUTO_MERGE] Waiting for four INTACT backend acknowledgements.");
      }
      return;
    }

    if (!split_requested_) {
      if (waiting_criterion_recovery_) {
        publishStatus("AUTO_MERGE WAIT_CRITERION_INTACT");
        return;
      }
      publishStatus("AUTO_SPLIT ARMED");
      return;
    }
    if (splitConfirmed()) {
      if (!split_confirmed_) {
        split_confirmed_ = true;
        split_confirmed_time_ = ros::Time::now();
        ROS_WARN("[AUTO_SPLIT] All four planners confirmed the split backend.");
      }
      if (auto_merge_enabled_ && statusStartsWith("AUTO_MERGE ")) {
        return;
      }
      publishStatus("AUTO_SPLIT CONFIRMED");
      return;
    }

    // Retry the same epoch.  Planners ACK it again without re-planning.
    publishSplitCommand("ACK_RETRY");
    if (acknowledgementInconsistent()) {
      publishStatus("AUTO_SPLIT ACK_INCONSISTENT");
    } else {
      publishStatus("AUTO_SPLIT COMMAND_SENT");
    }
    if ((ros::Time::now() - request_time_).toSec() > ack_timeout_) {
      ROS_ERROR_THROTTLE(2.0,
                         "[AUTO_SPLIT] Waiting for four consistent backend acknowledgements.");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber criterion_subscriber_;
  ros::Subscriber criterion_summary_subscriber_;
  ros::Subscriber topology_ack_subscriber_;
  ros::Subscriber assignment_subscriber_;
  ros::Publisher topology_command_publisher_;
  ros::Publisher status_publisher_;
  ros::Timer timer_;
  std::array<uint64_t, kUavCount> ack_epochs_{{0, 0, 0, 0}};
  std::array<uint8_t, kUavCount> ack_modes_{{0, 0, 0, 0}};
  std::array<std::string, kUavCount> ack_components_;
  std::array<double, kUavCount> ack_formation_scales_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, kUavCount> ack_progress_scales_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, kUavCount> ack_seen_{{false, false, false, false}};
  ego_planner::FormationTopologyCommand active_command_;
  uint64_t next_epoch_{0};
  std::string criterion_state_topic_;
  std::string criterion_summary_topic_;
  std::string last_status_;
  std::string latest_components_;
  double ack_timeout_ = 2.0;
  bool auto_merge_enabled_ = true;
  double merge_wall_x_ = 0.0;
  double merge_wall_half_x_ = 0.25;
  double merge_after_wall_margin_ = 1.5;
  double merge_ready_persistence_ = 1.0;
  double merge_candidate_min_score_ = 0.35;
  double post_merge_intact_persistence_ = 1.0;
  bool staged_merge_enabled_ = true;
  bool autonomous_edge_merge_enabled_ = true;
  double row_merge_after_wall_margin_ = 0.5;
  double row_merge_ready_persistence_ = 0.5;
  double row_merge_score_threshold_ = 0.70;
  double split_formation_weight_scale_ = 0.0;
  double split_progress_sync_weight_scale_ = 0.0;
  double row_merge_formation_weight_scale_ = 0.25;
  double edge_soft_reconnect_score_threshold_ = 0.50;
  double edge_merge_score_threshold_ = 0.70;
  double edge_merge_ready_persistence_ = 0.5;
  double edge_full_merge_ready_persistence_ = 0.5;
  double edge_merge_formation_weight_scale_ = 0.15;
  double edge_merge_max_distance_ = 4.0;
  double edge_full_merge_max_distance_ = 2.5;
  bool edge_merge_use_wall_gate_ = false;
  bool global_intact_soft_reconnect_enabled_ = false;
  double global_intact_soft_reconnect_persistence_ = 0.4;
  double edge_cut_score_threshold_ = 0.34;
  double edge_hard_cut_score_threshold_ = 0.20;
  double edge_cut_ready_persistence_ = 1.2;
  double edge_cut_stall_progress_ = 0.10;
  double edge_cut_fallback_persistence_ = 3.0;
  double split_min_merge_delay_ = 0.8;
  double current_split_formation_weight_scale_ = 0.0;
  double current_split_progress_sync_weight_scale_ = 0.0;
  bool split_requested_ = false;
  bool split_confirmed_ = false;
  bool merge_requested_ = false;
  bool waiting_criterion_recovery_ = false;
  ros::Time request_time_;
  ros::Time split_confirmed_time_;
  ros::Time merge_candidate_since_;
  ros::Time staged_merge_candidate_since_;
  std::array<ros::Time, kEdgeCount> edge_merge_candidate_since_;
  std::array<ros::Time, kEdgeCount> edge_cut_candidate_since_;
  std::array<double, kEdgeCount> edge_cut_candidate_progress_{};
  std::array<double, kEdgeCount> edge_cut_candidate_previous_x_{};
  std::array<double, kEdgeCount> edge_cut_candidate_previous_y_{};
  std::array<bool, kEdgeCount> edge_cut_candidate_has_previous_center_{};
  ros::Time full_merge_candidate_since_;
  ros::Time global_intact_soft_reconnect_since_;
  ros::Time merge_request_time_;
  ros::Time post_merge_intact_since_;
  std::string staged_merge_candidate_components_;
  std::array<bool, kEdgeCount> active_edges_{{true, true, true, true}};
  bool active_edges_valid_ = false;
  bool edge_recovery_phase_ = false;
  bool assignment_locked_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_auto_split_bridge_4uav");
  try {
    AutoSplitBridge bridge;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to start four-UAV auto-split bridge: " << error.what());
    return 1;
  }
  return 0;
}

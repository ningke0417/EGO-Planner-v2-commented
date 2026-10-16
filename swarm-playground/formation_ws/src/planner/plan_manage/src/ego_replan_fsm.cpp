
#include <plan_manage/ego_replan_fsm.h>

#include <limits>
#include <plan_manage/distributed_edges.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    collision_recovery_active_ = false;
    mandatory_stop_ = false;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/arrival_position_tolerance", arrival_position_tolerance_, 0.30);
    nh.param("fsm/arrival_speed_tolerance", arrival_speed_tolerance_, 0.15);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("formation/manual_split", manual_formation_split_, false);
    nh.param("formation/general_dynamic_topology_enabled",
             general_dynamic_topology_enabled_, false);
    nh.param("formation/subgroup_route_intent_feedback_enabled",
             subgroup_route_intent_feedback_enabled_, false);
    nh.param<std::string>("formation/subgroup_route_intent_topic",
                          subgroup_route_intent_topic_,
                          "/subgroup_route_intent_4uav/intents");
    nh.param("formation/initial_assignment_enabled",
             initial_assignment_enabled_, false);
    nh.param("formation/subgroup_route_intent_timeout",
             subgroup_route_intent_timeout_, 1.0);
    nh.param("formation/subgroup_route_intent_replan_angle",
             subgroup_route_intent_replan_angle_, 0.261799);
    nh.param("formation/subgroup_route_intent_replan_gain_step",
             subgroup_route_intent_replan_gain_step_, 0.20);
    nh.param("formation/subgroup_route_intent_replan_interval",
             subgroup_route_intent_replan_interval_, 0.5);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("formation/num", form_num_, -1);
    nh.param("formation/distributed_enabled", distributed_enabled_, false);
    nh.param("formation/distributed_startup_timeout",
             distributed_startup_timeout_, 15.0);
    if (distributed_enabled_)
    {
      if (form_num_ != 8)
        throw std::runtime_error("distributed mode currently requires eight fixed slots");
      if (distributed_startup_timeout_ <= 0.0)
        throw std::runtime_error("distributed startup timeout must be positive");
      distributed_startup_begin_ = ros::Time::now();
      initial_assignment_enabled_ = false;
      subgroup_route_intent_feedback_enabled_ = false;
      general_dynamic_topology_enabled_ = true;
      manual_formation_split_ = true;
    }
    if (form_num_ < planner_manager_->pp_.drone_id + 1)
    {
      ROS_ERROR("formation_num is smaller than the drone number, illegal!");
      return;
    }
    if (manual_formation_split_ && form_num_ != 4 &&
        !general_dynamic_topology_enabled_)
    {
      ROS_ERROR("formation/manual_split for non-four-UAV formations requires "
                "formation/general_dynamic_topology_enabled=true; keeping the "
                "formation intact.");
      manual_formation_split_ = false;
    }
    planner_manager_->setManualFormationSplit(manual_formation_split_);
    std::vector<double> pos;
    formation_slots_.resize(form_num_);
    formation_assignment_.resize(form_num_);
    for (int slot = 0; slot < form_num_; ++slot)
    {
      formation_assignment_[slot] = slot;
      if (!nh.getParam("formation/drone" + to_string(slot), pos) || pos.size() != 3)
      {
        ROS_ERROR("Missing or invalid formation/drone%d", slot);
        return;
      }
      formation_slots_[slot] << pos[0], pos[1], pos[2];
    }
    formation_pos_ = formation_slots_[planner_manager_->pp_.drone_id];
    nh.getParam("formation/start", pos);
    formation_start_ << pos[0], pos[1], pos[2];

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);

      // if ( i==0 )
      // {
      //   Eigen::Vector3d dir(waypoints_[0][0]-formation_start_(0), waypoints_[0][1]-formation_start_(1), waypoints_[0][2]-formation_start_(2));
      //   dir.normalize();

      // }
      // else
      // {
      //   Eigen::Vector3d dir(waypoints_[i][0]-waypoints_[i-1][0], waypoints_[i][1]-waypoints_[i-1][1], waypoints_[i][2]-waypoints_[i-1][2]);
      //   dir.normalize();

      // }
    }

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);
    if (distributed_enabled_)
    {
      topology_command_sub_ = nh.subscribe<FormationTopologyCommand>(
          "/drone_" + std::to_string(planner_manager_->pp_.drone_id) +
          "_formation_agent/topology", 1,
          &EGOReplanFSM::distributedTopologyCallback, this);
    }
    else
    {
    manual_split_sub_ = nh.subscribe<std_msgs::Bool>(
        "/formation_backend/manual_split", 1,
        &EGOReplanFSM::manualFormationSplitCallback, this);
    dynamic_topology_sub_ = nh.subscribe<std_msgs::String>(
        "/formation_backend/topology_components", 1,
        &EGOReplanFSM::dynamicTopologyCallback, this);
    topology_command_sub_ = nh.subscribe<FormationTopologyCommand>(
        "/formation_backend/topology_command", 10,
        &EGOReplanFSM::formationTopologyCommandCallback, this);
    split_formation_scale_sub_ = nh.subscribe<std_msgs::Float64>(
        "/formation_backend/split_formation_scale", 1,
        &EGOReplanFSM::splitFormationScaleCallback, this);
    split_progress_sync_scale_sub_ = nh.subscribe<std_msgs::Float64>(
        "/formation_backend/split_progress_sync_scale", 1,
        &EGOReplanFSM::splitProgressSyncScaleCallback, this);
    }
    if (initial_assignment_enabled_)
      initial_assignment_sub_ = nh.subscribe<std_msgs::UInt32MultiArray>(
          "/formation_backend/initial_assignment", 1,
          &EGOReplanFSM::initialAssignmentCallback, this);
    else
      initial_assignment_received_ = true;
    if (subgroup_route_intent_feedback_enabled_)
      subgroup_route_intent_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
          subgroup_route_intent_topic_, 1,
          &EGOReplanFSM::subgroupRouteIntentCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);
    formation_group_pub_ = nh.advertise<std_msgs::String>(
        "/formation_backend/drone_" +
            std::to_string(planner_manager_->pp_.drone_id) + "_group",
        1, true);
    topology_ack_pub_ = nh.advertise<FormationTopologyAck>(
        distributed_enabled_
            ? "/drone_" + std::to_string(planner_manager_->pp_.drone_id) + "/formation_applied"
            : "/formation_backend/topology_ack", 10, false);
    assignment_ack_pub_ = nh.advertise<std_msgs::UInt32MultiArray>(
        "/formation_backend/assignment_ack", 20, false);
    subgroup_route_intent_status_pub_ = nh.advertise<std_msgs::String>(
        "/formation_backend/drone_" +
            std::to_string(planner_manager_->pp_.drone_id) +
            "_route_intent",
        1, true);
    safety_status_pub_ = nh.advertise<std_msgs::String>(
        "/formation_safety/drone_" +
            std::to_string(planner_manager_->pp_.drone_id) + "/status",
        5, false);
    first_wall_violation_pub_ = nh.advertise<std_msgs::String>(
        "/formation_safety/drone_" +
            std::to_string(planner_manager_->pp_.drone_id) +
            "/first_wall_violation",
        1, true);
    planning_metrics_pub_ = nh.advertise<std_msgs::String>(
        "/formation_metrics/drone_" +
            std::to_string(planner_manager_->pp_.drone_id) + "/planning",
        1, true);
    publishFormationGroupState();
    publishSubgroupRouteIntentState(
        subgroup_route_intent_feedback_enabled_ ? "WAITING" : "DISABLED");
    if (distributed_enabled_) expireDistributedTopology();

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for odometry and trigger");

      while (ros::ok() &&
             (!have_odom_ || !have_trigger_ || !initial_assignment_received_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::publishFormationGroupState()
  {
    std_msgs::String message;
    if (!manual_formation_split_)
      message.data = "INTACT";
    else if (dynamic_topology_active_)
      message.data = "C" + std::to_string(dynamicGroupForDrone(planner_manager_->pp_.drone_id));
    else
      message.data = planner_manager_->pp_.drone_id < 2 ? "G0" : "G1";
    formation_group_pub_.publish(message);
  }

  int EGOReplanFSM::dynamicGroupForDrone(const int drone_id) const
  {
    if (dynamic_topology_active_ &&
        drone_id >= 0 &&
        drone_id < static_cast<int>(dynamic_formation_groups_.size()))
      return dynamic_formation_groups_[drone_id];
    return drone_id < form_num_ / 2 ? 0 : 1;
  }

  bool EGOReplanFSM::dynamicTopologyMatchesLegacySplit() const
  {
    if (!dynamic_topology_active_ ||
        static_cast<int>(dynamic_formation_groups_.size()) < form_num_ ||
        form_num_ != 4)
      return true;
    std::vector<int> drone_for_slot(form_num_, -1);
    for (int drone = 0; drone < form_num_; ++drone)
      drone_for_slot[formation_assignment_[drone]] = drone;
    const int front_left = drone_for_slot[0];
    const int front_right = drone_for_slot[1];
    const int rear_left = drone_for_slot[2];
    const int rear_right = drone_for_slot[3];
    return dynamic_formation_groups_[front_left] ==
               dynamic_formation_groups_[front_right] &&
           dynamic_formation_groups_[rear_left] ==
               dynamic_formation_groups_[rear_right] &&
           dynamic_formation_groups_[front_left] !=
               dynamic_formation_groups_[rear_left];
  }

  void EGOReplanFSM::dynamicTopologyCallback(const std_msgs::String::ConstPtr &msg)
  {
    const std::string &text = msg->data;
    if (text == "INTACT" || text.empty())
    {
      if (!dynamic_topology_active_)
      {
        publishFormationGroupState();
        return;
      }
      dynamic_topology_active_ = false;
      dynamic_formation_groups_.clear();
      planner_manager_->setDynamicFormationGroups(dynamic_formation_groups_);
      if (have_odom_)
        planner_manager_->beginMergeSynchronization(odom_pos_);
      publishFormationGroupState();
      if (have_target_ &&
          (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
           exec_state_ == GEN_NEW_TRAJ))
        changeFSMExecState(REPLAN_TRAJ, "DYNAMIC_TOPOLOGY_MERGE");
      return;
    }

    std::vector<int> groups;
    if (!parseDynamicTopology(text, &groups))
    {
      ROS_WARN_THROTTLE(1.0, "[FORMATION_BACKEND] invalid dynamic topology: %s",
                        text.c_str());
      return;
    }

    if (dynamic_topology_active_ && groups == dynamic_formation_groups_)
    {
      publishFormationGroupState();
      return;
    }

    const int previous_component_count =
        dynamic_topology_active_ && !dynamic_formation_groups_.empty()
            ? *std::max_element(dynamic_formation_groups_.begin(),
                                dynamic_formation_groups_.end()) + 1
            : 1;
    const int next_component_count =
        *std::max_element(groups.begin(), groups.end()) + 1;
    const bool components_merged =
        dynamic_topology_active_ &&
        next_component_count < previous_component_count;

    dynamic_formation_groups_ = groups;
    dynamic_topology_active_ = true;
    if (!manual_formation_split_)
    {
      manual_formation_split_ = true;
      planner_manager_->setManualFormationSplit(true);
    }
    planner_manager_->setDynamicFormationGroups(dynamic_formation_groups_);
    if (components_merged && have_odom_)
      planner_manager_->beginMergeSynchronization(odom_pos_);
    publishFormationGroupState();

    ROS_WARN("[FORMATION_BACKEND] drone %d uses legacy dynamic topology %s as C%d",
             planner_manager_->pp_.drone_id, canonicalTopology(groups).c_str(),
             dynamicGroupForDrone(planner_manager_->pp_.drone_id));

    if (have_target_ && (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
                         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "DYNAMIC_TOPOLOGY");
  }

  bool EGOReplanFSM::parseDynamicTopology(const std::string &text,
                                           std::vector<int> *groups) const
  {
    groups->assign(form_num_, -1);
    std::vector<bool> seen(form_num_, false);
    int component = 0;
    bool in_component = false;
    int value = -1;
    bool reading_number = false;
    for (const char ch : text)
    {
      if (ch == '{')
      {
        in_component = true;
        value = -1;
        reading_number = false;
      }
      else if (ch >= '0' && ch <= '9' && in_component)
      {
        if (!reading_number)
        {
          value = 0;
          reading_number = true;
        }
        value = value * 10 + (ch - '0');
      }
      else
      {
        if (reading_number)
        {
          if (value < 0 || value >= form_num_ || seen[value])
            return false;
          (*groups)[value] = component;
          seen[value] = true;
        }
        value = -1;
        reading_number = false;
        if (ch == '}' && in_component)
        {
          in_component = false;
          ++component;
        }
      }
    }
    if (reading_number)
    {
      if (value < 0 || value >= form_num_ || seen[value])
        return false;
      (*groups)[value] = component;
      seen[value] = true;
    }

    if (component <= 0 ||
        !std::all_of(groups->begin(), groups->end(),
                     [](int group) { return group >= 0; }))
      return false;

    // Component labels are presentation details.  Normalize them by first UAV
    // occurrence so semantically identical strings compare equal.
    std::vector<int> old_labels;
    for (int &group : *groups)
    {
      const auto found = std::find(old_labels.begin(), old_labels.end(), group);
      if (found == old_labels.end())
      {
        old_labels.push_back(group);
        group = static_cast<int>(old_labels.size()) - 1;
      }
      else
      {
        group = static_cast<int>(std::distance(old_labels.begin(), found));
      }
    }
    return true;
  }

  std::string EGOReplanFSM::canonicalTopology(
      const std::vector<int> &groups) const
  {
    if (groups.empty()) return "INTACT";
    const int component_count =
        *std::max_element(groups.begin(), groups.end()) + 1;
    std::ostringstream text;
    for (int component = 0; component < component_count; ++component)
    {
      if (component > 0) text << "|";
      text << "{";
      bool first = true;
      for (int id = 0; id < static_cast<int>(groups.size()); ++id)
      {
        if (groups[id] != component) continue;
        if (!first) text << ",";
        text << id;
        first = false;
      }
      text << "}";
    }
    return text.str();
  }

  void EGOReplanFSM::publishTopologyAck()
  {
    if (topology_epoch_ == 0) return;
    FormationTopologyAck ack;
    ack.header.stamp = ros::Time::now();
    ack.epoch = topology_epoch_;
    ack.drone_id = planner_manager_->pp_.drone_id;
    ack.mode = topology_mode_;
    ack.components = topology_components_;
    ack.edge_count = topology_edge_count_;
    ack.edge_keep_mask = topology_edge_keep_mask_;
    ack.weak_edge_mask = topology_weak_edge_mask_;
    std::copy(distributed_assignment_versions_.begin(),
              distributed_assignment_versions_.end(),
              ack.assignment_versions.begin());
    for (int id = 0; id < distributed::kAgents; ++id)
      ack.drone_to_slot[id] =
          id < static_cast<int>(formation_assignment_.size())
              ? formation_assignment_[id] : id;
    ack.group_id = dynamic_topology_active_
                       ? dynamicGroupForDrone(planner_manager_->pp_.drone_id)
                       : -1;
    ack.formation_scale = std::max(0.0, split_formation_scale_);
    ack.progress_sync_scale = std::max(0.0, split_progress_sync_scale_);
    topology_ack_pub_.publish(ack);
  }

  void EGOReplanFSM::logEffectiveTopology(const std::string &reason) const
  {
    std::ostringstream edges;
    std::vector<std::array<int, 2>> formation_edges;
    if (form_num_ == 6 && general_dynamic_topology_enabled_)
      formation_edges = {{{0, 1}}, {{1, 2}}, {{3, 4}}, {{4, 5}},
                         {{0, 3}}, {{1, 4}}, {{2, 5}}};
    else if (form_num_ == 8 && general_dynamic_topology_enabled_)
      formation_edges = {{{0, 1}}, {{1, 2}}, {{2, 3}},
                         {{4, 5}}, {{5, 6}}, {{6, 7}},
                         {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}};
    else
      formation_edges = {{{0, 1}}, {{1, 3}}, {{3, 2}}, {{2, 0}}};
    if (formation_assignment_.size() == static_cast<std::size_t>(form_num_))
    {
      std::vector<int> drone_for_slot(form_num_, -1);
      for (int drone = 0; drone < form_num_; ++drone)
        drone_for_slot[formation_assignment_[drone]] = drone;
      for (auto &edge : formation_edges)
      {
        edge[0] = drone_for_slot[edge[0]];
        edge[1] = drone_for_slot[edge[1]];
      }
    }
    for (std::size_t edge = 0; edge < formation_edges.size(); ++edge)
    {
      if (edge > 0) edges << " ";
      const int first = formation_edges[edge][0];
      const int second = formation_edges[edge][1];
      const bool have_exact_edges =
          topology_edge_count_ == formation_edges.size();
      const bool connected = have_exact_edges
          ? (topology_edge_keep_mask_ & (uint32_t{1} << edge)) != 0
          : (!dynamic_topology_active_ ||
             (first < static_cast<int>(dynamic_formation_groups_.size()) &&
              second < static_cast<int>(dynamic_formation_groups_.size()) &&
              dynamic_formation_groups_[first] == dynamic_formation_groups_[second]));
      const bool weak = connected && have_exact_edges &&
          (topology_weak_edge_mask_ & (uint32_t{1} << edge)) != 0;
      edges << first << "-" << second << ":"
            << (weak ? "weak" : connected ? "on" : "cut");
    }
    ROS_INFO_STREAM("[TOPOLOGY_APPLIED] drone="
                    << planner_manager_->pp_.drone_id
                    << " epoch=" << topology_epoch_
                    << " mode="
                    << (topology_mode_ == FormationTopologyCommand::SPLIT
                            ? "SPLIT" : "INTACT")
                    << " components=" << topology_components_
                    << " formation_scale=" << split_formation_scale_
                    << " progress_sync_scale=" << split_progress_sync_scale_
                    << " edges=[" << edges.str() << "] reason=" << reason);
  }

  void EGOReplanFSM::formationTopologyCommandCallback(
      const FormationTopologyCommand::ConstPtr &msg)
  {
    if (msg->mode != FormationTopologyCommand::INTACT &&
        msg->mode != FormationTopologyCommand::SPLIT)
    {
      ROS_ERROR("[FORMATION_BACKEND] Reject epoch %lu with invalid mode %u",
                static_cast<unsigned long>(msg->epoch), msg->mode);
      return;
    }
    const uint32_t expected_edge_count =
        form_num_ == 8 ? 10u : (form_num_ == 6 ? 7u : 0u);
    if (msg->edge_count != 0u &&
        (msg->edge_count != expected_edge_count ||
         msg->edge_keep_mask >= (uint32_t{1} << msg->edge_count)))
    {
      ROS_ERROR("[FORMATION_BACKEND] Reject epoch %lu with invalid edge mask %u/%u",
                static_cast<unsigned long>(msg->epoch),
                msg->edge_keep_mask, msg->edge_count);
      return;
    }
    if (msg->epoch < topology_epoch_)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[FORMATION_BACKEND] Ignore stale topology epoch %lu; current=%lu",
                        static_cast<unsigned long>(msg->epoch),
                        static_cast<unsigned long>(topology_epoch_));
      return;
    }

    std::vector<int> groups;
    std::string components = "INTACT";
    if (msg->mode == FormationTopologyCommand::SPLIT)
    {
      if ((form_num_ != 4 && !general_dynamic_topology_enabled_) ||
          !parseDynamicTopology(msg->components, &groups))
      {
        ROS_ERROR("[FORMATION_BACKEND] Reject epoch %lu with invalid components '%s'",
                  static_cast<unsigned long>(msg->epoch), msg->components.c_str());
        return;
      }
      components = canonicalTopology(groups);
    }

    const double formation_scale =
        msg->mode == FormationTopologyCommand::SPLIT
            ? std::max(0.0, msg->formation_scale) : 0.0;
    const double progress_scale =
        msg->mode == FormationTopologyCommand::SPLIT
            ? std::max(0.0, msg->progress_sync_scale) : 0.0;
    const bool same_payload =
        msg->mode == topology_mode_ && components == topology_components_ &&
        msg->edge_count == topology_edge_count_ &&
        msg->edge_keep_mask == topology_edge_keep_mask_ &&
        msg->weak_edge_mask == topology_weak_edge_mask_ &&
        std::fabs(formation_scale - split_formation_scale_) < 1e-6 &&
        std::fabs(progress_scale - split_progress_sync_scale_) < 1e-6;
    if (msg->epoch == topology_epoch_)
    {
      if (!same_payload)
      {
        ROS_ERROR("[FORMATION_BACKEND] Conflicting payload for topology epoch %lu",
                  static_cast<unsigned long>(msg->epoch));
        return;
      }
      publishFormationGroupState();
      publishTopologyAck();
      return;
    }

    const bool topology_changed =
        msg->mode != topology_mode_ || components != topology_components_;
    const bool edge_topology_changed =
        msg->edge_count != topology_edge_count_ ||
        msg->edge_keep_mask != topology_edge_keep_mask_;
    const bool weak_changed = msg->weak_edge_mask != topology_weak_edge_mask_;
    const bool scale_changed =
        std::fabs(formation_scale - split_formation_scale_) >= 1e-6 ||
        std::fabs(progress_scale - split_progress_sync_scale_) >= 1e-6;

    const int previous_component_count =
        dynamic_topology_active_ && !dynamic_formation_groups_.empty()
            ? *std::max_element(dynamic_formation_groups_.begin(),
                                dynamic_formation_groups_.end()) + 1
            : 1;
    const int next_component_count = groups.empty()
                                         ? 1
                                         : *std::max_element(groups.begin(),
                                                             groups.end()) + 1;
    const bool components_merged = topology_changed &&
        ((topology_mode_ == FormationTopologyCommand::SPLIT &&
          msg->mode == FormationTopologyCommand::INTACT) ||
         (msg->mode == FormationTopologyCommand::SPLIT &&
          next_component_count < previous_component_count));

    topology_epoch_ = msg->epoch;
    topology_mode_ = msg->mode;
    topology_components_ = components;
    topology_edge_count_ = msg->edge_count;
    topology_edge_keep_mask_ = msg->edge_keep_mask;
    topology_weak_edge_mask_ = msg->weak_edge_mask;
    manual_formation_split_ = msg->mode == FormationTopologyCommand::SPLIT;
    dynamic_topology_active_ = manual_formation_split_;
    dynamic_formation_groups_ = groups;
    split_formation_scale_ = formation_scale;
    split_progress_sync_scale_ = progress_scale;

    planner_manager_->setManualFormationSplit(manual_formation_split_);
    planner_manager_->setDynamicFormationGroups(dynamic_formation_groups_);
    if (manual_formation_split_ && topology_edge_count_ > 0 &&
        !planner_manager_->setDynamicFormationEdges(
            topology_edge_count_, topology_edge_keep_mask_,
            formation_assignment_))
      ROS_ERROR("[FORMATION_BACKEND] Failed to apply exact formation edge mask");
    planner_manager_->setSplitSubgroupFormationWeightScale(split_formation_scale_);
    planner_manager_->setSplitProgressSyncWeightScale(split_progress_sync_scale_);
    if (components_merged && have_odom_ && !distributed_enabled_)
      planner_manager_->beginMergeSynchronization(odom_pos_);

    if (!manual_formation_split_ && subgroup_route_intent_active_)
    {
      subgroup_route_intent_active_ = false;
      subgroup_route_intent_gain_ = 0.0;
      planner_manager_->setSubgroupRouteIntent(
          false, odom_pos_, Eigen::Vector3d::UnitX(), 0.0);
      publishSubgroupRouteIntentState("WAITING_SPLIT");
    }

    publishFormationGroupState();
    publishTopologyAck();
    if (topology_changed || edge_topology_changed || scale_changed || weak_changed)
      logEffectiveTopology(msg->reason);

    if ((topology_changed || edge_topology_changed || scale_changed) && have_target_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "TOPOLOGY_COMMAND");
  }

  void EGOReplanFSM::distributedTopologyCallback(
      const FormationTopologyCommand::ConstPtr &msg)
  {
    const ros::Time now = ros::Time::now();
    std::array<uint32_t, distributed::kColumns> received_versions{};
    std::copy(msg->assignment_versions.begin(),
              msg->assignment_versions.end(), received_versions.begin());
    const auto expected_assignment =
        distributed::assignmentFromVersions(received_versions);
    bool valid_assignment = true;
    for (int id = 0; id < distributed::kAgents; ++id)
      valid_assignment = valid_assignment &&
          msg->drone_to_slot[id] ==
              static_cast<uint32_t>(expected_assignment[id]);
    for (int column = 0; column < distributed::kColumns; ++column)
      valid_assignment = valid_assignment &&
          received_versions[column] >= distributed_assignment_versions_[column];
    if (msg->header.stamp.isZero() ||
        msg->header.stamp <= distributed_topology_stamp_ ||
        (now - msg->header.stamp).toSec() > distributed_topology_timeout_ ||
        (msg->header.stamp - now).toSec() > 0.05 ||
        msg->mode != FormationTopologyCommand::SPLIT ||
        msg->edge_count != distributed::kEdges ||
        msg->edge_keep_mask >= (uint32_t{1} << distributed::kEdges) ||
        (msg->weak_edge_mask & ~msg->edge_keep_mask) != 0 ||
        !std::isfinite(msg->formation_scale) || msg->formation_scale < 0 ||
        msg->progress_sync_scale != 0 ||
        !valid_assignment ||
        msg->components != distributed::components(
            msg->edge_keep_mask, expected_assignment) ||
        (msg->reason != "DISTRIBUTED_STARTUP_WAIT" &&
         msg->reason != "DISTRIBUTED_ENDPOINT_LEASES"))
      return;
    constexpr uint32_t full_mask =
        (uint32_t{1} << distributed::kEdges) - uint32_t{1};
    if (!distributed_startup_ready_ &&
        msg->reason == "DISTRIBUTED_ENDPOINT_LEASES")
    {
      // The barrier transition itself must still carry the complete graph.
      if (msg->edge_keep_mask != full_mask)
      {
        ROS_ERROR_THROTTLE(
            1.0, "Reject distributed startup release without a full edge mask");
        return;
      }
      distributed_startup_ready_ = true;
      ROS_INFO("[DISTRIBUTED_STARTUP] barrier passed; target flight enabled");
    }
    const bool assignment_changed =
        !std::equal(formation_assignment_.begin(), formation_assignment_.end(),
                    expected_assignment.begin());
    if (assignment_changed)
    {
      const std::vector<int> next_assignment(expected_assignment.begin(),
                                             expected_assignment.end());
      const std::vector<int> previous_assignment = formation_assignment_;
      const Eigen::Vector3d previous_pos = formation_pos_;
      if (!planner_manager_->setFormationAssignment(next_assignment)) return;
      formation_assignment_ = next_assignment;
      formation_pos_ = formation_slots_[expected_assignment[planner_manager_->pp_.drone_id]];
      if (have_target_ && !refreshAssignedWaypointTarget())
      {
        planner_manager_->setFormationAssignment(previous_assignment);
        formation_assignment_ = previous_assignment;
        formation_pos_ = previous_pos;
        expireDistributedTopology();
        ROS_ERROR_THROTTLE(1.0, "[SLOT_SWAP] target refresh failed; local edges cleared, retrying");
        return;
      }
      assignment_generation_ = distributed::assignmentSignature(received_versions);
    }
    distributed_assignment_versions_ = received_versions;
    FormationTopologyCommand::Ptr local(new FormationTopologyCommand(*msg));
    // Local application order: never compare epochs produced by different agents.
    local->epoch = topology_epoch_ + 1;
    formationTopologyCommandCallback(local);
    distributed_topology_stamp_ = msg->header.stamp;
    if (assignment_changed && have_target_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "SLOT_SWAP");
  }

  void EGOReplanFSM::expireDistributedTopology()
  {
    FormationTopologyCommand::Ptr local(new FormationTopologyCommand);
    local->epoch = topology_epoch_ + 1;
    local->mode = FormationTopologyCommand::SPLIT;
    local->edge_count = distributed::kEdges;
    local->edge_keep_mask = 0;
    local->weak_edge_mask = 0;
    distributed::Assignment assignment = distributed::identityAssignment();
    for (int id = 0; id < distributed::kAgents; ++id)
      assignment[id] = formation_assignment_[id];
    local->components = distributed::components(0, assignment);
    std::copy(distributed_assignment_versions_.begin(),
              distributed_assignment_versions_.end(),
              local->assignment_versions.begin());
    for (int id = 0; id < distributed::kAgents; ++id)
      local->drone_to_slot[id] = assignment[id];
    local->formation_scale = 0.15;
    local->progress_sync_scale = 0;
    local->reason = "LOCAL_AGENT_MISSING_OR_STALE";
    formationTopologyCommandCallback(local);
  }

  void EGOReplanFSM::initialAssignmentCallback(
      const std_msgs::UInt32MultiArray::ConstPtr &msg)
  {
    const bool versioned =
        msg->data.size() == static_cast<std::size_t>(form_num_ + 1);
    if (!versioned &&
        msg->data.size() != static_cast<std::size_t>(form_num_))
    {
      ROS_ERROR("[FORMATION_ASSIGNMENT] expected %d or %d entries, received %zu",
                form_num_, form_num_ + 1, msg->data.size());
      return;
    }
    const uint32_t generation = versioned ? msg->data[0] : 0;
    if ((versioned && generation <= assignment_generation_) ||
        (!versioned && initial_assignment_received_))
      return;
    const std::size_t offset = versioned ? 1 : 0;

    std::vector<int> assignment(form_num_, -1);
    std::vector<bool> used(form_num_, false);
    for (int id = 0; id < form_num_; ++id)
    {
      const int slot = static_cast<int>(msg->data[id + offset]);
      if (slot < 0 || slot >= form_num_ || used[slot])
      {
        ROS_ERROR("[FORMATION_ASSIGNMENT] rejected invalid permutation");
        return;
      }
      assignment[id] = slot;
      used[slot] = true;
    }
    if (!planner_manager_->setFormationAssignment(assignment))
    {
      ROS_ERROR("[FORMATION_ASSIGNMENT] optimizer rejected assignment");
      return;
    }

    const int my_id = planner_manager_->pp_.drone_id;
    const std::vector<int> previous_assignment = formation_assignment_;
    const Eigen::Vector3d previous_formation_pos = formation_pos_;
    const bool changed = assignment != formation_assignment_;
    const bool my_slot_changed =
        assignment[my_id] != formation_assignment_[my_id];
    formation_assignment_ = assignment;
    formation_pos_ = formation_slots_[assignment[my_id]];
    if (manual_formation_split_ && topology_edge_count_ > 0 &&
        !planner_manager_->setDynamicFormationEdges(
            topology_edge_count_, topology_edge_keep_mask_,
            formation_assignment_))
      ROS_ERROR("[FORMATION_ASSIGNMENT] unable to remap exact formation edges");
    if (my_slot_changed && have_target_ && !refreshAssignedWaypointTarget())
    {
      planner_manager_->setFormationAssignment(previous_assignment);
      formation_assignment_ = previous_assignment;
      formation_pos_ = previous_formation_pos;
      if (manual_formation_split_ && topology_edge_count_ > 0)
        planner_manager_->setDynamicFormationEdges(
            topology_edge_count_, topology_edge_keep_mask_,
            formation_assignment_);
      ROS_ERROR_STREAM("[FORMATION_ASSIGNMENT] generation=" << generation
                       << " unable to refresh assigned waypoint; assignment rolled back");
      return;
    }
    assignment_generation_ = generation;
    initial_assignment_received_ = true;
    std_msgs::UInt32MultiArray ack;
    ack.data = {generation, static_cast<uint32_t>(my_id)};
    assignment_ack_pub_.publish(ack);
    ROS_WARN_STREAM("[FORMATION_ASSIGNMENT] generation=" << generation
                    << " drone " << my_id << " assigned slot "
                    << assignment[my_id] << " offset="
                    << formation_pos_.transpose());
    if (changed && have_target_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "FORMATION_REASSIGNMENT");
  }

  void EGOReplanFSM::manualFormationSplitCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if (form_num_ != 4 && !general_dynamic_topology_enabled_)
    {
      ROS_ERROR_THROTTLE(
          2.0,
          "Manual formation split for non-four-UAV formations is disabled; "
          "set formation/general_dynamic_topology_enabled=true explicitly.");
      return;
    }
    if (manual_formation_split_ == msg->data)
    {
      publishFormationGroupState();
      return;
    }

    manual_formation_split_ = msg->data;
    planner_manager_->setManualFormationSplit(manual_formation_split_);
    if (manual_formation_split_ && dynamic_topology_active_)
      planner_manager_->setDynamicFormationGroups(dynamic_formation_groups_);
    if (!manual_formation_split_ && subgroup_route_intent_active_)
    {
      subgroup_route_intent_active_ = false;
      subgroup_route_intent_gain_ = 0.0;
      planner_manager_->setSubgroupRouteIntent(
          false, odom_pos_, Eigen::Vector3d::UnitX(), 0.0);
      publishSubgroupRouteIntentState("WAITING_SPLIT");
    }
    if (!manual_formation_split_)
    {
      dynamic_topology_active_ = false;
      dynamic_formation_groups_.clear();
      planner_manager_->setDynamicFormationGroups(dynamic_formation_groups_);
    }
    publishFormationGroupState();
    ROS_WARN("[FORMATION_BACKEND] drone %d switched to %s",
             planner_manager_->pp_.drone_id,
             manual_formation_split_
                 ? ("SPLIT/C" +
                    std::to_string(dynamicGroupForDrone(planner_manager_->pp_.drone_id))).c_str()
                 : "INTACT");

    // Apply the new formation cost from the next optimization.  Collision
    // avoidance remains global because swarm trajectories are not filtered.
    if (have_target_ && (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
                         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "MANUAL_TOPOLOGY");
  }

  void EGOReplanFSM::splitFormationScaleCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    const double scale = std::max(0.0, msg->data);
    if (std::fabs(scale - split_formation_scale_) < 1e-6)
      return;
    split_formation_scale_ = scale;
    planner_manager_->setSplitSubgroupFormationWeightScale(split_formation_scale_);
    if (have_target_ && manual_formation_split_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "SPLIT_FORMATION_SCALE");
  }

  void EGOReplanFSM::splitProgressSyncScaleCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    const double scale = std::max(0.0, msg->data);
    if (std::fabs(scale - split_progress_sync_scale_) < 1e-6)
      return;
    split_progress_sync_scale_ = scale;
    planner_manager_->setSplitProgressSyncWeightScale(split_progress_sync_scale_);
    if (have_target_ && manual_formation_split_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
         exec_state_ == GEN_NEW_TRAJ))
      changeFSMExecState(REPLAN_TRAJ, "SPLIT_PROGRESS_SYNC_SCALE");
  }

  void EGOReplanFSM::publishSubgroupRouteIntentState(
      const std::string &state)
  {
    std_msgs::String message;
    message.data = state;
    subgroup_route_intent_status_pub_.publish(message);
  }

  void EGOReplanFSM::subgroupRouteIntentCallback(
      const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    // Two group records with the schema documented by the publisher:
    // group, member0, member1, cx, cy, angle, free, usable, score,
    // coordination_active, lock_active, locked_side, feedback_gain.  Older
    // publishers used a Boolean in the last field, which remains compatible.
    constexpr std::size_t stride = 13;
    const bool per_drone_records =
        msg->data.size() >= static_cast<std::size_t>(form_num_) * stride;
    const std::size_t group =
        per_drone_records
            ? static_cast<std::size_t>(planner_manager_->pp_.drone_id)
            : (planner_manager_->pp_.drone_id < 2 ? 0 : 1);
    const std::size_t base = group * stride;
    if (msg->data.size() < base + stride)
      return;

    const double gain = std::max(0.0, std::min(1.0, msg->data[base + 12]));
    const bool active = gain > 1e-3 &&
                        manual_formation_split_ &&
                        subgroup_route_intent_feedback_enabled_;
    const double angle = msg->data[base + 5];
    const bool was_active = subgroup_route_intent_active_;
    const double previous_gain = subgroup_route_intent_gain_;
    const double angle_step = std::fabs(
        std::atan2(std::sin(angle - subgroup_route_intent_angle_),
                   std::cos(angle - subgroup_route_intent_angle_)));

    subgroup_route_intent_active_ = active;
    subgroup_route_intent_angle_ = angle;
    subgroup_route_intent_gain_ = active ? gain : 0.0;
    subgroup_route_intent_stamp_ = ros::Time::now();
    const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0.0);
    planner_manager_->setSubgroupRouteIntent(
        active, odom_pos_, direction, subgroup_route_intent_gain_);

    if (!active)
    {
      if (was_active)
      {
        publishSubgroupRouteIntentState("RELEASED");
        if (exec_state_ == EXEC_TRAJ)
          changeFSMExecState(REPLAN_TRAJ, "SUBGROUP_ROUTE_RELEASED");
      }
      return;
    }

    const int locked_side = static_cast<int>(std::round(msg->data[base + 11]));
    publishSubgroupRouteIntentState(
        std::string("ACTIVE side=") + (locked_side < 0 ? "-" : "+") +
        " angle=" + std::to_string(angle) +
        " gain=" + std::to_string(subgroup_route_intent_gain_));

    const ros::Time now = ros::Time::now();
    const bool request_replan = !was_active ||
                                angle_step >= subgroup_route_intent_replan_angle_ ||
                                std::fabs(subgroup_route_intent_gain_ -
                                          previous_gain) >=
                                    subgroup_route_intent_replan_gain_step_;
    if (request_replan && exec_state_ == EXEC_TRAJ &&
        (subgroup_route_intent_last_replan_.isZero() ||
         (now - subgroup_route_intent_last_replan_).toSec() >=
             subgroup_route_intent_replan_interval_))
    {
      subgroup_route_intent_last_replan_ = now;
      changeFSMExecState(REPLAN_TRAJ, "SUBGROUP_ROUTE_INTENT");
    }
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);
    const ros::Time now = ros::Time::now();
    if (initial_assignment_enabled_ && initial_assignment_received_ &&
        (last_assignment_ack_stamp_.isZero() ||
         (now - last_assignment_ack_stamp_).toSec() >= 0.5))
    {
      std_msgs::UInt32MultiArray ack;
      ack.data = {assignment_generation_,
                  static_cast<uint32_t>(planner_manager_->pp_.drone_id)};
      assignment_ack_pub_.publish(ack);
      last_assignment_ack_stamp_ = now;
    }

    if (distributed_enabled_ && !distributed_topology_stamp_.isZero() &&
        (now - distributed_topology_stamp_).toSec() > distributed_topology_timeout_)
    {
      expireDistributedTopology();
      distributed_topology_stamp_ = ros::Time();
    }

    if (subgroup_route_intent_active_ &&
        (now - subgroup_route_intent_stamp_).toSec() >
            subgroup_route_intent_timeout_)
    {
      subgroup_route_intent_active_ = false;
      subgroup_route_intent_gain_ = 0.0;
      planner_manager_->setSubgroupRouteIntent(
          false, odom_pos_, Eigen::Vector3d::UnitX(), 0.0);
      publishSubgroupRouteIntentState("STALE_DISABLED");
      ROS_ERROR_THROTTLE(
          2.0, "Subgroup route intent timed out; dynamic guidance disabled.");
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_ || !initial_assignment_received_)
        goto force_return; // return;
      if (distributed_enabled_ && !distributed_startup_ready_)
      {
        const double elapsed = distributed_startup_begin_.isZero()
                                   ? 0.0
                                   : (now - distributed_startup_begin_).toSec();
        if (elapsed >= distributed_startup_timeout_)
          ROS_ERROR_THROTTLE(
              2.0,
              "[DISTRIBUTED_STARTUP] not ready after %.1f s; holding position "
              "(fail-closed)",
              elapsed);
        else
          ROS_WARN_THROTTLE(
              2.0,
              "[DISTRIBUTED_STARTUP] waiting for all-agent full-edge barrier "
              "(%.1f/%.1f s)",
              elapsed, distributed_startup_timeout_);
        goto force_return;
      }
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (distributed_enabled_ || planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_ERROR("Failed to generate the first trajectory!!!");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wpt_id_ < waypoint_num_ - 1) &&
          (end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        formation_start_ = wps_[wpt_id_];
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_], formation_start_);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // local target close to the global target
      {
        const double goal_error = (end_pt_ - odom_pos_).norm();
        const bool wall_safe =
            !planner_manager_->violatesWallCollisionGuard(odom_pos_);
        const bool map_safe =
            !planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
        const bool actually_arrived =
            goal_error <= arrival_position_tolerance_ &&
            odom_vel_.norm() <= arrival_speed_tolerance_ && wall_safe && map_safe;

        if (!actually_arrived)
        {
          ROS_ERROR_THROTTLE(
              1.0,
              "[arrival_guard] drone %d trajectory expired but vehicle has not safely arrived: goal_error=%.3f speed=%.3f map_occ=%d wall_guard=%d odom=(%.3f,%.3f,%.3f). Replan from odometry.",
              planner_manager_->pp_.drone_id, goal_error, odom_vel_.norm(),
              static_cast<int>(!map_safe), static_cast<int>(!wall_safe),
              odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
          if (!wall_safe)
          {
            latchFirstWallViolation("arrival_guard", !map_safe, info, t_cur);
            callEmergencyStop(odom_pos_);
            enable_fail_safe_ = false;
            changeFSMExecState(EMERGENCY_STOP,
                               "SAFETY_CURRENT_WALL_GUARD");
          }
          else
          {
            changeFSMExecState(GEN_NEW_TRAJ, "ARRIVAL_GUARD_REPLAN");
          }
          goto force_return;
        }

        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          formation_start_ = wps_[wpt_id_];
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_], formation_start_);
          // have_trigger_ = false; // must have trigger
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
        goto force_return;
      }
      else if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
        {
          ROS_ERROR("[arrival_guard] drone %d goal is occupied; task is not complete.",
                    planner_manager_->pp_.drone_id);
          callEmergencyStop(odom_pos_);
          enable_fail_safe_ = false;
          changeFSMExecState(EMERGENCY_STOP, "SAFETY_GOAL_OCCUPIED");
          goto force_return;
        }
        else
        {
          // pass;
        }
      }
      else if (t_cur > replan_thresh_ ||
               (!touch_the_goal && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP)
      last_emergency_reason_ = pos_call;
    cout << "[drone " << planner_manager_->pp_.drone_id << "][" << pos_call
         << "]: from " << state_str[pre_s] << " to "
         << state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    // static int last_printed_state = -1, dot_nums = 0;

    // if (exec_state_ != last_printed_state)
    //   dot_nums = 0;
    // else
    //   dot_nums++;

    cout << "\r[FSM][drone " << planner_manager_->pp_.drone_id
         << "]: state: " << state_str[int(exec_state_)];

    // last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }
    if (exec_state_ == EMERGENCY_STOP && !last_emergency_reason_.empty())
    {
      cout << ", reason=" << last_emergency_reason_;
    }

    cout << endl;

    // cout << string(dot_nums, '.');

    // fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::publishSafetyStatus(
      const int map_occupancy,
      const bool wall_guard_violation,
      const LocalTrajData *info,
      const double trajectory_time)
  {
    const ros::Time now = ros::Time::now();
    if (!last_safety_status_stamp_.isZero() &&
        (now - last_safety_status_stamp_).toSec() < 0.20)
      return;
    last_safety_status_stamp_ = now;

    static const char *state_names[] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
        "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    const int state_index = static_cast<int>(exec_state_);
    const char *state = state_index >= 0 && state_index < 7
                            ? state_names[state_index]
                            : "UNKNOWN";
    const int traj_id = info ? info->traj_id : -1;
    const double goal_error = have_target_ ? (end_pt_ - odom_pos_).norm() : -1.0;

    std_msgs::String message;
    std::ostringstream stream;
    stream << "drone=" << planner_manager_->pp_.drone_id
           << " state=" << state
           << " traj_id=" << traj_id
           << " traj_t=" << trajectory_time
           << " odom=(" << odom_pos_.x() << "," << odom_pos_.y() << ","
           << odom_pos_.z() << ")"
           << " speed=" << odom_vel_.norm()
           << " goal_error=" << goal_error
           << " map_occ=" << map_occupancy
           << " wall_guard=" << static_cast<int>(wall_guard_violation)
           << " target=" << static_cast<int>(have_target_);
    message.data = stream.str();
    safety_status_pub_.publish(message);
  }

  void EGOReplanFSM::latchFirstWallViolation(
      const std::string &source,
      const int map_occupancy,
      const LocalTrajData *info,
      const double trajectory_time)
  {
    if (first_wall_violation_latched_)
      return;
    first_wall_violation_latched_ = true;

    std_msgs::String message;
    std::ostringstream stream;
    stream << "source=" << source
           << " drone=" << planner_manager_->pp_.drone_id
           << " stamp=" << ros::Time::now().toSec()
           << " state=" << static_cast<int>(exec_state_)
           << " traj_id=" << (info ? info->traj_id : -1)
           << " traj_t=" << trajectory_time
           << " odom=(" << odom_pos_.x() << "," << odom_pos_.y() << ","
           << odom_pos_.z() << ")"
           << " velocity=(" << odom_vel_.x() << "," << odom_vel_.y() << ","
           << odom_vel_.z() << ")"
           << " map_occ=" << map_occupancy
           << " wall_guard=1";
    message.data = stream.str();
    first_wall_violation_pub_.publish(message);
    ROS_ERROR_STREAM("[FIRST_WALL_VIOLATION] " << message.data);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    double height;
    measureGroundHeight(height);

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    const int current_map_occupancy =
        have_odom_ ? map->getInflateOccupancy(odom_pos_) : -2;
    const bool current_wall_guard =
        have_odom_ && planner_manager_->violatesWallCollisionGuard(odom_pos_);
    publishSafetyStatus(current_map_occupancy, current_wall_guard, info, t_cur);

    // This is a hard runtime invariant, independent of trajectory age and FSM
    // state.  It turns a silent wall-crossing/completion into an explicit,
    // latched failure with the map result recorded at the same odometry sample.
    if (current_wall_guard)
    {
      latchFirstWallViolation("current_odometry", current_map_occupancy,
                              info, t_cur);
      if (!collision_recovery_active_)
      {
        Eigen::Vector3d recovery_target;
        const bool recovery_installed =
            planner_manager_->findNearestCollisionFreePosition(
                odom_pos_, recovery_target) &&
            callEmergencyEscape(odom_pos_, recovery_target);
        if (!recovery_installed)
          callEmergencyStop(odom_pos_);

        // Preserve the escape command. Otherwise the EMERGENCY_STOP state's
        // next tick would replace it with a stationary trajectory.
        if (recovery_installed)
        {
          flag_escape_emergency_ = false;
          collision_recovery_active_ = true;
          fail_safe_before_collision_recovery_ = enable_fail_safe_;
          collision_recovery_target_ = recovery_target;
        }
        enable_fail_safe_ = false;
        changeFSMExecState(EMERGENCY_STOP,
                           recovery_installed
                               ? "SAFETY_COLLISION_RECOVERY"
                               : "SAFETY_CURRENT_WALL_GUARD");
      }
      return;
    }

    if (collision_recovery_active_)
    {
      // Leaving the inflated cell is not enough: allow the escape trajectory
      // to settle at its buffered endpoint before returning to normal
      // planning.  Otherwise the normal planner can overwrite it mid-escape.
      const bool recovery_finished =
          t_cur >= info->duration - 0.05 && odom_vel_.norm() < 0.15 &&
          (odom_pos_ - collision_recovery_target_).norm() < 0.15;
      if (recovery_finished)
      {
        collision_recovery_active_ = false;
        enable_fail_safe_ = fail_safe_before_collision_recovery_;
        flag_escape_emergency_ = true;
        ROS_WARN("[drone %d] Collision recovery complete; resuming planning.",
                 planner_manager_->pp_.drone_id);
        changeFSMExecState(GEN_NEW_TRAJ,
                           "SAFETY_COLLISION_RECOVERY_COMPLETE");
      }
      return;
    }

    // Break symmetric swarm deadlocks deterministically.  When two stopped
    // agents are already inside the safety radius, only the higher-id agent
    // performs a short obstacle-aware move away from the lower-id agent.  The
    // lower-id agent keeps priority, so both sides cannot choose mirrored
    // avoidance manoeuvres and block each other indefinitely.
    int closest_peer_id = -1;
    double closest_peer_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d closest_peer_position;
    const double now = ros::Time::now().toSec();
    const double deadlock_distance =
        0.8 * planner_manager_->getSwarmClearance();
    const bool locally_stopped =
        have_target_ && odom_vel_.norm() < 0.15 &&
        exec_state_ != INIT && exec_state_ != WAIT_TARGET;
    for (size_t id = 0;
         locally_stopped &&
         id < planner_manager_->traj_.swarm_traj.size(); ++id)
    {
      const auto &peer = planner_manager_->traj_.swarm_traj[id];
      if (peer.drone_id != static_cast<int>(id) ||
          peer.drone_id == planner_manager_->pp_.drone_id ||
          peer.start_time < 1e9 || peer.duration <= 0.0)
        continue;

      const double peer_time =
          std::max(0.0, std::min(peer.duration, now - peer.start_time));
      const Eigen::Vector3d peer_position = peer.traj.getPos(peer_time);
      const Eigen::Vector3d peer_velocity = peer.traj.getVel(peer_time);
      const double distance = (odom_pos_ - peer_position).norm();
      if (peer_velocity.norm() < 0.15 && distance < deadlock_distance &&
          distance < closest_peer_distance)
      {
        closest_peer_id = static_cast<int>(id);
        closest_peer_distance = distance;
        closest_peer_position = peer_position;
      }
    }

    if (closest_peer_id >= 0 &&
        planner_manager_->pp_.drone_id > closest_peer_id)
    {
      const Eigen::Vector3d away_direction =
          odom_pos_ - closest_peer_position;
      const double separation_distance = std::max(
          0.6, planner_manager_->getSwarmClearance() -
                   closest_peer_distance + 0.30);
      Eigen::Vector3d recovery_target;
      const bool recovery_installed =
          planner_manager_->findDirectionalCollisionFreePosition(
              odom_pos_, away_direction, separation_distance,
              recovery_target) &&
          callEmergencyEscape(odom_pos_, recovery_target);
      if (recovery_installed)
      {
        flag_escape_emergency_ = false;
        collision_recovery_active_ = true;
        fail_safe_before_collision_recovery_ = enable_fail_safe_;
        collision_recovery_target_ = recovery_target;
        enable_fail_safe_ = false;
        ROS_WARN("[drone %d] Resolving swarm deadlock with lower-id drone %d; "
                 "distance=%.3f, escape_target=(%.3f, %.3f, %.3f).",
                 planner_manager_->pp_.drone_id, closest_peer_id,
                 closest_peer_distance, recovery_target.x(),
                 recovery_target.y(), recovery_target.z());
        changeFSMExecState(EMERGENCY_STOP,
                           "SAFETY_SWARM_DEADLOCK_RECOVERY");
        return;
      }
    }

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("[drone %d] Depth lost; entering EMERGENCY_STOP.",
                planner_manager_->pp_.drone_id);
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY_DEPTH_LOST");
      return;
    }

    // bool close_to_the_end_of_safe_segment = (pts_chk.back().back().first - t_cur) < emergency_time_;
    // // bool close_to_goal = (info->traj.getPos(info->duration) - end_pt_).norm() < 1e-5;
    // if (close_to_the_end_of_safe_segment)
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   return;

    //   // if (!close_to_goal)
    //   // {
    //   //   // ROS_INFO("current position is close to the safe segment end.");
    //   //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //   return;
    //   // }
    //   // else
    //   // {
    //   //   double t_step = map->getResolution() / planner_manager_->pp_.max_vel_;
    //   //   for (double t = pts_chk.back().back().first; t < info->duration; t += t_step)
    //   //   {
    //   //     if (map->getInflateOccupancy(info->traj.getPos(t)))
    //   //     {
    //   //       if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
    //   //       {
    //   //         ROS_ERROR("Dense obstacles close to the goal, stop planning.");
    //   //         callEmergencyStop(odom_pos_);
    //   //         have_target_ = false;
    //   //         changeFSMExecState(WAIT_TARGET, "SAFETY");
    //   //         return;
    //   //       }
    //   //       else
    //   //       {
    //   //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //         return;
    //   //       }
    //   //     }
    //   //   }
    //   // }
    // }

    /* ---------- check trajectory ---------- */
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    const double wall_guard_horizon = planner_manager_->getWallGuardCheckHorizon();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);

    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;

    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());
    if (i_start >= pts_chk.size())
    {
      // ROS_ERROR("i_start >= pts_chk.size()");
      return;
    }
    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    // Eigen::Vector3d last_pt = pts_chk[0][0].second;
    // for (size_t i = 0; i < pts_chk.size(); ++i)
    // {
    //   cout << "--------------------" << endl;
    //   for (size_t j = 0; j < pts_chk[i].size(); ++j)
    //   {
    //     cout << pts_chk[i][j].first << " @ " << pts_chk[i][j].second.transpose() << " @ " << (pts_chk[i][j].second - last_pt).transpose() << " @ " << map->getInflateOccupancy(pts_chk[i][j].second) << endl;
    //     last_pt = pts_chk[i][j].second;
    //   }
    // }

    // cout << "pts_chk[i_start][j_start].first - t_cur = " << pts_chk[i_start][j_start].first - t_cur << endl;
    // cout << "devi = " << (pts_chk[i_start][j_start].second - info->traj.getPos(t_cur)).transpose() << endl;

    // cout << "pts_chk.size()=" << pts_chk.size() << " i_start=" << i_start << endl;
    // Eigen::Vector3d p_last = pts_chk[i_start][j_start].second;
    const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    Eigen::Vector3d previous_checked =
        have_odom_ ? odom_pos_ : info->traj.getPos(std::max(0.0, t_cur));
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;
        // if ( (p - p_last).cwiseAbs().maxCoeff() > planner_manager_->grid_map_->getResolution() * 1.05 )
        // {
        //   ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
        //   cout << "p=" << p.transpose() << " p_last=" << p_last.transpose() << " dist=" << (p - p_last).cwiseAbs().maxCoeff() << endl;
        // }
        // p_last = p;

        // cout << "t=" << t << " @ "
        //      << "p=" << p.transpose() << endl;
        // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        // if (t_cur < t_2_3 && t >= t_2_3)
        //   break;

        bool occ = false;
        std::string collision_reason;
        if (map->getInflateOccupancy(p))
        {
          occ = true;
          collision_reason = "map_inflate_occupancy";
        }
        const bool inside_wall_horizon =
            wall_guard_horizon <= 0.0 || t - t_cur <= wall_guard_horizon;
        if (inside_wall_horizon &&
            (planner_manager_->violatesWallCollisionGuard(p) ||
             planner_manager_->segmentViolatesWallCollisionGuard(
                 previous_checked, p)))
        {
          ROS_ERROR_THROTTLE(
              0.5,
              "[traj_safety] Wall guard predicts segment collision for drone %d in %.2fs from=(%.3f, %.3f, %.3f) to=(%.3f, %.3f, %.3f).",
              planner_manager_->pp_.drone_id,
              t - t_cur,
              previous_checked.x(), previous_checked.y(), previous_checked.z(),
              p.x(), p.y(), p.z());
          occ = true;
          collision_reason = "wall_guard_segment";
        }

        // cout << "p=" << p.transpose() << endl;

        // if (occ)
        // {
        //   ROS_WARN("AAAAAAAAAAAAAAAAAAA");
        //   cout << "pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
        //   cout << "i=" << i << " j=" << j << " i_start=" << i_start << " j_start=" << j_start << endl;
        //   cout << "pts_chk.size()=" << pts_chk.size() << endl;
        //   cout << "t=" << t << endl;
        //   cout << "from t=" << info->traj.getPos(t).transpose() << endl;
        //   cout << "from rec=" << p.transpose() << endl;
        // }

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();

            if (dist < CLEARANCE)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              occ = true;
              collision_reason =
                  "swarm_close_to_drone_" + std::to_string(static_cast<int>(id));
              break;
            }
          }
        }

        if (occ)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("[drone %d] Plan success after predicted collision (%s). ratio=%f",
                     planner_manager_->pp_.drone_id,
                     collision_reason.c_str(),
                     t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("[drone %d] Emergency stop because %s is %.3fs ahead at p=(%.3f, %.3f, %.3f).",
                       planner_manager_->pp_.drone_id,
                       collision_reason.c_str(),
                       t - t_cur,
                       p.x(), p.y(), p.z());
              changeFSMExecState(EMERGENCY_STOP, "SAFETY_COLLISION_NEAR");
            }
            else
            {
              ROS_WARN("[drone %d] Current trajectory predicts %s in %.3fs; replan.",
                       planner_manager_->pp_.drone_id,
                       collision_reason.c_str(),
                       t - t_cur);
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
        previous_checked = p;
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (!planner_manager_->EmergencyStop(stop_pos))
    {
      ROS_ERROR("[drone %d] Failed to install emergency-stop trajectory.",
                planner_manager_->pp_.drone_id);
      return false;
    }

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callEmergencyEscape(
      const Eigen::Vector3d &start_pos,
      const Eigen::Vector3d &free_pos)
  {
    if (!planner_manager_->EmergencyEscape(start_pos, free_pos))
    {
      ROS_ERROR("[drone %d] Failed to install collision-recovery trajectory.",
                planner_manager_->pp_.drone_id);
      return false;
    }

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);
    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        formation_start_, wps_[wpt_id_],
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    ++planning_attempt_count_;
    if (plan_success)
      ++planning_success_count_;
    else
      ++planning_failure_count_;
    std_msgs::String planning_metrics;
    std::ostringstream metrics_stream;
    metrics_stream << "{\"drone_id\":" << planner_manager_->pp_.drone_id
                   << ",\"attempts\":" << planning_attempt_count_
                   << ",\"successes\":" << planning_success_count_
                   << ",\"failures\":" << planning_failure_count_ << "}";
    planning_metrics.data = metrics_stream.str();
    planning_metrics_pub_.publish(planning_metrics);
    if (!plan_success)
      ROS_WARN_STREAM("[PLANNING_METRICS] " << planning_metrics.data);

    if (plan_success)
    {

      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  Eigen::Vector3d EGOReplanFSM::assignedWaypointTarget(
      const Eigen::Vector3d &next_wp,
      const Eigen::Vector3d &previous_wp) const
  {
    const Eigen::Vector3d displacement = next_wp - previous_wp;
    if (displacement.head<2>().norm() < 1e-6)
      return next_wp + formation_pos_;
    const Eigen::Vector3d dir = displacement.normalized();
    return next_wp +
           Eigen::Vector3d(dir(0) * formation_pos_(0) -
                               dir(1) * formation_pos_(1),
                           dir(1) * formation_pos_(0) +
                               dir(0) * formation_pos_(1),
                           formation_pos_(2));
  }

  bool EGOReplanFSM::refreshAssignedWaypointTarget()
  {
    if (!have_odom_ || wpt_id_ < 0 ||
        wpt_id_ >= static_cast<int>(wps_.size()))
      return false;

    const Eigen::Vector3d previous_end = end_pt_;
    const Eigen::Vector3d assigned_end =
        assignedWaypointTarget(wps_[wpt_id_], formation_start_);
    std::vector<Eigen::Vector3d> one_pt_wps{assigned_end};
    if (!planner_manager_->planGlobalTrajWaypoints(
            odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), one_pt_wps,
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false))
      return false;

    end_pt_ = assigned_end;
    have_new_target_ = true;
    touch_goal_ = false;
    visualization_->displayGoalPoint(
        assigned_end, Eigen::Vector4d(1.0, 0.5, 0.0, 1.0), 0.25,
        form_num_ + planner_manager_->pp_.drone_id);
    ROS_WARN_STREAM("[FORMATION_ASSIGNMENT_TARGET] drone="
                    << planner_manager_->pp_.drone_id
                    << " old_end=(" << previous_end.transpose()
                    << ") new_end=(" << assigned_end.transpose() << ")");
    return true;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp)
  {
    end_pt_ = assignedWaypointTarget(next_wp, previous_wp);

    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;
      // have_trigger_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory! Undefined actions!");
    }
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    Eigen::Vector3d next_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
    wps_.push_back(next_wp);
    wpt_id_ = wps_.size() - 1;
    if (wpt_id_ >= 1)
    {
      formation_start_ = wps_[wpt_id_ - 1];
    }
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  // void EGOReplanFSM::planGlobalTrajbyGivenWps()
  // {
  //   std::vector<Eigen::Vector3d> wps(waypoint_num_);
  //   for (int i = 0; i < waypoint_num_; i++)
  //   {
  //     wps[i](0) = waypoints_[i][0];
  //     wps[i](1) = waypoints_[i][1];
  //     wps[i](2) = waypoints_[i][2];

  //     end_pt_ = wps.back();
  //   }
  //   bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero(), wps,
  //                                                            Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero());

  //   for (size_t i = 0; i < (size_t)waypoint_num_; i++)
  //   {
  //     visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //     ros::Duration(0.001).sleep();
  //   }

  //   if (success)
  //   {

  //     /*** display ***/
  //     constexpr double step_size_t = 0.1;
  //     int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
  //     std::vector<Eigen::Vector3d> gloabl_traj(i_end);
  //     for (int i = 0; i < i_end; i++)
  //     {
  //       gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
  //     }

  //     have_target_ = true;
  //     have_new_target_ = true;

  //     // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  //     ros::Duration(0.001).sleep();
  //     visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  //     ros::Duration(0.001).sleep();
  //   }
  //   else
  //   {
  //     ROS_ERROR("Unable to generate global trajectory!");
  //   }
  // }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    if (!have_odom_)
    {
      ROS_ERROR("Reject formation flight!");
      return;
    }
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("[drone %d] Received a mandatory stop command!",
              planner_manager_->pp_.drone_id);
    changeFSMExecState(EMERGENCY_STOP, "MANDATORY_STOP");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        bool received_all_preceding_agents = true;
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            received_all_preceding_agents = false;
            break;
          }
        }
        have_recv_pre_agent_ = received_all_preceding_agents;
      }
    }
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner

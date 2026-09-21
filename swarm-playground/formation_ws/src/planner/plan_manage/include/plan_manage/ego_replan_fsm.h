#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>
#include <ego_planner/FormationTopologyAck.h>
#include <ego_planner/FormationTopologyCommand.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double arrival_position_tolerance_{0.30};
    double arrival_speed_tolerance_{0.15};
    double waypoints_[50][3];
    int form_num_;
    Eigen::MatrixXd formation_;
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool flag_escape_emergency_;
    bool collision_recovery_active_{false};
    bool fail_safe_before_collision_recovery_{true};
    Eigen::Vector3d collision_recovery_target_{Eigen::Vector3d::Zero()};
    bool manual_formation_split_;
    bool general_dynamic_topology_enabled_{false};
    bool subgroup_route_intent_feedback_enabled_;
    std::string subgroup_route_intent_topic_;
    bool initial_assignment_enabled_{false};
    bool initial_assignment_received_{false};
    std::vector<Eigen::Vector3d> formation_slots_;
    std::vector<int> formation_assignment_;
    uint32_t assignment_generation_{0};
    bool subgroup_route_intent_active_{false};
    bool dynamic_topology_active_{false};
    std::vector<int> dynamic_formation_groups_;
    uint64_t topology_epoch_{0};
    uint8_t topology_mode_{FormationTopologyCommand::INTACT};
    std::string topology_components_{"INTACT"};
    uint32_t topology_edge_count_{0};
    uint32_t topology_edge_keep_mask_{0};
    double subgroup_route_intent_angle_{0.0};
    double subgroup_route_intent_gain_{0.0};
    double subgroup_route_intent_timeout_{1.0};
    double subgroup_route_intent_replan_angle_{0.261799};
    double subgroup_route_intent_replan_gain_step_{0.20};
    double subgroup_route_intent_replan_interval_{0.5};
    double split_formation_scale_{-1.0};
    double split_progress_sync_scale_{-1.0};
    ros::Time subgroup_route_intent_stamp_;
    ros::Time subgroup_route_intent_last_replan_;
    ros::Time last_assignment_ack_stamp_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    bool first_wall_violation_latched_{false};
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};
    std::string last_emergency_reason_;

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d end_pt_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;

    Eigen::Vector3d formation_start_;
    // int formation_num_;
    Eigen::Vector3d formation_pos_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, manual_split_sub_, dynamic_topology_sub_, topology_command_sub_, split_formation_scale_sub_, split_progress_sync_scale_sub_, subgroup_route_intent_sub_, initial_assignment_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_, formation_group_pub_, topology_ack_pub_, assignment_ack_pub_, subgroup_route_intent_status_pub_, safety_status_pub_, first_wall_violation_pub_, planning_metrics_pub_;
    ros::Time last_safety_status_stamp_;
    uint64_t planning_attempt_count_{0};
    uint64_t planning_success_count_{0};
    uint64_t planning_failure_count_{0};

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    bool callEmergencyEscape(const Eigen::Vector3d &start_pos,
                             const Eigen::Vector3d &free_pos);
    void publishSafetyStatus(const int map_occupancy,
                             const bool wall_guard_violation,
                             const LocalTrajData *info,
                             const double trajectory_time);
    void latchFirstWallViolation(const std::string &source,
                                 const int map_occupancy,
                                 const LocalTrajData *info,
                                 const double trajectory_time);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    // void planGlobalTrajbyGivenWps();
    void readGivenWpsAndPlan();
    Eigen::Vector3d assignedWaypointTarget(
        const Eigen::Vector3d &next_wp,
        const Eigen::Vector3d &previous_wp) const;
    bool refreshAssignedWaypointTarget();
    void planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp);

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void initialAssignmentCallback(
        const std_msgs::UInt32MultiArray::ConstPtr &msg);
    void manualFormationSplitCallback(const std_msgs::Bool::ConstPtr &msg);
    void dynamicTopologyCallback(const std_msgs::String::ConstPtr &msg);
    void formationTopologyCommandCallback(
        const FormationTopologyCommand::ConstPtr &msg);
    bool parseDynamicTopology(const std::string &text,
                              std::vector<int> *groups) const;
    std::string canonicalTopology(const std::vector<int> &groups) const;
    void publishTopologyAck();
    void logEffectiveTopology(const std::string &reason) const;
    int dynamicGroupForDrone(const int drone_id) const;
    bool dynamicTopologyMatchesLegacySplit() const;
    void splitFormationScaleCallback(const std_msgs::Float64::ConstPtr &msg);
    void splitProgressSyncScaleCallback(const std_msgs::Float64::ConstPtr &msg);
    void subgroupRouteIntentCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void publishFormationGroupState();
    void publishSubgroupRouteIntentState(const std::string &state);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
  };

} // namespace ego_planner

#endif

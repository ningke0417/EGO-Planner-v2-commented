#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &formation_start_pt,
        const Eigen::Vector3d &formation_end_pt, const bool flag_polyInit, 
        const bool flag_randomPolyTraj, const bool touch_goal);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
        const bool reset_local_trajectory = true);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool EmergencyEscape(const Eigen::Vector3d &start_pos,
                         const Eigen::Vector3d &free_pos);
    bool findNearestCollisionFreePosition(const Eigen::Vector3d &position,
                                           Eigen::Vector3d &free_position) const;
    bool findDirectionalCollisionFreePosition(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &preferred_direction,
        const double minimum_displacement,
        Eigen::Vector3d &free_position) const;
    bool checkCollision(int drone_id);
    bool setFormationAssignment(const std::vector<int> &assignment);
    void setManualFormationSplit(const bool enabled);
    void setDynamicFormationGroups(const std::vector<int> &groups);
    bool setDynamicFormationEdges(uint32_t edge_count, uint32_t edge_keep_mask,
                                  const std::vector<int> &assignment);
    void setSplitSubgroupFormationWeightScale(const double scale);
    void setSplitProgressSyncWeightScale(const double scale);
    void beginMergeSynchronization(const Eigen::Vector3d &position);
    void setSubgroupRouteIntent(const bool active,
                                const Eigen::Vector3d &origin,
                                const Eigen::Vector3d &direction,
                                const double gain = 1.0);
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt,
                             const bool touch_goal);
    bool violatesWallCollisionGuard(const Eigen::Vector3d &position) const;
    bool segmentViolatesWallCollisionGuard(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &end) const;
    inline double getWallGuardCheckHorizon(void) const { return wall_guard_check_horizon_; }
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};
    bool use_wall_collision_guard_{false};
    double wall_x_{0.0};
    double wall_y_{0.0};
    double wall_size_x_{0.0};
    double wall_size_y_{0.0};
    double wall_safety_distance_{0.0};
    double wall_guard_check_horizon_{3.0};

    bool isInsideWallGuard(const Eigen::Vector3d &position) const;

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif

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
    bool computeInitState(//计算初始状态/初始轨迹优化问题
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,//输入：起点位置速度加速度、局部目标位置速度、一些开关、时间参数 ts
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);//输出：initMJO（最小 jerk 优化器对象）。返回是否成功
    bool reboundReplan(//执行“反弹式重规划”（遇障碍后的局部重算）
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal);//输入起终状态和策略开关，返回是否成功
    bool planGlobalTrajWaypoints(//按多个航点规划全局轨迹,输入起点状态、航点、终点速度加速度
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(//从全局目标中截取局部目标
        const double planning_horizen,//输入规划视野、起点、全局终点；
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);//输出局部目标位置速度及是否触达终点标志
    bool EmergencyStop(Eigen::Vector3d stop_pos);//紧急停车接口，让轨迹快速收敛到 stop_pos
    bool checkCollision(int drone_id);//检查指定无人机是否发生碰撞风险
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);//将优化器 opt 的结果写入本地轨迹容器，并记录是否触达目标
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }//获取集群安全间距参数
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }//获取每段轨迹对应控制点数量
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;//规划参数对象
    GridMap::Ptr grid_map_;//栅格地图智能指针
    TrajContainer traj_;//轨迹容器，保存局部/全局/集群轨迹数据

  private:
    PlanningVisualization::Ptr visualization_;//可视化模块指针

    PolyTrajOptimizer::Ptr ploy_traj_opt_;//多项式轨迹优化器指针

    int continous_failures_count_{0};//连续失败计数器，使用 C++11 花括号初始化为 0

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;//定义别名 Ptr，表示 unique_ptr<EGOPlannerManager>

    // !SECTION
  };
} // namespace ego_planner

#endif
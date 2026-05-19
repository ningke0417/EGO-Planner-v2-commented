#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{
//定义类型别名 PtsChk_t：二维 vector，每个元素是 (double, Eigen::Vector3d) 对,常用于“待检查点”集合（时间/标量 + 三维点）
  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;//全局轨迹本体
    double global_start_time; // world time
    double duration;//轨迹总时长

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;//当前局部目标对应的“全局轨迹时间”
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;//上一次局部目标对应的“全局轨迹时间
  };

  struct LocalTrajData//局部轨迹数据结构
  {
    poly_traj::Trajectory traj;//局部轨迹
    PtsChk_t pts_chk;//用于碰撞/约束检查的点集
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;
    double des_clearance;//期望安全间距

  };

  typedef std::vector<LocalTrajData> SwarmTrajData;//定义集群轨迹容器：多个 LocalTrajData 的数组

  class TrajContainer//TrajContainer：统一保存全局、局部、集群轨迹数据
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time)//设置全局轨迹接口
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.drone_id = -1;
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const PtsChk_t &pts_to_chk, const double &world_time, const int drone_id = -1)//设置局部轨迹接口
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
      local_traj.pts_chk = pts_to_chk;
    }

  };

  struct PlanParameters//规划参数结构体
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double polyTraj_piece_length;  // distance between adjacient B-spline control points轨迹分段长度（注释说 B-spline 控制点间距）
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits可行性容忍度（速度/加速度超限允许比例）
    double planning_horizen_;//规划视野
    bool use_multitopology_trajs;//是否启用多拓扑轨迹
    bool touch_goal;//是否触达目标
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif
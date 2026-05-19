#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"//引入 L-BFGS 优化器
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"

namespace ego_planner
{

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points约束点数量
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // 每个约束点对应的“障碍物参考点”或碰撞点，The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // 每个约束点应该往哪个方向推开，Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
//重新设置约束点数量
    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);//把 points 矩阵调整为3 行 × size_set 列，每一列表示一个三维点
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }
//从当前约束点集合里截取一段 [start, end]，放到 buf 里
    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)//点矩阵不是 3 行，说明不是三维点数据
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);//从当前 points 矩阵中截取一块数据，赋值给 buf.points，points.block(起始行, 起始列, 行数, 列数)，也就是截取第 start 到第 end 个三维点
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];//把原来的第 i 个约束点的 base_point 拷贝到 buf 中
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }//如果到终点 → 返回最后一个点，否则 → 返回“靠后 2/3 位置的点”

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free，指向“其他无人机轨迹列表”的裸指针
    ConstraintPoints cps_;//保存当前这次优化使用的约束点集合。里面包括轨迹上的约束点、障碍参考点、避障方向等
    // PtsChk_t pts_check_;

    int drone_id_;
    int cps_num_prePiece_;   // 每一段轨迹里要设置多少个代表性的约束点，number of distinctive constraint points each piece
    int variable_num_;       // 优化变量总数optimization variables
    int piece_num_;          // 轨迹被分成多少段多项式poly traj piece numbers
    int iter_num_;           // 当前求解器迭代了多少次iteration of the solver
    std::vector<double> min_ellip_dist2_; // 记录和其他无人机轨迹之间的最小“椭球距离平方”min trajectory distance in swarm
    bool touch_goal_;//标记这次局部轨迹是否已经真正触达最终目标
    struct MultitopologyData_t
    {
      bool use_multitopology_trajs{false}; //是否启用多拓扑轨迹搜索
      bool initial_obstacles_avoided{false}; //初始轨迹是否已经绕开了初始障碍
    }multitopology_data_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,//不要停止，优化正常继续
      STOP_FOR_REBOUND,//因为“rebound/反弹式处理”需要中止
      STOP_FOR_ERROR//因为错误而中止
    } force_stop_type_;//这个变量保存当前的停止状态，供优化迭代回调函数检查

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;                               // wei_obs_：正常障碍避让代价的权重，wei_obs_soft_：软障碍代价的权重obstacle weight
    double wei_swarm_, wei_swarm_mod_;                            // wei_swarm_：基础的群体避让权重，wei_swarm_mod_：运行时动态调整过的群体避让权重swarm weight
    double wei_feas_;                                             // 可行性代价权重，可行性主要指速度、加速度、jerk 等是否超限feasibility weight
    double wei_sqrvar_;                                           // 平方方差项的权重，一般是为了让轨迹点分布更均匀、轨迹形状更稳定squared variance weight
    double wei_time_;                                             // 时间代价权重，用来控制优化器对“轨迹总时间”的偏好time weight
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_; // obs_clearance_：和障碍物之间希望保持的安全距离，obs_clearance_soft_：软约束版本的障碍安全距离，swarm_clearance_：和其他无人机之间希望保持的安全距离，safe distance
    double max_vel_, max_acc_, max_jer_;                          // 动力学限制，dynamic limits

    double t_now_;//当前时刻

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,//表示没有障碍冲突，轨迹或点集当前是安全的
      ERR,//表示过程出错了
      FINISH//表示检查或处理已经完成，但不一定等价于“完全无障碍”
    };

    /* set variables 给优化器喂输入*/
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);//把其他无人机的轨迹数据传进来
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setConstraintPoints(ConstraintPoints cps);//直接把已经构造好的约束点集合传进去
    void setUseMultitopologyTrajs(bool use_multitopology_trajs);//设置是否启用“多拓扑轨迹”

    /* helper functions拿内部状态 */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }//返回当前保存的约束点集合
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }//返回内部的最小 jerk 轨迹对象
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }//返回“每段轨迹用多少个约束点”
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }//返回多机避碰的安全距离阈值

    /* main planning API */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);//根据一条轨迹生成“后续要重点检查的点”

    std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);//对给定点集做更细致的碰撞检查，只返回“有问题的区间”

    /* check collision and set {p,v} pairs to constraint points */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const bool flag_first_init /*= true*/);//细致检查当前轨迹哪些地方碰障碍，根据碰撞结果生成/设置约束点、障碍参考点和方向

    bool roughlyCheckConstraintPoints(void);//粗略检查约束点

    bool allowRebound(void);//如果轨迹还撞障碍，是否允许通过重新设置约束/中止当前优化再来一轮的方式继续调整。

    /* multi-topo support 根据碰撞区间 segments，构造多组“拓扑不同”的约束点方案，返回值是多个 ConstraintPoints，每个元素代表一种候选绕障方案*/
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);
  private:
    /* callbacks by the L-BFGS optimizer下面两个函数是给 lbfgs.hpp 这个数值优化器回调用的。 */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);//给定当前轨迹参数，算出总代价和对应梯度

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);//在优化过程中，如果发现需要中断，就让 L-BFGS 提前退出

    /* mappings between real world time and unconstrained virtual time 下面几个模板函数负责在“真实时间”和“虚拟时间变量”之间转换*/
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);//把真实时间 RT 映射到虚拟时间 VT

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);//把优化变量里的虚拟时间 VT 还原成真实时间 RT

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);//把“代价对真实时间 RT 的梯度”转换成“代价对虚拟时间 VT 的梯度”

    /* gradient and cost evaluation functions 下面这些函数是在计算各种代价项以及梯度*/
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);//初始化轨迹，并计算平滑项的梯度和代价

    template <typename EIGENVEC>
    void addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);//累加位置/速度/加速度/jerk 相关约束项的梯度和代价

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);//计算单个点 p 的障碍物代价和对位置的梯度

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,//对当前时间变量的梯度
                        double &grad_prev_t,//对前一个时间变量的梯度
                        double &costp);//计算单个点在某个时刻与其他无人机之间的避碰代价和梯度

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);//计算速度可行性代价和梯度

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);//计算加速度可行性代价和梯度

    bool feasibilityGradCostJ(const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);//计算 jerk 可行性代价和梯度

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);//计算一组点之间距离平方的方差，以及对应梯度，这个函数一般用于让点分布更均匀

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);//计算线段长度方差及梯度，针对“相邻段长度是否均匀”

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
// #include <fstream>局部/全局轨迹组织和碰撞检查
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}//构造函数，当前是空实现

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }//析构函数，对象销毁时打印 "des manager"，便于调试生命周期。

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);//可行性容忍度
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);//多项式轨迹分段长度
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);//规划视野/时间窗
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);//是否启用多拓扑轨迹
    nh.param("manager/drone_id", pp_.drone_id, -1);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);//创建多项式轨迹优化器对象
    ploy_traj_opt_->setParam(nh);//给优化器设置参数
    ploy_traj_opt_->setEnvironment(grid_map_);//给优化器注入环境（地图）对象，供碰撞检查/约束使用

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);//把群体轨迹数据地址传给优化器，用于多机协同/避让
    ploy_traj_opt_->setDroneId(pp_.drone_id);//设置优化器所属无人机 ID（区分集群中的个体）
  }
//从当前状态出发，生成一条初始轨迹，再对轨迹做优化，最后把成功的结果存起来并可视化
  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;//统计初始化阶段耗时。统计优化阶段耗时。

    static int count = 0;//用来记录第几次 replan
    cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;
    // if ((start_pt - local_target_pt).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    /*** STEP 1:初始化初始轨迹 INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);//告诉优化器这次是否触达终点，因为如果这条局部轨迹已经直达最终目标，优化器在检查范围、约束处理上会更严格一些
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;//用“每段轨迹长度 / 最大速度”估算每段时间长度 ts

    poly_traj::MinJerkOpt initMJO;//创建初始最小 jerk 优化对象 initMJO，这里先用它保存“初始轨迹”
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;//调用 computeInitState 生成初始可行轨迹；失败直接返回 false
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());//从初始轨迹提取约束点
    vector<std::pair<int, int>> segments;//用来保存碰撞段区间。
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;//对初始轨迹做精细碰撞检查，同时根据碰撞结果，为约束点生成 base_point 和 direction，true 表示这是第一次初始化检查，如果这里出错，说明连约束构造都做不出来，直接失败
    }

    t_init = ros::Time::now() - t_start;//记录初始化耗时

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));//把约束点矩阵按列转成点数组。
    visualization_->displayInitPathList(point_set, 0.2, 0);//用于在可视化中显示“初始路径/初始约束点”

    t_start = ros::Time::now();

    /*** STEP 2:轨迹优化 OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;//如果是多拓扑模式，用来保存多条候选轨迹做可视化。
    poly_traj::MinJerkOpt best_MJO;//最终选中的最优轨迹。

    // ROS_ERROR("BBBB");

    if (pp_.use_multitopology_trajs)//如果启用了“多拓扑轨迹”，进入多候选路径优化分支
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);//根据碰撞段 segments 生成多组“拓扑不同”的候选约束点
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());//记录每条候选是否优化成功。
      poly_traj::Trajectory initTraj = initMJO.getTraj();//从初始轨迹里取出完整轨迹对象
      int PN = initTraj.getPieceNum();//轨迹段数
      Eigen::MatrixXd all_pos = initTraj.getPositions();//所有连接点位置。
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);//取中间点，作为优化变量初值。
      Eigen::Matrix<double, 3, 3> headState, tailState;//构造轨迹首端和末端状态矩阵
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;//当前候选优化后的总代价，目前找到的最小总代价

      for (int i = trajs.size() - 1; i >= 0; i--)//遍历所有拓扑候选
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);//把当前候选约束点送进优化器
        ploy_traj_opt_->setUseMultitopologyTrajs(true);//告诉优化器当前运行在多拓扑模式下
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))//调用真正的轨迹优化
        {
          success[i] = true;//这条候选轨迹优化成功

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();//保存当前最优的轨迹对象
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);//把当前候选优化结果的约束点提出来，保存到 vis_trajs，后面统一显示多条候选路径
        }
      }

      t_opt = ros::Time::now() - t_start;//多拓扑优化阶段耗时结束

      if (trajs.size() > 1)//如果候选不止一条，就打印总候选数，成功数，失败数
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); //把多条候选结果轨迹显示出来 This visuallization will take up several milliseconds.
    }
    else//如果没启用多拓扑，就只做一条轨迹优化
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();//从初始轨迹里取出完整轨迹对象
      int PN = initTraj.getPieceNum();//轨迹段数
      Eigen::MatrixXd all_pos = initTraj.getPositions();//所有连接点位置。
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);//取中间点，作为优化变量初值。
      Eigen::Matrix<double, 3, 3> headState, tailState;//构造轨迹首端和末端状态矩阵
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();//无论成功与否，都把优化器里最后一条轨迹结果取出来

      t_opt = ros::Time::now() - t_start;
    }

    /*** STEP 3:保存与显示结果 Store and display results ***/
    cout << "Success=" << (flag_success ? "yes" : "no") << endl;
    if (flag_success)//成功分支
    {
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;//累加这次总耗时和成功次数
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",//本次总耗时，初始化耗时，优化耗时，历史平均成功耗时
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_time / count_success << endl;

      setLocalTrajFromOpt(best_MJO, touch_goal);//把 best_MJO 里的轨迹结果写入 traj_.local_traj，这一步相当于正式接受这条轨迹，作为当前局部执行轨迹
      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());//提取最终最优轨迹的约束点
      visualization_->displayOptimalList(cstr_pts, 0);//用于显示最终优化成功的轨迹结果

      continous_failures_count_ = 0;//连续失败计数清零，说明这次规划成功了，后面初始化时不需要再加大随机扰动
    }
    else//失败分支
    {
      cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());//把“最后一次优化得到的约束点”拿出来
      visualization_->displayFailedList(cstr_pts, 0);//显示成失败轨迹，便于调试

      continous_failures_count_++;//连续失败次数加 1
    }

    return flag_success;
  }
//给后面的轨迹优化器准备一条“初始轨迹”作为初值
  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;//这个函数是不是第一次被调用
//满足“这是第一次调用”或“外部明确要求强制多项式初始化”
    if (flag_first_call || flag_polyInit) /*** case 1: 从起点到局部目标直接重新造一条初始多项式轨迹，适合第一次规划或强制重启polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;//定义每一段轨迹的持续时间向量
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;//定义“初始化用的初始化轨迹”的总时长固定为 2 秒
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)//如果不启用随机轨迹初始化，就走最简单的直连方式
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;//先用一个最简单的起点到终点的单段最小 jerk 曲线
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;//把时间向量长度设为 1，这一段持续 2 秒
      }
      else//如果启用了随机初始化，就走带随机中间点的分支。
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();//得到一个与“起终点连线”和 z 轴都垂直的方向，相当于“左右偏移方向”
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();//再构造一个“垂直偏移方向”
        innerPs.resize(3, 1);//随机初始化时只插入 1 个中间点，所以矩阵是 3x1
        innerPs = (start_pt + local_target_pt) / 2 +//中间点先放在起点和目标点的中点附近，再加两个方向的随机扰动
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;//因为有一个中间点，所以整条轨迹被分成 2 段
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);//两段平均分配时间，每段 1 秒
      }

      /* 先生成一条“粗初始轨迹generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* 再把粗轨迹细化成真正用于优化的初始轨迹generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;//至少保证 2 段，避免段数太少
      double piece_dur = init_of_init_totaldur / (double)piece_nums;//把前面那条“2 秒粗轨迹”平均切成 piece_nums 份，每份对应一个采样时间间隔
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);//真实初始化轨迹的每一段时间都统一设成 ts
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;//t_s：从第一段结束处开始采样，t_e：到接近轨迹尾部前停止采样
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2:沿着“当前局部轨迹剩余部分 + 全局轨迹”拼一条连续初始轨迹，适合正常连续重规划 initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;//计算当前局部轨迹已经执行了多久
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;//计算从“现在”到当前局部轨迹结束还剩多久
      if (t_to_lc_end < 0)//如果已经超过当前局部轨迹终点时间了，说明这次调用时机不对，直接返回失败，等下一次调用。
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);//从“现在”到新的局部目标在时间轴上大概还需要多久
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);//把总剩余时间 t_to_lc_tgt 平均分给每一段
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);//从第一段时间末尾开始采样中间点
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)//如果这个中间点对应的时间还落在“当前局部轨迹剩余部分”里，就直接从当前局部轨迹上取点
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)//如果这个中间点时间已经超过当前局部轨迹剩余时间，但还没超过总目标时间，那就转而从全局轨迹上取点
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);//时间往后推进一段，继续取下一个中间点
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;//在准备选新局部目标之前，先把旧局部目标时间存起来

    double t_step = planning_horizen / 20 / pp_.max_vel_;//设置沿全局轨迹扫描时的时间步长
    // double dist_min = 9999, dist_min_t = 0.0;
    //开始沿全局轨迹向前找“离当前起点约 planning_horizen 远”的点
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);//取全局轨迹上时间 t 对应的位置点
      double dist = (pos_t - start_pt).norm();//当前候选点 pos_t到局部规划起点 start_pt的欧氏距离

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }
    //如果循环一直找到全局轨迹末尾，说明本次局部目标应该直接设为最终目标
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }
    //局部目标离最终终点是不是已经足够近，以至于应该开始减速到 0
    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else//如果离最终目标还不够近，就直接沿全局轨迹读取该点的目标速度
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }
  //从优化结果轨迹里提取出真正需要执行的轨迹和碰撞检查点，然后把它们写入当前局部轨迹容器
  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());//从这条轨迹里提取约束点矩阵 cps
    PtsChk_t pts_to_check;//每个约束点对应的一组轨迹采样点
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    }

    return ret;
  }
  //生成一条紧急停车轨迹，让无人机尽快停在 stop_pos 附近
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);//一旦调用 EmergencyStop()，系统当前执行轨迹就会被替换成这条停住的轨迹

    return true;
  }
//检查“我当前的局部轨迹”和编号为 drone_id 的另一架无人机轨迹，在时间重叠区间内是否会过于接近
  bool EGOPlannerManager::checkCollision(int drone_id)//drone_id：要检查的另一架无人机编号
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;
    if (traj_.swarm_traj[drone_id].drone_id != drone_id) // The trajectory is invalid
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance) )
      {
        return true;//这两个轨迹在时刻 t 上过于接近，有碰撞风险，返回true
      }
    }

    return false;
  }
//根据起点状态和一串航点，用最小jerk方法生成一条穿过这些航点的全局轨迹，并用简单的时间缩放策略尽量让轨迹最大速度不超限。
  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;//位置取 waypoints 最后一个航点，也就是全局目标位置
    Eigen::MatrixXd innerPts;
//如果航点数大于 1，说明：除了最后一个终点外，前面的航点都应该作为中间点参与轨迹生成
    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;//期望巡航速度
    Eigen::VectorXd time_vec(waypoints.size());//定义每段时间向量time_vec，长度等于轨迹段数

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;//第 i 段航程对应的粗略时间分配
      }

      globalMJO.generate(innerPts, time_vec);
      //检查轨迹最大速度是否超限
      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)//j 只可能取 0 或 1。if (j == 2) 这一分支实际上永远不会触发
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;//如果当前轨迹最大速度还不满足要求，就把期望速度再缩小
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner

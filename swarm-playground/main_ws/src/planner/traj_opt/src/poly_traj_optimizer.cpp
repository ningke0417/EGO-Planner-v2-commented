#include "optimizer/poly_traj_optimizer.h"

using namespace std;

#define VERBOSE_OUTPUT false//默认不打印详细调试信息
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)//定义一个条件打印宏。只有 VERBOSE_OUTPUT 为 true 时，才会执行 printf

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,//初始状态，终止状态，包含位置，速度，加速度
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,//初始中间点，初始每段持续时间
      double &final_cost)//输出参数，用来返回最终优化出来的总代价
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");//如果列数不等于时间段数减 1，说明输入结构不合法
      return false;
    }

    // Preparision 1: Some mise params
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;//restart_nums：因为“结果还不安全”而重新优化的次数，rebound_times：因为“rebound/反弹处理”而被迫中断后重来的次数
    bool flag_force_return, flag_still_unsafe, flag_success, flag_swarm_too_close;
    multitopology_data_.initial_obstacles_avoided = false;//表示一开始先假设初始障碍还没被避开
    wei_swarm_mod_ = wei_swarm_;//把动态群体避碰权重 wei_swarm_mod_ 重置为基础值 wei_swarm_

    // Preparision 2: Trajectory related params
    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();
    jerkOpt_.reset(iniState, finState, piece_num_);
    variable_num_ = 4 * (piece_num_ - 1) + 1;//计算优化变量总数
    double x_init[variable_num_];
    memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));//把初始中间点的原始数据拷贝进 x_init 的前半部分
    Eigen::Map<Eigen::VectorXd> Vt(x_init + initInnerPts.size(), initT.size());//把 x_init 后半段“映射”为一个 Eigen 向量 Vt
    RealT2VirtualT(initT, Vt);
    min_ellip_dist2_.resize(swarm_trajs_->size());//调整min_ellip_dist2_（当前轨迹和每架其他无人机之间的最小椭球距离平方）的大小，使其和群体轨迹数量一致

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);//先加载一套默认参数，作为基础配置
    lbfgs_params.mem_size = 16;//设置 L-BFGS 的历史记忆长度为 16
    lbfgs_params.max_iterations = 200;//设置每轮优化最多迭代 200 次
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;//用于基于过去若干轮的目标值变化来判断是否收敛
    lbfgs_params.delta = 1.0e-2;//设置收敛判据的阈值
    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_unsafe = false;
      flag_success = false;
      flag_swarm_too_close = false;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();//记录本轮 L-BFGS 结束时间
      double time_ms = (t2 - t1).toSec() * 1000;//这一轮 L-BFGS 本身耗时
      double total_time_ms = (t2 - t0).toSec() * 1000;//从最开始到现在总共耗时

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||//正常收敛
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||//达到最大迭代
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||//一开始就已经最优
          result == lbfgs::LBFGS_STOP)//满足停止条件
      {
        flag_force_return = false;

        /* double check: fine collision check */
        std::vector<std::pair<int, int>> segments_nouse;
        for (size_t i = 0; i < swarm_trajs_->size(); ++i)//检查当前优化结果是否离别的无人机太近，|=表示“只要有一架太近，就把flag_swarm_too_close设为真”。
        {
          flag_swarm_too_close |= min_ellip_dist2_[i] < pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 1.25, 2);
        }
        if (!flag_swarm_too_close)//如果群体距离没有问题，再去检查障碍物碰撞
        {
          if (finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)//调用精细碰撞检查
          {
            flag_success = true;//打印迭代次数，单轮耗时，总耗时，最终代价
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          }
          else
          {
            // A not-blank return value means collision to obstales
            flag_still_unsafe = true;
            restart_nums++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
          }
        }
        else//如果不是障碍问题，而是与其他无人机太近，则进入这里
        {
          PRINTF_COND("Swarm clearance not satisfied, keep optimizing. iter=%d,time(ms)=%5.3f, wei_swarm_mod_=%f\n", iter_num_, time_ms, wei_swarm_mod_);
          flag_still_unsafe = true;
          restart_nums++;
          wei_swarm_mod_ *= 2;//把群体避碰权重翻倍，意思是：下一轮优化时，更加强调“远离其他无人机”
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)//如果 L-BFGS 返回“被取消”，进入这里
      {
        flag_force_return = true;
        rebound_times++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }
    //这是整个 do...while 的继续条件，只要满足以下任一情况，就继续新一轮优化
    } while ((flag_still_unsafe && restart_nums < 3) ||//结果还不安全，并且重启次数还没超过 3 次
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));//这轮被 rebound 强制中断，并且反弹次数还没超过 20 次

    return flag_success;//至少有一轮真正通过了精细碰撞检查才会是 true
  }
  //沿着一条轨迹按时间采样，生成一组“待检查点” pts_check，供后面做碰撞检查或安全验证
  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    pts_check.resize(id_cps_end);
    const double RES = grid_map_->getResolution(), RES_2 = RES / 2;
    Eigen::VectorXd durations = traj.getDurations();
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);//构造“每一段轨迹的起始时间表”,t_seg_start(0)=0,t_seg_start(1)=d0,t_seg_start(2)=d0+d1...
    const double DURATION = durations.sum();//整条轨迹总时长
    double t = 0.0, t_step = min(RES / max_vel_, durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
    Eigen::Vector3d pt_last = traj.getPos(0.0);
    // pts_check[0].push_back(pt_last);
    int id_cps_curr = 0, id_piece_curr = 0;//id_cps_curr：当前属于第几个约束点组，id_piece_curr：当前时间 t 落在第几段轨迹里

    while (true)
    {
      if (t > DURATION)
      {
        if (touch_goal_ && pts_check.size() > 0)//如果当前轨迹是“到最终目标”的那种轨迹，并且 pts_check 非空，就尝试做一个宽松收尾
        {
          while (pts_check.back().size() == 0)
          {
            pts_check.pop_back();//把末尾那些空组删掉
          }

          if (pts_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }
          else
          {
            return true;//如果删完之后整个 pts_check 都空了，就失败。否则视为成功
          }
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)pts_check.size());
          pts_check.clear();
          return false;
        }
      }
      //当前这段轨迹里，下一个约束点边界对应的时间
      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_prePiece_ * ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
        {
          ++id_piece_curr;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.getPos(t);//取当前时间 t 对应的轨迹位置
      //满足以下任一条件，就把当前点存进去：
      //t < 1e-5，一开始的起点必须存
      //pts_check[id_cps_curr].size() == 0，当前约束点组里还一个点都没有，至少先放一个
      //(pt - pt_last).cwiseAbs().maxCoeff() > RES_2，当前点和上次已存点在某个坐标轴上的差值超过半个栅格，说明这个点足够“新”
      if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }

    return true;//如果正常跳出循环，说明成功生成了所有需要的检查点
  }

  /* check collision and set {p,v} pairs to constrain points */
  //先找轨迹哪些区域撞障碍，再借助 A* 给这些碰撞区域的约束点生成避障参考点和推开方向
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &pt_data,
      const bool flag_first_init /*= true*/)
  {
  //先把轨迹和初始约束点取出来
    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece_);
    poly_traj::Trajectory traj = pt_data.getTraj();
  //如果是第一次初始化，就把 cps_ 重置
    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** 开始按障碍把轨迹分段，Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;//保存碰撞段，格式是 (起点索引, 终点索引)
    constexpr int ENOUGH_INTERVAL = 2;//要求状态连续稳定几次后才确认切换，避免抖动
    int in_id = -1, out_id = -1;//记录某段碰撞区间的开始和结束位置
    int same_occ_state_times = ENOUGH_INTERVAL + 1;//连续多少次占据状态没变
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_); // only check closed 2/3 points.

    PtsChk_t pts_check;//每个约束点对应一组轨迹采样点
    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      return CHK_RET::ERR;
    }
    //遍历每个约束点对应的采样点，找碰撞段
    for (int i = 0; i < i_end; ++i)//外层 i：第几个约束点组
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)//内层 j：这一组里的第几个采样点
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);//查该点是否落在膨胀障碍里
        
        if (occ && !last_occ)//从自由区进入障碍区
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)//如果前面已经稳定自由了足够久，或者这是第一个点，就认定这里是碰撞段起点
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)//从障碍区离开到自由区
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;//如果没有发生占据状态切换，就把“连续同状态次数”加 1
        }
        //如果前面已经“可能离开障碍”，并且后面又连续稳定自由了几次，或者已经到最后一个点，那就正式认定碰撞段结束
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;
        //如果这时起点和终点都找到了，就形成一段完整碰撞区间
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /* 如果没有任何碰撞段，提前返回，Collision free and return in advance */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    /*** 对每个碰撞段跑 A*，a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());//如果 A* 成功，就把路径保存下来
      }
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) //如果这段 A* 搜索失败，但后面还有下一段，尝试把当前段和下一段合并，再重新处理 connect the next segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;//扩大当前段终点
        segment_ids.erase(segment_ids.begin() + i + 1);//删掉下一段
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");//其他情况都视为 A* 失败，直接返回 ERR
        return CHK_RET::ERR;
      }
    }

    /*** 给每段计算允许扩张的上下边界calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment第一段
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // 最后一段last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else//中间段
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length适当拉长碰撞段 ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;//左边尽量往前扩，但不能越过 bounds[i].first

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;//右边尽量往后扩，但不能越过 bounds[i].second
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;//如果本来就够长，就直接保持原段不变
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) //防止调整后的段互相重叠 Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment 对每段真正生成 base_point 和 direction***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1先把这一段的临时标志清空
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2为段内点寻找与 A* 路径的交点
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;//如果交点和当前约束点几乎重合，说明方向信息无效，作废
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)//如果碰撞段太短，只包含两个约束点，前面用中间点找方向可能不稳，所以单独处理
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3把方向传播给整段其他没成功的点
      if (got_intersection_id >= 0)//如果这一段至少有一个点成功找到方向，就把它扩散给整段
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)//从交点往右传播，哪些点没成功生成方向，就继承左边邻点的base_point/direction
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)//再从交点往左传播，哪些点没成功生成方向，就继承右边邻点的信息
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;//把最终有效的碰撞段交给外部
    return CHK_RET::FINISH;//返回 FINISH，表示：已经发现了碰撞，也完成了约束点信息设置，后续可以继续优化了
  }
//在优化迭代中快速查看当前约束点是否又撞上了“新的障碍”；如果撞上了，就用A*生成新的避障参考点和方向，并强制当前优化中断，重新开始下一轮rebound 优化
  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    // int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;//保存所有检测到的碰撞段，每段是 (in_id, out_id)
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // 可能只是靠近之前已经处理过的障碍，而不是新的有效碰撞current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;
      //向左找碰撞段起点
        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }
        //向右找碰撞段终点
        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));//把找到的碰撞段存起来
      }
    }

    if (flag_new_obs_valid)//对每个碰撞段跑 A*
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());//A* 成功就把路径存起来
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // 避免碰撞段重叠，Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** 为每段生成 base_point 和 directionAssign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1先清空这一段的临时标记
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2尝试在段内找到至少一个交点，生成真实方向
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));//记录这个点的障碍参考点
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());//记录对应的推开方向
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3把成功点的方向扩散到整段
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)//从成功点往右传播
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)//再从成功点往左传播
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;//当前约束点没有检测到新的有效碰撞，不需要 rebound，当前 L-BFGS 可以继续正常优化
  }

  bool PolyTrajOptimizer::allowRebound(void) //zxzxzx判断当前这轮优化，是否允许触发“rebound 机制”
  {
    // criterion 1迭代次数太少时，不允许 rebound，刚开始的前几次迭代，轨迹通常还很不稳定，很多“碰撞”可能只是暂时现象
    if (iter_num_ < 3)
      return false;

    // criterion 2轨迹弯得太厉害时，不允许 rebound
    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.87) // 30 degree
      return false;//如果轨迹某处前后两段夹角超过约 30 度，说明轨迹现在弯得比较厉害，这时不允许 rebound

    // criterion 3如果启用了多拓扑轨迹，必须先确认初始障碍已经绕开
    if (multitopology_data_.use_multitopology_trajs)
    {
      if (!multitopology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < cps_.points.cols() - 1; ++i)
        {
          if (cps_.base_point[i].size() > 0)//如果这个约束点之前确实记录过障碍参考点 base_point，才检查它
          {
            //只检查“初始障碍”有没有真正绕开。 Only adopts "0" since finelyCheckAndSetConstraintPoints() after one optimization can add more base_points.
            if ((cps_.points.col(i) - cps_.base_point[i][0]).dot(cps_.direction[i][0]) < 0)//当前约束点相对于原始障碍参考点，是不是已经沿着正确方向推到安全一侧了
            {
              avoided = false;
              break;
            }
          }
        }

        multitopology_data_.initial_obstacles_avoided = avoided;
      }

      if (!multitopology_data_.initial_obstacles_avoided)
      {
        return false;
      }
    }

    // all the criterion passed
    return true;
  }

  /* multi-topo support多拓扑支持，给定若干个碰撞段 segments，在每个碰撞段上构造两种绕障版本，然后把这些段按组合方式拼起来，生成多条“拓扑不同”的候选约束点序列 */
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;//没有任何碰撞，直接把约束点返回
    }

    constexpr int MAX_TRAJS = 8;//最多生成 8 条候选轨迹
    constexpr int VARIS = 2;//每个碰撞段只有 2 种变化方式，保持原绕障方向，改成反方向绕障
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));//最多允许前多少个碰撞段参与组合
    std::vector<ConstraintPoints> control_pts_buf;// 用来存最终生成的候选约束点序列
    control_pts_buf.reserve(MAX_TRAJS);//提前分配容量，减少内存重分配
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);//粗略估计相邻控制点之间的平均距离

    // Step 1.为每个碰撞段准备两套信息 Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;//first：原始版本，second：反向版本
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);//这一轮做完后，每个碰撞段都有两个副本，但两者暂时完全相同
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 找每个碰撞段内真正进入障碍和离开障碍的位置Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)//从前往后找第一次撞障碍的位置
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)//从后往前找最后一个撞障碍的位置
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check双重检查：如果根本没找到起止，就丢弃这段
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 为这一段构造“反方向版本”Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;//反方向的避障方向，反方向对应的障碍参考点
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];//取负号，得到“反方向绕障”的方向

          //首尾点特殊处理 The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (grid_map_->getInflateOccupancy(base_pt_reverse)) //如果这个反向参考点还在障碍里，就继续往外搜 Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold“阈值”.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) //如果反向点本来就在自由区，而且离约束点足够远，直接用 Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else//如果反向点离约束点太近，也放弃这段
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            if (VERBOSE_OUTPUT)
              cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 把段外部分也填上参考方向Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)//对于段前面那些不在真实碰撞区里的点，直接继承 occ_start_id 的 base_point 和 direction
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)//对段后面那些点，继承 occ_end_id 的 base_point 和 direction
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else//单点段的特殊处理
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!, 2");
          if (VERBOSE_OUTPUT)
            cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. 把每一段的“原版本/反版本”组合成完整候选轨迹Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1生成当前这一轮的选择表 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2按选择表拼装整条约束点序列 Assign params according to the selection table.
      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)//如果当前点不在某个碰撞段里，就保持原样
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)//如果当前点在某个碰撞段里，就按选择表取版本
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)//如果当前已经到这个碰撞段的末尾
          {
            cp_of_seg_id = 0;//段内索引重置为 0
            seg_id++;//就准备切到下一个碰撞段
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  /* callbacks by the L-BFGS optimizer给 L-BFGS 一个当前变量 x，它负责算出：当前这条轨迹的总代价是多少，这个总代价对每个优化变量的梯度是多少 */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    fill(opt->min_ellip_dist2_.begin(), opt->min_ellip_dist2_.end(), std::numeric_limits<double>::max());

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;//平滑项代价，时间项代价
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4);//障碍物代价，群体避碰代价，可行性代价，方差/均匀性代价

    opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAJGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;//返回总代价
  }
  //在优化迭代过程中，检查当前是否应该立刻中断
  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    //只要内部状态表明“出错了”或者“需要 rebound 了”，就中断当前优化
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* 真实时间和虚拟时间的映射mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }
  //把虚拟时间 VT 映射回真实时间 RT。
  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }
  //把“对真实时间 RT 的梯度”转换成“对虚拟时间 VT 的梯度”，并顺便计算时间代价
  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;//如果虚拟时间在正半轴，真实时间对虚拟时间的导数就是 VT + 1
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;//把真实时间梯度变成虚拟时间梯度
    }

    costT = RT.sum() * wei_time_;
  }

  /* 计算轨迹平滑项（最小 jerk）的代价和梯度gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }
  //沿着当前轨迹逐段、逐采样点地计算各种代价和梯度，并把它们累加到优化变量上
  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    //
    int N = gdT.size();//轨迹段数
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double costp, costv, costa, costj;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;//位置,速度,...代价对系数矩阵 C 的梯度
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;//位置项,速度项...对时间的梯度贡献
    double omg;
    int i_dp = 0;
    costs.setZero();
    // Eigen::MatrixXd constraint_pts(3, N * K + 1);

    // printf("A\n");

    //按段、按采样点计算障碍/群体/可行性代价, int innerLoop;
    double t = 0;
    for (int i = 0; i < N; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;//位置基函数[1, t, t², t³, t⁴, t⁵]^T
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;//速度基函数
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;//如果当前是段首或段尾采样点，积分权重设为 0.5，中间采样点设为 1.0

        cps_.points.col(i_dp) = pos;

        //障碍物代价 collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        //群体避碰代价 swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        //可行性代价：速度 feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }
        //可行性代价：加速度
        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(2) += omg * step * costa;
        }
        //可行性代价：jerk
        if (feasibilityGradCostJ(jer, gradj, costj))
        {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * sna;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costj / K + step * gradViolaJt);
          costs(2) += omg * step * costj;
        }

        // printf("L\n");

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }

    //计算方差代价 quratic variance
    Eigen::MatrixXd gdp;
    double var;
    // lengthVarianceWithGradCost2p(cps_.points, K, gdp, var);
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);
    //再把方差项梯度传播回系数和时间
    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(3) += var;//把方差代价加到 costs(3)
  }
  //当前采样点和障碍物之间的代价与梯度，检查当前点离障碍够不够远，如果不够远，就根据base_point + direction 给出一个把点推离障碍的代价和梯度
  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);//从障碍参考点 base_point指向当前点 p的向量
      double dist = ray.dot(cps_.direction[i_dp][j]);//当前点沿着避障方向的投影距离
      double dist_err = obs_clearance_ - dist;
      double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

      if (dist_err > 0)//硬障碍代价
      {
        ret = true;
        costp += wei_obs_ * pow(dist_err, 3);
        gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)//软障碍代价
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += wei_obs_soft_ * rsqr * (term - 1.0);
        gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }
  //预测当前时刻其他无人机的位置，计算当前点和它们的椭球安全距离，如果太近，就施加群体避碰惩罚
  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();//位置梯度清零
    gradt = 0;//当前段时间梯度清零
    grad_prev_t = 0;//前面段时间梯度清零
    costp = 0;//代价清零

    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      double pt_time = (t_now_ - traj_i_satrt_time) + t; // never assign a high-precision golbal time to a double directly!
      const double CLEARANCE = (swarm_clearance_ + swarm_trajs_->at(id).des_clearance) * 1.5; // 定义这对无人机之间的安全距离1.5 is to compensate slight constraint violation
      const double CLEARANCE2 = CLEARANCE * CLEARANCE;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time);
      }
      else
      {
        double exceed_time = pt_time - swarm_trajs_->at(id).duration;
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)//如果离得太近，就施加 swarm 惩罚
      {
        ret = true;

        costp += wei_swarm_mod_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_mod_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_[id] > ellip_dist2)
      {
        min_ellip_dist2_[id] = ellip_dist2;
      }
    }

    return ret;
  }
//根据当前速度 v，判断是否超过最大速度 max_vel_；如果超过，就计算速度超限代价和对应梯度
  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)//速度真的超限
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }
//根据当前加速度 a，判断是否超过最大加速度 max_acc_；如果超过，就计算超限代价和梯度
  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }
//判断当前 jerk 向量 j 是否超过最大 jerk 限制 max_jer_；如果超限，就给出惩罚代价和梯度
  bool PolyTrajOptimizer::feasibilityGradCostJ(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    double jpen = j.squaredNorm() - max_jer_ * max_jer_;
    if (jpen > 0)
    {
      gradj = wei_feas_ * 6 * jpen * jpen * j;
      costj = wei_feas_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }
//根据一串点 ps，计算相邻点之间距离平方的“均匀性代价”，并求出代价对每个点坐标的梯度
  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);//最终代价
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)//当前点如果不是第一个点，会受到左边线段的正向贡献
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)//当前点如果不是最后一个点，会受到右边线段的负向贡献
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }
//把一串点按组看待，要求每组总长度尽量均匀，同时总长度也别太长，并把这个要求转换成对每个点位置的梯度
  void PolyTrajOptimizer::lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int N = ps.cols() - 1;
    int M = N / n;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(M), lsqrs(M);
    for (int i = 0; i < M; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = wei_sqrvar_ * (lsqrm - lm * lm) + 250.0 * M * lm;
    Eigen::VectorXd gdls = wei_sqrvar_ * 2.0 / M * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i < M; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);//读取每段轨迹有多少个约束点
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);//读取障碍物硬惩罚权重
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);//读取障碍物软惩罚权重
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);//读取群体避碰权重
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);//读取可行性代价权重
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);//读取方差项权重，管点分布均匀性代价
    nh.param("optimization/weight_time", wei_time_, -1.0);//读取时间代价权重
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);//读取障碍物硬安全距离
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);//读取障碍物软安全距离
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);//读取多机安全距离
    nh.param("optimization/max_vel", max_vel_, -1.0);//读取最大速度限制
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);
  }
  //给优化器设置环境地图
  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }//把外部多机轨迹列表指针传给优化器

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }//设置当前这台无人机的编号

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }//设置当前轨迹是否已经触达最终目标

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }//把外部构造好的整套约束点结构直接赋值给内部

  void PolyTrajOptimizer::setUseMultitopologyTrajs(bool use_multitopology_trajs) { multitopology_data_.use_multitopology_trajs = use_multitopology_trajs; }

} // namespace ego_planner
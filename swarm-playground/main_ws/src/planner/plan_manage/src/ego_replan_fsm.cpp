
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;//多机时，是否已经收到前序编号无人机的轨迹
    flag_escape_emergency_ = true;//紧急避障标志
    mandatory_stop_ = false;//mandatory强制的

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);//目标来源，手动目标或预设 waypoint
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);//执行多久后主动重规划。
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);//局部规划视野。
    nh.param("fsm/emergency_time", emergency_time_, 1.0);//距离碰撞剩余时间小于这个值，就急停。
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);//急停后是否允许重新规划逃离。
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);//是否启用地面高度测量

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)//循环读取每个航点三维坐标。
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));//创建可视化模块对象
    planner_manager_.reset(new EGOPlannerManager);//创建规划管理器对象
    planner_manager_->initPlanModules(nh, visualization_);//初始化规划相关子模块，并传入节点句柄与可视化对象

    have_trigger_ = !flag_realworld_experiment_;//若不是实机实验，则默认“已触发”；实机实验通常需要外部触发后再启动
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;//计算“无需重规划”的距离阈值（按“半个紧急时间窗内最大位移”估算）

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);//发布 MINCO 轨迹用于多机广播（减小通信消息体积）。
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());//订阅其他无人机广播来的 MINCO 轨迹；tcpNoDelay() 减少 TCP 延迟。

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);//发布本机多项式轨迹。
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);//发布可视化/调试数据
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);//发布心跳包，表示规划节点在正常工作
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);//发布地面高度测量值

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)//如果目标来源是“手动输入目标”。
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

      readGivenWpsAndPlan();//读取预设航点并立即规划
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)//FSM 主循环回调
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;//构造一个空心跳消息
    heartbeat_pub_.publish(heartbeat_msg);//发布心跳，告诉外部“FSM 还活着”

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();//打印当前 FSM 状态到日志
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");//有里程计后转到等待目标状态
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)//没目标或没触发信号就继续等
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo  紧急后或其他场景下重新从全局轨迹生成新轨迹
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

      if (planFromLocalTraj(1))////基于当前局部轨迹做短时重规划
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
      LocalTrajData *info = &planner_manager_->traj_.local_traj;//取当前局部轨迹信息
      double t_cur = ros::Time::now().toSec() - info->start_time;//当前已执行时间
      t_cur = min(info->duration, t_cur);//限幅到轨迹总时长内
      Eigen::Vector3d pos = info->traj.getPos(t_cur);//计算当前时刻对应轨迹位置
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);//判断“当前局部目标是否就是最终目标”

      const PtsChk_t* chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;//取碰撞检查时间点缓存
      //若检查数组非空，则判断“离当前轨迹末端是否不足 emergency_time_”；空数组时置 false
      bool close_to_current_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? chk_ptr->back().back().first - t_cur < emergency_time_ : 0; // In case of empty vector

      if (mondifyInCollisionFinalGoal()) // case 1: find that current goal is in obstacles
      {
        // pass
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) // case 2: assign the next waypoint预设航点任务且接近当前终点，并且还有下一个航点
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);//切到下一个航点并规划下一段
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)//若是预设任务，重置到第一个航点，为下一轮任务做准备。
        {
          // prepare for next round
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) // case 3: time to perform next replan到了重规划时间，或尚未到最终目标但轨迹快结束
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls首次进入紧急状态时执行急停（避免重复调用）
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)//若已急停过，且启用 fail-safe 且速度已很小，则尝试重新生成轨迹脱困。
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();//更新显示/调试数据时间戳
    data_disp_pub_.publish(data_disp_);

  force_return:;//标签，供前面 goto 提前跳转到统一收尾
    exec_timer_.start();//重新启动定时器，保证下个周期继续运行 FSM
  }
//定义状态切换函数：把 FSM 切到 new_state，pos_call 用来标记是谁调用了切换
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)//判断“新状态是否和当前状态相同”
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);//把切换前状态保存成整数索引，后面用于打印“from”。
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()//打印当前 FSM 状态和等待信息
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }
//定义函数：返回“连续调用次数 + 当前状态”
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);//组装并返回一个 pair
  }
//安全定时器回调：周期检查碰撞风险（含地面高度附带测量）
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;//取当前局部轨迹数据指针
    auto map = planner_manager_->grid_map_;
    const double t_cur = ros::Time::now().toSec() - info->start_time;//当前时刻相对轨迹起点的已执行时间
    PtsChk_t pts_chk = info->pts_chk;//拷贝轨迹采样检查点（每个点含时间和位置）

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)//若还在等目标或当前无有效轨迹，直接不检查
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;//关闭 fail-safe 重规划
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!复制时间变量
    int i_start = info->traj.locatePieceIdx(t_temp);//定位 t_cur 所在轨迹 piece 索引

    if (i_start >= (int)pts_chk.size())//起始 piece 超范围就退出
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)//找到后跳到标签，作为后续遍历起点
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;//若已接近最终目标则检查到末尾；否则只检查前 3/4（降低负担、滚动前瞻）
    for (size_t i = i_start; i < i_end; ++i)//遍历 piece
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)//遍历 piece 内采样点
      {

        double t = pts_chk[i][j].first;//取该检查点的时间
        Eigen::Vector3d p = pts_chk[i][j].second;//取该检查点的位置

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);//先检查该点是否与膨胀障碍物冲突。

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)//再逐个检查其他无人机轨迹间距
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;//跳过无效条目和自己
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);//将本机时间对齐到对方轨迹时间轴
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)//只在对方轨迹有效时间内比较。
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);//预测对方在 t_X 的位置。
            double dist = (p - swarm_pridicted).norm();//计算两机距离
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;//允许最小安全距离 = 本机集群间隔 + 对方期望间隔
            if (dist < allowed_dist)//若距离过近则告警，标记危险并退出群机循环。
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance先尝试“就地局部重规划”挽救
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);//马上要撞，进 EMERGENCY_STOP
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");//还有时间，进 REPLAN_TRAJ
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)//定义“急停”函数，输入急停位置 stop_pos，返回是否成功
  {

    planner_manager_->EmergencyStop(stop_pos);//调用规划管理器执行急停轨迹/控制逻辑，目标是尽快停在 stop_pos 附近
    traj_utils::PolyTraj poly_msg;//定义普通多项式轨迹 ROS 消息对象
    traj_utils::MINCOTraj MINCO_msg;//定义 MINCO 轨迹消息对象（通信更精简）
    polyTraj2ROSMsg(poly_msg, MINCO_msg);//把当前规划结果转换成两种 ROS 消息格式
    poly_traj_pub_.publish(poly_msg);//发布本机轨迹消息
    broadcast_ploytraj_pub_.publish(MINCO_msg);//广播 MINCO 轨迹给其他无人机/模块

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)//定义“rebound 重规划”函数
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

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

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo定义“从全局任务出发做重规划”的函数，trial_times 是最多重试次数
  {

    start_pt_ = odom_pos_;//规划起点设为当前里程计位置
    start_vel_ = odom_vel_;//起始速度设为当前速度
    start_acc_.setZero();

    bool flag_random_poly_init;//声明是否使用随机多项式初始化的标志
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;//首次先不用随机初始化
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;//调用重规划：use_poly_init=true，随机开关按上面逻辑；一旦成功立即返回 true。
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)//基于当前已执行局部轨迹继续重规划”的函数
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;//取当前局部轨迹信息
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);//第一层尝试：不用多项式初始化、不随机

    if (!success)//若失败，第二层尝试：用多项式初始化，但不随机
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);//还失败则进入第三层：多项式初始化 + 随机初始化，最多试 trial_times 次
          if (success)
            break;
        }
        if (!success)
        {
          return false;//三层策略都失败则返回 false
        }
      }
    }

    return true;//任一策略成功即返回 true
  }
//给定下一个航点 next_wp，规划到它的全局轨迹，并触发后续 FSM 行为。
  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;//创建航点数组
    one_pt_wps.push_back(next_wp);//把单个目标点放进数组
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      final_goal_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;//设可视化采样时间步长 0.1s
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);//根据全局轨迹总时长计算采样点个数
      vector<Eigen::Vector3d> gloabl_traj(i_end);//创建轨迹点数组
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);//取每个采样时刻的位置点
      }

      have_target_ = true;
      have_new_target_ = true;

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

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);//显示采样后的全局路径（用于 RViz/调试可视化）
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }
//若当前 final_goal_ 落在障碍物里，尝试把它改到全局轨迹上的最近无碰撞点。返回是否修改成功
  bool EGOReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;//计算沿全局轨迹回退搜索的时间步长
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)//从全局轨迹末端（原目标附近）向前回退扫描
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);//取该时刻对应的轨迹点 pt
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))//如果这个点不在障碍里（候选安全点）
        {
          if (planNextWaypoint(pt)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= t_step)//如果已经回退到轨迹开头还没找到安全点，报错
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;//以下两种情况返回 false：原目标本来就不碰撞（无需修改），原目标碰撞但没找到可替代安全点
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)//手动目标回调：收到 /goal 消息后执行
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
      return;//过滤无效目标：不是发给本机的 drone_id，忽略，目标高度 z < -0.1，也忽略
    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp))
    {
      have_trigger_ = true;
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()//读取预设航点并启动第一段规划
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);//调整 wps_ 容器大小
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);//在可视化中画目标点（颜色、尺寸、ID）
      ros::Duration(0.001).sleep();//每次绘制后短暂停 1ms，避免刷太快
    }

    // plan first global waypoint
    wpt_id_ = 0;//当前航点索引置为第一个。
    planNextWaypoint(wps_[wpt_id_]);//先规划到第一个航点
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)//强制停车回调（收到 mandatory_stop 话题）
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)//里程计回调：更新当前位置和速度
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)//触发回调（预设航点模式常用）
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;//取发送方无人机 ID
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself自己发给自己的轨迹直接忽略
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)//只支持 5 阶轨迹，不是就拒收
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))//轨迹参数维度不一致，拒收
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;//旧包/乱序/重复包丢弃
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)//时间戳差超过 0.25s，进入时钟偏差处理
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());//偏差较大但没离谱，警告并继续
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());//偏差超大（>10s）判定群时钟不同步，拒收
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)//若缓存数组不够大，先扩容到能放下 recv_id
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);//新增空槽位，drone_id=-1 表示该槽无有效轨迹
      }
    }

    if ( msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_nums = msg->duration.size();//轨迹段数
    Eigen::Matrix<double, 3, 3> headState, tailState;//定义起/终状态矩阵（每轴位置、速度、加速度）
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];//填充起点状态
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];//填充终点状态
    Eigen::MatrixXd innerPts(3, piece_nums - 1);//定义中间控制点矩阵
    Eigen::VectorXd durations(piece_nums);//定义每段时长向量
    for (int i = 0; i < piece_nums - 1; i++)//读取每个中间点
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)//读取每段时长
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;//创建最小 jerk 优化器对象
    MJO.reset(headState, tailState, piece_nums);//设置边界条件和段数
    MJO.generate(innerPts, durations);//用接收参数生成轨迹

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient取约束采样点（K=5）用于“远近判断”
    bool far_away = true;//默认认为“离我很远
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // Accept a far traj if no previous agent received近轨迹一定收；若还没收到前序机轨迹，则远轨迹也先收（保证启动链路）
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();//取生成好的轨迹对象
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (planner_manager_->checkCollision(recv_id))
      {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");//若新收到的他机轨迹与我机有冲突，触发我机重规划
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)//若之前还没满足“已收前序机”条件，则尝试更新该标志
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)//当缓存覆盖到自身 ID 前的所有编号时，检查 0~drone_id-1 是否都有效
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
            {
              break;//只要有一个前序机没收到，就退出
            }

            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; // Means this trajectory is invalid若轨迹太远且已不需要它，则标记该槽无效
    }
  }
//把当前局部轨迹同时转换成两种 ROS 消息：PolyTraj 和 MINCOTraj
  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;//取当前局部轨迹数据指针
    Eigen::VectorXd durs = data->traj.getDurations();//取每一段轨迹的时长
    int piece_num = data->traj.getPieceNum();//取轨迹段数

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);//为 x/y/z 三轴系数分配空间（每段 6 个）
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;//计算该段在一维系数数组中的起始偏移
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
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);//取起点位置并写入 start_p
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
    MINCO_msg.inner_x.resize(piece_num - 1);//为内部控制点（不含首尾）分配空间
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();//获取轨迹关键位置点矩阵
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];//写每段时长到 MINCO 消息
  }
//测量地面高度，并通过引用参数 height 返回结果；函数返回是否测量成功。
  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;//如果局部轨迹检查点太少（说明规划还没真正开始）直接返回失败
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m计算“前看 2 米”对应的时间（距离=速度×时间）
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;//得到轨迹上“当前时刻往前 2 米处”的时间点
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)//无限循环：每次把 z 高度降低一个栅格分辨率，向下探测
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;//若返回 -1，表示探测到地图底部（越界/无效）
        }
        if (ret == 1) // reach the ground若返回 1，表示命中地面/障碍（这里按地面处理
        {
          height = forward_p(2);//记录当前 z 作为地面高度

          std_msgs::Float64 height_msg;//构造高度消息
          height_msg.data = height;//写入高度值
          ground_height_pub_.publish(height_msg);//发布地面高度话题

          return true;
        }
      }
    }

    return false;//前视点不在有效轨迹内，返回失败
  }
} // namespace ego_planner

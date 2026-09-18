// #include <fstream>
#include <cmath>
#include <limits>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{
  bool EGOPlannerManager::setFormationAssignment(
      const std::vector<int> &assignment)
  {
    return ploy_traj_opt_->setFormationAssignment(assignment);
  }

  void EGOPlannerManager::setManualFormationSplit(const bool enabled)
  {
    ploy_traj_opt_->setManualFormationSplit(enabled);
  }

  void EGOPlannerManager::setDynamicFormationGroups(const std::vector<int> &groups)
  {
    ploy_traj_opt_->setDynamicFormationGroups(groups);
  }

  void EGOPlannerManager::setSplitSubgroupFormationWeightScale(const double scale)
  {
    ploy_traj_opt_->setSplitSubgroupFormationWeightScale(scale);
  }

  void EGOPlannerManager::setSplitProgressSyncWeightScale(const double scale)
  {
    ploy_traj_opt_->setSplitProgressSyncWeightScale(scale);
  }

  void EGOPlannerManager::beginMergeSynchronization(
      const Eigen::Vector3d &position)
  {
    ploy_traj_opt_->beginMergeSynchronization(position);
  }

  void EGOPlannerManager::setSubgroupRouteIntent(
      const bool active, const Eigen::Vector3d &origin,
      const Eigen::Vector3d &direction, const double gain)
  {
    ploy_traj_opt_->setSubgroupRouteIntent(active, origin, direction, gain);
  }


  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/use_wall_collision_guard", use_wall_collision_guard_, false);
    nh.param("manager/wall_x", wall_x_, 0.0);
    nh.param("manager/wall_y", wall_y_, 0.0);
    nh.param("manager/wall_size_x", wall_size_x_, 0.0);
    nh.param("manager/wall_size_y", wall_size_y_, 0.0);
    nh.param("manager/wall_safety_distance", wall_safety_distance_, 0.0);
    nh.param("manager/wall_guard_check_horizon", wall_guard_check_horizon_, 3.0);

    ROS_INFO_STREAM("[wall_guard_config] drone=" << pp_.drone_id
                    << " enabled=" << (use_wall_collision_guard_ ? "true" : "false")
                    << " source=inflated_grid"
                    << " segment_check=voxel_traversal"
                    << " center=(" << wall_x_ << "," << wall_y_ << ")"
                    << " physical_size=(" << wall_size_x_ << "," << wall_size_y_ << ")"
                    << " requested_safety=" << wall_safety_distance_
                    << " horizon=" << wall_guard_check_horizon_);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  bool EGOPlannerManager::isInsideWallGuard(
      const Eigen::Vector3d &position) const
  {
    if (!use_wall_collision_guard_ || !grid_map_)
      return false;

    // The inflated occupancy grid is the single source of truth for the
    // guard boundary.  Checking the analytic wall rectangle here used to
    // disagree with voxel quantization by up to one cell near its boundary.
    return grid_map_->getInflateOccupancy(position) == 1;
  }

  bool EGOPlannerManager::violatesWallCollisionGuard(
      const Eigen::Vector3d &position) const
  {
    return isInsideWallGuard(position);
  }

  bool EGOPlannerManager::segmentViolatesWallCollisionGuard(
      const Eigen::Vector3d &start,
      const Eigen::Vector3d &end) const
  {
    if (!use_wall_collision_guard_ || !grid_map_)
      return false;

    if (grid_map_->getInflateOccupancy(start) == 1 ||
        grid_map_->getInflateOccupancy(end) == 1)
      return true;

    const double resolution = grid_map_->getResolution();
    if (resolution <= 0.0)
      return false;

    // Traverse every voxel touched between consecutive trajectory samples.
    // This preserves the anti-tunnelling purpose of the wall guard while
    // using exactly the same inflated cells as the optimizer and map checks.
    const Eigen::Vector3d origin = grid_map_->getOrigin();
    RayCaster raycaster;
    if (!raycaster.setInput((start - origin) / resolution,
                            (end - origin) / resolution))
      return false;

    Eigen::Vector3d ray_index;
    while (raycaster.step(ray_index))
    {
      const Eigen::Vector3i index = ray_index.cast<int>();
      if (grid_map_->getInflateOccupancyByIndex(index) == 1)
        return true;
    }

    return false;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
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
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if ( t_to_lc_end < 0 )
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
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

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt,
                                              const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret)
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());

    return ret;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const Eigen::Vector3d &formation_start_pt,
      const Eigen::Vector3d &formation_end_pt, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {

    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    ploy_traj_opt_->setFStartFEnd(formation_start_pt, formation_end_pt);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // continous_failures_count_++;
      // return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(),
                                               cstr_pts, final_cost))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(),
                                                        cstr_pts, final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    // // save and display planned results
    cout << "plan_success=" << flag_success << endl;
    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success << endl;

    if (!setLocalTrajFromOpt(best_MJO, touch_goal))
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }
    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (!stop_pos.allFinite())
    {
      ROS_ERROR("Cannot create emergency-stop trajectory from a non-finite position.");
      return false;
    }

    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    // A stationary trajectory does not travel far enough for the normal
    // distance-based collision sampler. Install it directly with one sample
    // per checked control-point interval so emergency stop cannot silently
    // fall back to the previous moving trajectory.
    poly_traj::Trajectory stop_traj = stopMJO.getTraj();
    Eigen::MatrixXd cps =
        stopMJO.getInitConstraintPoints(getCpsNumPrePiece());
    const int checked_point_count = std::max(
        1, ConstraintPoints::two_thirds_id(cps, false));
    const double duration = stop_traj.getTotalDuration();
    PtsChk_t stop_points(checked_point_count);
    for (int index = 0; index < checked_point_count; ++index)
    {
      const double sample_time =
          duration * static_cast<double>(index + 1) /
          static_cast<double>(checked_point_count);
      stop_points[index].emplace_back(sample_time, stop_pos);
    }
    traj_.setLocalTraj(stop_traj, stop_points, ros::Time::now().toSec());

    ROS_WARN("[drone %d] Installed emergency-stop trajectory id=%d at (%.3f, %.3f, %.3f).",
             pp_.drone_id, traj_.local_traj.traj_id,
             stop_pos.x(), stop_pos.y(), stop_pos.z());

    return true;
  }

  bool EGOPlannerManager::findNearestCollisionFreePosition(
      const Eigen::Vector3d &position,
      Eigen::Vector3d &free_position) const
  {
    if (!grid_map_ || !position.allFinite())
      return false;

    Eigen::Vector3i origin_index;
    grid_map_->posToIndex(position, origin_index);
    if (!grid_map_->isInMap(origin_index))
      return false;

    // Search only in the horizontal plane. The random-scene walls span the
    // flight height, so a vertical recovery would be an unintended bypass.
    const int max_steps = std::max(
        1, static_cast<int>(std::ceil(2.0 / grid_map_->getResolution())));
    // Prefer a target with two extra free rings around it.  Merely choosing
    // the first non-inflated voxel can produce a few-centimetre escape that is
    // immediately classified occupied again because of tracking/map jitter.
    for (int clearance_steps = 2; clearance_steps >= 0; --clearance_steps)
    {
      for (int radius = 1; radius <= max_steps; ++radius)
      {
        bool found = false;
        double best_distance = std::numeric_limits<double>::infinity();
        Eigen::Vector3d best_candidate;
        for (int dx = -radius; dx <= radius; ++dx)
        {
          for (int dy = -radius; dy <= radius; ++dy)
          {
            if (std::max(std::abs(dx), std::abs(dy)) != radius)
              continue;

            const Eigen::Vector3i candidate_index =
                origin_index + Eigen::Vector3i(dx, dy, 0);
            bool clearance_is_free = true;
            for (int cx = -clearance_steps;
                 cx <= clearance_steps && clearance_is_free; ++cx)
            {
              for (int cy = -clearance_steps; cy <= clearance_steps; ++cy)
              {
                const Eigen::Vector3i clearance_index =
                    candidate_index + Eigen::Vector3i(cx, cy, 0);
                if (!grid_map_->isInMap(clearance_index) ||
                    grid_map_->getInflateOccupancyByIndex(clearance_index) != 0)
                {
                  clearance_is_free = false;
                  break;
                }
              }
            }
            if (!clearance_is_free)
              continue;

            Eigen::Vector3d candidate;
            grid_map_->indexToPos(candidate_index, candidate);
            candidate.z() = position.z();
            const double distance = (candidate - position).squaredNorm();
            if (distance < best_distance)
            {
              found = true;
              best_distance = distance;
              best_candidate = candidate;
            }
          }
        }
        if (found)
        {
          free_position = best_candidate;
          return true;
        }
      }
    }

    return false;
  }

  bool EGOPlannerManager::EmergencyEscape(
      const Eigen::Vector3d &start_pos,
      const Eigen::Vector3d &free_pos)
  {
    if (!start_pos.allFinite() || !free_pos.allFinite() || !grid_map_ ||
        grid_map_->getInflateOccupancy(free_pos) != 0)
      return false;

    const double distance = (free_pos - start_pos).norm();
    if (distance < 1e-3)
      return EmergencyStop(start_pos);

    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> head_state, tail_state;
    head_state << start_pos, zero, zero;
    tail_state << free_pos, zero, zero;

    poly_traj::MinJerkOpt escape_mjo;
    escape_mjo.reset(head_state, tail_state, 2);
    Eigen::Matrix<double, 3, 1> midpoint;
    midpoint.col(0) = 0.5 * (start_pos + free_pos);
    const double duration = std::max(1.0, distance / 0.25);
    escape_mjo.generate(
        midpoint, Eigen::Vector2d(duration * 0.5, duration * 0.5));

    poly_traj::Trajectory escape_traj = escape_mjo.getTraj();
    Eigen::MatrixXd cps =
        escape_mjo.getInitConstraintPoints(getCpsNumPrePiece());
    const int checked_point_count = std::max(
        1, ConstraintPoints::two_thirds_id(cps, false));
    PtsChk_t escape_points(checked_point_count);
    for (int index = 0; index < checked_point_count; ++index)
    {
      const double sample_time =
          escape_traj.getTotalDuration() * static_cast<double>(index + 1) /
          static_cast<double>(checked_point_count);
      escape_points[index].emplace_back(
          sample_time, escape_traj.getPos(sample_time));
    }
    traj_.setLocalTraj(escape_traj, escape_points, ros::Time::now().toSec());

    ROS_ERROR("[drone %d] Installed collision-recovery trajectory id=%d from "
              "(%.3f, %.3f, %.3f) to nearest collision-free voxel "
              "(%.3f, %.3f, %.3f).",
              pp_.drone_id, traj_.local_traj.traj_id,
              start_pos.x(), start_pos.y(), start_pos.z(),
              free_pos.x(), free_pos.y(), free_pos.z());
    return true;
  }

  bool EGOPlannerManager::findDirectionalCollisionFreePosition(
      const Eigen::Vector3d &position,
      const Eigen::Vector3d &preferred_direction,
      const double minimum_displacement,
      Eigen::Vector3d &free_position) const
  {
    if (!grid_map_ || !position.allFinite() ||
        !preferred_direction.allFinite())
      return false;

    Eigen::Vector3d direction = preferred_direction;
    direction.z() = 0.0;
    if (direction.norm() < 1e-3)
      direction = Eigen::Vector3d::UnitY();
    else
      direction.normalize();

    Eigen::Vector3i origin_index;
    grid_map_->posToIndex(position, origin_index);
    if (!grid_map_->isInMap(origin_index))
      return false;

    const double resolution = grid_map_->getResolution();
    const int min_steps = std::max(
        1, static_cast<int>(std::ceil(minimum_displacement / resolution)));
    const int max_steps = std::max(
        min_steps, static_cast<int>(std::ceil(1.5 / resolution)));

    // Search short moves first.  A candidate must point at least 60 degrees
    // away from the peer and retain two additional free voxel rings.
    for (int radius = min_steps; radius <= max_steps; ++radius)
    {
      bool found = false;
      double best_alignment = -1.0;
      Eigen::Vector3d best_candidate;
      for (int dx = -radius; dx <= radius; ++dx)
      {
        for (int dy = -radius; dy <= radius; ++dy)
        {
          if (std::max(std::abs(dx), std::abs(dy)) != radius)
            continue;

          const Eigen::Vector3d grid_direction(
              static_cast<double>(dx), static_cast<double>(dy), 0.0);
          const double alignment = grid_direction.normalized().dot(direction);
          if (alignment < 0.5)
            continue;

          const Eigen::Vector3i candidate_index =
              origin_index + Eigen::Vector3i(dx, dy, 0);
          bool clearance_is_free = true;
          for (int cx = -2; cx <= 2 && clearance_is_free; ++cx)
          {
            for (int cy = -2; cy <= 2; ++cy)
            {
              const Eigen::Vector3i clearance_index =
                  candidate_index + Eigen::Vector3i(cx, cy, 0);
              if (!grid_map_->isInMap(clearance_index) ||
                  grid_map_->getInflateOccupancyByIndex(clearance_index) != 0)
              {
                clearance_is_free = false;
                break;
              }
            }
          }
          if (!clearance_is_free)
            continue;

          Eigen::Vector3d candidate;
          grid_map_->indexToPos(candidate_index, candidate);
          candidate.z() = position.z();
          if (segmentViolatesWallCollisionGuard(position, candidate))
            continue;

          if (!found || alignment > best_alignment)
          {
            found = true;
            best_alignment = alignment;
            best_candidate = candidate;
          }
        }
      }
      if (found)
      {
        free_position = best_candidate;
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
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
              .norm() < getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
      const bool reset_local_trajectory)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

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

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec(),
                        reset_local_trajectory);

    return true;
  }

} // namespace ego_planner

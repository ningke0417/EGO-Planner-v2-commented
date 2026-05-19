#include "plan_env/grid_map.h"

// 这份实现文件对应的是 EGO-PLANNER-V2 当前主线使用的局部 3D 占据地图。
// 核心思想是：
// 1. 始终只维护无人机附近的一块局部地图，而不是全局大地图；
// 2. 这块局部地图通过 ring buffer 的方式随着无人机移动；
// 3. 深度图路径会先投影，再做 raycast，同时更新“障碍”和“自由空间”；
// 4. 点云路径更简单，直接把点打成障碍，不额外恢复自由空间。

/* ============================== initialization ============================== */

// 初始化整个地图模块：
// 1. 读取参数；
// 2. 按局部窗口大小分配 ring buffer；
// 3. 建立深度/点云/里程计/外参订阅；
// 4. 建立占据更新、可视化、衰减定时器。
void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* ---------- 读取 ROS 参数 ---------- */
  // 这里读到的都是“配置项”，真正的局部地图边界要等拿到当前位姿后才能确定。
  // double x_size, y_size, z_size;
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range3d_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range3d_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range3d_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/enable_virtual_wall", mp_.enable_virtual_walll_, false);//是否启用这个虚拟边界开关
  node_.param("grid_map/virtual_ceil", mp_.virtual_ceil_, -1.0);
  node_.param("grid_map/virtual_ground", mp_.virtual_ground_, -1.0);
//深度相机的标定参数，用来完成“像素 <-> 三维点”的转换。
  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);//是否启用深度滤波
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, 0.1);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/fading_time", mp_.fading_time_, 1000.0);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, 0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);

  /* ---------- 根据分辨率与膨胀半径推导离散参数 ---------- */
  // inf_grid_ 表示“障碍物向外膨胀多少个 voxel”。
  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)
  {
    // 保护性约束：膨胀层数过大时，3D 邻域开销会迅速上升。
    // 这里通过增大 resolution 降低体素数量，避免计算量爆炸。
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // 先把连续尺寸变成整数 voxel 半径，再反写回真实可表示的物理尺寸。
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  // 原始局部窗口与膨胀窗口的 ring buffer 尺寸。
  md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
  md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

  /* ---------- 预计算 log-odds 形式的占据参数 ---------- */
  // 后面做占据更新时统一在 log-odds 空间累加，比直接维护概率更方便。
  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  /* ---------- 分配 ring buffer ---------- */
  // 注意：这里分配的是“局部窗口大小”的 buffer，不是全局地图大小。
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
  md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  md_.raycast_num_ = 0;
  md_.proj_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;

  /* ---------- 设置默认相机外参 ---------- */
  // 如果外部没有立刻给出外参，就先使用这一组默认 cam2body 变换。
  // 含义是：把相机系下的点转换到机体系。
  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* ---------- 建立订阅器 ---------- */
  // 深度图是主输入；位姿既可以来自 PoseStamped，也可以来自 Odometry。
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    // 深度图 + 相机位姿
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    // 深度图 + 机体里程计；相机位姿由 odom 和 cam2body 共同计算。
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // 独立点云路径：cloud 直接建图，odom 只负责给出当前机体/相机位置。
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);

  /* ---------- 建立定时器 ---------- */
  // occ_timer_: 真正的建图主循环；
  // vis_timer_: 周期发布原始占据和膨胀占据；
  // fading_timer_: 长时间未再次观测到的障碍逐渐衰减。
  occ_timer_ = node_.createTimer(ros::Duration(0.032), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.125), &GridMap::visCallback, this);
  if (mp_.fading_time_ > 0)
    fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::fadingCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_.fromSec(0);

  md_.flag_have_ever_received_depth_ = false;
  md_.flag_depth_odom_timeout_ = false;
}

// 深度路径的主更新入口。
// 每次只要收到新的同步深度观测，就在定时器里走一轮完整的融合流程。
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!checkDepthOdomNeedupdate())
    return;

  /* ---------- 主流程：移动窗口 -> 深度投影 -> raycast -> 膨胀 ---------- */
  ros::Time t1, t2, t3, t4, t5;
  t1 = ros::Time::now();

  // 先把局部窗口移动到当前相机附近；这一步可能会清理掉已经滑出窗口的旧体素。
  moveRingBuffer();
  t2 = ros::Time::now();

  // 把深度图采样点投影到世界系。
  projectDepthImage();
  t3 = ros::Time::now();

  if (md_.proj_points_cnt_ > 0)
  {
    // 对每个投影点做射线遍历，更新 hit/miss。
    raycastProcess();
    t4 = ros::Time::now();

    // 根据原始占据图结果同步维护膨胀后的占据图。
    clearAndInflateLocalMap();
    t5 = ros::Time::now();

    if (mp_.show_occ_time_)
    {
      cout << setprecision(7);
      cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      static int updatetimes = 0;
      static double raycasttime = 0;
      static double max_raycasttime = 0;
      static double inflationtime = 0;
      static double max_inflationtime = 0;
      raycasttime += (t4 - t3).toSec();
      max_raycasttime = max(max_raycasttime, (t4 - t3).toSec());
      inflationtime += (t5 - t4).toSec();
      max_inflationtime = max(max_inflationtime, (t5 - t4).toSec());
      ++updatetimes;

      printf("Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t4 - t3).toSec() * 1000, raycasttime / updatetimes * 1000, max_raycasttime * 1000);
      printf("Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t5 - t4).toSec() * 1000, inflationtime / updatetimes * 1000, max_inflationtime * 1000);
    }
  }

  md_.occ_need_update_ = false;
}

// 周期发布地图到 ROS topic，用于 RViz 可视化或其他模块订阅。
void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  // 局部窗口还没围绕当前位置初始化好时，不发布任何内容。
  if (!mp_.have_initialized_)
    return;

  ros::Time t0 = ros::Time::now();
  publishMapInflate();
  publishMap();
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    printf("Visualization(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}

// 让占据概率随着时间衰减。
// 作用是：如果某个障碍很久没有再次被观测到，就逐渐把它“淡出”地图。
void GridMap::fadingCallback(const ros::TimerEvent & /*event*/)
{
  const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz
  const double low_thres = mp_.clamp_min_log_ + reduce;

  ros::Time t0 = ros::Time::now();
  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
  {
    if (md_.occupancy_buffer_[i] > low_thres)
    {
      bool obs_flag = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;
      md_.occupancy_buffer_[i] -= reduce;
      if (obs_flag && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_)
      {
        Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
        int inf_buf_idx = globalIdx2InfBufIdx(idx);
        changeInfBuf(false, inf_buf_idx, idx);
      }
    }
  }
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    printf("Fading(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}

// 输入形式一：直接收到“深度图 + 相机位姿”。
// 这是最直接的路径，相机位姿不需要通过机体 odom 和外参再转换。
void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* ---------- 取深度图 ---------- */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  // 如果上游给的是米制浮点深度，这里统一转换成 uint16 深度图，便于后续统一处理。
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // 第一次拿到深度图时，再按实际分辨率分配投影点缓存。
  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
  }

  /* ---------- 取相机位姿 ---------- */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();

  // 告诉定时器：有一帧新观测可以拿来更新地图了。
  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_depth_ = true;
}

// 输入形式二：收到“深度图 + 机体里程计”。
// 相机位姿 = body2world * cam2body，因此这里依赖 extrinsicCallback 给出的外参。
void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{

  /* ---------- 先由 odom 计算 body 在世界系中的位姿 ---------- */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  // 再利用相机外参，得到相机在世界系中的位姿。
  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* ---------- 取深度图 ---------- */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
  }

  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_depth_ = true;
}

// 独立的 odom 输入只服务于点云路径。
// 一旦深度路径已经开始工作，就不再需要这个独立 odom 订阅了。
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.flag_have_ever_received_depth_)
  {
    // 深度路径已经能同步拿到位姿，这个独立订阅可以关闭，减少冗余。
    indep_odom_sub_.shutdown();
    return;
  }

  /* ---------- 计算当前相机位姿 ---------- */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  md_.has_odom_ = true;
}

// 输入形式三：直接接收点云。
// 与深度路径不同，这里不做 raycast，因此不会显式更新自由空间。
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  /* Note: no obstalce elimination in this function! */

  // 点云本身不携带可靠的当前相机位置，因此必须先有 odom。
  if (!md_.has_odom_)
  {
    std::cout << "grid_map: no odom!" << std::endl;
    return;
  }

  // ROS PointCloud2 -> PCL
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  moveRingBuffer();

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    if (p3d.array().isNaN().sum())
      continue;

    if (isInBuf(p3d))
    {
      // 只要点落在局部窗口内，就直接把该体素标成“强占据”。
      // 这条路径不会沿着传感器射线清空自由空间，所以比深度路径更保守。
      Eigen::Vector3i idx = pos2GlobalIdx(p3d);
      int buf_id = globalIdx2BufIdx(idx);
      int inf_buf_id = globalIdx2InfBufIdx(idx);
      md_.occupancy_buffer_[buf_id] = mp_.clamp_max_log_;

      // 如果此前膨胀图里还没把它视为障碍，就补一次膨胀更新。
      if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
      {
        changeInfBuf(true, inf_buf_id, idx);
      }
    }
  }
}

// 接收相机相对于机体的外参，供 depthOdom / odom 路径计算相机位姿使用。
void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::moveRingBuffer()
{
  // 第一次调用时，局部地图窗口还没有围绕当前位置摆好。
  // 这里会根据当前相机位置完成“首次放置”。
  if (!mp_.have_initialized_)
    initMapBoundary();

  /* ---------- 计算新的局部窗口边界 ---------- */
  // center_new 是当前相机所在 voxel 的全局索引。
  Eigen::Vector3i center_new = pos2GlobalIdx(md_.camera_pos_);
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mp_.resolution_;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);

  // 膨胀窗口比原始窗口在各个方向再多 inf_grid 个 voxel。
  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

  /* ---------- 清理从局部窗口中“滑出去”的旧切片 ---------- */
  // casein 编码：
  // 0/1: x 负向移动 / x 正向移动
  // 2/3: y 负向移动 / y 正向移动
  // 4/5: z 负向移动 / z 正向移动
  //
  // 比如 center_new(0) > center_last3i_(0) 表示窗口整体向 +x 方向走了，
  // 那么旧窗口最靠近 -x 的那一层就不再属于当前局部地图，需要清空。
  if (center_new(0) < md_.center_last3i_(0))
    clearBuffer(0, ringbuffer_upbound3i_new(0));
  if (center_new(0) > md_.center_last3i_(0))
    clearBuffer(1, ringbuffer_lowbound3i_new(0));
  if (center_new(1) < md_.center_last3i_(1))
    clearBuffer(2, ringbuffer_upbound3i_new(1));
  if (center_new(1) > md_.center_last3i_(1))
    clearBuffer(3, ringbuffer_lowbound3i_new(1));
  if (center_new(2) < md_.center_last3i_(2))
    clearBuffer(4, ringbuffer_upbound3i_new(2));
  if (center_new(2) > md_.center_last3i_(2))
    clearBuffer(5, ringbuffer_lowbound3i_new(2));

  /* ---------- 调整 ring buffer 的数组原点 ---------- */
  // ringbuffer_origin3i_ / ringbuffer_inf_origin3i_ 是“当前数组地址 0 对应的全局索引”。
  // 当窗口移动后，需要通过加减一个整 buffer 尺寸，把 origin 拉回到合法区间内。
  for (int i = 0; i < 3; ++i)
  {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }

  /* ---------- 提交新的窗口边界 ---------- */
  md_.center_last3i_ = center_new;
  md_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
  md_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
  md_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
  md_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
  md_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
  md_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
  md_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
  md_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;
}

// 把当前缓存的深度图离散采样后投影到世界系，结果存进 md_.proj_points_。
// 后续 raycast 就是沿着“相机中心 -> 这些投影点”逐条处理。
void GridMap::projectDepthImage()
{
  md_.proj_points_cnt_ = 0;

  uint16_t *row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  /* ---------- 无滤波路径：直接把深度图稀疏采样投影出去 ---------- */
  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
        row_ptr = row_ptr + mp_.skip_pixel_;

        // 太近的点通常是无效值或噪声，直接跳过。
        if (depth < 0.1)
          continue;

        // 像素坐标 + 深度 -> 相机系 3D 点。
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        // 相机系 -> 世界系。
        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        md_.proj_points_[md_.proj_points_cnt_++] = proj_pt;
      }
    }
  }
  /* ---------- 深度滤波路径 ---------- */
  // 当前版本只做最基本的近距离裁剪；时间一致性检查的代码保留了，但被显式关闭。
  else
  {

    // 第一帧没有上一时刻图像可比，因此只把“拿到第一帧”的标志置起来。
    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          // 小于最小可信测距的值直接忽略。
          if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }

          // 像素 + 深度 -> 相机系。
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          // 相机系 -> 世界系。
          pt_world = camera_r * pt_cur + md_.camera_pos_;

          md_.proj_points_[md_.proj_points_cnt_++] = pt_world;

          // 下面这段原本打算做“与上一帧重投影一致性检查”，
          // 但当前通过 if(false) 关闭了，因此现在不会真正执行。
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt_++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt_++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* ---------- 保存上一帧信息 ---------- */
  // 即使时间一致性检查目前关闭，也依然维护上一帧位姿与图像，
  // 这样后续如果重新启用这部分逻辑，就不需要再改数据流。

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

// 对投影点执行射线遍历，统一完成：
// 1. 终点 hit；
// 2. 射线路径上的 free miss；
// 3. 对同一轮 update 的观测做去重；
// 4. 最后用 log-odds 融合到 occupancy_buffer_。
void GridMap::raycastProcess()
{
  // cache_voxel_ 保存“这一轮中被 touch 到过的所有 voxel 全局索引”。
  md_.cache_voxel_cnt_ = 0;

  ros::Time t1, t2, t3;

  // 每执行一轮 raycast 都让编号 +1，后面的 flag_rayend_ / flag_traverse_
  // 通过比较这个编号判断“本轮是否已经访问过”。
  md_.raycast_num_ += 1;

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;

  int pts_num = 0;
  t1 = ros::Time::now();
  for (int i = 0; i < md_.proj_points_cnt_; ++i)
  {
    int vox_idx;
    pt_w = md_.proj_points_[i];

    /* ---------- 先处理射线终点 ---------- */
    // 如果终点落在局部窗口外，就把它裁到窗口边界上，并把它当作 miss；
    // 这样可以表达“射线打到了视野边界，但超出局部地图的那一段我不维护”。
    // 如果终点落在窗口内，则把它当作 hit。

    if (!isInBuf(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);
      pts_num++;
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      pts_num++;
      vox_idx = setCacheOccupancy(pt_w, 1);
    }

    /* ---------- 避免同一终点在同一轮被重复处理 ---------- */

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    /* ---------- 对“终点 <-> 相机中心”做体素射线遍历 ---------- */
    // 注意这里输入的是“除以分辨率后的连续 voxel 坐标”，
    // RayCaster 返回的也是 voxel 网格上的离散坐标。
    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      // RayCaster 返回的是 voxel 索引空间中的格子坐标，
      // 这里再换回世界坐标里的体素中心。
      Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;

      pts_num++;
      // 射线途经的格子都看作 free miss。
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        // 同一轮里如果已经从另一条射线访问过这个 free voxel，就停止继续深入。
        // 这是一个典型的去重/剪枝手段，用来减少重复更新。
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  t2 = ros::Time::now();

  /* ---------- 将本轮缓存的 hit/miss 统计真正融合进 occupancy_buffer_ ---------- */
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {

    int idx_ctns = globalIdx2BufIdx(md_.cache_voxel_[i]);

    // 如果该 voxel 在本轮内 hit 次数 >= miss 次数，就走 p_hit，否则走 p_miss。
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    // 统计值只在“当前这轮”有意义，融合完就清零。
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 已经饱和的格子就不再继续往上/往下推。
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      continue;
    }

    // 真正的 log-odds 累加，并限制在 [clamp_min_log_, clamp_max_log_] 区间内。
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }

  t3 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}

// 根据 occupancy_buffer_ 的最新结果，增量维护膨胀地图。
// 只有当前轮被 touch 到过的 voxel 才需要检查，避免全图重扫。
void GridMap::clearAndInflateLocalMap()
{
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {
    Eigen::Vector3i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    // 原始图从“非障碍 -> 障碍”时，给膨胀图做一次增量增加。
    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
    {
      changeInfBuf(true, inf_buf_id, idx);
    }

    // 原始图从“障碍 -> 非障碍”时，把对应膨胀贡献减掉。
    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_)
    {
      changeInfBuf(false, inf_buf_id, idx);
    }
  }
}

// 第一次根据当前相机位置初始化局部地图窗口。
// 这一步只做一次，之后都靠 moveRingBuffer() 维护窗口移动。
void GridMap::initMapBoundary()
{
  mp_.have_initialized_ = true;

  // 用当前相机所在体素作为局部地图中心。
  md_.center_last3i_ = pos2GlobalIdx(md_.camera_pos_);

  // 原始占据图窗口上下界。
  md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.ringbuffer_lowbound3d_ = md_.ringbuffer_lowbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.ringbuffer_upbound3d_ = md_.ringbuffer_upbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);

  // 膨胀图窗口上下界。
  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf_grid3i;
  md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf_grid3d;
  md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf_grid3i;
  md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf_grid3d;

  // 将数组原点拉回到当前窗口覆盖范围内。
  for (int i = 0; i < 3; ++i)
  {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    }
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
    {
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    }

    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    }
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
    {
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
    }
  }

#if GRID_MAP_NEW_PLATFORM_TEST
  testIndexingCost();
#endif
}

// 清空某个从窗口中移出的“切片”。
// casein 指定方向，bound 指定本次需要清空的边界坐标。
void GridMap::clearBuffer(char casein, int bound)
{
  // 这一组三重循环会遍历当前需要被清掉的整片体素。
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z)
      {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);
        // 这个变量原本是想显式定位“需要清理的膨胀切片”，当前版本没有直接使用它。
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        // int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);

        // 把该格在本轮 raycast 中的统计状态全部恢复为“未使用”。
        md_.count_hit_[id_buf] = 0;
        md_.count_hit_and_miss_[id_buf] = 0;
        md_.flag_traverse_[id_buf] = md_.raycast_num_;
        md_.flag_rayend_[id_buf] = md_.raycast_num_;
        // 原始占据图恢复到最小 log-odds，即“最不容易被认为是障碍”的状态。
        md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;

        // 如果该格此前曾经是障碍，需要把它对膨胀图的贡献一并移除。
        if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
        {
          changeInfBuf(false, id_buf_inf, id_global);
        }
      }

#if GRID_MAP_NEW_PLATFORM_TEST
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z)
      {
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);
        if (md_.occupancy_buffer_inflate_[id_buf_inf_clr] != 0)
        {
          ROS_ERROR("Here should be 0!!! md_.occupancy_buffer_inflate_[id_buf_inf_clr]=%d", md_.occupancy_buffer_inflate_[id_buf_inf_clr]);
        }
      }
#endif
}

// 当投影点落在局部窗口外时，沿着“相机 -> 该点”的方向，
// 把它裁剪到局部窗口边界附近，避免访问越界的 voxel。
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = md_.ringbuffer_upbound3d_ - camera_pt;
  Eigen::Vector3d min_tc = md_.ringbuffer_lowbound3d_ - camera_pt;

  // 在参数方程 camera_pt + t * diff 中，寻找最小的正 t，
  // 使得点刚好打到盒子边界。
  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  // 减去一个很小的 epsilon，确保结果仍然落在盒子内，而不是刚好在边界外侧。
  return camera_pt + (min_t - 1e-3) * diff;
}

// 判断本轮是否需要进行建图更新，同时负责深度/里程计超时检测。
bool GridMap::checkDepthOdomNeedupdate()
{
  // 第一次进入时给 last_occ_update_time_ 一个合理初值。
  if (md_.last_occ_update_time_.toSec() < 1.0)
  {
    md_.last_occ_update_time_ = ros::Time::now();
  }
  if (!md_.occ_need_update_)
  {
    // 已经拿到过深度，但长时间没有新同步观测，认为深度或位姿链路丢失。
    if (md_.flag_have_ever_received_depth_ && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return false;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  return true;
}

// 发布“原始占据图”。
// 这里只发布当前局部窗口内、且大致位于相机前方的障碍体素，用于减少可视化开销。
void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  // heading 近似表示相机当前朝向，用于剔除身后区域的体素显示。
  Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // 如果启用了虚拟天花板/地板，则可视化时也一并裁掉这些高度外的区域。
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
  if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_lowbound3d_(0) + mp_.resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1) + mp_.resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd <= ubz; zd += mp_.resolution_)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          // 只显示大致位于视线前方（夹角小于约 60 度）的体素。
          if (heading.dot(relative_dir.normalized()) > 0.5)
          {
            if (md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))] >= mp_.min_occupancy_log_)
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

// 发布“膨胀后的占据图”。
// 路径规划更关心的是这张图，因为它已经把无人机的安全半径折算进障碍体积里。
void GridMap::publishMapInflate()
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_inf_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_inf_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_inf_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_inf_upbound3d_(2);
  if (md_.ringbuffer_inf_upbound3d_(0) - md_.ringbuffer_inf_lowbound3d_(0) > mp_.resolution_ &&
      (md_.ringbuffer_inf_upbound3d_(1) - md_.ringbuffer_inf_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.ringbuffer_inf_lowbound3d_(0) + mp_.resolution_ / 2; xd < md_.ringbuffer_inf_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_inf_lowbound3d_(1) + mp_.resolution_ / 2; yd < md_.ringbuffer_inf_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd < ubz; zd += mp_.resolution_)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          if (heading.dot(relative_dir.normalized()) > 0.5)
          {
            if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))])
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

// 纯调试/性能测试函数：
// 用来粗略比较几种索引转换函数在当前平台上的耗时。
// 主逻辑不会依赖这个函数，只有在 GRID_MAP_NEW_PLATFORM_TEST 打开时才可能被调用。
void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  ros::Time t0 = ros::Time::now();
  double a = 0;
  int b = 0;

  // 基线：只做最简单的整数累加。
  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          b += x + y + z;
        }
  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z)); // 8396us = 7970
          b += id_buf_inf_clr;
          b += x + y + z;
        }
  ros::Time t2 = ros::Time::now();

  // 测 globalIdx2Pos()
  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          a += pos.sum();
          b += x + y + z;
        }
  ros::Time t3 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          a += xd + yd + zd;
        }
  ros::Time t4 = ros::Time::now();

  // 测 pos2GlobalIdx()
  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          Eigen::Vector3i idx = pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)); // 7088us = 478us
          a += xd + yd + zd;
          b += idx.sum();
        }
  ros::Time t5 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound3d_(0); xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound3d_(1); yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_)
        for (double zd = md_.ringbuffer_lowbound3d_(2); zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.resolution_)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)));
          a += xd + yd + zd;
          b += id_buf_inf_clr;
        }
  ros::Time t6 = ros::Time::now();

  // 基线：只遍历 buffer 地址，不做任何反向索引恢复。
  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      b += i;
    }
  ros::Time t7 = ros::Time::now();

  // 测 BufIdx2GlobalIdx()
  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      Eigen::Vector3i idx = BufIdx2GlobalIdx(i); // 36939
      b += i;
      b += idx.sum();
    }
  ros::Time t8 = ros::Time::now();

  int n = md_.occupancy_buffer_.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%d, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f\n", n, (t1 - t0).toSec(), (t2 - t1).toSec(), (t3 - t2).toSec(), (t4 - t3).toSec(), (t5 - t4).toSec(), (t6 - t5).toSec(), (t7 - t6).toSec(), (t8 - t7).toSec());
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).toSec() * 1e9 / n, ((t3 - t2) - (t1 - t0)).toSec() * 1e9 / n, ((t5 - t4) - (t4 - t3)).toSec() * 1e9 / n, ((t6 - t5) - (t4 - t3)).toSec() * 1e9 / n, ((t8 - t7) - (t7 - t6)).toSec() * 1e9 / n);
}

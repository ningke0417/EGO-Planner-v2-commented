#include "plan_env/grid_map.h"

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  // double x_size, y_size, z_size;
  // node_.param("/grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("/grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("/grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);
  // ROS_INFO_STREAM("[GridMap] Read Param /grid_map/pose_type: " << mp_.pose_type_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/frame_id: " << mp_.frame_id_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/odom_depth_timeout: " << mp_.odom_depth_timeout_);

  node_.param("/grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("/grid_map/point_skip_num", mp_.point_skip_num_, mp_.point_skip_num_);//控制点云稀疏采样的步长，值越大，跳过的点越多，计算越省
  node_.param("/grid_map/local_update_range/x", mp_.local_update_range3d_(0), -1.0);
  node_.param("/grid_map/local_update_range/y", mp_.local_update_range3d_(1), -1.0);
  node_.param("/grid_map/local_update_range/z", mp_.local_update_range3d_(2), -1.0);
  node_.param("/grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("/grid_map/enable_virtual_wall", mp_.enable_virtual_walll_, false);
  node_.param("/grid_map/virtual_ceil", mp_.virtual_ceil_, -1.0);
  node_.param("/grid_map/virtual_ground", mp_.virtual_ground_, -1.0);
  node_.param("/grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);

  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/resolution: " << mp_.resolution_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/local_update_range: " << mp_.local_update_range3d_.transpose());//transpose() 把 Eigen 向量/矩阵转置,列向量转行向量
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/obstacles_inflation: " << mp_.obstacles_inflation_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/enable_virtual_wall: " << mp_.enable_virtual_walll_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/virtual_ceil: " << mp_.virtual_ceil_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/virtual_ground: " << mp_.virtual_ground_);

  node_.param("/grid_map/p_hit", mp_.p_hit_, 0.70);//表示射线命中障碍时使用的占据概率
  node_.param("/grid_map/p_miss", mp_.p_miss_, 0.35);//表示射线穿过空闲体素时的更新概率
  node_.param("/grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("/grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("/grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("/grid_map/fading_time", mp_.fading_time_, 1000.0);//地图衰减时间，用于动态环境中逐渐淡化旧障碍信息
  node_.param("/grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("/grid_map/max_ray_length", mp_.max_ray_length_, -0.1);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/p_hit: " << mp_.p_hit_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/p_miss: " << mp_.p_miss_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/p_min: " << mp_.p_min_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/p_max: " << mp_.p_max_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/p_occ: " << mp_.p_occ_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/fading_time: " << mp_.fading_time_);
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/min_ray_length: " << mp_.min_ray_length_);

  node_.param("/grid_map/show_occ_time", mp_.show_occ_time_, false);//是否输出占据地图更新耗时
  ROS_INFO_STREAM("[GridMap] Read Param /grid_map/show_occ_time: " << mp_.show_occ_time_);

  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)//如果膨胀格数大于 4，说明膨胀范围太大了
  {
    mp_.inf_grid_ = 4;//强行把膨胀格数限制为 4
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;//自动把地图分辨率调大，使得膨胀半径 / 分辨率 = 4
    ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;//现在的 local_update_range3d_ 已经不是原始输入值，而是“对齐到分辨率后的实际有效范围”。
  md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
  md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

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

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
  md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);//分配普通占据地图缓冲区，大小是 buffer_size，每个元素初始值是 clamp_min_log_
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);//分配 ray 终点标记数组，初始值是 -1，用于避免一轮中重复处理同一个终点 voxel
  md_.flag_traverse_ = vector<char>(buffer_size, -1);//分配 ray 穿过标记数组，用于避免重复遍历处理
  md_.cache_voxel_ = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));//分配体素缓存数组

  md_.raycast_num_ = 0;
  md_.cloud_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;//初始化缓存体素数量为 0。

  // md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
  //     -1.0, 0.0, 0.0, 0.0,
  //     0.0, -1.0, 0.0, 0.0,
  //     0.0, 0.0, 0.0, 1.0;
  // t_mavros_lidar: [0.021465, -0.007588, 0.146199]
  // R_mavros_lidar: [0.966930, -0.028054, 0.253495, 0.029818, 0.999550, -0.003120, -0.253293, 0.010576, 0.967332]

  // 使用雷达估计状态（未确定）
  //这个矩阵结构是：
  //[R | t]
  //[0 | 1]
  //左上角 3x3 是旋转矩阵 R，右上角 3x1 是平移向量 t，最后一行 0 0 0 1 是齐次坐标标准形式，它表示：把雷达坐标系下的点变换到机体坐标系下。
  md_.lidar2body_ << 0.966930, -0.028054, 0.253495, 0.021465,
  0.029818, 0.999550, -0.003120, -0.007588, 
  -0.253293, 0.010576, 0.967332, 0.146199,
  0.0, 0.0, 0.0, 1.0;

  // 使用PX4 IMU估计状态。直接把 lidar2body_ 设置为 4x4 单位矩阵，这意味着雷达坐标系和机体坐标系被认为完全重合，没有旋转，没有平移
  //这一行覆盖掉了上面刚赋的真实外参矩阵（？？？）
  md_.lidar2body_ = Eigen::Matrix4d::Identity();

  /* init callback */
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  // use odometry and point cloud
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.032), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.125), &GridMap::visCallback, this);
  if (mp_.fading_time_ > 0)
    fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::fadingCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_.fromSec(0);//把上一次占据地图更新时间初始化为时间戳 0，表示“还没有真正更新过”

  // md_.flag_have_ever_received_depth_ = false;
  // md_.flag_depth_odom_timeout_ = false;
}
//有新点云时，把它融合进局部占据地图，并同步更新膨胀障碍图
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  // if (!checkDepthOdomNeedupdate())
  // {
  //   ROS_WARN("checkDepthOdomNeedupdate Fail!");
  //   return;
  // }

  // if (!checkCloudNeedupdate())
  // {
  //   ROS_WARN("checkCloudNeedupdate Fail!");
  //   return;
  // }

  if(!md_.occ_need_update_) {
    return;
  }

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5;
  t1 = ros::Time::now();

  moveRingBuffer();// 移动环形缓冲区（实现动态窗口更新）
  t2 = ros::Time::now();

  // projectDepthImage();
  t3 = ros::Time::now();

  if (md_.cloud_points_cnt_ > 0)
  {
    raycastProcess();
    t4 = ros::Time::now();

    clearAndInflateLocalMap();// 清除旧数据并膨胀障碍物
    t5 = ros::Time::now();

    if (mp_.show_occ_time_)
    {
      cout << setprecision(7);//设置 cout 输出浮点数精度为 7 位有效数字
      cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      static int updatetimes = 0;//记录已经更新了多少次。
      static double raycasttime = 0;//累计 raycast 总耗时
      static double max_raycasttime = 0;
      static double inflationtime = 0;//累计膨胀阶段总耗时
      static double max_inflationtime = 0;
      raycasttime += (t4 - t3).toSec();
      max_raycasttime = max(max_raycasttime, (t4 - t3).toSec());
      inflationtime += (t5 - t4).toSec();
      max_inflationtime = max(max_inflationtime, (t5 - t4).toSec());
      ++updatetimes;

      ROS_INFO("Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t4 - t3).toSec() * 1000, raycasttime / updatetimes * 1000, max_raycasttime * 1000);
      ROS_INFO("Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t5 - t4).toSec() * 1000, inflationtime / updatetimes * 1000, max_inflationtime * 1000);
    }
  }

  md_.occ_need_update_ = false;
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  if (!mp_.have_initialized_)
    return;//为了避免在地图还没准备好时进行可视化

  ros::Time t0 = ros::Time::now();
  publishMapInflate();
  publishMap();
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    printf("Visualization(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}
//定期衰减旧障碍物的占据概率，使地图能够适应动态环境（如移动物体离开、临时障碍物消失等场景），避免因过时的障碍物信息影响机器人导航
void GridMap::fadingCallback(const ros::TimerEvent & /*event*/)//让地图中的旧障碍信息逐渐减弱
{// 计算每次衰减的步长
  const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz(每0.5秒一次）,因此总调用次数为fading_time_ * 2
  // (0.9 - 0.8) / 10，计算每次回调要减少多少 log-odds 值，mp_.clamp_max_log_：占据值上限，mp_.min_occupancy_log_：判定为障碍的阈值，它们的差值表示“从强障碍状态衰减到刚低于障碍阈值”需要跨过的范围
  const double low_thres = mp_.clamp_min_log_ + reduce;//定义一个低阈值 low_thres，避免对已接近 “非占据” 的栅格过度处理

  ros::Time t0 = ros::Time::now();
  // 遍历所有网格，衰减占据概率
  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
  {
    if (md_.occupancy_buffer_[i] > low_thres)// 只处理需要衰减的栅格
    {
      bool obs_flag = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;//先记录这个 voxel 在衰减前是否被认为是障碍物
      md_.occupancy_buffer_[i] -= reduce;
      // 若衰减后从“障碍物”变为“非障碍物”，清除对应的膨胀标记
      if (obs_flag && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_)//这个 voxel 刚刚从“障碍”变成“非障碍”
      {
        Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
        int inf_buf_idx = globalIdx2InfBufIdx(idx);
        changeInfBuf(false, inf_buf_idx, idx);//把这个障碍从膨胀地图中移除。
      }
    }
  }
  ros::Time t1 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    // printf("Fading(ms):%f\n", (t1 - t0).toSec() * 1000);
    ROS_INFO("Fading(ms):%f, md_.occupancy_buffer_.size(): %d\n", (t1 - t0).toSec() * 1000, md_.occupancy_buffer_.size());
  }
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  // if (md_.flag_have_ever_received_depth_)
  // {
  //   indep_odom_sub_.shutdown();
  //   return;
  // }

  /* get pose */
  //从 odometry 消息中取出姿态四元数，构造一个 Eigen 的四元数对象 body_q，表示：机体 body 坐标系相对于世界坐标系的旋转
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();//把四元数 body_q 转换成 3x3 旋转矩阵body_r_m，表示body 坐标轴如何旋转到 world 坐标轴
  Eigen::Matrix4d body2world;//一个 4x4 齐次变换矩阵，用来表示body -> world的刚体变换，并未完全初始化，可能存在潜在风险？？？
  body2world.block<3, 3>(0, 0) = body_r_m;//把 body2world 左上角的 3x3 子块赋值为旋转矩阵 body_r_m
  body2world(0, 3) = odom->pose.pose.position.x;//设置平移向量的 x 分量。
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;//右上角的 3x1 向量就是t = [x, y, z]^T，表示 body 原点在 world 中的位置
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d lidar_T = body2world * md_.lidar2body_;//得到lidar -> world的齐次变换矩阵 lidar_T，最终 lidar_T 表示雷达在世界坐标系下的姿态
  md_.lidar_pos_(0) = lidar_T(0, 3);//雷达在世界坐标系中的 x 位置
  md_.lidar_pos_(1) = lidar_T(1, 3);
  md_.lidar_pos_(2) = lidar_T(2, 3);
  md_.lidar_r_m_ = lidar_T.block<3, 3>(0, 0);//提取 lidar_T 左上角 3x3 旋转矩阵，保存到 md_.lidar_r_m_，表示当前雷达在世界坐标系下的姿态旋转矩阵

  md_.has_odom_ = true;//地图模块现在知道当前雷达/机体在世界中的位置
}
//接收一帧点云，把其中有效障碍点保存下来，并立刻把这些点对应的体素标成占据，同时触发后续 occupancy 更新
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  /* Note: no obstalce elimination in this function! */

  if (!md_.has_odom_)
  {
    std::cout << "grid_map: no odom!" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;//存放转换后的点云。
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.lidar_pos_(0)) || isnan(md_.lidar_pos_(1)) || isnan(md_.lidar_pos_(2)))
    return;//如果当前雷达位置 lidar_pos_ 的任一坐标是 NaN，直接返回

  moveRingBuffer();
  // ROS_ERROR("latest_cloud.points.size(): %ld", latest_cloud.points.size());
  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;
  std::lock_guard<std::mutex> lock(md_.cloud_points_mutex_);//加锁，保护 md_.cloud_points_ 等共享数据
  if (md_.cloud_points_.size() < latest_cloud.points.size()) {
    md_.cloud_points_.resize(latest_cloud.points.size());
  }
  md_.cloud_points_cnt_ = 0;//把当前有效点数量清零，表示这一帧要重新统计有效点

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    if (i % mp_.point_skip_num_ != 0) {
      continue;//做点云降采样/跳点处理，如果当前点序号 i 不能被 point_skip_num_ 整除，就跳过。例如 point_skip_num_ = 8 时，大致每 8 个点只处理 1 个，减少计算量
    }
    
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    if (p3d.array().isNaN().sum()) {
      continue;//如果有任何一维是 NaN，就跳过这个点
    }

    double dist = (p3d - md_.lidar_pos_).norm();//计算当前点到雷达位置的欧氏距离。
    // // if (dist > mp_.max_ray_length_) {
    // //   continue;
    // // } else if (dist < mp_.min_ray_length_) {
    // //   continue;
    // // }
    if (dist < mp_.min_ray_length_) {
      continue;
    }
      

    if (isInBuf(p3d))//如果这个点在当前主地图缓冲区范围内
    {
      md_.cloud_points_[md_.cloud_points_cnt_++] = p3d;//把这个有效点保存到 cloud_points_ 缓存中，并把有效点计数加 1
      /* inflate the point */
      Eigen::Vector3i idx = pos2GlobalIdx(p3d);
      int buf_id = globalIdx2BufIdx(idx);
      int inf_buf_id = globalIdx2InfBufIdx(idx);
      md_.occupancy_buffer_[buf_id] = mp_.clamp_max_log_;//这帧点云的落点直接被强制认为是障碍物
//这个点目前还没有被标记成“真实障碍中心”且确实已经达到障碍阈值
      if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
      {
        changeInfBuf(true, inf_buf_id, idx);//同步更新膨胀地图
      }
    }
  }
  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_cloud_ = true;
}
//从一个 Odometry 消息中读取雷达到机体的外参，并更新 md_.lidar2body_ 这个 4x4 变换矩阵，更新的是“雷达相对机体怎么安装”
void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond lidar2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d lidar2body_r_m = lidar2body_q.toRotationMatrix();//把这个四元数转换成 3x3 旋转矩阵 lidar2body_r_m
  md_.lidar2body_.block<3, 3>(0, 0) = lidar2body_r_m;
  md_.lidar2body_(0, 3) = odom->pose.pose.position.x;
  md_.lidar2body_(1, 3) = odom->pose.pose.position.y;
  md_.lidar2body_(2, 3) = odom->pose.pose.position.z;
  md_.lidar2body_(3, 3) = 1.0;
}
//把局部地图窗口跟着雷达一起滑动，并循环复用底层数组空间
void GridMap::moveRingBuffer()
{
  if (!mp_.have_initialized_)
    initMapBoundary();//moveRingBuffer() 第一次运行时，会顺便把 ring buffer 的初始边界建立好

  Eigen::Vector3i center_new = pos2GlobalIdx(md_.lidar_pos_);//新的地图中心,计算当前相机位置的网格索引
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mp_.resolution_;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);//把上边界索引每个方向减 1，让索引边界变成闭区间

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);//构造一个三维整数向量 inf_grid3i，三个方向都等于膨胀格数 inf_grid_
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;
// 根据相机移动方向清除旧数据
  if (center_new(0) < md_.center_last3i_(0))//如果新中心的 x 比上次更小，说明窗口向 -x 方向移动了
    clearBuffer(0, ringbuffer_upbound3i_new(0));//调用 clearBuffer(0, ...) 清理滑出窗口的那部分 x 区域
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
// 更新环形缓冲区索引
  for (int i = 0; i < 3; ++i)
  {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
    {//如果普通地图的 ring buffer 原点在当前下边界之外，就加上整个 buffer 尺寸，让它“绕回”到合法范围内
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
// 保存当前中心位置和边界
  md_.center_last3i_ = center_new;//更新“上一次中心”为这一次的新中心。
  md_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
  md_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
  md_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
  md_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
  md_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
  md_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
  md_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
  md_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;
}

//把点云观测转成“哪些 voxel 更像障碍，哪些 voxel 更像空闲”的统计，然后再批量写回占据地图
void GridMap::raycastProcess()
{
  md_.cache_voxel_cnt_ = 0;//先把“本轮被访问到的 voxel 数量”清零,重置缓存栅格计数器

  ros::Time t1, t2, t3;

  md_.raycast_num_ += 1;// 射线投射次数计数器+1

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;//ray_pt：raycaster 每一步输出的当前体素坐标，pt_w：当前处理的点云点，世界坐标系下的位置

  int pts_num = 0;
  t1 = ros::Time::now();
  // ROS_INFO("md_.proj_points_cnt_: %d", md_.proj_points_cnt_);
  {
    std::lock_guard<std::mutex> lock(md_.cloud_points_mutex_);//加锁，保护 md_.cloud_points_ 和 md_.cloud_points_cnt_
    for (int i = 0; i < md_.cloud_points_cnt_; ++i)//遍历当前缓存的所有有效点云点
    {
      int vox_idx;
      double length;
      pt_w = md_.cloud_points_[i];

      // set flag for projected point

      if (!isInBuf(pt_w))//如果点云点不在当前地图窗口内
      {
        pt_w = closetPointInMap(pt_w, md_.lidar_pos_);//把地图外的点投影到地图边缘
        pts_num++;
        length = (pt_w - md_.lidar_pos_).norm();//计算裁剪后点到雷达的距离
        if (length > mp_.max_ray_length_) {//如果这个点距离雷达仍然超过最大射线长度，就沿着雷达到该点的方向把它缩短到 max_ray_length_
          pt_w = (pt_w - md_.lidar_pos_) / length * mp_.max_ray_length_ + md_.lidar_pos_;
        }
        vox_idx = setCacheOccupancy(pt_w, 0);//把这个终点先按“空闲”记入缓存统计
      }
      else
      {
        pts_num++;
        // vox_idx = setCacheOccupancy(pt_w, 1);

        length = (pt_w - md_.lidar_pos_).norm();
        if (length > mp_.max_ray_length_) {
          pt_w = (pt_w - md_.lidar_pos_) / length * mp_.max_ray_length_ + md_.lidar_pos_;
          vox_idx = setCacheOccupancy(pt_w, 0);
        } else {
          vox_idx = setCacheOccupancy(pt_w, 1);
        }
      }

      // raycasting between camera center and point下面开始处理雷达到终点之间的整条射线

      if (vox_idx != INVALID_IDX)//如果刚才终点缓存成功
      {
        if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
        {
          continue;//如果这个终点 voxel 在本轮里已经被当作终点处理过了，就直接跳过，避免重复 raycast。
        }
        else
        {
          md_.flag_rayend_[vox_idx] = md_.raycast_num_;
        }
      }
    //输入射线起点和终点，并把后续体素遍历所需的所有内部状态一次性初始化好
      raycaster.setInput(pt_w / mp_.resolution_, md_.lidar_pos_ / mp_.resolution_);

      while (raycaster.step(ray_pt))//不断沿射线一步一步遍历经过的体素，每次 step(ray_pt) 会输出一个当前经过的体素坐标。
      {
        Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;

        // min distance filter
        length = (tmp - md_.lidar_pos_).norm();//计算当前射线经过体素中心到雷达的距离
        if (length < mp_.min_ray_length_) {
          break;
        }

        pts_num++;
        vox_idx = setCacheOccupancy(tmp, 0);//把当前射线经过的体素按“空闲”记入缓存

        if (vox_idx != INVALID_IDX)
        {
          if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)//如果这个 voxel 在本轮当前射线遍历中已经访问过，就停止
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
  }

  t2 = ros::Time::now();
//下面开始第二阶段：根据缓存统计，统一更新 occupancy_buffer_
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)//遍历本轮所有被访问过的 voxel。
  {

    int idx_ctns = globalIdx2BufIdx(md_.cache_voxel_[i]);//把第 i 个缓存 voxel 的三维索引转换成一维 buffer 索引。
// 根据命中次数计算对数几率更新值
    double log_odds_update =//如果命中次数不少于空闲次数，就使用 prob_hit_log_；否则使用 prob_miss_log_
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
// 重置计数器
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;//如果本次是正向更新（趋向占据），并且该 voxel 已经达到上限 clamp_max_log_，就不用再更新了
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      continue;//如果本次是负向更新（趋向空闲），并且它已经达到下限 clamp_min_log_，也不用再更新
    }
// 带约束的概率更新
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);//真正更新该 voxel 的 occupancy 值
  }

  t3 = ros::Time::now();

  if (mp_.show_occ_time_)
  {//raycast time：射线遍历和缓存统计耗时，prob time：根据统计更新 occupancy 的耗时，pts_num：本轮处理的点/体素数量
    ROS_WARN("Raycast time: raycast time: %f, prob time: %f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}
//根据主占据图的当前障碍判定，给膨胀地图补上新增障碍，或者删掉失效障碍
void GridMap::clearAndInflateLocalMap()
{
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)//遍历本轮被访问过的所有 voxel
  {
    Eigen::Vector3i idx = md_.cache_voxel_[i];// 获取缓存中的体素索引
    int buf_id = globalIdx2BufIdx(idx);// 转换为主地图缓冲区索引
    int inf_buf_id = globalIdx2InfBufIdx(idx);
//情况1：主地图中是障碍物，但膨胀地图尚未标记 → 执行膨胀
    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
    {
      changeInfBuf(true, inf_buf_id, idx);//增加障碍影响，把这个 voxel 本身标成障碍中心，同时把周围膨胀邻域计数加上去
    }
//情况2：主地图中不再是障碍物，但膨胀地图仍有标记 → 清除膨胀
    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_)
    {
      changeInfBuf(false, inf_buf_id, idx);
    }
  }
}
//第一次根据当前雷达位置，初始化主地图和膨胀地图的 ring buffer 边界，只在第一次调用时运行，负责“建起第一张局部地图窗口
void GridMap::initMapBoundary()
{
  mp_.have_initialized_ = true;

  md_.center_last3i_ = pos2GlobalIdx(md_.lidar_pos_);

  md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.ringbuffer_lowbound3d_ = md_.ringbuffer_lowbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.ringbuffer_upbound3d_ = md_.ringbuffer_upbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);//前面算出来更像“右开端点”，减去 1 后变成真正最后一个有效 voxel 的索引

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);//构造一个三维整数向量 inf_grid3i，三个方向都等于膨胀格数 inf_grid_
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf_grid3i;
  md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf_grid3d;
  md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf_grid3i;
  md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf_grid3d;

  // cout << "md_.ringbuffer_lowbound3i_=" << md_.ringbuffer_lowbound3i_.transpose() << " md_.ringbuffer_lowbound3d_=" << md_.ringbuffer_lowbound3d_.transpose() << " md_.ringbuffer_upbound3i_=" << md_.ringbuffer_upbound3i_.transpose() << " md_.ringbuffer_upbound3d_=" << md_.ringbuffer_upbound3d_.transpose() << endl;

  for (int i = 0; i < 3; ++i)//调整 ring buffer 的原点，使它落到当前有效窗口范围内
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
//把滑出窗口的旧体素整片清掉，并同步撤销它们对膨胀地图造成的影响
void GridMap::clearBuffer(char casein, int bound)
{//casein = 0~5 就是在指定局部三维窗口的 6 个边界面
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z)
      {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);
        //计算一个“对应的膨胀清理位置” id_global_inf_clr，如果清理的是 casein == 0 那个 x 面，就往 +x 方向偏移 inf_grid_，如果是 casein == 1，就往 -x 偏移 inf_grid_
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        // int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);

        // md_.occupancy_buffer_inflate_[id_buf_inf_clr] = 0;
        md_.count_hit_[id_buf] = 0;
        md_.count_hit_and_miss_[id_buf] = 0;
        md_.flag_traverse_[id_buf] = md_.raycast_num_;//把 traverse 标志设为当前 raycast 轮次号，避免这一轮 raycast 里再错误访问到刚清空的旧区域
        md_.flag_rayend_[id_buf] = md_.raycast_num_;
        md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;

        if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
        {
          changeInfBuf(false, id_buf_inf, id_global);//调用 changeInfBuf(false, ...)，把这个障碍点从膨胀地图中移除
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
//把地图外的点 pt 沿着从 camera_pt 指向它的方向裁剪到地图边界内，返回那个最近的合法点
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)//输入pt：目标点，可能在地图外，camera_pt：相机/雷达位置，通常在地图内
{
  Eigen::Vector3d diff = pt - camera_pt;//计算从相机点 camera_pt 指向目标点 pt 的方向向量 diff
  Eigen::Vector3d max_tc = md_.ringbuffer_upbound3d_ - camera_pt;//计算地图上边界相对相机点的位移向量
  Eigen::Vector3d min_tc = md_.ringbuffer_lowbound3d_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)//如果当前方向上 diff[i] 不为 0，说明这条线在这个轴上有变化，才有可能和该轴的边界相交
    {

      double t1 = max_tc[i] / diff[i];//计算这条线与当前轴“上边界平面”的交点参数 t1
      if (t1 > 0 && t1 < min_t)
        min_t = t1;//如果这个交点在前进方向上（t1 > 0），并且比当前记录的最小正交点还小，就更新 min_t，即保留“最先碰到的边界”

      double t2 = min_tc[i] / diff[i];//计算与当前轴“下边界平面”的交点参数 t2
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }//经过这一轮后，min_t 就是：从 camera_pt 朝 pt 方向走，最先撞到地图包围盒某个面的参数值

  return camera_pt + (min_t - 1e-3) * diff;//返回沿射线方向，在地图内紧贴边界的那个点
}

// bool GridMap::checkDepthOdomNeedupdate()
// {
//   if (md_.last_occ_update_time_.toSec() < 1.0)
//   {
//     md_.last_occ_update_time_ = ros::Time::now();
//   }
//   if (!md_.occ_need_update_)
//   {
//     if (md_.flag_have_ever_received_cloud_ && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
//     {
//       ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
//                 ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
//       md_.flag_depth_odom_timeout_ = true;
//     }
//     return false;
//   }
//   md_.last_occ_update_time_ = ros::Time::now();

//   return true;

// }

bool GridMap::checkCloudNeedupdate()
{
  if (md_.last_occ_update_time_.toSec() < 1.0)//检查 last_occ_update_time_ 是否还接近初始状态。如果它小于 1.0 秒，基本可以认为“还没有正常记录过更新时间”
  {
    md_.last_occ_update_time_ = ros::Time::now();
  }
  // 暂时认为odom不会丢失。
  // if (!md_.occ_need_update_)
  // {
  //   if (md_.flag_have_ever_received_depth_ && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
  //   {
  //     ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
  //               ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
  //     md_.flag_depth_odom_timeout_ = true;
  //   }
  //   return false;
  // }
  md_.last_occ_update_time_ = ros::Time::now();

  return true;
}

void GridMap::publishMap()//把当前局部主占据地图里所有“被判定为障碍物”的 voxel 中心提取出来，作为点云发布出去做可视化
{

  if (map_pub_.getNumSubscribers() <= 0)//如果当前 map_pub_ 这个话题没有任何订阅者，就直接返回
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);//如果启用了虚拟墙，不显示低于虚拟地面的部分，如果没启用直接用地图本身的 z 下边界
  double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
  if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_) {
    for (double xd = md_.ringbuffer_lowbound3d_(0) + mp_.resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.resolution_) {
      for (double yd = md_.ringbuffer_lowbound3d_(1) + mp_.resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.resolution_) {
        for (double zd = lbz + mp_.resolution_ / 2; zd <= ubz; zd += mp_.resolution_) {
          if (zd >= mp_.visualization_truncate_height_) {
            continue;
          }

          if (md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))] >= mp_.min_occupancy_log_) {
            cloud.push_back(pcl::PointXYZ(xd, yd, zd));//如果当前 voxel 是障碍物，就把它的中心点加入点云 cloud
          }
        }
      }
    }
  }
    

  cloud.width = cloud.points.size();//设置点云宽度为点的数量
  cloud.height = 1;//表示这是一维排列的无组织点云，而不是图像式点云
  cloud.is_dense = true;//标记点云是“稠密的”，也就是没有 NaN/无效点
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;//创建一个 ROS 点云消息对象

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}
//把膨胀地图中所有非零的体素提取成点云并发布出去，用来可视化障碍物及其安全膨胀区域
void GridMap::publishMapInflate()
{
  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;//创建一个 PCL 点云对象 cloud，后面用来保存所有“膨胀障碍 voxel”的中心点
  double lbz = mp_.enable_virtual_walll_ ? 
               max(md_.ringbuffer_inf_lowbound3d_(2), mp_.virtual_ground_) 
               : md_.ringbuffer_inf_lowbound3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? 
               min(md_.ringbuffer_inf_upbound3d_(2), mp_.virtual_ceil_) 
               : md_.ringbuffer_inf_upbound3d_(2);

  if (md_.ringbuffer_inf_upbound3d_(0) - md_.ringbuffer_inf_lowbound3d_(0) > mp_.resolution_ &&
      (md_.ringbuffer_inf_upbound3d_(1) - md_.ringbuffer_inf_lowbound3d_(1)) > mp_.resolution_ && 
      (ubz - lbz) > mp_.resolution_) {
    for (double xd = md_.ringbuffer_inf_lowbound3d_(0) + mp_.resolution_ / 2; xd < md_.ringbuffer_inf_upbound3d_(0); xd += mp_.resolution_) {
      for (double yd = md_.ringbuffer_inf_lowbound3d_(1) + mp_.resolution_ / 2; yd < md_.ringbuffer_inf_upbound3d_(1); yd += mp_.resolution_) {
        for (double zd = lbz + mp_.resolution_ / 2; zd < ubz; zd += mp_.resolution_) {
          if (zd >= mp_.visualization_truncate_height_) {
            continue;
          }

          // 直接判断是否占据
          if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))]) {
            cloud.push_back(pcl::PointXYZ(xd, yd, zd));//如果值非 0，就认为该位置在膨胀障碍区，就把它的中心点加入点云。
          }
        }
      }
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

void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  ros::Time t0 = ros::Time::now();
  double a = 0;
  int b = 0;
  for (int i = 0; i < 10; ++i)//做 10 次完整的三维整数网格遍历
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          b += x + y + z;//目的：测一个“纯循环本身”的基线耗时，不包含额外函数调用
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

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound3i_(0); x <= md_.ringbuffer_upbound3i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound3i_(1); y <= md_.ringbuffer_upbound3i_(1); ++y)
        for (int z = md_.ringbuffer_lowbound3i_(2); z <= md_.ringbuffer_upbound3i_(2); ++z)
        {
          Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          a += pos.sum();//为了防止返回值被优化掉
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

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      b += i;//对一维 buffer 做 10 次遍历，只累加索引。这也是一个基线，用于给后面的 BufIdx2GlobalIdx() 对比
    }
  ros::Time t7 = ros::Time::now();

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
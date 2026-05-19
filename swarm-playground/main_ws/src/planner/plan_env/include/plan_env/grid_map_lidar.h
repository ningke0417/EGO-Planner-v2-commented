#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>//引入 ROS 中的位姿消息类型 PoseStamped，表示带时间戳的位姿。
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>//近似时间同步策略。适合多个话题时间戳不完全一致但接近的情况。
#include <message_filters/sync_policies/exact_time.h>//精确时间同步策略。要求多个话题时间戳严格一致。
#include <message_filters/time_synchronizer.h>//消息时间同步器，用来把多个订阅的话题同步后一起处理

#include <plan_env/raycast.h>
#include <mutex>//标准线程互斥锁库，用于多线程下保护共享数据

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false

using namespace std;

// constant parameters

struct MappingParameters
{
  bool have_initialized_ = false;

  /* map properties */
  Eigen::Vector3d local_update_range3d_;//局部地图在 x、y、z 三个方向上的更新半范围
  Eigen::Vector3i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;//膨胀半径对应的栅格数
  string frame_id_;
  // int pose_type_;
  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;

  /* time out */
  double odom_depth_timeout_;//里程计和深度数据的超时时间

  /* raycasting*/
  //p_hit和p_miss：一次观测对概率更新的影响强度
  //p_min和p_max：概率的上下限，防止因多次观测导致概率过于极端（绝对空闲或绝对占据），保持系统对动态障碍物变化的敏感性。
  //p_occ：判断一个栅格是否为障碍物的概率阈值。高于此值，则标记为占据。
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                          // occupancy probability
  //prob_hit_log_：观测到占据时的更新值（正数）,mp_.prob_miss_log_：观测到空闲时的更新值（负数）,mp_.clamp_min_log_和mp_.clamp_max_log_：限制对数几率的范围，避免数值溢出或过度自信
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit of occupancy probability
  double min_ray_length_, max_ray_length_;                                                                   // range of doing raycasting
  double fading_time_;//衰减时间，通常表示地图中的占据信息在多长时间后开始减弱或失效，适合动态环境

  /* visualization and computation time display */
  bool show_occ_time_;
  double visualization_truncate_height_;//可视化截断高度

  int point_skip_num_{8};//表示点云降采样时“每隔多少个点取一个”，或者说跳过点的数量，参数越大，处理点越少，计算更快，但地图精度可能下降、障碍细节可能变少
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3i center_last3i_;//上一次地图中心对应的栅格索引
  // Eigen::Vector3d ringbuffer_origin3d_;
  Eigen::Vector3i ringbuffer_origin3i_;//ringbuffer环形缓冲区
  // Eigen::Vector3d ringbuffer_division3d_;
  // Eigen::Vector3i ringbuffer_division3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_;//ring buffer 的三维下边界
  Eigen::Vector3i ringbuffer_lowbound3i_;
  Eigen::Vector3d ringbuffer_upbound3d_;
  Eigen::Vector3i ringbuffer_upbound3i_;
  // Eigen::Vector3d ringbuffer_size3d_;
  Eigen::Vector3i ringbuffer_size3i_;
  Eigen::Vector3i ringbuffer_inf_origin3i_;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_;
  Eigen::Vector3d ringbuffer_inf_upbound3d_;
  Eigen::Vector3i ringbuffer_inf_upbound3i_;
  Eigen::Vector3i ringbuffer_inf_size3i_;

  // main map data, occupancy of each voxel

  std::vector<double> occupancy_buffer_;//主占据缓冲区
  std::vector<uint16_t> occupancy_buffer_inflate_;

  // camera position and pose data

  Eigen::Vector3d lidar_pos_, last_lidar_pos_;//当前雷达位置和上一次雷达位置
  Eigen::Matrix3d lidar_r_m_, last_lidar_r_m_;//当前和上一帧的姿态旋转矩阵
  Eigen::Matrix4d lidar2body_;


  // flags of map state

  bool occ_need_update_;
  bool has_odom_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;//上一次更新占据地图的 ROS 时间戳。可用于检测数据是否超时。
  // bool flag_depth_odom_timeout_;
  // bool flag_have_ever_received_depth_;
  bool flag_have_ever_received_cloud_;

  // depth image projected point cloud

  std::mutex cloud_points_mutex_;//互斥锁，用来保护点云数据。如果多个线程同时访问 cloud_points_，需要加锁避免数据竞争
  vector<Eigen::Vector3d> cloud_points_;
  int cloud_points_cnt_;//当前点云点数

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;//count_hit_：记录某个 voxel 被“命中”的次数，count_hit_and_miss_：记录被“命中或穿过”的总次数
  vector<char> flag_traverse_, flag_rayend_;//标记某个 voxel 是否被射线穿过，标记某个 voxel 是否是射线终点
  char raycast_num_;//当前 raycasting 的编号或轮次标记

  vector<Eigen::Vector3i> cache_voxel_;//缓存体素列表，存放这一轮处理中涉及到的 voxel 索引。
  int cache_voxel_cnt_;//缓存体素数量

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  void initMap(ros::NodeHandle &nh);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline double getResolution();
  // bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
    
  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    CLOUD_ODOMETRY = 3,
    INVALID_IDX = -10000
  };

  inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id);  // 1.69ns把离散栅格编号映射回该栅格中心坐标
  inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos); // 0.13ns把连续空间坐标映射到离散栅格编号
  inline int globalIdx2BufIdx(const Eigen::Vector3i &id);           // 2.2ns把三维全局栅格索引映射到一维数组缓冲区下标
  inline int globalIdx2InfBufIdx(const Eigen::Vector3i &id);        // 2.2ns
  inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address);          // 10.18ns
  inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address);       // 10.18ns
  inline bool isInBuf(const Eigen::Vector3d &pos);
  inline bool isInBuf(const Eigen::Vector3i &idx);
  inline bool isInInfBuf(const Eigen::Vector3d &pos);
  inline bool isInInfBuf(const Eigen::Vector3i &idx);

  void publishMap();
  void publishMapInflate();

  // get depth image and camera pose
  // void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
  //                        const geometry_msgs::PoseStampedConstPtr &pose);
  void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  // void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);

  // update occupancy by raycasting
  void updateOccupancyCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);
  void fadingCallback(const ros::TimerEvent & /*event*/);//用于周期性执行地图衰减处理

  void clearBuffer(char casein, int bound);

  // main update process
  void moveRingBuffer();
  // void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();

  inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx);//根据方向 dir 修改膨胀地图缓冲区中某个位置的值。通常用于增减膨胀占据计数
  inline int setCacheOccupancy(Eigen::Vector3d pos, int occ);//cache缓存
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  void testIndexingCost();//测试索引转换相关函数的耗时
  // bool checkDepthOdomNeedupdate();
  bool checkCloudNeedupdate();
  void initMapBoundary();

  ros::NodeHandle node_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;//独立点云订阅器，独立里程计订阅器，外参或额外 odom 信息订阅器
  ros::Publisher map_pub_, map_inf_pub_;
  ros::Timer occ_timer_, vis_timer_, fading_timer_;
  // ros::Subscriber lidar_odom_sub_, cloud_odom_sub_;

  //
  uniform_real_distribution<double> rand_noise_;//均匀分布随机数生成器，通常用于生成随机噪声
  normal_distribution<double> rand_noise2_;//高斯分布随机数生成器，比均匀分布更常用于模拟传感器噪声
  default_random_engine eng_;//默认随机数引擎，前面两个分布会基于这个引擎生成随机数
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)//把某个点的占据信息先记到缓存里，它不是立刻改地图，而是先把“这一轮 raycasting 的统计信息”记下来，后面再统一更新 occupancy
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id = pos2GlobalIdx(pos);
  int idx_ctns = globalIdx2BufIdx(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;//该体素的“命中或穿过总次数”加 1，无论是障碍点还是空闲射线经过，都会记一次。

  if (md_.count_hit_and_miss_[idx_ctns] == 1)//如果这个体素这轮是第一次被访问到
  {
    md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx)
{
  int inf_grid = mp_.inf_grid_;
  if (dir)//dir：方向标志，true 通常表示增加障碍影响，false 表示减少障碍影响
    md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  else
    md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
      {
        Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));//构造邻域中某个体素的全局索引 id_inf。等于中心体素 global_idx 加上当前偏移量。
#if GRID_MAP_NEW_PLATFORM_TEST
        if (isInInfBuf(id_inf))
        {
          int id_inf_buf = globalIdx2InfBufIdx(id_inf);
          if (dir)
            ++md_.occupancy_buffer_inflate_[id_inf_buf];
          else
          {
            --md_.occupancy_buffer_inflate_[id_inf_buf];
            if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case
            {
              ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");//附近障碍计数出现负值，说明逻辑出错了，重置地图
              fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_);
              fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0L);
            }
          }

          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 60000)
          {
            cout << "2 occ=" << md_.occupancy_buffer_inflate_[id_inf_buf] << " id_inf_buf=" << id_inf_buf << " id_inf=" << id_inf.transpose() << " pos=" << globalIdx2Pos(id_inf).transpose() << endl;
          }
        }
        else
        {
          cout << "id_inf=" << id_inf.transpose() << " md_.ringbuffer_inf_upbound3i_=" << md_.ringbuffer_inf_upbound3i_.transpose() << " md_.ringbuffer_upbound3i_=" << md_.ringbuffer_upbound3i_.transpose() << endl;
          ROS_ERROR("isInInfBuf return false 1");
        }

#else
        int id_inf_buf = globalIdx2InfBufIdx(id_inf);//直接把邻居体素的全局索引转成膨胀缓冲区索引
        if (dir)
          ++md_.occupancy_buffer_inflate_[id_inf_buf];
        else
        {
          --md_.occupancy_buffer_inflate_[id_inf_buf];
          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case检查是否发生了无符号下溢
          {
            ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
            fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_);
            fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0L);
          }
        }
#endif
      }
}

inline int GridMap::globalIdx2BufIdx(const Eigen::Vector3i &id)
{
  int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);//计算 x 方向在 ring buffer 中的位置
  int y_buffer = (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_origin3i_(2)) % md_.ringbuffer_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_size3i_(0);//如果 x_buffer 是负数，就加上 buffer 大小，把它变回合法范围。
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_size3i_(2);
//把三维 buffer 坐标 (x_buffer, y_buffer, z_buffer) 展平成一维数组下标
  return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer + md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i &id)//把全局三维索引 id 转换成“膨胀地图缓冲区”的一维下标
{
  int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
  int z_buffer = (id(2) - md_.ringbuffer_inf_origin3i_(2)) % md_.ringbuffer_inf_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_inf_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_inf_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_inf_size3i_(2);

  return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) * z_buffer + md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address)//把普通地图缓冲区中的一维下标 address 转换回三维全局索引
{

  const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);//计算 ring buffer 一个 z 层中包含多少个元素
  int zid_in_buffer = address / ringbuffer_xysize;//根据一维地址算出它位于第几个 z 层
  address %= ringbuffer_xysize;//把 address 更新成该 z 层内的偏移
  int yid_in_buffer = address / md_.ringbuffer_size3i_(0);//在当前 z 层中，根据剩余地址算出 y 坐标
  int xid_in_buffer = address % md_.ringbuffer_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
  if (xid_global > md_.ringbuffer_upbound3i_(0))
    xid_global -= md_.ringbuffer_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
  if (yid_global > md_.ringbuffer_upbound3i_(1))
    yid_global -= md_.ringbuffer_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_origin3i_(2);
  if (zid_global > md_.ringbuffer_upbound3i_(2))
    zid_global -= md_.ringbuffer_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address)
{

  const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_inf_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
  if (xid_global > md_.ringbuffer_inf_upbound3i_(0))
    xid_global -= md_.ringbuffer_inf_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
  if (yid_global > md_.ringbuffer_inf_upbound3i_(1))
    yid_global -= md_.ringbuffer_inf_size3i_(1);
  int zid_global = zid_in_buffer + md_.ringbuffer_inf_origin3i_(2);
  if (zid_global > md_.ringbuffer_inf_upbound3i_(2))
    zid_global -= md_.ringbuffer_inf_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInBuf(pos))//该处处理有些粗糙，后期可能需改动？
    return 0;//如果点不在地图范围内，直接返回 0

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;//只要启用了 enable_virtual_walll_,并且某个点的 z 高于 virtual_ceil_ 或低于 virtual_ground_,就直接返回 -1,也就是把这些位置当成“不可用/障碍”

  return md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(pos))] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInInfBuf(pos))
    return 0;

  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;
//这里不是返回简单的 0/1 判断，而是直接返回膨胀缓冲区中的计数值，返回 0：该位置当前不在任何障碍物的膨胀影响范围内，返回正数：该位置处于一个或多个障碍物的膨胀范围内
  return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline bool GridMap::isInBuf(const Eigen::Vector3d &pos)
{
  if (pos(0) < md_.ringbuffer_lowbound3d_(0) || pos(1) < md_.ringbuffer_lowbound3d_(1) || pos(2) < md_.ringbuffer_lowbound3d_(2))
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_upbound3d_(0) || pos(1) > md_.ringbuffer_upbound3d_(1) || pos(2) > md_.ringbuffer_upbound3d_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInBuf(const Eigen::Vector3i &idx)
{
  if (idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) || idx(2) < md_.ringbuffer_lowbound3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) || idx(2) > md_.ringbuffer_upbound3i_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3d &pos)
{
  if (pos(0) < md_.ringbuffer_inf_lowbound3d_(0) || pos(1) < md_.ringbuffer_inf_lowbound3d_(1) || pos(2) < md_.ringbuffer_inf_lowbound3d_(2))
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_inf_upbound3d_(0) || pos(1) > md_.ringbuffer_inf_upbound3d_(1) || pos(2) > md_.ringbuffer_inf_upbound3d_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3i &idx)
{
  if (idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) || idx(2) < md_.ringbuffer_inf_lowbound3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1) || idx(2) > md_.ringbuffer_inf_upbound3i_(2))
  {
    return false;
  }
  return true;
}

inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i &id) // t ~ 0us，把离散栅格编号映射回该栅格中心坐标
{
  return Eigen::Vector3d((id(0) + 0.5) * mp_.resolution_, (id(1) + 0.5) * mp_.resolution_, (id(2) + 0.5) * mp_.resolution_);//返回voxel 中心点坐标
}

inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d &pos)//把连续空间坐标映射到离散栅格编号
{//.array().floor()对每一维分别向下取整，.cast<int>()把结果转换成整数类型 
  return (pos * mp_.resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
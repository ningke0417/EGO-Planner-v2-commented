#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);//与cpp文件里的类型定义不一致？？？

double mod(double value, double modulus);

double intbound(double s, double ds);

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;//射线方向向量
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;//终点所在 voxel 的整数坐标，用于判断射线什么时候走到终点
  int endY_;
  int endZ_;
  double maxDist_;//最大允许行进距离
  double dx_;//射线方向在 x/y/z 三个方向上的分量。
  double dy_;
  double dz_;
  int stepX_;//射线在每个轴上的前进方向，+1：往正方向走，-1：往负方向走，0：该方向不动
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;//每跨过一个 voxel，在对应轴上 t 应增加多少，每走过一个 x 方向网格边界，tMaxX_ += tDeltaX_
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;//当前已经走过的距离，或者当前步对应的累计参数距离

  int step_num_;//当前已经执行的 step 数量

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_
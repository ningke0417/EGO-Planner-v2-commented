#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>//在 RViz 中显示点、球、线、箭头等
#include <visualization_msgs/MarkerArray.h>//一次发送多个 Marker
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;//发布目标点
    ros::Publisher global_list_pub;//发布全局路径点列表
    ros::Publisher init_list_pub;//发布初始路径点列表
    ros::Publisher optimal_list_pub;//发布优化后的路径点列表
    ros::Publisher failed_list_pub;//发布失败路径或失败点列表
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;//发布引导向量，通常是箭头之类的方向可视化

    ros::Publisher intermediate_pt0_pub;
    ros::Publisher intermediate_pt1_pub;
    ros::Publisher intermediate_grad0_pub;
    ros::Publisher intermediate_grad1_pub;
    ros::Publisher intermediate_grad_smoo_pub;//发布平滑项梯度
    ros::Publisher intermediate_grad_dist_pub;//发布距离项梯度
    ros::Publisher intermediate_grad_dist_pub;//发布距离项梯度
    ros::Publisher intermediate_grad_feas_pub;//发布可行性项梯度
    ros::Publisher intermediate_grad_swarm_pub;//发布群体/编队相关梯度

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;//定义一个类型别名 Ptr

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);//显示一个目标点
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);//显示全局路径点列表
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);//显示初始路径点列表
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);//显示多条初始轨迹
    void displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale);//显示多条优化后的轨迹
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);//显示优化后的点集合
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);//显示失败点或失败轨迹点集合
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);//显示 A* 生成的多条路径
    //通过给定发布器把一组箭头可视化出来
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    
    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);//显示中间点
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);//显示中间点对应的梯度
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::PolyTrajOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif
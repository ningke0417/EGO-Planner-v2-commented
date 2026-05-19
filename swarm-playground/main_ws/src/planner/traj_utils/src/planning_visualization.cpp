#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);//用于发布初始路径点列表
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);//用于发布失败的路径、无效点、碰撞点，或者某些优化失败的可视化结果
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);

    // intermediate_pt0_pub = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    // intermediate_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    // intermediate_pt1_pub = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    // intermediate_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    // intermediate_grad_smoo_pub = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    // intermediate_grad_dist_pub = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    // intermediate_grad_feas_pub = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    // intermediate_grad_swarm_pub = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad_dur_opt", 10);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;//sphere：用来显示一组球点，line_strip：用来显示折线
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;//设置 sphere 的显示类型为 SPHERE_LIST，points 数组中的每一个点，都会被画成一个小球
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;//设置 line_strip 的显示类型为 LINE_STRIP。把 points 数组中的点按顺序连接成一条连续折线
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;//把marker 添加到可视化场景中
    sphere.id = id;
    line_strip.id = id + 1000;//设置折线 marker 的编号为 id + 1000，避免互相覆盖或冲突

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;//设置两个 marker 的姿态四元数中的 w = 1.0
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;//设置球在 x 方向的尺度
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)//遍历 list 中所有三维点
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);//pt 就是由 Eigen 三维向量转换成了 ROS 的 geometry_msgs::Point
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}把一组 3D 路径点 list 转成 RViz 可显示的两种标记，并塞进 MarkerArray 里，一组球点 SPHERE_LIST，一条折线 LINE_STRIP
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);//把球 Marker 加入数组
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}把 list 里的点两两配对，生成多支箭头，并依次放进 MarkerArray 里给 RViz 显示
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;//设置箭杆直径
    arrow.scale.y = 2 * scale;//设置箭头头部直径
    arrow.scale.z = 2 * scale;//设置箭头头部长度

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);//箭头 1：p0 -> p1，箭头 2：p2 -> p3...
      arrow.points.clear();//每次都从空数组开始，保证当前 arrow 只包含本轮的起点和终点
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }
    sphere.pose.orientation.w = 1.0;
    sphere.pose.orientation.w = 1.0;
//把一个目标点显示成 RViz 里的球
  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);//设置球心位置的 x 坐标
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }
//把一组全局路径点按固定颜色发布出去显示
  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }
//显示多条初始轨迹
  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;//记录上一次显示了多少条初始轨迹

    for ( int id=0; id<last_nums; id++ )//清空上一次显示过的所有初始轨迹
    {
      Eigen::Vector4d color(0, 0, 0, 0);//完全透明
      vector<Eigen::Vector3d> blank;//创建一个空的点集 blank，这表示这次不传任何路径点
      displayMarkerList(init_list_pub, blank, scale, color, id, false);//把以前这个 ID 对应的轨迹显示内容清空或隐藏掉
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)init_trajs.size(); id++ )//重新显示这一次的所有初始轨迹
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);//半透明蓝色
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;//循环结束后，last_nums 就等于当前显示的轨迹总数
    }

  }
//显示一条初始轨迹
  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);//完全不透明的蓝色
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }
//显示多条最优轨迹
  void PlanningVisualization::displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale) // zxzxzx
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(optimal_list_pub, blank, scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)optimal_trajs.size(); id++ )
    {
      Eigen::Vector4d color(1, 0, 0, 0.7);//半透明红色
      displayMarkerList(optimal_list_pub, optimal_trajs[id], scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }
//显示最优路径点集
  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)//遍历矩阵的每一列。每次处理一个三维点
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();//把矩阵的第 i 列提取成一个三维点 pt
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);//完全不透明的红色
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }
//显示失败路径点集
  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);//较暗的深红色/暗红色
    displayMarkerList(failed_list_pub, list, 0.15, color, id);
  }
//显示多条 A* 路径
  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {//外层 vector 表示“多条路径”，内层vector<Eigen::Vector3d> 表示“一条路径上的多个点”

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)//a_star_paths 里装着很多条路径，每次循环拿出一条路径，这条路径变量名叫 block，auto 的意思是“让编译器自动推导变量类型
    {
      list.clear();
      for (auto pt : block)//遍历当前路径 block 中的每一个点
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }
//把一组点按“两两配对”的方式显示成箭头
  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);//先发空数组，再发新数组”的刷新动作。
    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }
//把一个矩阵里的中间点转成点列表，然后根据 type 发到不同的话题显示
  void PlanningVisualization::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());//预先给 pts_ 分配容量，大小等于 pts 的列数
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));//把矩阵的第 i 列直接构造成一个 Eigen::Vector3d，并放进 pts_
    }

    if ( !type.compare("0") )//如果 type == "0"
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )//数据一致性检查
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);//每个点最终要生成一支箭头，而每支箭头需要两个点：一个起点，一个终点
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));//grad 是另一个空间点
      }
    }//循环结束后，arrow_ 中就存好了[p0, q0, p1, q1, p2, q2, ...]，对应箭头：p0 -> q0，p1 -> q1，p2 -> q2
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));//grad 是方向向量，终点 = 起点 + 梯度，从 pts.col(i) 出发，沿 grad.col(i) 方向走一段，终点落在 pts.col(i) + grad.col(i)
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )//距离相关的梯度
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )//平滑项梯度
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )//可行性梯度
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("swarm") )
    {
      displayArrowList(intermediate_grad_swarm_pub, arrow_, 0.02, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace ego_planner
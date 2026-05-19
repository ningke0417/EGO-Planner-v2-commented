#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];//释放每个 GridNode 对象的堆内存
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;//保存池子尺寸
    CENTER_IDX_ = pool_size / 2;//计算池中心索引，用于坐标与索引转换时居中

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;//在每个体素上创建一个 GridNode 实例
            }
        }
    }

    grid_map_ = occ_map;//保存外部传入的地图智能指针，后续可查占据状态
}
//定义 A* 的“3D 对角距离启发函数”，输入两个节点，返回估计代价 h
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));//计算 x 方向索引差绝对值
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;//初始化启发代价
    int diag = min(min(dx, dy), dz);//先取三轴最小差值，表示可走“3D 对角步”（一次同时改变 x,y,z）的最大次数
    dx -= diag;//扣掉已用的 3D 对角步后，x 方向剩余差值
    dy -= diag;
    dz -= diag;

    if (dx == 0)//若剩余 x 为 0，说明剩余只在 y-z 平面内移动
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }//diag 次 3D 对角步（每步代价 sqrt(3)），在 y-z 平面尽量走 2D 对角步（每步 sqrt(2)），最后单轴直走（每步 1）
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)//计算两个栅格节点之间的“曼哈顿启发距离”
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)//定义欧氏距离启发函数
{
    return (node2->index - node1->index).norm();//.norm() 是 Eigen 的二范数，即 sqrt(dx^2 + dy^2 + dz^2)
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)//定义路径回溯函数
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)//当当前节点还有前驱节点时继续回溯
    {
        current = current->cameFrom;//指针沿父链向前走一步
        path.push_back(current);//把父节点加入路径
    }

    return path;
}
//输入：起点/终点世界坐标 start_pt、end_pt，输出：起点/终点栅格索引，返回：true 表示转换并调整成功，false 表示失败
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;//如果起点或终点无法转索引，直接结束

    int occ;
    if (checkOccupancy(Index2Coord(start_idx)))//进入“起点在障碍里”的处理分支
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;//把起点沿“远离终点”方向推一步。
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))//推动后的新起点再次转索引，检查是否还在地图池中
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));//查询新起点是否占据，并存到 occ
            if (occ == -1)//如果返回 -1，认为在地图区域外（或无效区域）
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);//只要 occ 非 0（仍被占据/异常），继续循环推起点。当 occ==0（空闲）退出
    }

    if (checkOccupancy(Index2Coord(end_idx)))////进入“终点在障碍里”的处理分支
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;//“终点沿远离起点方向”挪一步
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));//存疑？？？
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}
//定义 A* 主搜索函数。输入步长、起点、终点；返回搜索状态码
ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;//搜索轮次加一。这个类复用节点池，用 rounds_ 区分“本轮是否访问过该节点”

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);//用空队列交换，清空旧 openSet_（比逐个弹出更干净）

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;//标记起点“已在本轮被初始化”
    startPtr->gScore = 0;//起点到自身代价为 0
    startPtr->fScore = getHeu(startPtr, endPtr);//起点总代价估计 f=g+h，此时 g=0，所以等于启发值
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;//临时变量，存候选路径 g 值

    int num_iter = 0;//记录循环迭代次数
    while (!openSet_.empty())//A* 主循环：只要开放集非空就继续
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();//取出 fScore 最小节点作为当前扩展节点

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))//如果当前节点就是终点，搜索成功
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);//从终点沿 cameFrom 回溯路径保存到 gridPath_
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.当前节点移入关闭集，不再重复扩展

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)//枚举 26 邻域（3D 中心点周围所有方向）
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;//跳过自己本身

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;//计算邻居格子坐标
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;//跳过池边缘和越界区域，避免访问非法节点/边界不稳定区域
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;//判断该节点在本轮是否已经见过。

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);//从 current 到该邻居一步代价
                    tentative_gScore = current->gScore + static_cost;//通过当前节点到邻居的候选 g 值

                    if (!flag_explored)//若邻居是本轮首次发现
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()//返回类型是 vector<Vector3d>，即“由三维坐标点组成的路径
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());//将 path 原地反转，因为回溯路径通常是“终点->起点”，反转后变成“起点->终点”
    return path;
}

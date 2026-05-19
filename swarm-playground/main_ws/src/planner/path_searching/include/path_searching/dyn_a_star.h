#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;//试图定义“无穷大代价”。但这里有问题：1 >> 20 是位右移，结果是 0，所以 inf 实际是 0.0。通常应写成 1 << 20 或 1e20
struct GridNode;
typedef GridNode *GridNodePtr;//定义类型别名：GridNodePtr 是 GridNode*

enum ASTAR_RET//A* 返回状态：成功、初始化错误、搜索错误
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct GridNode
{
	enum enum_state//节点状态：在开放集、在关闭集、未定义
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call记录节点属于第几次搜索轮次（区分多次调用）
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;//节点的三维栅格索引

	double gScore{inf}, fScore{inf};//gScore：起点到当前点代价；fScore：g+h。默认用 inf 初始化
	GridNodePtr cameFrom{NULL};//回溯路径的前驱指针， A* 在搜索过程中记录的“父节点”
};

class NodeComparator//优先队列比较器类
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)//重载比较函数，给优先队列用
	{
		return node1->fScore > node2->fScore;//fScore 小的优先级高（构造成小顶堆行为）
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);//快速坐标转索引

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);//对角启发函数
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);//曼哈顿启发函数
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);//欧氏启发函数
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);//统一启发函数入口（本文件里定义为对角启发）
    //将起终点转索引并做边界/合法性调整
	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;//索引转世界坐标
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;//世界坐标转索引，并检查是否越界

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos) { return grid_map_->getInflateOccupancy(pos); }//内联占据检查

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);//从终点节点沿 cameFrom 回溯得到路径

	double step_size_, inv_step_size_;//栅格步长及其倒数
	Eigen::Vector3d center_;//地图中心点坐标
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;//中心索引、栅格池大小
	const double tie_breaker_ = 1.0 + 1.0 / 10000;//启发函数轻微放大，打破平局，减少搜索抖动

	std::vector<GridNodePtr> gridPath_;//存储最终节点路径

	GridNodePtr ***GridNodeMap_;//3D 节点指针数组（节点池）
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;//开放集优先队列

	int rounds_{0};//当前搜索轮次计数

public:
	typedef std::shared_ptr<AStar> Ptr;//AStar 智能指针别名

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);//初始化地图和池大小

	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);//进行一次 A* 搜索

	std::vector<Eigen::Vector3d> getPath();//获取最终坐标路径
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)//实际启发值 = 对角启发 × tie_breaker_
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const//把栅格索引还原为世界坐标。
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;//做四舍五入到最近栅格，再加中心索引。

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))//检查 3 个维度是否在 [0, POOL_SIZE_) 内
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif

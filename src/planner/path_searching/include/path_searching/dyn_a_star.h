/// @file
/// 局部 A* 节点池与搜索接口；方向相关占用通过 GridMap 直接查询。
/// 默认节点池为 100³，步长和池中心由每次搜索配置；当前状态不包含独立时间或 yaw 维度。
#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

// 保留原始初值表达式；当前 1 >> 20 实际为 0，不能把名为 inf 的值理解为无穷大。搜索代码另行设置有效评分。
constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

/// @brief 返回码区分成功、起终点初始化失败和搜索失败；日志需结合该返回值分析。
enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

/// @brief 每轮复用的搜索节点：rounds 防止混入上一轮状态，cameFrom 用于成功后的路径回溯。
struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	// 初值 0；与搜索器 rounds_ 比较以区分该池节点是否已在本轮访问，避免每轮清空整个节点池。
	int rounds{0}; // Distinguish every call
	// 节点所属集合，初值 UNDEFINED=3；本轮发现/扩展时更新为 OPENSET=1 或 CLOSEDSET=2。
	enum enum_state state
	{
		UNDEFINED
	};
	// 本搜索节点在固定池内的整数位置，由当前搜索设置；转换成世界坐标需结合中心与步长。
	Eigen::Vector3i index;

	// gScore：从本轮起点累计的索引步长代价；fScore=g+启发项。只对本轮有效节点读取，入队时更新。
	double gScore{inf}, fScore{inf};
	// 本轮最优前驱，起点为空；成功后沿它回溯碰撞段绕行路径。
	GridNodePtr cameFrom{NULL};
};

/// @brief priority_queue 以较小 fScore 优先出队；同值处理还受启发项 tie-breaker 影响。
class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

/// @brief 管理节点池、开放集与本次搜索结果；仅在 SUCCESS 后使用 getPath()。
class AStar
{
private:
	// 管理器注入的共享局部地图，按每次扩展方向估计 yaw 并查询膨胀占用。
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	// 本轮搜索网格步长（m）及倒数，调用方通常传地图分辨率 0.05 m；不由节点池维数自动决定。
	double step_size_, inv_step_size_;
	// 本轮起终点中点（世界系 m），用于将固定节点池平移到当前碰撞段附近。
	Eigen::Vector3d center_;
	// POOL_SIZE_ 来自 initGridMap()，当前管理器设置 100³；CENTER_IDX_ 为池的中心索引。
	// 每轴索引 0..99、中心索引 50；世界坐标还需乘本轮 step_size_，不是用起终点当两个相对角。
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	// 启发项倍率 1.0001，轻微打破相同评分；该启发设置不应直接描述为严格最优保证。
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	// 成功回溯保存的节点路径；getPath() 转成世界点并整理顺序，失败后不应直接消费为新路径。
	std::vector<GridNodePtr> gridPath_;

	// 固定容量的搜索节点池，在 initGridMap() 分配并复用，与占用地图的体素缓冲区分离。
	GridNodePtr ***GridNodeMap_;
	// 本轮待扩展开放集，按较小 fScore 优先弹出；每轮搜索重新组织。
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	// 搜索轮次，初值 0，每次 AstarSearch() 递增；与各 GridNode::rounds 配套区分旧数据。
	int rounds_{0};

public:
	// 当前类型别名允许共同持有 AStar；最后一个强所有者释放时调用其析构函数。
	// 当前主链仅在 BsplineOptimizer::a_star_ 保存它，不能因为用了 shared_ptr 就推断已有多个所有者。
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

/// @brief 根据当前搜索中心和步长将池索引映射到世界坐标，而非直接使用 GridMap 缓冲地址。
inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

/// @brief 世界位置转换为池索引并检查容量边界，越界报告搜索池不足。
inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif

#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <limits>

// constexpr double inf = 1 >> 20;
constexpr double inf = std::numeric_limits<double>::infinity();// inf定义为无穷大
struct GridNode;
typedef GridNode *GridNodePtr;// GridNodePtr是指向该结构体的指针类型的别名，更推荐 using GridNodePtr = GridNode*;
/*结构体：GridNode，代表网格地图的第一个节点
*
*
*/
struct GridNode
{
	/*定义了一个无作用域的枚举类型enum_state，
	包含三个枚举值：OPENSET、CLOSEDSET和UNDEFINED
	分别表示网格节点的状态：1开放集、2关闭集和3未定义
	 * 网格节点的状态枚举
	 */
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3//同时作为枚举常量和后续成员变量的默认初始值
	};

	//使用{}表示初始化，默认初始值
	int rounds{0}; // Distinguish every call 当前节点的轮次，用于区分每次调用
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector2i index;//改为二维向量，表示网格地图中节点的索引

	//这里inf是一个常量，表示无穷大，用于初始化gScore和fScore
	double gScore{inf}, fScore{inf};//实际代价 估计总代价
	GridNodePtr cameFrom{NULL}; //指向当前节点的父节点，用于回溯路径，形成搜索树中的父子关系
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;
	//二维中不需要了
	// inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector2d start_pt, const Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx);

	inline Eigen::Vector2d Index2Coord(const Eigen::Vector2i &index) const;
	inline bool Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector2d &pos) const
	{
		return grid_map_->getInflateOccupancy(
				Eigen::Vector3d(pos.x(), pos.y(), 0.0)) != 0;
	}

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector2d center_;
	Eigen::Vector2i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr **GridNodeMap_;//表示2级指针变量
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector2d AStar::Index2Coord(const Eigen::Vector2i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) )
	{
		RCLCPP_ERROR(rclcpp::get_logger("Coord2Index"), "Ran out of pool, index=%d %d, POOL_SIZE=%d %d", idx(0), idx(1),POOL_SIZE_(0), POOL_SIZE_(1));
		return false;
	}

	return true;
};

#endif

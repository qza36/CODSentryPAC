#ifndef _GRIDNODE_
#define _GRIDNODE_

#include <boost/type.hpp>
#include "Eigen/Eigen"

#define inf (1 << 20)  // A*算法中的无穷大值

struct GridNode;
typedef GridNode* GridNodePtr;
struct OccupancyNode;
typedef OccupancyNode* OccupancyNodePtr;

/**
 * @brief 栅格节点，用于A*搜索和地图表示
 */
struct GridNode
{
    int id;                 // 节点状态: 0-未访问, 1-open set, -1-closed set
    double height;          // 该栅格的地面高度
    Eigen::Vector3d coord;  // 世界坐标 (x, y, z)
    Eigen::Vector3i dir;    // 扩展方向
    Eigen::Vector3i index;  // 栅格索引 (ix, iy, iz)

    double gScore, fScore;  // A*代价: g(实际代价), f(总代价=g+h)
    GridNodePtr cameFrom;   // 父节点指针，用于回溯路径
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;  // 在open list中的迭代器

    // 桥洞相关属性
    bool exist_second_height = false;   // 是否存在第二层高度(桥洞区域)
    double second_height = 0.08;        // 第二层高度值
    bool second_local_occupancy = false;// 第二层局部占用状态
    bool second_local_swell = false;    // 第二层膨胀状态

    double visibility = 0.0;  // 可见性值，用于拓扑采样

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord) {
        id = 0;
        height = 0.0;  // 默认高度为0（平面）
        index = _index;
        coord = _coord;
        dir = Eigen::Vector3i::Zero();
        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
        exist_second_height = false;
    }

    GridNode(int _X, int _Y, int _Z) {
        id = 1;
        height = -1.0;
        index(0) = _X;
        index(1) = _Y;
        index(2) = _Z;
        dir = Eigen::Vector3i::Zero();
        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
        exist_second_height = false;
    }

    bool operator<(const GridNode& node) const {
        return (node.index(0) < index(0)) ||
               ((node.index(0) == index(0)) && (node.index(1) < index(1)));
    }

    GridNode() {
        id = 0;
        height = 0.0;
        dir = Eigen::Vector3i::Zero();
        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
        exist_second_height = false;
    };
    ~GridNode() {};
};

/**
 * @brief 占用节点，用于动态障碍物检测
 */
struct OccupancyNode
{
    Eigen::Vector3i index;  // 栅格索引
    bool unknow;            // 是否为未知区域
    bool occ;               // 是否被占用
    int count;              // 观测计数
    int m_raycast_num;      // 射线投射次数

    OccupancyNode(Eigen::Vector3i _index, bool _unknow) {
        index = _index;
        unknow = _unknow;
        occ = false;
        count = 0;
        m_raycast_num = 0;
    }

    OccupancyNode() {};
    ~OccupancyNode() {};
};

#endif

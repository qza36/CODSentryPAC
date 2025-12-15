#ifndef _GRIDNODE_
#define _GRIDNODE_
#include <boost/type.hpp>

#include "Eigen/Eigen"
#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;
struct OccupancyNode;
typedef OccupancyNode* OccupancyNodePtr;
struct GridNode
{
    int id;        // 1--> open set, -1 --> closed set
    double height;
    Eigen::Vector3d coord; //坐标x,y
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index; //索引

    double gScore, fScore;
    GridNodePtr cameFrom; //父节点
    std::multimap<double, GridNodePtr>::iterator nodeMapIt; //用来记录自己在OPENLIST中的位置
    bool exist_second_height = false;
    double second_height = 0.08;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){
        id = 0;
        index = _index;
        coord = _coord;
        dir   = Eigen::Vector3i::Zero();

        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};
struct OccupancyNode //障碍物节点
{
   Eigen::Vector3i index;
    bool unknow;
    bool occ;
    int count;
    int m_raycast_num;

    OccupancyNode(Eigen::Vector3i _index,bool _unknow)
    {
        index = _index;
        unknow = _unknow;
        occ = false;
        count = 0;
        m_raycast_num = 0;
    }
    OccupancyNode(){};
    ~OccupancyNode(){};
};
#endif

#ifndef _GRIDNODE_
#define _GRIDNODE_
#include "Eigen/Eigen"
#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;
struct GridNode
{
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; //坐标x,y
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index; //索引

    double gScore, fScore;
    GridNodePtr cameFrom; //父节点
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

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
#endif

#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H
#include <iostream>
#include "trajectory_generation/GridNode.hpp"
class GlobalMap
{
public:
    uint8_t *data;
    uint8_t *l_data;

    int GLX_SIZE,GLY_SIZE,GLZ_SIZE;
    int GLXYZ_SIZE,GLYZ_SIZE,GLXY_SIZE;
    double m_resolution,m_inv_resolution;

    GridNodePtr **GridNodeMap;
    GridNodePtr **GridNodeLocalMap;  //local



};
class AstarPathFinder
{

};
#endif

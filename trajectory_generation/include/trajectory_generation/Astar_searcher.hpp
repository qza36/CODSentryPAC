#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H
#include <iostream>

#include "planner_config.hpp"
#include "trajectory_generation/GridNode.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "trajectory_generation/GridMap.hpp"
class AstarPathFinder
{
AstarPathFinder()
{
    GlobalMap gm;
}
};
#endif
